"""PC-only YOLO preview. Reads camera HTTP frames; has no robot command route."""
from collections import OrderedDict
from datetime import datetime,timezone,timedelta
import argparse
import hashlib
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import re
import secrets
import signal
import threading
import time
import urllib.request
import urllib.error
from urllib.parse import urlparse

ROOT=Path(__file__).resolve().parents[2]
ASSETS=Path(__file__).with_name('yolo_assets')


def options(conf,masks=True):
    if isinstance(conf,bool) or not isinstance(conf,(int,float)) or not math.isfinite(conf) or not .05<=conf<=.95:
        raise ValueError('置信度应在 0.05–0.95 之间')
    if not isinstance(masks,bool):raise ValueError('分割显示开关无效')
    return float(conf),masks


class YoloPreview:
    def __init__(self,model,camera_url,output,*,device='auto',imgsz=640,max_fps=5.):
        self.model_path=Path(model).resolve()
        if not self.model_path.is_file():raise ValueError('模型文件不存在')
        parsed=urlparse(camera_url)
        if parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost') or parsed.path not in ('','/') or parsed.query or parsed.fragment:
            raise ValueError('相机地址应为电脑上的 localhost SSH 转发地址')
        if parsed.username or parsed.password:raise ValueError('相机地址不得包含凭据')
        if imgsz not in (320,640,960) or not 0<max_fps<=15:raise ValueError('推理尺寸或频率无效')
        self.camera_url=camera_url.rstrip('/');self.output=Path(output);self.device=device;self.imgsz=imgsz
        self.max_fps=max_fps;self.lock=threading.Lock();self.stop=threading.Event()
        self.thread=None;self.enabled=False;self.conf=.25;self.masks=True;self.revision=0
        self.model_info=dict(file=self.model_path.name,run=self.model_path.parent.parent.name,
                             sha256=hashlib.sha256(self.model_path.read_bytes()).hexdigest())
        self.phase='正在加载模型';self.error=None;self.cache=OrderedDict();self.latest=None;self.counter=0
        self.token=secrets.token_urlsafe(32);self.completed=[]

    def configure(self,enabled,conf,masks):
        if not isinstance(enabled,bool):raise ValueError('检测开关无效')
        conf,masks=options(conf,masks)
        with self.lock:
            self.enabled,self.conf,self.masks=enabled,conf,masks;self.revision+=1

    def state(self):
        with self.lock:
            latest=None if self.latest is None else dict(self.latest['meta'])
            if latest is not None:
                latest['age_ms']=(time.monotonic_ns()-self.latest['finished_ns'])*1e-6
                latest['fresh']=self.enabled and latest['age_ms']<2000
            return dict(enabled=self.enabled,phase=self.phase,error=self.error,model=dict(self.model_info),
                        conf=self.conf,masks=self.masks,imgsz=self.imgsz,max_fps=self.max_fps,
                        latest=latest,robot_commands_sent=0)

    def frame(self,identity,kind):
        if kind not in ('raw','annotated'):raise ValueError('图像类型无效')
        with self.lock:
            result=self.cache.get(identity)
            if result is None:raise ValueError('结果已更新，请等待当前帧')
            return result[kind],result.get('raw_mime','image/jpeg') if kind=='raw' else 'image/jpeg'

    def save(self,identity):
        with self.lock:
            result=self.cache.get(identity)
            if result is None:raise ValueError('结果已离开缓存，请重新保存当前结果')
        stamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
        folder=self.output/('capture_'+stamp+'_'+identity)
        folder.mkdir(parents=True,exist_ok=False)
        (folder/('rgb.png' if result.get('raw_mime')=='image/png' else 'rgb.jpg')).write_bytes(result['raw'])
        (folder/'prediction.jpg').write_bytes(result['annotated'])
        (folder/'prediction.json').write_text(json.dumps(dict(result['meta'],model=self.model_info),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        return dict(folder=str(folder),frame_id=result['meta']['source_frame_id'])

    def camera(self):
        with urllib.request.urlopen(self.camera_url+'/api/camera/state',timeout=2.) as response:state=json.load(response)
        rgb=state.get('rgb')
        if not state.get('running'):raise ValueError('请先在机器人控制网页开启 RGB 相机预览')
        if not rgb or not rgb.get('fresh'):raise ValueError(state.get('error') or '等待新鲜 RGB 图像')
        identity=rgb['id']
        if not re.fullmatch(r'[a-f0-9]{12}_[0-9]{1,12}',identity):raise ValueError('上游帧编号无效')
        return rgb

    def work(self):
        try:
            os.environ['YOLO_AUTOINSTALL']='False';os.environ['WANDB_DISABLED']='true'
            settings_dir=self.output/'runtime_settings';settings_dir.mkdir(parents=True,exist_ok=True)
            os.environ['YOLO_CONFIG_DIR']=str(settings_dir)
            import cv2
            import numpy as np
            import torch
            import ultralytics
            from ultralytics import YOLO
            torch.set_num_threads(min(4,os.cpu_count() or 1))
            selected=(0 if torch.cuda.is_available() else 'cpu') if self.device=='auto' else self.device
            model=YOLO(str(self.model_path))
            model.predict(np.zeros((640,640,3),dtype=np.uint8),device=selected,imgsz=self.imgsz,
                          conf=.25,verbose=False,save=False,retina_masks=True)
            with self.lock:
                self.model_info.update(task=model.task,names=model.names,ultralytics=ultralytics.__version__,
                    torch=torch.__version__,device=torch.cuda.get_device_name(0) if selected==0 else str(selected))
                self.phase='模型就绪，点击开始检测'
            last_key=None
            while not self.stop.is_set():
                with self.lock:enabled,conf,masks,revision=self.enabled,self.conf,self.masks,self.revision
                if not enabled:self.stop.wait(.1);continue
                began=time.perf_counter()
                try:
                    frame=self.camera();key=(frame['id'],revision)
                    if key==last_key:self.stop.wait(.04);continue
                    with urllib.request.urlopen(self.camera_url+'/api/camera/frame/'+frame['id'],timeout=2.) as response:raw=response.read()
                    if len(raw)>24*1024*1024:raise ValueError('RGB 图像过大')
                    image=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
                    if image is None:raise ValueError('RGB 解码失败')
                    result=model.predict(image,imgsz=self.imgsz,device=selected,conf=conf,verbose=False,
                                         save=False,retina_masks=True,max_det=30)[0]
                    detections=[]
                    polygons=None if result.masks is None else result.masks.xy
                    for i,box in enumerate(result.boxes):
                        cls=int(box.cls.item())
                        detections.append(dict(class_id=cls,label=result.names[cls],confidence=float(box.conf.item()),
                            xyxy=box.xyxy[0].cpu().tolist(),
                            polygon_xy=None if polygons is None or i>=len(polygons) else polygons[i].tolist()))
                    annotated=result.plot(masks=masks,boxes=True,conf=True,labels=True,line_width=2)
                    ok,encoded=cv2.imencode('.jpg',annotated,[cv2.IMWRITE_JPEG_QUALITY,88])
                    if not ok:raise ValueError('检测图编码失败')
                    elapsed=(time.perf_counter()-began)*1000
                    with self.lock:
                        # A paused or reconfigured detector must not publish an old result.
                        if not self.enabled or revision!=self.revision:continue
                        self.counter+=1;identity=str(self.counter)
                        meta=dict(id=identity,source_frame_id=frame['id'],source_ns=frame['source_ns'],
                            width=int(image.shape[1]),height=int(image.shape[0]),detections=detections,
                            speed_ms=result.speed,fetch_to_result_ms=elapsed,
                            estimated_source_age_ms=float(frame.get('age_ms',0))+elapsed,
                            conf=conf,masks=masks,completed_ns=time.time_ns())
                        packet=dict(raw=raw,raw_mime=frame.get('mime','image/jpeg'),annotated=encoded.tobytes(),meta=meta,finished_ns=time.monotonic_ns())
                        self.cache[identity]=packet;self.latest=packet
                        while len(self.cache)>6:self.cache.popitem(last=False)
                        self.phase='实时检测中';self.error=None
                        self.completed.append(elapsed);self.completed=self.completed[-50:]
                    last_key=key
                except (OSError,ValueError,RuntimeError) as error:
                    with self.lock:self.error=str(error);self.phase='等待图像 / 接入异常'
                self.stop.wait(max(.02,1/self.max_fps-(time.perf_counter()-began)))
        except Exception as error:
            with self.lock:self.phase='模型加载失败';self.error=str(error)


def make_server(service,port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*unused):pass
        def reply(self,status,value,mime='application/json; charset=utf-8'):
            if not isinstance(value,bytes):value=json.dumps(value,ensure_ascii=False).encode()
            self.send_response(status);self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(value)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Content-Security-Policy',"default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' blob:; frame-ancestors 'none'; base-uri 'none'")
            self.end_headers()
            try:self.wfile.write(value)
            except (BrokenPipeError,ConnectionResetError):pass
        def allowed(self):
            return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}')
        def do_GET(self):
            if not self.allowed():return self.reply(403,dict(error='Use localhost'))
            if self.path=='/api/session':return self.reply(200,dict(token=service.token))
            if self.path=='/api/state':return self.reply(200,service.state())
            if self.path.startswith('/api/image/'):
                match=re.fullmatch(r'/api/image/([0-9]+)/(raw|annotated)',self.path)
                if match:
                    try:
                        data,mime=service.frame(*match.groups());return self.reply(200,data,mime)
                    except ValueError as error:return self.reply(409,dict(error=str(error)))
            assets={'/':('index.html','text/html; charset=utf-8'),'/app.js':('app.js','text/javascript; charset=utf-8'),'/styles.css':('styles.css','text/css; charset=utf-8')}
            if self.path in assets:
                name,mime=assets[self.path];return self.reply(200,(ASSETS/name).read_bytes(),mime)
            self.reply(404,dict(error='Not found'))
        def do_POST(self):
            if (not self.allowed() or self.headers.get('Origin')!='http://'+self.headers.get('Host','') or
                not secrets.compare_digest(self.headers.get('X-Preview-Token',''),service.token)):
                return self.reply(403,dict(error='Invalid local session'))
            try:
                n=int(self.headers.get('Content-Length',0))
                if not 0<n<=2048 or self.headers.get('Content-Type')!='application/json':raise ValueError('Invalid request')
                body=json.loads(self.rfile.read(n))
                if self.path=='/api/config':service.configure(body.get('enabled'),body.get('conf'),body.get('masks'));return self.reply(200,dict(accepted=True))
                if self.path=='/api/save':return self.reply(200,service.save(str(body.get('id'))))
                self.reply(404,dict(error='Not found'))
            except (ValueError,TypeError,AttributeError) as error:self.reply(409,dict(error=str(error)))
            except OSError:self.reply(507,dict(error='检测结果保存失败'))
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler);server.daemon_threads=True;return server


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,required=True);parser.add_argument('--camera-url',default='http://127.0.0.1:8765')
    parser.add_argument('--port',type=int,default=8774);parser.add_argument('--device',default='auto')
    parser.add_argument('--output',type=Path,default=ROOT/'reports/yolo_preview')
    args=parser.parse_args();service=YoloPreview(args.model,args.camera_url,args.output,device=args.device)
    server=make_server(service,args.port);server.timeout=.25
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *unused:service.stop.set())
    service.thread=threading.Thread(target=service.work,daemon=True);service.thread.start()
    print(json.dumps(dict(url=f'http://127.0.0.1:{server.server_port}/',model=args.model.name,robot_commands_sent=0)),flush=True)
    try:
        while not service.stop.is_set():server.handle_request()
    finally:service.stop.set();server.server_close();service.thread.join(timeout=3.)


if __name__=='__main__':main()
