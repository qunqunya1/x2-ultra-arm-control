"""Read-only RGB-D process, bounded preview cache and exact displayed-RGB capture.

No arm SDK, dynamics model or command publisher is used in this process.
"""
from collections import OrderedDict
from datetime import datetime, timezone, timedelta
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
import json
import multiprocessing as mp
import os
from pathlib import Path
from queue import Empty
import re
import secrets
import threading
import time
import urllib.request
import urllib.error

import numpy as np

PREFIX='/aima/hal/sensor/rgbd_head_front/'
FRAME_ID=re.compile(r'[a-f0-9]{12}_[0-9]{1,12}')
CAPTURE_NAME=re.compile(r'rgb_[0-9]{8}_[0-9]{6}_[0-9]{6}_[a-f0-9]{12}_[0-9]{1,12}\.(jpg|png)')


def image_array(message, channels, dtype):
    height,width,step=int(message.height),int(message.width),int(message.step)
    item=np.dtype(dtype).itemsize
    if not (0<height<=4320 and 0<width<=7680 and height*width<=16_777_216):
        raise ValueError('相机图像尺寸无效')
    if step<width*channels*item or len(message.data)!=height*step:
        raise ValueError('相机图像步长或数据长度无效')
    dt=np.dtype(dtype).newbyteorder('>' if message.is_bigendian else '<')
    return np.ndarray((height,width,channels),dtype=dt,buffer=bytes(message.data),
                      strides=(step,channels*item,item)).copy()


def depth_preview(message, near=.15, far=2.):
    from PIL import Image
    if message.encoding=='16UC1':
        depth=image_array(message,1,'u2')[:,:,0].astype(float)*.001
    elif message.encoding=='32FC1':
        depth=image_array(message,1,'f4')[:,:,0].astype(float)
    else:raise ValueError('深度图仅支持 16UC1 或 32FC1')
    valid=np.isfinite(depth)&(depth>0)
    normalized=np.clip((np.nan_to_num(depth,nan=far,posinf=far)-near)/(far-near),0,1)
    # Fixed near/far scale; invalid samples are black, rather than false depth.
    rgb=np.stack([255*(1-normalized),220*(1-np.abs(2*normalized-1)),255*normalized],axis=-1).astype('uint8')
    rgb[~valid]=0
    image=Image.fromarray(rgb);image.thumbnail((640,480))
    output=BytesIO();image.save(output,format='JPEG',quality=78)
    return output.getvalue(),dict(width=int(message.width),height=int(message.height),
        encoding=message.encoding,valid_fraction=float(valid.mean()),near_m=near,far_m=far,
        unit_assumption='16UC1: millimetres; 32FC1: metres',preview_only=True)


class CameraFrames:
    def __init__(self, output, *, backend='robot', capacity=48, timeout_s=2.):
        self.output=Path(output);self.backend=backend;self.capacity=capacity;self.timeout=timeout_s
        self.lock=threading.Lock();self.frames=OrderedDict();self.latest={};self.sequence=0
        self.prefix=secrets.token_hex(6);self.error=None;self.errors={};self.captures=0;self.cached_bytes=0

    def add(self, kind, data, mime, source_ns, **metadata):
        with self.lock:
            self.sequence+=1;identity=f'{self.prefix}_{self.sequence}'
            frame=dict(id=identity,kind=kind,data=data,mime=mime,source_ns=int(source_ns),
                       received_ns=time.monotonic_ns(),metadata=dict(metadata))
            self.frames[identity]=frame;self.latest[kind]=identity;self.errors.pop(kind,None)
            self.cached_bytes+=len(data)
            while len(self.frames)>1 and (len(self.frames)>self.capacity or self.cached_bytes>48*1024*1024):
                _,old=self.frames.popitem(last=False);self.cached_bytes-=len(old['data'])
            return identity

    def state(self):
        with self.lock:
            result=dict(running=True,backend=self.backend,error=self.error or '; '.join(self.errors.values()) or None,captures=self.captures,
                        rgb=None,depth=None,capture_contents='RGB only; depth is never saved')
            for kind,identity in self.latest.items():
                frame=self.frames.get(identity)
                if frame is None:continue
                age=(time.monotonic_ns()-frame['received_ns'])*1e-9
                result[kind]=dict(id=identity,mime=frame['mime'],source_ns=frame['source_ns'],
                                 age_ms=age*1000,fresh=age<=self.timeout,**frame['metadata'])
            return result

    def get(self, identity):
        if not isinstance(identity,str) or FRAME_ID.fullmatch(identity) is None:
            raise ValueError('图像帧编号无效')
        with self.lock:
            frame=self.frames.get(identity)
            if frame is None:raise ValueError('显示帧已离开缓存，请更新画面后采集')
            return frame

    def capture(self, identity):
        frame=self.get(identity)
        if frame['kind']!='rgb':raise ValueError('采集只允许保存 RGB 彩色图')
        # Save the immutable requested frame, never substitute the latest one.
        suffix='jpg' if frame['mime']=='image/jpeg' else 'png'
        timestamp=datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_%f')
        name=f'rgb_{timestamp}_{identity}.{suffix}'
        self.output.mkdir(parents=True,exist_ok=True)
        path=self.output/name;temporary=path.with_name(name+'.partial')
        created=False
        try:
            with temporary.open('xb') as stream:
                created=True;stream.write(frame['data']);stream.flush();os.fsync(stream.fileno())
            os.link(temporary,path)  # Atomic publication without replacing a prior capture.
        finally:
            if created and temporary.is_file():temporary.unlink()
        with self.lock:self.captures+=1
        return dict(name=name,frame_id=identity,source_ns=frame['source_ns'],mime=frame['mime'],
                    bytes=len(frame['data']),saved_under='captures/rgb/',depth_saved=False,
                    download_url='/api/camera/files/'+name,**frame['metadata'])

    def compressed_rgb(self,message):
        from PIL import Image
        data=bytes(message.data)
        if not data or len(data)>24*1024*1024:raise ValueError('压缩图像大小无效')
        with Image.open(BytesIO(data)) as image:
            width,height=image.size;kind=image.format
            if width*height>16_777_216 or kind not in ('JPEG','PNG'):raise ValueError('RGB 压缩格式或尺寸不支持')
            image.verify()
        return self.add('rgb',data,'image/jpeg' if kind=='JPEG' else 'image/png',source_stamp(message),
                        width=width,height=height,encoding=kind,frame=message.header.frame_id)


def source_stamp(message):
    return int(message.header.stamp.sec)*1_000_000_000+int(message.header.stamp.nanosec)


def _demo(frames,stop,hz):
    from PIL import Image,ImageDraw
    from types import SimpleNamespace as Obj
    while not stop.is_set():
        stamp=time.time_ns();phase=(time.monotonic()%6)/6
        image=Image.new('RGB',(640,360),'#dbe6e0');draw=ImageDraw.Draw(image)
        draw.rectangle((0,160,640,360),fill='#bdb8a8')
        x=int(120+phase*150)
        draw.rounded_rectangle((x,190,x+95,255),radius=10,fill='#33596b',outline='#182e39',width=3)
        draw.rectangle((430,187,520,250),fill='#cb814b',outline='#54351d',width=3)
        for offset in (15,40,65):draw.rectangle((435+offset,200,445+offset,230),fill='#302e2a')
        draw.text((20,18),'SIMULATED RGB - NOT A ROBOT CAMERA',fill='#183b30')
        draw.text((20,40),f'Frame time: {stamp}',fill='#183b30')
        rgb=BytesIO();image.save(rgb,format='JPEG',quality=88)
        frames.add('rgb',rgb.getvalue(),'image/jpeg',stamp,width=640,height=360,encoding='JPEG',frame='demo_rgbd')
        depth=np.tile(np.linspace(300,1900,640,dtype='<u2'),(360,1));depth[190:255,x:x+95]=700;depth[187:250,430:520]=900
        message=Obj(height=360,width=640,step=1280,is_bigendian=False,encoding='16UC1',data=depth.tobytes())
        preview,meta=depth_preview(message)
        frames.add('depth',preview,'image/jpeg',stamp,**meta)
        stop.wait(1/hz)


def _ros(frames,stop,config):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from sensor_msgs.msg import CompressedImage,Image
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=Node('x2_rgbd_web_preview');last={'rgb':0.,'depth':0.}
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
    interval=1/float(config.get('panel_camera_preview_hz',8.))
    def callback(message,kind):
        now=time.monotonic()
        if now-last[kind]<interval:return
        last[kind]=now
        try:
            if kind=='rgb':frames.compressed_rgb(message)
            else:
                data,meta=depth_preview(message,float(config.get('panel_camera_depth_near_m',.15)),
                                        float(config.get('panel_camera_depth_far_m',2.)))
                frames.add('depth',data,'image/jpeg',source_stamp(message),frame=message.header.frame_id,**meta)
        except Exception as error:
            with frames.lock:frames.errors[kind]=str(error)
    node.create_subscription(CompressedImage,config.get('panel_camera_rgb_topic',PREFIX+'rgb_image/compressed'),
                             lambda m:callback(m,'rgb'),qos)
    node.create_subscription(Image,config.get('panel_camera_depth_topic',PREFIX+'depth_image'),
                             lambda m:callback(m,'depth'),qos)
    try:
        while not stop.is_set() and rclpy.ok():rclpy.spin_once(node,timeout_sec=.05)
    finally:node.destroy_node();rclpy.shutdown()


def _work(config,backend,output,token,stop,ready):
    try:
        # Pin camera work away from the project's control CPUs when supported.
        import os
        try:
            if hasattr(os,'sched_setaffinity'):os.sched_setaffinity(0,set(config.get('panel_camera_cpu_affinity',[2,3])))
        except (OSError,ValueError):pass
        frames=CameraFrames(output,backend=backend)
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*unused):pass
            def reply(self,status,data,mime='application/json'):
                if not isinstance(data,bytes):data=json.dumps(data,ensure_ascii=False).encode()
                self.send_response(status);self.send_header('Content-Type',mime)
                self.send_header('Content-Length',str(len(data)));self.end_headers()
                try:self.wfile.write(data)
                except (BrokenPipeError,ConnectionResetError):pass
            def do_GET(self):
                if not secrets.compare_digest(self.headers.get('X-Camera-Token',''),token):return self.reply(403,dict(error='Forbidden'))
                try:
                    if self.path=='/state':return self.reply(200,frames.state())
                    if self.path.startswith('/frame/'):
                        frame=frames.get(self.path.removeprefix('/frame/'));return self.reply(200,frame['data'],frame['mime'])
                    self.reply(404,dict(error='Not found'))
                except ValueError as e:self.reply(409,dict(error=str(e)))
            def do_POST(self):
                if not secrets.compare_digest(self.headers.get('X-Camera-Token',''),token):return self.reply(403,dict(error='Forbidden'))
                try:
                    if self.path!='/capture':return self.reply(404,dict(error='Not found'))
                    length=int(self.headers.get('Content-Length',0))
                    if not 0<length<=1024:raise ValueError('Invalid camera request')
                    request=json.loads(self.rfile.read(length))
                    self.reply(200,frames.capture(request.get('frame_id')))
                except (ValueError,TypeError,AttributeError) as e:self.reply(409,dict(error=str(e)))
                except OSError:self.reply(507,dict(error='采集保存失败，请检查磁盘空间和目录权限'))
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler);server.daemon_threads=True
        server.timeout=.15;ready.put(dict(port=server.server_port))
        def receive():
            try:
                if backend=='demo':_demo(frames,stop,float(config.get('panel_camera_preview_hz',8.)))
                else:_ros(frames,stop,config)
            except Exception as error:
                with frames.lock:frames.error='相机接入失败：'+str(error)
        incoming=threading.Thread(target=receive,daemon=True);incoming.start()
        while not stop.is_set():server.handle_request()
        server.server_close();incoming.join(timeout=1.)
    except Exception as error:ready.put(dict(error=str(error)))


class CameraWorker:
    def __init__(self,config,backend,output):
        self.config,self.backend,self.output=dict(config),backend,Path(output)
        self.process=self.stop_event=self.ready=None;self.address=None
        self.lock=threading.Lock();self.token=secrets.token_urlsafe(32)
        hz=float(config.get('panel_camera_preview_hz',8.))
        near=float(config.get('panel_camera_depth_near_m',.15));far=float(config.get('panel_camera_depth_far_m',2.))
        if not np.isfinite([hz,near,far]).all() or not 1<=hz<=30 or not 0<near<far:
            raise ValueError('Invalid camera preview frequency or depth scale')

    def start(self):
        with self.lock:
            if self.process is not None and self.process.is_alive():return
            ctx=mp.get_context('spawn');self.stop_event=ctx.Event();self.ready=ctx.Queue(maxsize=1)
            self.address=None
            self.process=ctx.Process(target=_work,args=(self.config,self.backend,str(self.output),self.token,self.stop_event,self.ready),
                                     name='x2-camera-readonly',daemon=True)
            self.process.start()
            try:result=self.ready.get(timeout=5.)
            except Empty:raise ValueError('相机进程启动超时，请停止预览后重试')
            if 'error' in result:raise ValueError(result['error'])
            self.address=f"http://127.0.0.1:{result['port']}"

    def request(self,path,body=None):
        address=self.address
        if address is None:raise ValueError('请先开启相机预览')
        request=urllib.request.Request(address+path,data=None if body is None else json.dumps(body).encode(),
                    headers={'X-Camera-Token':self.token,'Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(request,timeout=3.) as response:return response.read(),response.headers['Content-Type']
        except urllib.error.HTTPError as error:raise ValueError(json.loads(error.read()).get('error','相机请求失败'))
        except OSError:raise ValueError('相机进程连接中断，可停止后重新开启预览')

    def state(self):
        if self.process is None:return dict(running=False,backend=self.backend,error=None)
        if not self.process.is_alive():return dict(running=False,backend=self.backend,error='相机进程已退出，可重新开启预览')
        if self.address is None:return dict(running=True,backend=self.backend,error='相机进程尚未就绪，请停止后重试')
        return json.loads(self.request('/state')[0])

    def download(self,name):
        if CAPTURE_NAME.fullmatch(name) is None:raise ValueError('采集文件名无效')
        path=self.output/name
        if path.is_symlink() or not path.is_file() or path.resolve().parent!=self.output.resolve():raise ValueError('采集文件不存在')
        return path.read_bytes(),'image/jpeg' if name.endswith('.jpg') else 'image/png'

    def close(self):
        with self.lock:
            if self.process is not None:
                self.stop_event.set();self.process.join(timeout=1.)
                if self.process.is_alive():self.process.terminate();self.process.join(timeout=1.)
                self.process.close();self.ready.close()
            self.process=self.stop_event=self.ready=None;self.address=None
