"use strict";

function rgbDirectoryRecord(operation,value){
  return new Promise((resolve,reject)=>{
    if(typeof indexedDB==='undefined')return reject(new Error('浏览器无法记忆文件夹'));
    const opening=indexedDB.open('x2-rgb-capture',1);
    opening.onupgradeneeded=()=>opening.result.createObjectStore('settings');
    opening.onerror=()=>reject(opening.error);
    opening.onblocked=()=>reject(new Error('目录记忆被其他页面阻塞'));
    opening.onsuccess=()=>{
      const db=opening.result,tx=db.transaction('settings',operation==='read'?'readonly':'readwrite');
      const store=tx.objectStore('settings');let request;
      try{request=operation==='read'?store.get('directory'):operation==='clear'?store.delete('directory'):store.put(value,'directory');}
      catch(error){db.close();reject(error);return;}
      tx.oncomplete=()=>{db.close();resolve(request.result);};
      tx.onerror=tx.onabort=()=>{db.close();reject(tx.error||new Error('无法记忆文件夹'));};
    };
  });
}

class RgbCaptureFolder {
  constructor({picker=options=>window.showDirectoryPicker(options),record=rgbDirectoryRecord}={}){
    this.picker=picker;this.record=record;this.directory=null;this.permission='prompt';this.remembered=false;
  }
  async restore(){
    try{
      const handle=await this.record('read');
      if(handle?.kind==='directory'){
        this.directory=handle;this.permission=await handle.queryPermission({mode:'readwrite'});this.remembered=true;
      }
    }catch(error){this.remembered=false;}
  }
  async choose(){
    // Call the picker directly within the human click, before any other await.
    const handle=await this.picker({id:'x2-rgb-capture',mode:'readwrite',startIn:this.directory||'pictures'});
    if(handle?.kind!=='directory')throw new Error('请选择一个电脑文件夹');
    const permission=await handle.queryPermission({mode:'readwrite'});
    this.directory=handle;this.permission=permission;
    try{await this.record('write',handle);this.remembered=true;}
    catch(error){this.remembered=false;}
  }
  async authorize(){
    if(!this.directory)return false;
    this.permission=await this.directory.requestPermission({mode:'readwrite'});
    if(this.permission!=='granted')throw new Error('电脑文件夹未授权，请重新选择文件夹或使用浏览器下载');
    return true;
  }
  async clear(){
    try{await this.record('clear');}catch(error){if(this.remembered)throw error;}
    this.directory=null;this.permission='prompt';this.remembered=false;
  }
  async save(name,blob){
    if(!this.directory||this.permission!=='granted')throw new Error('请先授权电脑保存文件夹');
    if(!/^[A-Za-z0-9][A-Za-z0-9_.-]{0,219}$/.test(name)||name.includes('..'))throw new Error('采集文件名无效');
    try{await this.directory.getFileHandle(name);throw new Error('文件夹中已有同名文件，已避免覆盖，请重新采集');}
    catch(error){if(error.name!=='NotFoundError')throw error;}
    const handle=await this.directory.getFileHandle(name,{create:true});
    const writable=await handle.createWritable();
    try{await writable.write(blob);await writable.close();}
    catch(error){try{await writable.abort();}catch(unused){}throw error;}
    return this.directory.name+'/'+name;
  }
}
if(typeof module!=='undefined'&&module.exports)module.exports={RgbCaptureFolder};
(() => {
  if(typeof document==='undefined')return;
  const el=id=>document.getElementById(id);
  let session=null,running=false,changing=false,capturing=false,displayed=null,generation=0;
  const urls={rgb:null,depth:null},shown={rgb:null,depth:null};
  let rgbFresh=false,rgbBlob=null,folderLoading=true;
  const folder=new RgbCaptureFolder();
  const folderSupported=typeof window.showDirectoryPicker==='function'&&window.isSecureContext;
  function renderFolder(){
    const selected=folder.directory;
    el('camera-folder-name').value=selected?'电脑文件夹：'+selected.name:'浏览器默认下载文件夹';
    el('camera-folder-pick').textContent=selected?'更换文件夹':'选择文件夹';
    el('camera-folder-pick').disabled=!folderSupported||capturing||folderLoading;
    el('camera-folder-clear').hidden=!selected;
    el('camera-folder-clear').disabled=capturing||folderLoading;
    el('camera-folder-status').textContent=!folderSupported?'当前浏览器不支持选择电脑目录，请用 Edge 或 Chrome 打开；也可继续使用浏览器下载。':selected?(folder.permission==='granted'?'采集后直接保存到此文件夹，不再逐张弹窗。':'已记住文件夹，首次采集时需点击允许保存。')+(folder.remembered?' 刷新后仍会记住选择。':' 当前页面有效，目录记忆不可用。'):'选择一次后连续采集直接保存；重新打开网页时可能需要重新授权一次。';
  }
  function buttons(){
    el('camera-preview').textContent=changing?'正在切换…':running?'停止预览':'开启预览';
    el('camera-preview').disabled=changing||capturing;
    el('camera-capture').disabled=!running||!displayed||!rgbFresh||capturing||changing||folderLoading;
    el('camera-capture').textContent=capturing?'正在保存…':'采集当前 RGB 帧';
    renderFolder();
  }
  async function post(path,body,retry=true){
    if(!session)session=(await(await fetch('/api/session')).json()).token;
    const response=await fetch('/api/camera/'+path,{method:'POST',
      headers:{'Content-Type':'application/json','X-Panel-Token':session},
      body:JSON.stringify(body),signal:AbortSignal.timeout(8000)});
    if(response.status===403&&retry){session=null;return post(path,body,false);}
    const data=await response.json();if(!response.ok)throw new Error(data.error||'相机请求失败');
    return data;
  }
  function resetImages(){
    generation++;displayed=null;rgbBlob=null;rgbFresh=false;
    for(const kind of ['rgb','depth']){
      shown[kind]=null;el('camera-'+kind).hidden=true;
      el('camera-'+kind+'-placeholder').hidden=false;
      if(urls[kind])URL.revokeObjectURL(urls[kind]);urls[kind]=null;
    }
  }
  async function renderFrame(kind,frame,current){
    if(!frame||frame.id===shown[kind]||(kind==='rgb'&&capturing))return;
    const response=await fetch('/api/camera/frame/'+encodeURIComponent(frame.id),{signal:AbortSignal.timeout(2500)});
    if(!response.ok)throw new Error('显示帧已更新，正在获取新画面');
    const blob=await response.blob(),url=URL.createObjectURL(blob);
    const probe=new Image();probe.src=url;
    try{await probe.decode();}catch(error){URL.revokeObjectURL(url);throw error;}
    if(current!==generation||!running||(kind==='rgb'&&capturing)){URL.revokeObjectURL(url);return;}
    const image=el('camera-'+kind),previous=urls[kind];
    if(kind==='rgb'){rgbFresh=false;buttons();}
    image.src=url;
    try{await image.decode();}catch(error){URL.revokeObjectURL(url);throw error;}
    if(current!==generation||!running){URL.revokeObjectURL(url);return;}
    urls[kind]=url;shown[kind]=frame.id;image.hidden=false;
    el('camera-'+kind+'-placeholder').hidden=true;
    if(previous)URL.revokeObjectURL(previous);
    if(kind==='rgb'){
      displayed={id:frame.id,source_ns:frame.source_ns};rgbBlob=blob;rgbFresh=frame.fresh;
      const stamp=frame.source_ns>0?new Date(frame.source_ns/1e6).toLocaleTimeString('zh-CN',{timeZone:'Asia/Shanghai',hour12:false}):'无时间戳';
      el('camera-rgb-meta').textContent=`${frame.width} × ${frame.height} · ${stamp} · 帧 ${frame.id.split('_').pop()}`;
    }else el('camera-depth-meta').textContent=`${frame.width} × ${frame.height} · ${frame.near_m}–${frame.far_m} m · 黑色为无效深度`;
    buttons();
  }
  async function poll(){
    try{
      const response=await fetch('/api/camera/state',{signal:AbortSignal.timeout(2000)});
      const meta=await response.json();
      if(!response.ok)throw new Error(meta.error||'相机状态读取失败');
      const wasRunning=running;running=!!meta.running;
      if(wasRunning&&!running)resetImages();
      el('camera-source').textContent=!running?'预览未开启':meta.backend==='demo'?'模拟 RGB-D · 非实机画面':'机器人头部 RGB-D';
      el('camera-source').className='pill '+(meta.backend==='demo'&&running?'warn':!running?'neutral':'');
      el('camera-status').textContent=meta.error||(running?(meta.rgb?.fresh?'实时预览 · RGB 单帧采集':'等待相机图像 / 图像已过期'):'独立于手臂控制，可单独查看相机。');
      if(running){
        const current=generation;
        for(const kind of ['rgb','depth'])el('camera-'+kind).parentElement.classList.toggle('stale',!meta[kind]?.fresh);
        rgbFresh=!!meta.rgb?.fresh;
        await Promise.all([renderFrame('rgb',meta.rgb,current),renderFrame('depth',meta.depth,current)]);
      }
      buttons();
    }catch(error){
      rgbFresh=false;buttons();el('camera-status').textContent=error.message;
      for(const kind of ['rgb','depth'])el('camera-'+kind).parentElement.classList.add('stale');
    }finally{setTimeout(poll,running?125:700);}
  }
  el('camera-preview').addEventListener('click',async()=>{
    changing=true;buttons();
    try{
      const next=!running;generation++;
      await post('preview',{enabled:next});running=next;resetImages();
    }catch(error){el('camera-status').textContent=error.message;}
    finally{changing=false;buttons();}
  });
  el('camera-capture').addEventListener('click',async()=>{
    if(!displayed||capturing||!rgbFresh||folderLoading)return;
    const chosen={...displayed},chosenBlob=rgbBlob;capturing=true;buttons();
    let capture=null;
    const result=el('camera-capture-result');
    try{
      // Obtain write access while this click still has user activation.
      const direct=await folder.authorize();
      capture=await post('capture',{frame_id:chosen.id});
      if(capture.frame_id!==chosen.id)throw new Error('采集返回的帧编号不一致');
      result.replaceChildren();
      const link=document.createElement('a');link.href=capture.download_url;link.download=capture.name;
      link.textContent='下载此图';
      if(direct){
        if(!chosenBlob)throw new Error('当前显示图像未就绪，请重新采集');
        const saved=await folder.save(capture.name,chosenBlob);
        result.append(document.createTextNode('已保存到电脑：'+saved));
      }else{
        result.append(document.createTextNode('已采集当前 RGB：'+capture.name));
      }
      result.append(link);
      const note=document.createElement('small');
      note.textContent=`原始分辨率 ${capture.width} × ${capture.height} · 帧 ${capture.frame_id} · 后端 ${capture.saved_under} · 不含深度`;
      result.append(note);if(!direct)link.click();
    }catch(error){
      result.replaceChildren();
      result.append(document.createTextNode((capture?'后端已采集，电脑保存未完成：':'采集未完成：')+error.message));
      if(capture){const link=document.createElement('a');link.href=capture.download_url;link.download=capture.name;link.textContent='下载已采集的 RGB';result.append(link);}
    }
    finally{capturing=false;buttons();}
  });
  el('camera-folder-pick').addEventListener('click',async()=>{
    folderLoading=true;buttons();
    try{await folder.choose();}
    catch(error){if(error.name!=='AbortError')el('camera-capture-result').textContent='文件夹选择未完成：'+error.message;}
    finally{folderLoading=false;buttons();}
  });
  el('camera-folder-clear').addEventListener('click',async()=>{
    folderLoading=true;buttons();
    try{await folder.clear();}
    catch(error){el('camera-capture-result').textContent='无法清除目录记忆：'+error.message;}
    finally{folderLoading=false;buttons();}
  });
  folder.restore().finally(()=>{folderLoading=false;buttons();});
  window.addEventListener('pagehide',()=>{
    // Closing this page does not stop another page's read-only camera process.
    for(const kind of ['rgb','depth'])if(urls[kind])URL.revokeObjectURL(urls[kind]);
  });
  poll();
})();
