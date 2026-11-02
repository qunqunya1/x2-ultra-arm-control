"use strict";
const el=id=>document.getElementById(id);
let state,token,lastId=null,editing=false,busy=false;
const urls={raw:null,annotated:null};
const fmt=x=>typeof x==='number'?x.toFixed(1):'—';
async function command(path,body){
  if(!token)token=(await(await fetch('/api/session')).json()).token;
  const response=await fetch('/api/'+path,{method:'POST',headers:{'Content-Type':'application/json','X-Preview-Token':token},body:JSON.stringify(body)});
  const data=await response.json();if(!response.ok)throw new Error(data.error||'请求失败');return data;
}
async function configure(enabled){
  await command('config',{enabled,conf:Number(el('conf').value),masks:el('masks').checked});editing=false;
}
async function images(result){
  if(result.id===lastId)return;
  const loaded=await Promise.all(['raw','annotated'].map(async kind=>{
    const response=await fetch('/api/image/'+result.id+'/'+kind);
    if(!response.ok)throw new Error('结果帧已更新');
    const url=URL.createObjectURL(await response.blob()),probe=new Image();probe.src=url;
    try{await probe.decode();}catch(error){URL.revokeObjectURL(url);throw error;}return {kind,url};
  }));
  for(const {kind,url} of loaded){if(urls[kind])URL.revokeObjectURL(urls[kind]);urls[kind]=url;el(kind).src=url;el(kind).hidden=false;}
  lastId=result.id;
  el('source-frame').textContent='RGB 帧 '+result.source_frame_id;
  el('count').textContent=result.detections.length;
  el('inference').textContent=fmt(result.speed_ms.inference)+' ms';
  el('latency').textContent=fmt(result.fetch_to_result_ms)+' ms';
  const rows=result.detections.map(item=>{
    const row=document.createElement('tr');
    for(const text of [item.label,(item.confidence*100).toFixed(1)+'%',item.xyxy.map(x=>x.toFixed(0)).join(', '),item.polygon_xy?item.polygon_xy.length+' 个点':'—']){
      const cell=document.createElement('td');cell.textContent=text;row.append(cell);
    }return row;
  });
  if(!rows.length){const row=document.createElement('tr'),cell=document.createElement('td');cell.colSpan=4;cell.textContent='本帧未检测到目标';row.append(cell);rows.push(row);}
  el('detections').replaceChildren(...rows);
}
async function poll(){
  try{
    const response=await fetch('/api/state',{signal:AbortSignal.timeout(2500)});state=await response.json();
    el('phase').textContent=state.enabled?state.phase:'检测已暂停 · '+state.phase;
    el('model').textContent=`${state.model.run} / ${state.model.file} · ${state.model.task||'加载中'} · 输入 ${state.imgsz} · ${(Object.values(state.model.names||{})).join(' / ')}`;
    el('device').textContent=state.model.device||'加载中';
    el('toggle').disabled=busy||!state.model.task;
    el('toggle').textContent=state.enabled?'暂停检测':'开始检测';
    if(!editing){el('conf').value=state.conf;el('conf-value').textContent=state.conf.toFixed(2);el('masks').checked=state.masks;}
    el('notice').textContent=state.error||(!state.enabled?'点击开始检测；相机预览需在机器人网页单独开启。':'本页在电脑上运行 YOLO，只读取 RGB，不输出机器人动作。');
    if(state.latest){
      await images(state.latest);
      el('age').textContent=(state.latest.fresh?'结果年龄 ':'已暂停 / 过期 ')+fmt(state.latest.age_ms)+' ms';
      document.querySelector('.images').classList.toggle('stale',!state.latest.fresh);
    }
    el('save').disabled=!lastId||busy;
  }catch(error){el('notice').textContent='连接或图像更新异常：'+error.message;}
  finally{setTimeout(poll,150);}
}
el('toggle').addEventListener('click',async()=>{
  busy=true;el('toggle').disabled=true;
  try{await configure(!state.enabled);}catch(error){el('notice').textContent=error.message;}finally{busy=false;}
});
el('conf').addEventListener('input',()=>{editing=true;el('conf-value').textContent=Number(el('conf').value).toFixed(2);});
for(const id of ['conf','masks'])el(id).addEventListener('change',async()=>{
  editing=true;try{await configure(!!state?.enabled);}catch(error){el('notice').textContent=error.message;}
});
el('save').addEventListener('click',async()=>{
  if(!lastId)return;busy=true;
  try{const data=await command('save',{id:lastId});el('saved').textContent='已保存原图、检测图和 JSON：'+data.folder;}
  catch(error){el('saved').textContent='保存失败：'+error.message;}finally{busy=false;}
});
poll();
