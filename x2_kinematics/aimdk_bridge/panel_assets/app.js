"use strict";
const $ = id => document.getElementById(id);
const names = ["肩俯仰", "肩侧摆", "肩旋转", "肘关节", "腕偏航", "腕俯仰", "腕旋转"];
const components = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"];
const clientId = crypto.randomUUID();
let token, state, draft = null, sequence = 0, targetTimer, toastTimer, lastFetch = 0, wasOwner = false;
const histories = {}, forceStamps = {};
const gripperDraft={left:null,right:null},gripperTimers={};let gripperSequence=0;
const gripperEffort={left:null,right:null},cartesianDraft={left:[0,0,0,0,0,0],right:[0,0,0,0,0,0]};
let pdDirty=false,cartesianGainsDirty=false,cartesianTimer,cartesianReference=null,modePending=null,dragPending=null,dragPendingUntil=0;
const deg = x => x * 180 / Math.PI;
const rad = x => x * Math.PI / 180;
const finite = x => typeof x === "number" && Number.isFinite(x);
const fmt = (x, digits=1) => finite(x) ? x.toFixed(digits) : "—";
const reasons = {poor_observability:"姿态可观测性不足",virtual_flange_calibration_unconfirmed:"法兰坐标未标定",fixed_upright_pelvis_assumption_unconfirmed:"骨盆支撑条件待确认",sdk_effort_feedback_semantics_unconfirmed:"力矩反馈含义待确认",onboard_sdk_release_not_verified:"SDK 待核验",outside_urdf_planning_limits:"部分反馈超出模型规划限位",base_imu_not_ready:"胯部 IMU 尚未有效或稳定",base_motion_estimate_diagnostic_only:"骨盆运动中，末端力估计仅作诊断"};
Object.assign(reasons,{waist_posture_changed_restart_required:"接管后腰部偏离参考姿态，需要处理后重启后端",waist_moving_arm_only_model_invalid:"腰部仍在运动，暂不能使用静止腰部模型",waist_feedback_missing_or_stale:"腰部数据缺失或时间戳过期",waist_settling:"等待腰部稳定 0.5 秒",waiting_for_waist_feedback:"等待第一帧腰部消息",waist_not_stationary:"腰部尚未稳定，估力仅作诊断"});
function translate(x) { return reasons[x] || x; }
function toast(text) { $("toast").textContent = text; $("toast").style.display="block"; clearTimeout(toastTimer); toastTimer=setTimeout(()=>$("toast").style.display="none",4000); }
function build() {
  for (const [side, label, offset] of [["left","左臂",0],["right","右臂",7]]) {
    const card=document.createElement("article"); card.className="arm-card";
    card.innerHTML=`<div class="card-head"><div><h3>${label}</h3><small>${side.toUpperCase()} ARM</small></div><span class="arm-index">7 DOF</span></div>`;
    names.forEach((name,j)=>{
      const i=offset+j, row=document.createElement("div"); row.className="joint-row";
      row.innerHTML=`<div class="joint-heading"><span><span class="joint-number">0${j+1}</span>${name}</span><span class="joint-actual" id="actual-${i}">实际 —</span></div><div class="target-line"><span>目标预览</span><span class="target-value" id="target-${i}">—</span></div><input type="range" id="joint-${i}" aria-label="${label}${name}目标角度" min="-180" max="180" step="0.1" value="0" disabled><div class="joint-range"><span id="min-${i}">—</span><span id="max-${i}">—</span></div>`;
      row.insertAdjacentHTML('beforeend',`<div class="pd-inputs"><label>Kp <input id="kp-${i}" type="number" min="0" step="0.1" aria-label="${label}${name}Kp"></label><label>Kd <input id="kd-${i}" type="number" min="0" step="0.05" aria-label="${label}${name}Kd"></label></div><div id="pd-actual-${i}" class="pd-actual">实际下发 P / D —</div>`);
      for(const key of ['kp','kd'])row.querySelector('#'+key+'-'+i).addEventListener('input',()=>{pdDirty=true;});
      card.append(row);
      row.querySelector("input").addEventListener("input", e=>{
        if(!draft)return; draft[i]=rad(Number(e.target.value)); $("target-"+i).textContent=fmt(deg(draft[i]))+"°";
        clearTimeout(targetTimer);
        if(ownsControl()&&state.active_control_mode==='joint') targetTimer=setTimeout(()=>command("targets",{q_rad:[...draft],sequence:++sequence}).catch(e=>toast(e.message)),80);
      });
    });
    $("arms").append(card);
    const force=document.createElement("article"); force.className="force-card";
    force.innerHTML=`<div class="card-head"><h3>${label}末端</h3><span class="pill neutral" id="force-status-${side}">等待数据</span></div><div class="force-grid">${components.map((c,j)=>`<div class="force-cell invalid" id="cell-${side}-${j}"><div class="force-label"><span>${c}</span><span>${j<3?"N":"N·m"}</span></div><div class="force-value" id="value-${side}-${j}">—</div><svg class="spark" viewBox="0 0 100 23" preserveAspectRatio="none" aria-hidden="true"><polyline id="line-${side}-${j}" points=""></polyline></svg></div>`).join("")}</div><div class="force-meta" id="force-meta-${side}">坐标：${label}虚拟法兰局部坐标</div>`;
    $("forces").append(force);
    const pose=document.createElement('article');pose.className='pose-card';
    pose.innerHTML=`<div class="card-head"><h3>${label}法兰</h3><span id="pose-state-${side}" class="pill neutral">等待反馈</span></div><div class="pose-grid">${['X','Y','Z','Roll','Pitch','Yaw'].map((axis,j)=>`<div class="pose-cell"><span>${axis} <small>${j<3?'mm':'°'}</small></span><strong id="pose-actual-${side}-${j}">—</strong><small id="pose-target-${side}-${j}">目标 —</small></div>`).join('')}</div><details class="pose-details"><summary>姿态约定与四元数</summary><p>R = Rz(Yaw) Ry(Pitch) Rx(Roll)</p><p id="pose-quaternion-${side}">xyzw：—</p></details>`;
    $('current-poses').append(pose);
    const cart=document.createElement('article');cart.className='arm-card';
    cart.innerHTML=`<div class="card-head"><h3>${label}末端</h3><span class="arm-index">6D IMPEDANCE</span></div>`;
    ['X','Y','Z','Rx','Ry','Rz'].forEach((axis,j)=>{
      const row=document.createElement('div');row.className='cartesian-row';
      row.innerHTML=`<div class="cartesian-axis"><span>Δ${axis}</span><span id="cartesian-value-${side}-${j}">0 ${j<3?'mm':'°'}</span></div><input type="range" id="cartesian-target-${side}-${j}" min="${j<3?-30:-10}" max="${j<3?30:10}" step="0.1" value="0" aria-label="${label}末端${axis}偏移"><div class="pd-inputs"><label>K <input id="cartesian-k-${side}-${j}" type="number" min="0" step="${j<3?1:.1}" aria-label="${label}${axis}刚度"></label><label>D <input id="cartesian-d-${side}-${j}" type="number" min="0" step="${j<3?.1:.01}" aria-label="${label}${axis}阻尼"></label></div>`;
      cart.append(row);
      row.querySelector('input[type=range]').addEventListener('input',event=>{
        const value=Number(event.target.value);cartesianDraft[side][j]=j<3?value/1000:rad(value);
        $('cartesian-value-'+side+'-'+j).textContent=fmt(value)+' '+(j<3?'mm':'°');
        clearTimeout(cartesianTimer);
        if(ownsControl()&&state.active_control_mode==='cartesian')cartesianTimer=setTimeout(sendCartesian,80);
      });
      for(const key of ['k','d'])row.querySelector('#cartesian-'+key+'-'+side+'-'+j).addEventListener('input',()=>{cartesianGainsDirty=true;});
    });
    cart.insertAdjacentHTML('beforeend',`<div id="cartesian-position-${side}" class="cartesian-position">法兰位置：等待反馈</div>`);
    $('cartesian-arms').append(cart);
  }
  $("endpoint").textContent=location.host;
  for(const [side,label] of [["left","左夹爪"],["right","右夹爪"]]){
    const card=document.createElement('article');card.className='gripper-card';
    card.innerHTML=`<div class="gripper-heading"><h3>${label}</h3><span class="joint-actual" id="gripper-actual-${side}">实际 —</span></div><div class="target-line"><span>目标开度</span><span class="target-value" id="gripper-value-${side}">—</span></div><input type="range" id="gripper-${side}" aria-label="${label}目标开度" min="0" max="100" step="1" value="0" disabled><div class="joint-range"><span>0% 全闭</span><span>100% 全开</span></div><div class="target-line gripper-effort-line"><span>夹持力度上限</span><span id="gripper-effort-value-${side}" class="target-value">—</span></div><input id="gripper-effort-${side}" type="range" min="0" max="100" step="1" value="20" aria-label="${label}夹持力度上限" disabled><div class="gripper-actions"><label><input type="checkbox" id="gripper-live-${side}" disabled>拖动实时下发</label><button id="gripper-send-${side}" disabled>发送开度与力度</button></div><div class="gripper-meta" id="gripper-meta-${side}">等待夹爪反馈</div>`;
    $("grippers").append(card);
    $("gripper-"+side).addEventListener('input',event=>{
      gripperDraft[side]=Number(event.target.value)/100;$("gripper-value-"+side).textContent=fmt(gripperDraft[side]*100,0)+'%';
      clearTimeout(gripperTimers[side]);
      if($("gripper-live-"+side).checked)gripperTimers[side]=setTimeout(()=>sendGripper(side),100);
    });
    $("gripper-send-"+side).addEventListener('click',()=>sendGripper(side));
    $("gripper-live-"+side).addEventListener('change',()=>{clearTimeout(gripperTimers[side]);});
    $('gripper-effort-'+side).addEventListener('input',event=>{
      gripperEffort[side]=Number(event.target.value)/100;
      $('gripper-effort-value-'+side).textContent=fmt(gripperEffort[side]*100,0)+'%';
      clearTimeout(gripperTimers[side]);
      if($('gripper-live-'+side).checked)gripperTimers[side]=setTimeout(()=>sendGripper(side),100);
    });
  }
}
async function sendGripper(side){
  const opening=gripperDraft[side];if(!finite(opening))return;
  try{await command('gripper_target',{side,opening,effort:gripperEffort[side],sequence:++gripperSequence});toast('已提交夹爪开度 '+fmt(opening*100,0)+'% / 力度 '+fmt(gripperEffort[side]*100,0)+'%');}
  catch(error){toast(error.message);}
}
function ownsControl() { return !!state && state.control_owner===clientId && state.control_state==="armed" && Date.now()-lastFetch<500; }
async function sendCartesian(){
  try{await command('cartesian_targets',{offsets:cartesianDraft,sequence:++sequence,reference:cartesianReference});}catch(error){toast(error.message);}
}
function numberValue(id){const n=$(id).valueAsNumber;if(!finite(n)||n<0)throw new Error('请填写非负的有限参数');return n;}
function renderTuning(owner){
  const selected=state.selected_control_mode||'joint',cart=selected==='cartesian',settings=state.cartesian;
  const configure=state.control_available&&(!state.control_owner||owner);
  const dragging=!!settings?.live?.drag_enabled;
  if(dragPending!==null&&(dragPending===dragging||Date.now()>dragPendingUntil||!owner))dragPending=null;
  $('drag-panel').hidden=!cart;
  $('drag-panel').classList.toggle('dragging',dragging);
  $('drag-mode').checked=dragPending??dragging;
  $('drag-mode').disabled=!owner||state.active_control_mode!=='cartesian'||dragPending!==null;
  $('drag-status').textContent=dragPending!==null?'正在平滑切换，请等待状态更新。':dragging?'拖拽中：重力补偿＋关节阻尼；关闭后从当前实际位姿恢复阻抗。':owner?'打开即可手动拖动双臂；关闭时捕获当前位置，偏移归零。':'启动笛卡尔控制后可开启；关闭时以当前实际位姿建立新的阻抗目标。';
  if(modePending===selected)modePending=null;
  $('control-mode').value=modePending||selected;
  $('control-mode').disabled=state.control_state==='armed'||!configure;
  $('control-mode').querySelector('option[value=cartesian]').disabled=!settings?.available;
  $('joint-section').hidden=cart;$('cartesian-panel').hidden=!cart;$('sync').hidden=cart;
  $('mode-help').textContent=state.active_control_mode&&state.active_control_mode!==selected?'当前仍在保持原模式；点击启动以切换。':cart?'笛卡尔：启动时捕获法兰参考，目标平滑跟踪。':'关节：每个关节独立设置 PD，点击应用后生效。';
  $('apply-pd').disabled=!configure||cart;
  $('apply-cartesian-gains').disabled=!configure||!cart;
  $('cartesian-rebase').disabled=!owner||state.active_control_mode!=='cartesian'||!state.cartesian_pose?.fresh||dragging;
  const g=state.joint_gains;
  if(g)for(let i=0;i<14;i++){
    for(const key of ['kp','kd']){
      const input=$(key+'-'+i);input.max=g[key+'_max'][i];input.disabled=!configure||cart;
      if(!pdDirty)input.value=g[key+'_target'][i];
    }
    $('pd-actual-'+i).textContent='当前命令 P / D '+fmt(g.kp_actual?.[i])+' / '+fmt(g.kd_actual?.[i],2);
  }
  $('pd-status').textContent=`Kp：N·m/rad，Kd：N·m·s/rad · 本次运行生效 · ${pdDirty?'有未应用的编辑':'在线修改平滑过渡'} · 版本 ${state.parameter_revision??0}`;
  if(!settings)return;
  const current=settings.live,key=current?`${current.started_ns}:${current.reference_generation}`:null;
  if(key!==cartesianReference){
    clearTimeout(cartesianTimer);cartesianReference=key;
    for(const side of ['left','right'])for(let j=0;j<6;j++){
      cartesianDraft[side][j]=current?.offsets?.[side]?.[j]??0;
      const v=j<3?cartesianDraft[side][j]*1000:deg(cartesianDraft[side][j]);
      $('cartesian-target-'+side+'-'+j).value=v;$('cartesian-value-'+side+'-'+j).textContent=fmt(v)+' '+(j<3?'mm':'°');
    }
  }
  for(const [i,side] of ['left','right'].entries()){
    for(let j=0;j<6;j++){
      const slider=$('cartesian-target-'+side+'-'+j),bound=j<3?settings.offset_limit_m*1000:deg(settings.rotation_limit_rad);
      slider.min=-bound;slider.max=bound;slider.disabled=!owner||state.active_control_mode!=='cartesian'||dragging;
      for(const [short,long] of [['k','stiffness'],['d','damping']]){
        const input=$('cartesian-'+short+'-'+side+'-'+j);input.disabled=!configure||!cart;
        input.max=settings[long+'_max'][j];if(!cartesianGainsDirty)input.value=settings[long][i][j];
      }
    }
    const pose=current?.actual_poses?.[side];
    $('cartesian-position-'+side).textContent=pose?'法兰位置 / torso_link：'+['X','Y','Z'].map((axis,i)=>axis+' '+fmt(pose[i][3]*1000)+' mm').join(' · '):'启动时捕获当前末端参考';
  }
  $('cartesian-status').textContent=state.active_control_mode==='cartesian'?(state.fault?'故障：任务力矩已撤除，仅保留重力与关节阻尼。':dragging?'双臂拖拽中；末端目标滑条已暂停，关闭后从当前位姿恢复。':state.control_state==='armed'?'末端弹簧 / 阻尼控制中；拖动偏移滑条调整目标。':'保持捕获的末端姿态；重新启动后可调目标。'):settings.available?'尚未接管；点击启动后以当前实际法兰位姿为初始目标。':'实机笛卡尔模式未启用。';
}
function renderPoses(){
  const pose=state.cartesian_pose,fresh=!!pose?.fresh&&Date.now()-lastFetch<500;
  $('pose-status').textContent=!pose?'等待新版位姿反馈':fresh?`实时更新 · 消息年龄 ${fmt(pose.source_age_ms,0)} ms`:'数据过期 / 等待有效反馈';
  for(const side of ['left','right']){
    const item=pose?.arms?.[side],actual=item?.actual,target=item?.target;
    $('pose-state-'+side).textContent=!actual?'等待反馈':!fresh?'已过期':actual.rpy_singular?'RPY 奇异 · 见四元数':'实时位姿';
    $('pose-state-'+side).className='pill '+(fresh?'':'neutral');
    for(let j=0;j<6;j++){
      const a=j<3?actual?.xyz_m?.[j]:actual?.rpy_rad?.[j-3];
      const t=j<3?target?.xyz_m?.[j]:target?.rpy_rad?.[j-3];
      $('pose-actual-'+side+'-'+j).textContent=fmt(finite(a)?(j<3?a*1000:deg(a)):null,2);
      $('pose-actual-'+side+'-'+j).classList.toggle('stale-pose',!fresh);
      $('pose-target-'+side+'-'+j).textContent=(pose?.target_is_preview?'目标预览 ':'目标 ')+fmt(finite(t)?(j<3?t*1000:deg(t)):null,2);
    }
    $('pose-quaternion-'+side).textContent='xyzw：'+(actual?actual.quaternion_xyzw.map(x=>fmt(x,5)).join(', '):'—');
  }
}
async function command(op,extra={}) {
  const response=await fetch("/api/command",{method:"POST",headers:{"Content-Type":"application/json","X-Panel-Token":token},body:JSON.stringify({op,client_id:clientId,...extra}),signal:AbortSignal.timeout(1200),keepalive:op==="hold"});
  const result=await response.json(); if(!response.ok)throw new Error(result.error||"请求失败"); return result;
}
function syncDraft() {
  if(!state?.q_rad)return;
  draft=[...state.q_rad];
  draft.forEach((q,i)=>{ $("joint-"+i).value=deg(q); $("target-"+i).textContent=fmt(deg(q))+"°"; });
}
function render() {
  const live=state.connection==="live", owner=ownsControl(), demo=state.backend==="demo",dragging=!!state.cartesian?.live?.drag_enabled;
  renderTuning(owner);
  renderPoses();
  $("mode").textContent=demo?"数值仿真":dragging&&state.control_state==="armed"?"实机 · 拖拽中":state.control_state==="armed"?"实机 · 控制中":state.control_state==="fault"?"实机 · 控制故障":state.control_state==="holding"?"实机 · 保持中":"实机 · 观察中";
  $("mode").className="pill "+(demo?"warn":"");
  $("connection").textContent=live?"● 反馈已连接":"● 等待 / 反馈过期";
  $("connection").className="pill "+(live?"":"neutral");
  const labels={observe:"只读观察",idle:"尚未接管",armed:"滑块控制中",holding:"位置保持",fault:"故障保持"};
  const cartesianActive=state.active_control_mode==='cartesian';
  $("control-state").textContent=cartesianActive?(state.control_state==='fault'?'故障 · 重力与关节阻尼':state.control_state==='armed'?(dragging?'双臂拖拽 · 重力补偿＋阻尼':'笛卡尔阻抗控制中'):'末端保持'):state.control_state==="fault"?(state.control_commands_generated?"故障 · 已请求保持":"故障 · 未发送控制指令"):(labels[state.control_state]||"等待反馈");
  $("limit-note").textContent=state.selected_control_mode==='cartesian'?`末端限速 ${fmt((state.cartesian?.max_velocity_m_s??.01)*1000)} mm/s · ${fmt(deg(state.cartesian?.max_angular_velocity_rad_s??.05))}°/s`:`速度上限 ${fmt(deg(state.velocity_limit_rad_s))}°/s · ${owner?"拖动后平滑跟踪":"滑块仅预览目标"}`;
  $('hold-explanation').textContent=cartesianActive?'保持按钮捕获当前末端姿态。笛卡尔故障时撤除任务力矩，仅保留重力与关节阻尼；这不是硬件急停。':'保持按钮请求关节位置保持，仍会输出关节指令，不是硬件急停。';
  $("arm").textContent=owner?"控制已启动":demo?"启动仿真控制":"启动实机控制";
  $("arm").disabled=!live || !state.control_available || !!state.control_owner || !!modePending;
  $("hold").disabled=!owner;
  $('hold').textContent=state.active_control_mode==='cartesian'?'保持当前末端':'保持当前姿态';
  $("sync").disabled=!live;
  const checks=state.startup_checks||[], failed=checks.filter(c=>!c.ok);
  $("readiness-title").textContent=failed.length?`启动条件：${failed.length} 项待完成`:(owner?"控制已启动 · 检查通过":"启动条件已满足 · 点击按钮接管");
  if(state.fault)$("readiness-title").textContent=failed.length?`重新接管：${failed.length} 项待恢复`:"可点击启动从当前角度重新接管";
  $("startup-checks").replaceChildren(...checks.map(c=>{
    const li=document.createElement("li");li.className=c.ok?"ready":"pending";
    let detail="";
    if(c.key==="feedback"&&finite(state.feedback_source_age_ms))detail=` · 消息年龄 ${fmt(state.feedback_source_age_ms,0)} ms`;
    if(c.key==="base_imu"&&state.base_imu){const b=state.base_imu;detail=` · 倾角 ${fmt(b.orientation_tilt_deg)}° · 角速度 ${fmt(b.angular_speed_rad_s,3)} rad/s`;if(finite(b.angular_speed_filtered_rad_s))detail+=`（滤波 ${fmt(b.angular_speed_filtered_rad_s,3)}）`;if(finite(b.acceleration_error_m_s2))detail+=` · Δa ${fmt(b.acceleration_error_m_s2,2)} m/s²`;if(!b.motion_guard_enabled)detail+=' · 运动不触发退出';if(!c.ok){detail+=' · '+(b.reason||'等待稳定');if(b.reason==='等待骨盆持续稳定')detail+=` ${fmt(b.stable_s,2)}/${fmt(b.warmup_s??1,1)} s`;if(b.last_rejection?.reason){detail+=' · 上次触发：'+b.last_rejection.reason;const m=b.last_rejection.metrics;if(m)detail+=`（当时角速率 ${fmt(m.angular_speed_rad_s,3)}、Δa ${fmt(m.acceleration_error_m_s2,2)}）`;}}}
    if((c.key==="waist"||c.key==="waist_stationary")&&state.waist_feedback){const w=state.waist_feedback;detail=c.key==="waist"?` · 消息年龄 ${fmt(w.source_age_ms,0)} ms`:` · 稳定 ${fmt(w.stable_s)} s`;if(!c.ok&&w.reason)detail+=" · "+translate(w.reason);}
    if(c.key==="limits"&&state.joint_limit_violations?.length)detail=" · "+(c.ok?"允许向范围内回收：":"")+state.joint_limit_violations.map(v=>{const i=state.joint_names.indexOf(v.name);return `${i<7?"左":"右"}${names[i%7]} ${fmt(deg(v.position_rad))}°，范围 ${fmt(deg(v.lower_rad))}～${fmt(deg(v.upper_rad))}°`;}).join("；");
    li.textContent=(c.ok?"✓ ":"○ ")+c.label+detail;return li;
  }));
  if(state.error && state.error.startsWith("尚不能启动"))$("readiness").open=true;
  if(!draft || (wasOwner && !owner) || (!wasOwner && owner))syncDraft();
  wasOwner=owner;
  if(state.q_rad) state.q_rad.forEach((q,i)=>{
    const input=$("joint-"+i);
    input.min=deg((state.slider_lower_rad||state.lower_rad)[i]); input.max=deg((state.slider_upper_rad||state.upper_rad)[i]); input.disabled=!live || state.selected_control_mode==='cartesian' || (!!state.control_owner&&!owner);
    $("actual-"+i).textContent="实际 "+fmt(deg(q))+"°";
    $("min-"+i).textContent=fmt(deg(state.lower_rad[i]),0)+"°";
    $("max-"+i).textContent=fmt(deg(state.upper_rad[i]),0)+"°";
  });
  const hints=[...(state.quality?.reasons||[]),...(state.quality?.warnings||[])].map(translate);
  let note=demo?"Pinocchio 数值仿真 · 滑块只作用于虚拟动力学模型，未连接真实电机。":state.mode==="observe"?"强制只读连接 · 滑块仅预览，不允许启动控制。":owner?"实机控制已启动 · 滑块目标经限速后跟踪。":state.control_state==="fault"&&!state.motor_commands_sent?"启动未完成，故障已锁存，尚未下发电机指令。":state.control_state==="idle"&&state.hardware_command_publisher_created?"控制接口已准备，尚未接管；条件恢复后可再次点击启动。":state.hardware_command_publisher_created?"当前处于位置保持；重新启动会从最新反馈角度接管。":"当前仅观察反馈。点击“启动实机控制”后检查条件，通过后从当前角度接管。";
  if(cartesianActive&&state.control_state==='fault'&&state.control_commands_generated)note='笛卡尔故障回退：仍在输出重力补偿与关节阻尼，末端位置不受保持。';
  if(dragging&&owner)note=demo?'本地数值仿真 · 双臂拖拽中；未连接真实电机。':'双臂拖拽中 · 重力补偿＋关节阻尼；关闭开关后捕获当前实际位姿，不回到旧目标。';
  if(state.fault||state.error||state.arm_blocked)note+=" "+translate(state.fault||state.error||state.arm_blocked);
  if(state.limit_recovery_enabled&&state.limit_recovery_allowed&&state.joint_limit_violations?.length)note+=" 小幅越界回收：启动后先保持当前角度，越界关节只接受向范围内的滑块目标，不会自动回零。";
  if(hints.length)note+=" "+hints.join("；")+"。";
  $("notice").textContent=note;
  const w=state.wrench, stale=!live || !finite(state.wrench_age_ms) || state.wrench_age_ms>150;
  for(const side of ["left","right"]) {
    const arm=w?.arms?.[side], good=!!arm?.valid && !stale && state.quality?.physical_assumptions_confirmed;
    $("force-status-"+side).textContent=!arm?"等待窗口":stale?"数据过期":good?(demo?"仿真有效":"有效"):"诊断预览";
    $("force-status-"+side).className="pill "+(good?"":"warn");
    $("force-meta-"+side).textContent=`局部法兰系 · 条件数 ${fmt(arm?.scaled_condition)}${arm?.reasons?.length?" · "+arm.reasons.map(translate).join("，"):""}`;
    components.forEach((c,j)=>{
      const value=arm?.wrench?.[j], key=side+j;
      $("value-"+side+"-"+j).textContent=fmt(value,2);
      $("cell-"+side+"-"+j).classList.toggle("invalid",!good);
      if(finite(value) && !stale && forceStamps[key]!==w.available_at_s) {
        const h=histories[key] ||= []; h.push(value); if(h.length>80)h.shift(); forceStamps[key]=w.available_at_s;
        const lo=Math.min(...h), hi=Math.max(...h), span=Math.max(hi-lo,.1);
        $("line-"+side+"-"+j).setAttribute("points",h.map((x,k)=>`${k*100/79},${20-(x-lo)*16/span}`).join(" "));
      }
    });
  }
  $("feedback-age").textContent="关节消息 "+fmt(state.feedback_source_age_ms??state.feedback_age_ms,0)+" ms";
  $("display-age").textContent="角度显示年龄 "+fmt(state.feedback_age_ms,0)+" ms";
  $("force-age").textContent="估计更新 "+fmt(state.wrench_age_ms,0)+" ms";
  $("command-count").textContent="真实电机指令 "+(state.motor_commands_sent||0);
  for(const side of ['left','right']){
    const hand=state.grippers?.sides?.[side],ready=!!hand?.ready;
    const available=ready&&state.gripper_available&&(!state.control_owner||state.control_owner===clientId);
    $("gripper-actual-"+side).textContent='实际 '+(finite(hand?.position)?fmt(hand.position*100,0)+'%':'—');
    if(gripperDraft[side]===null&&finite(hand?.position)){
      gripperDraft[side]=hand.position;$("gripper-"+side).value=hand.position*100;$("gripper-value-"+side).textContent=fmt(hand.position*100,0)+'%';
    }
    $("gripper-"+side).disabled=!ready;
    $('gripper-effort-'+side).disabled=!available;
    if(gripperEffort[side]===null){gripperEffort[side]=state.gripper_drives?.[side]?.effort??.2;$('gripper-effort-'+side).value=gripperEffort[side]*100;$('gripper-effort-value-'+side).textContent=fmt(gripperEffort[side]*100,0)+'%';}
    $("gripper-send-"+side).disabled=$("gripper-live-"+side).disabled=!available;
    if(!available){clearTimeout(gripperTimers[side]);$("gripper-live-"+side).checked=false;}
    const status=['已到位','移动中','堵转','物体掉落'][hand?.state]||'等待反馈';
    const drive=state.gripper_drives?.[side]||state.gripper_drive;
    const effortLabel=Object.hasOwn(state.gripper_last_targets||{},side)?'已设置力度上限':'预设力度（尚未下发）';
    $("gripper-meta-"+side).textContent=ready?`${status} · 速度 ${fmt((drive?.velocity??.2)*100,0)}% · ${effortLabel} ${fmt((drive?.effort??.2)*100,0)}%`:(hand?.reason||'等待夹爪反馈');
  }
  $("gripper-note").textContent=state.gripper_error?('夹爪：'+state.gripper_error):'左右独立控制。默认只预览，点击发送目标或开启拖动实时下发；无需启动手臂控制。';
}
async function poll() {
  try {
    if(!token)token=(await (await fetch("/api/session",{signal:AbortSignal.timeout(1500)})).json()).token;
    const response=await fetch("/api/state",{signal:AbortSignal.timeout(1500)}); if(!response.ok)throw new Error("反馈读取失败");
    const next=await response.json();
    if(state && next.instance_id!==state.instance_id){token=null;draft=null;sequence=0;wasOwner=false;gripperSequence=0;pdDirty=false;cartesianGainsDirty=false;modePending=null;cartesianReference=null;dragPending=null;clearTimeout(cartesianTimer);for(const side of ['left','right']){clearTimeout(gripperTimers[side]);gripperDraft[side]=gripperEffort[side]=null;$("gripper-live-"+side).checked=false;}}
    state=next; lastFetch=Date.now(); render();
  } catch(e) {
    $("connection").textContent="● 连接已断开"; $("connection").className="pill warn";
    $("arm").disabled=$("hold").disabled=true;
    clearTimeout(targetTimer);clearTimeout(cartesianTimer);modePending=null;
    $('control-mode').disabled=$('apply-pd').disabled=$('apply-cartesian-gains').disabled=true;
    $('drag-mode').disabled=true;dragPending=null;$('cartesian-rebase').disabled=true;$('pose-status').textContent='连接中断 · 位姿已过期';
    document.querySelectorAll('.pose-cell strong').forEach(x=>x.classList.add('stale-pose'));
    for(const side of ['left','right']){$('pose-state-'+side).textContent='已过期';$('pose-state-'+side).className='pill neutral';}
    document.querySelectorAll('.pd-inputs input,#cartesian-arms input').forEach(input=>{input.disabled=true;});
    for(let i=0;i<14;i++)$("joint-"+i).disabled=true;
    for(const side of ['left','right']){clearTimeout(gripperTimers[side]);$("gripper-live-"+side).checked=false;$("gripper-"+side).disabled=$('gripper-effort-'+side).disabled=$("gripper-send-"+side).disabled=$("gripper-live-"+side).disabled=true;}
    $("notice").textContent="网页与后端连接中断。最后显示的数值已过期；控制后端会在心跳超时后请求位置保持。";
    document.querySelectorAll(".force-cell").forEach(x=>x.classList.add("invalid"));
  } finally { setTimeout(poll,100); }
}
$("sync").addEventListener("click",()=>{syncDraft();toast("目标预览已对齐实际角度");});
$("arm").addEventListener("click",async()=>{try{await command("start_control");toast("已提交启动请求，等待后端检查结果");}catch(e){toast(e.message);}});
$("hold").addEventListener("click",()=>command("hold").catch(e=>toast(e.message)));
$('control-mode').addEventListener('change',async event=>{
  clearTimeout(targetTimer);clearTimeout(cartesianTimer);modePending=event.target.value;
  try{await command('control_mode',{control_mode:modePending});}catch(error){modePending=null;toast(error.message);}
});
$('apply-pd').addEventListener('click',async()=>{
  try{const kp=Array.from({length:14},(_,i)=>numberValue('kp-'+i)),kd=Array.from({length:14},(_,i)=>numberValue('kd-'+i));await command('joint_gains',{kp,kd});pdDirty=false;toast('已提交 PD，等待后端应用');}catch(error){toast(error.message);}
});
$('apply-cartesian-gains').addEventListener('click',async()=>{
  try{const stiffness=['left','right'].map(s=>Array.from({length:6},(_,j)=>numberValue('cartesian-k-'+s+'-'+j))),damping=['left','right'].map(s=>Array.from({length:6},(_,j)=>numberValue('cartesian-d-'+s+'-'+j)));await command('cartesian_gains',{stiffness,damping});cartesianGainsDirty=false;toast('已提交末端 K / D，等待后端应用');}catch(error){toast(error.message);}
});
$('cartesian-rebase').addEventListener('click',async()=>{
  clearTimeout(cartesianTimer);
  try{await command('cartesian_rebase');toast('已请求以当前实际位姿重新建立参考，增量将归零');}catch(error){toast(error.message);}
});
$('drag-mode').addEventListener('change',async event=>{
  clearTimeout(cartesianTimer);dragPending=event.target.checked;dragPendingUntil=Date.now()+2000;
  event.target.disabled=true;
  try{await command('drag_mode',{enabled:dragPending});}
  catch(error){dragPending=null;event.target.checked=!!state?.cartesian?.live?.drag_enabled;toast(error.message);}
});
setInterval(()=>{if(ownsControl())command("heartbeat").catch(()=>{});},150);
document.addEventListener("visibilitychange",()=>{if(document.hidden){clearTimeout(targetTimer);clearTimeout(cartesianTimer);if(ownsControl())command("hold").catch(()=>{});}});
document.addEventListener('visibilitychange',()=>{if(document.hidden)for(const side of ['left','right']){clearTimeout(gripperTimers[side]);$("gripper-live-"+side).checked=false;}});
window.addEventListener("pagehide",()=>{if(ownsControl())command("hold").catch(()=>{});});
build(); poll();
