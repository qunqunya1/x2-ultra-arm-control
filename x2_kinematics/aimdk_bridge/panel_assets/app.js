"use strict";
const $ = id => document.getElementById(id);
const names = ["肩俯仰", "肩侧摆", "肩旋转", "肘关节", "腕偏航", "腕俯仰", "腕旋转"];
const components = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"];
const clientId = crypto.randomUUID();
let token, state, draft = null, sequence = 0, targetTimer, toastTimer, lastFetch = 0, wasOwner = false;
const histories = {}, forceStamps = {};
const gripperDraft={left:null,right:null},gripperTimers={};let gripperSequence=0;
const deg = x => x * 180 / Math.PI;
const rad = x => x * Math.PI / 180;
const finite = x => typeof x === "number" && Number.isFinite(x);
const fmt = (x, digits=1) => finite(x) ? x.toFixed(digits) : "—";
const reasons = {poor_observability:"姿态可观测性不足",virtual_flange_calibration_unconfirmed:"法兰坐标未标定",fixed_upright_pelvis_assumption_unconfirmed:"骨盆支撑条件待确认",sdk_effort_feedback_semantics_unconfirmed:"力矩反馈含义待确认",onboard_sdk_release_not_verified:"SDK 待核验",outside_urdf_planning_limits:"部分反馈超出模型规划限位",base_imu_not_ready:"胯部 IMU 尚未有效或稳定"};
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
      card.append(row);
      row.querySelector("input").addEventListener("input", e=>{
        if(!draft)return; draft[i]=rad(Number(e.target.value)); $("target-"+i).textContent=fmt(deg(draft[i]))+"°";
        clearTimeout(targetTimer);
        if(ownsControl()) targetTimer=setTimeout(()=>command("targets",{q_rad:[...draft],sequence:++sequence}).catch(e=>toast(e.message)),80);
      });
    });
    $("arms").append(card);
    const force=document.createElement("article"); force.className="force-card";
    force.innerHTML=`<div class="card-head"><h3>${label}末端</h3><span class="pill neutral" id="force-status-${side}">等待数据</span></div><div class="force-grid">${components.map((c,j)=>`<div class="force-cell invalid" id="cell-${side}-${j}"><div class="force-label"><span>${c}</span><span>${j<3?"N":"N·m"}</span></div><div class="force-value" id="value-${side}-${j}">—</div><svg class="spark" viewBox="0 0 100 23" preserveAspectRatio="none" aria-hidden="true"><polyline id="line-${side}-${j}" points=""></polyline></svg></div>`).join("")}</div><div class="force-meta" id="force-meta-${side}">坐标：${label}虚拟法兰局部坐标</div>`;
    $("forces").append(force);
  }
  $("endpoint").textContent=location.host;
  for(const [side,label] of [["left","左夹爪"],["right","右夹爪"]]){
    const card=document.createElement('article');card.className='gripper-card';
    card.innerHTML=`<div class="gripper-heading"><h3>${label}</h3><span class="joint-actual" id="gripper-actual-${side}">实际 —</span></div><div class="target-line"><span>目标开度</span><span class="target-value" id="gripper-value-${side}">—</span></div><input type="range" id="gripper-${side}" aria-label="${label}目标开度" min="0" max="100" step="1" value="0" disabled><div class="joint-range"><span>0% 全闭</span><span>100% 全开</span></div><div class="gripper-actions"><label><input type="checkbox" id="gripper-live-${side}" disabled>拖动实时下发</label><button id="gripper-send-${side}" disabled>发送${label}目标</button></div><div class="gripper-meta" id="gripper-meta-${side}">等待夹爪反馈</div>`;
    $("grippers").append(card);
    $("gripper-"+side).addEventListener('input',event=>{
      gripperDraft[side]=Number(event.target.value)/100;$("gripper-value-"+side).textContent=fmt(gripperDraft[side]*100,0)+'%';
      clearTimeout(gripperTimers[side]);
      if($("gripper-live-"+side).checked)gripperTimers[side]=setTimeout(()=>sendGripper(side),100);
    });
    $("gripper-send-"+side).addEventListener('click',()=>sendGripper(side));
    $("gripper-live-"+side).addEventListener('change',()=>{clearTimeout(gripperTimers[side]);});
  }
}
async function sendGripper(side){
  const opening=gripperDraft[side];if(!finite(opening))return;
  try{await command('gripper_target',{side,opening,sequence:++gripperSequence});toast('已提交'+(side==='left'?'左':'右')+'夹爪目标 '+fmt(opening*100,0)+'%');}
  catch(error){toast(error.message);}
}
function ownsControl() { return !!state && state.control_owner===clientId && state.control_state==="armed" && Date.now()-lastFetch<500; }
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
  const live=state.connection==="live", owner=ownsControl(), demo=state.backend==="demo";
  $("mode").textContent=demo?"数值仿真":state.control_state==="armed"?"实机 · 控制中":state.control_state==="fault"?"实机 · 控制故障":state.control_state==="holding"?"实机 · 保持中":"实机 · 观察中";
  $("mode").className="pill "+(demo?"warn":"");
  $("connection").textContent=live?"● 反馈已连接":"● 等待 / 反馈过期";
  $("connection").className="pill "+(live?"":"neutral");
  const labels={observe:"只读观察",idle:"尚未接管",armed:"滑块控制中",holding:"位置保持",fault:"故障保持"};
  $("control-state").textContent=state.control_state==="fault"?(state.control_commands_generated?"故障 · 已请求保持":"故障 · 未发送控制指令"):(labels[state.control_state]||"等待反馈");
  $("limit-note").textContent=`速度上限 ${fmt(deg(state.velocity_limit_rad_s))}°/s · ${owner?"拖动后平滑跟踪":"滑块仅预览目标"}`;
  $("arm").textContent=owner?"控制已启动":demo?"启动仿真控制":"启动实机控制";
  $("arm").disabled=!live || !state.control_available || !!state.control_owner;
  $("hold").disabled=!owner;
  $("sync").disabled=!live;
  const checks=state.startup_checks||[], failed=checks.filter(c=>!c.ok);
  $("readiness-title").textContent=failed.length?`启动条件：${failed.length} 项待完成`:(owner?"控制已启动 · 检查通过":"启动条件已满足 · 点击按钮接管");
  if(state.fault)$("readiness-title").textContent=failed.length?`重新接管：${failed.length} 项待恢复`:"可点击启动从当前角度重新接管";
  $("startup-checks").replaceChildren(...checks.map(c=>{
    const li=document.createElement("li");li.className=c.ok?"ready":"pending";
    let detail="";
    if(c.key==="feedback"&&finite(state.feedback_source_age_ms))detail=` · 消息年龄 ${fmt(state.feedback_source_age_ms,0)} ms`;
    if(c.key==="base_imu"&&state.base_imu){const b=state.base_imu;detail=` · 倾角 ${fmt(b.orientation_tilt_deg)}° · 角速度 ${fmt(b.angular_speed_rad_s,3)} rad/s`;if(!b.ready)detail+=" · "+(b.reason||"等待稳定");}
    if((c.key==="waist"||c.key==="waist_stationary")&&state.waist_feedback){const w=state.waist_feedback;detail=c.key==="waist"?` · 消息年龄 ${fmt(w.source_age_ms,0)} ms`:` · 稳定 ${fmt(w.stable_s)} s`;if(!c.ok&&w.reason)detail+=" · "+translate(w.reason);}
    if(c.key==="limits"&&state.joint_limit_violations?.length)detail=" · "+(c.ok?"允许向范围内回收：":"")+state.joint_limit_violations.map(v=>{const i=state.joint_names.indexOf(v.name);return `${i<7?"左":"右"}${names[i%7]} ${fmt(deg(v.position_rad))}°，范围 ${fmt(deg(v.lower_rad))}～${fmt(deg(v.upper_rad))}°`;}).join("；");
    li.textContent=(c.ok?"✓ ":"○ ")+c.label+detail;return li;
  }));
  if(state.error && state.error.startsWith("尚不能启动"))$("readiness").open=true;
  if(!draft || (wasOwner && !owner) || (!wasOwner && owner))syncDraft();
  wasOwner=owner;
  if(state.q_rad) state.q_rad.forEach((q,i)=>{
    const input=$("joint-"+i);
    input.min=deg((state.slider_lower_rad||state.lower_rad)[i]); input.max=deg((state.slider_upper_rad||state.upper_rad)[i]); input.disabled=!live || (!!state.control_owner&&!owner);
    $("actual-"+i).textContent="实际 "+fmt(deg(q))+"°";
    $("min-"+i).textContent=fmt(deg(state.lower_rad[i]),0)+"°";
    $("max-"+i).textContent=fmt(deg(state.upper_rad[i]),0)+"°";
  });
  const hints=[...(state.quality?.reasons||[]),...(state.quality?.warnings||[])].map(translate);
  let note=demo?"Pinocchio 数值仿真 · 滑块只作用于虚拟动力学模型，未连接真实电机。":state.mode==="observe"?"强制只读连接 · 滑块仅预览，不允许启动控制。":owner?"实机控制已启动 · 滑块目标经限速后跟踪。":state.control_state==="fault"&&!state.motor_commands_sent?"启动未完成，故障已锁存，尚未下发电机指令。":state.control_state==="idle"&&state.hardware_command_publisher_created?"控制接口已准备，尚未接管；条件恢复后可再次点击启动。":state.hardware_command_publisher_created?"当前处于位置保持；重新启动会从最新反馈角度接管。":"当前仅观察反馈。点击“启动实机控制”后检查条件，通过后从当前角度接管。";
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
    $("gripper-send-"+side).disabled=$("gripper-live-"+side).disabled=!available;
    if(!available){clearTimeout(gripperTimers[side]);$("gripper-live-"+side).checked=false;}
    const status=['已到位','移动中','堵转','物体掉落'][hand?.state]||'等待反馈';
    $("gripper-meta-"+side).textContent=ready?`${status} · 速度 ${fmt((state.gripper_drive?.velocity??.2)*100,0)}% · 夹持力矩 ${fmt((state.gripper_drive?.effort??.2)*100,0)}%`:(hand?.reason||'等待夹爪反馈');
  }
  $("gripper-note").textContent=state.gripper_error?('夹爪：'+state.gripper_error):'左右独立控制。默认只预览，点击发送目标或开启拖动实时下发；无需启动手臂控制。';
}
async function poll() {
  try {
    if(!token)token=(await (await fetch("/api/session",{signal:AbortSignal.timeout(1500)})).json()).token;
    const response=await fetch("/api/state",{signal:AbortSignal.timeout(1500)}); if(!response.ok)throw new Error("反馈读取失败");
    const next=await response.json();
    if(state && next.instance_id!==state.instance_id){token=null;draft=null;sequence=0;wasOwner=false;gripperSequence=0;for(const side of ['left','right']){clearTimeout(gripperTimers[side]);gripperDraft[side]=null;$("gripper-live-"+side).checked=false;}}
    state=next; lastFetch=Date.now(); render();
  } catch(e) {
    $("connection").textContent="● 连接已断开"; $("connection").className="pill warn";
    $("arm").disabled=$("hold").disabled=true;
    for(let i=0;i<14;i++)$("joint-"+i).disabled=true;
    for(const side of ['left','right']){clearTimeout(gripperTimers[side]);$("gripper-live-"+side).checked=false;$("gripper-"+side).disabled=$("gripper-send-"+side).disabled=$("gripper-live-"+side).disabled=true;}
    $("notice").textContent="网页与后端连接中断。最后显示的数值已过期；控制后端会在心跳超时后请求位置保持。";
    document.querySelectorAll(".force-cell").forEach(x=>x.classList.add("invalid"));
  } finally { setTimeout(poll,100); }
}
$("sync").addEventListener("click",()=>{syncDraft();toast("目标预览已对齐实际角度");});
$("arm").addEventListener("click",async()=>{try{await command("start_control");toast("已提交启动请求，等待后端检查结果");}catch(e){toast(e.message);}});
$("hold").addEventListener("click",()=>command("hold").catch(e=>toast(e.message)));
setInterval(()=>{if(ownsControl())command("heartbeat").catch(()=>{});},150);
document.addEventListener("visibilitychange",()=>{if(document.hidden&&ownsControl())command("hold").catch(()=>{});});
document.addEventListener('visibilitychange',()=>{if(document.hidden)for(const side of ['left','right']){clearTimeout(gripperTimers[side]);$("gripper-live-"+side).checked=false;}});
window.addEventListener("pagehide",()=>{if(ownsControl())command("hold").catch(()=>{});});
build(); poll();
