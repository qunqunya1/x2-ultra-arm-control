"""Local joint panel. Web startup reads feedback; control starts on a UI request."""
import argparse
from collections import Counter, deque
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import secrets
import signal
import threading
import time
from types import SimpleNamespace as Obj

import numpy as np

from .dynamics import LiveWaistArmDynamics
from .joint_panel_control import JointPanelControl, admissible_start, recovery_margins
from .serialization import diagnostic_json
from .protocol import ARM_NAMES, WAIST_NAMES, decode_arm, build_command, verify_sdk_types, ordered_joints, stamp_ns
from .runtime import ArmRuntime
from .base_monitor import StaticBaseImu
from .gripper import GripperFeedback,build_gripper_command,normalized

ROOT = Path(__file__).resolve().parents[2]
ASSETS = Path(__file__).with_name("panel_assets")


class PanelStore:
    """HTTP threads only exchange copies and requests; no robot SDK calls."""
    def __init__(self, instance_id, control_available):
        self.lock = threading.Lock()
        self.token = secrets.token_urlsafe(32)
        self.instance_id = instance_id
        self.control_available = control_available
        self.snapshot = dict(connection="waiting", instance_id=instance_id)
        self.pending = deque()
        self.owner = None
        self.contact_ns = 0
        self.last_seq = {}
        self.last_gripper_seq={}
        self.updated_ns = time.monotonic_ns()
        self.signals = {}

    def note_signal(self, key, value):
        with self.lock:
            self.signals[key]=(time.monotonic_ns(),dict(value))

    def read(self):
        with self.lock:
            value = dict(self.snapshot)
            elapsed_ms = max(0., (time.monotonic_ns()-self.updated_ns)*1e-6)
            now=time.monotonic_ns()
            signals={key:(max(0.,(now-stamp)*1e-6),dict(payload)) for key,(stamp,payload) in self.signals.items()}
        value["ui_snapshot_age_ms"]=elapsed_ms
        for key in ("feedback_age_ms", "wrench_age_ms"):
            if value.get(key) is not None:
                value[key] += elapsed_ms
        if value.get("feedback_age_ms") is not None:
            if value["feedback_age_ms"] > 100:
                value["connection"] = "stale"
            if value["feedback_age_ms"] > value.get("feedback_timeout_ms", 50):
                value["can_arm"] = False
                value["startup_checks"]=[dict(c,ok=False) if c["key"]=="feedback" else dict(c) for c in value.get("startup_checks",[])]
        if value.get("base_imu") is not None:
            base=dict(value["base_imu"])
            base_lag=elapsed_ms
            if 'base_imu' in signals:
                base_lag,base=signals['base_imu']
            if base.get("age_ms") is not None:
                base["age_ms"]+=base_lag
                if base.get('source_age_ms') is not None:base['source_age_ms']+=base_lag
                if base["age_ms"] > value.get("base_imu_timeout_ms",100) or (base.get('source_age_ms') is not None and not -10 <= base['source_age_ms'] <= value.get('base_imu_timeout_ms',100)):
                    base.update(ready=False,reason="胯部 IMU 反馈过期")
                if not base.get('ready'):
                    value["can_arm"]=False
                    value["startup_checks"]=[dict(c,ok=False) if c["key"]=="base_imu" else dict(c) for c in value.get("startup_checks",[])]
                    if "quality" in value:
                        value["quality"]=dict(value["quality"],physical_assumptions_confirmed=False,
                            reasons=list(value["quality"].get("reasons",[]))+["base_imu_not_ready"])
            value["base_imu"]=base
            if 'base_imu' in signals:
                value['startup_checks']=[dict(c,ok=bool(base.get('ready'))) if c['key']=='base_imu' else dict(c) for c in value.get('startup_checks',[])]
        if value.get("waist_feedback") is not None:
            waist=dict(value["waist_feedback"])
            for key in ("age_ms","source_age_ms"):
                if waist.get(key) is not None:waist[key]+=elapsed_ms
            waist["fresh"]=bool(waist.get("valid") and waist.get("age_ms") is not None and
                0 <= waist["age_ms"] <= value.get("waist_timeout_ms",100) and
                waist.get("source_age_ms") is not None and -.01*1000 <= waist["source_age_ms"] <= value.get("waist_source_timeout_ms",50))
            if not waist["fresh"]:
                waist.update(stationary=False,reason="waist_feedback_missing_or_stale")
                value["can_arm"]=False
                value["startup_checks"]=[dict(c,ok=False) if c["key"] in ("waist","waist_stationary") else dict(c) for c in value.get("startup_checks",[])]
            value["waist_feedback"]=waist
        # Stream heartbeats are updated by accepted telemetry callbacks,
        # independently of the lower-rate display snapshots. A frozen
        # producer expires both; a slow browser alone does not fake a gap.
        checks=[dict(c) for c in value.get("startup_checks",[])]
        if "arm" in signals:
            lag,arm=signals["arm"]
            received=arm.get("age_ms",0.)+lag
            source=None if arm.get("source_age_ms") is None else arm["source_age_ms"]+lag
            ready=bool(arm.get("valid") and source is not None and -.01*1000 <= source <= value.get("feedback_timeout_ms",50)
                       and received <= value.get("feedback_timeout_ms",50))
            value["feedback_stream_age_ms"]=received
            value["feedback_source_age_ms"]=source
            value["connection"]="live" if ready else "stale"
            for c in checks:
                if c["key"]=="feedback":c["ok"]=ready
        if "waist" in signals:
            lag,waist=signals["waist"]
            for key in ("age_ms","source_age_ms"):
                if waist.get(key) is not None:waist[key]+=lag
            waist["fresh"]=bool(waist.get("valid") and waist.get("age_ms") is not None and
                0 <= waist["age_ms"] <= value.get("waist_timeout_ms",100) and waist.get("source_age_ms") is not None and
                -.01*1000 <= waist["source_age_ms"] <= value.get("waist_source_timeout_ms",50))
            waist["stationary"]=bool(waist["fresh"] and waist.get("stationary"))
            if not waist["fresh"]:waist["reason"]="waist_feedback_missing_or_stale"
            value["waist_feedback"]=waist
            for c in checks:
                if c["key"]=="waist":c["ok"]=bool(waist["valid"] and waist["fresh"])
                if c["key"]=="waist_stationary":c["ok"]=waist["stationary"]
        value["startup_checks"]=checks
        if checks:
            ready=all(c["ok"] for c in checks)
            value["can_arm"]=ready
        if value.get('grippers') is not None:
            hands=dict(value['grippers']);hands['sides']={s:dict(v) for s,v in hands['sides'].items()}
            for key in ('age_ms','source_age_ms'):
                if hands.get(key) is not None:hands[key]+=elapsed_ms
            fresh=bool(hands.get('age_ms') is not None and 0<=hands['age_ms']<=hands['timeout_ms'] and
                       hands.get('source_age_ms') is not None and -10<=hands['source_age_ms']<=hands['timeout_ms'])
            for hand in hands['sides'].values():
                hand['fresh']=fresh;hand['ready']=bool(fresh and hand['valid'])
                if hand['valid'] and not fresh:hand['reason']='夹爪反馈已过期'
            value['grippers']=hands
        return value

    def write(self, value):
        with self.lock:
            self.snapshot = value
            self.updated_ns = time.monotonic_ns()

    def submit(self, message, now_ns):
        if not self.control_available:
            raise ValueError("当前为只读模式，滑块目标不会下发")
        op, client = message.get("op"), message.get("client_id")
        if op == "start_control":
            op = "arm"
            message = dict(message, op=op)
        if op not in ("arm", "targets", "heartbeat", "hold","gripper_target") or not isinstance(client, str) or not 1 <= len(client) <= 80:
            raise ValueError("Invalid command")
        with self.lock:
            if self.owner is not None and self.owner != client:
                raise ValueError("另一个网页持有控制权")
            if op not in ("arm","gripper_target") and self.owner != client:
                raise ValueError("请先从当前角度接管")
            if op=='gripper_target':
                side=message.get('side');normalized(message.get('opening'))
                seq=message.get('sequence')
                if side not in ('left','right'):raise ValueError('未知夹爪')
                if isinstance(seq,bool) or not isinstance(seq,int) or seq<=self.last_gripper_seq.get(client,-1):raise ValueError('过期的夹爪目标')
                self.last_gripper_seq[client]=seq
                self.pending=deque(x for x in self.pending if not(x['op']=='gripper_target' and x['side']==side))
            if op == "targets":
                q, seq = message.get("q_rad"), message.get("sequence")
                if (not isinstance(q, list) or len(q) != 14 or
                    any(isinstance(x, bool) or not isinstance(x, (int, float)) or not np.isfinite(x) for x in q)):
                    raise ValueError("Target requires fourteen finite radians")
                if not isinstance(seq, int) or isinstance(seq, bool) or seq <= self.last_seq.get(client, -1):
                    raise ValueError("过期的滑块指令")
                self.last_seq[client] = seq
                # Replace pending target updates; never accumulate a motion queue.
                self.pending = deque(x for x in self.pending if x["op"] != "targets")
            if op == "hold":
                self.pending.clear()
            if op != "heartbeat":
                if len(self.pending) >= 8:
                    raise ValueError("Command queue full")
                self.pending.append(dict(message))
            if op!='gripper_target':self.contact_ns = int(now_ns)

    def drain(self):
        with self.lock:
            result = list(self.pending)
            self.pending.clear()
            return result

    def set_owner(self, owner, now_ns):
        with self.lock:
            self.owner, self.contact_ns = owner, int(now_ns)
            self.last_seq.clear()

    def lease(self):
        with self.lock:
            return self.owner, self.contact_ns


def make_server(store, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, status, body, content_type="application/json; charset=utf-8"):
            if not isinstance(body, bytes):
                body = diagnostic_json(body).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("X-Frame-Options", "DENY")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def allowed_host(self):
            return self.headers.get("Host") in (f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}")

        def do_GET(self):
            if not self.allowed_host():
                return self.reply(403, dict(error="Use localhost via SSH forwarding"))
            if self.path == "/api/health":
                return self.reply(200, dict(instance_id=store.instance_id, ready=True))
            if self.path == "/api/session":
                return self.reply(200, dict(token=store.token))
            if self.path == "/api/state":
                return self.reply(200, store.read())
            assets = {"/": ("index.html", "text/html; charset=utf-8"),
                      "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                      "/styles.css": ("styles.css", "text/css; charset=utf-8")}
            if self.path not in assets:
                return self.reply(404, dict(error="Not found"))
            name, kind = assets[self.path]
            return self.reply(200, (ASSETS/name).read_bytes(), kind)

        def do_POST(self):
            if (not self.allowed_host() or self.headers.get("Origin") != "http://"+self.headers.get("Host", "") or
                    not secrets.compare_digest(self.headers.get("X-Panel-Token", ""), store.token)):
                return self.reply(403, dict(error="Invalid local session"))
            if self.path != "/api/command":
                return self.reply(404, dict(error="Not found"))
            try:
                if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                    raise ValueError("JSON required")
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 8192:
                    raise ValueError("Invalid request size")
                request = json.loads(self.rfile.read(length))
                if not isinstance(request, dict):
                    raise ValueError("Object required")
                store.submit(request, time.monotonic_ns())
                self.reply(202, dict(accepted=True))
            except (ValueError, TypeError) as error:
                self.reply(409, dict(error=str(error)))

    # Only loopback is supported. Robot container uses host networking.
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.daemon_threads = True
    return server


class PanelEngine:
    def __init__(self, cfg, flange, store, *, backend="robot", mode="observe"):
        self.cfg, self.store, self.backend, self.mode = dict(cfg), store, backend, mode
        self.model = LiveWaistArmDynamics(base_gravity=cfg["base_gravity_m_s2"])
        self.flange_config=flange
        self.wrench_worker=None
        self.wrench_epoch=0
        self.timing={name:deque(maxlen=512) for name in ('arm_callback_ms','arm_gap_ms','tick_gap_ms','tick_work_ms','source_age_ms')}
        self.last_arm_callback_ns=self.last_tick_ns=None
        self.timing_cache={};self.last_timing_snapshot_ns=0
        observer_cfg = dict(cfg, max_drift_from_start_rad=2*np.pi,
                            window_s=cfg["panel_wrench_window_s"], max_sample_gap_s=cfg["panel_wrench_max_gap_s"])
        self.observer = ArmRuntime(self.model, observer_cfg, flange, mode="observe")
        if mode not in ("observe", "web", "control"):
            raise ValueError("Invalid panel mode")
        # Both web and legacy control modes start with subscriptions only.
        self.control_enabled = mode != "observe"
        self.static_startup_checks=None
        self.control = None
        self.base_monitor = None
        if backend == "robot" and cfg.get("panel_base_imu_required"):
            if cfg["panel_base_imu_topic"] != "/aima/hal/imu/torso/state":
                raise ValueError("Use the pelvis IMU topic; chest IMU axes require a different model transform")
            rotation=cfg.get("panel_imu_to_base_rotation",self.model.nominal_pelvis_imu_rotation())
            self.base_monitor=StaticBaseImu(cfg,rotation)
        self.activate_transport = None
        self.refresh_clocks = None
        self.hardware_publisher_created = False
        self.plans_sent = 0
        self.action_error = None
        self.sample = self.gravity = self.wrench = None
        self.error = self.arm_blocked = None
        self.last_source_ns = None
        self.last_snapshot_ns = 0
        self.failures = Counter()
        self.motor_commands_sent = 0
        self.sequence = 0
        self.grippers=GripperFeedback()
        self.gripper_transport=None
        self.gripper_error=None
        self.gripper_last_targets={}
        self.gripper_commands_sent=0
        self.gripper_hardware_publisher_created=False
        self.gripper_requests_generated=0
        self.waist_observation = None
        self.waist_reference = None
        self.waist_stable_since = None
        self.waist_settle_s = float(cfg.get("panel_waist_stable_duration_s", .5))
        if not np.isfinite(self.waist_settle_s) or self.waist_settle_s < 0:
            raise ValueError("Invalid waist settling duration")

    def invalid(self, reason):
        self.error = str(reason)
        self.store.note_signal("arm",dict(valid=False,age_ms=0.,source_age_ms=None))
        self.failures[str(reason)] += 1
        self.wrench_epoch+=1
        self.wrench=None
        if self.control:
            self.control.fail(reason)
            if self.control.fault:
                self.store.set_owner(None, time.monotonic_ns())

    def waist_feedback(self, msg, now_ns, ros_now_ns):
        began=time.monotonic_ns()
        observed = dict(received_ns=int(now_ns), source_ns=None, valid=False,
                        model_valid=False, q_rad=None, dq_rad_s=None, reason=None)
        self.waist_observation = observed
        try:
            source=stamp_ns(msg)
            q,dq,_,_=ordered_joints(msg,WAIST_NAMES)
            observed.update(source_ns=source,q_rad=q.tolist(),dq_rad_s=dq.tolist())
            self.observer._age(source,ros_now_ns,self.cfg["source_max_age_s"])
            observed["valid"]=True
            previous=self.observer.waist
            if self.control is not None and self.control.state=='armed' and previous is not None:
                interval=max(0.,(source-previous[2])*1e-9)
                allowance=self.cfg['max_waist_change_rad']+self.cfg['max_waist_velocity_rad_s']*interval
                if np.max(np.abs(q-previous[0])) > allowance:
                    raise ValueError('waist_moving_arm_only_model_invalid')
            # The web model already uses measured waist angles and IMU
            # gravity. Permit quasi-static pose updates; the legacy gravity
            # entry keeps its separate fixed-waist policy.
            self.observer.waist_start=None
            self.observer.update_waist(msg, now_ns, ros_now_ns)
            changed=(self.waist_reference is None or self.waist_stable_since is None or
                np.max(np.abs(q-self.waist_reference)) > self.cfg["max_waist_change_rad"])
            if changed:
                self.observer.estimator.reset()
                self.wrench_epoch+=1
                self.wrench=None
                self.waist_reference=q.copy()
                self.waist_stable_since=int(now_ns)
            observed["model_valid"]=True
        except ValueError as error:
            observed["reason"]=str(error)
            if str(error) not in ("waist_moving_arm_only_model_invalid","waist_posture_changed_restart_required"):
                observed["valid"]=False
            self.waist_stable_since=None
            self.invalid(error)
        processing=time.monotonic_ns()-began
        self.store.note_signal("waist",self.waist_status(now_ns+processing,ros_now_ns+processing))

    def waist_status(self, now_ns, ros_now_ns):
        observed=self.waist_observation
        if observed is None:
            return dict(valid=False,fresh=False,stationary=False,age_ms=None,source_age_ms=None,
                        reason="waiting_for_waist_feedback",q_rad=None,dq_rad_s=None,stable_s=0.)
        age=(now_ns-observed["received_ns"])*1e-9
        source_age=None if observed["source_ns"] is None else (ros_now_ns-observed["source_ns"])*1e-9
        fresh=bool(0 <= age <= self.cfg["waist_timeout_s"] and source_age is not None and
                   -.01 <= source_age <= self.cfg["source_max_age_s"])
        stable=0. if self.waist_stable_since is None else max(0.,(now_ns-self.waist_stable_since)*1e-9)
        stationary=bool(observed["valid"] and fresh and observed["model_valid"] and stable >= self.waist_settle_s)
        reason=observed["reason"]
        if reason is None and not fresh:reason="waist_feedback_missing_or_stale"
        if reason is None and not stationary:reason="waist_settling"
        return dict(valid=observed["valid"],fresh=fresh,stationary=stationary,age_ms=age*1000,
                    source_age_ms=None if source_age is None else source_age*1000,
                    reason=reason,q_rad=observed["q_rad"],dq_rad_s=observed["dq_rad_s"],stable_s=stable)

    def base_feedback(self, msg, now_ns, ros_now_ns):
        began=time.monotonic_ns()
        if self.base_monitor is not None:
            self.base_monitor.update(msg,now_ns,ros_now_ns)
            if self.base_monitor.good:
                self.model.set_base_gravity(self.base_monitor.gravity)
            else:
                self.observer.estimator.reset()
                self.wrench_epoch+=1
                self.wrench=None
            processing=time.monotonic_ns()-began
            self.store.note_signal('base_imu',self.base_monitor.status(now_ns+processing,ros_now_ns+processing))

    def gripper_feedback(self,msg,now_ns,ros_now_ns):
        self.grippers.update(msg,now_ns,ros_now_ns)

    def arm_feedback(self, msg, now_ns, ros_now_ns):
        began=time.monotonic_ns()
        if self.last_arm_callback_ns is not None:self.timing['arm_gap_ms'].append((began-self.last_arm_callback_ns)*1e-6)
        self.last_arm_callback_ns=began
        if self.observer.waist is None:
            return
        if self.waist_observation is not None and not self.waist_observation["model_valid"]:
            return  # Do not clear the waist error or estimate with an old pose.
        try:
            sample = decode_arm(msg, now_ns)
            if self.last_source_ns is not None and sample.source_ns <= self.last_source_ns:
                raise ValueError("nonmonotonic_arm_source_timestamp")
            try:
                gravity, result = self.observer.accept_sample(sample, now_ns, ros_now_ns,estimate=self.wrench_worker is None)
            except ValueError as error:
                if not str(error).startswith("Stale or nonmonotonic telemetry:"):
                    raise
                # The observer already checked raw feedback before detecting
                # a window gap. Fresh position control does not require a
                # usable force estimate; force remains explicitly invalid.
                self.failures["wrench_window_gap_reset"] += 1
                gravity, result = self.model.gravity(sample.q), None
                self.wrench = None
            self.sample, self.gravity, self.last_source_ns = sample, gravity, sample.source_ns
            processing=(time.monotonic_ns()-began)*1e-6
            self.timing['arm_callback_ms'].append(processing)
            self.timing['source_age_ms'].append((ros_now_ns-sample.source_ns)*1e-6)
            self.store.note_signal("arm",dict(valid=True,age_ms=processing,source_age_ms=(ros_now_ns-sample.source_ns)*1e-6+processing))
            self.error = None
            if result is not None:
                self.wrench = result.as_dict()
            if self.wrench_worker is not None:
                self.wrench_worker.submit(sample,self.model.waist,self.model.model.gravity.linear,self.wrench_epoch)
            if self.control:
                self.control.feedback(sample, gravity)
        except ValueError as error:
            self.invalid(error)

    def step(self, now_ns, ros_now_ns, send):
        if self.wrench_worker is not None:
            latest=self.wrench_worker.poll(self.wrench_epoch)
            if latest is not None:
                self.wrench=latest['result']
                if latest.get('error'):self.failures['async_wrench_reset']+=1
        for message in self.store.drain():
            try:
                if not self.control_enabled:
                    raise ValueError("Observe mode")
                op = message["op"]
                if op=='gripper_target':
                    opening=self.grippers.require(message['side'],message['opening'],now_ns,ros_now_ns)
                    if self.gripper_transport is None:raise ValueError('夹爪控制接口尚未就绪')
                    self.gripper_transport(message['side'],opening,ros_now_ns)
                    self.gripper_last_targets[message['side']]=opening
                    self.gripper_requests_generated+=1
                    self.gripper_error=None
                elif op == "arm":
                    owner, _ = self.store.lease()
                    if owner is not None and owner != message["client_id"]:
                        raise ValueError("另一个网页持有控制权")
                    blockers = [check["label"] for check in self.startup_checks(now_ns, ros_now_ns) if not check["ok"]]
                    if blockers:
                        raise ValueError("尚不能启动："+"；".join(blockers))
                    self.observer._waist_current(now_ns, ros_now_ns)
                    candidate = JointPanelControl(self.model, self.cfg)
                    candidate.feedback(self.sample, self.gravity)
                    candidate.arm(now_ns, ros_now_ns)
                    # Validate the full candidate before touching SDK transport.
                    # ROS publisher creation happens only on the ROS thread.
                    if self.activate_transport is not None:
                        self.activate_transport()
                    elif self.backend == "robot":
                        raise ValueError("实机控制接口尚未连接")
                    if self.refresh_clocks is not None:
                        commit_now,commit_ros=self.refresh_clocks()
                        # DDS publisher creation may consume the feedback
                        # budget on a single-thread executor. Keep observing
                        # rather than committing a controller with old data.
                        blockers=[c["label"] for c in self.startup_checks(commit_now,commit_ros) if not c["ok"]]
                        if blockers:
                            raise ValueError("控制接口已准备；请待反馈恢复后再次点击启动："+"；".join(blockers))
                        candidate.feedback(self.sample,self.gravity)
                        candidate.arm(commit_now,commit_ros)
                        now_ns,ros_now_ns=commit_now,commit_ros
                    self.control = candidate
                    self.observer.waist_start=self.observer.waist[0].copy()
                    self.waist_reference=self.observer.waist[0].copy()
                    self.store.set_owner(message["client_id"], now_ns)
                elif op == "targets":
                    if self.control is None:
                        raise ValueError("请先启动控制")
                    self.control.target(message["q_rad"])
                elif op == "hold":
                    if self.control is not None:
                        self.control.hold()
                    self.store.set_owner(None, now_ns)
                if op!='gripper_target':self.action_error = None
            except (ValueError, RuntimeError) as error:
                if message['op']=='gripper_target':self.gripper_error=str(error)
                else:self.action_error = str(error)
        if self.refresh_clocks is not None:
            now_ns, ros_now_ns = self.refresh_clocks()
        if self.control:
            owner, contact = self.store.lease()
            if self.control.state == "armed" and (owner is None or (now_ns-contact)*1e-9 > self.control.timeout):
                self.control.hold()
                self.store.set_owner(None, now_ns)
                self.error = "网页心跳超时，已请求保持最后反馈姿态"
            if self.control.active:
                try:
                    if self.base_monitor is not None:
                        base=self.base_monitor.status(now_ns,ros_now_ns)
                        if not base["ready"]:
                            raise ValueError(base["reason"])
                        # Refresh gravity at the latest IMU pose before output,
                        # while keeping the original measured torque sample.
                        self.gravity=self.model.gravity(self.sample.q)
                        self.control.feedback(self.sample,self.gravity)
                    self.observer._waist_current(now_ns, ros_now_ns)
                    if self.arm_blocked:
                        raise ValueError(self.arm_blocked)
                except ValueError as error:
                    self.control.fail(error)
                plan = self.control.step(now_ns, ros_now_ns)
                if plan is not None and (self.control.fault is None or self.plans_sent > 0):
                    send(plan)
                    self.plans_sent += 1
                if self.control.fault:
                    self.store.set_owner(None, now_ns)
        if now_ns-self.last_snapshot_ns >= 50_000_000:
            self.store.write(self.snapshot(now_ns, ros_now_ns))
            self.last_snapshot_ns = now_ns

    def startup_checks(self, now_ns, ros_now_ns):
        sample = self.sample
        waist = self.waist_status(now_ns,ros_now_ns)
        def fresh(source, received, limit):
            return 0 <= (now_ns-received)*1e-9 <= limit and -.01 <= (ros_now_ns-source)*1e-9 <= limit
        if self.static_startup_checks is None:
            self.static_startup_checks=[("interface", "后端允许网页启动控制", self.control_enabled)]
            for flag,label in (("actual_onboard_sdk_verified","机载 SDK 已核验"),
                               ("base_assumption_confirmed","骨盆支撑与重力方向已确认"),
                               ("effort_feedback_semantics_confirmed","关节力矩反馈含义已确认")):
                self.static_startup_checks.append((flag,label,self.cfg.get(flag) is True))
        checks=list(self.static_startup_checks)
        if self.base_monitor is not None:
            checks.append(("base_imu", "胯部 IMU 有效且骨盆接近静止", self.base_monitor.status(now_ns,ros_now_ns)["ready"]))
        checks.extend([
            ("feedback", "关节反馈有效且新鲜", sample is not None and
             fresh(sample.source_ns, sample.received_ns, min(self.cfg["feedback_timeout_s"], self.cfg["source_max_age_s"])) and self.error is None),
            ("waist", "腰部消息有效且新鲜", waist["valid"] and waist["fresh"]),
            ("waist_stationary", "腰部运动满足当前模型限制", waist["stationary"]),
            ("limits", "关节角可接管（小幅越界仅允许回收）" if np.any(recovery_margins(self.cfg)) else "实际关节角位于控制范围",
             sample is not None and admissible_start(self.model,self.cfg,sample.q)),
            ("publisher", "控制通道可用且无其他指令发布器", self.arm_blocked is None)])
        return [dict(key=key, label=label, ok=bool(ok)) for key, label, ok in checks]

    def snapshot(self, now_ns, ros_now_ns):
        if now_ns-self.last_timing_snapshot_ns >= 1_000_000_000:
            self.timing_cache={name:dict(samples=len(values),mean_ms=float(np.mean(values)),p95_ms=float(np.percentile(values,95)),max_ms=float(max(values)))
                               for name,values in self.timing.items() if values}
            self.last_timing_snapshot_ns=now_ns
        sample = self.sample
        age = None if sample is None else max(0., (now_ns-sample.received_ns)*1e-6)
        force_age = None if self.wrench is None else max(0., (ros_now_ns-self.wrench["available_at_s"]*1e9)*1e-6)
        control = self.control
        checks = self.startup_checks(now_ns, ros_now_ns)
        base=None if self.base_monitor is None else self.base_monitor.status(now_ns,ros_now_ns)
        quality=self.observer.quality()
        if base is not None:
            quality["model_scope"]="supported quasi-static pelvis; gravity from pelvis IMU and model mounting rotation; no base/waist acceleration dynamics"
        if base is not None and not base["ready"]:
            quality["physical_assumptions_confirmed"]=False
            quality["reasons"].append("base_imu_not_ready")
        if not self.waist_status(now_ns,ros_now_ns)["stationary"]:
            quality["physical_assumptions_confirmed"]=False
            quality["reasons"].append("waist_not_stationary")
        owner, _ = self.store.lease()
        margin=recovery_margins(self.cfg)
        reference=None if sample is None else sample.q if control is None or control.position is None else control.position
        slider_lower=self.model.lower if reference is None else np.where(margin>0,np.minimum(self.model.lower,reference),self.model.lower)
        slider_upper=self.model.upper if reference is None else np.where(margin>0,np.maximum(self.model.upper,reference),self.model.upper)
        return dict(instance_id=self.store.instance_id, backend=self.backend, mode=self.mode,
                    grippers=self.grippers.status(now_ns,ros_now_ns),gripper_error=self.gripper_error,
                    gripper_available=self.gripper_transport is not None and self.control_enabled,
                    gripper_last_targets=dict(self.gripper_last_targets),gripper_commands_sent=self.gripper_commands_sent,
                    gripper_drive=self.cfg.get('panel_gripper_drive',dict(velocity=.2,acceleration=.2,deceleration=.2,effort=.2)),
                    gripper_requests_generated=self.gripper_requests_generated,
                    gripper_hardware_publisher_created=self.gripper_hardware_publisher_created,
                    timing=self.timing_cache,command_hz=self.cfg['command_hz'],
                    wrench_worker_alive=None if self.wrench_worker is None else self.wrench_worker.process.is_alive(),
                    connection="live" if age is not None and age <= 100 else "stale" if sample is not None else "waiting",
                    feedback_age_ms=age, wrench_age_ms=force_age,
                    feedback_timeout_ms=self.cfg["feedback_timeout_s"]*1000,
                    base_imu_timeout_ms=self.cfg.get("panel_base_imu_timeout_s",.1)*1000,
                    waist_timeout_ms=self.cfg["waist_timeout_s"]*1000,
                    waist_source_timeout_ms=self.cfg["source_max_age_s"]*1000,
                    waist_feedback=self.waist_status(now_ns,ros_now_ns),
                    joint_names=ARM_NAMES, lower_rad=self.model.lower.tolist(), upper_rad=self.model.upper.tolist(),
                    slider_lower_rad=slider_lower.tolist(),slider_upper_rad=slider_upper.tolist(),
                    limit_recovery_enabled=bool(np.any(margin)),
                    limit_recovery_allowed=sample is not None and admissible_start(self.model,self.cfg,sample.q),
                    q_rad=None if sample is None else sample.q.tolist(),
                    goal_rad=None if control is None or control.goal is None else control.goal.tolist(),
                    trajectory_rad=None if control is None or control.position is None else control.position.tolist(),
                    gravity_Nm=None if self.gravity is None else self.gravity.tolist(),
                    wrench=self.wrench, quality=quality, base_imu=base, failures=dict(self.failures),
                    joint_limit_violations=[] if sample is None else [dict(name=ARM_NAMES[i],position_rad=float(sample.q[i]),lower_rad=float(self.model.lower[i]),upper_rad=float(self.model.upper[i]))
                        for i in range(14) if not self.model.lower[i] <= sample.q[i] <= self.model.upper[i]],
                    control_available=self.control_enabled, startup_checks=checks,
                    can_arm=all(check["ok"] for check in checks),
                    control_state=("idle" if self.control_enabled else "observe") if control is None else control.state,
                    control_owner=owner, fault=None if control is None else control.fault,
                    error=self.error or self.action_error, last_request_error=self.action_error, arm_blocked=self.arm_blocked,
                    velocity_limit_rad_s=self.cfg["panel_max_velocity_rad_s"],
                    motor_commands_sent=self.motor_commands_sent,
                    control_commands_generated=self.plans_sent,
                    hardware_command_publisher_created=self.hardware_publisher_created)


def telemetry(names, q, dq, tau, source_ns):
    sec, ns = divmod(int(source_ns), 1_000_000_000)
    return Obj(header=Obj(stamp=Obj(sec=sec, nanosec=ns)), state=Obj(value=0),
               joints=[Obj(name=n, position=float(p), velocity=float(v), effort=float(t), error_code=0)
                       for n, p, v, t in zip(names, q, dq, tau)])


class DemoPlant:
    """Fixed-pelvis Pinocchio numerical plant; no ROS, hardware or collision world."""
    def __init__(self, engine):
        self.engine = engine
        self.q = np.array([-.4, .7, .2, -1., .1, .1, .1, -.4, -.7, -.2, -1., -.1, .1, -.1])
        self.dq = np.zeros(14)
        self.home = self.q.copy()
        self.plan = None
        self.tau = engine.model.gravity(self.q)
        self.elapsed = 0.
        self.gripper_openings={'left':.5,'right':.5}

    def step(self, dt, now_ns, source_ns):
        model = self.engine.model
        if self.plan is None:
            target, target_velocity, kp, kd, extra = self.home, np.zeros(14), np.full(14,20.), np.full(14,3.), model.gravity(self.q)
        else:
            p = self.plan
            target, target_velocity, kp, kd, extra = p.target, p.velocity, p.kp, p.kd, p.extra
        external = np.zeros(14)
        for i, side in enumerate(("left", "right")):
            f = np.array([.8*np.sin(self.elapsed*.8+i), .4*np.cos(self.elapsed*.5+i), .5, 0, 0, 0])
            external[i*7:(i+1)*7] = model.jacobian(self.q, side).T @ f
        mass, bias = model.terms(self.q, self.dq)
        # Implicit PD integration avoids explicit damping instability on
        # low-inertia wrist joints. The reported effort is the actual drive
        # torque used in this integration step, including PD and feedforward.
        system = mass+np.diag(dt*kd+dt*dt*kp)
        self.dq = np.linalg.solve(system, mass@self.dq+dt*(kp*(target-self.q)+kd*target_velocity+extra+external-bias))
        self.q += self.dq*dt
        tau = kp*(target-self.q)+kd*(target_velocity-self.dq)+extra
        self.tau, self.elapsed = tau, self.elapsed+dt
        self.engine.waist_feedback(telemetry(WAIST_NAMES, np.zeros(3), np.zeros(3), np.zeros(3), source_ns), now_ns, source_ns)
        self.engine.arm_feedback(telemetry(ARM_NAMES, self.q, self.dq, tau, source_ns), now_ns, source_ns)
        self.engine.gripper_feedback(Obj(header=Obj(stamp=Obj(sec=source_ns//1_000_000_000,nanosec=source_ns%1_000_000_000)),
            left_hand_type=Obj(value=2),right_hand_type=Obj(value=2),
            left_hands=[Obj(name='left_claw_joint',position=self.gripper_openings['left'],state=0,faultcode=0)],
            right_hands=[Obj(name='right_claw_joint',position=self.gripper_openings['right'],state=0,faultcode=0)]),now_ns,source_ns)


def run_robot(engine, stop):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from aimdk_msgs.msg import JointStateArray, JointState, JointCommandArray, JointCommand
    from sensor_msgs.msg import Imu
    from aimdk_msgs.msg import HandCommandArray,HandCommand,HandType,HandStateArray
    from rclpy.executors import SingleThreadedExecutor
    from .async_wrench import WrenchWorker
    verify_sdk_types(JointCommandArray, JointCommand, JointStateArray, JointState)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("x2_joint_web_panel")
    executor=SingleThreadedExecutor()
    executor.add_node(node)
    if engine.cfg.get('panel_async_wrench',True):
        engine.wrench_worker=WrenchWorker(engine.cfg,engine.flange_config)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    pub = [None]
    hand_pub=[None]
    engine.arm_blocked = "等待 DDS 发现" if engine.control_enabled else None
    def activate_transport():
        own_count = 1 if pub[0] is not None else 0
        if node.count_publishers(engine.cfg["arm_command_topic"]) > own_count:
            raise ValueError("检测到其他手臂指令发布器")
        if pub[0] is None:
            pub[0] = node.create_publisher(JointCommandArray, engine.cfg["arm_command_topic"], qos)
            engine.hardware_publisher_created = True
    engine.activate_transport = activate_transport
    engine.refresh_clocks = lambda: (time.monotonic_ns(), node.get_clock().now().nanoseconds)
    def send_gripper(side,opening,source_ns):
        drive=engine.cfg.get('panel_gripper_drive',dict(velocity=.2,acceleration=.2,deceleration=.2,effort=.2))
        topic='/aima/hal/joint/hand/command'
        own=1 if hand_pub[0] is not None else 0
        if node.count_publishers(topic)>own:raise ValueError('检测到其他夹爪指令发布器')
        if hand_pub[0] is None:
            hand_pub[0]=node.create_publisher(HandCommandArray,topic,qos)
            engine.gripper_hardware_publisher_created=True
        msg=build_gripper_command(HandCommandArray,HandCommand,HandType,side,opening,node.get_clock().now().nanoseconds,
                                 sequence=engine.gripper_commands_sent,drive=drive)
        hand_pub[0].publish(msg);engine.gripper_commands_sent+=1
    # Type/serialization validation creates message objects only, no publisher.
    build_gripper_command(HandCommandArray,HandCommand,HandType,'left',.5,1,drive=engine.cfg.get('panel_gripper_drive'))
    engine.gripper_transport=send_gripper
    hand_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(HandStateArray,'/aima/hal/joint/hand/state',
        lambda m:engine.gripper_feedback(m,time.monotonic_ns(),node.get_clock().now().nanoseconds),hand_qos)
    def send(plan):
        if pub[0] is not None:
            pub[0].publish(build_command(JointCommandArray, JointCommand, q_target=plan.target,
                velocity_target=plan.velocity, kp=plan.kp, kd=plan.kd, extra=plan.extra,
                source_ns=node.get_clock().now().nanoseconds, sequence=engine.sequence))
            engine.sequence += 1
            engine.motor_commands_sent += 1
    node.create_subscription(JointStateArray, engine.cfg["waist_state_topic"],
        lambda m: engine.waist_feedback(m, time.monotonic_ns(), node.get_clock().now().nanoseconds), qos)
    node.create_subscription(JointStateArray, engine.cfg["arm_state_topic"],
        lambda m: engine.arm_feedback(m, time.monotonic_ns(), node.get_clock().now().nanoseconds), qos)
    if engine.base_monitor is not None:
        node.create_subscription(Imu,engine.cfg["panel_base_imu_topic"],
            lambda m:engine.base_feedback(m,time.monotonic_ns(),node.get_clock().now().nanoseconds),qos)
    created = time.monotonic_ns()
    graph_checked = [0]
    def tick():
        now = time.monotonic_ns()
        if engine.last_tick_ns is not None:engine.timing['tick_gap_ms'].append((now-engine.last_tick_ns)*1e-6)
        engine.last_tick_ns=now
        if engine.control_enabled and now-graph_checked[0] > 200_000_000:
            graph_checked[0] = now
            engine.arm_blocked = ("等待 DDS 发现" if now-created < 2_000_000_000 else
                "检测到其他手臂指令发布器" if node.count_publishers(engine.cfg["arm_command_topic"]) > (1 if pub[0] is not None else 0) else None)
        engine.step(now, node.get_clock().now().nanoseconds, send)
        engine.timing['tick_work_ms'].append((time.monotonic_ns()-now)*1e-6)
    node.create_timer(1/engine.cfg["command_hz"], tick)
    try:
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=.002)
    finally:
        if engine.motor_commands_sent:
            engine.control.hold()
            deadline = time.monotonic()+engine.cfg["exit_hold_duration_s"]
            while rclpy.ok() and time.monotonic() < deadline:
                send(engine.control.hold_plan())
                time.sleep(1/engine.cfg["command_hz"])
        publishers = node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace())
        print(diagnostic_json(dict(motor_commands_sent=engine.motor_commands_sent,
            gripper_commands_sent=engine.gripper_commands_sent,
            own_hand_command_publishers=[p for p,_ in publishers if p=='/aima/hal/joint/hand/command'],
            own_hardware_command_publishers=[p for p, _ in publishers if p == engine.cfg["arm_command_topic"]],
            observer_windows=engine.observer.output_count, failures=dict(engine.failures))))
        if engine.wrench_worker is not None:engine.wrench_worker.close()
        executor.remove_node(node);executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("robot", "demo"), default="robot")
    parser.add_argument("--mode", choices=("observe", "web", "control"), default="web")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--instance-id", default="manual")
    parser.add_argument("--config", type=Path, default=ROOT/"configs/robot.json")
    parser.add_argument("--duration-s", type=float, default=0.)
    args = parser.parse_args(argv)
    cfg = json.loads(args.config.read_text(encoding="utf-8"))
    cfg.update(json.loads((ROOT/"configs/joint_panel.json").read_text(encoding="utf-8")))
    cfg['command_hz']=float(cfg.get('panel_command_hz',50.))
    if not np.isfinite(cfg['command_hz']) or not 1 <= cfg['command_hz'] <= 250:
        raise ValueError('Invalid panel command frequency')
    flange = json.loads((ROOT/cfg["flange_config"]).read_text(encoding="utf-8"))
    if args.backend == "demo":
        cfg.update({k: True for k in ("actual_onboard_sdk_verified", "base_assumption_confirmed",
                   "control_ownership_confirmed", "effort_feedback_semantics_confirmed")})
        flange["calibration_confirmed"] = True
    store = PanelStore(args.instance_id, args.mode != "observe")
    engine = PanelEngine(cfg, flange, store, backend=args.backend, mode=args.mode)
    store.write(engine.snapshot(time.monotonic_ns(), time.time_ns()))
    server = make_server(store, args.port)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    stop = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *unused: stop.set())
    start = time.monotonic()
    timer = None
    if args.duration_s > 0:
        timer = threading.Timer(args.duration_s, stop.set)
        timer.daemon = True
        timer.start()
    print(diagnostic_json(dict(url=f"http://127.0.0.1:{server.server_port}", backend=args.backend,
                              mode=args.mode, instance_id=args.instance_id)), flush=True)
    try:
        if args.backend == "robot":
            run_robot(engine, stop)
        else:
            plant = DemoPlant(engine)
            engine.gripper_transport=lambda side,opening,source:plant.gripper_openings.__setitem__(side,opening)
            last = time.monotonic()
            while not stop.is_set():
                now = time.monotonic()
                dt = min(.02, max(.0001, now-last))
                last = now
                plant.step(dt, time.monotonic_ns(), time.time_ns())
                engine.step(time.monotonic_ns(), time.time_ns(), lambda p: setattr(plant, "plan", p))
                stop.wait(max(0., .004-(time.monotonic()-now)))
            print(diagnostic_json(dict(backend="demo", motor_commands_sent=0,
                observer_windows=engine.observer.output_count, fault=None if engine.control is None else engine.control.fault)))
    finally:
        if timer:
            timer.cancel()
        stop.set()
        server.shutdown()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
