"""Waist/leg zero-and-hold, with no URDF position-limit or collision gate."""
import numpy as np
from .body_feedback import GROUPS, decode

KP=dict(leg=np.array([50,50,40,60,40,30]*2,float),waist=np.array([50,60,50],float))
KD=dict(leg=np.array([5,5,4,6,4,3]*2,float),waist=np.array([5,6,5],float))


class BodyPanelControl:
    def __init__(self):
        self.samples = {}
        self.feedback_errors = {}
        self.active = False
        self.phase = 'idle'
        self.fault = None
        self.commands_sent = 0
        self.started = None
        self.seconds = None
        self.hold_q = None
        self.settled_since = None
        self.zero_reached = False
        self.publish_rate_hz=100.

    def feedback(self, group, message, now_ns, ros_ns):
        try:
            values = decode(message, GROUPS[group])
            stamp = message.header.stamp
            source = stamp.sec * 10**9 + stamp.nanosec
            if not -.01 <= (ros_ns - source) * 1e-9 <= .15:
                raise ValueError(group + ' source feedback stale')
            if group in self.samples and source < self.samples[group][2]:
                raise ValueError(group + ' source timestamp reversed')
            self.samples[group] = (values, now_ns, source)
            self.feedback_errors.pop(group, None)
        except (ValueError, TypeError, AttributeError) as error:
            self.feedback_errors[group] = str(error)

    def require(self, now_ns, ros_ns):
        data = {}
        for group in GROUPS:
            if group in self.feedback_errors:
                raise ValueError(self.feedback_errors[group])
            if group not in self.samples:
                raise ValueError(group + ' feedback missing')
            value, received, source = self.samples[group]
            if not 0 <= (now_ns - received) * 1e-9 <= .15 or not -.01 <= (ros_ns - source) * 1e-9 <= .15:
                raise ValueError(group + ' feedback stale')
            data[group] = value
        return data

    def start(self, now_ns, ros_ns):
        if self.active and self.phase in ('capturing', 'returning_zero'):
            raise ValueError('腰腿正在回零，请等待完成或点击保持')
        data = self.require(now_ns, ros_ns)
        self.initial = {group: v.copy() for group, v in data.items()}
        self.started = now_ns
        self.seconds = 3.0
        self.hold_q = None
        self.hold_data = None
        self.active = True
        self.phase = 'returning_zero'
        self.fault = None
        self.zero_reached = False
        self.settled_since = None

    def hold(self):
        if not self.active:
            return
        self.hold_data = {group: self.samples[group][0].copy() for group in GROUPS}
        self.hold_q = {group: v[:, 0].copy() for group, v in self.hold_data.items()}
        self.phase = 'holding_current'

    def emergency_stop(self):
        self.active=False
        self.phase='damping'
        self.zero_reached=False
        self.hold_q=None

    def step(self, now_ns, ros_ns):
        if not self.active:
            return {}
        try:
            data = self.require(now_ns, ros_ns)
        except ValueError as error:
            if self.fault is None:
                self.hold()
                self.fault = str(error)
            self.phase = 'holding_fault'
            data = None
        elapsed = max(0., (now_ns - self.started) * 1e-9)
        u = float(np.clip(elapsed / self.seconds,0.,1.))
        plans = {}
        for group in GROUPS:
            kp = KP[group].copy()
            kd = KD[group].copy()
            if self.hold_q is None:
                q = self.initial[group][:, 0] * (1 - u)
                velocity = -self.initial[group][:, 0] / self.seconds if u<1 else np.zeros(len(q))
            else:
                q = self.hold_q[group]
                velocity = np.zeros(len(q))
            extra = np.zeros(len(q))
            plans[group] = dict(q=q, velocity=velocity, kp=kp, kd=kd, extra=extra)
        if self.hold_q is None:
            self.phase = 'returning_zero' if u < 1 else 'holding_zero'
            good = data is not None and u >= 1 and all(np.max(np.abs(v[:, 0])) < .02 and np.max(np.abs(v[:, 1])) < .04 for v in data.values())
            if not good:
                self.settled_since = None
            elif self.settled_since is None:
                self.settled_since = now_ns
            self.zero_reached = good and (now_ns - self.settled_since) * 1e-9 >= 3
        return plans

    def snapshot(self, now_ns, ros_ns):
        try:
            data = self.require(now_ns, ros_ns)
            ready, error = True, None
        except ValueError as exc:
            data, ready, error = {}, False, str(exc)
        return dict(available=True, ready=ready, feedback_error=error, active=self.active, phase=self.phase,
            fault=self.fault, zero_reached=self.zero_reached, commands_sent=self.commands_sent,
            duration_s=self.seconds, elapsed_s=None if self.started is None else (now_ns-self.started)*1e-9,
            publish_rate_hz=self.publish_rate_hz,trajectory='linear_to_exact_zero',feedforward='zero',
            position_limits_enforced=False, joint_names={g:list(n) for g,n in GROUPS.items()},
            q_rad={g:v[:,0].tolist() for g,v in data.items()},
            kp={g:v.tolist() for g,v in KP.items()}, kd={g:v.tolist() for g,v in KD.items()})


def build_body_command(Array, Joint, group, plan, stamp, sequence):
    message = Array()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(int(stamp), 10**9)
    if hasattr(message.header, 'sequence'):
        message.header.sequence = sequence
    if hasattr(message.header, 'frame_id'):
        message.header.frame_id = 'x2_body_zero'
    for i, name in enumerate(GROUPS[group]):
        joint = Joint()
        joint.name = name
        joint.position = float(plan['q'][i])
        joint.velocity = float(plan['velocity'][i])
        joint.stiffness = float(plan['kp'][i])
        joint.damping = float(plan['kd'][i])
        joint.effort = float(plan['extra'][i])
        message.joints.append(joint)
    return message
