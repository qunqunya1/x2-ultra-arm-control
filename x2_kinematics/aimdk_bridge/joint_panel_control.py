"""Joint slider control independent of HTTP and ROS transports."""
from dataclasses import dataclass
import numpy as np
from .runtime import repeated
from .protocol import ARM_NAMES


def recovery_margins(config):
    margin=float(config.get("panel_limit_recovery_margin_rad",0.))
    names=config.get("panel_limit_recovery_joints",[])
    if not np.isfinite(margin) or not 0 <= margin <= np.deg2rad(5) or not isinstance(names,list) or any(n not in ARM_NAMES for n in names):
        raise ValueError("Invalid joint limit recovery configuration")
    return np.array([margin if n in names else 0. for n in ARM_NAMES])


def admissible_start(model, config, q):
    margin=recovery_margins(config)
    return bool(np.isfinite(q).all() and np.all(q >= model.lower-margin) and np.all(q <= model.upper+margin))


@dataclass
class JointPlan:
    target: np.ndarray
    velocity: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    extra: np.ndarray


class JointPanelControl:
    def __init__(self, dynamics, config):
        for key in ("actual_onboard_sdk_verified", "base_assumption_confirmed",
                    "effort_feedback_semantics_confirmed"):
            if config.get(key) is not True:
                raise ValueError("Control requires "+key+"=true")
        self.model, self.cfg = dynamics, dict(config)
        self.recovery_margin=recovery_margins(config)
        self.recovery_lower=self.recovery_upper=None
        self.kp = repeated(config["panel_joint_kp"], "panel_joint_kp")
        self.kd = repeated(config["panel_joint_kd"], "panel_joint_kd")
        if np.any(self.kp <= 0):
            raise ValueError("Position control requires positive stiffness")
        self.vmax = float(config["panel_max_velocity_rad_s"])
        self.amax = float(config["panel_max_acceleration_rad_s2"])
        self.timeout = float(config["panel_lease_timeout_s"])
        self.tracking_limit = float(config["panel_max_tracking_error_rad"])
        self.ramp_s = float(config["gravity_ramp_s"])
        self.scale = float(config["gravity_scale"])
        self.extra_limit = repeated(config["max_extra_torque_Nm"], "max_extra_torque_Nm")
        if not all(np.isfinite(x) and x > 0 for x in
                   (self.vmax, self.amax, self.timeout, self.tracking_limit, self.ramp_s)):
            raise ValueError("Panel limits must be finite and positive")
        if not np.isfinite(self.scale) or not 0 <= self.scale <= 1:
            raise ValueError("Invalid gravity scale")
        self.sample = self.gravity = self.goal = self.position = self.velocity = None
        self.state, self.fault = "idle", None
        self.started_ns = self.last_command_ns = self.initial_extra = None

    @property
    def active(self):
        return self.state in ("armed", "holding", "fault") and self.position is not None

    def feedback(self, sample, gravity):
        self.sample, self.gravity = sample, np.asarray(gravity, float).copy()

    def _fresh(self, now_ns, ros_now_ns):
        if self.sample is None:
            raise ValueError("No joint feedback")
        age = (now_ns-self.sample.received_ns)*1e-9
        source_age = (ros_now_ns-self.sample.source_ns)*1e-9
        if not 0 <= age <= self.cfg["feedback_timeout_s"] or not -.01 <= source_age <= self.cfg["source_max_age_s"]:
            raise ValueError("Joint feedback is stale")
        lower=self.model.lower-self.recovery_margin if self.recovery_lower is None else self.recovery_lower
        upper=self.model.upper+self.recovery_margin if self.recovery_upper is None else self.recovery_upper
        if np.any(self.sample.q < lower-1e-12) or np.any(self.sample.q > upper+1e-12):
            raise ValueError("Actual joint position is outside control limits")
        if self.recovery_lower is not None:
            # Once measured feedback returns inside, revoke its temporary
            # allowance. A later excursion is again a normal limit fault.
            self.recovery_lower=np.where(self.sample.q >= self.model.lower,self.model.lower,self.recovery_lower)
            self.recovery_upper=np.where(self.sample.q <= self.model.upper,self.model.upper,self.recovery_upper)

    def arm(self, now_ns, ros_now_ns):
        if self.fault:
            raise ValueError("Latched fault; restart after resolving: "+self.fault)
        self._fresh(now_ns, ros_now_ns)
        self.initial_extra = self.sample.torque+self.kd*self.sample.dq
        if np.any(np.abs(self.initial_extra) > self.extra_limit):
            raise ValueError("Initial measured torque exceeds configured limit")
        self.position, self.velocity = self.sample.q.copy(), np.zeros(14)
        self.goal = self.position.copy()
        # Only a small allowance around the captured out-of-range pose is
        # accepted as feedback. Do not grant the full configured window
        # for later outward drift. This tolerance is 0.17 degrees.
        self.recovery_lower=np.maximum(self.model.lower-self.recovery_margin,np.where(self.position < self.model.lower,self.position-.003,self.model.lower))
        self.recovery_upper=np.minimum(self.model.upper+self.recovery_margin,np.where(self.position > self.model.upper,self.position+.003,self.model.upper))
        self.started_ns, self.last_command_ns = int(now_ns), int(now_ns)
        self.state = "armed"

    def target(self, value):
        q = np.asarray(value, float)
        if self.state != "armed":
            raise ValueError("Slider control is not armed")
        if q.shape != (14,) or not np.isfinite(q).all():
            raise ValueError("Target requires fourteen finite radians")
        lower=np.minimum(self.model.lower,self.position)
        upper=np.maximum(self.model.upper,self.position)
        if np.any(q < lower-1e-12) or np.any(q > upper+1e-12):
            raise ValueError("Target is outside joint limits")
        self.goal = q.copy()

    def hold(self):
        if self.active:
            self.position, self.velocity = self.sample.q.copy(), np.zeros(14)
            self.goal = self.position.copy()
            self.state = "fault" if self.fault else "holding"

    def fail(self, reason):
        if self.active and self.fault is None:
            self.fault = str(reason)
            self.hold()

    def step(self, now_ns, ros_now_ns):
        if not self.active:
            return None
        try:
            self._fresh(now_ns, ros_now_ns)
            dt = (now_ns-self.last_command_ns)*1e-9
            if not 0 <= dt <= self.cfg["feedback_timeout_s"]:
                raise ValueError("Control loop stalled")
            if np.max(np.abs(self.position-self.sample.q)) > self.tracking_limit:
                raise ValueError("Joint tracking error exceeded")
            if self.state == "armed":
                error = self.goal-self.position
                speed = np.minimum(self.vmax, np.minimum(2*np.abs(error), .8*np.sqrt(2*self.amax*np.abs(error))))
                v_new = self.velocity+np.clip(np.sign(error)*speed-self.velocity, -self.amax*dt, self.amax*dt)
                q_new = self.position+.5*(self.velocity+v_new)*dt
                lower=np.minimum(self.model.lower,self.position)
                upper=np.maximum(self.model.upper,self.position)
                if np.any(q_new < lower-1e-12) or np.any(q_new > upper+1e-12):
                    raise ValueError("Trajectory reached a joint limit")
                self.position, self.velocity = q_new, v_new
            amount = min(1., max(0., (now_ns-self.started_ns)*1e-9/self.ramp_s))
            extra = (1-amount)*self.initial_extra+amount*self.gravity*self.scale
            total = self.kp*(self.position-self.sample.q)+self.kd*(self.velocity-self.sample.dq)+extra
            if np.any(np.abs(extra) > self.extra_limit) or np.any(np.abs(total) > self.model.effort_limits):
                raise ValueError("Joint torque limit exceeded")
            self.last_command_ns = int(now_ns)
            return JointPlan(self.position.copy(), self.velocity.copy(), self.kp.copy(), self.kd.copy(), extra.copy())
        except ValueError as error:
            self.fail(error)
            return self.hold_plan()

    def hold_plan(self):
        if not self.active:
            return None
        extra = np.clip(self.gravity*self.scale, -self.extra_limit, self.extra_limit)
        return JointPlan(self.position.copy(), np.zeros(14), self.kp.copy(), self.kd.copy(), extra)
