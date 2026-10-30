"""ROS-independent runtime state machine for observe/gravity modes."""
from dataclasses import dataclass
import numpy as np

from ..dual_wrench import JointFeedback
from ..flange_sensor import VirtualFlangeSensorEstimator
from .protocol import decode_arm, ordered_joints, stamp_ns, WAIST_NAMES


def repeated(value, label):
    x = np.asarray(value, float)
    if x.shape == (7,):
        x = np.tile(x, 2)
    if x.shape != (14,) or not np.isfinite(x).all() or np.any(x < 0):
        raise ValueError(label+" requires seven or fourteen nonnegative finite values")
    return x.copy()


@dataclass(frozen=True)
class CommandPlan:
    target: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    extra: np.ndarray


class ArmRuntime:
    def __init__(self, dynamics, config, flange, *, mode="observe"):
        if mode not in ("observe", "gravity"):
            raise ValueError("Unknown runtime mode")
        self.dynamics, self.config, self.mode = dynamics, dict(config), mode
        for key in ("command_hz", "window_s", "max_sample_gap_s", "source_max_age_s", "feedback_timeout_s",
                    "waist_timeout_s", "gravity_ramp_s", "max_joint_velocity_rad_s", "max_drift_from_start_rad",
                    "max_waist_velocity_rad_s", "max_waist_change_rad", "exit_hold_duration_s"):
            if not np.isfinite(config[key]) or config[key] <= 0:
                raise ValueError("Invalid "+key)
        if not 0 <= config["gravity_scale"] <= 1:
            raise ValueError("gravity_scale must be in [0,1]")
        if mode == "gravity":
            for flag in ("actual_onboard_sdk_verified", "base_assumption_confirmed", "control_ownership_confirmed",
                         "effort_feedback_semantics_confirmed"):
                if config.get(flag) is not True:
                    raise ValueError("Gravity output requires "+flag+"=true")
        self.kp = repeated(config["joint_kp"], "joint_kp")
        self.kd = repeated(config["joint_kd"], "joint_kd")
        self.exit_kp = repeated(config["exit_hold_kp"], "exit_hold_kp")
        self.exit_kd = repeated(config["exit_hold_kd"], "exit_hold_kd")
        self.extra_limit = repeated(config["max_extra_torque_Nm"], "max_extra_torque_Nm")
        self.estimator = VirtualFlangeSensorEstimator(
            dynamics, wrist_from_sensor=flange["wrist_from_sensor"],
            sensor_convention=flange["sensor_convention"],
            calibration_confirmed=flange["calibration_confirmed"],
            window_s=config["window_s"], max_sample_gap_s=config["max_sample_gap_s"])
        self.waist = self.latest = self.q_start = self.start_ns = self.waist_start = None
        self.last_source_ns = None
        self.fault = None
        self.last_gravity = None
        self.command_start_ns = self.command_initial_extra = None
        self.output_count = 0
        self.limit_mismatch = False

    def _age(self, source, ros_now, limit):
        age = (int(ros_now)-int(source))*1e-9
        if not -.01 <= age <= limit:
            raise ValueError("source_timestamp_stale_or_clock_mismatch")

    def update_waist(self, message, received_ns, ros_now_ns):
        try:
            source = stamp_ns(message)
            self._age(source, ros_now_ns, self.config["source_max_age_s"])
            if self.waist is not None and source <= self.waist[2]:
                raise ValueError("nonmonotonic_waist_source_timestamp")
            q, dq, _, _ = ordered_joints(message, WAIST_NAMES)
            if np.max(np.abs(dq)) > self.config["max_waist_velocity_rad_s"]:
                raise ValueError("waist_moving_arm_only_model_invalid")
            if self.waist_start is not None and np.max(np.abs(q-self.waist_start)) > self.config["max_waist_change_rad"]:
                raise ValueError("waist_posture_changed_restart_required")
            self.waist = (q, int(received_ns), source)
            lower = self.dynamics.model.lowerPositionLimit[self.dynamics.waist_qi]
            upper = self.dynamics.model.upperPositionLimit[self.dynamics.waist_qi]
            if np.any(q < lower) or np.any(q > upper):
                self.limit_mismatch = True
                if self.mode == "gravity":
                    raise ValueError("waist_outside_urdf_limits")
        except ValueError as error:
            self.invalidate(str(error))
            raise

    def invalidate(self, reason):
        self.estimator.reset()
        self.last_source_ns = None
        if self.mode == "gravity":
            self.fault = str(reason)

    def _waist_current(self, received_ns, ros_now_ns):
        if self.waist is None or not 0 <= (received_ns-self.waist[1])*1e-9 <= self.config["waist_timeout_s"]:
            raise ValueError("waist_feedback_missing_or_stale")
        self._age(self.waist[2], ros_now_ns, self.config["waist_timeout_s"])
        self.dynamics.set_waist(self.waist[0])

    def update_arm(self, message, received_ns, ros_now_ns):
        try:
            sample=decode_arm(message,received_ns)
        except ValueError as error:
            self.invalidate(str(error))
            raise
        return self.accept_sample(sample,received_ns,ros_now_ns)

    def accept_sample(self, sample, received_ns, ros_now_ns, *, estimate=True):
        """Validate an already decoded sample; optionally defer wrench work."""
        if self.fault is not None:
            raise ValueError("latched_fault:"+self.fault)
        try:
            self._age(sample.source_ns, ros_now_ns, self.config["source_max_age_s"])
            self._waist_current(received_ns, ros_now_ns)
            if self.last_source_ns is not None and sample.source_ns <= self.last_source_ns:
                raise ValueError("nonmonotonic_arm_source_timestamp")
            if np.any(sample.q < self.dynamics.lower) or np.any(sample.q > self.dynamics.upper):
                self.limit_mismatch = True
                if self.mode == "gravity":
                    raise ValueError("arm_outside_urdf_limits")
            if np.max(np.abs(sample.dq)) > self.config["max_joint_velocity_rad_s"]:
                raise ValueError("arm_velocity_exceeded")
            if self.q_start is not None and np.max(np.abs(sample.q-self.q_start)) > self.config["max_drift_from_start_rad"]:
                raise ValueError("arm_drift_from_start_exceeded")
            gravity = self.dynamics.gravity(sample.q)
            output = self.estimator.update(JointFeedback(sample.source_ns*1e-9, sample.q, sample.dq, sample.torque)) if estimate else None
            if self.q_start is None:
                self.q_start = sample.q.copy()
                self.start_ns = int(received_ns)
                self.waist_start = self.waist[0].copy()
            self.latest, self.last_source_ns, self.last_gravity = sample, sample.source_ns, gravity.copy()
            if output is not None:
                self.output_count += 1
            return gravity, output
        except ValueError as error:
            self.invalidate(str(error))
            raise

    def command(self, now_ns, ros_now_ns):
        if self.mode != "gravity":
            return None  # Observer can never produce a motor command plan.
        if self.fault is not None:
            raise ValueError("latched_fault:"+self.fault)
        try:
            if self.latest is None:
                return None
            sample = self.latest
            if not 0 <= (now_ns-sample.received_ns)*1e-9 <= self.config["feedback_timeout_s"]:
                raise ValueError("arm_feedback_watchdog_timeout")
            self._age(sample.source_ns, ros_now_ns, self.config["source_max_age_s"])
            self._waist_current(now_ns, ros_now_ns)
            if self.command_start_ns is None:
                # Start ramp when output actually begins, not on the first
                # feedback frame during discovery. Preserve measured total
                # drive torque at the first command (bumpless transfer).
                self.command_start_ns = int(now_ns)
                self.command_initial_extra = sample.torque-self.kp*(self.q_start-sample.q)+self.kd*sample.dq
            amount = min(1., max(0., (now_ns-self.command_start_ns)*1e-9/self.config["gravity_ramp_s"]))
            extra = (1-amount)*self.command_initial_extra+amount*self.last_gravity*self.config["gravity_scale"]
            total = self.kp*(self.q_start-sample.q)-self.kd*sample.dq+extra
            if np.any(np.abs(extra) > self.extra_limit) or np.any(np.abs(total) > self.dynamics.effort_limits):
                raise ValueError("gravity_or_total_torque_limit_exceeded")
            return CommandPlan(self.q_start.copy(), self.kp.copy(), self.kd.copy(), extra.copy())
        except ValueError as error:
            self.invalidate(str(error))
            raise

    def exit_hold(self):
        # Best-effort SDK pose hold, not a solution to link loss/kill/power loss.
        # Last measured pose is captured without inventing zeros for any joint.
        if self.mode != "gravity" or self.latest is None or self.last_gravity is None:
            return None
        extra = self.last_gravity*self.config["gravity_scale"]
        if np.any(np.abs(extra) > self.extra_limit):
            # Do not send a gravity value already known to exceed configured limits.
            extra = np.zeros(14)
        return CommandPlan(self.latest.q.copy(), self.exit_kp.copy(), self.exit_kd.copy(), extra)

    def quality(self):
        reasons = []
        if not self.config.get("actual_onboard_sdk_verified"):
            reasons.append("onboard_sdk_release_not_verified")
        if not self.config.get("base_assumption_confirmed"):
            reasons.append("fixed_upright_pelvis_assumption_unconfirmed")
        if not self.config.get("effort_feedback_semantics_confirmed"):
            reasons.append("sdk_effort_feedback_semantics_unconfirmed")
        if not self.estimator.calibration_confirmed:
            reasons.append("virtual_flange_calibration_unconfirmed")
        return dict(physical_assumptions_confirmed=not reasons, reasons=reasons,
                    warnings=["outside_urdf_planning_limits"] if self.limit_mismatch else [],
                    model_scope="fixed supported pelvis; near-stationary waist; no waist/base acceleration compensation")
