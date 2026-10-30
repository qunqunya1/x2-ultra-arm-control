"""Sensor/backend-independent joint-impedance and endpoint wrench estimation.

Wrench order [Fx,Fy,Fz,Mx,My,Mz], environment acting ON the robot.
J rows [linear velocity; angular velocity] at the same point and in the
same axes as the wrench. A valid numerical result is not a safety guarantee.
"""
from dataclasses import dataclass
import numpy as np

from .core import vector


def impedance_torque(q, dq, q_target, kp, kd, torque_extra, dq_target=None):
    """Requested joint torque BEFORE clipping/internal servo dynamics.

    With no velocity command supplied, assume dq_target=0. Real firmware
    semantics and gain units must be confirmed before using reconstruction.
    """
    q = vector(q, 7, "q")
    dq = vector(dq, 7, "dq")
    target = vector(q_target, 7, "q_target")
    p, d = vector(kp, 7, "kp"), vector(kd, 7, "kd")
    if np.any(p < 0) or np.any(d < 0):
        raise ValueError("PD gains must be nonnegative")
    velocity_target = np.zeros(7) if dq_target is None else vector(dq_target, 7, "dq_target")
    extra = vector(torque_extra, 7, "torque_extra")
    return p*(target-q)+d*(velocity_target-dq)+extra


def external_joint_residual(model_required_torque, actuator_torque, torque_bias=None):
    """For M*a + h - passive = tau_act + J.T*w: residual = required-act.

    model_required_torque must match the actuator signal convention. If the
    signal already has gravity removed, subtract gravity on the model side
    too. torque_bias is an identified *residual* zero-contact offset.
    """
    required = vector(model_required_torque, 7, "model_required_torque")
    actual = vector(actuator_torque, 7, "actuator_torque")
    bias = np.zeros(7) if torque_bias is None else vector(torque_bias, 7, "torque_bias")
    return required-actual-bias


def window_model_torque(mass_previous, mass_current, bias_previous, bias_current,
                        velocity_previous, velocity_current, dt):
    """Average model torque over a *past* synchronized observation window.

    Use the matching interval-average actuator torque when subtracting this
    value. Acceleration is a window difference; do not low-pass it separately
    against instantaneous torque. Changing mass/bias is trapezoid-approximated.
    Output wrench must be timestamped as a trailing-window estimate.
    """
    matrices = [np.asarray(value, float) for value in (mass_previous, mass_current)]
    if any(value.shape != (7, 7) or not np.isfinite(value).all() for value in matrices):
        raise ValueError("Finite 7x7 mass matrices required")
    if not np.isfinite(dt) or dt <= 0 or dt > .1:
        raise ValueError("Window duration must be positive and <=0.1s")
    previous = vector(velocity_previous, 7, "velocity_previous")
    current = vector(velocity_current, 7, "velocity_current")
    previous_bias = vector(bias_previous, 7, "bias_previous")
    current_bias = vector(bias_current, 7, "bias_current")
    return .5*(matrices[0]+matrices[1]) @ ((current-previous)/dt)+.5*(previous_bias+current_bias)


class LowPass:
    def __init__(self, cutoff_hz):
        if not np.isfinite(cutoff_hz) or cutoff_hz <= 0:
            raise ValueError("cutoff_hz must be positive")
        self.cutoff_hz = float(cutoff_hz)
        self.value = None

    def update(self, value, dt):
        value = np.asarray(value, dtype=float)
        if not np.isfinite(value).all() or not np.isfinite(dt) or dt <= 0:
            raise ValueError("Finite value and positive dt required")
        if self.value is None:
            self.value = value.copy()
        elif self.value.shape != value.shape:
            raise ValueError("Filter signal shape changed")
        else:
            alpha = -np.expm1(-2*np.pi*self.cutoff_hz*dt)
            self.value += alpha*(value-self.value)
        return self.value.copy()


class AccelerationFromVelocity:
    """Causal backward velocity difference followed by a first-order LPF.

    This intentionally does not accept simulator acceleration or applied forces.
    Reset the object between trials or after a sensor time discontinuity.
    """
    def __init__(self, cutoff_hz=20.):
        self.filter = LowPass(cutoff_hz)
        self.previous_velocity = None
        self.previous_time = None

    def update(self, velocity, timestamp_s):
        velocity = vector(velocity, 7, "velocity")
        if not np.isfinite(timestamp_s):
            raise ValueError("Finite timestamp required")
        if self.previous_time is None:
            self.previous_velocity = velocity.copy()
            self.previous_time = float(timestamp_s)
            return np.zeros(7), False
        dt = float(timestamp_s)-self.previous_time
        if dt <= 0 or dt > .1:
            raise ValueError("Nonmonotonic or stale velocity samples; reset acceleration estimator")
        difference = (velocity-self.previous_velocity)/dt
        acceleration = self.filter.update(difference, dt)
        self.previous_velocity = velocity.copy()
        self.previous_time = float(timestamp_s)
        return acceleration, True


@dataclass(frozen=True)
class WrenchResult:
    wrench: np.ndarray
    valid: bool
    reasons: tuple
    scaled_rank: int
    scaled_condition: float
    joint_fit_residual_Nm: float
    torque_source: str


def estimate_wrench(jacobian, joint_residual, *, torque_source="measured",
                    characteristic_length_m=.1, damping_m=.0001,
                    max_condition=100., torque_saturated=False):
    """Scaled damped least squares, no access to applied Cartesian load.

    Solve r = J.T D z, where w=Dz and D=diag(1,1,1,L,L,L). The six
    entries of z are all force-equivalent quantities; damping has length units.
    'reconstructed' is a model assumption, not measured actuator feedback.
    """
    j = np.asarray(jacobian, dtype=float)
    if j.shape != (6, 7) or not np.isfinite(j).all():
        raise ValueError("Expected finite 6x7 geometric Jacobian")
    residual = vector(joint_residual, 7, "joint_residual")
    if torque_source not in ("measured", "reconstructed"):
        raise ValueError("torque_source must be measured or reconstructed")
    if any(not np.isfinite(v) or v <= 0 for v in (characteristic_length_m, damping_m, max_condition)):
        raise ValueError("Positive finite scaling, damping and condition limit required")
    scale = np.diag([1., 1., 1., *([characteristic_length_m]*3)])
    a = j.T @ scale
    singular_values = np.linalg.svd(a, compute_uv=False)
    rank = int(np.count_nonzero(singular_values > max(1e-10, singular_values[0]*1e-8)))
    condition = float(singular_values[0]/singular_values[-1]) if singular_values[-1] > 1e-12 else float("inf")
    augmented = np.vstack((a, damping_m*np.eye(6)))
    rhs = np.r_[residual, np.zeros(6)]
    z = np.linalg.lstsq(augmented, rhs, rcond=None)[0]
    wrench = scale @ z
    reasons = []
    if rank < 6:
        reasons.append("rank_deficient")
    if condition > max_condition:
        reasons.append("poor_observability")
    if torque_saturated:
        reasons.append("actuator_saturation")
    return WrenchResult(wrench=wrench, valid=not reasons, reasons=tuple(reasons), scaled_rank=rank,
                        scaled_condition=condition, joint_fit_residual_Nm=float(np.linalg.norm(j.T @ wrench-residual)),
                        torque_source=torque_source)
