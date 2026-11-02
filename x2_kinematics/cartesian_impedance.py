"""Dual-arm torque-based Cartesian impedance; command calculation only.

Control coordinates are torso_link. Diagonal K/D are expressed along each
desired tool pose's axes. No external-wrench measurement or IK is required.
Firmware contract: tau = Kp*(q_target-q) - Kd*dq + torque_extra.
"""
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .core import rigid
from .pinocchio_backend import arm_vector

SIDES = ("left", "right")


def gains(value, name):
    a = np.asarray(value, dtype=float)
    if a.shape == (6,):
        a = np.tile(a, (2, 1))
    if a.shape != (2, 6) or not np.isfinite(a).all() or np.any(a < 0):
        raise ValueError(name+" requires six or 2x6 nonnegative finite gains")
    return a.copy()


@dataclass(frozen=True)
class ImpedanceCommand:
    q_target_rad: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    torque_extra_Nm: np.ndarray
    predicted_total_torque_Nm: np.ndarray
    task_torque_Nm: np.ndarray
    nullspace_torque_Nm: np.ndarray
    gravity_torque_Nm: np.ndarray
    wrench_desired_axes: np.ndarray
    error_desired_axes: np.ndarray
    task_saturated: np.ndarray
    joint_saturated: np.ndarray
    valid: bool
    reasons: tuple

    def as_dict(self):
        names = ("q_target_rad", "kp", "kd", "torque_extra_Nm", "predicted_total_torque_Nm",
                 "task_torque_Nm", "nullspace_torque_Nm", "gravity_torque_Nm",
                 "wrench_desired_axes", "error_desired_axes", "task_saturated", "joint_saturated")
        result = {name: getattr(self, name).tolist() for name in names}
        result.update(valid=self.valid, reasons=list(self.reasons), execution_enabled=False,
                      frame="K/D and wrench in desired tool axes; poses/Jacobians/twists in torso_link",
                      external_wrench_feedback_used=False, inverse_kinematics_used=False)
        return result


class DualArmCartesianImpedance:
    def __init__(self, dynamics, *, stiffness=(300., 300., 300., 5., 5., 5.),
                 damping=(30., 30., 30., .7, .7, .7), joint_damping=.05,
                 nullspace_stiffness=.5, nullspace_damping=.35, q_rest=None,
                 max_force_N=12., max_moment_Nm=.8, max_position_error_m=.06,
                 max_orientation_error_rad=.35, enforce_model_position_limits=True):
        self.dynamics = dynamics
        self.enforce_model_position_limits = bool(enforce_model_position_limits)
        self.stiffness, self.damping = gains(stiffness, "stiffness"), gains(damping, "damping")
        self.joint_damping = np.broadcast_to(np.asarray(joint_damping, float), (14,)).copy()
        if not np.isfinite(self.joint_damping).all() or np.any(self.joint_damping < 0):
            raise ValueError("Joint damping must be nonnegative and finite")
        for name, value in (("nullspace_stiffness", nullspace_stiffness), ("nullspace_damping", nullspace_damping)):
            if not np.isfinite(value) or value < 0:
                raise ValueError(name+" must be nonnegative and finite")
        self.nullspace_stiffness, self.nullspace_damping = nullspace_stiffness, nullspace_damping
        self.q_rest = None if q_rest is None else arm_vector(q_rest, "q_rest")
        settings = (max_force_N, max_moment_Nm, max_position_error_m, max_orientation_error_rad)
        if any(not np.isfinite(x) or x <= 0 for x in settings):
            raise ValueError("Positive finite wrench/error limits required")
        self.max_force_N, self.max_moment_Nm, self.max_position_error_m, self.max_orientation_error_rad = settings

    def compute(self, q_current, dq_current, targets, *, desired_twist_torso=None, gravity_torque_Nm=None):
        q, dq = arm_vector(q_current, "q_current"), arm_vector(dq_current, "dq_current")
        if set(targets) != set(SIDES):
            raise ValueError("Both left and right target poses are required")
        poses = {side: rigid(targets[side]) for side in SIDES}
        twist = np.zeros((2, 6)) if desired_twist_torso is None else np.asarray(desired_twist_torso, float)
        if twist.shape != (2, 6) or not np.isfinite(twist).all():
            raise ValueError("desired_twist_torso requires finite 2x6 linear/angular velocities")
        rest = q if self.q_rest is None else self.q_rest
        mass, _ = self.dynamics.terms(q, dq)
        gravity = (self.dynamics.gravity(q) if gravity_torque_Nm is None
                   else arm_vector(gravity_torque_Nm, "gravity_torque_Nm"))
        task, null = np.zeros(14), np.zeros(14)
        wrenches, errors, saturation = [], [], []
        reasons = []
        if self.enforce_model_position_limits and (np.any(q < self.dynamics.lower) or np.any(q > self.dynamics.upper)):
            reasons.append("joint_position_outside_urdf_limits")
        for i, side in enumerate(SIDES):
            s = slice(i*7, (i+1)*7)
            current = self.dynamics.pose(q, side)
            jac = self.dynamics.jacobian(q, side)
            rotation = poses[side][:3, :3]
            transform = np.zeros((6, 6))
            transform[:3, :3] = transform[3:, 3:] = rotation
            error_torso = np.r_[poses[side][:3, 3]-current[:3, 3],
                                Rotation.from_matrix(rotation @ current[:3, :3].T).as_rotvec()]
            error = transform.T @ error_torso
            if np.linalg.norm(error[:3]) > self.max_position_error_m or np.linalg.norm(error[3:]) > self.max_orientation_error_rad:
                reasons.append(side+"_target_error_exceeds_configured_bound")
            wrench = self.stiffness[i]*error+self.damping[i]*(transform.T @ (twist[i]-jac @ dq[s]))
            clipped = False
            for part, limit in ((slice(0, 3), self.max_force_N), (slice(3, 6), self.max_moment_Nm)):
                norm = np.linalg.norm(wrench[part])
                if norm > limit:
                    wrench[part] *= limit/norm
                    clipped = True
            task[s] = jac.T @ (transform @ wrench)
            # Dynamically consistent torque projector. Main task mapping does
            # not invert J. SVD pseudoinverse is only for redundant posture.
            minv_jt = np.linalg.solve(mass[s, s], jac.T)
            jbar = minv_jt @ np.linalg.pinv(jac @ minv_jt, rcond=1e-8)
            projector = np.eye(7)-jac.T @ jbar.T
            null[s] = projector @ (self.nullspace_stiffness*(rest[s]-q[s])-self.nullspace_damping*dq[s])
            wrenches.append(wrench); errors.append(error); saturation.append(clipped)
        desired_total = task+null+gravity-self.joint_damping*dq
        # Limits apply to TOTAL motor torque, including gravity and damping.
        total = np.clip(desired_total, -self.dynamics.effort_limits, self.dynamics.effort_limits)
        joint_saturation = np.abs(total-desired_total) > 1e-10
        if any(saturation):
            reasons.append("task_wrench_limited")
        if np.any(joint_saturation):
            reasons.append("total_joint_torque_limited")
        extra = total+self.joint_damping*dq
        if self.q_rest is None:
            self.q_rest = q.copy()
        return ImpedanceCommand(q, np.zeros(14), self.joint_damping.copy(), extra, total, task, null,
                                gravity.copy(), np.array(wrenches), np.array(errors), np.array(saturation),
                                joint_saturation, not reasons, tuple(reasons))
