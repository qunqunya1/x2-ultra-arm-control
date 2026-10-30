"""Full URDF computation with current waist pose, fixed pelvis assumption.

Arm M/h/J are selected by name. Waist motion is gated by the runtime because
waist/base accelerations are not estimated by this arm-only observer.
"""
import numpy as np
import pinocchio as pin

from ..core import rigid
from ..model_config import selected_urdf
from ..pinocchio_backend import ARM_NAMES, arm_vector
from .protocol import WAIST_NAMES


class LiveWaistArmDynamics:
    def __init__(self, *, tip_from_tcp=None, base_gravity=(0, 0, -9.81)):
        self.urdf = selected_urdf()
        self.model = pin.buildModelFromXML(self.urdf.read_text(encoding="utf-8"))
        self.set_base_gravity(base_gravity)
        self.data, self.pose_data = self.model.createData(), self.model.createData()
        self.q_neutral = pin.neutral(self.model)
        self.qi = np.array([self.model.joints[self.model.getJointId(n)].idx_q for n in ARM_NAMES])
        self.vi = np.array([self.model.joints[self.model.getJointId(n)].idx_v for n in ARM_NAMES])
        self.waist_qi = np.array([self.model.joints[self.model.getJointId(n)].idx_q for n in WAIST_NAMES])
        if any(self.model.getJointId(n) == 0 for n in (*ARM_NAMES, *WAIST_NAMES)):
            raise ValueError("Selected URDF joint names do not match X2")
        self.waist = np.zeros(3)
        self.lower = np.asarray(self.model.lowerPositionLimit)[self.qi].copy()
        self.upper = np.asarray(self.model.upperPositionLimit)[self.qi].copy()
        self.effort_limits = np.asarray(self.model.effortLimit)[self.vi].copy()
        self.torso = self.model.getFrameId("torso_link")
        self.tip = {s: self.model.getFrameId(s+"_wrist_roll_link") for s in ("left", "right")}
        self.tip_from_tcp = {s: rigid((tip_from_tcp or {}).get(s, np.eye(4))) for s in self.tip}

    def set_base_gravity(self, value):
        gravity=np.asarray(value,float)
        if gravity.shape != (3,) or not np.isfinite(gravity).all() or not 9.7 <= np.linalg.norm(gravity) <= 9.9:
            raise ValueError("Expected a finite gravity vector in model-base axes, m/s^2")
        self.model.gravity.linear=gravity

    def nominal_pelvis_imu_rotation(self):
        frame=self.model.getFrameId('imu_in_pelvis_link')
        if frame >= self.model.nframes:
            raise ValueError("Selected model has no pelvis IMU frame")
        pin.framesForwardKinematics(self.model,self.pose_data,self.q_neutral)
        return self.pose_data.oMf[frame].rotation.copy()

    def set_waist(self, q):
        value = np.asarray(q, float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError("Invalid waist angles")
        # FK/dynamics remain defined beyond the URDF planning limits. The
        # runtime reports/excludes control there; observe may still preview.
        self.waist = value.copy()

    def _q(self, q):
        result = self.q_neutral.copy()
        result[self.qi], result[self.waist_qi] = arm_vector(q, "q"), self.waist
        return result

    def gravity(self, q):
        return np.asarray(pin.computeGeneralizedGravity(self.model, self.data, self._q(q)))[self.vi].copy()

    def terms(self, q, dq):
        velocity = np.zeros(self.model.nv)
        velocity[self.vi] = arm_vector(dq, "dq")
        m = np.asarray(pin.crba(self.model, self.data, self._q(q))).copy()
        m = np.triu(m)+np.triu(m, 1).T
        h = np.asarray(pin.nonLinearEffects(self.model, self.data, self._q(q), velocity))[self.vi].copy()
        return m[np.ix_(self.vi, self.vi)], h

    def pose(self, q, side):
        pin.framesForwardKinematics(self.model, self.pose_data, self._q(q))
        return (self.pose_data.oMf[self.torso].inverse()*self.pose_data.oMf[self.tip[side]]).homogeneous.copy() @ self.tip_from_tcp[side]

    def jacobian(self, q, side):
        pin.framesForwardKinematics(self.model, self.pose_data, self._q(q))
        j = np.asarray(pin.computeFrameJacobian(self.model, self.pose_data, self._q(q), self.tip[side],
                                               pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)).copy()
        torso_r = self.pose_data.oMf[self.torso].rotation
        wrist_r = self.pose_data.oMf[self.tip[side]].rotation
        r = wrist_r @ self.tip_from_tcp[side][:3, 3]
        j[:3] += np.cross(j[3:].T, r).T
        j[:3], j[3:] = torso_r.T @ j[:3], torso_r.T @ j[3:]
        indices = self.vi[:7] if side == "left" else self.vi[7:]
        return j[:, indices]
