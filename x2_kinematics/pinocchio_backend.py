"""Pinocchio model backend for the X2 Ultra arms, no simulator data access.

Public vectors are left seven joints followed by right seven. Non-arm joints
are locked at a reference configuration. Poses/Jacobians use torso axes.
"""
from pathlib import Path

import numpy as np
import pinocchio as pin

from .core import rigid
from .model_config import selected_urdf

ROOT = Path(__file__).resolve().parents[1]
ARM_SUFFIXES = ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_yaw", "wrist_pitch", "wrist_roll")
ARM_NAMES = tuple(f"{side}_{joint}_joint" for side in ("left", "right") for joint in ARM_SUFFIXES)


def arm_vector(value, label):
    a = np.asarray(value, dtype=float)
    if a.shape != (14,) or not np.isfinite(a).all():
        raise ValueError(f"{label} must contain fourteen finite values")
    return a.copy()


class PinocchioDynamics:
    def __init__(self, urdf=None, *, waist_reference_rad=(0., 0., 0.), rotor_inertia=0., viscous_damping=0.,
                 tip_from_tcp=None):
        self.urdf = Path(urdf).resolve() if urdf is not None else selected_urdf()
        # XML loading avoids Windows native narrow-path limitations.
        full = pin.buildModelFromXML(self.urdf.read_text(encoding="utf-8"))
        reference = pin.neutral(full)
        waist = np.asarray(waist_reference_rad, dtype=float)
        if waist.shape != (3,) or not np.isfinite(waist).all():
            raise ValueError("Three finite waist reference angles required")
        for name, angle in zip(("waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint"), waist):
            reference[full.joints[full.getJointId(name)].idx_q] = angle
        to_lock = [i for i in range(1, full.njoints) if full.names[i] not in ARM_NAMES]
        self.model = pin.buildReducedModel(full, to_lock, reference)
        if self.model.nq != 14 or self.model.nv != 14:
            raise ValueError("Expected fourteen one-DoF arm joints after reduction")
        self.joint_names = ARM_NAMES
        joint_ids = [self.model.getJointId(name) for name in ARM_NAMES]
        if any(i == 0 or i >= self.model.njoints for i in joint_ids):
            raise ValueError("Required X2 arm joint is missing")
        self.q_indices = np.array([self.model.joints[i].idx_q for i in joint_ids])
        self.v_indices = np.array([self.model.joints[i].idx_v for i in joint_ids])
        self.lower = np.asarray(self.model.lowerPositionLimit)[self.q_indices].copy()
        self.upper = np.asarray(self.model.upperPositionLimit)[self.q_indices].copy()
        self.effort_limits = np.asarray(self.model.effortLimit)[self.v_indices].copy()
        self.data = self.model.createData()
        self.gravity_data = self.model.createData()
        self.frame_data = self.model.createData()
        self.torso_frame = self.model.getFrameId("torso_link")
        self.tip_frames = {side: self.model.getFrameId(side+"_wrist_roll_link") for side in ("left", "right")}
        if any(i >= self.model.nframes for i in [self.torso_frame, *self.tip_frames.values()]):
            raise ValueError("Required torso/wrist frame is absent")
        self.rotor_inertia = np.broadcast_to(np.asarray(rotor_inertia, float), (14,)).copy()
        self.viscous_damping = np.broadcast_to(np.asarray(viscous_damping, float), (14,)).copy()
        if (not np.isfinite(self.rotor_inertia).all() or not np.isfinite(self.viscous_damping).all()
                or np.any(self.rotor_inertia < 0) or np.any(self.viscous_damping < 0)):
            raise ValueError("Rotor inertia and viscous damping must be finite and nonnegative")
        self.tip_from_tcp = {side: rigid((tip_from_tcp or {}).get(side, np.eye(4))) for side in ("left", "right")}
        self.waist_reference_rad = waist.copy()

    def _q(self, q):
        result = pin.neutral(self.model)
        result[self.q_indices] = arm_vector(q, "q")
        return result

    def _v(self, dq):
        result = np.zeros(self.model.nv)
        result[self.v_indices] = arm_vector(dq, "dq")
        return result

    def gravity(self, q):
        g = pin.computeGeneralizedGravity(self.model, self.gravity_data, self._q(q))
        return np.asarray(g)[self.v_indices].copy()

    def terms(self, q, dq):
        q_model, v_model = self._q(q), self._v(dq)
        m = np.asarray(pin.crba(self.model, self.data, q_model)).copy()
        m = np.triu(m)+np.triu(m, 1).T
        mass = m[np.ix_(self.v_indices, self.v_indices)]
        mass += np.diag(self.rotor_inertia)
        nonlinear = np.asarray(pin.nonLinearEffects(self.model, self.data, q_model, v_model))[self.v_indices].copy()
        # Explicit identified/synthetic motor terms, not supplied by URDF RNEA.
        return mass, nonlinear+self.viscous_damping*arm_vector(dq, "dq")

    def inverse_dynamics(self, q, dq, ddq):
        acceleration = self._v(ddq)
        tau = np.asarray(pin.rnea(self.model, self.data, self._q(q), self._v(dq), acceleration))[self.v_indices].copy()
        return tau+self.rotor_inertia*arm_vector(ddq, "ddq")+self.viscous_damping*arm_vector(dq, "dq")

    def pose(self, q, side):
        if side not in self.tip_frames:
            raise ValueError("Unknown arm side")
        pin.framesForwardKinematics(self.model, self.frame_data, self._q(q))
        torso = self.frame_data.oMf[self.torso_frame]
        wrist = self.frame_data.oMf[self.tip_frames[side]]
        return (torso.inverse()*wrist).homogeneous.copy() @ self.tip_from_tcp[side]

    def jacobian(self, q, side):
        if side not in self.tip_frames:
            raise ValueError("Unknown arm side")
        q_model = self._q(q)
        pin.framesForwardKinematics(self.model, self.frame_data, q_model)
        j = np.asarray(pin.computeFrameJacobian(self.model, self.frame_data, q_model, self.tip_frames[side],
                                               pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)).copy()
        torso_rotation = self.frame_data.oMf[self.torso_frame].rotation
        wrist_rotation = self.frame_data.oMf[self.tip_frames[side]].rotation
        displacement = wrist_rotation @ self.tip_from_tcp[side][:3, 3]
        # Shift the linear-velocity reference point from wrist to configured TCP.
        j[:3] += np.cross(j[3:].T, displacement).T
        j[:3] = torso_rotation.T @ j[:3]
        j[3:] = torso_rotation.T @ j[3:]
        selected = self.v_indices[:7] if side == "left" else self.v_indices[7:]
        return j[:, selected]

    def metadata(self):
        return dict(library="Pinocchio", version=pin.__version__, urdf=str(self.urdf),
                    public_joint_order=list(self.joint_names), reduced_model_joint_order=list(self.model.names),
                    locked_waist_reference_rad=self.waist_reference_rad.tolist(),
                    rotor_inertia_kg_m2=self.rotor_inertia.tolist(), viscous_damping_Nm_s_rad=self.viscous_damping.tolist(),
                    frame="torso_link axes, configured wrist/TCP point", simulator_data_access=False)
