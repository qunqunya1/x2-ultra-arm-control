"""URDF FK, geometric Jacobian [linear; angular], and local bounded pose IK.

Units: metres, radians. Matrices T_parent_child map child coordinates into
parent coordinates. All returned poses/Jacobians are in torso_link by default.
Only the seven selected arm joints move; waist/legs are outside this solver.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def vector(value, length, label):
    a = np.asarray(value, dtype=float)
    if a.shape != (length,) or not np.isfinite(a).all():
        raise ValueError(f"{label} must be a finite vector of length {length}")
    return a


def rigid(value):
    t = np.array(value, dtype=float, copy=True)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError("Pose must be a finite 4x4 matrix")
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
        raise ValueError("Invalid homogeneous row")
    r = t[:3, :3]
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-7, rtol=0) or abs(np.linalg.det(r)-1) > 1e-7:
        raise ValueError("Pose rotation must be SO(3)")
    return t


def pose_matrix(position_m, quaternion_wxyz):
    p = vector(position_m, 3, "position_m")
    q = vector(quaternion_wxyz, 4, "quaternion_wxyz")
    if not np.isclose(np.linalg.norm(q), 1, atol=1e-6, rtol=0):
        raise ValueError("Quaternion must have unit norm; convention is [w,x,y,z]")
    t = np.eye(4)
    t[:3, :3] = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    t[:3, 3] = p
    return t


def pose_error(target, current):
    return np.r_[target[:3, 3]-current[:3, 3],
                 Rotation.from_matrix(target[:3, :3] @ current[:3, :3].T).as_rotvec()]


def cartesian_target(current, translation_m, rotation_vector_rad=(0, 0, 0), frame="base"):
    """Increment position and orientation. 'base' means the solver's base.

    In base frame: p'=p+dp, R'=Exp(dr)R. In TCP frame: p'=p+Rdp,
    R'=R Exp(dr). Rotation inputs are rotation vectors, not Euler angles.
    """
    t = rigid(current)
    dp = vector(translation_m, 3, "translation_m")
    dr = vector(rotation_vector_rad, 3, "rotation_vector_rad")
    inc = Rotation.from_rotvec(dr).as_matrix()
    if frame == "base":
        t[:3, 3] += dp
        t[:3, :3] = inc @ t[:3, :3]
    elif frame == "tcp":
        t[:3, 3] += t[:3, :3] @ dp
        t[:3, :3] = t[:3, :3] @ inc
    else:
        raise ValueError("frame must be 'base' or 'tcp'")
    return t


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float


@dataclass(frozen=True)
class IKOptions:
    position_tolerance_m: float = 1e-5
    orientation_tolerance_rad: float = 1e-4
    orientation_length_m: float = 0.15
    damping: float = 0.002
    max_iteration_step_rad: float = 0.12
    max_seed_change_rad: float = 0.4
    joint_margin_rad: float = 0.0
    max_iterations: int = 200

    def validate(self):
        for name in ("position_tolerance_m", "orientation_tolerance_rad", "orientation_length_m",
                     "damping", "max_iteration_step_rad", "max_seed_change_rad"):
            value = getattr(self, name)
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not np.isfinite(self.joint_margin_rad) or self.joint_margin_rad < 0:
            raise ValueError("joint_margin_rad must be finite and nonnegative")
        if not isinstance(self.max_iterations, int) or isinstance(self.max_iterations, bool) or self.max_iterations < 1:
            raise ValueError("max_iterations must be a positive integer")


@dataclass(frozen=True)
class IKResult:
    success: bool
    reason: str
    q_rad: np.ndarray
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    max_seed_change_rad: float

    def as_dict(self):
        return dict(success=self.success, reason=self.reason, q_rad=self.q_rad.tolist(),
                    position_error_m=self.position_error_m,
                    orientation_error_rad=self.orientation_error_rad,
                    iterations=self.iterations, max_seed_change_rad=self.max_seed_change_rad)


class ArmKinematics:
    def __init__(self, urdf_path, side, tip_from_tcp=None, base_link="torso_link"):
        if side not in ("left", "right"):
            raise ValueError("side must be left or right")
        self.side = side
        self.base_link = base_link
        self.tip_link = f"{side}_wrist_roll_link"
        self.urdf_path = Path(urdf_path).resolve()
        self.tip_from_tcp = rigid(np.eye(4) if tip_from_tcp is None else tip_from_tcp)
        self.point_label = self.tip_link if tip_from_tcp is None else "configured_tcp"
        root = ET.parse(self.urdf_path).getroot()
        links = {link.attrib["name"] for link in root.findall("link")}
        if base_link not in links or self.tip_link not in links:
            raise ValueError("Requested base/tip link is absent from URDF")
        by_child = {}
        for joint in root.findall("joint"):
            child = joint.find("child").attrib["link"]
            if child in by_child:
                raise ValueError("URDF child link has multiple parents")
            by_child[child] = joint
        chain = []
        link = self.tip_link
        seen = set()
        while link != base_link:
            if link in seen or link not in by_child:
                raise ValueError("No acyclic URDF path from selected base to arm tip")
            seen.add(link)
            joint = by_child[link]
            kind = joint.attrib["type"]
            if kind not in ("revolute", "fixed"):
                raise ValueError(f"Unsupported joint type in arm chain: {kind}")
            origin = joint.find("origin")
            xyz = np.fromstring(origin.get("xyz", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            rpy = np.fromstring(origin.get("rpy", "0 0 0") if origin is not None else "0 0 0", sep=" ")
            t = np.eye(4)
            t[:3, 3] = vector(xyz, 3, "URDF origin xyz")
            t[:3, :3] = Rotation.from_euler("xyz", vector(rpy, 3, "URDF origin rpy")).as_matrix()
            axis = np.zeros(3)
            lower = upper = 0.0
            if kind == "revolute":
                element = joint.find("axis")
                axis = vector(np.fromstring(element.get("xyz", "1 0 0") if element is not None else "1 0 0", sep=" "), 3, "URDF axis")
                if np.linalg.norm(axis) < 1e-12:
                    raise ValueError("Revolute joint axis cannot be zero")
                axis = axis / np.linalg.norm(axis)
                limit = joint.find("limit")
                if limit is None:
                    raise ValueError("Revolute joint must have limits")
                lower, upper = float(limit.attrib["lower"]), float(limit.attrib["upper"])
                if not np.isfinite([lower, upper]).all() or lower >= upper:
                    raise ValueError("Invalid URDF joint limits")
            chain.append(Joint(joint.attrib["name"], kind, t, axis, lower, upper))
            link = joint.find("parent").attrib["link"]
        self.chain = tuple(reversed(chain))
        active = [j for j in self.chain if j.kind == "revolute"]
        expected = tuple(f"{side}_{name}_joint" for name in
                         ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow", "wrist_yaw", "wrist_pitch", "wrist_roll"))
        self.joint_names = tuple(j.name for j in active)
        if self.joint_names != expected:
            raise ValueError("Expected exactly seven X2 arm joints; do not include the waist in this arm solver")
        self.lower = np.array([j.lower for j in active])
        self.upper = np.array([j.upper for j in active])

    def validate_q(self, q):
        q = vector(q, 7, "q_rad")
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError("Joint angles outside URDF limits")
        return q.copy()

    def _walk(self, q):
        q = self.validate_q(q)
        t = np.eye(4)
        origins, axes = [], []
        k = 0
        for joint in self.chain:
            t = t @ joint.origin
            if joint.kind == "revolute":
                origins.append(t[:3, 3].copy())
                axes.append(t[:3, :3] @ joint.axis)
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(joint.axis*q[k]).as_matrix()
                t = t @ rotation
                k += 1
        return t @ self.tip_from_tcp, np.array(origins), np.array(axes)

    def fk(self, q):
        """T_base_tcp; with no tool transform this is T_torso_wrist_roll."""
        return self._walk(q)[0]

    def jacobian(self, q):
        """6x7 geometric Jacobian at the configured point, rows [v; omega]."""
        t, origins, axes = self._walk(q)
        return np.vstack((np.cross(axes, t[:3, 3]-origins).T, axes.T))

    def ik(self, target, seed, options=None):
        """Damped least squares, local seed bounds, joint limits, backtracking.

        Failure returns a diagnostic candidate, not an executable target. Failure
        to converge locally does not prove that the pose is globally unreachable.
        Numerical tolerances/steps are not real robot accuracy or rate guarantees.
        """
        target = rigid(target)
        seed = self.validate_q(seed)
        opts = options or IKOptions()
        opts.validate()
        low = np.maximum(self.lower+opts.joint_margin_rad, seed-opts.max_seed_change_rad)
        high = np.minimum(self.upper-opts.joint_margin_rad, seed+opts.max_seed_change_rad)
        if np.any(low > high) or np.any(seed < low) or np.any(seed > high):
            raise ValueError("Seed violates the requested joint margin")
        q = seed.copy()
        weight = np.array([1., 1., 1., *([opts.orientation_length_m]*3)])

        def finish(success, reason, iterations):
            e = pose_error(target, self.fk(q))
            return IKResult(success, reason, q.copy(), float(np.linalg.norm(e[:3])),
                            float(np.linalg.norm(e[3:])), iterations, float(np.max(np.abs(q-seed))))

        for iteration in range(opts.max_iterations):
            error = pose_error(target, self.fk(q))
            if np.linalg.norm(error[:3]) <= opts.position_tolerance_m and np.linalg.norm(error[3:]) <= opts.orientation_tolerance_rad:
                return finish(True, "converged", iteration)
            jac = weight[:, None] * self.jacobian(q)
            residual = weight * error
            cost = float(residual @ residual)
            accepted = False
            # Retry a more damped local direction if clipping/curvature rejects it.
            for multiplier in (1., 10., 100.):
                damping = opts.damping*multiplier
                dq = jac.T @ np.linalg.solve(jac @ jac.T+damping*damping*np.eye(6), residual)
                largest = np.max(np.abs(dq))
                if largest > opts.max_iteration_step_rad:
                    dq *= opts.max_iteration_step_rad/largest
                for alpha in (1., .5, .25, .125, .0625, .03125, .015625, .0078125):
                    candidate = np.clip(q+alpha*dq, low, high)
                    next_error = weight*pose_error(target, self.fk(candidate))
                    if float(next_error @ next_error) < cost-1e-16:
                        q = candidate
                        accepted = True
                        break
                if accepted:
                    break
            if not accepted:
                return finish(False, "local_limit_or_stagnation", iteration+1)
        result = finish(False, "iteration_limit", opts.max_iterations)
        if result.position_error_m <= opts.position_tolerance_m and result.orientation_error_rad <= opts.orientation_tolerance_rad:
            return finish(True, "converged", opts.max_iterations)
        return result


def solve_dual_arm(arms, targets, seeds, options=None):
    """All-or-nothing numerical acceptance; no collision/whole-body checks."""
    for packet in (arms, targets, seeds):
        if set(packet) != {"left", "right"}:
            raise ValueError("Both left and right arms are required")
    if any(arms[side].side != side for side in ("left", "right")):
        raise ValueError("Arm side mapping mismatch")
    if arms["left"].base_link != arms["right"].base_link:
        raise ValueError("Both targets must share the same base frame")
    results = {side: arms[side].ik(targets[side], seeds[side], options) for side in ("left", "right")}
    accepted = all(result.success for result in results.values())
    return dict(success=accepted, results=results,
                accepted_q_rad={side: result.q_rad.copy() for side, result in results.items()} if accepted else None)
