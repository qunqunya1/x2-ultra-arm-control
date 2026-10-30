"""Virtual flange sensor: change wrench origin, rotate axes, declare sign.

The underlying estimator stays in torso axes during temporal filtering. Each
output is then expressed in the CURRENT sensor frame, avoiding a moving-frame
low-pass filter that mixes historical axes. No force truth is an input.
The quantity is the equivalent external load relative to the dynamics model.
Unmodeled tool weight/inertia can appear in that load. This module neither
synthesizes nor independently removes a downstream tool support baseline.
"""
from dataclasses import replace

import numpy as np

from .core import rigid
from .dual_wrench import DualArmWrenchEstimator

SIDES = ("left", "right")
CONVENTIONS = ("environment_on_robot", "support_delta")


def wrench_in_sensor_frame(wrench_torso, torso_from_point, torso_from_sensor):
    """Wrench at P, in torso axes -> wrench at sensor origin S, in S axes.

    M_S = R_TS.T (M_P - (p_S - p_P) x F). T_A_B maps B coordinates to A.
    """
    w = np.asarray(wrench_torso, dtype=float)
    if w.shape != (6,) or not np.isfinite(w).all():
        raise ValueError("A finite six-component wrench is required")
    point, sensor = rigid(torso_from_point), rigid(torso_from_sensor)
    displacement = sensor[:3, 3]-point[:3, 3]
    rotation = sensor[:3, :3].T
    return np.r_[rotation @ w[:3], rotation @ (w[3:]-np.cross(displacement, w[:3]))]


class VirtualFlangeSensorEstimator:
    """Streaming joint telemetry -> left/right local virtual flange wrenches.

    Explicit wrist_from_sensor transforms are required for BOTH arms. An
    identity guess must remain calibration_confirmed=False. 'support_delta'
    reverses the residual load for a support-sensor interface; it does not
    synthesize or compensate a raw tool gravity/inertia baseline or a tare.
    """
    def __init__(self, dynamics=None, *, wrist_from_sensor,
                 sensor_convention="environment_on_robot", calibration_confirmed=False,
                 **estimator_settings):
        if set(wrist_from_sensor) != set(SIDES):
            raise ValueError("Explicit left and right wrist_from_sensor transforms required")
        if sensor_convention not in CONVENTIONS:
            raise ValueError("Unknown sensor convention")
        self.wrist_from_sensor = {side: rigid(wrist_from_sensor[side]) for side in SIDES}
        self.sensor_convention = sensor_convention
        self.calibration_confirmed = bool(calibration_confirmed)
        self.estimator = DualArmWrenchEstimator(dynamics, **estimator_settings)
        self.dynamics = self.estimator.dynamics
        # pose()/jacobian() can already refer to a configured TCP rather than
        # the wrist. Account for that offset exactly once, including rotation.
        self.point_from_sensor = {
            side: np.linalg.inv(rigid(self.dynamics.tip_from_tcp[side])) @ self.wrist_from_sensor[side]
            for side in SIDES}

    def reset(self):
        self.estimator.reset()

    def update(self, packet):
        result = self.estimator.update(packet)
        if result is None:
            return None
        q = self.estimator.previous_packet.q_rad
        converted, raw = [], []
        sign = -1. if self.sensor_convention == "support_delta" else 1.
        for i, side in enumerate(SIDES):
            point = self.dynamics.pose(q, side)
            sensor = point @ self.point_from_sensor[side]
            converted.append(sign*wrench_in_sensor_frame(result.wrench[i], point, sensor))
            raw.append(sign*wrench_in_sensor_frame(result.raw_wrench[i], point, sensor))
        metadata = dict(axes="each arm's moving sensor axes", origin="configured virtual sensor origin",
                        wrist_frames={s: s+"_wrist_roll_link" for s in SIDES},
                        wrist_from_sensor={s: t.tolist() for s, t in self.wrist_from_sensor.items()},
                        calibration_confirmed=self.calibration_confirmed,
                        sensor_convention=self.sensor_convention,
                        quantity="model-residual equivalent external load",
                        orientation_timestamp_s=result.available_at_s,
                        filtering="torso axes before current-pose origin/axes conversion",
                        tool_weight_inertia="depends on downstream inertia included in dynamics; no extra compensation or synthesis",
                        policy_observation_compatibility_verified=False)
        convention = ("environment acting on robot" if sign > 0 else
                      "negative model-residual external load (support-reaction delta)")
        return replace(result, wrench=np.array(converted), raw_wrench=np.array(raw),
                       frame="sensor-local axes, configured flange sensor origin",
                       convention=convention, frame_metadata=metadata)
