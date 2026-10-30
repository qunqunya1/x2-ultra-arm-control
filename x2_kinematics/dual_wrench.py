"""Streaming dual-arm wrench estimation from timestamped joint telemetry.

Inputs contain joint state and actuator torque only; no acceleration, external
load, simulator data, or desired wrench is accepted by the estimator.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .pinocchio_backend import PinocchioDynamics, arm_vector
from .wrench import LowPass, estimate_wrench, impedance_torque


@dataclass(frozen=True)
class JointFeedback:
    timestamp_s: float
    q_rad: np.ndarray
    dq_rad_s: np.ndarray
    actuator_torque_Nm: np.ndarray
    torque_source: str = "measured"
    saturated: tuple = (False, False)

    def validated(self):
        if not np.isfinite(self.timestamp_s):
            raise ValueError("Finite timestamp required")
        if self.torque_source not in ("measured", "reconstructed"):
            raise ValueError("Unknown torque source")
        if len(self.saturated) != 2:
            raise ValueError("One saturation flag per arm required")
        return JointFeedback(float(self.timestamp_s), arm_vector(self.q_rad, "q_rad"),
                             arm_vector(self.dq_rad_s, "dq_rad_s"),
                             arm_vector(self.actuator_torque_Nm, "actuator_torque_Nm"),
                             self.torque_source, tuple(bool(x) for x in self.saturated))


def feedback_from_impedance(timestamp_s, q, dq, q_target, kp, kd, torque_extra, saturated=(False, False)):
    """Explicit fallback when actual actuator torque is unavailable.

    These are reconstructed requests, not independent torque measurements.
    Firmware gains, velocity semantics, filtering and saturation must match.
    """
    q, dq, target = arm_vector(q, "q"), arm_vector(dq, "dq"), arm_vector(q_target, "q_target")
    kp, kd, extra = arm_vector(kp, "kp"), arm_vector(kd, "kd"), arm_vector(torque_extra, "torque_extra")
    tau = np.concatenate([impedance_torque(q[s], dq[s], target[s], kp[s], kd[s], extra[s])
                          for s in (slice(0, 7), slice(7, 14))])
    return JointFeedback(timestamp_s, q, dq, tau, "reconstructed", saturated).validated()


@dataclass(frozen=True)
class DualWrenchResult:
    available_at_s: float
    window_start_s: float
    window_end_s: float
    window_samples: int
    wrench: np.ndarray
    raw_wrench: np.ndarray
    valid: np.ndarray
    reasons: tuple
    scaled_condition: np.ndarray
    fit_residual_Nm: np.ndarray
    torque_source: str
    frame: str = "torso_link axes, backend-configured wrist/TCP point"
    convention: str = "environment acting on robot"
    frame_metadata: dict | None = None

    def as_dict(self):
        arms = {}
        for i, side in enumerate(("left", "right")):
            arms[side] = dict(wrench=self.wrench[i].tolist(), raw_wrench=self.raw_wrench[i].tolist(),
                              valid=bool(self.valid[i]), reasons=list(self.reasons[i]),
                              scaled_condition=float(self.scaled_condition[i]),
                              joint_fit_residual_Nm=float(self.fit_residual_Nm[i]))
        return dict(available_at_s=self.available_at_s, window_start_s=self.window_start_s,
                    window_end_s=self.window_end_s, window_samples=self.window_samples,
                    frame=self.frame, frame_metadata=self.frame_metadata,
                    components=["Fx", "Fy", "Fz", "Mx", "My", "Mz"],
                    convention=self.convention, torque_source=self.torque_source,
                    arms=arms, robot_commands_sent=False)


class DualArmWrenchEstimator:
    def __init__(self, dynamics=None, *, window_s=.005, max_sample_gap_s=.004,
                 output_cutoff_hz=10., characteristic_length_m=.1, damping_m=.0001, max_condition=100.):
        if any(not np.isfinite(v) or v <= 0 for v in (window_s, max_sample_gap_s, output_cutoff_hz,
                                                     characteristic_length_m, damping_m, max_condition)):
            raise ValueError("Positive finite estimator settings required")
        if window_s > .1 or max_sample_gap_s > .1:
            raise ValueError("Estimator windows and sample gaps must be <=0.1s")
        self.dynamics = dynamics or PinocchioDynamics()
        self.window_s, self.max_sample_gap_s = window_s, max_sample_gap_s
        self.output_cutoff_hz = output_cutoff_hz
        self.characteristic_length_m, self.damping_m = characteristic_length_m, damping_m
        self.max_condition = max_condition
        self.reset()

    def reset(self):
        self.previous_packet = None
        self.start_packet = None
        self.start_mass = self.start_bias = None
        self.torque_integral = np.zeros(14)
        self.saturation_in_window = np.zeros(2, dtype=bool)
        self.samples_in_window = 0
        self.filters = [LowPass(self.output_cutoff_hz), LowPass(self.output_cutoff_hz)]

    def _start(self, packet):
        self.start_mass, self.start_bias = self.dynamics.terms(packet.q_rad, packet.dq_rad_s)
        self.start_packet = self.previous_packet = packet
        self.saturation_in_window[:] = packet.saturated

    def update(self, packet):
        try:
            packet = packet.validated()
        except ValueError:
            self.reset()
            raise
        if self.previous_packet is None:
            self._start(packet)
            return None
        dt_sample = packet.timestamp_s-self.previous_packet.timestamp_s
        if dt_sample <= 0 or dt_sample > self.max_sample_gap_s+1e-12:
            self.reset()
            raise ValueError("Stale or nonmonotonic telemetry: estimator reset; fresh warm-up required")
        if packet.torque_source != self.previous_packet.torque_source:
            self.reset()
            raise ValueError("Torque source changed: estimator reset")
        # Causal left-rectangle integration of previously observed actuator
        # torque over exactly the same interval as the velocity difference.
        self.torque_integral += self.previous_packet.actuator_torque_Nm*dt_sample
        self.saturation_in_window |= np.asarray(packet.saturated, bool)
        self.samples_in_window += 1
        window = packet.timestamp_s-self.start_packet.timestamp_s
        self.previous_packet = packet
        if window+1e-12 < self.window_s:
            return None
        mass, bias = self.dynamics.terms(packet.q_rad, packet.dq_rad_s)
        required = .5*(self.start_mass+mass) @ ((packet.dq_rad_s-self.start_packet.dq_rad_s)/window)+.5*(self.start_bias+bias)
        residual = required-self.torque_integral/window
        raw, filtered, validity, reasons, conditions, fits = [], [], [], [], [], []
        for i, side in enumerate(("left", "right")):
            result = estimate_wrench(self.dynamics.jacobian(packet.q_rad, side), residual[i*7:(i+1)*7],
                                     torque_source=packet.torque_source, characteristic_length_m=self.characteristic_length_m,
                                     damping_m=self.damping_m, max_condition=self.max_condition,
                                     torque_saturated=bool(self.saturation_in_window[i]))
            raw.append(result.wrench)
            filtered.append(self.filters[i].update(result.wrench, window))
            validity.append(result.valid); reasons.append(result.reasons)
            conditions.append(result.scaled_condition); fits.append(result.joint_fit_residual_Nm)
        output = DualWrenchResult(packet.timestamp_s, self.start_packet.timestamp_s, packet.timestamp_s,
                                  self.samples_in_window, np.array(filtered), np.array(raw), np.array(validity),
                                  tuple(reasons), np.array(conditions), np.array(fits), packet.torque_source)
        self.start_packet, self.start_mass, self.start_bias = packet, mass, bias
        self.torque_integral[:] = 0.; self.samples_in_window = 0
        self.saturation_in_window[:] = packet.saturated
        return output
