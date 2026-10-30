"""Documented AimDK ROS message conversion; no ROS dependency or network."""
from dataclasses import dataclass
import numpy as np

from ..pinocchio_backend import ARM_NAMES, arm_vector

WAIST_NAMES = ("waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint")


def stamp_ns(message):
    stamp = message.header.stamp
    sec, ns = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or not 0 <= ns < 1_000_000_000 or sec*1_000_000_000+ns <= 0:
        raise ValueError("missing_or_invalid_source_timestamp")
    return sec*1_000_000_000+ns


def ordered_joints(message, names):
    if len(message.joints) != len(names):
        raise ValueError("unexpected_joint_count")
    if int(message.state.value) != 0:
        raise ValueError("joint_group_not_normal:"+str(message.state.value))
    supplied = [str(j.name) for j in message.joints]
    normalized = [n if n.endswith("_joint") else n+"_joint" for n in supplied]
    if all(not n for n in supplied):
        # Documented fixed order, explicitly surfaced to diagnostics.
        joints, method = list(message.joints), "documented_index_order_empty_names"
    elif set(normalized) == set(names) and len(set(normalized)) == len(names):
        lookup = dict(zip(normalized, message.joints))
        joints, method = [lookup[n] for n in names], "by_joint_name"
    else:
        raise ValueError("unrecognized_duplicate_or_partial_joint_names")
    if any(int(j.error_code) != 0 for j in joints):
        raise ValueError("joint_motor_error")
    values = np.array([[j.position, j.velocity, j.effort] for j in joints], float)
    if not np.isfinite(values).all():
        raise ValueError("nonfinite_joint_feedback")
    return values[:, 0].copy(), values[:, 1].copy(), values[:, 2].copy(), method


@dataclass(frozen=True)
class ArmSample:
    source_ns: int
    received_ns: int
    q: np.ndarray
    dq: np.ndarray
    torque: np.ndarray
    ordering: str


def decode_arm(message, received_ns):
    q, dq, torque, method = ordered_joints(message, ARM_NAMES)
    return ArmSample(stamp_ns(message), int(received_ns), q, dq, torque, method)


def build_command(array_type, joint_type, *, q_target, kp, kd, extra, source_ns, sequence=0, velocity_target=None):
    q, p, d, tau = [arm_vector(v, n) for v, n in (
        (q_target, "q_target"), (kp, "kp"), (kd, "kd"), (extra, "extra"))]
    if np.any(p < 0) or np.any(d < 0):
        raise ValueError("negative_joint_gain")
    velocity = np.zeros(14) if velocity_target is None else arm_vector(velocity_target, "velocity_target")
    command = array_type()
    # MessageHeader is checked by the SDK contract inspector at runtime.
    command.header.stamp.sec = int(source_ns//1_000_000_000)
    command.header.stamp.nanosec = int(source_ns % 1_000_000_000)
    if hasattr(command.header, "sequence"):
        command.header.sequence = int(sequence)
    if hasattr(command.header, "frame_id"):
        command.header.frame_id = "x2_arm_command"
    for i, name in enumerate(ARM_NAMES):
        joint = joint_type()
        joint.name, joint.position, joint.velocity = name, float(q[i]), float(velocity[i])
        joint.effort, joint.stiffness, joint.damping = float(tau[i]), float(p[i]), float(d[i])
        command.joints.append(joint)
    return command


def verify_sdk_types(array_type, joint_type, state_array_type, state_type):
    """Inspect actual generated SDK types. Never initializes ROS or publishes."""
    expected = ((array_type, ("header", "joints")), (joint_type,
                ("name", "position", "velocity", "effort", "stiffness", "damping")),
                (state_array_type, ("header", "state", "joints")),
                (state_type, ("name", "position", "velocity", "effort", "error_code")))
    report = {}
    for typ, required in expected:
        instance = typ()
        missing = [field for field in required if not hasattr(instance, field)]
        if missing:
            raise ValueError(typ.__name__+" missing fields: "+",".join(missing))
        report[typ.__name__] = (instance.get_fields_and_field_types()
                                if hasattr(instance, "get_fields_and_field_types") else list(required))
    for typ in (array_type, state_array_type):
        obj = typ()
        if not hasattr(obj.header, "stamp") or not hasattr(obj.header.stamp, "sec") or not hasattr(obj.header.stamp, "nanosec"):
            raise ValueError("Unsupported SDK MessageHeader timestamp")
    if not hasattr(state_array_type().state, "value"):
        raise ValueError("Unsupported SDK DomainErrorState")
    return report
