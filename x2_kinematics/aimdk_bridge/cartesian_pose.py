"""Read-only flange poses, independent of whether motion control is enabled."""
import warnings
import numpy as np
from scipy.spatial.transform import Rotation
from ..core import rigid

SIDES = ('left', 'right')


def pose_record(transform):
    transform = rigid(transform)
    rotation = Rotation.from_matrix(transform[:3, :3])
    # R = Rz(yaw) Ry(pitch) Rx(roll); quaternion remains unambiguous at gimbal lock.
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter('always', UserWarning)
        rpy = rotation.as_euler('xyz')
    return dict(xyz_m=transform[:3, 3].tolist(), rpy_rad=rpy.tolist(),
                quaternion_xyzw=rotation.as_quat().tolist(),
                rpy_singular=bool(caught), torso_from_flange=transform.tolist())


def flange_pose_snapshot(model, sample, wrist_from_sensor, now_ns, ros_now_ns,
                         timeout_s, error=None, targets=None):
    result = dict(frame='torso_link', control_point='configured flange sensor origin',
                  orientation='RPY: Rz(yaw) Ry(pitch) Rx(roll)',
                  age_ms=None, source_age_ms=None, source_ns=None,
                  timeout_ms=timeout_s*1000, fresh=False, error=error,
                  target_is_preview=targets is None, arms={s:None for s in SIDES})
    if sample is None:
        return result
    age = (now_ns-sample.received_ns)*1e-9
    source_age = (ros_now_ns-sample.source_ns)*1e-9
    result.update(age_ms=age*1000, source_age_ms=source_age*1000,
                  source_ns=sample.source_ns,
                  fresh=bool(error is None and 0 <= age <= timeout_s and -.01 <= source_age <= timeout_s))
    for side in SIDES:
        # pose() may already use a TCP; remove it before applying the flange mount.
        point_from_sensor = np.linalg.inv(model.tip_from_tcp[side]) @ rigid(wrist_from_sensor[side])
        actual = model.pose(sample.q, side) @ point_from_sensor
        result['arms'][side] = dict(actual=pose_record(actual),
            target=pose_record(actual if targets is None else targets[side]))
    return result
