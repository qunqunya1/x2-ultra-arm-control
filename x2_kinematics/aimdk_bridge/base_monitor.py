"""Quasi-static gravity direction from the pelvis IMU, not the chest IMU.

The vendor uses frame_id=base_link for both IMUs. Select the pelvis by
topic. The nominal URDF pelvis IMU rotation is identity; this check does
not replace factory extrinsic or force calibration.
"""
import math
from collections import Counter
import numpy as np


class StaticBaseImu:
    def __init__(self, config, model_from_imu=None):
        self.max_tilt = float(config.get("panel_base_max_tilt_deg", 45.))
        self.max_gyro = float(config.get("panel_base_max_gyro_rad_s", .03))
        self.acc_tolerance = float(config.get("panel_base_acceleration_tolerance_m_s2", .5))
        self.timeout = float(config.get("panel_base_imu_timeout_s", .1))
        self.warmup = float(config.get("panel_base_stable_duration_s", 1.))
        self.max_disagreement = float(config.get("panel_base_gravity_disagreement_deg", 5.))
        self.filter_tau = float(config.get("panel_base_motion_filter_tau_s", .05))
        self.motion_guard = config.get("panel_base_motion_guard_enabled", True)
        if not isinstance(self.motion_guard, bool):
            raise ValueError("panel_base_motion_guard_enabled must be boolean")
        self.rotation = np.asarray(np.eye(3) if model_from_imu is None else model_from_imu,float)
        if (self.rotation.shape != (3,3) or not np.isfinite(self.rotation).all() or
            not np.allclose(self.rotation.T@self.rotation,np.eye(3),atol=1e-6) or np.linalg.det(self.rotation)<.999999):
            raise ValueError("IMU to model rotation must be SO(3)")
        if not all(math.isfinite(x) and x > 0 for x in (self.max_tilt,self.max_gyro,self.acc_tolerance,self.timeout,self.warmup,self.max_disagreement,self.filter_tau)):
            raise ValueError("Invalid base IMU monitor limits")
        self.last_source = self.last_received = self.stable_since = None
        self.good = False
        self.reason = "等待胯部 IMU"
        self.metrics = {}
        self.gravity = None
        self.motion_filtered = None
        self.rejections = Counter()
        self.last_rejection = None
        self.motion_warnings = []

    def update(self, message, now_ns, ros_now_ns):
        current_metrics = None
        try:
            source = int(message.header.stamp.sec)*1_000_000_000+int(message.header.stamp.nanosec)
            if source <= 0 or not -.01 <= (ros_now_ns-source)*1e-9 <= self.timeout:
                raise ValueError("胯部 IMU 时间戳过期")
            if self.last_source is not None and source <= self.last_source:
                raise ValueError("胯部 IMU 时间戳不递增")
            dt = None if self.last_source is None else (source-self.last_source)*1e-9
            if dt is not None and dt > self.timeout:
                self.stable_since=None
                self.motion_filtered=None
            q=message.orientation; a=message.linear_acceleration; w=message.angular_velocity
            quat=np.array([q.w,q.x,q.y,q.z],float)
            acceleration=np.array([a.x,a.y,a.z],float)
            gyro=np.array([w.x,w.y,w.z],float)
            if not all(np.isfinite(x).all() for x in (quat,acceleration,gyro)):
                raise ValueError("胯部 IMU 数值无效")
            norm=float(np.linalg.norm(quat)); acc_norm=float(np.linalg.norm(acceleration))
            if not .8 <= norm <= 1.2 or acc_norm < 1:
                raise ValueError("胯部 IMU 姿态/加速度无效")
            quat /= norm
            qw,qx,qy,qz=quat
            # q rotates IMU axes into world. R_world_imu.T @ [0,0,-g]
            # uses the third ROW of R, not its third column.
            sensor_gravity=-9.81*np.array([2*(qx*qz-qw*qy),2*(qy*qz+qw*qx),1-2*(qx*qx+qy*qy)])
            gravity=self.rotation@sensor_gravity
            acceleration_model=self.rotation@acceleration
            orientation_tilt=math.degrees(math.acos(float(np.clip(-gravity[2]/9.81,-1,1))))
            acceleration_tilt=math.degrees(math.acos(float(np.clip(acceleration_model[2]/acc_norm,-1,1))))
            disagreement=math.degrees(math.acos(float(np.clip(np.dot(-sensor_gravity/9.81,acceleration/acc_norm),-1,1))))
            rate=float(np.linalg.norm(gyro))
            acceleration_error=abs(acc_norm-9.81)
            raw_motion=np.array([rate,acceleration_error,disagreement])
            alpha=1. if self.motion_filtered is None or dt is None else -math.expm1(-dt/self.filter_tau)
            self.motion_filtered=raw_motion.copy() if self.motion_filtered is None else self.motion_filtered+alpha*(raw_motion-self.motion_filtered)
            self.metrics=dict(orientation_tilt_deg=orientation_tilt,acceleration_tilt_deg=acceleration_tilt,
                              angular_speed_rad_s=rate,acceleration_norm_m_s2=acc_norm,
                              gravity_disagreement_deg=disagreement,
                              angular_speed_filtered_rad_s=float(self.motion_filtered[0]),
                              acceleration_error_m_s2=acceleration_error,
                              acceleration_error_filtered_m_s2=float(self.motion_filtered[1]),
                              gravity_disagreement_filtered_deg=float(self.motion_filtered[2]))
            current_metrics=dict(self.metrics)
            self.last_source, self.last_received = source, int(now_ns)
            # With motion gating disabled, acceleration is diagnostic only:
            # suspended-base acceleration is not a measurement of its tilt.
            if (max(orientation_tilt,acceleration_tilt) if self.motion_guard else orientation_tilt) > self.max_tilt:
                raise ValueError("骨盆倾角超出本阶段已配置范围")
            # A short, moderate spike is filtered, not treated as loss of the
            # IMU. Large raw excursions bypass the filter and reject at once.
            warnings=[]
            if rate > 3*self.max_gyro or acceleration_error > 3*self.acc_tolerance:
                warnings.append("骨盆运动明显超限")
            if disagreement > 3*self.max_disagreement:
                warnings.append("IMU 姿态与加速度方向不一致")
            if self.motion_filtered[0] > self.max_gyro or self.motion_filtered[1] > self.acc_tolerance:
                warnings.append("骨盆运动持续超限")
            if self.motion_filtered[2] > self.max_disagreement:
                warnings.append("IMU 姿态与加速度方向持续不一致")
            self.motion_warnings=warnings
            if self.motion_guard and warnings:
                raise ValueError(warnings[0])
            self.gravity=gravity
            self.good=True;self.reason=None
            if self.stable_since is None:self.stable_since=int(now_ns)
        except (ValueError,AttributeError) as error:
            self.good=False;self.stable_since=None;self.reason=str(error)
            self.rejections[str(error)]+=1
            self.last_rejection=dict(reason=str(error),received_ns=int(now_ns),metrics=current_metrics)

    def status(self, now_ns, ros_now_ns):
        age=None if self.last_received is None else (now_ns-self.last_received)*1e-9
        fresh=bool(age is not None and 0 <= age <= self.timeout and
                   -.01 <= (ros_now_ns-self.last_source)*1e-9 <= self.timeout)
        stable=0. if self.stable_since is None else max(0.,(now_ns-self.stable_since)*1e-9)
        tracking_valid=self.good and fresh and self.gravity is not None
        ready=tracking_valid and (not self.motion_guard or stable >= self.warmup)
        reason=None if ready else self.reason or ("胯部 IMU 反馈过期" if not fresh else "等待骨盆持续稳定")
        return dict(ready=bool(ready),tracking_valid=bool(tracking_valid),reason=reason,
                    warmup_s=self.warmup,motion_filter_tau_s=self.filter_tau,
                    motion_guard_enabled=self.motion_guard,motion_warnings=list(self.motion_warnings),
                    rejections=dict(self.rejections),last_rejection=self.last_rejection,
                    age_ms=None if age is None else age*1000,
                    source_age_ms=None if self.last_source is None else (ros_now_ns-self.last_source)*1e-6,
                    stable_s=stable,max_tilt_deg=self.max_tilt,
                    gravity_model_m_s2=None if self.gravity is None else self.gravity.tolist(),
                    model_from_imu_rotation=self.rotation.tolist(),**self.metrics)
