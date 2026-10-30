"""Quasi-static gravity direction from the pelvis IMU, not the chest IMU.

The vendor uses frame_id=base_link for both IMUs. Select the pelvis by
topic. The nominal URDF pelvis IMU rotation is identity; this check does
not replace factory extrinsic or force calibration.
"""
import math
import numpy as np


class StaticBaseImu:
    def __init__(self, config, model_from_imu=None):
        self.max_tilt = float(config.get("panel_base_max_tilt_deg", 45.))
        self.max_gyro = float(config.get("panel_base_max_gyro_rad_s", .03))
        self.acc_tolerance = float(config.get("panel_base_acceleration_tolerance_m_s2", .5))
        self.timeout = float(config.get("panel_base_imu_timeout_s", .1))
        self.warmup = float(config.get("panel_base_stable_duration_s", 1.))
        self.max_disagreement = float(config.get("panel_base_gravity_disagreement_deg", 5.))
        self.rotation = np.asarray(np.eye(3) if model_from_imu is None else model_from_imu,float)
        if (self.rotation.shape != (3,3) or not np.isfinite(self.rotation).all() or
            not np.allclose(self.rotation.T@self.rotation,np.eye(3),atol=1e-6) or np.linalg.det(self.rotation)<.999999):
            raise ValueError("IMU to model rotation must be SO(3)")
        if not all(math.isfinite(x) and x > 0 for x in (self.max_tilt,self.max_gyro,self.acc_tolerance,self.timeout,self.warmup,self.max_disagreement)):
            raise ValueError("Invalid base IMU monitor limits")
        self.last_source = self.last_received = self.stable_since = None
        self.good = False
        self.reason = "等待胯部 IMU"
        self.metrics = {}
        self.gravity = None

    def update(self, message, now_ns, ros_now_ns):
        try:
            source = int(message.header.stamp.sec)*1_000_000_000+int(message.header.stamp.nanosec)
            if source <= 0 or not -.01 <= (ros_now_ns-source)*1e-9 <= self.timeout:
                raise ValueError("胯部 IMU 时间戳过期")
            if self.last_source is not None and source <= self.last_source:
                raise ValueError("胯部 IMU 时间戳不递增")
            if self.last_source is not None and (source-self.last_source)*1e-9 > self.timeout:
                self.stable_since=None
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
            self.metrics=dict(orientation_tilt_deg=orientation_tilt,acceleration_tilt_deg=acceleration_tilt,
                              angular_speed_rad_s=rate,acceleration_norm_m_s2=acc_norm,
                              gravity_disagreement_deg=disagreement)
            self.last_source, self.last_received = source, int(now_ns)
            if max(orientation_tilt,acceleration_tilt) > self.max_tilt:
                raise ValueError("骨盆倾角超出本阶段已配置范围")
            if rate > self.max_gyro or abs(acc_norm-9.81) > self.acc_tolerance:
                raise ValueError("骨盆尚未静止")
            if disagreement > self.max_disagreement:
                raise ValueError("IMU 姿态与加速度方向不一致")
            self.gravity=gravity
            self.good=True;self.reason=None
            if self.stable_since is None:self.stable_since=int(now_ns)
        except (ValueError,AttributeError) as error:
            self.good=False;self.stable_since=None;self.reason=str(error)

    def status(self, now_ns, ros_now_ns):
        age=None if self.last_received is None else (now_ns-self.last_received)*1e-9
        fresh=bool(age is not None and 0 <= age <= self.timeout and
                   -.01 <= (ros_now_ns-self.last_source)*1e-9 <= self.timeout)
        stable=0. if self.stable_since is None else max(0.,(now_ns-self.stable_since)*1e-9)
        ready=self.good and fresh and stable >= self.warmup
        reason=None if ready else self.reason or ("胯部 IMU 反馈过期" if not fresh else "等待骨盆持续稳定")
        return dict(ready=bool(ready),reason=reason,age_ms=None if age is None else age*1000,
                    source_age_ms=None if self.last_source is None else (ros_now_ns-self.last_source)*1e-6,
                    stable_s=stable,max_tilt_deg=self.max_tilt,
                    gravity_model_m_s2=None if self.gravity is None else self.gravity.tolist(),
                    model_from_imu_rotation=self.rotation.tolist(),**self.metrics)
