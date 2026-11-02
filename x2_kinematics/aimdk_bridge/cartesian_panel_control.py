"""Web Cartesian impedance adapter; numerical validation, robot disabled by default.

Targets are absolute offsets from the captured starting flange frame. The
observer's wrench is diagnostic only: this is J.T impedance, not force feedback.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from ..cartesian_impedance import DualArmCartesianImpedance, gains
from .dynamics import LiveWaistArmDynamics
from .joint_panel_control import JointPanelControl, JointPlan
from .runtime import repeated

SIDES = ("left", "right")


def validated_task_gains(config, stiffness, damping):
    result = []
    for value, key, default in ((stiffness, "stiffness", [600,600,600,10,10,10]),
                                (damping, "damping", [30,30,30,1,1,1])):
        if np.asarray(value).dtype.kind not in "fiu" or any(isinstance(x,(bool,np.bool_)) for x in np.asarray(value,dtype=object).flat):
            raise ValueError("Cartesian gains must be numeric arrays")
        values = gains(value, key)
        maximum = gains(config.get("panel_cartesian_"+key+"_max", default), key+"_max")
        if np.any(values > maximum):
            raise ValueError("Cartesian "+key+" exceeds the configured tuning range")
        result.append(values)
    return tuple(result)


class CartesianPanelControl(JointPanelControl):
    mode = "cartesian"

    def __init__(self, source_model, config, wrist_from_sensor):
        self.source_model = source_model
        model = LiveWaistArmDynamics(tip_from_tcp=wrist_from_sensor)
        super().__init__(model, config)
        self.kp = np.zeros(14)
        self.kd = repeated(config.get("panel_cartesian_joint_damping", [1.5,1.5,1,1,.5,.5,.5]), "cartesian_joint_damping")
        stiffness, damping = validated_task_gains(config,
            config.get("panel_cartesian_stiffness", [150,150,150,2,2,2]),
            config.get("panel_cartesian_damping", [12,12,12,.25,.25,.25]))
        self.kernel = DualArmCartesianImpedance(model, stiffness=stiffness, damping=damping,
            joint_damping=self.kd, max_force_N=8., max_moment_Nm=.6,
            max_position_error_m=.08, max_orientation_error_rad=.4,
            enforce_model_position_limits=False)
        self.stiffness_target, self.damping_target = stiffness.copy(), damping.copy()
        self.anchors = self.targets = self.goal_poses = None
        self.offsets = {side: np.zeros(6) for side in SIDES}
        self.max_offset = float(config.get("panel_cartesian_offset_limit_m", .03))
        self.max_rotation = float(config.get("panel_cartesian_rotation_limit_rad", np.deg2rad(10)))
        self.linear_speed = float(config.get("panel_cartesian_max_velocity_m_s", .01))
        self.angular_speed = float(config.get("panel_cartesian_max_angular_velocity_rad_s", .05))
        if any(not np.isfinite(x) or x <= 0 for x in (self.max_offset,self.max_rotation,self.linear_speed,self.angular_speed)):
            raise ValueError("Invalid Cartesian target bounds or velocity")
        self.reference_generation = 0
        self.diagnostics = None
        self.drag_enabled = False
        self.last_extra = None

    def _sync_model(self):
        self.model.set_waist(self.source_model.waist)
        self.model.set_base_gravity(self.source_model.model.gravity.linear)

    def _capture(self):
        self._sync_model()
        self.anchors = {s:self.model.pose(self.sample.q,s) for s in SIDES}
        self.targets = {s:t.copy() for s,t in self.anchors.items()}
        self.goal_poses = {s:t.copy() for s,t in self.anchors.items()}
        self.offsets = {s:np.zeros(6) for s in SIDES}
        self.kernel.q_rest = self.sample.q.copy()
        self.reference_generation += 1

    def arm(self, now_ns, ros_now_ns):
        super().arm(now_ns, ros_now_ns)
        self.drag_enabled = False
        self.last_extra = self.initial_extra.copy()
        # JointPanelControl already enforces the configured recovery margin,
        # captured-pose outward tolerance, and revokes the allowance on return.
        # The torque kernel must not apply a second, stricter URDF-only gate.
        self._capture()

    def set_drag(self, enabled, now_ns, ros_now_ns):
        if not isinstance(enabled, bool):
            raise ValueError("拖拽开关必须为布尔值")
        if self.state != 'armed' or self.fault:
            raise ValueError("请先启动笛卡尔控制，再切换拖拽")
        self._fresh(now_ns, ros_now_ns)
        if enabled == self.drag_enabled:
            return
        # Both transitions forget old task targets. Exiting drag must never
        # pull the hand back to its pre-drag pose. Blend from the last output.
        self._capture()
        self.initial_extra = self.last_extra.copy()
        self.started_ns = self.last_command_ns = int(now_ns)
        self.drag_enabled = enabled

    def update_gains(self, kp, kd):
        raise ValueError("Joint PD tuning applies to joint mode; Cartesian mode uses task K/D")

    def update_task_gains(self, stiffness, damping):
        self.stiffness_target, self.damping_target = validated_task_gains(self.cfg, stiffness, damping)

    def target(self, value):
        raise ValueError("Joint targets are not accepted in Cartesian mode")

    def target_offsets(self, offsets):
        if self.state != "armed":
            raise ValueError("Please start Cartesian control first")
        if self.drag_enabled:
            raise ValueError("请先关闭双臂拖拽，再调整末端目标")
        if not isinstance(offsets, dict) or set(offsets) != set(SIDES):
            raise ValueError("Both flange offsets are required")
        values, poses = {}, {}
        for side in SIDES:
            raw = np.asarray(offsets[side])
            if raw.dtype.kind not in "fiu" or any(isinstance(x,(bool,np.bool_)) for x in np.asarray(offsets[side],dtype=object).flat):
                raise ValueError("Cartesian offsets must be numeric")
            v = raw.astype(float)
            if v.shape != (6,) or not np.isfinite(v).all():
                raise ValueError("Offsets require six finite SI values per arm")
            if np.any(np.abs(v[:3]) > self.max_offset+1e-12) or np.any(np.abs(v[3:]) > self.max_rotation+1e-12):
                raise ValueError("Cartesian target exceeds the configured offset range")
            anchor = self.anchors[side]
            pose = anchor.copy()
            pose[:3,3] += anchor[:3,:3] @ v[:3]
            pose[:3,:3] = anchor[:3,:3] @ Rotation.from_rotvec(v[3:]).as_matrix()
            values[side], poses[side] = v.copy(), pose
        self.offsets, self.goal_poses = values, poses

    def hold(self):
        if self.active:
            self.drag_enabled = False
            self._capture()
            self.position, self.velocity = self.sample.q.copy(), np.zeros(14)
            self.goal = self.position.copy()
            self.state = "fault" if self.fault else "holding"

    def hold_plan(self):
        if not self.active:
            return None
        # Fault fallback removes task torque; it is gravity plus damping, not
        # a promise of position holding with missing/stale measurements.
        extra = np.clip(self.gravity*self.scale, -self.extra_limit, self.extra_limit)
        return JointPlan(self.sample.q.copy(), np.zeros(14), np.zeros(14), self.kd.copy(), extra)

    def _output(self, desired_extra, now_ns):
        ramp = min(1., max(0., (now_ns-self.started_ns)*1e-9/self.ramp_s))
        extra = (1-ramp)*self.initial_extra+ramp*desired_extra
        total = extra-self.kd*self.sample.dq
        if np.any(np.abs(extra) > self.extra_limit) or np.any(np.abs(total) > self.model.effort_limits):
            raise ValueError("Cartesian torque exceeds configured joint limits")
        self.position = self.sample.q.copy()
        self.last_command_ns = int(now_ns)
        self.last_extra = extra.copy()
        return JointPlan(self.position.copy(),np.zeros(14),self.kp.copy(),self.kd.copy(),extra)

    def step(self, now_ns, ros_now_ns):
        if not self.active:
            return None
        if self.fault:
            return self.hold_plan()
        try:
            self._fresh(now_ns, ros_now_ns)
            dt = (now_ns-self.last_command_ns)*1e-9
            if not 0 <= dt <= self.cfg["feedback_timeout_s"]:
                raise ValueError("Cartesian control loop stalled")
            self._sync_model()
            if self.drag_enabled:
                self.diagnostics = None
                return self._output(self.gravity*self.scale, now_ns)
            twist = np.zeros((2,6))
            for i,side in enumerate(SIDES):
                reference, goal = self.targets[side], self.goal_poses[side]
                for part in ("linear", "angular"):
                    delta = (goal[:3,3]-reference[:3,3] if part == "linear" else
                             Rotation.from_matrix(goal[:3,:3] @ reference[:3,:3].T).as_rotvec())
                    bound = (self.linear_speed if part == "linear" else self.angular_speed)*dt
                    delta *= min(1., bound/max(np.linalg.norm(delta), 1e-15))
                    if part == "linear": reference[:3,3] += delta
                    else: reference[:3,:3] = Rotation.from_rotvec(delta).as_matrix() @ reference[:3,:3]
                    if dt > 0:
                        twist[i,:3] = delta/dt if part == "linear" else twist[i,:3]
                        if part == "angular": twist[i,3:] = delta/dt
            # Convex interpolation keeps gain changes inside validated bounds.
            amount = min(1., dt/.4)
            self.kernel.stiffness += amount*(self.stiffness_target-self.kernel.stiffness)
            self.kernel.damping += amount*(self.damping_target-self.kernel.damping)
            command = self.kernel.compute(self.sample.q, self.sample.dq, self.targets,
                desired_twist_torso=twist, gravity_torque_Nm=self.gravity*self.scale)
            self.diagnostics = command.as_dict()
            fatal = [r for r in command.reasons if r != "task_wrench_limited"]
            if fatal:
                raise ValueError("Cartesian command rejected: "+", ".join(fatal))
            return self._output(command.torque_extra_Nm, now_ns)
        except (ValueError,np.linalg.LinAlgError) as error:
            self.fail(error)
            return self.hold_plan()

    def snapshot(self):
        actual={s:self.model.pose(self.sample.q,s).tolist() for s in SIDES}
        return dict(offsets={s:v.tolist() for s,v in self.offsets.items()},
            drag_enabled=self.drag_enabled,transition_ramp_s=self.ramp_s,
            started_ns=self.started_ns,
            reference_generation=self.reference_generation,
            actual_poses=actual,
            target_poses=actual if self.drag_enabled else {s:t.tolist() for s,t in self.targets.items()},
            stiffness=self.kernel.stiffness.tolist(),damping=self.kernel.damping.tolist(),
            diagnostics=self.diagnostics, frame="captured flange-local offsets; poses in torso_link",
            fault_fallback="gravity_and_joint_damping")
