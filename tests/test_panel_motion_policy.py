"""Synthetic telemetry and dynamics only; no ROS imports or robot output."""
import json
from pathlib import Path
from types import SimpleNamespace as Obj
import unittest

import numpy as np

from x2_kinematics.aimdk_bridge.dashboard import PanelEngine,PanelStore,DemoPlant
from x2_kinematics.aimdk_bridge.protocol import ARM_NAMES,WAIST_NAMES

ROOT=Path(__file__).resolve().parents[1]
NOW=10_000_000_000
SOURCE=1_800_000_000_000_000_000


def telemetry(names,q,source,velocity=None):
    return Obj(header=Obj(stamp=Obj(sec=source//10**9,nanosec=source%10**9)),state=Obj(value=0),
        joints=[Obj(name=name,position=float(value),velocity=0. if velocity is None else float(velocity[i]),
            effort=0.,error_code=0) for i,(name,value) in enumerate(zip(names,q))])


def setup(guard=False,mode='web'):
    cfg=json.loads((ROOT/'configs/robot.json').read_text(encoding='utf8'))
    cfg.update(json.loads((ROOT/'configs/joint_panel.json').read_text(encoding='utf8')))
    cfg.update(actual_onboard_sdk_verified=True,base_assumption_confirmed=True,
        control_ownership_confirmed=True,effort_feedback_semantics_confirmed=True,
        panel_waist_motion_guard_enabled=guard,panel_waist_stable_duration_s=0.)
    flange=json.loads((ROOT/'configs/virtual_flange_sensor.json').read_text(encoding='utf8'))
    flange['calibration_confirmed']=True
    store=PanelStore('synthetic-test',mode!='observe')
    engine=PanelEngine(cfg,flange,store,backend='demo',mode=mode)
    plant=DemoPlant(engine);plant.step(.004,NOW,SOURCE)
    return engine,store,plant


def arm(engine,store):
    store.submit(dict(op='arm',client_id='test'),NOW)
    engine.step(NOW,SOURCE,lambda plan:None)
    assert engine.control.state=='armed'


class MotionPolicyTests(unittest.TestCase):
    def test_velocity_spike_is_diagnostic_when_disabled(self):
        engine,store,_=setup(False);arm(engine,store)
        engine.waist_feedback(telemetry(WAIST_NAMES,[.0002,0,0],SOURCE+4_000_000,[.03663,0,0]),NOW+4_000_000,SOURCE+4_000_000)
        self.assertIsNone(engine.control.fault)
        self.assertFalse(engine.waist_status(NOW+4_000_000,SOURCE+4_000_000)['motion_guard_enabled'])

    def test_enabled_policy_still_stops_for_velocity(self):
        engine,store,_=setup(True);arm(engine,store)
        engine.waist_feedback(telemetry(WAIST_NAMES,[.0002,0,0],SOURCE+4_000_000,[.2,0,0]),NOW+4_000_000,SOURCE+4_000_000)
        self.assertEqual(engine.control.fault,'waist_moving_arm_only_model_invalid')

    def test_disabled_policy_updates_model_for_new_waist_pose(self):
        engine,store,plant=setup(False);arm(engine,store)
        now,source=NOW+4_000_000,SOURCE+4_000_000
        engine.waist_feedback(telemetry(WAIST_NAMES,[.1,0,0],source),now,source)
        engine.arm_feedback(telemetry(ARM_NAMES,plant.q,source),now,source)
        np.testing.assert_allclose(engine.model.waist,[.1,0,0])
        self.assertIsNone(engine.control.fault)
        self.assertTrue(engine.waist_status(now,source)['stationary'])

    def test_disabled_policy_retains_stale_feedback_rejection(self):
        engine,store,_=setup(False);arm(engine,store)
        engine.waist_feedback(telemetry(WAIST_NAMES,[0,0,0],SOURCE-1_000_000_000),NOW+4_000_000,SOURCE+4_000_000)
        self.assertFalse(engine.waist_status(NOW+4_000_000,SOURCE+4_000_000)['valid'])
        self.assertEqual(engine.control.state,'fault')

    def test_disabled_policy_retains_nonfinite_rejection(self):
        engine,store,_=setup(False);arm(engine,store)
        engine.waist_feedback(telemetry(WAIST_NAMES,[np.nan,0,0],SOURCE+4_000_000),NOW+4_000_000,SOURCE+4_000_000)
        self.assertFalse(engine.waist_status(NOW+4_000_000,SOURCE+4_000_000)['valid'])
        self.assertEqual(engine.control.state,'fault')

    def test_policy_flag_requires_boolean(self):
        with self.assertRaisesRegex(ValueError,'must be boolean'):setup('false')

    def test_observe_mode_has_no_controller_or_command_output(self):
        engine,_,plant=setup(False,'observe');sent=[]
        for i in range(5):
            now,source=NOW+i*4_000_000,SOURCE+i*4_000_000
            plant.step(.004,now,source);engine.step(now,source,sent.append)
        self.assertIsNone(engine.control);self.assertEqual(sent,[])
        self.assertEqual(engine.motor_commands_sent,0)


if __name__=='__main__':unittest.main()
