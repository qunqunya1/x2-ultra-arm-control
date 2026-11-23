import unittest
from types import SimpleNamespace as O
import numpy as np
from x2_kinematics.aimdk_bridge.body_panel_control import BodyPanelControl, GROUPS, build_body_command
from x2_kinematics.aimdk_bridge.dashboard import PanelStore, telemetry


class BodyPanelTests(unittest.TestCase):
    def body(self, q=.8):
        body=BodyPanelControl()
        for group,names in GROUPS.items():
            body.feedback(group,telemetry(names,np.full(len(names),q),np.zeros(len(names)),np.ones(len(names)),10**9),10**9,10**9)
        return body

    def test_linear_three_second_exact_zero_without_urdf_clipping(self):
        body=self.body(2.)  # Deliberately outside typical model ranges.
        body.start(10**9,10**9)
        seconds=body.seconds
        self.assertEqual(seconds,3.)
        for t in np.linspace(0,seconds+4,101):
            now=int(10**9+t*10**9)
            for group,names in GROUPS.items():
                body.feedback(group,telemetry(names,np.full(len(names),2.),np.zeros(len(names)),np.ones(len(names)),now),now,now)
            plans=body.step(now,now)
            for group,plan in plans.items():
                self.assertTrue(np.all(np.isfinite(plan['q'])))
                np.testing.assert_allclose(plan['q'],2.*(1-min(t/3.,1.)),atol=1e-8)
                np.testing.assert_allclose(plan['velocity'],-2./3. if t<3 else 0.)
                np.testing.assert_allclose(plan['extra'],0.)
                self.assertLess(np.max(plan['kp']),80.)
        for plan in plans.values():np.testing.assert_allclose(plan['q'],0)
        self.assertEqual(body.phase,'holding_zero')
        self.assertFalse(body.snapshot(now,now)['position_limits_enforced'])
        self.assertEqual(body.snapshot(now,now)['publish_rate_hz'],100.)

    def test_missing_or_stale_feedback_cannot_start_and_stale_motion_holds(self):
        with self.assertRaises(ValueError):BodyPanelControl().start(10**9,10**9)
        body=self.body()
        with self.assertRaises(ValueError):body.start(2*10**9,2*10**9)
        body.start(10**9,10**9)
        plans=body.step(2*10**9,2*10**9)
        self.assertEqual(body.phase,'holding_fault')
        for plan in plans.values():
            np.testing.assert_allclose(plan['q'],.8)
            np.testing.assert_allclose(plan['velocity'],0)

    def test_hold_stops_reference_with_zero_feedforward(self):
        body=self.body()
        body.start(10**9,10**9)
        plans=body.step(10**9,10**9)
        for plan in plans.values():np.testing.assert_allclose(plan['extra'],0)
        body.hold()
        self.assertEqual(body.phase,'holding_current')
        for plan in body.step(10**9,10**9).values():
            np.testing.assert_allclose(plan['q'],.8)
            np.testing.assert_allclose(plan['extra'],0)

    def test_three_second_motion_does_not_trip_old_slow_speed_gate(self):
        body=self.body(.6);body.start(10**9,10**9)
        now=2*10**9
        for group,names in GROUPS.items():
            body.feedback(group,telemetry(names,np.full(len(names),.4),np.full(len(names),-.2),np.ones(len(names)),now),now,now)
        plans=body.step(now,now)
        self.assertIsNone(body.fault)
        self.assertEqual(body.phase,'returning_zero')
        for plan in plans.values():np.testing.assert_allclose(plan['q'],.4)

    def test_body_commands_use_only_named_body_groups(self):
        body=self.body();body.start(10**9,10**9)
        class Array:
            def __init__(self):self.header=O(stamp=O(sec=0,nanosec=0),sequence=0,frame_id='');self.joints=[]
        for group,plan in body.step(10**9,10**9).items():
            command=build_body_command(Array,O,group,plan,10**9,1)
            self.assertEqual([j.name for j in command.joints],list(GROUPS[group]))
            self.assertTrue(all('shoulder' not in j.name for j in command.joints))

    def test_body_request_requires_control_mode_and_respects_other_owner(self):
        with self.assertRaises(ValueError):PanelStore('x',False).submit(dict(op='body_zero',client_id='a'),0)
        store=PanelStore('x',True)
        store.submit(dict(op='body_zero',client_id='a'),0)
        self.assertEqual(store.drain()[0]['op'],'body_zero')
        store.set_owner('other',1)
        with self.assertRaises(ValueError):store.submit(dict(op='body_zero',client_id='a'),2)


if __name__=='__main__':unittest.main()
