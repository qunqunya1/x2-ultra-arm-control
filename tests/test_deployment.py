"""Offline deployment checks; no engine invocation, ROS init or robot output."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import x2_kinematics.aimdk_bridge.dashboard as dashboard

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('dds_generator',ROOT/'deploy/jetson/make_dds_profile.py')
dds=importlib.util.module_from_spec(spec);spec.loader.exec_module(dds)


class DeploymentTests(unittest.TestCase):
    def test_dds_profile_uses_requested_interface_and_udp_only(self):
        with tempfile.TemporaryDirectory() as temp:
            target=Path(temp)/'dds.xml';dds.generate('198.51.100.23',target)
            root=ET.parse(target).getroot();ns={'f':'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
            self.assertEqual(root.find('.//f:address',ns).text,'198.51.100.23')
            self.assertEqual(root.find('.//f:type',ns).text,'UDPv4')
            self.assertEqual(root.find('.//f:useBuiltinTransports',ns).text,'false')

    def test_existing_dds_profile_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            target=Path(temp)/'dds.xml';target.write_text('existing')
            with self.assertRaises(FileExistsError):dds.generate('198.51.100.23',target)
            self.assertEqual(target.read_text(),'existing')

    def test_invalid_interface_does_not_create_profile(self):
        with tempfile.TemporaryDirectory() as temp:
            for address in ('127.0.0.1','0.0.0.0','::1','not-an-ip'):
                target=Path(temp)/'dds.xml'
                with self.assertRaises(ValueError):dds.generate(address,target)
                self.assertFalse(target.exists())

    def test_explicit_local_panel_file_is_loaded_without_ros(self):
        captured=[]
        def inspect(engine,stop):
            captured.append(engine)
            self.assertTrue(engine.cfg['panel_cartesian_robot_enabled'])
            for arm in engine.cfg['panel_cartesian_stiffness']:
                self.assertEqual(arm[:3],[321,321,321])
            self.assertIsNone(engine.control);self.assertEqual(engine.motor_commands_sent,0)
        with tempfile.TemporaryDirectory() as temp:
            target=Path(temp)/'panel.json';cfg=json.loads((ROOT/'configs/joint_panel.json').read_text())
            cfg.update(panel_cartesian_robot_enabled=True,panel_camera_enabled=False,
                panel_cartesian_stiffness=[321,321,321,2,2,2])
            target.write_text(json.dumps(cfg))
            with patch.object(dashboard,'run_robot',side_effect=inspect),contextlib.redirect_stdout(io.StringIO()):
                dashboard.main(['--backend','robot','--mode','observe','--port','0',
                    '--config',str(ROOT/'configs/robot.json'),'--panel-config',str(target)])
        self.assertEqual(len(captured),1)


if __name__=='__main__':unittest.main()
