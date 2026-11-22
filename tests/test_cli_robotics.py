"""Command-line workflows remain testable without starting a GUI or RTX driver."""
import contextlib
from io import StringIO
import json
import importlib.util
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.cli import main


def invoke(*args):
    out = StringIO()
    err = StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
        code = main(list(args))
    return code, json.loads((out if code == 0 or out.getvalue() else err).getvalue())


class RoboticsCliTests(unittest.TestCase):
    def test_forward_kinematics_json(self):
        code, data = invoke("robot-fk", "0","0","0","0","0","0")
        self.assertEqual(code, 0)
        self.assertEqual(data["model"], "dsiq_reference_6r_v1")
        self.assertAlmostEqual(data["end_effector_m"][0], 1.2)
        self.assertAlmostEqual(data["end_effector_m"][2], .5)

    def test_inverse_kinematics_success_and_failure_status(self):
        good, solved = invoke("robot-ik", "1.2", "0", "0.5")
        self.assertEqual(good, 0)
        self.assertTrue(solved["converged"])
        bad, report = invoke("robot-ik", "9", "8", "7", "--max-iterations", "30")
        self.assertEqual(bad, 3)
        self.assertFalse(report["converged"])

    def test_joint_limit_errors_are_machine_readable(self):
        code, error = invoke("robot-fk", "0", "200", "0", "0", "0", "0")
        self.assertEqual(code, 2)
        self.assertFalse(error["ok"])
        self.assertIn("limits", error["error"])


@unittest.skipUnless(importlib.util.find_spec("pxr"), "USD integration requires pxr")
class RoboticsCliUsdTests(unittest.TestCase):
    def test_playground_cli_creates_and_controls_actual_usd(self):
        with TemporaryDirectory() as tmp:
            scene = Path(tmp) / "robotics.usda"
            code, data = invoke("playground", str(scene), "--boxes", "2")
            self.assertEqual(code, 0)
            self.assertEqual(len(data["payloads"]), 2)
            self.assertTrue(scene.exists())
            code, report = invoke("playground-report", str(scene))
            self.assertEqual(code, 0)
            self.assertEqual(report["revolute_joints"], 6)
            code, result = invoke("robot-drive", str(scene), "0","-15","35","0","0","0")
            self.assertEqual(code, 0)
            self.assertAlmostEqual(result["target_degrees"][1], -15)
            code, payload = invoke("spawn-box", str(scene), "--mass", "2.0")
            self.assertEqual(code, 0)
            self.assertTrue(payload["spawned_prim"].endswith("/Box_003"))
            code, error = invoke("playground", str(scene))
            self.assertEqual(code, 2)
            self.assertFalse(error["ok"])


@unittest.skipUnless(importlib.util.find_spec("pybullet"), "Optional Bullet required")
class BulletSensorCliTests(unittest.TestCase):
    def test_sensor_cli_outputs_real_rgb_depth_and_lidar(self):
        with TemporaryDirectory() as tmp:
            png = Path(tmp) / "color.png"
            depth = Path(tmp) / "depth.npz"
            report_path = Path(tmp) / "report.json"
            code, data = invoke(
                "bullet-run", "--steps", "12", "--boxes", "1",
                "--lidar-rays", "12", "--camera-png", str(png),
                "--depth-npz", str(depth), "--json-out", str(report_path),
            )
            self.assertEqual(code, 0, data)
            self.assertEqual(data["lidar"]["scan_count"], 12)
            self.assertEqual(data["camera"]["renderer"], "PYBULLET_TINY_RENDERER_CPU")
            self.assertTrue(png.is_file())
            self.assertTrue(depth.is_file())
            self.assertTrue(report_path.is_file())
            self.assertFalse(data["physically_qualified"])



if __name__ == "__main__":
    unittest.main()
