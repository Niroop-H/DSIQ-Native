"""Three core engine commands execute actual optional Bullet/OpenUSD runtimes."""
import contextlib
from io import StringIO
import importlib.util
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.cli import main
from dsiq_native.machine_designer import MachineDesign,save_project

BULLET=importlib.util.find_spec("pybullet") is not None
USD=importlib.util.find_spec("pxr") is not None


def invoke(*arguments):
    out,err=StringIO(),StringIO()
    with contextlib.redirect_stdout(out),contextlib.redirect_stderr(err):
        status=main(list(map(str,arguments)))
    return status,json.loads(out.getvalue() or err.getvalue())


@unittest.skipUnless(BULLET,"Bullet optional runtime required")
class CorePhysicsRobotCLITests(unittest.TestCase):
    def test_energy_and_momentum_are_accepted_from_actual_bullet(self):
        status,report=invoke("physics-conservation")
        self.assertEqual(status,0,report)
        self.assertTrue(report["passed"],report)
        self.assertEqual(report["case_count"],2)
        self.assertFalse(report["independent_physical_qualification"])

    def test_verified_user_urdf_motion_tracks_multiple_axes(self):
        with TemporaryDirectory() as folder:
            project=save_project(
                MachineDesign(asset_id="cli_generality_check"),
                Path(folder)/"machine"
            )
            status,report=invoke(
                "robot-urdf-motion",project["manifest"],
                "--target","actuator_x=0.20",
                "--target","actuator_y=0.10",
                "--record-every","10",
            )
            self.assertEqual(status,0,report)
            self.assertEqual(len(report["plan"]["joint_names"]),3)
            self.assertGreater(
                report["tracking"]["final_actual"]["actuator_x"],.02
            )
            self.assertGreater(
                report["tracking"]["final_actual"]["actuator_y"],.005
            )
            self.assertFalse(
                report["tracking"]["collision_free_certified"]
            )
            self.assertFalse(
                report["tracking"]["physical_hardware_qualified"]
            )

    def test_unknown_motor_name_is_rejected(self):
        with TemporaryDirectory() as folder:
            project=save_project(
                MachineDesign(asset_id="cli_reject_fixture"),
                Path(folder)/"machine",
            )
            status,report=invoke(
                "robot-urdf-motion",project["manifest"],
                "--target","wrong_axis=0.15",
            )
            self.assertNotEqual(status,0)
            self.assertFalse(report["ok"])


@unittest.skipUnless(BULLET and USD,"OpenUSD and Bullet both required")
class CoreUSDCLITests(unittest.TestCase):
    def test_physics_recording_has_real_time_samples(self):
        with TemporaryDirectory() as folder:
            project=save_project(
                MachineDesign(asset_id="usd_cli_gantry"),
                Path(folder)/"machine",
            )
            scene=Path(folder)/"measured_robot.usda"
            status,report=invoke(
                "bullet-usd-record",scene,
                "--frames",4,"--steps-per-frame",12,
                "--urdf-manifest",project["manifest"],
                "--joint","actuator_x","--target",.2,
            )
            self.assertEqual(status,0,report)
            self.assertTrue(scene.is_file())
            self.assertEqual(report["captured_frames"],4)
            self.assertEqual(report["imported_robot"],"usd_cli_gantry")
            status,inspection=invoke("usd-telemetry-inspect",scene)
            self.assertEqual(status,0,inspection)
            self.assertEqual(len(inspection["keyframe_times"]),4)
            self.assertGreater(inspection["pose_tracks"],10)
            self.assertFalse(inspection["visual_oem_geometry_included"])


if __name__=="__main__":
    unittest.main()
