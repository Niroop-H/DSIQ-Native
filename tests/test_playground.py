"""Real Pixar OpenUSD schema composition checks for DSIQ robotics playground."""
import importlib.util
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.usd_stage import new_stage
from dsiq_native.playground import (
    PlaygroundSettings, create_playground, spawn_box, playground_report,
    set_reference_joint_targets, PLAYGROUND_PATH,
)
from dsiq_native.robotics import JOINT_LIMITS_DEG

HAS_USD = importlib.util.find_spec("pxr") is not None


@unittest.skipUnless(HAS_USD, "Requires official Pixar OpenUSD python bindings")
class PlaygroundUsdTests(unittest.TestCase):
    def setUp(self):
        from pxr import Usd
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "playground.usda"
        new_stage(self.path)
        self.stage = Usd.Stage.Open(str(self.path))

    def test_serializes_physics_robot_and_payloads(self):
        from pxr import Usd, UsdGeom, UsdPhysics
        result = create_playground(self.stage)
        self.assertEqual(len(result["payloads"]), 5)
        self.assertEqual(len(result["robot"]["joints"]), 6)
        self.stage.GetRootLayer().Save()
        self.stage = Usd.Stage.Open(str(self.path))
        report = playground_report(self.stage)
        self.assertTrue(report["exists"])
        self.assertTrue(report["is_robot_present"])
        self.assertEqual(report["revolute_joints"], 6)
        self.assertEqual(report["rigid_bodies"], 11)
        self.assertGreaterEqual(report["colliders"], 12)
        self.assertFalse(report["qualified_for_factory"])
        self.assertIsNotNone(self.stage.GetPrimAtPath(
            PLAYGROUND_PATH + "/Ground"
        ))
        scene = UsdPhysics.Scene.Get(self.stage, "/World/Physics")
        self.assertAlmostEqual(scene.GetGravityMagnitudeAttr().Get(), 9.81)
        self.assertEqual(str(UsdGeom.GetStageUpAxis(self.stage)), "Z")
        robot = self.stage.GetPrimAtPath(PLAYGROUND_PATH + "/Reference6R")
        self.assertTrue(robot.HasAPI(UsdPhysics.ArticulationRootAPI))
        for i in range(6):
            prim = self.stage.GetPrimAtPath(
                PLAYGROUND_PATH + f"/Reference6R/Joints/Joint_{i+1}"
            )
            self.assertTrue(prim.IsA(UsdPhysics.RevoluteJoint))
            joint = UsdPhysics.RevoluteJoint(prim)
            lo, hi = JOINT_LIMITS_DEG[i]
            self.assertAlmostEqual(joint.GetLowerLimitAttr().Get(), lo)
            self.assertAlmostEqual(joint.GetUpperLimitAttr().Get(), hi)
            self.assertEqual(len(joint.GetBody1Rel().GetTargets()), 1)

    def test_drive_targets_are_written_in_degrees(self):
        create_playground(self.stage, PlaygroundSettings(payload_count=0))
        requested = (0.15,-0.25,0.20,0.10,0.0,-0.15)
        result = set_reference_joint_targets(self.stage, requested)
        self.assertAlmostEqual(result[0], math.degrees(requested[0]))
        from pxr import UsdPhysics
        prim = self.stage.GetPrimAtPath(PLAYGROUND_PATH + "/Reference6R/Joints/Joint_1")
        drive = UsdPhysics.DriveAPI.Get(self.stage, prim.GetPath(), "angular")
        self.assertAlmostEqual(drive.GetTargetPositionAttr().Get(), result[0], places=6)
        with self.assertRaises(ValueError):
            set_reference_joint_targets(self.stage, [math.pi] * 6)

    def test_playground_replacement_is_explicit(self):
        create_playground(self.stage, PlaygroundSettings(payload_count=1))
        with self.assertRaises(FileExistsError):
            create_playground(self.stage)
        created = create_playground(self.stage,
            PlaygroundSettings(payload_count=0, include_reference_robot=False), replace=True)
        self.assertEqual(len(created["payloads"]), 0)
        self.assertIsNone(created["robot"])
        self.assertEqual(playground_report(self.stage)["rigid_bodies"], 0)

    def test_payload_cap_and_numeric_validation(self):
        create_playground(self.stage, PlaygroundSettings(payload_count=0))
        for i in range(40):
            spawn_box(self.stage, position_m=(0.1*i - 2.0, 0.0, 1.0))
        with self.assertRaises(ValueError):
            spawn_box(self.stage)
        with self.assertRaises(ValueError):
            PlaygroundSettings(payload_count=41).validate()
        with self.assertRaises(ValueError):
            PlaygroundSettings(payload_mass_kg=float("nan")).validate()
        with self.assertRaises(ValueError):
            spawn_box(self.stage, position_m=(math.inf, 0., 0.))


if __name__ == "__main__":
    unittest.main()
