"""Real Bullet-to-Pixar OpenUSD time sampling of measured robot motion."""
import importlib.util
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

HAS_BOTH=(importlib.util.find_spec("pybullet") is not None
          and importlib.util.find_spec("pxr") is not None)


@unittest.skipUnless(HAS_BOTH,"Requires real OpenUSD and Bullet bindings")
class BulletUSDTimeSamplingTests(unittest.TestCase):
    def test_native_usd_contains_measured_robot_movement_not_dummy_meshes(self):
        from dsiq_native.usd_telemetry import (
            record_bullet_usd,inspect_bullet_usd,
        )
        from dsiq_native.machine_designer import MachineDesign,save_project
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        from pxr import Usd,UsdGeom
        with TemporaryDirectory() as folder:
            project=save_project(
                MachineDesign(asset_id="telemetry_xyz_gantry"),
                Path(folder)/"machine",
            )
            scene=Path(folder)/"recording.usda"
            with BulletPlayground(BulletSettings(
                payload_count=1,gravity_m_s2=0.
            )) as world:
                imported=world.import_urdf_robot(
                    project["manifest"],base_position_m=(-1.,1.3,0.)
                )
                world.command_imported_joint(
                    imported["asset_id"],"actuator_x",.3,
                )
                result=record_bullet_usd(
                    world,scene,frames=4,steps_per_frame=16,
                )
                self.assertEqual(result["captured_frames"],4)
                self.assertEqual(result["end_step"],64)
                self.assertEqual(result["articulation_tracks"],2)
                self.assertGreaterEqual(result["pose_tracks"],12)
                self.assertFalse(result["visual_meshes_exported"])
                self.assertFalse(result["physics_certified"])
                self.assertTrue(scene.is_file())
                measured=world.p.getJointState(
                    imported["body_id"],
                    next(j["joint_index"] for j in imported["joints"]
                         if j["name"]=="actuator_x"),
                    physicsClientId=world.client,
                )[0]
                stage=Usd.Stage.Open(str(scene))
                machine=stage.GetPrimAtPath(
                    "/World/Telemetry/Imported_telemetry_xyz_gantry"
                )
                self.assertTrue(machine)
                keyframes=machine.GetAttribute("dsiq:jointPositions").GetTimeSamples()
                self.assertEqual(keyframes,[16.,32.,48.,64.])
                values=machine.GetAttribute("dsiq:jointPositions")
                first=values.Get(Usd.TimeCode(16))[0]
                final=values.Get(Usd.TimeCode(64))[0]
                self.assertGreater(final,first)
                self.assertAlmostEqual(final,measured,places=8)
                self.assertFalse(machine.GetCustomDataByKey("dsiq_oem_calibrated") is True)
                self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(stage),1.)
                self.assertEqual(str(UsdGeom.GetStageUpAxis(stage)),"Z")
                with self.assertRaises(FileExistsError):
                    record_bullet_usd(world,scene,frames=1)
            inspected=inspect_bullet_usd(scene)
            self.assertEqual(inspected["start_time_code"],16.)
            self.assertEqual(inspected["end_time_code"],64.)
            self.assertEqual(len(inspected["keyframe_times"]),4)
            self.assertEqual(inspected["joint_tracks"],2)
            self.assertFalse(inspected["visual_oem_geometry_included"])

    def test_sample_count_and_steps_are_bounded(self):
        from dsiq_native.usd_telemetry import record_bullet_usd
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        with TemporaryDirectory() as folder:
            with BulletPlayground(BulletSettings(payload_count=0)) as world:
                with self.assertRaises(ValueError):
                    record_bullet_usd(world,Path(folder)/"n.usda",frames=100000)
                with self.assertRaises(ValueError):
                    record_bullet_usd(
                        world,Path(folder)/"n.usda",frames=3,steps_per_frame=1000
                    )


if __name__=="__main__":
    unittest.main()
