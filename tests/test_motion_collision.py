"""Discrete Bullet robot-path screening must reject real geometry obstruction."""
import importlib.util
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

HAS_BULLET=importlib.util.find_spec("pybullet") is not None


@unittest.skipUnless(HAS_BULLET,"Real Bullet C++ physics runtime required")
class SampledCollisionTests(unittest.TestCase):
    def setUp(self):
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        from dsiq_native.machine_designer import MachineDesign,save_project
        from dsiq_native.imported_trajectory import plan_imported_trajectory
        self.temp=TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        project=save_project(
            MachineDesign(asset_id="screening_xyz_machine"),
            Path(self.temp.name)/"machine",
        )
        self.world=BulletPlayground(BulletSettings(payload_count=0))
        self.addCleanup(self.world.close)
        imported=self.world.import_urdf_robot(
            project["manifest"],base_position_m=(-1.,1.3,0.)
        )
        self.asset=imported["asset_id"]
        self.plan=plan_imported_trajectory(self.world,self.asset,{
            "actuator_x":.3,"actuator_y":.1,
        })

    def test_clear_geometry_and_original_world_state_restored(self):
        from dsiq_native.motion_collision import screen_imported_path
        import_info=self.world.imported_robots[self.asset]
        body=import_info["body_id"]
        before=self.world.p.getBasePositionAndOrientation(
            body,physicsClientId=self.world.client,
        )
        step_before=self.world.step_index
        screened=screen_imported_path(self.world,self.plan)
        self.assertTrue(screened["collision_screen_passed"],screened)
        self.assertEqual(screened["evaluated_samples"],len(self.plan.samples))
        self.assertTrue(screened["world_snapshot_restored"])
        self.assertFalse(screened["full_swept_collision_detection"])
        self.assertEqual(self.world.step_index,step_before)
        after=self.world.p.getBasePositionAndOrientation(
            body,physicsClientId=self.world.client,
        )
        self.assertEqual(before,after)

    def test_real_obstacle_blocks_path_and_does_not_mutate_world(self):
        from dsiq_native.motion_collision import screen_imported_path
        body=self.world.spawn_box(
            side_m=.4,mass_kg=1.,position_m=(-1.6,.9,1.0)
        )
        before=self.world.p.getBasePositionAndOrientation(
            body,physicsClientId=self.world.client,
        )
        result=screen_imported_path(self.world,self.plan)
        self.assertFalse(result["collision_screen_passed"],result)
        self.assertIsNotNone(result["first_blocking_collision"])
        self.assertLess(result["evaluated_samples"],len(self.plan.samples))
        self.assertTrue(result["world_snapshot_restored"])
        after=self.world.p.getBasePositionAndOrientation(
            body,physicsClientId=self.world.client,
        )
        self.assertEqual(before,after)

    def test_invalid_clearance_is_rejected(self):
        from dsiq_native.motion_collision import screen_imported_path
        for value in (-.1,.1,math.nan):
            with self.subTest(value=value),self.assertRaises(ValueError):
                screen_imported_path(self.world,self.plan,clearance_m=value)


if __name__=="__main__":
    unittest.main()
