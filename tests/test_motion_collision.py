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

    def test_adaptive_interpolation_detects_thin_midpath_obstacle(self):
        from dataclasses import replace
        from dsiq_native.motion_collision import screen_imported_path
        # Keep only endpoints, forcing a joint-space gap that a sampled-only
        # checker could miss. Actual Bullet geometry is queried along it.
        coarse=replace(self.plan,samples=(self.plan.samples[0],self.plan.samples[-1]))
        obstacle=self.world.p.createCollisionShape(
            self.world.p.GEOM_BOX,halfExtents=[.01,.01,.01],
            physicsClientId=self.world.client
        )
        body=self.world.p.createMultiBody(
            baseMass=0.,baseCollisionShapeIndex=obstacle,
            basePosition=[-1.45,.9,1.3],physicsClientId=self.world.client,
        )
        p=self.world.p;cid=self.world.client
        before=p.getBasePositionAndOrientation(body,physicsClientId=cid)
        result=screen_imported_path(self.world,coarse,max_joint_step=.01)
        self.assertFalse(result["collision_screen_passed"],result)
        self.assertGreater(result["adaptive_sample_count"],2)
        self.assertTrue(result["subdivision_applied"])
        self.assertFalse(result["inter_sample_collision_guaranteed"])
        self.assertTrue(result["world_snapshot_restored"])
        self.assertEqual(before,p.getBasePositionAndOrientation(
            body,physicsClientId=cid
        ))

    def test_adaptive_budget_fails_closed(self):
        from dataclasses import replace
        from dsiq_native.motion_collision import screen_imported_path
        coarse=replace(self.plan,samples=(self.plan.samples[0],self.plan.samples[-1]))
        with self.assertRaisesRegex(ValueError,"budget"):
            screen_imported_path(
                self.world,coarse,max_joint_step=.001,max_total_samples=100,
            )

    def test_invalid_clearance_is_rejected(self):
        from dsiq_native.motion_collision import screen_imported_path
        for value in (-.1,.1,math.nan):
            with self.subTest(value=value),self.assertRaises(ValueError):
                screen_imported_path(self.world,self.plan,clearance_m=value)


    def test_reject_modified_joint_indices_and_source_speed_limits(self):
        from dataclasses import replace
        from dsiq_native.motion_collision import screen_imported_path
        altered_index=replace(
            self.plan,
            joint_indices=(999,) + self.plan.joint_indices[1:],
        )
        with self.assertRaisesRegex(ValueError,"joint indices"):
            screen_imported_path(self.world,altered_index)
        altered_speed=replace(
            self.plan,
            declared_speed_limits=(999.,) + self.plan.declared_speed_limits[1:],
        )
        with self.assertRaisesRegex(ValueError,"velocity ratings"):
            screen_imported_path(self.world,altered_speed)

    def test_reject_stale_bullet_start_pose_without_resetting_it(self):
        from dsiq_native.motion_collision import screen_imported_path
        info=self.world.imported_robots[self.asset]
        index=self.plan.joint_indices[0]
        p=self.world.p
        cid=self.world.client
        changed=self.plan.start[0]+.01
        p.resetJointState(info["body_id"],index,changed,physicsClientId=cid)
        with self.assertRaisesRegex(ValueError,"state changed"):
            screen_imported_path(self.world,self.plan)
        self.assertAlmostEqual(p.getJointState(
            info["body_id"],index,physicsClientId=cid
        )[0],changed)

    def test_tampered_endpoint_nan_and_outside_urdf_limits_are_rejected(self):
        from dataclasses import replace
        from dsiq_native.motion_collision import screen_imported_path
        q=list(self.plan.samples)
        q[-1]=tuple(x+.01 for x in q[-1])
        with self.assertRaisesRegex(ValueError,"endpoint"):
            screen_imported_path(self.world,replace(self.plan,samples=tuple(q)))
        q=list(self.plan.samples)
        q[1]=(math.nan,)+q[1][1:]
        with self.assertRaisesRegex(ValueError,"nonfinite"):
            screen_imported_path(self.world,replace(self.plan,samples=tuple(q)))
        q=list(self.plan.samples)
        q[1]=(100.,)+q[1][1:]
        with self.assertRaisesRegex(ValueError,"absolute joint bound"):
            screen_imported_path(self.world,replace(self.plan,samples=tuple(q)))

    def test_invalid_clock_values_block_screening(self):
        from dataclasses import replace
        from dsiq_native.motion_collision import screen_imported_path
        for field,value in (("duration_s",math.nan),("timestep_s",math.inf),
                            ("duration_s",0.),("timestep_s",-1.)):
            with self.subTest(field=field,value=value),self.assertRaises(ValueError):
                screen_imported_path(self.world,replace(self.plan,**{field:value}))

    def test_coarse_path_uses_full_elapsed_duration_and_disclaims_ccd(self):
        from dataclasses import replace
        from dsiq_native.motion_collision import screen_imported_path
        coarse=replace(self.plan,samples=(self.plan.samples[0],self.plan.samples[-1]))
        report=screen_imported_path(self.world,coarse)
        self.assertTrue(report["path_preflight_identity_and_limits_passed"])
        self.assertTrue(report["start_state_verified_against_live_bullet"])
        self.assertAlmostEqual(report["screened_path_duration_s"],self.plan.duration_s)
        self.assertEqual(report["time_mapping"],
                         "UNIFORM_PATH_PROGRESS_NOT_CONTINUOUS_SERVO_TRAJECTORY")
        self.assertFalse(report["inter_sample_collision_guaranteed"])



if __name__=="__main__":
    unittest.main()
