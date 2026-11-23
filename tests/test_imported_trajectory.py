"""General imported URDF trajectory tests (not specialized 6R kinematics)."""
import importlib.util
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.imported_trajectory import (
    plan_imported_trajectory,execute_imported_trajectory,
)
from dsiq_native.machine_designer import MachineDesign,save_project

HAS_BULLET=importlib.util.find_spec("pybullet") is not None


@unittest.skipUnless(HAS_BULLET,"Real Bullet optional dependency required")
class ImportedTrajectoryTests(unittest.TestCase):
    def setUp(self):
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        self.temp=TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        source=save_project(
            MachineDesign(asset_id="tested_three_axis_gantry"),
            Path(self.temp.name)/"design",
        )
        self.world=BulletPlayground(
            BulletSettings(payload_count=0,gravity_m_s2=0.)
        )
        self.addCleanup(self.world.close)
        self.asset=self.world.import_urdf_robot(
            source["manifest"],base_position_m=(-1.,1.3,0.)
        )["asset_id"]

    def test_three_prismatic_axes_and_fixed_flange_are_handled(self):
        path=plan_imported_trajectory(
            self.world,self.asset,{"actuator_x":.25,"actuator_y":.12},
        )
        self.assertEqual(len(path.names),3)
        self.assertEqual(len(path.samples[0]),3)
        self.assertNotIn("flange_attachment",path.names)
        self.assertGreater(len(path.samples),40)
        self.assertEqual(path.start,path.samples[0])
        self.assertAlmostEqual(path.samples[-1][0],.25,places=8)
        self.assertAlmostEqual(path.samples[-1][1],.12,places=8)
        self.assertAlmostEqual(path.samples[-1][2],0.,places=8)
        self.assertFalse(path.summary()["collision_checked"])

    def test_actual_bullet_motor_execution_reports_feedback(self):
        path=plan_imported_trajectory(
            self.world,self.asset,{"actuator_x":.30,"actuator_y":.13}
        )
        record=execute_imported_trajectory(self.world,path,record_every=30)
        self.assertEqual(record["simulated_steps"],len(path.samples)-1)
        self.assertEqual(self.world.step_index,len(path.samples)-1)
        self.assertGreater(record["final_actual"]["actuator_x"],.02)
        self.assertGreater(record["final_actual"]["actuator_y"],.005)
        self.assertTrue(math.isfinite(record["p95_joint_tracking_error"]))
        self.assertGreaterEqual(
            record["max_joint_tracking_error"],
            record["p95_joint_tracking_error"],
        )
        self.assertFalse(record["physical_hardware_qualified"])
        self.assertFalse(record["collision_free_certified"])

    def test_fail_closed_on_bad_speed_limits_and_out_of_range_goals(self):
        with self.assertRaises(ValueError):
            plan_imported_trajectory(self.world,self.asset,{"unknown_joint":.3})
        with self.assertRaises(ValueError):
            plan_imported_trajectory(self.world,self.asset,{"actuator_x":20.})
        with self.assertRaises(ValueError):
            plan_imported_trajectory(self.world,self.asset,{"flange_attachment":1.})
        with self.assertRaises(ValueError):
            plan_imported_trajectory(
                self.world,self.asset,{"actuator_x":.35},duration_s=.01
            )

    def test_execution_refuses_changed_intermediate_samples(self):
        from dataclasses import replace
        from dsiq_native.imported_trajectory import validate_execution_plan
        plan=plan_imported_trajectory(
            self.world,self.asset,{"actuator_x":.28},
        )
        good=validate_execution_plan(self.world,plan)
        self.assertTrue(good["all_samples_match_plan"])
        values=list(plan.samples)
        values[5]=(values[5][0]+.01,)+values[5][1:]
        mutated=replace(plan,samples=tuple(values))
        step_before=self.world.step_index
        with self.assertRaisesRegex(ValueError,"quintic samples"):
            execute_imported_trajectory(self.world,mutated)
        self.assertEqual(self.world.step_index,step_before)

    def test_execution_refuses_decimated_or_mistimed_motor_commands(self):
        from dataclasses import replace
        plan=plan_imported_trajectory(self.world,self.asset,{"actuator_x":.28})
        coarse=replace(plan,samples=(plan.samples[0],plan.samples[-1]))
        with self.assertRaisesRegex(ValueError,"sample count"):
            execute_imported_trajectory(self.world,coarse)
        too_fast=replace(plan,timestep_s=plan.timestep_s*.5)
        with self.assertRaisesRegex(ValueError,"fixed timestep"):
            execute_imported_trajectory(self.world,too_fast)
        wrong_speed=replace(plan,declared_speed_limits=tuple(
            x*2 for x in plan.declared_speed_limits
        ))
        with self.assertRaisesRegex(ValueError,"velocity ratings"):
            execute_imported_trajectory(self.world,wrong_speed)
        self.assertEqual(self.world.step_index,0)

    def test_post_step_collision_aborts_future_motor_targets(self):
        plan=plan_imported_trajectory(
            self.world,self.asset,{"actuator_x":.30,"actuator_y":.10},
        )
        # These exact fixture dimensions were first validated against
        # Bullet in the discrete path-screening regression tests.
        self.world.spawn_box(
            side_m=.4,mass_kg=1.,position_m=(-1.6,.9,1.0)
        )
        report=execute_imported_trajectory(
            self.world,plan,runtime_clearance_m=.003,
        )
        self.assertFalse(report["trajectory_completed"],report)
        self.assertFalse(report["runtime_monitor_passed"])
        self.assertIn("CLEARANCE",report["runtime_abort_reason"])
        self.assertLess(report["simulated_steps"],report["planned_steps"])
        self.assertTrue(report["post_step_neutralization_requested"])
        self.assertEqual(report["simulated_steps"],self.world.step_index)
        self.assertFalse(report["physical_hardware_qualified"])

    def test_runtime_monitor_parameters_reject_nonfinite_values(self):
        plan=plan_imported_trajectory(self.world,self.asset,{"actuator_x":.25})
        for kwargs in (
            {"runtime_clearance_m":math.nan},
            {"runtime_clearance_m":-.01},
            {"max_tracking_error":math.inf},
            {"max_tracking_error":-1.},
        ):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):
                execute_imported_trajectory(self.world,plan,**kwargs)
        self.assertEqual(self.world.step_index,0)

    def test_must_match_actual_measured_joint_start(self):
        path=plan_imported_trajectory(
            self.world,self.asset,{"actuator_x":.25}
        )
        j=next(j for j in self.world.imported_robots[self.asset]["joints"]
               if j["name"]=="actuator_x")
        self.world.p.resetJointState(
            self.world.imported_robots[self.asset]["body_id"],
            j["joint_index"],.15,
            physicsClientId=self.world.client,
        )
        with self.assertRaisesRegex(ValueError,"start state"):
            execute_imported_trajectory(self.world,path)


if __name__=="__main__":
    unittest.main()
