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
