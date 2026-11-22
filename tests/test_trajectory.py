"""Joint-space quintic planner: exact boundary and reference-limit tests."""
import importlib.util
import math
import unittest

from dsiq_native.trajectory import (
    plan_joint_trajectory, minimum_duration,
    BULLET_STEP_SECONDS,
)


class TrajectoryMathTests(unittest.TestCase):
    def test_endpoint_positions_velocities_accelerations_are_exact(self):
        start = (0.,-.3,.2,0.,0.,0.)
        goal = (.25,.15,-.2,.4,-.1,.2)
        plan = plan_joint_trajectory(start, goal)
        self.assertAlmostEqual(plan.dt_s, 1/240, places=12)
        self.assertGreater(len(plan.steps), 10)
        for a,b in zip(plan.steps[0].angles_rad, start):
            self.assertAlmostEqual(a,b,places=12)
        for a,b in zip(plan.steps[-1].angles_rad,goal):
            self.assertAlmostEqual(a,b,places=12)
        for x in plan.steps[0].velocities_rad_s + plan.steps[-1].velocities_rad_s:
            self.assertAlmostEqual(x,0,places=10)
        for x in plan.steps[0].accelerations_rad_s2 + plan.steps[-1].accelerations_rad_s2:
            self.assertAlmostEqual(x,0,places=10)
        self.assertFalse(plan.summary()["collision_free"])
        self.assertFalse(plan.summary()["torque_qualified"])

    def test_reference_kinematics_limits_enforced(self):
        start=(0.,)*6
        end=(.8,-.7,.6,.3,-.2,.15)
        min_t=minimum_duration(start,end)
        self.assertGreater(min_t,0.)
        with self.assertRaises(ValueError):
            plan_joint_trajectory(start,end,duration_s=min_t/2)
        plan=plan_joint_trajectory(start,end)
        self.assertGreaterEqual(plan.duration_s,min_t)
        self.assertLessEqual(plan.max_reference_velocity_rad_s,2.5+1e-8)
        self.assertLessEqual(plan.max_reference_acceleration_rad_s2,5.0+1e-8)

    def test_zero_movement_has_valid_hold_trajectory(self):
        q=(0.,)*6
        plan=plan_joint_trajectory(q,q)
        self.assertTrue(all(all(abs(v)<1e-12 for v in x.velocities_rad_s)
                            for x in plan.steps))
        self.assertGreaterEqual(plan.duration_s,.2)

    def test_bad_api_inputs_fail_closed(self):
        with self.assertRaises(ValueError):
            plan_joint_trajectory((0.,)*6,(math.nan,0.,0.,0.,0.,0.))
        with self.assertRaises(ValueError):
            plan_joint_trajectory((0.,)*6,(3.5,0.,0.,0.,0.,0.))
        with self.assertRaises(ValueError):
            plan_joint_trajectory((0.,)*6,(0.,)*6,dt_s=0.)
        with self.assertRaises(ValueError):
            minimum_duration((0.,)*6,(0.,)*6,max_velocity_rad_s=(0.,)*6)
        with self.assertRaises(ValueError):
            plan_joint_trajectory((0.,)*6,(.4,)*6,duration_s=.01)


@unittest.skipUnless(importlib.util.find_spec("pybullet"), "Requires Bullet optional runtime")
class BulletTrajectoryTests(unittest.TestCase):
    def test_actual_motor_tracking_report_has_measured_residuals(self):
        from dsiq_native.bullet_runtime import BulletPlayground, BulletSettings
        from dsiq_native.trajectory import replay_in_bullet
        plan=plan_joint_trajectory((0.,)*6,(0.,-.18,.25,.1,-.05,.05))
        with BulletPlayground(BulletSettings(payload_count=0)) as world:
            measured=replay_in_bullet(world,plan,record_every=15)
            self.assertEqual(measured["simulated_steps"],len(plan.steps)-1)
            self.assertEqual(world.step_index,len(plan.steps)-1)
            self.assertTrue(math.isfinite(measured["max_abs_joint_tracking_error_rad"]))
            self.assertGreaterEqual(measured["max_abs_joint_tracking_error_rad"],0.)
            self.assertGreaterEqual(
                measured["max_abs_joint_tracking_error_rad"],
                measured["p95_max_joint_tracking_error_rad"]
            )
            self.assertFalse(measured["physical_calibration"])
            self.assertFalse(measured["collision_free_trajectory"])
            self.assertGreater(len(measured["samples"]),1)

    def test_replay_rejects_wrong_start_state(self):
        from dsiq_native.bullet_runtime import BulletPlayground, BulletSettings
        from dsiq_native.trajectory import replay_in_bullet
        plan=plan_joint_trajectory((0.,)*6,(.15,)*6)
        with BulletPlayground(BulletSettings(payload_count=0)) as world:
            world.reset_angles((.4,0.,0.,0.,0.,0.))
            with self.assertRaises(ValueError):
                replay_in_bullet(world,plan)


if __name__ == "__main__":
    unittest.main()
