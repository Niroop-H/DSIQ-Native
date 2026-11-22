"""Independent CPU Bullet runtime tests: real rigid bodies, joints and contacts."""
import importlib.util
import math
import unittest

from dsiq_native.robotics import forward_kinematics

HAS_BULLET = importlib.util.find_spec("pybullet") is not None


@unittest.skipUnless(HAS_BULLET, "PyBullet optional physics dependency not installed")
class BulletSimulationTests(unittest.TestCase):
    def setUp(self):
        from dsiq_native.bullet_runtime import BulletPlayground, BulletSettings
        self.world = BulletPlayground(BulletSettings(payload_count=2))
        self.addCleanup(self.world.close)

    def test_six_link_reference_chain_and_finite_sensor_state(self):
        p = self.world.p
        self.assertEqual(p.getNumJoints(self.world.robot, physicsClientId=self.world.client), 6)
        state = self.world.report()
        self.assertEqual(len(state["joint_radians"]), 6)
        self.assertEqual(len(state["payload_positions_m"]), 2)
        self.assertFalse(state["physically_qualified"])
        self.assertTrue(all(math.isfinite(x) for x in state["tool_position_m"]))
        self.assertLess(state["bullet_analytic_position_difference_m"], .01)

    def test_robot_home_and_reference_fk_across_joint_configs(self):
        for q in [
            (0.,)*6,
            (.20,-.40,.30,.10,-.20,.10),
            (-.15,.20,-.30,-.10,.20,-.15),
        ]:
            self.world.reset_angles(q)
            report = self.world.report()
            expected = forward_kinematics(q).end_effector_m
            self.assertLess(math.dist(report["tool_position_m"], expected), .01)
            self.assertLess(report["bullet_analytic_position_difference_m"], .01)

    def test_physics_steps_have_correct_time_and_falling_object(self):
        initial = self.world.report()["payload_positions_m"][0][2]
        result = self.world.step(240)
        later = result["payload_positions_m"][0][2]
        self.assertEqual(result["step_index"], 240)
        self.assertAlmostEqual(result["elapsed_sim_seconds"], 1., places=8)
        self.assertLess(later, initial)
        self.assertGreater(later, 0.)
        self.assertEqual(result["engine"], "pybullet_cpu")

    def test_joint_motors_accept_real_targets_and_move(self):
        before = self.world.report()["joint_radians"]
        self.world.command_joints((0.,-.3,.5,0.,0.,0.))
        after = self.world.step(300)["joint_radians"]
        self.assertGreater(abs(after[1]-before[1]), .05)
        self.assertGreater(abs(after[2]-before[2]), .05)
        self.assertTrue(all(math.isfinite(v) for v in after))

    def test_invalid_commands_are_rejected(self):
        with self.assertRaises(ValueError):
            self.world.command_joints((math.nan, 0.,0.,0.,0.,0.))
        with self.assertRaises(ValueError):
            self.world.command_joints([0.]*5)
        with self.assertRaises(ValueError):
            self.world.step(0)
        with self.assertRaises(ValueError):
            self.world.spawn_box(mass_kg=-1)
        with self.assertRaises(ValueError):
            self.world.spawn_box(position_m=(math.inf,0.,0.))
        for _ in range(38):
            self.world.spawn_box()
        with self.assertRaises(ValueError):
            self.world.spawn_box()

    def test_world_uses_isolated_physics_client(self):
        from dsiq_native.bullet_runtime import BulletPlayground, BulletSettings
        with BulletPlayground(BulletSettings(payload_count=0)) as other:
            self.assertNotEqual(self.world.client, other.client)
            self.world.step(100)
            self.assertEqual(other.step_index, 0)
            self.assertEqual(len(other.payloads), 0)


if __name__ == "__main__":
    unittest.main()
