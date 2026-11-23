"""Real physics diversity and deterministic parameter draws, no sim-to-real claim."""
import importlib.util
import math
import unittest

from dsiq_native.domain_randomization import run_domain_randomization


class RandomizationArgumentTests(unittest.TestCase):
    def test_bounded_resource_limits(self):
        for args in ({"envs":0},{"envs":9},{"iterations":0},
                     {"iterations":301},{"seed":-1},{"seed":2**32}):
            with self.subTest(args=args),self.assertRaises(ValueError):
                run_domain_randomization(**args)


@unittest.skipUnless(importlib.util.find_spec("pybullet"),
                     "A real Bullet solver is needed")
class ActualBulletParameterRandomizationTests(unittest.TestCase):
    def test_bullet_physically_receives_seeded_parameters(self):
        out=run_domain_randomization(envs=2,iterations=4,seed=2026)
        repeat=run_domain_randomization(envs=2,iterations=4,seed=2026)
        self.assertTrue(out["actual_bullet_stepped"])
        self.assertEqual(out["physics_steps"],64)
        self.assertEqual(out["isolated_clients"],2)
        self.assertEqual(out["parameter_draws_sha256"],
                         repeat["parameter_draws_sha256"])
        self.assertEqual(len(out["environment_parameters"]),2)
        for item in out["environment_parameters"]:
            self.assertTrue(0<=item["simulator_gravity_m_s2"]<=9.81)
            self.assertTrue(.2<=item["actual_solver_lateral_friction"]<=1.2)
        self.assertGreater(out["measured_steps_per_second"],0)
        self.assertTrue(all(
            math.isfinite(row["joint_position_m"])
            for snapshot in out["simulation_trajectories"]
            for row in snapshot["actual_bullet_joint_feedback"]
        ))
        self.assertFalse(out["gpu_parallel_training"])
        self.assertFalse(out["real_world_transfer_tested"])

if __name__=="__main__":
    unittest.main()
