"""Multiple isolated real Bullet client observations and actions for learning."""
import importlib.util
import math
import unittest

HAS_BULLET=importlib.util.find_spec("pybullet") is not None


@unittest.skipUnless(HAS_BULLET,"Actual Bullet engine is required")
class LearningVectorTests(unittest.TestCase):
    def test_multiple_independent_real_worlds(self):
        from dsiq_native.vector_learning import BulletVectorGantry
        with BulletVectorGantry(num_envs=3,seed=1007) as worlds:
            reset=worlds.reset()
            self.assertEqual(len(reset),3)
            self.assertEqual(len({id(w) for w in worlds._worlds}),3)
            self.assertEqual(len(set(w.client for w in worlds._worlds)),3)
            observed=worlds.step([1,-1,0],frame_skip=8)
            self.assertEqual(len(observed),3)
            self.assertTrue(all(x["simulation_step_index"]==8 for x in observed))
            self.assertTrue(all(math.isfinite(x["velocity_m_s"]) for x in observed))
            self.assertTrue(all(math.isfinite(x["reward"]) for x in observed))
            self.assertEqual(worlds._steps,24)
            with self.assertRaises(ValueError):
                worlds.step([1,4])
        self.assertFalse(worlds._worlds)

    def test_bounded_cpu_throughput_report(self):
        from dsiq_native.vector_learning import benchmark_vector_learning
        report=benchmark_vector_learning(num_envs=2,steps=5)
        self.assertEqual(report["num_isolated_bullet_clients"],2)
        self.assertEqual(report["fixed_physics_steps"],80)
        self.assertGreater(report["measured_total_physics_steps_per_second"],0.)
        self.assertFalse(report["gpu_parallel_training"])
        self.assertFalse(report["real_hardware_policy_qualified"])

    def test_fail_on_excessive_world_count(self):
        from dsiq_native.vector_learning import BulletVectorGantry
        for value in (-1,0,9,1.0):
            with self.subTest(value=value),self.assertRaises(ValueError):
                BulletVectorGantry(num_envs=value)


if __name__=="__main__":
    unittest.main()
