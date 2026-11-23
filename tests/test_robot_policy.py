"""Actual Bullet Q-learning reference baseline with reproducible held-out metrics."""
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.robot_policy import train_gantry_q_policy


class PolicyValidationTests(unittest.TestCase):
    def test_bounded_arguments(self):
        for bad in ({"episodes":11},{"episodes":513},
                    {"steps_per_episode":3},{"steps_per_episode":41},
                    {"seed":-1},{"seed":2**32}):
            with self.subTest(bad=bad),self.assertRaises(ValueError):
                train_gantry_q_policy("bad.json",**bad)


@unittest.skipUnless(importlib.util.find_spec("pybullet"),
                     "Actual Bullet C++ solver required")
class ActualBulletPolicyTests(unittest.TestCase):
    def test_training_updates_q_from_real_motor_and_reports_out_of_sample(self):
        with TemporaryDirectory() as temp:
            output=Path(temp)/"learned.json"
            result=train_gantry_q_policy(
                output,episodes=24,steps_per_episode=12,seed=1007,
            )
            self.assertGreater(result["training_updates"],10)
            self.assertGreater(result["physics_steps"],200)
            self.assertTrue(output.is_file())
            model=json.loads(output.read_text())
            self.assertEqual(model["algorithm"],"tabular_Q_learning_epsilon_greedy")
            self.assertEqual(model["discrete_states"],13)
            self.assertEqual(model["train_episodes"],24)
            self.assertEqual(len(model["heldout_evaluation"]),12)
            self.assertEqual(len(model["q_values"]),13)
            self.assertEqual(len(model["q_values"][0][0]),3)
            self.assertTrue(any(abs(v)>0 for row in model["q_values"]
                                for x in row for v in x))
            self.assertTrue(0<=result["heldout_success_fraction"]<=1)
            self.assertFalse(result["sim_to_real_qualified"])
            self.assertFalse(result["hardware_control"])
            with self.assertRaises(FileExistsError):
                train_gantry_q_policy(
                    output,episodes=12,steps_per_episode=8
                )

if __name__=="__main__":
    unittest.main()
