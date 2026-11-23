"""Real Bullet COM inertial probe acceptance, with no artificial first sample."""
from __future__ import annotations
import importlib.util
import math
import unittest

HAS_BULLET=importlib.util.find_spec("pybullet") is not None


@unittest.skipUnless(HAS_BULLET,"PyBullet native physics optional")
class BulletInertialProbeTests(unittest.TestCase):
    def test_real_com_angular_velocity_and_specific_force(self):
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        from dsiq_native.inertial_sensor import IdealBulletInertialProbe
        with BulletPlayground(BulletSettings(payload_count=0)) as world:
            imu=IdealBulletInertialProbe(world)
            first=imu.sample(link_index=5)
            self.assertFalse(first["acceleration_is_valid"])
            self.assertIsNone(first["specific_force_com_m_s2"])
            self.assertIsNone(first["linear_acceleration_world_m_s2"])
            self.assertIsNone(first["sample_delta_t_s"])
            world.command_joints((0.1,0.2,0.3,0.1,0.2,0.1))
            world.step(24)
            second=imu.sample(link_index=5)
            self.assertTrue(second["acceleration_is_valid"])
            self.assertAlmostEqual(second["sample_delta_t_s"],24/240,places=12)
            self.assertEqual(second["physics_step_index"],24)
            self.assertEqual(len(second["angular_velocity_com_rad_s"]),3)
            self.assertEqual(len(second["specific_force_com_m_s2"]),3)
            self.assertFalse(second["sensor_calibrated"])
            self.assertFalse(second["sensor_noise_added"])
            for vec in ("angular_velocity_com_rad_s",
                        "specific_force_com_m_s2",
                        "linear_acceleration_world_m_s2"):
                self.assertTrue(all(math.isfinite(x) for x in second[vec]))
            same_step=imu.sample(link_index=5)
            self.assertFalse(same_step["acceleration_is_valid"])
            self.assertIsNone(same_step["specific_force_com_m_s2"])
            world.step(1)
            third=imu.sample(link_index=5)
            self.assertTrue(third["acceleration_is_valid"])
            self.assertAlmostEqual(third["sample_delta_t_s"],1/240)

    def test_link_index_bounds_and_separate_link_sampling(self):
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        from dsiq_native.inertial_sensor import IdealBulletInertialProbe
        with BulletPlayground(BulletSettings(payload_count=0)) as world:
            imu=IdealBulletInertialProbe(world)
            for invalid in (-1,6,True,2.5):
                with self.subTest(invalid=invalid):
                    with self.assertRaises(ValueError):
                        imu.sample(link_index=invalid)
            self.assertFalse(imu.sample(link_index=0)["acceleration_is_valid"])
            world.step(5)
            self.assertTrue(imu.sample(link_index=0)["acceleration_is_valid"])
            self.assertFalse(imu.sample(link_index=1)["acceleration_is_valid"])


if __name__=="__main__":
    unittest.main()
