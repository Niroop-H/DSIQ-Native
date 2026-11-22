"""Real raycast, RGB-D, segmentation and kinematic sensor gates."""
import importlib.util
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

HAS_BULLET = importlib.util.find_spec("pybullet") is not None


@unittest.skipUnless(HAS_BULLET, "Requires optional PyBullet physics backend")
class BulletSensorTests(unittest.TestCase):
    def setUp(self):
        from dsiq_native.bullet_runtime import BulletPlayground, BulletSettings
        from dsiq_native.sensors import BulletSensors
        self.world = BulletPlayground(BulletSettings(payload_count=0))
        self.addCleanup(self.world.close)
        self.sensors = BulletSensors(self.world)

    def test_lidar_reports_actual_collision_geometry(self):
        obstacle = self.world.spawn_box(
            mass_kg=2, side_m=.4, position_m=(2.,0.,1.)
        )
        scan = self.sensors.lidar_2d(origin_m=(0.,0.,1.),
                                     count=36, max_range_m=4.)
        self.assertEqual(scan["scan_count"], 36)
        self.assertEqual(scan["sensor_type"], "BULLET_IDEAL_GEOMETRIC_LIDAR_2D")
        self.assertFalse(scan["calibrated"])
        ray_x = scan["measurements"][0]
        self.assertTrue(ray_x["hit"])
        self.assertEqual(ray_x["body_id"], obstacle)
        self.assertGreater(ray_x["range_m"], 1.)
        self.assertLess(ray_x["range_m"], 2.)

    def test_rgbd_cpu_frame_and_png_are_real_files(self):
        with TemporaryDirectory() as folder:
            frame = self.sensors.camera_rgbd(width=64, height=48)
            self.assertEqual(len(frame.rgba), 64*48*4)
            self.assertEqual(len(frame.metric_depth_m), 64*48)
            self.assertEqual(len(frame.segmentation), 64*48)
            self.assertTrue(all(math.isfinite(x) and x>0 for x in frame.metric_depth_m))
            self.assertFalse(frame.metadata()["sensor_calibrated"])
            image = Path(frame.save_png(Path(folder) / "color.png"))
            self.assertTrue(image.is_file())
            self.assertEqual(image.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
            with self.assertRaises(FileExistsError):
                frame.save_png(image)
            archive = Path(frame.save_depth_npz(Path(folder) / "depth.npz"))
            self.assertTrue(archive.exists())
            import numpy as np
            with np.load(archive) as data:
                self.assertEqual(data["depth_m"].shape, (48,64))
                self.assertEqual(data["segmentation"].shape, (48,64))

    def test_kinematics_velocity_is_actual_solver_result(self):
        self.world.command_joints((0.,-.25,.35,0.,0.,0.))
        self.world.step(50)
        observation = self.sensors.robot_link_velocity()
        self.assertEqual(observation["link_index"], 5)
        self.assertEqual(len(observation["linear_velocity_m_s"]), 3)
        self.assertTrue(all(math.isfinite(x) for x in
                            observation["angular_velocity_rad_s"]))
        self.assertFalse(observation["calibrated"])

    def test_sensor_parameter_validation(self):
        with self.assertRaises(ValueError):
            self.sensors.lidar_2d(count=500)
        with self.assertRaises(ValueError):
            self.sensors.lidar_2d(max_range_m=float("nan"))
        with self.assertRaises(ValueError):
            self.sensors.camera_rgbd(width=5000)
        with self.assertRaises(ValueError):
            self.sensors.camera_rgbd(eye_m=(0,0,0), look_at_m=(0,0,0))
        with self.assertRaises(ValueError):
            self.sensors.robot_link_velocity(link_index=6)


if __name__ == "__main__":
    unittest.main()
