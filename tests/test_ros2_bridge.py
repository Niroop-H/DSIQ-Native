"""Read-only ROS 2 message contracts from actual Bullet physics measurements."""
import importlib.util
import math
import unittest

from dsiq_native.ros2_bridge import RosBridgeSettings,preview_ros_contract,_sim_stamp

HAS_BULLET=importlib.util.find_spec("pybullet") is not None


class RosContractsWithoutMiddlewareTests(unittest.TestCase):
    def test_reject_excessive_streaming_and_invalid_scan_configs(self):
        for bad in (
            {"frames":1201},{"frames":0},
            {"steps_per_frame":25},{"steps_per_frame":0},
            {"lidar_rays":361},{"lidar_rays":3},
            {"scan_range_m":float("nan")},{"scan_range_m":1000.},
        ):
            with self.subTest(bad=bad),self.assertRaises(ValueError):
                RosBridgeSettings(**bad).validate()

    def test_clock_stamp_is_strictly_integer_nanoseconds(self):
        self.assertEqual(_sim_stamp(1.125),
                         {"sec":1,"nanosec":125000000})
        self.assertEqual(_sim_stamp(0.),{"sec":0,"nanosec":0})
        with self.assertRaises(ValueError):
            _sim_stamp(float("inf"))
        with self.assertRaises(ValueError):
            _sim_stamp(-1.)


@unittest.skipUnless(HAS_BULLET,"Optional Bullet physics backend required")
class ActualBulletRosContractTests(unittest.TestCase):
    def test_joint_feedback_lidar_and_clock_come_from_actual_physics(self):
        config=RosBridgeSettings(frames=1,steps_per_frame=12,
                                 lidar_rays=24,scan_range_m=4.)
        data=preview_ros_contract(config)
        self.assertEqual(data["schema"],"dsiq-ros2-preview-v1")
        self.assertEqual(data["joint_state"]["ros_type"],
                         "sensor_msgs/msg/JointState")
        self.assertEqual(data["laser_scan"]["ros_type"],
                         "sensor_msgs/msg/LaserScan")
        self.assertEqual(data["simulation_clock"]["ros_type"],
                         "rosgraph_msgs/msg/Clock")
        self.assertEqual(len(data["joint_state"]["position"]),6)
        self.assertEqual(len(data["joint_state"]["velocity"]),6)
        self.assertEqual(len(data["joint_state"]["effort"]),6)
        self.assertEqual(len(data["laser_scan"]["ranges"]),24)
        self.assertEqual(data["source"]["step_index"],12)
        self.assertEqual(data["simulation_clock"]["clock"],
                         {"sec":0,"nanosec":50000000})
        self.assertAlmostEqual(data["laser_scan"]["angle_increment"],
                               2*math.pi/24)
        self.assertFalse(data["source"]["hardware_control_enabled"])
        self.assertFalse(data["source"]["ros2_actuator_subscribers_enabled"])
        self.assertFalse(data["source"]["tf_published"])
        for value in data["laser_scan"]["ranges"]:
            if value is not None:
                self.assertTrue(math.isfinite(value))
                self.assertLessEqual(value,4.)

    def test_preview_does_not_require_ros2_installation(self):
        data=preview_ros_contract(RosBridgeSettings(
            frames=1,steps_per_frame=1,lidar_rays=4
        ))
        self.assertIn("joint_state",data)
        self.assertIn("no_hit_representation",data["laser_scan"])


if __name__=="__main__":
    unittest.main()
