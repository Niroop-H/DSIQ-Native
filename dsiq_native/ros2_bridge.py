"""Read-only ROS 2 interface for actual DSIQ Bullet robotics state.

The ROS 2 publisher runs only if a separately installed, compatible ROS 2
environment provides rclpy, sensor_msgs and rosgraph_msgs. CI verifies the
offline conversion contract; it does NOT claim live ROS 2 middleware tests.

No subscriber, actuator bridge, ROS controller, PLC write or remote
physical-robot command path exists here.
"""
from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass

from .bullet_runtime import BulletPlayground, BulletSettings, PHYSICS_DT
from .sensors import BulletSensors


@dataclass(frozen=True)
class RosBridgeSettings:
    frames: int = 50
    steps_per_frame: int = 12
    lidar_rays: int = 36
    scan_range_m: float = 5.

    def validate(self):
        for name, value, lo, hi in (
            ("frames",self.frames,1,1200),
            ("steps_per_frame",self.steps_per_frame,1,24),
            ("lidar_rays",self.lidar_rays,4,360),
        ):
            if type(value) is not int or not lo<=value<=hi:
                raise ValueError(f"{name} must be integer in [{lo},{hi}]")
        if (not isinstance(self.scan_range_m,(int,float)) or
                not math.isfinite(self.scan_range_m) or
                not .1<=self.scan_range_m<=50.):
            raise ValueError("scan_range_m must be finite between 0.1 and 50m")


def _sim_stamp(sim_seconds: float) -> dict:
    if not math.isfinite(sim_seconds) or sim_seconds<0:
        raise ValueError("Simulation timestamp must be finite and nonnegative")
    ns=int(round(sim_seconds*1_000_000_000))
    return {"sec":ns//1_000_000_000,
            "nanosec":ns%1_000_000_000}


def capture_ros_contract(world: BulletPlayground, settings: RosBridgeSettings) -> dict:
    """Produce real ROS standard message field values, without requiring ROS."""
    settings.validate()
    cid=world.client
    p=world.p
    measured=[p.getJointState(world.robot,j,physicsClientId=cid)
              for j in range(6)]
    report=world.report()
    stamp=_sim_stamp(world.step_index*PHYSICS_DT)
    scan=BulletSensors(world).lidar_2d(
        origin_m=(-1.5,0.,.7),count=settings.lidar_rays,
        max_range_m=settings.scan_range_m,
    )
    step=2*math.pi/settings.lidar_rays
    ranges=[
        float(x["range_m"]) if x["hit"] else float("inf")
        for x in scan["measurements"]
    ]
    # JSON does not permit Infinity. Serialize missing returns to a
    # nullable preview; ROS LaserScan uses +inf as the standard no-return
    # range convention.
    preview_ranges=[
        float(v) if math.isfinite(v) else None for v in ranges
    ]
    return {
        "schema":"dsiq-ros2-preview-v1",
        "simulation_clock":{
            "ros_type":"rosgraph_msgs/msg/Clock",
            "clock":stamp,
        },
        "joint_state":{
            "ros_type":"sensor_msgs/msg/JointState",
            "header":{"stamp":stamp,"frame_id":"dsiq_reference_base"},
            "name":[f"reference_joint_{i+1}" for i in range(6)],
            "position":[float(x[0]) for x in measured],
            "velocity":[float(x[1]) for x in measured],
            "effort":[float(x[3]) for x in measured],
        },
        "laser_scan":{
            "ros_type":"sensor_msgs/msg/LaserScan",
            "header":{"stamp":stamp,"frame_id":"dsiq_lidar_reference"},
            "angle_min":0.,
            "angle_max":2*math.pi-step,
            "angle_increment":step,
            "time_increment":0.,
            "scan_time":settings.steps_per_frame*PHYSICS_DT,
            "range_min":.05,
            "range_max":float(settings.scan_range_m),
            "ranges":preview_ranges,
            "intensities":[],
            "no_hit_representation":"null in JSON preview; +inf in actual ROS LaserScan",
        },
        "source":{
            "engine":report["engine"],
            "measured_joint_feedback":True,
            "sensor":"BULLET_IDEAL_GEOMETRIC_LIDAR_2D",
            "joint_count":6,
            "step_index":world.step_index,
            "physically_qualified":False,
            "ros2_actuator_subscribers_enabled":False,
            "tf_published":False,
            "hardware_control_enabled":False,
        },
    }


def preview_ros_contract(settings: RosBridgeSettings) -> dict:
    settings.validate()
    with BulletPlayground(BulletSettings(payload_count=1)) as world:
        world.step(settings.steps_per_frame)
        return capture_ros_contract(world,settings)


def publish_ros2(settings: RosBridgeSettings) -> dict:
    """Publish real Bullet state via rclpy, with bounded runtime and no commands."""
    settings.validate()
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState, LaserScan
        from rosgraph_msgs.msg import Clock
    except ImportError as exc:
        raise RuntimeError(
            "A functioning ROS 2 Python environment is required. "
            "Source your supported ROS 2 installation, then run ros2-stream. "
            "No ROS 2 middleware or fallback fake publishing was started."
        ) from exc

    if rclpy.ok():
        raise RuntimeError(
            "DSIQ read-only ROS 2 command requires an independent ROS 2 process "
            "to avoid modifying another application's rclpy context."
        )
    rclpy.init(args=None)
    node = None
    published=0
    try:
        node=Node("dsiq_native_reference_sim")
        joints_pub=node.create_publisher(
            JointState,"/dsiq/joint_states",10
        )
        scan_pub=node.create_publisher(
            LaserScan,"/dsiq/scan",10
        )
        clock_pub=node.create_publisher(
            Clock,"/dsiq/clock",10
        )
        with BulletPlayground(BulletSettings(payload_count=1)) as world:
            for _ in range(settings.frames):
                if not rclpy.ok():
                    break
                start=time.monotonic()
                world.step(settings.steps_per_frame)
                snapshot=capture_ros_contract(world,settings)
                js=JointState()
                jc=snapshot["joint_state"]
                js.header.frame_id=jc["header"]["frame_id"]
                js.header.stamp.sec=jc["header"]["stamp"]["sec"]
                js.header.stamp.nanosec=jc["header"]["stamp"]["nanosec"]
                js.name=list(jc["name"])
                js.position=list(jc["position"])
                js.velocity=list(jc["velocity"])
                js.effort=list(jc["effort"])
                joints_pub.publish(js)

                lc=snapshot["laser_scan"]
                sensor=LaserScan()
                sensor.header.frame_id=lc["header"]["frame_id"]
                sensor.header.stamp.sec=lc["header"]["stamp"]["sec"]
                sensor.header.stamp.nanosec=lc["header"]["stamp"]["nanosec"]
                sensor.angle_min=float(lc["angle_min"])
                sensor.angle_max=float(lc["angle_max"])
                sensor.angle_increment=float(lc["angle_increment"])
                sensor.time_increment=0.
                sensor.scan_time=float(lc["scan_time"])
                sensor.range_min=float(lc["range_min"])
                sensor.range_max=float(lc["range_max"])
                sensor.ranges=[
                    float(v) if v is not None else float("inf")
                    for v in lc["ranges"]
                ]
                scan_pub.publish(sensor)

                clock=Clock()
                clock.clock.sec=jc["header"]["stamp"]["sec"]
                clock.clock.nanosec=jc["header"]["stamp"]["nanosec"]
                clock_pub.publish(clock)
                published+=1
                # Bounded wall-time pacing is not hard real-time scheduling.
                target_time=settings.steps_per_frame*PHYSICS_DT
                delay=max(0.,target_time-(time.monotonic()-start))
                if delay:
                    time.sleep(delay)
        return {
            "published_frames":published,
            "topics":["/dsiq/joint_states","/dsiq/scan","/dsiq/clock"],
            "middleware":"rclpy",
            "read_only":True,
            "physical_robot_control":False,
            "real_time_guarantee":False,
            "tf_published":False,
        }
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
