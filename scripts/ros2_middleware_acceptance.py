"""Live ROS 2 DDS acceptance using REAL Bullet-captured upstream CI data.

This test runs in an official ROS Jazzy container, publishes the exact
measured simulator preview to four ROS 2 topics, and validates receipt
through independent rclpy listener nodes. It does not command hardware
or certify interoperability with proprietary robot controllers.
"""
from __future__ import annotations
import json
import math
from pathlib import Path
import sys
import time


def run(path: str) -> dict:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import JointState,LaserScan
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import TransformStamped

    data=json.loads(Path(path).read_text(encoding="utf-8"))
    assert data["source"]["step_index"]>0
    assert data["source"]["hardware_control_enabled"] is False
    assert data["tf_message"]["source"]=="actual_bullet_world_link_frames"
    expected_stamp=data["simulation_clock"]["clock"]
    assert data["tf_message"]["transform_count"]==8

    rclpy.init()
    sender=Node("dsiq_validation_sender")
    receiver=Node("dsiq_validation_receiver")
    executor=SingleThreadedExecutor()
    executor.add_node(sender)
    executor.add_node(receiver)
    topics=[
        ("/dsiq/validation/joint_states",JointState),
        ("/dsiq/validation/scan",LaserScan),
        ("/dsiq/validation/clock",Clock),
        ("/dsiq/validation/tf",TFMessage),
    ]
    messages={}
    def recv(topic):
        def callback(msg):
            messages[topic]=msg
        return callback
    publishers={}
    subs=[]
    try:
        for topic,typ in topics:
            publishers[topic]=sender.create_publisher(typ,topic,10)
            subs.append(receiver.create_subscription(typ,topic,recv(topic),10))
        js=JointState()
        j=data["joint_state"]
        js.header.frame_id=j["header"]["frame_id"]
        js.header.stamp.sec=expected_stamp["sec"]
        js.header.stamp.nanosec=expected_stamp["nanosec"]
        js.name=list(j["name"])
        js.position=list(j["position"])
        js.velocity=list(j["velocity"])
        js.effort=list(j["effort"])

        laser=LaserScan()
        scan=data["laser_scan"]
        laser.header.frame_id=scan["header"]["frame_id"]
        laser.header.stamp.sec=expected_stamp["sec"]
        laser.header.stamp.nanosec=expected_stamp["nanosec"]
        for key in ("angle_min","angle_max","angle_increment",
                    "time_increment","scan_time","range_min","range_max"):
            setattr(laser,key,float(scan[key]))
        laser.ranges=[float(x) if x is not None else float("inf")
                      for x in scan["ranges"]]
        laser.intensities=[]

        clock=Clock()
        clock.clock.sec=expected_stamp["sec"]
        clock.clock.nanosec=expected_stamp["nanosec"]

        tf=TFMessage()
        for frame in data["tf_message"]["transforms"]:
            msg=TransformStamped()
            msg.header.frame_id=frame["header"]["frame_id"]
            msg.header.stamp.sec=expected_stamp["sec"]
            msg.header.stamp.nanosec=expected_stamp["nanosec"]
            msg.child_frame_id=frame["child_frame_id"]
            pos=frame["transform"]["translation"]
            rotation=frame["transform"]["rotation"]
            for axis in ("x","y","z"):
                setattr(msg.transform.translation,axis,float(pos[axis]))
            for axis in ("x","y","z","w"):
                setattr(msg.transform.rotation,axis,float(rotation[axis]))
            tf.transforms.append(msg)

        values=[js,laser,clock,tf]
        deadline=time.monotonic()+12.
        while len(messages)!=4 and time.monotonic()<deadline:
            for (topic,_),value in zip(topics,values):
                publishers[topic].publish(value)
            executor.spin_once(timeout_sec=.1)
        if len(messages)!=4:
            raise RuntimeError(
                "ROS 2 DDS message discovery/delivery incomplete: "
                +str(sorted(messages))
            )
        received_j=messages[topics[0][0]]
        received_l=messages[topics[1][0]]
        received_c=messages[topics[2][0]]
        received_tf=messages[topics[3][0]]
        if list(received_j.name)!=list(j["name"]):
            raise AssertionError("ROS middleware changed joint names")
        for a,b in zip(received_j.position,j["position"]):
            if abs(a-b)>1e-10:
                raise AssertionError("ROS DDS joint position mismatch")
        if len(received_l.ranges)!=len(scan["ranges"]):
            raise AssertionError("ROS DDS LiDAR beam count mismatch")
        if (received_c.clock.sec!=expected_stamp["sec"] or
                received_c.clock.nanosec!=expected_stamp["nanosec"]):
            raise AssertionError("ROS simulation clock not transmitted intact")
        if [x.child_frame_id for x in received_tf.transforms] != [
            x["child_frame_id"] for x in data["tf_message"]["transforms"]
        ]:
            raise AssertionError("ROS TF child frames not delivered intact")
        if not all(abs(sum(
            getattr(x.transform.rotation,a)**2 for a in ("x","y","z","w")
        )-1.)<1e-3 for x in received_tf.transforms):
            raise AssertionError("ROS TF quaternion normalization invalid")
        return {
            "schema":"dsiq-ros2-jazzy-dds-validation-v1",
            "middleware":"rclpy_live_DDS",
            "measured_bullet_input":True,
            "topics_delivered":len(messages),
            "joint_count":len(received_j.name),
            "lidar_beams":len(received_l.ranges),
            "tf_frames":len(received_tf.transforms),
            "clock_stamp_verified":True,
            "real_robot_controllers_tested":False,
            "hardware_qualified":False,
        }
    finally:
        executor.shutdown()
        sender.destroy_node()
        receiver.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    if len(sys.argv)!=2:
        raise SystemExit("usage: ros2_middleware_acceptance.py REAL_BULLET_PREVIEW.json")
    print(json.dumps(run(sys.argv[1]),sort_keys=True))
