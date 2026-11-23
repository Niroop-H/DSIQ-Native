"""Exercise the REAL DSIQ ROS2 publisher process over Jazzy DDS transport.

Separate listener subscribes to the four actual DSIQ topics while the
production ros2-stream process executes real Bullet. This is a middleware
smoke test, NOT motor/controller/hardware interoperability testing.
"""
from __future__ import annotations
import json
import math
import subprocess
import sys
import time

def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState,LaserScan
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    rclpy.init()
    receiver=Node("dsiq_actual_publisher_acceptance")
    records={"joint_states":[],"scan":[],"clock":[],"tf":[]}
    subscriptions=[]
    for name,typ in (("joint_states",JointState),
                     ("scan",LaserScan),("clock",Clock),("tf",TFMessage)):
        def callback(msg,label=name):
            records[label].append(msg)
        subscriptions.append(receiver.create_subscription(
            typ,"/dsiq/"+name,callback,10
        ))
    args=[sys.executable,"-m","dsiq_native","ros2-stream",
          "--frames","110","--steps-per-frame","12","--lidar-rays","24"]
    proc=None
    try:
        proc=subprocess.Popen(args,stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE,text=True)
        deadline=time.monotonic()+16.
        while time.monotonic()<deadline:
            rclpy.spin_once(receiver,timeout_sec=.08)
            if all(len(x)>=2 for x in records.values()):
                break
            if proc.poll() is not None and time.monotonic()>deadline-12.:
                # Publisher may exit after 5.5 sec; continue briefly for DDS.
                continue
        if not all(len(x)>=2 for x in records.values()):
            raise RuntimeError(
                "Actual ros2-stream DDS delivery incomplete: "+str({
                    key:len(value) for key,value in records.items()
                })
            )
        stamps=[msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
                for msg in records["joint_states"]]
        if not stamps[0]<stamps[-1]:
            raise AssertionError("JointState timestamp did not progress with simulation")
        if not all(len(msg.name)==6 and len(msg.position)==6
                   for msg in records["joint_states"]):
            raise AssertionError("Reference joint data corrupted")
        if not all(len(msg.ranges)==24 for msg in records["scan"]):
            raise AssertionError("LaserScan ray count mismatch")
        if not all(len(msg.transforms)==8 for msg in records["tf"]):
            raise AssertionError("Reference TF tree incomplete")
        if not all(msg.clock.sec>=0 for msg in records["clock"]):
            raise AssertionError("Clock sent invalid timestamp")
        for msg in records["tf"]:
            if len({t.child_frame_id for t in msg.transforms})!=8:
                raise AssertionError("TF frame names are duplicated")
            for transform in msg.transforms:
                q=transform.transform.rotation
                if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-3:
                    raise AssertionError("ROS2 TF quaternion invalid")
        out,err=proc.communicate(timeout=15.)
        if proc.returncode!=0:
            raise RuntimeError("Actual DSIQ ROS2 publisher failed: "+err[-1500:])
        data=json.loads(out)
        if data.get("published_frames")!=110 or not data.get("read_only"):
            raise AssertionError("Publisher completion/count/read-only contract mismatch")
        result={
            "schema":"dsiq-ros2-real-publisher-dds-v1",
            "production_ros2_bridge_executed":True,
            "middleware":"rclpy_ros_jazzy",
            "real_bullet_simulation":True,
            "received_counts":{k:len(v) for k,v in records.items()},
            "verified_simulation_clock_progression":True,
            "verified_tf_quaternion_and_unique_frame_contract":True,
            "operator_hardware_commands_sent":False,
            "external_robots_integration_qualified":False,
        }
        print(json.dumps(result,indent=2))
    finally:
        if proc is not None and proc.poll() is None:
            proc.kill()
            proc.communicate()
        receiver.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
