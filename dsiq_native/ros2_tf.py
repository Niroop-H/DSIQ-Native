"""Measured Bullet -> ROS 2 TF tree, with correct parent-relative transforms.

No inferred OEM calibration, no fake robot frames. Reference robot links
are independently measured world link frames from actual Bullet,
then converted into the base frame. Sensor origin is a world-frame fixture.
"""
from __future__ import annotations

import math
from .bullet_runtime import PHYSICS_DT


def capture_reference_tf(world) -> dict:
    p=world.p
    cid=world.client
    base_pos,base_q=p.getBasePositionAndOrientation(
        world.robot,physicsClientId=cid
    )
    inverse=p.invertTransform(base_pos,base_q)
    nanos=round(world.step_index*PHYSICS_DT*1_000_000_000)
    stamp={"sec":nanos//1_000_000_000,
           "nanosec":nanos%1_000_000_000}

    def transform(parent,child,position,quaternion):
        vals=[float(v) for v in (*position,*quaternion)]
        if (len(vals)!=7 or
            not all(math.isfinite(v) for v in vals)):
            raise ArithmeticError("Nonfinite Bullet transform")
        norm=math.sqrt(sum(v*v for v in vals[3:]))
        if abs(norm-1.)>1e-3:
            raise ArithmeticError("Bullet quaternion is not normalized")
        return {
            "header":{"stamp":stamp,"frame_id":parent},
            "child_frame_id":child,
            "transform":{
                "translation":{"x":vals[0],"y":vals[1],"z":vals[2]},
                "rotation":{"x":vals[3]/norm,"y":vals[4]/norm,
                            "z":vals[5]/norm,"w":vals[6]/norm},
            },
        }

    frames=[transform(
        "dsiq_world","dsiq_reference_base",base_pos,base_q
    )]
    for index in range(6):
        link=p.getLinkState(
            world.robot,index,computeForwardKinematics=True,
            physicsClientId=cid,
        )
        relative_pos,relative_q=p.multiplyTransforms(
            inverse[0],inverse[1],link[4],link[5]
        )
        frames.append(transform(
            "dsiq_reference_base",f"dsiq_reference_link_{index}",
            relative_pos,relative_q,
        ))
    frames.append(transform(
        "dsiq_world","dsiq_lidar_reference",
        (-1.5,0.,.7),(0.,0.,0.,1.)
    ))
    children=[t["child_frame_id"] for t in frames]
    if len(children)!=len(set(children)):
        raise RuntimeError("Duplicate ROS child_frame_id in generated TF tree")
    return {
        "ros_type":"tf2_msgs/msg/TFMessage",
        "transforms":frames,
        "transform_count":len(frames),
        "source":"actual_bullet_world_link_frames",
        "tree_root":"dsiq_world",
        "calibrated":False,
        "physical_robot_control":False,
    }
