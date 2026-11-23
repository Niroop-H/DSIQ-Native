"""Time-sampled OpenUSD telemetry authored from actual Bullet physics.

This is NOT synthetic manufacturer geometry, nor a PhysX or RTX renderer.
It serializes measured world-space rigid/link transforms and joint feedback
to interoperable, independently readable USD time samples in metric Z-up.
"""
from __future__ import annotations

import math
from pathlib import Path
import re

from .bullet_runtime import PHYSICS_DT, BulletPlayground
from .usd_stage import _load, _existing_layer, new_stage

_SAFE = re.compile(r"[^A-Za-z0-9_]")


def _name(raw: str, prefix="Part") -> str:
    val=_SAFE.sub("_",raw)
    if not val or not (val[0].isalpha() or val[0]=="_"):
        val=prefix+"_"+val
    return val[:70]


def _pose_ops(prim):
    from pxr import UsdGeom
    xform=UsdGeom.Xform(prim)
    return (
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble),
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble),
    )


def _put_pose(ops, position, orientation, timecode):
    from pxr import Gf
    if len(position)!=3 or len(orientation)!=4:
        raise ValueError("Bullet pose requires 3 XYZ and 4 XYZW quaternion values")
    if not all(math.isfinite(x) for x in (*position,*orientation)):
        raise ArithmeticError("Solver yielded nonfinite pose")
    trans,rot=ops
    trans.Set(Gf.Vec3d(*[float(v) for v in position]),timecode)
    q=orientation
    rot.Set(
        Gf.Quatd(float(q[3]),Gf.Vec3d(float(q[0]),float(q[1]),float(q[2]))),
        timecode,
    )


def record_bullet_usd(world: BulletPlayground, scene_file: str | Path,
                      *, frames: int=12, steps_per_frame: int=8) -> dict:
    """Capture measured physics state with a bounded real USD temporal archive."""
    if type(frames) is not int or not 1<=frames<=120:
        raise ValueError("frames must be in [1,120]")
    if type(steps_per_frame) is not int or not 1<=steps_per_frame<=24:
        raise ValueError("steps_per_frame must be in [1,24]")
    try:
        from pxr import Gf, Sdf, Usd, UsdGeom
    except ImportError as exc:
        raise RuntimeError('Install actual OpenUSD bindings: pip install -e ".[usd]"') from exc
    scene=Path(scene_file).resolve()
    new_stage(scene)
    stage=_load(scene)
    stage.SetTimeCodesPerSecond(240.)
    stage.SetFramesPerSecond(240./steps_per_frame)
    start=world.step_index
    stage.SetStartTimeCode(start+steps_per_frame)
    stage.SetEndTimeCode(start+frames*steps_per_frame)
    root=UsdGeom.Scope.Define(stage,"/World/Telemetry").GetPrim()
    root.SetCustomDataByKey("dsiq_recording_source","ACTUAL_PYBULLET_SOLVER")
    root.SetCustomDataByKey("dsiq_geometry_included",False)
    root.SetCustomDataByKey("dsiq_oem_calibrated",False)
    root.SetCustomDataByKey("dsiq_solver_fixed_dt_seconds",PHYSICS_DT)
    root.SetCustomDataByKey("dsiq_steps_per_frame",steps_per_frame)
    p=world.p
    cid=world.client
    pose_entries=[]
    joint_entries=[]
    for group,body,joints,label in [
        ("Reference6R",world.robot,list(range(6)),"reference_6r"),
        *[(f"Imported_{_name(key)}",item["body_id"],
           list(range(item["joint_count"])),key)
          for key,item in sorted(world.imported_robots.items())],
    ]:
        parent=UsdGeom.Xform.Define(stage,f"/World/Telemetry/{group}").GetPrim()
        parent.SetCustomDataByKey("dsiq_asset_id",label)
        parent.SetCustomDataByKey("dsiq_source_body_id",body)
        joint_attr=parent.CreateAttribute(
            "dsiq:jointPositions",Sdf.ValueTypeNames.DoubleArray,custom=True,
        )
        joint_entries.append((body,joints,joint_attr))
        base_prim=UsdGeom.Xform.Define(
            stage,f"/World/Telemetry/{group}/Base"
        ).GetPrim()
        pose_entries.append((body,-1,_pose_ops(base_prim)))
        for index in joints:
            info=p.getJointInfo(body,index,physicsClientId=cid)
            link=_name(info[12].decode("utf8","replace"),f"link{index}")
            part=UsdGeom.Xform.Define(
                stage,f"/World/Telemetry/{group}/Link_{index:03d}_{link}"
            ).GetPrim()
            part.SetCustomDataByKey("dsiq_joint_name",
                info[1].decode("utf8","replace"))
            part.SetCustomDataByKey("dsiq_link_index",index)
            pose_entries.append((body,index,_pose_ops(part)))
    if len(world.payloads)>40:
        raise ValueError("Exceeded supported payload capture budget")
    for i,body in enumerate(world.payloads):
        obj=UsdGeom.Xform.Define(
            stage,f"/World/Telemetry/Payload_{i:03d}"
        ).GetPrim()
        obj.SetCustomDataByKey("dsiq_asset_class","test_fixture")
        pose_entries.append((body,-1,_pose_ops(obj)))
    for _ in range(frames):
        world.step(steps_per_frame)
        tc=Usd.TimeCode(world.step_index)
        for body,index,ops in pose_entries:
            if index==-1:
                position,orientation=p.getBasePositionAndOrientation(
                    body,physicsClientId=cid,
                )
            else:
                state=p.getLinkState(
                    body,index,computeForwardKinematics=True,
                    physicsClientId=cid,
                )
                position,orientation=state[4],state[5]
            _put_pose(ops,position,orientation,tc)
        for body,joints,attribute in joint_entries:
            measured=[float(p.getJointState(
                body,j,physicsClientId=cid
            )[0]) for j in joints]
            if not all(math.isfinite(x) for x in measured):
                raise ArithmeticError("Nonfinite joint feedback in USD recorder")
            attribute.Set(measured,tc)
    if not stage.GetRootLayer().Save():
        raise RuntimeError("Failed to save actual OpenUSD animation layer")
    return {
        "file":str(scene),
        "usd_schema":"measured_world_space_kinematic_telemetry_v1",
        "captured_frames":frames,
        "steps_per_frame":steps_per_frame,
        "start_step":start+steps_per_frame,
        "end_step":world.step_index,
        "pose_tracks":len(pose_entries),
        "articulation_tracks":len(joint_entries),
        "meters_per_unit":1.0,
        "up_axis":"Z",
        "source_engine":"pybullet_cpu",
        "visual_meshes_exported":False,
        "physics_certified":False,
    }


def inspect_bullet_usd(scene_file: str | Path) -> dict:
    """Reopen the authored USD and verify time-sampled solver measurements."""
    from pxr import Usd, UsdGeom
    stage=_load(_existing_layer(scene_file))
    root=stage.GetPrimAtPath("/World/Telemetry")
    if not root or root.GetCustomDataByKey("dsiq_recording_source")!="ACTUAL_PYBULLET_SOLVER":
        raise ValueError("Not an authored DSIQ actual-Bullet USD telemetry recording")
    if (abs(UsdGeom.GetStageMetersPerUnit(stage)-1.)>1e-9 or
            str(UsdGeom.GetStageUpAxis(stage))!="Z"):
        raise ValueError("Telemetry USD must be canonical metric Z-up")
    joint_tracks=0
    pose_tracks=0
    times=set()
    for prim in stage.Traverse():
        if not prim.GetPath().HasPrefix(root.GetPath()):
            continue
        positions=prim.GetAttribute("dsiq:jointPositions")
        if positions:
            joint_tracks+=1
            times.update(positions.GetTimeSamples())
        op=prim.GetAttribute("xformOp:translate")
        if op:
            pose_tracks+=1
            times.update(op.GetTimeSamples())
            orient=prim.GetAttribute("xformOp:orient")
            if not orient:
                raise ValueError("Telemetry pose track has no orientation")
            if op.GetTimeSamples()!=orient.GetTimeSamples():
                raise ValueError("Telemetry pose XYZ/quaternion times differ")
    if not pose_tracks or not joint_tracks or not times:
        raise ValueError("No measured Bullet pose/joint animation tracks exist")
    if max(times)>stage.GetEndTimeCode() or min(times)<stage.GetStartTimeCode():
        raise ValueError("Recorded keyframes exceed stage playback interval")
    return {
        "file":str(Path(scene_file).resolve()),
        "pose_tracks":pose_tracks,
        "joint_tracks":joint_tracks,
        "keyframe_times":sorted(times),
        "start_time_code":stage.GetStartTimeCode(),
        "end_time_code":stage.GetEndTimeCode(),
        "meters_per_unit":1.0,
        "up_axis":"Z",
        "source":"pybullet_cpu_measured",
        "visual_oem_geometry_included":False,
    }
