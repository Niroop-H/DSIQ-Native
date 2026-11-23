"""Source-exact URDF primitive visuals animated by actual Bullet link poses.

Only primitives with *precisely described* geometry are supported. If a URDF
requires STL/OBJ/DAE conversion we FAIL CLOSED rather than fabricate OEM mesh
geometry. All visual dimensions and offsets are parsed from the verified URDF.
The stage stores each link's measured WORLD pose (asset root is identity).
"""
from __future__ import annotations

import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

from .bullet_runtime import PHYSICS_DT
from .urdf_bridge import validate_robot_manifest
from .usd_stage import new_stage, _load

_ID=re.compile(r"[^A-Za-z0-9_]")


def _triplet(raw, default="0 0 0"):
    vals=[float(x) for x in (raw or default).split()]
    if len(vals)!=3 or any(not math.isfinite(x) for x in vals):
        raise ValueError("Invalid source URDF visual XYZ/RPY")
    return vals


def _create_visual(stage, path, visual, p):
    from pxr import Gf, UsdGeom
    origin=visual.find("origin")
    xyz=_triplet(origin.get("xyz") if origin is not None else None)
    rpy=_triplet(origin.get("rpy") if origin is not None else None)
    holder=UsdGeom.Xform.Define(stage,path)
    holder.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*xyz))
    q=p.getQuaternionFromEuler(rpy)
    holder.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(float(q[3]),Gf.Vec3d(float(q[0]),float(q[1]),float(q[2])))
    )
    geom=visual.find("geometry")
    if geom is None or len(geom)!=1:
        raise ValueError("URDF source visual has no unambiguous geometry")
    shape=geom[0]
    path=path+"/SourceGeometry"
    if shape.tag=="box":
        dims=_triplet(shape.get("size"))
        if any(x<=0 for x in dims):
            raise ValueError("Source box size must be positive")
        obj=UsdGeom.Cube.Define(stage,path)
        obj.CreateSizeAttr(1.)
        obj.AddScaleOp().Set(Gf.Vec3f(*dims))
    elif shape.tag=="cylinder":
        radius=float(shape.get("radius"))
        length=float(shape.get("length"))
        if not math.isfinite(radius) or not math.isfinite(length) or min(radius,length)<=0:
            raise ValueError("URDF cylinder dimensions invalid")
        obj=UsdGeom.Cylinder.Define(stage,path)
        obj.CreateRadiusAttr(radius)
        obj.CreateHeightAttr(length)
    elif shape.tag=="sphere":
        radius=float(shape.get("radius"))
        if not math.isfinite(radius) or radius<=0:
            raise ValueError("URDF sphere radius invalid")
        obj=UsdGeom.Sphere.Define(stage,path)
        obj.CreateRadiusAttr(radius)
    else:
        raise ValueError(
            f"URDF visual geometry {shape.tag!r} cannot be converted exactly. "
            "Provide a rights-verified USD visual asset instead of substituting a box."
        )
    material=visual.find("material")
    if material is not None:
        color=material.find("color")
        if color is not None:
            rgba=[float(x) for x in color.get("rgba","").split()]
            if len(rgba)!=4 or any(not math.isfinite(v) or not 0<=v<=1 for v in rgba):
                raise ValueError("Invalid source URDF RGBA material")
            obj.CreateDisplayColorAttr([Gf.Vec3f(*rgba[:3])])
            obj.CreateDisplayOpacityAttr([float(rgba[3])])
    obj.GetPrim().SetCustomDataByKey("dsiq_visual_source","SOURCE_URDF_VERIFIED")
    return shape.tag


def record_urdf_visual_usd(world, manifest_path: str|Path, scene_file: str|Path,
                           *, frames: int=8, steps_per_frame: int=12) -> dict:
    """Create actual original machine geometry and time-sampled rigid-link poses."""
    if type(frames) is not int or not 1<=frames<=120:
        raise ValueError("frames must be 1..120")
    if type(steps_per_frame) is not int or not 1<=steps_per_frame<=24:
        raise ValueError("steps_per_frame must be 1..24")
    from pxr import Gf, Usd, UsdGeom
    manifest,inspected=validate_robot_manifest(manifest_path)
    identity=manifest["asset_id"]
    if identity not in world.imported_robots:
        raise ValueError("Load the same verified URDF into Bullet before USD capture")
    imported=world.imported_robots[identity]
    if imported["joint_count"]!=len(inspected.joints):
        raise ValueError("URDF link count changed since import")
    source=ET.parse(inspected.path).getroot()
    linklist=source.findall("link")
    p=world.p
    cid=world.client
    body=imported["body_id"]
    link_name_to_index={
        p.getJointInfo(body,j,physicsClientId=cid)[12].decode("utf-8") : j
        for j in range(imported["joint_count"])
    }
    mapping={inspected.root_link:-1,**link_name_to_index}
    if len(mapping)!=len(linklist):
        raise ValueError("Bullet/URDF link identity mismatch; cannot bind visuals")
    # Validate unsupported mesh geometry BEFORE creating any output file.
    for part in linklist:
        for visual in part.findall("visual"):
            geom=visual.find("geometry")
            if geom is None or len(geom)!=1 or geom[0].tag not in ("box","sphere","cylinder"):
                raise ValueError(
                    "URDF has mesh or unsupported source geometry; "
                    "requires licensed native USD mesh conversion, not fabricated primitives"
                )
    target=Path(scene_file).resolve()
    new_stage(target)
    stage=_load(target)
    stage.SetTimeCodesPerSecond(240.)
    stage.SetFramesPerSecond(240./steps_per_frame)
    start=world.step_index
    stage.SetStartTimeCode(start+steps_per_frame)
    stage.SetEndTimeCode(start+frames*steps_per_frame)
    root=UsdGeom.Scope.Define(stage,"/World/ImportedURDFVisuals").GetPrim()
    root.SetCustomDataByKey("dsiq_asset_id",identity)
    root.SetCustomDataByKey("dsiq_source_sha256",manifest["geometry"]["sha256"])
    root.SetCustomDataByKey("dsiq_frame_convention","MEASURED_WORLD_SPACE_LINK_FRAMES")
    root.SetCustomDataByKey("dsiq_exact_source_mesh_conversion",False)
    prims=[]
    shapes={}
    for index,part in enumerate(linklist):
        linkname=part.get("name","")
        if linkname not in mapping:
            raise ValueError("Source URDF link is not present in actual Bullet model")
        safe=_ID.sub("_",linkname)
        if not safe or safe[0].isdigit():
            safe="Link_"+safe
        path=f"/World/ImportedURDFVisuals/Link_{index:03d}_{safe}"
        xform=UsdGeom.Xform.Define(stage,path)
        xform.GetPrim().SetCustomDataByKey("dsiq_original_urdf_link_name",linkname)
        xform.GetPrim().SetCustomDataByKey("dsiq_bullet_link_index",mapping[linkname])
        tx=xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
        orient=xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
        shape_names=[]
        for n,visual in enumerate(part.findall("visual")):
            shape_names.append(_create_visual(
                stage,f"{path}/Visual_{n:03d}",visual,p
            ))
        shapes[linkname]=shape_names
        prims.append((mapping[linkname],tx,orient,linkname))
    for _ in range(frames):
        world.step(steps_per_frame)
        tc=Usd.TimeCode(world.step_index)
        for index,tx,rot,name in prims:
            if index==-1:
                pos,quat=p.getBasePositionAndOrientation(body,physicsClientId=cid)
            else:
                link=p.getLinkState(
                    body,index,computeForwardKinematics=True,physicsClientId=cid
                )
                pos,quat=link[4],link[5]
            if any(not math.isfinite(v) for v in (*pos,*quat)):
                raise ArithmeticError("Bullet produced nonfinite visual pose")
            tx.Set(Gf.Vec3d(*map(float,pos)),tc)
            rot.Set(Gf.Quatd(float(quat[3]),Gf.Vec3d(
                float(quat[0]),float(quat[1]),float(quat[2])
            )),tc)
    if not stage.GetRootLayer().Save():
        raise RuntimeError("Failed to save visual OpenUSD animation")
    return {
        "file":str(target),"asset_id":identity,"source_sha256":manifest["geometry"]["sha256"],
        "link_tracks":len(prims),"visual_shapes":sum(len(v) for v in shapes.values()),
        "source_shape_types":shapes,"time_samples":frames,
        "start_step":start+steps_per_frame,"end_step":world.step_index,
        "units":"meters","up_axis":"Z","renderable_primitives":True,
        "converted_imported_meshes":False,"oem_calibration_verified":False,
        "world_link_poses_not_double_parented":True,
    }
