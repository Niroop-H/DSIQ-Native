"""OpenUSD animated geometry generated from hash-locked URDF CAD triangles.

Unlike primitive-only telemetry, authoring uses EXACT source mesh vertices
and triangle faces (subject to triangulation by original mesh format and
trimesh parser), with source URDF visual transforms and measured Bullet link
world poses. No OEM accuracy or renderer photometric calibration is implied.
"""
from __future__ import annotations

import math
from pathlib import Path
import re

from .cad_geometry import extract_verified_urdf_geometry
from .usd_stage import new_stage, _load
from .urdf_bridge import validate_robot_manifest

_SAFE = re.compile(r"[^A-Za-z0-9_]")


def _safe(raw: str):
    s=_SAFE.sub("_",raw)
    return s if s and (s[0].isalpha() or s[0]=="_") else "Part_"+s


def record_source_cad_usd(world, manifest_file: str|Path, output_file: str|Path,
                          *, frames: int=6, steps_per_frame: int=12) -> dict:
    if type(frames) is not int or not 1<=frames<=120:
        raise ValueError("CAD USD frames must be in [1,120]")
    if type(steps_per_frame) is not int or not 1<=steps_per_frame<=24:
        raise ValueError("CAD USD step-per-frame must be in [1,24]")
    from pxr import Gf, Usd, UsdGeom
    geometry=extract_verified_urdf_geometry(manifest_file)
    manifest,inspection=validate_robot_manifest(manifest_file)
    aid=manifest["asset_id"]
    model=world.imported_robots.get(aid)
    if model is None or model["joint_count"]!=len(inspection.joints):
        raise ValueError("Only currently loaded matching verified Bullet URDF is valid")
    target=Path(output_file).resolve()
    new_stage(target)
    stage=_load(target)
    stage.SetTimeCodesPerSecond(240.)
    stage.SetFramesPerSecond(240./steps_per_frame)
    start=world.step_index
    stage.SetStartTimeCode(start+steps_per_frame)
    stage.SetEndTimeCode(start+frames*steps_per_frame)
    top=UsdGeom.Scope.Define(stage,"/World/SourceCAD").GetPrim()
    top.SetCustomDataByKey("dsiq_source_manifest_id",aid)
    top.SetCustomDataByKey("dsiq_source_urdf_sha256",
                           geometry["source_urdf_sha256"])
    top.SetCustomDataByKey("dsiq_mesh_bytes_integrity_verified",True)
    top.SetCustomDataByKey("dsiq_physical_oem_calibrated",False)
    top.SetCustomDataByKey("dsiq_measured_link_world_transforms",True)
    p=world.p
    cid=world.client
    body=model["body_id"]
    link_indices={
        p.getJointInfo(body,j,physicsClientId=cid)[12].decode("utf8","replace"):j
        for j in range(model["joint_count"])
    }
    link_indices[model["root_link"]]=-1
    objects={}
    faces=0
    for n,part in enumerate(geometry["visual_parts"]):
        lname=part.link_name
        if lname not in link_indices:
            raise ValueError("Source geometry link missing in live Bullet body")
        if lname not in objects:
            path="/World/SourceCAD/Link_"+_safe(lname)
            xf=UsdGeom.Xform.Define(stage,path)
            xf.GetPrim().SetCustomDataByKey("dsiq_original_urdf_link",lname)
            xf.GetPrim().SetCustomDataByKey("dsiq_actual_bullet_joint_index",
                                            link_indices[lname])
            tx=xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
            rot=xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
            objects[lname]=(link_indices[lname],tx,rot,path)
        mesh=UsdGeom.Mesh.Define(stage,
                                objects[lname][3]+"/Visual_"+str(n).zfill(4))
        mesh.CreatePointsAttr([Gf.Vec3f(*point) for point in part.vertices])
        mesh.CreateFaceVertexCountsAttr([3]*len(part.faces))
        mesh.CreateFaceVertexIndicesAttr([idx for f in part.faces for idx in f])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*part.rgba[:3])])
        mesh.CreateDisplayOpacityAttr([float(part.rgba[3])])
        mins=[min(x[k] for x in part.vertices) for k in range(3)]
        maxs=[max(x[k] for x in part.vertices) for k in range(3)]
        mesh.CreateExtentAttr([Gf.Vec3f(*mins),Gf.Vec3f(*maxs)])
        mesh.GetPrim().SetCustomDataByKey("dsiq_source_visual",part.visual_name)
        mesh.GetPrim().SetCustomDataByKey("dsiq_source_type",part.source)
        if part.source_sha256:
            mesh.GetPrim().SetCustomDataByKey("dsiq_source_mesh_sha256",
                                              part.source_sha256)
        faces+=len(part.faces)
    for _ in range(frames):
        world.step(steps_per_frame)
        code=Usd.TimeCode(world.step_index)
        for index,tx,rot,_ in objects.values():
            if index==-1:
                xyz,q=p.getBasePositionAndOrientation(body,physicsClientId=cid)
            else:
                link=p.getLinkState(body,index,computeForwardKinematics=True,
                                     physicsClientId=cid)
                xyz,q=link[4],link[5]
            if not all(math.isfinite(v) for v in (*xyz,*q)):
                raise ArithmeticError("Actual Bullet link pose is nonfinite")
            tx.Set(Gf.Vec3d(*map(float,xyz)),code)
            rot.Set(Gf.Quatd(q[3],Gf.Vec3d(*map(float,q[:3]))),code)
    if not stage.GetRootLayer().Save():
        raise RuntimeError("OpenUSD source CAD animation save failed")
    return {
        "file":str(target),"asset_id":aid,"link_tracks":len(objects),
        "visual_meshes":len(geometry["visual_parts"]),"source_triangles":faces,
        "time_samples":frames,"source_mesh_hashes_verified":True,
        "geometry_substituted":False,"measured_pose_source":"actual_bullet",
        "units":"meters","up_axis":"Z",
        "oem_dimensions_independently_verified":False,
    }
