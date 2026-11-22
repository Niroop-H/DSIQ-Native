"""Export a semantic DSIQ reference environment as usable OpenUSD with PBR metadata.

Facility components are actual named authoring primitives with fixed collision,
structured semantics, material bindings and metric transforms. These are NOT
scanned CAD or photometric measurements. A separately installed Kit renderer
may produce higher quality images, but RTX3050 support remains unverified.
"""
from __future__ import annotations
from pathlib import Path
import re


def export_reference_environment(parts, scene_file: str | Path, preset: str) -> dict:
    from .usd_stage import new_stage, _load
    try:
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade
    except ImportError as exc:
        raise RuntimeError('Pixar OpenUSD required: pip install -e ".[usd]"') from exc

    parts=list(parts)
    if len(parts)>200:
        raise ValueError("Environment part count exceeds 200-piece authoring budget")
    if preset not in ("open","robotics_cell","cleanroom_bay","logistics_bay"):
        raise ValueError("Unknown reference environment preset")
    result=new_stage(scene_file)
    stage=_load(scene_file)
    root=stage.GetPrimAtPath("/World/Environment")
    root.SetCustomDataByKey("dsiq_environment_preset",preset)
    root.SetCustomDataByKey("dsiq_fidelity","REFERENCE_ONLY_NOT_CERTIFIED_MACHINE_CAD")
    UsdGeom.Scope.Define(stage,"/World/Looks")
    cache={}
    solids=0
    for index,part in enumerate(parts):
        safe=re.sub(r"[^A-Za-z0-9_]","_",part.name)
        path=f"/World/Environment/Part_{index:03d}_{safe[:48]}"
        if part.shape=="box":
            geom=UsdGeom.Cube.Define(stage,path)
            geom.CreateSizeAttr(1.0)
        elif part.shape=="cylinder":
            geom=UsdGeom.Cylinder.Define(stage,path)
            geom.CreateRadiusAttr(.5)
            geom.CreateHeightAttr(1.0)
        else:
            raise ValueError(f"Unsupported USD environment component: {part.shape}")
        geom.AddTranslateOp().Set(Gf.Vec3d(*part.center_m))
        if part.shape=="cylinder":
            geom.AddRotateXOp().Set(90.0)
        geom.AddScaleOp().Set(Gf.Vec3f(*part.dimensions_m))
        prim=geom.GetPrim()
        prim.SetCustomDataByKey("dsiq_role",part.role)
        prim.SetCustomDataByKey("dsiq_part_name",part.name)
        prim.SetCustomDataByKey("dsiq_calibration","NONE_REFERENCE_CONSTRUCTION")
        if part.solid:
            UsdPhysics.CollisionAPI.Apply(prim)
            solids+=1
        color=tuple(round(float(v),4) for v in part.color)
        metal=any(token in part.role.lower() for token in (
            "beam","frame","rail","tray","post","rack","housing",
            "stanchion","structural","stiffener","conveyor",
        ))
        cache_key=(color,metal)
        if cache_key not in cache:
            material=UsdShade.Material.Define(
                stage,f"/World/Looks/ReferenceMaterial_{len(cache):03d}"
            )
            shader=UsdShade.Shader.Define(stage,str(material.GetPath())+"/PreviewSurface")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput(
                "diffuseColor",Sdf.ValueTypeNames.Color3f
            ).Set(Gf.Vec3f(*color[:3]))
            shader.CreateInput(
                "opacity",Sdf.ValueTypeNames.Float
            ).Set(float(color[3]))
            shader.CreateInput(
                "metallic",Sdf.ValueTypeNames.Float
            ).Set(.50 if metal else .05)
            shader.CreateInput(
                "roughness",Sdf.ValueTypeNames.Float
            ).Set(.40 if metal else .65)
            material.CreateSurfaceOutput().ConnectToSource(
                shader.ConnectableAPI(),"surface"
            )
            cache[cache_key]=material
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(cache[cache_key])
    stage.GetRootLayer().Save()
    return {
        "stage":str(Path(scene_file).resolve()),
        "preset":preset,"prims_authored":len(parts),
        "static_colliders":solids,"usd_preview_materials":len(cache),
        "meters_per_unit":1.0,"up_axis":"Z",
        "geometry_class":"reference_facility_architecture_not_oem_cad",
        "rendering":"AUTHORING_ASSETS_NOT_RTX_RUNTIME",
        "physics":"STATIC_COLLISION_SCHEMAS_NOT_COMMISIONED_PHYSICS",
    }
