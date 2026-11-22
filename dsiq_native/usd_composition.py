"""Real OpenUSD layered assembly workspaces and authored representation variants.

Omniverse parity gap addressed: external USD references, layered authoring,
editable transforms, and selected LOD representations. Every imported asset
must pass the existing SHA-256, rights, and mesh-topology checks. No geometry
is generated to impersonate a manufacturer's actual machine.

Not an entire Composer: no network collaboration, live multiuser merges,
universal CAD tessellation, or RTX/PhysX engine in this module.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
from typing import Sequence

from .asset_quality import load_manifest, validate_asset
from .usd_stage import _load, _existing_layer, new_stage

_ASSET_ID = re.compile(r"^[a-z][a-z0-9_]{2,63}$")
_PROJECT_SCHEMA = "dsiq-usd-workspace-v1"
_MAX_ASSETS = 32
_MAX_TRIANGLES_PER_SOURCE = 250000


def _pxr():
    try:
        from pxr import Gf, Sdf, Usd, UsdGeom
    except ImportError as exc:
        raise RuntimeError('Pixar OpenUSD required: pip install -e ".[usd]"') from exc
    return Gf, Sdf, Usd, UsdGeom


def new_workspace(directory: str | Path) -> dict:
    """Create three distinct, persistent authored layers and a canonical stage."""
    Gf, Sdf, Usd, UsdGeom = _pxr()
    folder = Path(directory).resolve()
    if folder.exists() and (not folder.is_dir() or any(folder.iterdir())):
        raise FileExistsError("New USD workspace directory must be empty")
    folder.mkdir(parents=True, exist_ok=True)
    layers = folder / "layers"
    layers.mkdir(exist_ok=False)
    stage_path = folder / "project.usda"
    new_stage(stage_path)
    equipment_path = layers / "equipment.usda"
    layout_path = layers / "layout.usda"
    eq = Sdf.Layer.CreateNew(str(equipment_path))
    layout = Sdf.Layer.CreateNew(str(layout_path))
    if eq is None or layout is None:
        raise RuntimeError("Unable to create composed USD sublayers")
    eq.Save()
    layout.Save()
    stage = _load(stage_path)
    root = stage.GetRootLayer()
    # Stronger first: transforms in layout override equipment authored opinions.
    root.subLayerPaths = ["layers/layout.usda", "layers/equipment.usda"]
    root.Save()
    stage = _load(stage_path)
    if len(stage.GetLayerStack()) < 3:
        raise RuntimeError("USD composition sublayers did not load")
    spec = {
        "schema":_PROJECT_SCHEMA,
        "project_file":"project.usda",
        "equipment_layer":"layers/equipment.usda",
        "layout_layer":"layers/layout.usda",
        "units":"meters",
        "up_axis":"Z",
        "fidelity":"authoring_workspace_not_oem_calibrated",
    }
    (folder / "workspace.json").write_text(json.dumps(spec,indent=2)+"\n",encoding="utf-8")
    return {"directory":str(folder),"root_usd":str(stage_path),
            "layers":list(root.subLayerPaths),"asset_count":0,
            "physics_engine_attached":False}


def _open(directory):
    Gf, Sdf, Usd, UsdGeom = _pxr()
    folder = Path(directory).resolve()
    metadata_file=folder/"workspace.json"
    if not metadata_file.is_file() or metadata_file.stat().st_size>16*1024:
        raise ValueError("Missing or oversized DSIQ workspace metadata")
    spec=json.loads(metadata_file.read_text(encoding="utf-8"))
    if not isinstance(spec,dict) or spec.get("schema")!=_PROJECT_SCHEMA:
        raise ValueError("Expected genuine DSIQ OpenUSD workspace")
    for key,expected in (
        ("project_file","project.usda"),
        ("equipment_layer","layers/equipment.usda"),
        ("layout_layer","layers/layout.usda"),
    ):
        if spec.get(key)!=expected:
            raise ValueError("Invalid or unsafe workspace layer layout")
    stage_path = _existing_layer(folder/"project.usda")
    stage=_load(stage_path)
    sub=stage.GetRootLayer().subLayerPaths
    if list(sub)!=["layers/layout.usda","layers/equipment.usda"]:
        raise ValueError("Root OpenUSD sublayer order or paths have changed")
    eq = Sdf.Layer.FindOrOpen(str(_existing_layer(folder/"layers/equipment.usda")))
    layout = Sdf.Layer.FindOrOpen(str(_existing_layer(folder/"layers/layout.usda")))
    if eq is None or layout is None:
        raise ValueError("Unable to load authored USD sublayers")
    if (abs(float(UsdGeom.GetStageMetersPerUnit(stage))-1.)>1e-9 or
            str(UsdGeom.GetStageUpAxis(stage))!="Z"):
        raise ValueError("Workspace stage is not metric Z-up")
    return folder,stage,eq,layout


def _validated_asset(manifest_path):
    _,_,Usd,UsdGeom=_pxr()
    manifest=Path(manifest_path).resolve()
    report=validate_asset(manifest)
    if not report.valid_for_import:
        raise ValueError("Asset provenance rejected: "+"; ".join(report.errors))
    m=load_manifest(manifest)
    raw_path=m["geometry"]["usd_file"]
    source=(manifest.parent/raw_path).resolve()
    if not source.is_relative_to(manifest.parent):
        raise ValueError("USD asset escapes its declared manifest directory")
    asset=_load(_existing_layer(source))
    default=asset.GetDefaultPrim()
    if not default or not default.IsValid():
        raise ValueError("Asset source must have a valid USD defaultPrim")
    expected=m["geometry"]
    if abs(float(UsdGeom.GetStageMetersPerUnit(asset)) -
            float(expected["meters_per_unit"]))>1e-9:
        raise ValueError("USD source metersPerUnit differs from manifest")
    if str(UsdGeom.GetStageUpAxis(asset))!=expected["up_axis"]:
        raise ValueError("USD source up axis differs from manifest")
    if abs(float(UsdGeom.GetStageMetersPerUnit(asset))-1.)>1e-9 or str(UsdGeom.GetStageUpAxis(asset))!="Z":
        raise ValueError("Canonical DSIQ workspace needs metric Z-up assets")
    meshes=triangles=0
    for subassembly in expected["subassemblies"]:
        if not asset.GetPrimAtPath(subassembly["prim_path"]):
            raise ValueError("Real declared mesh subassembly missing in USD")
    for prim in asset.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            mesh=UsdGeom.Mesh(prim)
            vertex=mesh.GetPointsAttr().Get()
            counts=mesh.GetFaceVertexCountsAttr().Get()
            indices=mesh.GetFaceVertexIndicesAttr().Get()
            if vertex and counts and indices and len(vertex)>=3 and len(indices)>=3:
                tris=sum(max(0,int(n)-2) for n in counts)
                if tris>0:
                    meshes+=1
                    triangles+=tris
    if meshes==0 or triangles==0:
        raise ValueError("Source USD has no actual triangulable Mesh data")
    if triangles>_MAX_TRIANGLES_PER_SOURCE:
        raise ValueError(
            "Source exceeds conservative 250k-triangle per-asset viewport admission"
        )
    return {
        "source":source,"manifest":m,"default_prim_path":str(default.GetPath()),
        "mesh_count":meshes,"triangle_count":triangles,
        "qualified":report.simulation_qualified,
    }


def add_verified_asset(
    workspace: str | Path,
    source_manifest: str | Path,
    *,
    alternative_manifest: str | Path | None = None,
) -> dict:
    """Add a verified high-detail USD source and optional verified low-detail LOD.

    Variant names describe *source representation*, not a fabricated proxy.
    Work is authored only into the equipment layer, never flattened into root.
    """
    Gf,Sdf,Usd,UsdGeom=_pxr()
    primary=_validated_asset(source_manifest)
    alternative=(_validated_asset(alternative_manifest)
                 if alternative_manifest is not None else None)
    asset_id=primary["manifest"]["asset_id"]
    if not _ASSET_ID.fullmatch(asset_id):
        raise ValueError("Invalid authored asset identifier")
    if alternative is not None and alternative["manifest"]["asset_id"]!=asset_id:
        raise ValueError("Representation variants must belong to identical asset ID")
    if alternative is not None and alternative["triangle_count"]>primary["triangle_count"]:
        raise ValueError("Low-memory representation must not exceed source triangle count")
    folder,stage,eq,layout=_open(workspace)
    target=f"/World/Equipment/{asset_id}"
    if stage.GetPrimAtPath(target):
        raise FileExistsError("Asset prim already exists: "+target)
    count=len([p for p in stage.GetPrimAtPath("/World/Equipment").GetChildren()
               if p.GetName() not in ("",)])
    if count>=_MAX_ASSETS:
        raise ValueError("Equipment workspace budget is 32 asset placements")
    stage.SetEditTarget(Usd.EditTarget(eq))
    prim=UsdGeom.Xform.Define(stage,target).GetPrim()
    prim.SetCustomDataByKey("dsiq_asset_id",asset_id)
    prim.SetCustomDataByKey("dsiq_model",primary["manifest"]["model"])
    prim.SetCustomDataByKey("dsiq_asset_class",primary["manifest"]["asset_class"])
    prim.SetCustomDataByKey("dsiq_physics_metadata_only",bool(primary["qualified"]))
    variant=prim.GetVariantSets().AddVariantSet("representation")
    choices={"source":primary}
    if alternative is not None:
        choices["low_memory"]=alternative
    for key,item in choices.items():
        variant.AddVariant(key)
        variant.SetVariantSelection(key)
        with variant.GetVariantEditContext():
            referenced=stage.GetPrimAtPath(target)
            relative=os.path.relpath(item["source"],eq.realPath and Path(eq.realPath).parent or folder)
            relative=relative.replace("\\","/")
            referenced.GetReferences().AddReference(relative,item["default_prim_path"])
    variant.SetVariantSelection("source")
    if not eq.Save():
        raise RuntimeError("Unable to save actual OpenUSD equipment sublayer")
    reopened=_load(folder/"project.usda")
    present=reopened.GetPrimAtPath(target)
    if not present or present.GetVariantSets().GetVariantSet("representation").GetVariantSelection()!="source":
        raise RuntimeError("USD representation composition did not persist")
    return {
        "asset_id":asset_id,"prim_path":target,
        "variant_choices":list(choices),
        "source_triangles":primary["triangle_count"],
        "source_meshes":primary["mesh_count"],
        "authoring_layer":str(folder/"layers/equipment.usda"),
        "verified_geometry":True,
        "physical_qualification":"metadata_not_independent_test",
    }


def set_representation(workspace,asset_id,choice):
    Gf,Sdf,Usd,UsdGeom=_pxr()
    if not _ASSET_ID.fullmatch(asset_id):
        raise ValueError("Invalid asset ID")
    folder,stage,eq,layout=_open(workspace)
    stage.SetEditTarget(Usd.EditTarget(eq))
    prim=stage.GetPrimAtPath(f"/World/Equipment/{asset_id}")
    if not prim:
        raise ValueError("Imported USD asset does not exist")
    var=prim.GetVariantSets().GetVariantSet("representation")
    options=var.GetVariantNames()
    if choice not in options:
        raise ValueError("Representation not authored or not available: "+str(choice))
    if not var.SetVariantSelection(choice):
        raise RuntimeError("OpenUSD variant selection was rejected")
    if not eq.Save():
        raise RuntimeError("OpenUSD equipment layer Save() failed")
    return {"asset_id":asset_id,"representation":choice,
            "available":list(options),"viewport_lod":"authored_variant_source"}


def place_asset(workspace,asset_id,position_m:Sequence[float],rotation_z_deg:float=0.):
    """Author reversible layout opinions in the independent stronger layout layer."""
    Gf,Sdf,Usd,UsdGeom=_pxr()
    if not isinstance(asset_id,str) or not _ASSET_ID.fullmatch(asset_id):
        raise ValueError("Invalid USD equipment identifier")
    if len(position_m)!=3:
        raise ValueError("Expected three metric XYZ coordinates")
    xyz=tuple(float(v) for v in position_m)
    if any(not math.isfinite(x) or abs(x)>100 for x in xyz):
        raise ValueError("XYZ position must be finite within +/-100m")
    if not math.isfinite(rotation_z_deg) or abs(rotation_z_deg)>360:
        raise ValueError("Z rotation must be finite within +/-360 degrees")
    folder,stage,eq,layout=_open(workspace)
    path=f"/World/Equipment/{asset_id}"
    if not stage.GetPrimAtPath(path):
        raise ValueError("Asset must be imported before placement")
    stage.SetEditTarget(Usd.EditTarget(layout))
    x=UsdGeom.XformCommonAPI(stage.GetPrimAtPath(path))
    if not x.SetTranslate(Gf.Vec3d(*xyz)):
        raise RuntimeError("USD asset translation edit rejected")
    if not x.SetRotate(Gf.Vec3f(0.,0.,rotation_z_deg)):
        raise RuntimeError("USD asset rotation edit rejected")
    if not layout.Save():
        raise RuntimeError("Layout layer Save() failed")
    return {"asset_id":asset_id,"position_m":list(xyz),
            "rotation_z_degrees":rotation_z_deg,
            "edit_layer":str(folder/"layers/layout.usda")}


def inspect_workspace(workspace):
    Gf,Sdf,Usd,UsdGeom=_pxr()
    folder,stage,eq,layout=_open(workspace)
    assets=[]
    parent=stage.GetPrimAtPath("/World/Equipment")
    for prim in parent.GetChildren():
        variants=prim.GetVariantSets().GetVariantSet("representation")
        x=UsdGeom.XformCommonAPI(prim)
        trans,rotate,scale,pivot,order=x.GetXformVectors(Usd.TimeCode.Default())
        assets.append({
            "prim_path":str(prim.GetPath()),
            "asset_id":prim.GetName(),
            "representation":variants.GetVariantSelection() if variants else None,
            "translation_m":list(trans),
            "rotation_xyz_deg":list(rotate),
            "asset_class":prim.GetCustomDataByKey("dsiq_asset_class"),
            "usd_geometry_authored":any(p.IsA(UsdGeom.Mesh)
                    for p in Usd.PrimRange(prim)),
        })
    return {"directory":str(folder),"project":str(folder/"project.usda"),
            "layer_stack":["layers/layout.usda","layers/equipment.usda"],
            "asset_count":len(assets),"assets":assets,
            "stage_units":"meters","up_axis":"Z",
            "renderer":"not_instantiated",
            "physics_runtime":"none"}
