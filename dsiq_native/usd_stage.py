"""OpenUSD project creation, inspection, and verified reference attachment.

Native USD is mandatory here. No fallback silently converts OEM CAD to proxy boxes.
"""
from __future__ import annotations

from pathlib import Path
import math
import os

from .asset_quality import load_manifest, validate_asset

MAX_TOP_LEVEL_LAYER_MIB = 256


def _pxr():
    try:
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
    except ImportError as exc:
        raise RuntimeError(
            "Pixar OpenUSD Python modules (pxr) are required. "
            'Use "python -m pip install usd-core" or run inside an appropriate Kit Python environment.'
        ) from exc
    return Gf, Usd, UsdGeom, UsdPhysics


def _existing_layer(path: str | Path) -> Path:
    p = Path(path).resolve()
    if not p.is_file():
        raise FileNotFoundError(p)
    if p.stat().st_size > MAX_TOP_LEVEL_LAYER_MIB * 1024 * 1024:
        raise ValueError("Top-level USD layer exceeds conservative 256 MiB inspection admission limit")
    return p


def new_stage(path: str | Path) -> dict:
    """A clean, metrically defined stage; deliberately *no* fake equipment."""
    Gf, Usd, UsdGeom, UsdPhysics = _pxr()
    p = Path(path).resolve()
    if p.suffix.lower() not in (".usd", ".usda", ".usdc"):
        raise ValueError("New scene must be .usd, .usda or .usdc")
    if p.exists():
        raise FileExistsError(f"Refusing to overwrite {p}")
    p.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(p))
    if not stage:
        raise RuntimeError(f"Unable to create stage {p}")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetTimeCodesPerSecond(60.0)
    stage.SetFramesPerSecond(60.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.Scope.Define(stage, "/World/Equipment")
    UsdGeom.Scope.Define(stage, "/World/Robots")
    UsdGeom.Scope.Define(stage, "/World/Environment")
    scene = UsdPhysics.Scene.Define(stage, "/World/Physics")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    stage.GetRootLayer().Save()
    return {"path": str(p), "meters_per_unit": 1.0, "up_axis": "Z",
            "gravity_m_s2": [0.0, 0.0, -9.81],
            "note": "Physics SCHEMA authored; no real machine simulation has been qualified."}


def _load(path: Path):
    _, Usd, _, _ = _pxr()
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise ValueError(f"Not a readable OpenUSD stage: {path}")
    return stage


def inspect_stage(path: str | Path, *, max_prims: int = 20000) -> dict:
    _, _, UsdGeom, UsdPhysics = _pxr()
    p = _existing_layer(path)
    stage = _load(p)
    counts = {"prims": 0, "meshes": 0, "xforms": 0, "cameras": 0,
              "physics_scenes": 0, "rigid_body_prims": 0}
    points = 0
    triangles_upper_estimate = 0
    for prim in stage.Traverse():
        counts["prims"] += 1
        if counts["prims"] > max_prims:
            raise ValueError(f"Stage exceeds {max_prims} prim traversal admission cap")
        if prim.IsA(UsdGeom.Mesh):
            counts["meshes"] += 1
            geom = UsdGeom.Mesh(prim)
            arr = geom.GetPointsAttr().Get()
            face_counts = geom.GetFaceVertexCountsAttr().Get()
            points += len(arr) if arr is not None else 0
            if face_counts is not None:
                triangles_upper_estimate += sum(max(0, int(n) - 2) for n in face_counts)
        if prim.IsA(UsdGeom.Xform):
            counts["xforms"] += 1
        if prim.IsA(UsdGeom.Camera):
            counts["cameras"] += 1
        if prim.IsA(UsdPhysics.Scene):
            counts["physics_scenes"] += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            counts["rigid_body_prims"] += 1
    units = float(UsdGeom.GetStageMetersPerUnit(stage))
    up = str(UsdGeom.GetStageUpAxis(stage))
    warnings: list[str] = []
    if not math.isclose(units, 1.0, rel_tol=0, abs_tol=1e-9):
        warnings.append(f"Stage units are {units} meters/unit, not DSIQ canonical 1 meter/unit")
    if up != "Z":
        warnings.append(f"Stage is {up}-up, not DSIQ canonical Z-up")
    if counts["rigid_body_prims"] == 0:
        warnings.append("No rigid bodies authored. Presence of PhysicsScene alone does not simulate anything.")
    # Approximate only POSITION vertex payload + triangle indices. Real GPU costs are far greater.
    geometry_floor_mib = (points * 12 + triangles_upper_estimate * 3 * 4) / (1024 * 1024)
    return {
        "path": str(p), "counts": counts, "vertex_positions": points,
        "triangles_estimated": triangles_upper_estimate,
        "geometry_minimum_buffer_mib_estimate": round(geometry_floor_mib, 2),
        "meters_per_unit": units, "up_axis": up,
        "warnings": warnings + [
            "Triangle count is a topology estimate; GPU memory excludes normals, "
            "textures, materials, BVHs, shaders, ray-tracing buffers and the Kit runtime."
        ],
    }


def attach_asset(scene_file: str | Path, manifest_file: str | Path) -> dict:
    """Add a referenced real USD assembly, after geometry/path/source verification.

    Never add an unverified physics schema, generic proxy, or fabricated OEM identity.
    """
    _, _, UsdGeom, _ = _pxr()
    report = validate_asset(manifest_file)
    if not report.valid_for_import:
        raise ValueError("Asset validation failed: " + "; ".join(report.errors))
    manifest = Path(manifest_file).resolve()
    entry = load_manifest(manifest)
    asset_path = (manifest.parent / entry["geometry"]["usd_file"]).resolve()
    asset = _load(_existing_layer(asset_path))
    default = asset.GetDefaultPrim()
    if not default or not default.IsValid():
        raise ValueError("Source USD must define a valid defaultPrim")
    source_units = float(UsdGeom.GetStageMetersPerUnit(asset))
    source_up = str(UsdGeom.GetStageUpAxis(asset))
    expected = entry["geometry"]
    if not math.isclose(source_units, float(expected["meters_per_unit"]), rel_tol=0, abs_tol=1e-9):
        raise ValueError(f"USD metersPerUnit={source_units} mismatches manifest")
    if source_up != expected["up_axis"]:
        raise ValueError(f"USD upAxis={source_up} mismatches manifest")
    for part in expected["subassemblies"]:
        if not asset.GetPrimAtPath(part["prim_path"]).IsValid():
            raise ValueError(f"Expected physical subassembly not present in USD: {part['prim_path']}")

    # A valid USD root or a named subassembly is not itself a 3D model.
    # Reject empty transform hierarchies mislabeled as detailed equipment.
    real_meshes = 0
    real_triangles = 0
    for source_prim in asset.Traverse():
        if source_prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(source_prim)
            verts = mesh.GetPointsAttr().Get()
            counts = mesh.GetFaceVertexCountsAttr().Get()
            indices = mesh.GetFaceVertexIndicesAttr().Get()
            if verts is not None and counts is not None and indices is not None:
                if len(verts) >= 3 and len(indices) >= 3:
                    tris = sum(max(0, int(n) - 2) for n in counts)
                    if tris > 0:
                        real_meshes += 1
                        real_triangles += tris
    if real_meshes == 0 or real_triangles == 0:
        raise ValueError(
            "USD source has no nonempty mesh topology; refusing a placeholder "
            "hierarchy advertised as detailed equipment"
        )

    scene_path = _existing_layer(scene_file)
    stage = _load(scene_path)
    if not math.isclose(float(UsdGeom.GetStageMetersPerUnit(stage)), source_units, rel_tol=0, abs_tol=1e-9):
        raise ValueError("Scene and asset units differ: explicit, validated conversion required")
    if str(UsdGeom.GetStageUpAxis(stage)) != source_up:
        raise ValueError("Scene and asset up axes differ: explicit coordinate conversion required")
    parent = stage.GetPrimAtPath("/World/Equipment")
    if not parent or not parent.IsValid():
        raise ValueError("DSIQ scene must have /World/Equipment")
    target = f"/World/Equipment/{entry['asset_id']}"
    if stage.GetPrimAtPath(target).IsValid():
        raise FileExistsError("A prim already occupies asset path " + target)
    prim = UsdGeom.Xform.Define(stage, target).GetPrim()
    rel = os.path.relpath(asset_path, scene_path.parent).replace("\\", "/")
    prim.GetReferences().AddReference(rel, default.GetPath())
    # Preserve attribution as authored metadata; no claims of qualified physics.
    prim.SetCustomDataByKey("dsiq_asset_id", entry["asset_id"])
    prim.SetCustomDataByKey("dsiq_manufacturer", entry["manufacturer"])
    prim.SetCustomDataByKey("dsiq_model", entry["model"])
    prim.SetCustomDataByKey("dsiq_physics_qualified", bool(report.simulation_qualified))
    stage.GetRootLayer().Save()
    return {"scene": str(scene_path), "prim_path": target, "usd_reference": rel,
            "physics_qualification_metadata_only": report.simulation_qualified,
            "mesh_count_at_import": real_meshes, "triangle_count_at_import": real_triangles,
            "warnings": list(report.warnings)}
