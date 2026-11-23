"""Read-only bounded OpenUSD engineering asset validator.

This is a first set of measurable USD structure, mesh, material, physics and
composition checks. It is intentionally NOT an ISO 10303, AOUSD conformance
certificate, SimReady certification, OEM model validator, or malware scanner.
All geometry inspection is performed on composed Pixar USD data.
"""
from __future__ import annotations

import hashlib
import math
from pathlib import Path

from .usd_stage import _existing_layer

MAX_PRIMS = 20000
MAX_POINTS = 2_000_000
MAX_FACE_INDICES = 6_000_000
MAX_ISSUES = 128
_EXTENSIONS = {".usd", ".usda", ".usdc", ".usdz"}


def validate_usd_stage(path: str | Path, *, max_prims: int=MAX_PRIMS,
                       require_metric_z_up: bool=True,
                       require_default_prim: bool=True) -> dict:
    """Audit a real USD stage without saving/editing the layer.

    Errors fail default acceptance, warnings are advisory. Resource limits
    raise ValueError (closed) rather than silently truncating the stage.
    Unloaded payloads are NOT validated and are identified in the report.
    """
    if type(max_prims) is not int or not 1 <= max_prims <= MAX_PRIMS:
        raise ValueError("max_prims must be an integer between 1 and 20000")
    p = _existing_layer(path)
    if p.suffix.lower() not in _EXTENSIONS:
        raise ValueError("Expected .usd/.usda/.usdc/.usdz OpenUSD stage")
    rawhash = hashlib.sha256(p.read_bytes()).hexdigest()
    try:
        from pxr import Usd, UsdGeom, UsdShade, UsdPhysics
    except ImportError as exc:
        raise RuntimeError('OpenUSD required: pip install -e ".[usd]"') from exc

    # Avoid automatically loading heavyweight external payloads; the
    # report explicitly tracks unloaded references/payloads.
    stage = Usd.Stage.Open(str(p), load=Usd.Stage.LoadNone)
    if not stage:
        raise ValueError("OpenUSD stage could not be opened")
    errors, warnings = [], []
    issues_overflow = [0]

    def issue(severity: str, rule: str, prim: str, detail: str):
        dest = errors if severity=="error" else warnings
        if len(errors)+len(warnings) >= MAX_ISSUES:
            issues_overflow[0] += 1
        else:
            dest.append({"rule_id":rule,"prim":prim,"detail":detail})

    default = stage.GetDefaultPrim()
    if require_default_prim and not (default and default.IsValid()):
        issue("error","USD_DEFAULT_PRIM",str(stage.GetPseudoRoot().GetPath()),
              "Stage has no valid defaultPrim")
    meters = float(UsdGeom.GetStageMetersPerUnit(stage))
    up = str(UsdGeom.GetStageUpAxis(stage))
    if not math.isfinite(meters) or meters<=0:
        issue("error","USD_UNITS_INVALID","/",
              "metersPerUnit is nonpositive or not finite")
    elif require_metric_z_up and not math.isclose(meters,1.0,rel_tol=0,abs_tol=1e-9):
        issue("error","DSIQ_METRIC_UNITS","/",
              f"DSIQ canonical authored units require 1 meter/unit, got {meters}")
    if require_metric_z_up and up!="Z":
        issue("error","DSIQ_Z_UP","/",f"DSIQ workcells require Z-up, got {up}")

    stats = {
        "prims":0,"xforms":0,"meshes":0,"mesh_points":0,
        "mesh_faces":0,"face_indices":0,"triangles_estimated":0,
        "materials":0,"mesh_without_material":0,"cameras":0,
        "physics_scenes":0,"rigid_bodies":0,"colliders":0,
        "instanced_prims":0,"unloaded_payload_prims":0,
        "time_sampled_meshes":0,
    }
    for prim in stage.TraverseAll():
        if not prim.IsActive() or not prim.IsDefined():
            continue
        stats["prims"] += 1
        if stats["prims"]>max_prims:
            raise ValueError("USD prim inspection limit exceeded; no incomplete PASS")
        key=str(prim.GetPath())
        if prim.IsInstance():
            stats["instanced_prims"]+=1
        if prim.HasPayload() and not prim.IsLoaded():
            stats["unloaded_payload_prims"]+=1
            issue("warning","USD_PAYLOAD_NOT_VALIDATED",key,
                  "Payload intentionally left unloaded; external contents not inspected")
        if prim.IsA(UsdGeom.Xform):
            stats["xforms"]+=1
        if prim.IsA(UsdGeom.Camera):
            stats["cameras"]+=1
        if prim.IsA(UsdShade.Material):
            stats["materials"]+=1
        if prim.IsA(UsdPhysics.Scene):
            stats["physics_scenes"]+=1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            stats["rigid_bodies"]+=1
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            stats["colliders"]+=1
        if prim.HasAPI(UsdPhysics.MassAPI):
            mass=UsdPhysics.MassAPI(prim).GetMassAttr()
            if mass and mass.HasAuthoredValueOpinion():
                v=mass.Get()
                if v is not None and (not math.isfinite(float(v)) or float(v)<=0):
                    issue("error","USD_INVALID_RIGID_MASS",key,
                          "Authored mass must be finite and positive")
        if not prim.IsA(UsdGeom.Mesh):
            continue
        stats["meshes"] += 1
        mesh=UsdGeom.Mesh(prim)
        points=mesh.GetPointsAttr().Get()
        counts=mesh.GetFaceVertexCountsAttr().Get()
        indices=mesh.GetFaceVertexIndicesAttr().Get()
        if points is None or counts is None or indices is None:
            issue("error","MESH_MISSING_TOPOLOGY",key,
                  "Mesh is missing points, face counts or indices")
            continue
        npts, nfaces, ninds=len(points), len(counts), len(indices)
        stats["mesh_points"]+=npts
        stats["mesh_faces"]+=nfaces
        stats["face_indices"]+=ninds
        if stats["mesh_points"]>MAX_POINTS or stats["face_indices"]>MAX_FACE_INDICES:
            raise ValueError("USD cumulative mesh vertex/index admission limit exceeded")
        if npts==0 or nfaces==0 or ninds==0:
            issue("error","MESH_EMPTY",key,"Mesh contains no actual face geometry")
            continue
        if any(len(x)!=3 or any(not math.isfinite(float(v)) for v in x)
               for x in points):
            issue("error","MESH_NONFINITE_POINTS",key,
                  "Mesh has nonfinite or invalid vertex coordinates")
        if any(type(int(n)) is not int or int(n)<3 for n in counts):
            issue("error","MESH_FACE_DEGENERATE",key,
                  "Mesh face counts must be at least three")
        elif sum(int(n) for n in counts)!=ninds:
            issue("error","MESH_TOPOLOGY_COUNT",key,
                  "Face counts do not equal face vertex index array length")
        else:
            stats["triangles_estimated"]+=sum(int(n)-2 for n in counts)
        if any(int(n)<0 or int(n)>=npts for n in indices):
            issue("error","MESH_INDEX_BOUNDS",key,
                  "Mesh face references a nonexistent vertex index")
        if mesh.GetPointsAttr().GetNumTimeSamples()>0:
            stats["time_sampled_meshes"]+=1
            issue("warning","MESH_ANIMATED_SAMPLE_ONLY",key,
                  "Only the current default/timecode geometry is checked")
        try:
            bound=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            if not bound or not bound.GetPrim().IsValid():
                stats["mesh_without_material"]+=1
                issue("warning","MESH_MATERIAL_UNBOUND",key,
                      "No resolved material binding; display appearance may vary")
        except (TypeError, RuntimeError):
            issue("warning","MESH_MATERIAL_UNRESOLVED",key,
                  "Could not resolve composed material binding")

    if issues_overflow[0]:
        # Fail closed if the bounded issue report cannot express every fault.
        raise ValueError("USD issues exceed maximum report count; no incomplete PASS")
    if stats["prims"]==0:
        issue("warning","USD_EMPTY_STAGE","/","Stage has no authored active prims")
    afterhash=hashlib.sha256(p.read_bytes()).hexdigest()
    if afterhash!=rawhash:
        raise RuntimeError("OpenUSD source changed during supposedly read-only audit")
    return {
        "schema":"dsiq-openusd-engineering-validator-v1",
        "stage_sha256_before":rawhash,
        "stage_sha256_after":afterhash,
        "source_file":str(p),
        "passed":not errors,
        "counts":stats,
        "units_meters_per_unit":meters,
        "up_axis":up,
        "errors":errors,"warnings":warnings,
        "error_count":len(errors),"warning_count":len(warnings),
        "policy":{
            "require_metric_z_up":require_metric_z_up,
            "require_default_prim":require_default_prim,
            "max_prims":max_prims,
            "max_issues":MAX_ISSUES,
            "payloads_loaded":False,
        },
        "limitations":[
            "Unloaded payload contents and external dependency rights are not audited",
            "Geometry default-time validation does not prove all animation samples",
            "This is not an AOUSD/SimReady or industrial OEM certification",
        ],
    }
