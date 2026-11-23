"""Read-only, bounded OpenUSD mesh-reuse profiler for engineering workcells.

Inspired by the supplied NVIDIA Data Adapter deduplication/merge/pivot
studies and official OpenUSD performance guidance. Unlike an optimizer,
this NEVER changes USD composition, collision geometry, material bindings,
primitive identity, payloads, variants or transform stacks.

Identical local vertex positions and topology imply GEOMETRY-ONLY reuse
candidates, not safe instanceable assemblies. Animated, oversize or
malformed meshes are skipped for exact candidate matching. GPU VRAM
savings and FPS improvements cannot be inferred from these results.
"""
from __future__ import annotations

from collections import defaultdict
import hashlib
import math
from pathlib import Path
import struct

from .usd_stage import _existing_layer, _load

MAX_PRIMS = 30000
MAX_VERTICES_PER_MESH = 60000
MAX_TOPOLOGY_INDICES = 200000
MAX_CANDIDATE_BYTES = 48 * 1024 * 1024
MAX_EXAMPLES = 8


def _geometry_bytes(mesh):
    """Exact float32/float64 numeric geometry fingerprint, no materials.

    Preserve points in floating-point 64-bit representation to avoid
    equality affected by string formatting. Both topology arrays
    must also match exactly; vertex order and face winding matter.
    """
    points = mesh.GetPointsAttr().Get()
    counts = mesh.GetFaceVertexCountsAttr().Get()
    indices = mesh.GetFaceVertexIndicesAttr().Get()
    if points is None or counts is None or indices is None:
        return None, "MISSING_MESH_TOPOLOGY"
    if (len(points) > MAX_VERTICES_PER_MESH or
            len(indices) > MAX_TOPOLOGY_INDICES):
        return None, "MESH_EXCEEDS_FINGERPRINT_BUDGET"
    if not points or not counts or not indices:
        return None, "EMPTY_MESH"
    if (any(int(k) < 3 for k in counts) or
            sum(int(k) for k in counts) != len(indices) or
            any(int(k) < 0 or int(k) >= len(points) for k in indices)):
        return None, "MALFORMED_MESH_TOPOLOGY"
    out = bytearray()
    out.extend(struct.pack("<III", len(points),len(counts),len(indices)))
    for pt in points:
        xyz = tuple(float(v) for v in pt)
        if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
            return None, "NONFINITE_VERTEX"
        out.extend(struct.pack("<ddd",*xyz))
    for k in counts:
        out.extend(struct.pack("<I",int(k)))
    for k in indices:
        out.extend(struct.pack("<I",int(k)))
    return bytes(out), None


def profile_usd_scene(path: str | Path, *, max_prims: int=MAX_PRIMS) -> dict:
    """Audit exact local geometry reuse WITHOUT editing authored USD files."""
    if type(max_prims) is not int or not 1<=max_prims<=MAX_PRIMS:
        raise ValueError("USD profile max_prims must be 1..30000")
    source = _existing_layer(path)
    if source.suffix.lower() not in (".usd",".usda",".usdc",".usdz"):
        raise ValueError("Expected OpenUSD stage file (.usd/.usda/.usdc/.usdz)")
    before = hashlib.sha256(source.read_bytes()).hexdigest()
    from pxr import UsdGeom
    stage = _load(source)
    counts = {
        "prims":0, "meshes":0, "triangles_topology_estimate":0,
        "vertex_positions":0, "instance_roots":0, "time_varying_meshes":0,
        "skipped_geometry_fingerprints":0,
    }
    excluded = defaultdict(int)
    # Store full canonical bytes within a strict aggregate memory budget
    # to confirm candidate matches, not merely trust a SHA-256 collision.
    fingerprints = defaultdict(list)
    buffered = 0
    for prim in stage.Traverse():
        counts["prims"] += 1
        if counts["prims"] > max_prims:
            raise ValueError("OpenUSD scene exceeds bounded prim traversal budget")
        if prim.IsInstance():
            counts["instance_roots"] += 1
        if not prim.IsA(UsdGeom.Mesh):
            continue
        counts["meshes"] += 1
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        fc = mesh.GetFaceVertexCountsAttr().Get()
        counts["vertex_positions"] += len(points) if points is not None else 0
        if fc is not None:
            counts["triangles_topology_estimate"] += sum(
                max(0,int(v)-2) for v in fc
            )
        if any(attr.ValueMightBeTimeVarying() for attr in (
            mesh.GetPointsAttr(),mesh.GetFaceVertexCountsAttr(),
            mesh.GetFaceVertexIndicesAttr(),
        )):
            counts["time_varying_meshes"] += 1
            counts["skipped_geometry_fingerprints"] += 1
            excluded["TIME_VARYING_MESH"] += 1
            continue
        data, reason = _geometry_bytes(mesh)
        if reason is not None:
            counts["skipped_geometry_fingerprints"] += 1
            excluded[reason] += 1
            continue
        if buffered+len(data)>MAX_CANDIDATE_BYTES:
            counts["skipped_geometry_fingerprints"] += 1
            excluded["AGGREGATE_FINGERPRINT_BUDGET"] += 1
            continue
        buffered += len(data)
        digest = hashlib.sha256(data).hexdigest()
        matched = False
        for bucket in fingerprints[digest]:
            if bucket["bytes"] == data:
                bucket["paths"].append(str(prim.GetPath()))
                matched = True
                break
        if not matched:
            fingerprints[digest].append({
                "bytes":data, "paths":[str(prim.GetPath())],
            })
    groups = []
    repeated_payload_bytes = 0
    for digest,buckets in fingerprints.items():
        for entry in buckets:
            if len(entry["paths"])>1:
                repeated_payload_bytes += (
                    len(entry["bytes"])*(len(entry["paths"])-1)
                )
                groups.append({
                    "geometry_sha256":digest,
                    "identical_mesh_count":len(entry["paths"]),
                    "paths":entry["paths"][:MAX_EXAMPLES],
                    "more_paths_omitted":max(0,len(entry["paths"])-MAX_EXAMPLES),
                })
    groups.sort(key=lambda x:(-x["identical_mesh_count"],x["geometry_sha256"]))
    if hashlib.sha256(source.read_bytes()).hexdigest()!=before:
        raise RuntimeError("Source USD changed during read-only profile; rerun audit")
    return {
        "schema":"dsiq-read-only-usd-geometry-profile-v1",
        "stage":str(source),
        "source_stage_sha256":before,
        "composed_stage_counts":counts,
        "exact_local_geometry_reuse_groups":len(groups),
        "exact_local_geometry_reuse_meshes":sum(x["identical_mesh_count"] for x in groups),
        "duplicate_groups":groups[:50],
        "duplicate_groups_omitted":max(0,len(groups)-50),
        "redundant_raw_geometry_payload_bytes_upper_bound":repeated_payload_bytes,
        "fingerprint_buffered_bytes":buffered,
        "fingerprint_exclusion_reasons":dict(excluded),
        "stage_authoring_performed":False,
        "automatic_mesh_merge_performed":False,
        "automatic_instance_conversion_performed":False,
        "renderer_vram_or_fps_savings_measured":False,
        "industrial_physics_or_collision_fidelity_verified":False,
        "important_limitations":[
            "Matching local vertex positions, connectivity and winding alone DO NOT establish valid USD instancing or merge compatibility.",
            "Materials, primvars, inherited attributes, USD composition arcs, rigid-body semantics, skinning, collision and metadata must be audited before optimization.",
            "Animated meshes and meshes exceeding bounded fingerprint budgets are excluded from duplicate candidate analysis.",
            "Exact geometry reuse is not equivalent to geometry deduplication across deep transforms or tolerance-based shape matching.",
            "GPU memory and render performance must be measured on the intended hardware and representative scenes.",
        ],
    }
