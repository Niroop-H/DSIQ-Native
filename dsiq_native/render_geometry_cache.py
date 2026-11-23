"""Exact CAD draw-buffer deduplication keys (no geometry modification).

The key includes the fully transformed, source-validated triangle vertices,
face order, and authored RGBA. Only identical render data may share a Panda3D
Geom C++ object. The GeomNode/NodePath and Bullet articulated body remain
separate for each machine: this is NOT mesh merging, LOD, draw-call batching,
physics reuse, GPU residency proof, or OEM geometry certification.
"""
from __future__ import annotations

import hashlib
import math
import struct


def render_geometry_fingerprint(part) -> str:
    """Content identity for the data consumed by GPUStudio._panda_geometry.

    Big-endian exact IEEE-754 double packing provides a stable signature for
    the same immutable CADPart on Windows and Linux. Distinct vertex/face
    order or material colors never share a buffer, even when visually close.
    """
    vertices, faces, rgba = part.vertices, part.faces, part.rgba
    if not vertices or not faces or len(rgba)!=4:
        raise ValueError("Cache key requires nonempty triangulated CAD and RGBA")
    h=hashlib.sha256(b"dsiq-panda3d-render-geometry-key-v1\0")
    h.update(struct.pack("!II", len(vertices),len(faces)))
    for vertex in vertices:
        if len(vertex)!=3 or any(not math.isfinite(float(value)) for value in vertex):
            raise ValueError("Render geometry has nonfinite or invalid vertices")
        h.update(struct.pack("!3d",*(float(value) for value in vertex)))
    for triangle in faces:
        if (len(triangle)!=3 or any(
            type(index) is not int or index<0 or index>=len(vertices)
            for index in triangle
        )):
            raise ValueError("Render geometry has invalid triangle indices")
        h.update(struct.pack("!3I",*triangle))
    if any(not math.isfinite(float(c)) or not 0<=float(c)<=1 for c in rgba):
        raise ValueError("Render geometry has invalid material RGBA")
    h.update(struct.pack("!4d",*(float(c) for c in rgba)))
    return h.hexdigest()
