"""Verified URDF visual tessellation for DSIQ's OpenGL and OpenUSD pipelines.

The module never substitutes manufacturer CAD with fake boxes. It preserves
source mesh vertices/faces, applies declared URDF mesh scale and visual frame,
and enforces bounded triangles and source SHA-256 for external mesh files.

The underlying model may still be dimensionally wrong: source integrity and
licensing declarations are not independent OEM or physical certification.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .urdf_bridge import _safe_path, validate_robot_manifest

MAX_TOTAL_TRIANGLES = 500_000
MAX_FILE_TRIANGLES = 400_000
MAX_VISUALS = 512


@dataclass(frozen=True)
class CADPart:
    link_name: str
    visual_name: str
    vertices: tuple[tuple[float, float, float], ...]
    faces: tuple[tuple[int, int, int], ...]
    rgba: tuple[float, float, float, float]
    source: str
    source_sha256: str | None

    @property
    def triangle_count(self):
        return len(self.faces)


def _v(raw: str, count: int, default: str) -> tuple[float, ...]:
    vals = tuple(float(x) for x in (raw if raw is not None else default).split())
    if len(vals) != count or not all(math.isfinite(x) for x in vals):
        raise ValueError("Invalid finite CAD vector")
    return vals


def _pose(vertex, xyz, rpy):
    x, y, z = vertex
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    ry, rz = cr*y - sr*z, sr*y + cr*z
    rx, rz = cp*x + sp*rz, -sp*x + cp*rz
    rx, ry = cy*rx - sy*ry, sy*rx + cy*ry
    return (rx+xyz[0], ry+xyz[1], rz+xyz[2])


def _box(size):
    x, y, z = [v/2 for v in size]
    vs = [(-x,-y,-z),(x,-y,-z),(x,y,-z),(-x,y,-z),
          (-x,-y,z),(x,-y,z),(x,y,z),(-x,y,z)]
    fs = [(0,2,1),(0,3,2),(4,5,6),(4,6,7),
          (0,1,5),(0,5,4),(1,2,6),(1,6,5),
          (2,3,7),(2,7,6),(3,0,4),(3,4,7)]
    return vs,fs


def _cylinder(radius, height, segments=24):
    verts=[]
    for z in (-height/2,height/2):
        verts.extend((radius*math.cos(2*math.pi*i/segments),
                      radius*math.sin(2*math.pi*i/segments),z)
                     for i in range(segments))
    verts.extend(((0.,0.,-height/2),(0.,0.,height/2)))
    faces=[]
    for i in range(segments):
        j=(i+1)%segments
        faces.extend(((i,j,segments+j),(i,segments+j,segments+i),
                      (2*segments,j,i),(2*segments+1,segments+i,segments+j)))
    return verts,faces


def _sphere(radius, latitude=12, longitude=24):
    verts=[]
    for j in range(latitude+1):
        phi=math.pi*j/latitude
        for i in range(longitude+1):
            angle=2*math.pi*i/longitude
            verts.append((radius*math.sin(phi)*math.cos(angle),
                          radius*math.sin(phi)*math.sin(angle),
                          radius*math.cos(phi)))
    faces=[]
    for j in range(latitude):
        for i in range(longitude):
            a=j*(longitude+1)+i
            b=a+longitude+1
            if j>0:
                faces.append((a,b,a+1))
            if j<latitude-1:
                faces.append((a+1,b,b+1))
    return verts,faces


def extract_verified_urdf_geometry(manifest_file: str | Path, *,
                                   max_triangles: int=MAX_TOTAL_TRIANGLES) -> dict:
    if type(max_triangles) is not int or not 12<=max_triangles<=MAX_TOTAL_TRIANGLES:
        raise ValueError("CAD triangle budget must be 12..500000")
    manifest, inspected=validate_robot_manifest(manifest_file)
    xml=ET.parse(inspected.path).getroot()
    root=Path(inspected.path).resolve().parent
    mesh_hashes=manifest.get("geometry",{}).get("mesh_sha256",{})
    if not isinstance(mesh_hashes,dict):
        raise ValueError("mesh_sha256 provenance map must be an object")
    parts=[]
    total=0
    for link in xml.findall("link"):
        lname=link.attrib["name"]
        for index, visual in enumerate(link.findall("visual")):
            if len(parts)>=MAX_VISUALS:
                raise ValueError("Too many CAD visual components")
            origin=visual.find("origin")
            xyz=_v(origin.attrib.get("xyz") if origin is not None else None,3,"0 0 0")
            rpy=_v(origin.attrib.get("rpy") if origin is not None else None,3,"0 0 0")
            geom=visual.find("geometry")
            if geom is None or len(geom)!=1:
                raise ValueError("Invalid URDF visual shape")
            shape=geom[0]
            source=shape.tag
            sha=None
            if source=="box":
                vs,fs=_box(_v(shape.attrib["size"],3,""))
            elif source=="cylinder":
                vs,fs=_cylinder(float(shape.attrib["radius"]),float(shape.attrib["length"]))
            elif source=="sphere":
                vs,fs=_sphere(float(shape.attrib["radius"]))
            elif source=="mesh":
                filename=shape.attrib["filename"].replace("\\","/")
                filepath=_safe_path(root,filename)
                if filename not in mesh_hashes:
                    raise ValueError(
                        "Mesh source requires explicit geometry.mesh_sha256 entry: "+filename
                    )
                digest=hashlib.sha256(filepath.read_bytes()).hexdigest()
                if mesh_hashes[filename]!=digest:
                    raise ValueError("Source mesh SHA256 mismatch: "+filename)
                sha=digest
                try:
                    import trimesh
                except ImportError as exc:
                    raise RuntimeError(
                        'Install licensed mesh CAD support: pip install -e ".[gpu]"'
                    ) from exc
                raw=trimesh.load(filepath,process=False,force="mesh")
                if not isinstance(raw,trimesh.Trimesh) or raw.is_empty:
                    raise ValueError("Unable to load a triangle mesh without geometry substitution")
                if len(raw.faces)>MAX_FILE_TRIANGLES:
                    raise ValueError("Single source CAD mesh triangle budget exceeded")
                scale=_v(shape.attrib.get("scale"),3,"1 1 1")
                vs=[tuple(float(v*s) for v,s in zip(vertex,scale))
                    for vertex in raw.vertices]
                fs=[tuple(int(k) for k in tri) for tri in raw.faces]
                source=filepath.name
            else:
                raise ValueError("Unsupported rights-checked URDF visual type")
            if len(vs)>1_200_000 or not fs or total+len(fs)>max_triangles:
                raise ValueError("CAD scene exceeds bounded GPU triangle budget")
            if any(len(face)!=3 or any(k<0 or k>=len(vs) for k in face)
                   for face in fs):
                raise ValueError("Invalid source CAD mesh triangle indices")
            if any(not math.isfinite(v) or abs(v)>200.
                   for point in vs for v in point):
                raise ValueError("Source mesh nonfinite or outside 200m coordinate bound")
            material=visual.find("material")
            color=material.find("color") if material is not None else None
            rgba=_v(color.attrib.get("rgba") if color is not None else None,4,"0.68 0.73 0.82 1")
            if any(not 0<=c<=1 for c in rgba):
                raise ValueError("Invalid URDF color")
            transformed=tuple(_pose(point,xyz,rpy) for point in vs)
            parts.append(CADPart(
                link_name=lname,
                visual_name=visual.attrib.get("name",f"Visual_{index:03d}"),
                vertices=transformed,
                faces=tuple(fs),
                rgba=rgba,
                source=source,
                source_sha256=sha,
            ))
            total+=len(fs)
    if not parts:
        raise ValueError("Source URDF has no renderable visual geometry")
    return {
        "schema":"dsiq-provenance-locked-gpu-geometry-v1",
        "asset_id":manifest["asset_id"],
        "asset_class":manifest["asset_class"],
        "source_urdf_sha256":manifest["geometry"]["sha256"],
        "source_mesh_hashes_validated":True,
        "visual_parts":parts,
        "part_count":len(parts),
        "triangle_count":total,
        "independent_oem_calibration":False,
    }


def lock_mesh_hashes(manifest_file: str | Path) -> dict:
    """Return a proposed mesh lockfile mapping, NEVER modify user manifests.

    The maintainer must update and review the manifest via a separate
    explicit revision; hashes authenticate bytes, not licensing claims.
    """
    _,inspected=validate_robot_manifest(manifest_file)
    root=Path(inspected.path).resolve().parent
    xml=ET.parse(inspected.path).getroot()
    result={}
    for link in xml.findall("link"):
        for vis in link.findall("visual"):
            geom=vis.find("geometry")
            if geom is None or len(geom)!=1 or geom[0].tag!="mesh":
                continue
            filename=geom[0].attrib["filename"].replace("\\","/")
            source=_safe_path(root,filename)
            result[filename]=hashlib.sha256(source.read_bytes()).hexdigest()
    return {"mesh_sha256":result,"count":len(result),
            "note":"Copy under geometry.mesh_sha256 after verifying rights and revisions"}
