"""Defensive URDF robot import validation, provenance and joint-graph inspection.

Genuine equipment comes from actual user-supplied licensed URDF/CAD. No
fabricated OEM geometry is generated. The validation gate enforces a bounded
robot tree, SI values, local-only resource paths and byte-level provenance.
Validation is not proof of dimensional accuracy or industrial commissioning.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

MAX_URDF_BYTES = 2 * 1024 * 1024
MAX_MESH_BYTES = 80 * 1024 * 1024
MAX_LINKS = 64
MAX_JOINTS = 63
ALLOWED_MESH_EXT = {".stl", ".obj", ".dae"}
_ASSET_ID = re.compile(r"^[a-z][a-z0-9_]{2,63}$")
_NAME = re.compile(r"^[A-Za-z_][A-Za-z_0-9.-]{0,63}$")
_SHA256 = re.compile(r"^[a-fA-F0-9]{64}$")
_ALLOWED_JOINTS = {"fixed", "revolute", "continuous", "prismatic"}


def _safe_path(root: Path, path: str) -> Path:
    if not isinstance(path, str) or not path:
        raise ValueError("Expected nonempty local relative path")
    standard = path.replace("\\", "/")
    if (standard.startswith("/") or standard.startswith("//")
            or re.match(r"^[A-Za-z]:", standard)
            or "://" in standard):
        raise ValueError("External URI and absolute paths are not supported in a trusted URDF")
    rel = Path(standard)
    if ".." in rel.parts:
        raise ValueError("Parent-directory traversal is disallowed")
    target = (root.resolve() / rel).resolve()
    if not target.is_relative_to(root.resolve()):
        raise ValueError("Resource escapes its imported asset root")
    return target


def _parse_numbers(s: str, count: int, *, positive=False) -> tuple[float, ...]:
    try:
        values = tuple(float(x) for x in s.split())
    except (TypeError, ValueError) as exc:
        raise ValueError("Expected finite numeric vector") from exc
    if len(values) != count or any(not math.isfinite(x) for x in values):
        raise ValueError(f"Expected {count} finite numbers, got {s!r}")
    if positive and any(x <= 0 for x in values):
        raise ValueError("Dimension/mass/inertia values must be positive")
    return values


def _verify_inertia_tensor(elements: dict[str, float], link_name: str) -> None:
    """Necessary rigid-body inertia feasibility checks, not calibrated masses.

    An inertia tensor is symmetric positive-definite, and central moments
    of inertia obey the principal moment triangle inequalities. The latter
    are checked for diagonal elements in the provided inertial frame;
    rotation-invariant full feasibility requires deeper eigenanalysis.
    """
    xx,yy,zz=(elements[k] for k in ("ixx","iyy","izz"))
    xy,xz,yz=(elements[k] for k in ("ixy","ixz","iyz"))
    determinant=xx*(yy*zz-yz*yz)-xy*(xy*zz-xz*yz)+xz*(xy*yz-xz*yy)
    if not (xx>0 and yy>0 and zz>0 and xx*yy-xy*xy>1e-14
            and determinant>1e-16):
        raise ValueError(
            f"Link {link_name} has a non-positive-definite inertia tensor"
        )
    max_diag=max(xx,yy,zz)
    tol=1e-9*max(1.,max_diag)
    if xx+yy+tol<zz or xx+zz+tol<yy or yy+zz+tol<xx:
        raise ValueError(
            f"Link {link_name} violates rigid-body inertia moment inequalities"
        )


@dataclass(frozen=True)
class URDFJoint:
    name: str
    kind: str
    parent: str
    child: str
    lower: float | None
    upper: float | None
    axis: tuple[float,float,float]

    def as_dict(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class URDFInspection:
    path: str
    robot_name: str
    links: tuple[str, ...]
    joints: tuple[URDFJoint, ...]
    root_link: str
    visual_mesh_paths: tuple[str, ...]
    collision_mesh_paths: tuple[str, ...]
    links_without_inertial: tuple[str, ...]
    warnings: tuple[str, ...]

    def as_dict(self) -> dict:
        return {
            "path": self.path, "robot_name": self.robot_name,
            "links": list(self.links), "link_count": len(self.links),
            "root_link": self.root_link,
            "joint_count": len(self.joints),
            "actuated_joint_count": sum(j.kind != "fixed" for j in self.joints),
            "joints": [j.as_dict() for j in self.joints],
            "visual_mesh_paths": list(self.visual_mesh_paths),
            "collision_mesh_paths": list(self.collision_mesh_paths),
            "links_without_inertial": list(self.links_without_inertial),
            "warnings": list(self.warnings),
            "physically_qualified": False,
        }


def inspect_urdf(path: str | Path) -> URDFInspection:
    """Inspect a local URDF. Does not execute Xacro/Gazebo plugins or parse remote assets."""
    file = Path(path).resolve()
    if file.suffix.lower() != ".urdf":
        raise ValueError("Expected a pre-expanded .urdf file (Xacro not executed)")
    if not file.is_file() or not 0 < file.stat().st_size <= MAX_URDF_BYTES:
        raise ValueError("URDF must exist and be <=2 MiB")
    raw = file.read_bytes()
    if any(token in raw.upper() for token in (b"<!DOCTYPE", b"<!ENTITY", b"<XACRO:", b"<PLUGIN")):
        raise ValueError("URDF with XML entities, Xacro or executable plugin directives is prohibited")
    try:
        root = ET.fromstring(raw)
    except ET.ParseError as exc:
        raise ValueError(f"Malformed URDF: {exc}") from exc
    if root.tag != "robot":
        raise ValueError("URDF must have a root <robot>")
    robot_name = root.attrib.get("name", "")
    if not _NAME.fullmatch(robot_name):
        raise ValueError("Robot name is missing or invalid")
    links = root.findall("link")
    joints = root.findall("joint")
    if not 1 <= len(links) <= MAX_LINKS or len(joints) > MAX_JOINTS:
        raise ValueError("URDF link/joint count exceeds constrained import budget")
    names = []
    warnings: list[str] = []
    no_inertia = []
    visual_meshes: list[str] = []
    collision_meshes: list[str] = []
    for link in links:
        name = link.attrib.get("name", "")
        if not _NAME.fullmatch(name) or name in names:
            raise ValueError(f"Duplicate or invalid URDF link: {name!r}")
        names.append(name)
        inertial = link.find("inertial")
        if inertial is None:
            no_inertia.append(name)
        else:
            mass = inertial.find("mass")
            if mass is None:
                raise ValueError(f"Inertial element missing mass: {name}")
            _parse_numbers(mass.attrib.get("value", ""), 1, positive=True)
            inertia = inertial.find("inertia")
            if inertia is None:
                raise ValueError(f"Missing inertia tensor for {name}")
            tensor={}
            for component in ("ixx", "iyy", "izz"):
                tensor[component]=_parse_numbers(
                    inertia.attrib.get(component, ""), 1, positive=True
                )[0]
            for component in ("ixy", "ixz", "iyz"):
                tensor[component]=_parse_numbers(
                    inertia.attrib.get(component, ""), 1
                )[0]
            _verify_inertia_tensor(tensor,name)
        for component in ("visual", "collision"):
            for sub in link.findall(component):
                origin = sub.find("origin")
                if origin is not None:
                    _parse_numbers(origin.attrib.get("xyz", "0 0 0"), 3)
                    _parse_numbers(origin.attrib.get("rpy", "0 0 0"), 3)
                geom = sub.find("geometry")
                if geom is None or len(geom) != 1:
                    raise ValueError(f"{component} for {name} must have one geometry")
                shape = list(geom)[0]
                if shape.tag == "mesh":
                    uri = shape.attrib.get("filename", "")
                    mesh = _safe_path(file.parent, uri)
                    if mesh.suffix.lower() not in ALLOWED_MESH_EXT:
                        raise ValueError(f"Unsupported local mesh format: {mesh.suffix}")
                    if not mesh.is_file() or not 0 < mesh.stat().st_size <= MAX_MESH_BYTES:
                        raise ValueError(f"Mesh file missing or >80 MiB: {uri}")
                    scale = shape.attrib.get("scale", "1 1 1")
                    _parse_numbers(scale, 3, positive=True)
                    (visual_meshes if component=="visual" else collision_meshes).append(str(mesh))
                elif shape.tag == "box":
                    _parse_numbers(shape.attrib.get("size", ""), 3, positive=True)
                elif shape.tag == "cylinder":
                    _parse_numbers(shape.attrib.get("radius", ""), 1, positive=True)
                    _parse_numbers(shape.attrib.get("length", ""), 1, positive=True)
                elif shape.tag == "sphere":
                    _parse_numbers(shape.attrib.get("radius", ""), 1, positive=True)
                else:
                    raise ValueError(f"Unsupported URDF geometry type: {shape.tag}")
    if len(set(names)) != len(names):
        raise ValueError("URDF link names must be unique")
    link_names = set(names)
    parsed: list[URDFJoint] = []
    child_to_parent = {}
    used_joint_names = set()
    for joint in joints:
        name = joint.attrib.get("name","")
        kind = joint.attrib.get("type","")
        if not _NAME.fullmatch(name) or name in used_joint_names:
            raise ValueError(f"Duplicate/invalid joint name {name!r}")
        used_joint_names.add(name)
        if kind not in _ALLOWED_JOINTS:
            raise ValueError(f"Unsupported URDF joint type {kind!r} for {name}")
        parent = joint.find("parent")
        child = joint.find("child")
        p_name = parent.attrib.get("link") if parent is not None else None
        c_name = child.attrib.get("link") if child is not None else None
        if p_name not in link_names or c_name not in link_names or p_name == c_name:
            raise ValueError(f"Joint {name} references invalid parent/child links")
        if c_name in child_to_parent:
            raise ValueError(f"Link {c_name} has multiple parent joints")
        child_to_parent[c_name] = p_name
        origin = joint.find("origin")
        if origin is not None:
            _parse_numbers(origin.attrib.get("xyz", "0 0 0"), 3)
            _parse_numbers(origin.attrib.get("rpy", "0 0 0"), 3)
        axis_node = joint.find("axis")
        axis = _parse_numbers(
            axis_node.attrib.get("xyz", "1 0 0") if axis_node is not None else "1 0 0",
            3,
        )
        if kind != "fixed" and math.sqrt(sum(x*x for x in axis)) < 1e-9:
            raise ValueError("Movable joint cannot have zero axis")
        lower = upper = None
        limit = joint.find("limit")
        if kind in ("revolute", "prismatic"):
            if limit is None:
                raise ValueError(f"Joint {name} requires physical lower and upper limits")
            lower = _parse_numbers(limit.attrib.get("lower", ""), 1)[0]
            upper = _parse_numbers(limit.attrib.get("upper", ""), 1)[0]
            if lower >= upper:
                raise ValueError(f"Joint {name} has invalid limits")
        if kind != "fixed":
            if limit is None:
                raise ValueError(
                    f"Joint {name} requires explicit effort and velocity limits"
                )
            for key in ("effort","velocity"):
                if key not in limit.attrib:
                    raise ValueError(
                        f"Joint {name} missing required motor {key} limit"
                    )
                _parse_numbers(limit.attrib[key],1,positive=True)
        parsed.append(URDFJoint(name,kind,p_name,c_name,lower,upper,axis))

    roots = link_names - set(child_to_parent)
    if len(roots) != 1:
        raise ValueError("URDF must have exactly one connected root link")
    root_link = next(iter(roots))
    for name in names:
        visited = set()
        current = name
        while current in child_to_parent:
            if current in visited:
                raise ValueError("URDF has a cycle in joint hierarchy")
            visited.add(current)
            current = child_to_parent[current]
        if current != root_link:
            raise ValueError("URDF contains disconnected joint hierarchy")
    if no_inertia:
        warnings.append(
            "Missing inertial properties on links: " + ", ".join(no_inertia)
            + "; cannot claim calibrated rigid-body dynamics."
        )
    if not collision_meshes:
        warnings.append(
            "No mesh-based collision geometry: primitive contact shapes may be useful "
            "for tests but do not prove geometric fidelity."
        )
    warnings.append(
        "URDF inspection verifies bounded tree, local resources and numeric fields; "
        "it is NOT manufacturer/CAD accuracy or physical calibration."
    )
    return URDFInspection(
        str(file), robot_name, tuple(names), tuple(parsed), root_link,
        tuple(visual_meshes), tuple(collision_meshes), tuple(no_inertia),
        tuple(warnings),
    )


def validate_robot_manifest(manifest_path: str | Path) -> tuple[dict, URDFInspection]:
    """Only a provenance-checked manifest allows loading external URDF dynamics."""
    path = Path(manifest_path).resolve()
    if not path.is_file() or path.stat().st_size > 128 * 1024:
        raise ValueError("Robot manifest missing or too large")
    m = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(m, dict) or m.get("schema") != "dsiq-robot-urdf-v1":
        raise ValueError("Robot asset manifest schema must be dsiq-robot-urdf-v1")
    name = m.get("asset_id", "")
    if not isinstance(name, str) or not _ASSET_ID.fullmatch(name):
        raise ValueError("asset_id must be lowercase snake_case")
    for key in ("manufacturer","model","revision","source_uri"):
        v = m.get(key)
        if not isinstance(v,str) or not v.strip():
            raise ValueError(f"{key} must contain real provenance")
    aclass = m.get("asset_class")
    if aclass not in ("test_fixture","engineering_reference","oem_verified"):
        raise ValueError("asset_class must explicitly state model fidelity")
    lic = m.get("license")
    if not isinstance(lic,dict) or lic.get("usage_rights") not in ("owned","licensed","open"):
        raise ValueError("Robot license rights must be recorded")
    rights = _safe_path(path.parent, lic.get("evidence_file",""))
    if not rights.is_file() or not rights.stat().st_size:
        raise ValueError("License evidence file missing or empty")
    geometry = m.get("geometry")
    if not isinstance(geometry,dict):
        raise ValueError("geometry is missing")
    source = _safe_path(path.parent, geometry.get("urdf_file",""))
    expected = geometry.get("sha256","")
    if not isinstance(expected,str) or not _SHA256.fullmatch(expected):
        raise ValueError("URDF SHA256 must be a 64-character checksum")
    if not source.is_file() or hashlib.sha256(source.read_bytes()).hexdigest() != expected.lower():
        raise ValueError("Source URDF missing or checksum differs")
    inspected = inspect_urdf(source)
    if aclass != "test_fixture":
        if not inspected.visual_mesh_paths:
            raise ValueError("Industrial geometry requires real mesh assets, not only primitives")
        review = m.get("visual_review")
        if not isinstance(review, dict) or review.get("approved") is not True:
            raise ValueError("Industrial asset must have explicit approved visual_review")
        evidence = _safe_path(path.parent, review.get("evidence_file",""))
        if not evidence.is_file() or not evidence.stat().st_size:
            raise ValueError("Industrial geometry requires review evidence file")
        for key in ("reviewer","reviewed_against"):
            if not isinstance(review.get(key),str) or not review[key].strip():
                raise ValueError(f"visual_review.{key} missing")
    return m, inspected
