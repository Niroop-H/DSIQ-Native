"""Universal DSIQ robotics-workcell engineering gate and batch registry.

No hard-coded notion of "one demo robot": every workcell in the registry is
evaluated by the SAME evidence and real solver checks. A verified file hash
does not certify geometry, physics or safety; this module intentionally
never returns an "industrial certified" status.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any

from .urdf_bridge import validate_robot_manifest

SCHEMA = "dsiq-workcell-v1"
CATALOG_SCHEMA = "dsiq-workcell-catalog-v1"
_ID = re.compile(r"^[a-z][a-z0-9_]{2,63}$")
_HEX = re.compile(r"^[0-9a-f]{64}$")
ROLES = frozenset({"robot", "machine", "tooling", "fixture", "safety", "sensor", "material_handling"})
REAL_SENSOR_CONTRACTS = frozenset({"bullet_geometric_lidar_2d", "bullet_tinyrenderer_rgbd", "bullet_joint_state"})
REQUIRED_ROLES = frozenset({"robot", "tooling", "fixture", "safety"})
GATES = (
    "geometry_cad", "inertial_parameters", "mechanism_kinematics",
    "contact_dynamics", "actuator_controls", "sensors", "safety",
    "rendering_materials", "resource_performance", "reproducibility",
)
MAX_EQUIPMENT = 4  # Constrained Bullet runtime: 4 imported URDF assemblies
MAX_CATALOG = 64
MAX_EVIDENCE = 16 * 1024 * 1024
MAX_WORKCELL_FILE = 256 * 1024


def _digest(file: Path) -> str:
    h = hashlib.sha256()
    with file.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _relative(root: Path, raw: str) -> Path:
    if not isinstance(raw, str) or not raw:
        raise ValueError("Expected a nonempty relative resource path")
    normalized = raw.replace("\\", "/")
    if (normalized.startswith("/") or "://" in normalized or
            re.match(r"^[a-zA-Z]:", normalized) or ".." in Path(normalized).parts):
        raise ValueError("Absolute paths, URIs and parent-directory traversal are disallowed")
    path = (root / normalized).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError("Resource must stay inside workcell project root")
    return path


def _xyz(value: Any, *, label: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{label} requires exactly three coordinates")
    if any(type(v) not in (float, int) or not math.isfinite(v) or abs(v) > 20 for v in value):
        raise ValueError(f"{label} requires finite metric XYZ values within +/-20m")
    return tuple(float(v) for v in value)


def _load_json(file: Path, *, size: int = MAX_WORKCELL_FILE) -> dict:
    if not file.is_file() or not 0 < file.stat().st_size <= size:
        raise ValueError(f"Missing or oversized JSON resource: {file.name}")
    data = json.loads(file.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("JSON resource must be an object")
    return data


def create_workcell(
    output_file: str | Path, *,
    workcell_id: str, name: str, manifest: str | Path,
    environment: str = "robotics_cell",
) -> dict:
    """Create a self-describing, deliberately NOT YET QUALIFIED workcell."""
    from .environments import PRESETS
    if not isinstance(workcell_id, str) or not _ID.fullmatch(workcell_id):
        raise ValueError("workcell_id must be lower-case snake_case")
    if not isinstance(name, str) or not name.strip():
        raise ValueError("Workcell must have a descriptive engineering name")
    if environment not in PRESETS:
        raise ValueError("Unknown environment preset")
    target = Path(output_file).resolve()
    if target.exists():
        raise FileExistsError("Workcell project already exists; refusing overwrite")
    source = Path(manifest).resolve()
    if not source.is_file():
        raise FileNotFoundError("URDF source manifest not present")
    validate_robot_manifest(source)
    # Workcells are PORTABLE: their URDF manifests must remain within
    # the workcell project directory, not silently depend on arbitrary
    # files elsewhere on the user's machine.
    if not source.is_relative_to(target.parent):
        raise ValueError("Machine manifest must be inside the workcell project directory")
    target.parent.mkdir(parents=True, exist_ok=True)
    spec = {
        "schema": SCHEMA,
        "workcell_id": workcell_id,
        "name": name.strip(),
        "revision": "draft-1",
        "environment": environment,
        "units": "SI",
        "equipment": [{
            "instance_id": "primary_robot",
            "role": "robot",
            "manifest": source.relative_to(target.parent).as_posix(),
            "manifest_sha256": _digest(source),
            "base_position_m": [0., 1.25, 0.],
        }],
        "interface": {"required_sensors": ["bullet_geometric_lidar_2d"]},
        "acceptance_evidence": {},
        "intended_use": "research_reference_only",
    }
    target.write_text(json.dumps(spec, indent=2) + "\n", encoding="utf-8")
    return {
        "workcell_file": str(target), "workcell_id": workcell_id,
        "status": "INCOMPLETE", "required_roles": sorted(REQUIRED_ROLES),
        "acceptance_gates": list(GATES),
        "claim": "Reference project scaffold; not an industrial-qualified workcell",
    }


def inspect_workcell(file: str | Path) -> dict:
    """Fail closed on integrity, missing mechanisms, and missing evidence.

    Strict required roles and ten evidence gates apply to EVERY individual
    workcell; no exceptions for a particular robot brand/model/type.
    """
    root_file = Path(file).resolve()
    result = {
        "schema": SCHEMA, "file": str(root_file),
        "status": "REJECTED", "passed_engineering_gate": False,
        "certified": False, "errors": [], "blockers": [],
        "equipment": [], "evidence": {}, "required_roles": sorted(REQUIRED_ROLES),
        "required_gates": list(GATES),
        "source_qualified": False,
    }
    errors = result["errors"]
    blockers = result["blockers"]
    try:
        d = _load_json(root_file)
        if d.get("schema") != SCHEMA:
            raise ValueError("Wrong workcell schema")
        identifier = d.get("workcell_id", "")
        if not isinstance(identifier, str) or not _ID.fullmatch(identifier):
            raise ValueError("Invalid workcell identifier")
        result["workcell_id"] = identifier
        if not isinstance(d.get("name"), str) or not d["name"].strip():
            raise ValueError("Workcell name missing")
        if not isinstance(d.get("revision"), str) or not d["revision"].strip():
            raise ValueError("Workcell revision missing")
        if d.get("units") != "SI":
            raise ValueError("Canonical units must be SI (meter, kilogram, second)")
        from .environments import PRESETS
        if d.get("environment") not in PRESETS:
            raise ValueError("Unsupported engineering environment")
        result["environment"] = d["environment"]
        equipment = d.get("equipment")
        if not isinstance(equipment, list) or not 1 <= len(equipment) <= MAX_EQUIPMENT:
            raise ValueError(f"Workcell requires 1-{MAX_EQUIPMENT} machine assemblies")
        seen = set()
        seen_asset_ids = set()
        roles = set()
        fixture_only = False
        for instance in equipment:
            if not isinstance(instance, dict):
                raise ValueError("Equipment entry must be an object")
            instance_id = instance.get("instance_id", "")
            role = instance.get("role")
            if not isinstance(instance_id, str) or not _ID.fullmatch(instance_id):
                raise ValueError("Equipment instance_id must use snake_case")
            if instance_id in seen:
                raise ValueError("Duplicate equipment instance_id")
            seen.add(instance_id)
            if not isinstance(role,str) or role not in ROLES:
                raise ValueError(f"Invalid equipment role: {role!r}")
            roles.add(role)
            position = _xyz(instance.get("base_position_m"), label="equipment base pose")
            source = _relative(root_file.parent, instance.get("manifest"))
            checksum = instance.get("manifest_sha256")
            if not isinstance(checksum, str) or not _HEX.fullmatch(checksum):
                raise ValueError("Equipment manifest SHA256 must be lowercase hex")
            if not source.is_file() or _digest(source) != checksum:
                raise ValueError(f"Equipment manifest integrity failed for {instance_id}")
            manifest, details = validate_robot_manifest(source)
            asset_id = manifest["asset_id"]
            if asset_id in seen_asset_ids:
                raise ValueError(
                    "A machine asset_id can only be loaded once per Bullet workcell; "
                    "use distinct provenance instances or a future instancing backend"
                )
            seen_asset_ids.add(asset_id)
            if manifest["asset_class"] == "test_fixture":
                fixture_only = True
            result["equipment"].append({
                "instance_id": instance_id, "role": role,
                "asset_id": asset_id, "asset_class": manifest["asset_class"],
                "manifest": str(source), "base_position_m": list(position),
                "link_count": len(details.links), "joint_count": len(details.joints),
                "actuated_joint_count": sum(
                    j.kind != "fixed" for j in details.joints
                ),
                "source_integrity_verified": True,
            })
        if "robot" not in roles:
            blockers.append("Missing required robotic mechanism role")
        for role in sorted(REQUIRED_ROLES - roles):
            blockers.append(f"Missing required workcell role: {role}")
        robot = [e for e in result["equipment"] if e["role"] == "robot"]
        if not any(e["actuated_joint_count"] for e in robot):
            blockers.append("No controlled movable robot articulation")
        interface = d.get("interface")
        if not isinstance(interface, dict) or not isinstance(interface.get("required_sensors"), list):
            blockers.append("Sensor interface contract is missing")
        elif not interface["required_sensors"]:
            blockers.append("At least one real sensor contract must be declared")
        elif (any(not isinstance(x,str) or x not in REAL_SENSOR_CONTRACTS
                  for x in interface["required_sensors"])
              or len(interface["required_sensors"])!=len(set(interface["required_sensors"]))):
            errors.append("Sensor contract uses unknown or duplicate simulated sensor types")
        evidence = d.get("acceptance_evidence")
        if not isinstance(evidence, dict):
            raise ValueError("acceptance_evidence must be a dictionary")
        for key in GATES:
            record = evidence.get(key)
            if record is None:
                blockers.append(f"Missing independent acceptance evidence: {key}")
                result["evidence"][key] = "MISSING"
                continue
            if not isinstance(record, dict):
                errors.append(f"Invalid evidence document for {key}")
                continue
            try:
                source = _relative(root_file.parent, record.get("file"))
                sha = record.get("sha256")
                if not isinstance(sha, str) or not _HEX.fullmatch(sha):
                    raise ValueError("Evidence checksum invalid")
                if not source.is_file() or not 0 < source.stat().st_size <= MAX_EVIDENCE:
                    raise ValueError("Evidence file missing or too large")
                if _digest(source) != sha:
                    raise ValueError("Evidence checksum differs")
                result["evidence"][key] = "FILE_HASH_VERIFIED_ONLY"
                # Do not infer an independent engineering review from a file.
                blockers.append(
                    f"{key}: file integrity exists, independent technical "
                    "acceptance and uncertainty remain unverified"
                )
            except ValueError as exc:
                errors.append(f"{key}: {exc}")
        if fixture_only:
            blockers.append(
                "At least one assembly is a synthetic test fixture, "
                "not measured licensed industrial equipment"
            )
        # No automation can prove independent technical review solely from
        # user-submitted files. Industrial status requires signed external
        # qualification and real-device/physical validation in a later phase.
        result["source_qualified"] = not fixture_only
        if errors:
            result["status"] = "REJECTED"
        else:
            result["status"] = "INCOMPLETE" if blockers else "PENDING_INDEPENDENT_REVIEW"
    except (ValueError, OSError, KeyError, json.JSONDecodeError) as exc:
        errors.append(str(exc))
        result["status"] = "REJECTED"
    return result


def audit_workcell(file: str | Path, *, steps: int = 120) -> dict:
    """Execute *all* declared URDF assemblies in one isolated Bullet scene.

    Source-validation errors prevent execution. Missing completeness gates
    still allow an explicitly labeled reference simulation, but it cannot
    be promoted to industrial qualification.
    """
    if type(steps) is not int or not 1 <= steps <= 960:
        raise ValueError("Audit step count must be an integer in [1,960]")
    checked = inspect_workcell(file)
    if checked["errors"]:
        return {
            **checked, "runtime_executed": False,
            "runtime_passed": False, "runtime": None,
        }
    from .bullet_runtime import BulletPlayground, BulletSettings
    from .environments import ReferenceEnvironment
    from .sensors import BulletSensors
    with BulletPlayground(BulletSettings(payload_count=0)) as world:
        environment = ReferenceEnvironment(world)
        environment_report = environment.build(checked["environment"])
        runtime_parts = []
        for part in checked["equipment"]:
            imported = world.import_urdf_robot(
                part["manifest"], base_position_m=part["base_position_m"]
            )
            states = []
            for joint in imported["joints"]:
                if joint["lower"] is None:
                    continue
                state = world.p.getJointState(
                    imported["body_id"], joint["joint_index"],
                    physicsClientId=world.client,
                )[0]
                low, high = joint["lower"], joint["upper"]
                # Probe one small motion within physical limits. The
                # simulated actuator is never connected to real hardware.
                if high > low:
                    neutral = max(low, min(high, float(state)))
                    target = min(high, max(low, neutral + .10*(high-low)))
                else:
                    target = .1
                command = world.command_imported_joint(
                    part["asset_id"], joint["name"], target
                )
                states.append({
                    "name": joint["name"],
                    "initial": float(state),
                    "commanded": target,
                    "units": command["units"],
                })
            runtime_parts.append({
                "asset_id": part["asset_id"],
                "instance_id": part["instance_id"],
                "body_id": imported["body_id"],
                "joints_commanded": states,
            })
        measured = world.step(steps)
        lidar = BulletSensors(world).lidar_2d(
            count=24, origin_m=(-1.5,0.,.7), max_range_m=5.
        )
        actual_state = {
            entry["asset_id"]: entry
            for entry in measured["imported_robots"]
        }
        finite = True
        for part in runtime_parts:
            measured_joints = {
                joint["name"]: joint["position"]
                for joint in actual_state[part["asset_id"]]["joint_states"]
            }
            for joint in part["joints_commanded"]:
                joint["observed"] = measured_joints[joint["name"]]
                joint["delta_from_initial"] = (
                    joint["observed"] - joint["initial"]
                )
                if not math.isfinite(joint["observed"]):
                    finite = False
        report = {
            "solver": "pybullet_cpu",
            "steps": steps,
            "dt_seconds": 1/240,
            "environment": environment_report,
            "assemblies": runtime_parts,
            "num_contacts_observed": measured["num_contacts"],
            "lidar_rays_measured": len(lidar["measurements"]),
            "finite_joint_feedback": finite,
            "no_hardware_control": True,
            "physics_validated_for_industry": False,
        }
        return {
            **checked, "runtime_executed": True,
            "runtime_passed": finite,
            "runtime": report,
            "certified": False,
        }


def create_catalog(file: str | Path, workcells: list[str | Path]) -> dict:
    """Build a hashed, portable multi-workcell catalog with no curated exceptions."""
    target = Path(file).resolve()
    if target.exists():
        raise FileExistsError("Catalog file exists; cannot overwrite work")
    if not isinstance(workcells, list) or not 1 <= len(workcells) <= MAX_CATALOG:
        raise ValueError(f"Catalog build requires 1-{MAX_CATALOG} workcell files")
    entries = []
    used = set()
    for source in workcells:
        current = Path(source).resolve()
        if not current.is_file():
            raise FileNotFoundError(f"Workcell project missing: {current}")
        if not current.is_relative_to(target.parent):
            raise ValueError("Workcell catalog entries must be portable relative files")
        report = inspect_workcell(current)
        if report["errors"]:
            raise ValueError(
                "Workcell source cannot enter catalog: "+ "; ".join(report["errors"])
            )
        identifier = report["workcell_id"]
        if identifier in used:
            raise ValueError("Duplicate workcell ID: "+identifier)
        used.add(identifier)
        entries.append({
            "file":current.relative_to(target.parent).as_posix(),
            "sha256":_digest(current),
        })
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps({
        "schema":CATALOG_SCHEMA,
        "workcells":entries,
        "note":"Integrity registry; never an industrial certification roster",
    },indent=2)+"\n",encoding="utf-8")
    return {"catalog":str(target),"total":len(entries),
            "integrity_sources_verified":True,
            "every_workcell_certified":False}


def audit_catalog(path: str | Path, *, simulate: bool = False,
                  steps: int = 120) -> dict:
    """Audit every catalogued workcell, never first-only or one curated demo."""
    source = Path(path).resolve()
    catalog = _load_json(source)
    if catalog.get("schema") != CATALOG_SCHEMA:
        raise ValueError("Invalid workcell catalog schema")
    entries = catalog.get("workcells")
    if not isinstance(entries, list) or not 1 <= len(entries) <= MAX_CATALOG:
        raise ValueError("Workcell catalog must have 1-64 entries")
    reports = []
    seen = set()
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("Catalog entry must be an object")
        file = _relative(source.parent, entry.get("file"))
        sha = entry.get("sha256")
        if not isinstance(sha,str) or not _HEX.fullmatch(sha):
            raise ValueError("Catalog checksum missing")
        if not file.is_file() or _digest(file) != sha:
            raise ValueError(f"Catalog workcell checksum mismatch: {file.name}")
        report = (
            audit_workcell(file, steps=steps)
            if simulate else inspect_workcell(file)
        )
        ident = report.get("workcell_id")
        if ident in seen:
            raise ValueError(f"Duplicate workcell ID in catalog: {ident}")
        seen.add(ident)
        reports.append(report)
    return {
        "catalog": str(source), "total": len(reports),
        "results": reports,
        "rejected": sum(r["status"]=="REJECTED" for r in reports),
        "incomplete": sum(r["status"]=="INCOMPLETE" for r in reports),
        "runtime_checks_executed": simulate,
        "all_industrially_certified": False,
        "all_source_integrity_valid": all(not r["errors"] for r in reports),
        "all_runtime_checks_passed": (
            all(r.get("runtime_passed") for r in reports)
            if simulate else None
        ),
        "note": "No catalog entry can become industrial-certified based on self-reported evidence.",
    }
