"""Fail-closed provenance gate for real industrial OpenUSD assets.

Checking a manifest or checksum never constitutes physics/geometry certification.
A USD stage must also be opened by Pixar's USD parser before attaching it.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Any


_ALLOWED_USD = {".usd", ".usda", ".usdc", ".usdz"}
_ID = re.compile(r"^[a-z][a-z0-9_]{2,63}$")
_SHA256 = re.compile(r"^[a-fA-F0-9]{64}$")


@dataclass(frozen=True)
class Validation:
    asset_id: str
    valid_for_import: bool
    simulation_qualified: bool
    errors: tuple[str, ...]
    warnings: tuple[str, ...]

    def as_dict(self) -> dict:
        d = asdict(self)
        d["errors"] = list(self.errors)
        d["warnings"] = list(self.warnings)
        return d


def _within_manifest_dir(manifest: Path, relative_path: str) -> Path:
    if not isinstance(relative_path, str) or not relative_path:
        raise ValueError("Expected nonempty local relative file path")
    p = Path(relative_path)
    if p.is_absolute() or ".." in p.parts:
        raise ValueError("Paths must be relative and may not traverse parent directories")
    root = manifest.resolve().parent
    resolved = (root / p).resolve()
    if not resolved.is_relative_to(root):
        raise ValueError("File path escapes the asset manifest directory")
    return resolved


def _checksum(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def load_manifest(path: str | Path) -> dict[str, Any]:
    p = Path(path)
    content = json.loads(p.read_text(encoding="utf-8"))
    if not isinstance(content, dict):
        raise ValueError("Asset manifest root must be a JSON object")
    return content


def validate_asset(path: str | Path) -> Validation:
    manifest = Path(path)
    try:
        m = load_manifest(manifest)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        return Validation("", False, False, (f"Cannot read manifest: {exc}",), ())
    errors: list[str] = []
    warnings: list[str] = []
    id_value = m.get("asset_id", "")
    if not isinstance(id_value, str) or not _ID.fullmatch(id_value):
        errors.append("asset_id must be lowercase snake_case, 3–64 characters")
    if m.get("schema") != "dsiq-asset-v1":
        errors.append("schema must equal dsiq-asset-v1")
    for key in ("manufacturer", "model", "revision", "source_uri"):
        value = m.get(key)
        if not isinstance(value, str) or not value.strip():
            errors.append(f"{key} is required; do not fabricate OEM metadata")

    license_info = m.get("license")
    if not isinstance(license_info, dict):
        errors.append("license requires usage_rights and evidence_file")
        license_info = {}
    if license_info.get("usage_rights") not in ("owned", "licensed", "open"):
        errors.append("license.usage_rights must be owned, licensed, or open")
    try:
        license_evidence = _within_manifest_dir(manifest, license_info.get("evidence_file", ""))
        if not license_evidence.is_file() or not license_evidence.stat().st_size:
            errors.append("license.evidence_file must exist and not be empty")
    except ValueError as exc:
        errors.append(f"license.evidence_file: {exc}")

    geometry = m.get("geometry")
    if not isinstance(geometry, dict):
        errors.append("geometry must be an object")
        geometry = {}
    geometry_file: Path | None = None
    try:
        geometry_file = _within_manifest_dir(manifest, geometry.get("usd_file", ""))
        if geometry_file.suffix.lower() not in _ALLOWED_USD:
            errors.append("geometry.usd_file must be .usd/.usda/.usdc/.usdz")
        if not geometry_file.is_file():
            errors.append("geometry.usd_file is missing: import the actual approved CAD/USD asset")
    except ValueError as exc:
        errors.append(f"geometry.usd_file: {exc}")

    expected_hash = geometry.get("sha256")
    if not isinstance(expected_hash, str) or not _SHA256.fullmatch(expected_hash):
        errors.append("geometry.sha256 must be the 64-character SHA-256 of the actual USD file")
    elif geometry_file and geometry_file.is_file() and _checksum(geometry_file) != expected_hash.lower():
        errors.append("geometry.sha256 does not match the on-disk USD asset")

    units = geometry.get("meters_per_unit")
    if isinstance(units, bool) or not isinstance(units, (int, float)) or not math.isfinite(units) or units <= 0:
        errors.append("geometry.meters_per_unit must be a finite positive value")
    if geometry.get("up_axis") not in ("Y", "Z"):
        errors.append("geometry.up_axis must be Y or Z")
    dims = geometry.get("dimensions_m")
    if (not isinstance(dims, list) or len(dims) != 3
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) or x <= 0 for x in dims)):
        errors.append("geometry.dimensions_m must have three positive finite numbers")

    parts = geometry.get("subassemblies")
    if not isinstance(parts, list) or not parts:
        errors.append("geometry.subassemblies must identify real assembly names and prim paths")
    else:
        unique_paths: set[str] = set()
        for i, part in enumerate(parts):
            if (not isinstance(part, dict) or
                    not isinstance(part.get("name"), str) or not part["name"].strip() or
                    not isinstance(part.get("prim_path"), str) or not part["prim_path"].startswith("/")):
                errors.append(f"geometry.subassemblies[{i}] needs name and absolute USD prim_path")
                continue
            if part["prim_path"] in unique_paths:
                errors.append(f"Duplicate subassembly prim_path: {part['prim_path']}")
            unique_paths.add(part["prim_path"])

    physics = m.get("physics", {})
    if not isinstance(physics, dict):
        physics = {}
        errors.append("physics must be an object")
    qualified = physics.get("qualified") is True
    if qualified:
        for key in ("mass_kg", "acceptance_report"):
            if key not in physics:
                errors.append(f"physics.qualified=true requires {key}")
        mass = physics.get("mass_kg")
        if isinstance(mass, bool) or not isinstance(mass, (int, float)) or not math.isfinite(mass) or mass <= 0:
            errors.append("Qualified physics requires positive, finite mass_kg")
        try:
            evidence = _within_manifest_dir(manifest, physics.get("acceptance_report", ""))
            if not evidence.is_file() or not evidence.stat().st_size:
                errors.append("Qualified physics requires a nonempty acceptance_report")
        except ValueError as exc:
            errors.append(f"physics.acceptance_report: {exc}")
        warnings.append(
            "Reported qualification is metadata only: independent physics acceptance "
            "and reproducibility checks remain necessary."
        )
    else:
        warnings.append(
            "No independently qualified physics: asset may be viewed but must not "
            "be represented as a calibrated or safety-approved simulation."
        )
    warnings.append(
        "Manifest checks identity, provenance and on-disk hash. It does not establish "
        "geometric accuracy, executable contacts, OEM licensing sufficiency, or fidelity."
    )
    return Validation(
        id_value if isinstance(id_value, str) else "",
        not errors, qualified and not errors,
        tuple(errors), tuple(warnings),
    )
