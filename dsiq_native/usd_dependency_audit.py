"""Read-only OpenUSD *resolved dependency* manifest and portability gate.

Uses Pixar's actual UsdUtils.ComputeAllDependencies, NOT a text regex over
USDA files. Finds reachable layers, non-layer assets, and unresolved paths.
The returned files are then source-hashed under an explicit trusted root.

Security boundary: this is an audit, NOT an isolation sandbox for arbitrary
USD resolver plugins. USD may consult an installed Ar resolver while exploring
the stage. Do not pass untrusted remote URI stages to USD without a separately
restricted resolver/sandbox. No files are created, modified, or downloaded
by this module's own code.

It does not prove external intellectual property rights, validate payload
geometry, inspect archive-internal USDZ assets, or certify a SimReady model.
"""
from __future__ import annotations

import hashlib
import os
from pathlib import Path
import re

from .usd_stage import _existing_layer

USD_SUFFIXES = {".usd", ".usda", ".usdc"}
MAX_FILES = 256
MAX_TOTAL_BYTES = 256 * 1024 * 1024
MAX_FILE_BYTES = 96 * 1024 * 1024
MAX_ISSUES = 128
_WINDOWS_ABS = re.compile(r"^[A-Za-z]:[\\/]")


def _read_digest(path: Path) -> tuple[str, int]:
    """Hash by chunk with a pre/post size and mtime consistency check."""
    before = path.stat()
    if not path.is_file() or before.st_size > MAX_FILE_BYTES:
        raise ValueError("Dependency missing, not a file, or over file-size budget")
    sha = hashlib.sha256()
    total = 0
    with path.open("rb") as stream:
        while True:
            chunk = stream.read(1024 * 1024)
            if not chunk:
                break
            total += len(chunk)
            if total > MAX_FILE_BYTES:
                raise ValueError("Dependency grew beyond file-size budget while hashing")
            sha.update(chunk)
    after = path.stat()
    if total != before.st_size or total != after.st_size or before.st_mtime_ns != after.st_mtime_ns:
        raise RuntimeError("Dependency changed during read-only SHA-256 audit")
    return sha.hexdigest(), total


def _locate_local(identifier: str, *, root: Path, stage_dir: Path) -> tuple[Path | None, str | None]:
    """Interpret actual resolved identifiers, not a custom remote resolver."""
    if not isinstance(identifier, str) or not identifier.strip():
        return None, "EMPTY_DEPENDENCY_PATH"
    # Native Windows absolute drives are allowed only on Windows and only
    # after resolving beneath the explicitly approved asset root.
    foreign_windows_drive = bool(_WINDOWS_ABS.match(identifier)) and os.name!="nt"
    if ("://" in identifier or identifier.startswith(("anon:", "asset:", "data:"))
            or identifier.startswith(("//", "\\\\")) or foreign_windows_drive):
        return None, "REMOTE_OR_UNSUPPORTED_IDENTIFIER"
    # An in-package USDA identifier like archive.usdz[foo.usda] is not
    # an ordinary on-disk asset and must not be disguised as one.
    if "[" in identifier or "]" in identifier or "?" in identifier or "#" in identifier:
        return None, "PACKAGE_OR_DYNAMIC_IDENTIFIER"
    path = Path(identifier)
    if not path.is_absolute():
        path = stage_dir / path
    resolved = path.resolve()
    if not resolved.is_relative_to(root):
        return None, "OUTSIDE_APPROVED_ASSET_ROOT"
    return resolved, None


def audit_usd_dependencies(
    scene: str | Path, *, asset_root: str | Path | None = None,
    max_files: int = MAX_FILES, max_total_bytes: int = MAX_TOTAL_BYTES,
) -> dict:
    """Return source-hashed local dependencies discovered by REAL OpenUSD.

    A missing/unapproved resource yields passed=false; a resource budget
    violation raises ValueError, never an incomplete green result.
    """
    if type(max_files) is not int or not 1 <= max_files <= MAX_FILES:
        raise ValueError("max_files must be an integer between 1 and 256")
    if type(max_total_bytes) is not int or not 1 <= max_total_bytes <= MAX_TOTAL_BYTES:
        raise ValueError("max_total_bytes must be an integer between 1 and 268435456")
    p = _existing_layer(scene)
    if p.suffix.lower() not in USD_SUFFIXES:
        raise ValueError("Dependency audit requires on-disk .usd/.usda/.usdc scene (USDZ unsupported)")
    trusted = Path(asset_root).resolve() if asset_root is not None else p.parent.resolve()
    if not trusted.is_dir() or not p.is_relative_to(trusted):
        raise ValueError("The scene must live under an existing approved asset_root")
    from pxr import UsdUtils
    try:
        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(str(p))
    except Exception as exc:
        raise RuntimeError("Pixar OpenUSD could not enumerate dependency graph: " + str(exc)) from exc
    if layers is None or assets is None or unresolved is None:
        raise RuntimeError("OpenUSD did not produce a complete dependency tuple")
    errors = []
    files = {}
    total_bytes = 0

    def problem(kind: str, requested: str, detail: str):
        if len(errors)>=MAX_ISSUES:
            raise ValueError("Dependency issue budget exceeded; no incomplete PASS")
        errors.append({"rule_id":kind, "asset":requested[:256], "detail":detail})

    def register(identifier: str, kind: str):
        nonlocal total_bytes
        target, fault = _locate_local(identifier, root=trusted, stage_dir=p.parent)
        if fault is not None:
            problem(fault, identifier, "Not an approved on-disk dependency path")
            return
        if target in files:
            files[target]["types"].add(kind)
            return
        if len(files) >= max_files:
            raise ValueError("Dependency count exceeds bounded audit budget")
        if not target.is_file():
            problem("MISSING_LOCAL_DEPENDENCY", identifier, "Dependency not found under approved root")
            return
        if target.stat().st_size > MAX_FILE_BYTES:
            raise ValueError("Dependency exceeds per-file inspection budget")
        digest, size = _read_digest(target)
        total_bytes += size
        if total_bytes > max_total_bytes:
            raise ValueError("Total dependency bytes exceed inspection budget")
        files[target] = {"sha256":digest, "size_bytes":size, "types":{kind}}

    register(str(p), "root_stage")
    for layer in layers:
        name = str(getattr(layer, "realPath", "") or getattr(layer, "identifier", ""))
        register(name, "usd_layer")
    for asset in assets:
        register(str(asset), "non_layer_asset")
    for asset in unresolved:
        problem("UNRESOLVED_USD_ASSET",str(asset),
                "OpenUSD's resolver could not locate the referenced file or payload")
    out = [{
        "path_relative_to_asset_root":str(k.relative_to(trusted).as_posix()),
        "sha256":v["sha256"],
        "size_bytes":v["size_bytes"],
        "types":sorted(v["types"]),
    } for k,v in sorted(files.items(),key=lambda item:str(item[0]))]
    # Catch changed root stage even if Pixar re-opened it during resolution.
    now_hash, now_size = _read_digest(p)
    if not files.get(p) or now_hash!=files[p]["sha256"] or now_size!=files[p]["size_bytes"]:
        raise RuntimeError("Root USD stage changed during dependency audit")
    return {
        "schema":"dsiq-openusd-local-dependency-audit-v1",
        "source_stage_relative_to_asset_root":str(p.relative_to(trusted).as_posix()),
        "asset_root":str(trusted),
        "passed":not errors,
        "dependency_file_count":len(out),
        "dependency_total_bytes":total_bytes,
        "verified_files":out,
        "errors":errors,
        "unresolved_count":len(unresolved),
        "policy":{
            "approved_local_root_enforced_on_reported_dependencies":True,
            "max_files":max_files,
            "max_total_bytes":max_total_bytes,
            "max_single_file_bytes":MAX_FILE_BYTES,
            "usdz_packages_not_inspected":True,
            "external_ar_resolvers_not_sandboxed":True,
        },
        "limitations":[
            "OpenUSD Ar resolver may run before paths are approved; use a sandbox for untrusted scenes",
            "Resolver enumeration alone cannot verify contents of unloaded USDZ packages",
            "Source SHA-256 establishes bytes, not machine calibration or rights ownership",
        ],
    }
