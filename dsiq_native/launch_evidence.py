"""Source-linked launch evidence inventory for the eight outstanding P0/P1 gates.

The validator verifies original file integrity and consistent evidence
identifiers. This is NOT an automatic assessment of industrial competence.
Actual qualification needs an independent authorized assessor and,
where applicable, properly certified safety components.
"""
from __future__ import annotations

from collections import OrderedDict
import hashlib
import json
from pathlib import Path
import re

SCHEMA="dsiq-launch-evidence-inventory-v1"
SHA=re.compile(r"^[a-f0-9]{64}$")
REQUIREMENTS=OrderedDict((
    ("P0_ROBOT_SAFETY",
     "Independent ISO 10218:2025 robot-cell risk and safety-function validation"),
    ("P0_MACHINE_CALIBRATION",
     "Licensed OEM CAD, measured masses/inertia and traceable actuator calibration"),
    ("P0_RELEASE_SECURITY",
     "Real publisher signing, license review, SBOM, security audit and update rollout"),
    ("P1_RTX3050_PERFORMANCE",
     "Actual target-device GPU VRAM/FPS/latency/soak and out-of-memory evidence"),
    ("P1_EDITOR_WORKFLOWS",
     "Representative industrial CAD assembly editor UX and scalability acceptance"),
    ("P1_ROS2_HARDWARE",
     "Hardware-in-loop ROS2 controller and independent physical E-stop validation"),
    ("P1_PHYSICAL_AI",
     "Independent policy benchmark, domain shift and real machine transfer study"),
    ("P1_CUSTOMER_ACCEPTANCE",
     "Customer controlled pilot, recorded defects and signed acceptance"),
))


def _digest(path):
    h=hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda:f.read(1024*1024),b""):
            h.update(block)
    return h.hexdigest()


def _safe_local(root: Path, relative: str) -> Path:
    if (not isinstance(relative,str) or not relative or
            "\\" in relative or relative.startswith("/") or
            ":" in relative or
            ".." in Path(relative).parts):
        raise ValueError("Evidence path must be local, relative and traversal-free")
    result=(root/relative).resolve()
    if not result.is_relative_to(root):
        raise ValueError("Evidence escapes its audited directory")
    return result


def audit_launch_evidence(manifest_file: str|Path) -> dict:
    file=Path(manifest_file).resolve()
    if not file.is_file() or file.stat().st_size>262_144:
        raise ValueError("Missing or oversized launch-evidence inventory")
    root=file.parent
    doc=json.loads(file.read_text("utf-8"))
    if not isinstance(doc,dict) or doc.get("schema")!=SCHEMA:
        raise ValueError("Launch-evidence schema mismatch")
    evidence=doc.get("evidence")
    if not isinstance(evidence,dict):
        raise ValueError("Launch-evidence categories required")
    if set(evidence)-set(REQUIREMENTS):
        raise ValueError("Unknown launch-evidence category")
    reports={}
    n_valid=0
    for key,reason in REQUIREMENTS.items():
        entry=evidence.get(key)
        if entry is None:
            reports[key]={"state":"MISSING","requirement":reason}
            continue
        if not isinstance(entry,dict):
            raise ValueError(f"{key}: expected evidence object")
        attachments=entry.get("attachments")
        if not isinstance(attachments,list) or not 1<=len(attachments)<=16:
            reports[key]={"state":"MISSING_ATTACHMENT","requirement":reason}
            continue
        records=[]
        all_match=True
        for evidence_entry in attachments:
            if not isinstance(evidence_entry,dict):
                raise ValueError(f"{key}: invalid attachment entry")
            p=_safe_local(root,evidence_entry.get("file"))
            sha=evidence_entry.get("sha256")
            if not isinstance(sha,str) or not SHA.fullmatch(sha):
                raise ValueError(f"{key}: attachment SHA-256 is required")
            if p==file or not p.is_file() or p.stat().st_size>100*1024*1024:
                status="MISSING_OR_OVERSIZED"
                all_match=False
            else:
                actual=_digest(p)
                status="SHA256_VERIFIED" if actual==sha else "SHA256_MISMATCH"
                all_match=all_match and status=="SHA256_VERIFIED"
            records.append({"file":evidence_entry["file"],
                            "integrity_status":status})
        if all_match:
            n_valid+=1
        reports[key]={
            "state":"FILE_INTEGRITY_ONLY" if all_match else "UNTRUSTED_OR_TAMPERED",
            "requirement":reason,
            "files":records,
            "independent_reviewer_authority_verified":False,
            "physical_or_commercial_qualification_confirmed":False,
        }
    return {
        "schema":"dsiq-launch-inventory-audit-v1",
        "category_count":len(REQUIREMENTS),
        "file_integrity_categories_verified":n_valid,
        "missing_or_failed_categories":len(REQUIREMENTS)-n_valid,
        "categories":reports,
        "all_source_files_integrity_verified":n_valid==len(REQUIREMENTS),
        "independent_technical_assessment_completed":False,
        "production_or_industrial_launch_authorized":False,
        "score_out_of_100":"NOT_EARNED_FROM_FILE_EXISTENCE",
        "release_gate":"BLOCKED_PENDING_INDEPENDENT_EVIDENCE_AND_SIGNOFF",
        "note":"No submitted file, hash, signature string or CI result is automatic proof of industrial acceptance.",
    }
