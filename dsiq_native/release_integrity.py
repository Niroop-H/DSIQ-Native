"""Portable release archive integrity attestation, NOT code signing.

Verifies ZIP member names, CRC, exact SHA256 bytes and aggregate size.
This cannot prove software publisher identity, absence of malware,
supply-chain provenance, or compatibility with a user's hardware.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import zipfile

SCHEMA="dsiq-unsigned-preview-integrity-v1"
MAX_ARCHIVE=1024*1024*1024
MAX_MEMBER=256*1024*1024
MAX_MEMBERS=4096


def _hash_stream(src):
    digest=hashlib.sha256()
    count=0
    for block in iter(lambda:src.read(1024*1024),b""):
        digest.update(block)
        count+=len(block)
    return digest.hexdigest(),count


def _inspect_zip(file):
    size=file.stat().st_size
    if not 0<size<=MAX_ARCHIVE:
        raise ValueError("Release ZIP exceeds constrained archive size")
    members={}
    total=0
    with zipfile.ZipFile(file,"r") as archive:
        infos=archive.infolist()
        if not 1<=len(infos)<=MAX_MEMBERS:
            raise ValueError("Invalid ZIP member count")
        for info in infos:
            name=info.filename.replace("\\","/")
            parts=PurePosixPath(name).parts
            if (not parts or name.startswith("/") or
                    re.match(r"^[a-zA-Z]:",name) or
                    ".." in parts or info.flag_bits&1):
                raise ValueError("Release has unsafe/encrypted archive member")
            if info.is_dir():
                continue
            if name in members:
                raise ValueError("Duplicate release ZIP path")
            if not 0<info.file_size<=MAX_MEMBER:
                raise ValueError("Invalid/oversized release member")
            total+=info.file_size
            if total>MAX_ARCHIVE*3:
                raise ValueError("Release expansion budget exceeded")
            with archive.open(info,"r") as stream:
                sha,observed_size=_hash_stream(stream)
            if observed_size!=info.file_size:
                raise ValueError("Decompressed member differs from ZIP directory")
            members[name]={"size":observed_size,"sha256":sha}
    if not any(path.lower().endswith(("dsiq-native.exe","dsiq-native-gpu.exe")) for path in members):
        raise ValueError("Release does not contain DSIQ-Native.exe or DSIQ-Native-GPU.exe")
    return members,total


def make_attestation(archive_file: str | Path, report_file: str | Path) -> dict:
    archive=Path(archive_file).resolve()
    report=Path(report_file).resolve()
    if report.exists():
        raise FileExistsError("Refusing to overwrite an existing release manifest")
    members,total=_inspect_zip(archive)
    with archive.open("rb") as stream:
        digest,size=_hash_stream(stream)
    summary={
        "schema":SCHEMA,
        "archive_filename":archive.name,
        "archive_sha256":digest,
        "archive_bytes":size,
        "member_count":len(members),
        "uncompressed_bytes":total,
        "members":members,
        "platform":"windows",
        "software_package":"DSIQ-Native-CPU-Preview",
        "unsigned_preview":True,
        "publisher_identity_cryptographically_verified":False,
        "binary_antimalware_scanned":False,
        "dependency_SBOM_independently_verified":False,
        "industrial_hardware_qualified":False,
        "signature_requirement_before_commercial_release":True,
    }
    report.parent.mkdir(parents=True,exist_ok=True)
    report.write_text(json.dumps(summary,indent=2)+"\n",encoding="utf8")
    return {
        "file":str(report),"member_count":len(members),
        "archive_sha256":digest,
        "archive_integrity_verified":True,
        "publisher_signature_verified":False,
    }


def verify_attestation(archive_file: str|Path, report_file: str|Path) -> dict:
    archive=Path(archive_file).resolve()
    report=Path(report_file).resolve()
    if not report.is_file() or report.stat().st_size>1024*1024:
        raise ValueError("Missing or oversized integrity attestation")
    d=json.loads(report.read_text(encoding="utf8"))
    if d.get("schema")!=SCHEMA or d.get("archive_filename")!=archive.name:
        raise ValueError("Release attestation schema/filename mismatch")
    with archive.open("rb") as stream:
        digest,size=_hash_stream(stream)
    if digest!=d["archive_sha256"] or size!=d["archive_bytes"]:
        raise ValueError("Release ZIP SHA256 or size does not match attestation")
    members,total=_inspect_zip(archive)
    if (members!=d["members"] or total!=d["uncompressed_bytes"] or
            len(members)!=d["member_count"]):
        raise ValueError("Release member content hash mismatch")
    return {
        "archive":str(archive),"integrity_verified":True,
        "member_count":len(members),
        "archive_sha256":digest,
        "authenticode_verified":False,
        "commercial_distribution_qualified":False,
    }
