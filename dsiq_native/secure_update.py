"""Fail-closed local update *verification*, not an automatic updater.

The externally pinned Ed25519 public key is a required trust anchor.
A SHA256 manifest without a trusted signature is INSUFFICIENT.
No network, downloads, installer execution, private key storage or
automatic trust on first use are provided here.
"""
from __future__ import annotations

import base64
import hashlib
import json
import re
from pathlib import Path

SCHEMA="dsiq-signed-offline-update-v1"
NAME=re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{1,140}\.(?:zip|msix|msi|exe)$")
VERSION=re.compile(r"^[0-9]{1,4}\.[0-9]{1,4}\.[0-9]{1,4}$")


def _version(s):
    if not isinstance(s,str) or not VERSION.fullmatch(s):
        raise ValueError("Update version must be three numeric components (a.b.c)")
    return tuple(int(i) for i in s.split("."))


def _sha256(path):
    h=hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b""):
            h.update(chunk)
    return h.hexdigest()


def verify_offline_update(*, manifest_file: str|Path,
                          signature_file: str|Path,
                          public_key_file: str|Path,
                          archive_file: str|Path,
                          installed_version: str) -> dict:
    """Verify detached Ed25519 signature against independently pinned key.

    Signature covers canonical JSON bytes, not nondeterministic whitespace.
    Manifest specifies exact archive name, byte length, SHA256 and version.
    """
    from cryptography.exceptions import InvalidSignature
    from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PublicKey

    installed=_version(installed_version)
    manifest=Path(manifest_file).resolve()
    signature=Path(signature_file).resolve()
    public=Path(public_key_file).resolve()
    archive=Path(archive_file).resolve()
    for path,budget in ((manifest,16384),(signature,512),(public,512)):
        if not path.is_file() or not 0<path.stat().st_size<=budget:
            raise ValueError("Required signed update metadata missing/oversized")
    data=json.loads(manifest.read_text("utf-8"))
    if not isinstance(data,dict) or set(data)!={
        "schema","release_version","archive_name","archive_sha256","archive_bytes"
    } or data["schema"]!=SCHEMA:
        raise ValueError("Signed update manifest format/schema mismatch")
    future=_version(data["release_version"])
    if future<=installed:
        raise ValueError("Update rollback/downgrade or same version refused")
    name=data["archive_name"]
    if not isinstance(name,str) or not NAME.fullmatch(name) or archive.name!=name:
        raise ValueError("Update package name mismatch/unsafe path")
    sha=data["archive_sha256"]
    if not isinstance(sha,str) or not re.fullmatch(r"[a-f0-9]{64}",sha):
        raise ValueError("Signed manifest invalid digest")
    if type(data["archive_bytes"]) is not int or not 1<=data["archive_bytes"]<=1_073_741_824:
        raise ValueError("Signed update archive violates 1 GiB bound")
    try:
        raw_sig=base64.b64decode(signature.read_text("ascii").strip(),validate=True)
        raw_public=base64.b64decode(public.read_text("ascii").strip(),validate=True)
    except (UnicodeError,ValueError) as exc:
        raise ValueError("Invalid Ed25519 signature/public key encoding") from exc
    if len(raw_public)!=32 or len(raw_sig)!=64:
        raise ValueError("Invalid pinned Ed25519 key/signature length")
    canonical=json.dumps(data,sort_keys=True,separators=(",",":"),
                         allow_nan=False).encode("utf8")
    try:
        Ed25519PublicKey.from_public_bytes(raw_public).verify(raw_sig,canonical)
    except InvalidSignature as exc:
        raise ValueError("Update manifest signature is not trusted") from exc
    if not archive.is_file() or archive.stat().st_size!=data["archive_bytes"]:
        raise ValueError("Signed package missing or byte length does not match")
    actual_sha=_sha256(archive)
    if actual_sha!=sha:
        raise ValueError("Signed update package SHA256 mismatch")
    return {
        "schema":"dsiq-verified-offline-update-package-v1",
        "version_from":installed_version,
        "version_to":data["release_version"],
        "archive_sha256":sha,
        "ed25519_detached_manifest_signature_verified":True,
        "pinned_public_key_sha256":hashlib.sha256(raw_public).hexdigest(),
        "package_bytes_verified":True,
        "automatic_installer_execution":False,
        "publisher_identity_independently_established":False,
        "windows_authenticode_verified":False,
        "authorized_for_industrial_distribution":False,
        "note":"Verification does not install software or authenticate the independently supplied trust anchor.",
    }
