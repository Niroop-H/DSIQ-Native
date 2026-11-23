"""Independent offline update trust anchor, rollback and tamper checks."""
from __future__ import annotations
import base64
import hashlib
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

HAVE_CRYPTO=importlib.util.find_spec("cryptography") is not None
from dsiq_native.secure_update import SCHEMA, verify_offline_update


@unittest.skipUnless(HAVE_CRYPTO,"Install .[security] to verify real Ed25519")
class OfflineUpdateTests(unittest.TestCase):
    def setUp(self):
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
        from cryptography.hazmat.primitives.serialization import Encoding,PublicFormat
        self.tmp=TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)
        self.private=Ed25519PrivateKey.generate()  # independent test key, NEVER distributed
        self.public=self.path/"pinned-public-key.txt"
        self.public.write_text(base64.b64encode(
            self.private.public_key().public_bytes(Encoding.Raw,PublicFormat.Raw)
        ).decode("ascii"))
        self.bundle=self.path/"DSIQ-Native-1.2.3.zip"
        self.bundle.write_bytes(b"fake unit-test-only package bytes")
        self.manifest=self.path/"update.json"
        self.sig=self.path/"update.sig"
        self.info={
            "schema":SCHEMA,
            "release_version":"1.2.3",
            "archive_name":self.bundle.name,
            "archive_sha256":hashlib.sha256(self.bundle.read_bytes()).hexdigest(),
            "archive_bytes":self.bundle.stat().st_size,
        }
        self.resign()

    def resign(self):
        source=json.dumps(self.info,sort_keys=True,separators=(",",":")).encode()
        self.manifest.write_text(json.dumps(self.info,indent=2))
        self.sig.write_text(base64.b64encode(self.private.sign(source)).decode())

    def check(self,version="1.2.2"):
        return verify_offline_update(
            manifest_file=self.manifest,
            signature_file=self.sig,
            public_key_file=self.public,
            archive_file=self.bundle,
            installed_version=version,
        )

    def test_valid_signature_exact_bytes_and_upgrading_version(self):
        r=self.check()
        self.assertTrue(r["package_bytes_verified"])
        self.assertTrue(r["ed25519_detached_manifest_signature_verified"])
        self.assertFalse(r["authorized_for_industrial_distribution"])
        self.assertFalse(r["automatic_installer_execution"])

    def test_tamper_and_rollback_blocked(self):
        with self.assertRaisesRegex(ValueError,"rollback"):
            self.check("1.2.3")
        with self.assertRaisesRegex(ValueError,"rollback"):
            self.check("1.3.0")
        self.bundle.write_bytes(b"tampered package with same file path")
        with self.assertRaisesRegex(ValueError,"SHA256"):
            self.check()

    def test_fake_signature_or_other_signer_rejected(self):
        from cryptography.hazmat.primitives.asymmetric.ed25519 import Ed25519PrivateKey
        from cryptography.hazmat.primitives.serialization import Encoding,PublicFormat
        stranger=Ed25519PrivateKey.generate()
        self.public.write_text(base64.b64encode(
            stranger.public_key().public_bytes(Encoding.Raw,PublicFormat.Raw)
        ).decode())
        with self.assertRaisesRegex(ValueError,"signature"):
            self.check()

    def test_signed_malicious_path_is_refused(self):
        self.info["archive_name"]="../bad.zip"
        self.resign()
        with self.assertRaisesRegex(ValueError,"name mismatch"):
            self.check()


if __name__=="__main__":
    unittest.main()
