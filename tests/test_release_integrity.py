"""Release integrity gate rejects tampering, missing binaries and zip traversal."""
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from zipfile import ZipFile,ZIP_DEFLATED
from dsiq_native.release_integrity import make_attestation,verify_attestation


class ReleaseAttestationTests(unittest.TestCase):
    def test_exact_release_archive_and_tamper_rejection(self):
        with TemporaryDirectory() as temp:
            root=Path(temp)
            archive=root/"release.zip"
            report=root/"release.attestation.json"
            with ZipFile(archive,"w",ZIP_DEFLATED) as f:
                f.writestr("DSIQ-Native.exe",b"MZ"+b"\0"*50)
                f.writestr("_internal/physics.dll",b"actual bytes")
            generated=make_attestation(archive,report)
            self.assertTrue(generated["archive_integrity_verified"])
            checked=verify_attestation(archive,report)
            self.assertTrue(checked["integrity_verified"])
            self.assertEqual(checked["member_count"],2)
            self.assertFalse(checked["commercial_distribution_qualified"])
            with self.assertRaises(FileExistsError):
                make_attestation(archive,report)
            with ZipFile(archive,"a",ZIP_DEFLATED) as f:
                f.writestr("tampered.txt","extra")
            with self.assertRaisesRegex(ValueError,"SHA256"):
                verify_attestation(archive,report)

    def test_unsafe_zip_members_block_attestation(self):
        with TemporaryDirectory() as temp:
            root=Path(temp)
            for idx,name in enumerate(("../escape.txt","C:/windows.txt")):
                archive=root/f"unsafe{idx}.zip"
                with ZipFile(archive,"w") as f:
                    f.writestr("DSIQ-Native.exe","MZ")
                    f.writestr(name,"illegal")
                with self.subTest(path=name),self.assertRaisesRegex(
                    ValueError,"unsafe"
                ):
                    make_attestation(archive,root/f"out{idx}.json")

    def test_missing_executable_is_rejected(self):
        with TemporaryDirectory() as temp:
            root=Path(temp)
            archive=root/"unrelated.zip"
            with ZipFile(archive,"w") as f:
                f.writestr("README.txt","not a binary")
            with self.assertRaisesRegex(ValueError,"DSIQ-Native.exe"):
                make_attestation(archive,root/"out.json")

if __name__=="__main__":
    unittest.main()
