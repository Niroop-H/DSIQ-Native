"""Portable local dependency path policy, tested even without installed USD."""
from pathlib import Path
from tempfile import TemporaryDirectory
import os
import unittest

from dsiq_native.usd_dependency_audit import _locate_local


class DependencyPathPolicyTests(unittest.TestCase):
    def test_real_local_absolute_and_relative_paths_are_approved(self):
        with TemporaryDirectory() as tmp:
            root=Path(tmp).resolve()
            file=root/"asset.usda"
            file.write_bytes(b"test")
            for requested in (str(file),"asset.usda"):
                with self.subTest(path=requested):
                    resolved,error=_locate_local(requested,root=root,stage_dir=root)
                    self.assertEqual(resolved,file)
                    self.assertIsNone(error)

    def test_external_root_and_remote_asset_rejected(self):
        with TemporaryDirectory() as tmp:
            root=Path(tmp).resolve()/"trust"
            root.mkdir()
            for requested in ("../outside.usda","https://evil.invalid/a.usda",
                              "asset://system/a.usda","//server/share/a.usda",
                              "\\\\server\\share\\evil.usda","", "anon:a"):
                with self.subTest(path=requested):
                    resolved,error=_locate_local(requested,root=root,stage_dir=root)
                    self.assertIsNone(resolved)
                    self.assertIsNotNone(error)

    @unittest.skipUnless(os.name!="nt", "Foreign Windows drives are only foreign on POSIX")
    def test_windows_drive_on_posix_fails_closed(self):
        with TemporaryDirectory() as tmp:
            root=Path(tmp).resolve()
            _,error=_locate_local(r"C:\external\model.usda",root=root,stage_dir=root)
            self.assertEqual(error,"REMOTE_OR_UNSUPPORTED_IDENTIFIER")

    def test_symlink_escaping_root_does_not_pass(self):
        with TemporaryDirectory() as tmp:
            parent=Path(tmp).resolve()
            root=parent/"approved"
            root.mkdir()
            target=parent/"outside.usda"
            target.write_bytes(b"not trusted")
            link=root/"asset.usda"
            try:
                link.symlink_to(target)
            except (OSError,NotImplementedError):
                self.skipTest("Platform does not permit test symlink creation")
            resolved,error=_locate_local("asset.usda",root=root,stage_dir=root)
            self.assertIsNone(resolved)
            self.assertEqual(error,"OUTSIDE_APPROVED_ASSET_ROOT")

if __name__=="__main__":
    unittest.main()
