import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.asset_quality import validate_asset


class AssetValidationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.base = Path(self.tmp.name)
        (self.base / "source.usda").write_text(
            "#usda 1.0\n(def Xform \"Machine\" {})\n", encoding="utf-8"
        )
        (self.base / "RIGHTS.txt").write_text("Fixture license evidence only", encoding="utf-8")
        sha = hashlib.sha256((self.base / "source.usda").read_bytes()).hexdigest()
        self.manifest = {
            "schema": "dsiq-asset-v1",
            "asset_class": "test_fixture",
            "asset_id": "test_machine_fixture",
            "manufacturer": "Fixture Author",
            "model": "Unit Test Only",
            "revision": "1",
            "source_uri": "local://test-fixture-not-oem",
            "license": {"usage_rights": "owned", "evidence_file": "RIGHTS.txt"},
            "geometry": {
                "usd_file": "source.usda", "sha256": sha,
                "meters_per_unit": 1.0, "up_axis": "Z",
                "dimensions_m": [1.0, 2.0, 3.0],
                "subassemblies": [{"name": "Machine", "prim_path": "/Machine"}]
            },
            "physics": {"qualified": False},
        }
        self.path = self.base / "asset.manifest.json"
        self.save()

    def save(self):
        self.path.write_text(json.dumps(self.manifest), encoding="utf-8")

    def test_integrity_checks_are_not_physics_certification(self):
        result = validate_asset(self.path)
        self.assertTrue(result.valid_for_import, result.errors)
        self.assertFalse(result.simulation_qualified)
        self.assertTrue(any("not" in s.lower() for s in result.warnings))

    def test_file_tampering_is_detected(self):
        (self.base / "source.usda").write_text("tampered", encoding="utf-8")
        self.assertFalse(validate_asset(self.path).valid_for_import)

    def test_prevent_path_traversal(self):
        self.manifest["geometry"]["usd_file"] = "../unauthorized.usda"
        self.save()
        result = validate_asset(self.path)
        self.assertFalse(result.valid_for_import)
        self.assertTrue(any("relative" in e.lower() for e in result.errors))

    def test_missing_license_evidence_fails(self):
        (self.base / "RIGHTS.txt").unlink()
        self.assertFalse(validate_asset(self.path).valid_for_import)

    def test_qualified_flag_without_evidence_fails(self):
        self.manifest["physics"]["qualified"] = True
        self.manifest["physics"]["mass_kg"] = 20.0
        self.save()
        self.assertFalse(validate_asset(self.path).valid_for_import)


    def test_oem_visual_asset_requires_explicit_review(self):
        self.manifest["asset_class"] = "oem_verified"
        self.save()
        result = validate_asset(self.path)
        self.assertFalse(result.valid_for_import)
        self.assertTrue(any("visual_review" in error for error in result.errors))

    def test_review_evidence_record_is_required_and_distinct_from_physics(self):
        self.manifest["asset_class"] = "engineering_reference"
        (self.base / "REVIEW.txt").write_text(
            "Unit test document, NOT manufacturer qualification", encoding="utf-8"
        )
        self.manifest["visual_review"] = {
            "approved": True,
            "reviewer": "Unit Test",
            "reviewed_against": "local://test-reference",
            "evidence_file": "REVIEW.txt",
        }
        self.save()
        result = validate_asset(self.path)
        self.assertTrue(result.valid_for_import, result.errors)
        self.assertFalse(result.simulation_qualified)


if __name__ == "__main__":
    unittest.main()
