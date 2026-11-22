import hashlib
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.usd_stage import attach_asset, inspect_stage, new_stage


@unittest.skipUnless(importlib.util.find_spec("pxr"), "usd-core/pxr not installed")
class OpenUsdTests(unittest.TestCase):
    def setUp(self):
        self.tmp = TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)

    def test_new_stage_is_canonical_and_has_no_fake_assets(self):
        scene = self.folder / "site.usda"
        result = new_stage(scene)
        report = inspect_stage(scene)
        self.assertEqual(result["up_axis"], "Z")
        self.assertEqual(report["meters_per_unit"], 1.0)
        self.assertEqual(report["counts"]["physics_scenes"], 1)
        self.assertEqual(report["counts"]["meshes"], 0)
        self.assertEqual(report["counts"]["rigid_body_prims"], 0)
        with self.assertRaises(FileExistsError):
            new_stage(scene)

    def test_references_real_usd_and_rejects_duplicate_prim(self):
        from pxr import Gf, Usd, UsdGeom
        asset_path = self.folder / "fixture.usda"
        source = Usd.Stage.CreateNew(str(asset_path))
        UsdGeom.SetStageUpAxis(source, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(source, 1.0)
        root = UsdGeom.Xform.Define(source, "/Fixture")
        source.SetDefaultPrim(root.GetPrim())
        mesh = UsdGeom.Mesh.Define(source, "/Fixture/Body")
        mesh.CreatePointsAttr([Gf.Vec3f(0, 0, 0), Gf.Vec3f(1, 0, 0), Gf.Vec3f(0, 1, 0)])
        mesh.CreateFaceVertexCountsAttr([3])
        mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
        source.GetRootLayer().Save()
        (self.folder / "RIGHTS.txt").write_text("Test-author fixture", encoding="utf-8")
        manifest = {
            "schema": "dsiq-asset-v1",
            "asset_id": "fixture_assembly",
            "manufacturer": "Unit Test Author",
            "model": "Test Triangle (not OEM CAD)",
            "revision": "1",
            "source_uri": "local://unit-test",
            "license": {"usage_rights": "owned", "evidence_file": "RIGHTS.txt"},
            "geometry": {
                "usd_file": "fixture.usda",
                "sha256": hashlib.sha256(asset_path.read_bytes()).hexdigest(),
                "meters_per_unit": 1, "up_axis": "Z",
                "dimensions_m": [1, 1, 0.1],
                "subassemblies": [{"name": "Body", "prim_path": "/Fixture/Body"}],
            },
            "physics": {"qualified": False},
        }
        manifest_path = self.folder / "asset.manifest.json"
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
        scene = self.folder / "site.usda"
        new_stage(scene)
        attached = attach_asset(scene, manifest_path)
        self.assertEqual(attached["prim_path"], "/World/Equipment/fixture_assembly")
        report = inspect_stage(scene)
        self.assertGreaterEqual(report["counts"]["meshes"], 1)
        with self.assertRaises(FileExistsError):
            attach_asset(scene, manifest_path)


if __name__ == "__main__":
    unittest.main()
