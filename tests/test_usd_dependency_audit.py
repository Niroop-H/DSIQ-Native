"""Real Pixar USD layer/texture dependency audit and bounded path regression."""
from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest


@unittest.skipUnless(importlib.util.find_spec("pxr") is not None,
                     "Real usd-core required for dependency integration")
class USDDependencyAuditAcceptance(unittest.TestCase):
    def _write_scene(self, root: Path, *, missing=False, escape=False):
        from pxr import Gf, Sdf, Usd, UsdGeom
        root.mkdir(exist_ok=True)
        (root/"layers").mkdir(exist_ok=True)
        (root/"textures").mkdir(exist_ok=True)
        sub=root/"layers"/"component.usda"
        stage=Usd.Stage.CreateNew(str(sub))
        part=UsdGeom.Xform.Define(stage,"/Component")
        stage.SetDefaultPrim(part.GetPrim())
        material = stage.DefinePrim("/Component/Surface", "Scope")
        material.CreateAttribute(
            "source:baseColorTexture", Sdf.ValueTypeNames.Asset
        ).Set(Sdf.AssetPath("../textures/coating.png"))
        stage.GetRootLayer().Save()
        texture=root/"textures"/"coating.png"
        texture.write_bytes(b"\x89PNG\r\n\x1a\nDSIQ-reference-test-bytes")
        scene=root/"root.usda"
        entry=Usd.Stage.CreateNew(str(scene))
        world=UsdGeom.Xform.Define(entry,"/World")
        entry.SetDefaultPrim(world.GetPrim())
        child=UsdGeom.Xform.Define(entry,"/World/Imported")
        target="../outside.usda" if escape else (
            "layers/missing.usda" if missing else "layers/component.usda"
        )
        child.GetPrim().GetReferences().AddReference(target)
        entry.GetRootLayer().Save()
        return scene, texture, sub

    def test_exact_source_layer_and_texture_hashes(self):
        from dsiq_native.usd_dependency_audit import audit_usd_dependencies
        with TemporaryDirectory() as d:
            root=Path(d)/"assets"
            scene, texture, sub=self._write_scene(root)
            old={p:hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in (scene,texture,sub)}
            result=audit_usd_dependencies(scene,asset_root=root)
            self.assertTrue(result["passed"],result)
            self.assertEqual(result["unresolved_count"],0,result)
            files={x["path_relative_to_asset_root"]:x for x in result["verified_files"]}
            self.assertEqual(files["root.usda"]["sha256"],old[scene])
            self.assertEqual(files["layers/component.usda"]["sha256"],old[sub])
            self.assertEqual(files["textures/coating.png"]["sha256"],old[texture])
            self.assertEqual(len(files),3)
            for path, expected in old.items():
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),expected)
            self.assertTrue(result["policy"]["external_ar_resolvers_not_sandboxed"])

    def test_missing_reference_is_not_silently_accepted(self):
        from dsiq_native.usd_dependency_audit import audit_usd_dependencies
        with TemporaryDirectory() as d:
            root=Path(d)/"assets"
            scene, _, _=self._write_scene(root,missing=True)
            result=audit_usd_dependencies(scene,asset_root=root)
            self.assertFalse(result["passed"],result)
            self.assertGreater(result["unresolved_count"],0)
            self.assertIn("UNRESOLVED_USD_ASSET",
                          [x["rule_id"] for x in result["errors"]])

    def test_escape_from_approved_root_is_not_accepted(self):
        from dsiq_native.usd_dependency_audit import (
            _locate_local,audit_usd_dependencies
        )
        with TemporaryDirectory() as d:
            root=Path(d)/"assets"
            scene, _, _=self._write_scene(root,escape=True)
            escaped=root.parent/"outside.usda"
            from pxr import Usd,UsdGeom
            stage=Usd.Stage.CreateNew(str(escaped))
            world=UsdGeom.Xform.Define(stage,"/External")
            stage.SetDefaultPrim(world.GetPrim())
            stage.GetRootLayer().Save()
            result=audit_usd_dependencies(scene,asset_root=root)
            self.assertFalse(result["passed"],result)
            self.assertIn("OUTSIDE_APPROVED_ASSET_ROOT",
                          [x["rule_id"] for x in result["errors"]])
            self.assertEqual(_locate_local("https://bad.invalid/model.usda",
                                          root=root,stage_dir=root)[1],
                             "REMOTE_OR_UNSUPPORTED_IDENTIFIER")

    def test_budget_and_unsupported_usdz_are_fail_closed(self):
        from dsiq_native.usd_dependency_audit import audit_usd_dependencies
        with TemporaryDirectory() as d:
            root=Path(d)/"assets"
            scene,_,_=self._write_scene(root)
            with self.assertRaisesRegex(ValueError,"Dependency count"):
                audit_usd_dependencies(scene,asset_root=root,max_files=1)
            with self.assertRaisesRegex(ValueError,"bytes"):
                audit_usd_dependencies(scene,asset_root=root,max_total_bytes=1)
            with self.assertRaisesRegex(ValueError,"max_files"):
                audit_usd_dependencies(scene,asset_root=root,max_files=True)
            with self.assertRaisesRegex(ValueError,"asset_root"):
                audit_usd_dependencies(scene,asset_root=root/"not_exists")
            bad=root/"payload.usdz"
            bad.write_bytes(b"not a real USDZ package")
            with self.assertRaisesRegex(ValueError,"USDZ unsupported"):
                audit_usd_dependencies(bad,asset_root=root)

    def test_cli_dependency_audit_report(self):
        from contextlib import redirect_stdout
        from dsiq_native.cli import main
        from io import StringIO
        import json
        with TemporaryDirectory() as d:
            root=Path(d)/"assets"
            scene,_,_=self._write_scene(root)
            out=StringIO()
            with redirect_stdout(out):
                code=main(["usd-dependency-audit",str(scene),"--asset-root",str(root)])
            self.assertIn(code,(0,None))
            record=json.loads(out.getvalue())
            self.assertEqual(record["dependency_file_count"],3)
            self.assertTrue(record["passed"])

if __name__=="__main__":
    unittest.main()
