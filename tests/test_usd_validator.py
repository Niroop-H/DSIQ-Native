"""Read-only real OpenUSD engineering validation contract tests.

Fixtures contain authored Pixar USD, not text-parsed substitutes.
"""
from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

HAS_USD=importlib.util.find_spec("pxr") is not None


@unittest.skipUnless(HAS_USD,"Real usd-core is optional")
class USDSceneValidatorTests(unittest.TestCase):
    def _scene(self, directory, *, valid=True):
        from pxr import Usd, UsdGeom, Gf
        path=Path(directory)/"validator_sample.usda"
        stage=Usd.Stage.CreateNew(str(path))
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage,1.0)
        root=UsdGeom.Xform.Define(stage,"/World")
        stage.SetDefaultPrim(root.GetPrim())
        mesh=UsdGeom.Mesh.Define(stage,"/World/Fixture")
        mesh.CreatePointsAttr([
            Gf.Vec3f(0.,0.,0.),
            Gf.Vec3f(1.,0.,0.),
            Gf.Vec3f(0.,1.,0.),
        ])
        mesh.CreateFaceVertexCountsAttr([3 if valid else 4])
        mesh.CreateFaceVertexIndicesAttr([0,1,2])
        stage.GetRootLayer().Save()
        return path

    def test_real_stage_has_finite_topology_and_sha_preserved(self):
        from dsiq_native.usd_validator import validate_usd_stage
        with TemporaryDirectory() as root:
            path=self._scene(root)
            before=hashlib.sha256(path.read_bytes()).hexdigest()
            result=validate_usd_stage(path)
            self.assertTrue(result["passed"],result["errors"])
            self.assertEqual(result["counts"]["mesh_points"],3)
            self.assertEqual(result["counts"]["triangles_estimated"],1)
            self.assertEqual(result["counts"]["meshes"],1)
            self.assertEqual(result["counts"]["mesh_without_material"],1)
            self.assertEqual(result["stage_sha256_before"],before)
            self.assertEqual(result["stage_sha256_after"],before)
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),before)
            self.assertFalse(result["policy"]["payloads_loaded"])

    def test_composed_mesh_topology_mismatch_rejected(self):
        from dsiq_native.usd_validator import validate_usd_stage
        with TemporaryDirectory() as root:
            path=self._scene(root,valid=False)
            result=validate_usd_stage(path)
            self.assertFalse(result["passed"])
            self.assertIn("MESH_TOPOLOGY_COUNT",
                         [e["rule_id"] for e in result["errors"]])

    def test_zero_area_collinear_triangle_and_repeated_face_vertices(self):
        from pxr import Usd, UsdGeom, Gf
        from dsiq_native.usd_validator import validate_usd_stage
        with TemporaryDirectory() as root:
            path=self._scene(root)
            stage=Usd.Stage.Open(str(path))
            mesh=UsdGeom.Mesh.Get(stage,"/World/Fixture")
            mesh.GetPointsAttr().Set([
                Gf.Vec3f(0.,0.,0.),
                Gf.Vec3f(1.,1.,1.),
                Gf.Vec3f(2.,2.,2.),
            ])
            stage.GetRootLayer().Save()
            result=validate_usd_stage(path)
            self.assertFalse(result["passed"])
            self.assertEqual(result["counts"]["degenerate_mesh_faces"],1)
            self.assertIn("MESH_DEGENERATE_GEOMETRY",
                          [e["rule_id"] for e in result["errors"]])
            mesh.GetPointsAttr().Set([
                Gf.Vec3f(0.,0.,0.),
                Gf.Vec3f(1.,0.,0.),
                Gf.Vec3f(0.,1.,0.),
            ])
            mesh.GetFaceVertexIndicesAttr().Set([0,1,1])
            stage.GetRootLayer().Save()
            result=validate_usd_stage(path)
            self.assertEqual(result["counts"]["degenerate_mesh_faces"],1)

    def test_tiny_non_degenerate_triangle_is_not_falsely_rejected(self):
        from pxr import Usd, UsdGeom, Gf
        from dsiq_native.usd_validator import validate_usd_stage
        with TemporaryDirectory() as root:
            path=self._scene(root)
            stage=Usd.Stage.Open(str(path))
            mesh=UsdGeom.Mesh.Get(stage,"/World/Fixture")
            mesh.GetPointsAttr().Set([
                Gf.Vec3f(0.,0.,0.),
                Gf.Vec3f(1e-9,0.,0.),
                Gf.Vec3f(0.,1e-9,0.),
            ])
            stage.GetRootLayer().Save()
            result=validate_usd_stage(path)
            self.assertTrue(result["passed"],result["errors"])
            self.assertEqual(result["counts"]["degenerate_mesh_faces"],0)

    def test_usd_stage_missing_canonical_units_and_default_is_rejected(self):
        from pxr import Usd, UsdGeom
        from dsiq_native.usd_validator import validate_usd_stage
        with TemporaryDirectory() as root:
            path=Path(root)/"bad.usda"
            stage=Usd.Stage.CreateNew(str(path))
            UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.y)
            UsdGeom.SetStageMetersPerUnit(stage,.01)
            UsdGeom.Xform.Define(stage,"/Other")
            stage.GetRootLayer().Save()
            result=validate_usd_stage(path)
            ids={x["rule_id"] for x in result["errors"]}
            self.assertTrue({"USD_DEFAULT_PRIM","DSIQ_Z_UP","DSIQ_METRIC_UNITS"}<=ids)
            relaxed=validate_usd_stage(
                path,require_metric_z_up=False,require_default_prim=False
            )
            self.assertTrue(relaxed["passed"])

    def test_fail_closed_on_prim_budget(self):
        from dsiq_native.usd_validator import validate_usd_stage
        with TemporaryDirectory() as root:
            path=self._scene(root)
            with self.assertRaisesRegex(ValueError,"limit"):
                validate_usd_stage(path,max_prims=1)
            with self.assertRaisesRegex(ValueError,"max_prims"):
                validate_usd_stage(path,max_prims=True)
            with self.assertRaisesRegex(ValueError,"max_prims"):
                validate_usd_stage(path,max_prims=20001)

    def test_cli_real_usd_validation_success_and_failure(self):
        from dsiq_native.cli import main
        from contextlib import redirect_stdout
        from io import StringIO
        import json
        with TemporaryDirectory() as root:
            path=self._scene(root)
            stdout=StringIO()
            with redirect_stdout(stdout):
                code=main(["usd-validate",str(path)])
            self.assertIn(code,(0,None))
            self.assertTrue(json.loads(stdout.getvalue())["passed"])
            bad=Path(root)/"bad_copy.usda"
            content=path.read_text(encoding="utf8")
            content=content.replace("metersPerUnit = 1", "metersPerUnit = 0.01")
            bad.write_text(content,encoding="utf8")
            stdout=StringIO()
            with redirect_stdout(stdout):
                code=main(["usd-validate",str(bad)])
            self.assertEqual(code,3)
            self.assertFalse(json.loads(stdout.getvalue())["passed"])

if __name__=="__main__":
    unittest.main()
