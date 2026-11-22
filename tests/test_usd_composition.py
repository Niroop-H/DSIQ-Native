"""Actual OpenUSD composition: sublayers, external geometry variants and edits."""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

HAS_USD=importlib.util.find_spec("pxr") is not None


@unittest.skipUnless(HAS_USD,"Official Pixar OpenUSD bindings required")
class USDCompositionTests(unittest.TestCase):
    def setUp(self):
        from dsiq_native.usd_composition import new_workspace
        self.temp=TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.project=self.root/"engineering_project"
        new_workspace(self.project)
        self.asset=self._asset("hi",side=1.0)
        self.low=self._asset("low",side=.9)

    def _asset(self,name,*,side):
        from pxr import Usd,UsdGeom,Gf
        folder=self.root/name
        folder.mkdir()
        source=folder/"machine.usda"
        stage=Usd.Stage.CreateNew(str(source))
        UsdGeom.SetStageMetersPerUnit(stage,1.)
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        model=UsdGeom.Xform.Define(stage,"/LicensedFixture")
        stage.SetDefaultPrim(model.GetPrim())
        mesh=UsdGeom.Mesh.Define(stage,"/LicensedFixture/DetailedPanel")
        half=side/2
        points=[
            Gf.Vec3f(-half,-half,-half),Gf.Vec3f(half,-half,-half),
            Gf.Vec3f(half,half,-half),Gf.Vec3f(-half,half,-half),
            Gf.Vec3f(-half,-half,half),Gf.Vec3f(half,-half,half),
            Gf.Vec3f(half,half,half),Gf.Vec3f(-half,half,half),
        ]
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([4]*6)
        mesh.CreateFaceVertexIndicesAttr([
            0,1,2,3,4,5,6,7,0,1,5,4,
            1,2,6,5,2,3,7,6,3,0,4,7,
        ])
        stage.GetRootLayer().Save()
        (folder/"RIGHTS.txt").write_text(
            "Self-authored geometric unit test fixture, not OEM equipment.",
            encoding="utf-8",
        )
        manifest={
            "schema":"dsiq-asset-v1",
            "asset_class":"test_fixture",
            "asset_id":"user_test_fixture",
            "manufacturer":"Test Fixture Author",
            "model":"Original cube for composition API test only",
            "revision":"fixture-v1",
            "source_uri":"local://dsiq-fixture-geometry",
            "license":{"usage_rights":"owned","evidence_file":"RIGHTS.txt"},
            "geometry":{
                "usd_file":"machine.usda",
                "sha256":hashlib.sha256(source.read_bytes()).hexdigest(),
                "meters_per_unit":1.0,"up_axis":"Z",
                "dimensions_m":[side,side,side],
                "subassemblies":[
                    {"name":"DetailedPanel",
                     "prim_path":"/LicensedFixture/DetailedPanel"}
                ],
            },
            "physics":{"qualified":False},
        }
        path=folder/"asset.manifest.json"
        path.write_text(json.dumps(manifest),encoding="utf-8")
        return path

    def test_create_real_composed_workspace_and_source_variant(self):
        from dsiq_native.usd_composition import (
            add_verified_asset,inspect_workspace,new_workspace,
        )
        from pxr import Usd,UsdGeom
        report=add_verified_asset(self.project,self.asset,
                                  alternative_manifest=self.low)
        self.assertTrue(report["verified_geometry"])
        self.assertIn("source",report["variant_choices"])
        self.assertIn("low_memory",report["variant_choices"])
        self.assertGreaterEqual(report["source_triangles"],6)
        stage=Usd.Stage.Open(str(self.project/"project.usda"))
        self.assertEqual(
            list(stage.GetRootLayer().subLayerPaths),
            ["layers/layout.usda","layers/equipment.usda"],
        )
        prim=stage.GetPrimAtPath(report["prim_path"])
        self.assertTrue(prim)
        mesh=stage.GetPrimAtPath(report["prim_path"]+"/DetailedPanel")
        self.assertTrue(mesh.IsA(UsdGeom.Mesh))
        self.assertFalse(prim.GetCustomDataByKey("dsiq_physics_metadata_only"))
        listing=inspect_workspace(self.project)
        self.assertEqual(listing["asset_count"],1)
        self.assertEqual(listing["assets"][0]["representation"],"source")
        self.assertTrue(listing["assets"][0]["usd_geometry_authored"])
        with self.assertRaises(FileExistsError):
            add_verified_asset(self.project,self.asset)

    def test_variant_selection_and_transform_roundtrip_in_separate_layers(self):
        from dsiq_native.usd_composition import (
            add_verified_asset,place_asset,set_representation,inspect_workspace,
        )
        from pxr import Usd,UsdGeom
        add_verified_asset(self.project,self.asset,
                           alternative_manifest=self.low)
        selection=set_representation(self.project,"user_test_fixture","low_memory")
        self.assertEqual(selection["representation"],"low_memory")
        transform=place_asset(self.project,"user_test_fixture",
                              (1.0,-.25,.55),rotation_z_deg=30.)
        self.assertTrue(transform["edit_layer"].endswith("layout.usda"))
        inspection=inspect_workspace(self.project)
        self.assertEqual(inspection["assets"][0]["representation"],"low_memory")
        self.assertAlmostEqual(inspection["assets"][0]["translation_m"][0],1.)
        self.assertAlmostEqual(inspection["assets"][0]["rotation_xyz_deg"][2],30.)
        with self.assertRaises(ValueError):
            set_representation(self.project,"user_test_fixture","invented_mesh")
        with self.assertRaises(ValueError):
            place_asset(self.project,"user_test_fixture",(float("nan"),0,0))

    def test_integrity_and_identity_fail_before_any_stage_mutation(self):
        from dsiq_native.usd_composition import (
            add_verified_asset,inspect_workspace,
        )
        with self.assertRaises(ValueError):
            add_verified_asset(self.project,self.asset,
                               alternative_manifest=self.root/"missing")
        self.assertEqual(inspect_workspace(self.project)["asset_count"],0)
        m=json.loads(self.asset.read_text(encoding="utf-8"))
        m["geometry"]["sha256"]="0"*64
        self.asset.write_text(json.dumps(m),encoding="utf-8")
        with self.assertRaisesRegex(ValueError,"provenance"):
            add_verified_asset(self.project,self.asset)
        self.assertEqual(inspect_workspace(self.project)["asset_count"],0)

    def test_new_project_never_overwrites_user_work(self):
        from dsiq_native.usd_composition import new_workspace
        with self.assertRaises(FileExistsError):
            new_workspace(self.project)


if __name__=="__main__":
    unittest.main()
