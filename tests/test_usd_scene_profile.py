"""Non-destructive composed OpenUSD geometry analysis; not an optimizer."""
from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

HAS_USD = importlib.util.find_spec("pxr") is not None


@unittest.skipUnless(HAS_USD,"Official Pixar OpenUSD bindings required")
class ReadOnlySceneProfileTests(unittest.TestCase):
    def setUp(self):
        from pxr import Gf, Usd, UsdGeom
        self.temp=TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/"composed-profile.usda"
        stage=Usd.Stage.CreateNew(str(self.path))
        UsdGeom.SetStageMetersPerUnit(stage,1.)
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        UsdGeom.Xform.Define(stage,"/World")
        points=[
            Gf.Vec3f(0.,0.,0.),Gf.Vec3f(.5,0.,0.),
            Gf.Vec3f(0.,.5,0.),
        ]
        for name in ("First","Repeated","Different","Animated"):
            mesh=UsdGeom.Mesh.Define(stage,"/World/"+name)
            mesh.CreatePointsAttr(points)
            mesh.CreateFaceVertexCountsAttr([3])
            mesh.CreateFaceVertexIndicesAttr([0,1,2])
            if name=="Different":
                mesh.GetPointsAttr().Set([
                    Gf.Vec3f(0.,0.,0.),Gf.Vec3f(.9,0.,0.),
                    Gf.Vec3f(0.,.5,0.),
                ])
            if name=="Animated":
                mesh.GetPointsAttr().Set(points,1.)
                mesh.GetPointsAttr().Set([
                    Gf.Vec3f(0.,0.,0.),Gf.Vec3f(.8,0.,0.),
                    Gf.Vec3f(0.,.5,0.),
                ],2.)
        stage.GetRootLayer().Save()

    def test_exact_geometry_reuse_is_read_only_and_not_claimed_safe(self):
        from dsiq_native.usd_scene_profile import profile_usd_scene
        before=hashlib.sha256(self.path.read_bytes()).hexdigest()
        result=profile_usd_scene(self.path)
        self.assertEqual(result["composed_stage_counts"]["meshes"],4)
        self.assertEqual(result["composed_stage_counts"]["time_varying_meshes"],1)
        self.assertEqual(result["exact_local_geometry_reuse_groups"],1)
        self.assertEqual(result["exact_local_geometry_reuse_meshes"],2)
        self.assertEqual(set(result["duplicate_groups"][0]["paths"]),
                         {"/World/First","/World/Repeated"})
        self.assertFalse(result["stage_authoring_performed"])
        self.assertFalse(result["automatic_instance_conversion_performed"])
        self.assertFalse(result["renderer_vram_or_fps_savings_measured"])
        self.assertEqual(before,hashlib.sha256(self.path.read_bytes()).hexdigest())

    def test_bounded_prim_count_fails_without_modifying_source(self):
        from dsiq_native.usd_scene_profile import profile_usd_scene
        before=self.path.read_bytes()
        with self.assertRaisesRegex(ValueError,"prim traversal budget"):
            profile_usd_scene(self.path,max_prims=1)
        with self.assertRaisesRegex(ValueError,"max_prims"):
            profile_usd_scene(self.path,max_prims=0)
        self.assertEqual(before,self.path.read_bytes())


if __name__=="__main__":
    unittest.main()
