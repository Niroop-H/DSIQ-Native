"""Actual USD scene authoring and PBR material composition checks."""
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

HAS_BOTH=importlib.util.find_spec("pxr") is not None and importlib.util.find_spec("pybullet") is not None

@unittest.skipUnless(HAS_BOTH,"Requires usd-core plus Bullet")
class OpenUSDFacilityTests(unittest.TestCase):
    def test_export_is_genuine_meter_scale_usd_with_semantics_and_materials(self):
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        from dsiq_native.environments import ReferenceEnvironment
        from dsiq_native.environment_usd import export_reference_environment
        from pxr import Usd,UsdGeom,UsdPhysics,UsdShade
        with TemporaryDirectory() as folder:
            path=Path(folder)/"robotics-cell.usda"
            with BulletPlayground(BulletSettings(payload_count=0)) as world:
                env=ReferenceEnvironment(world)
                report=env.build("robotics_cell")
                out=export_reference_environment(env.parts,path,env.preset)
                self.assertEqual(out["prims_authored"],report["part_count"])
                self.assertGreater(out["static_colliders"],20)
                self.assertGreater(out["usd_preview_materials"],5)
                self.assertTrue(path.is_file())
                with self.assertRaises(FileExistsError):
                    export_reference_environment(env.parts,path,env.preset)
            stage=Usd.Stage.Open(str(path))
            self.assertAlmostEqual(UsdGeom.GetStageMetersPerUnit(stage),1.)
            self.assertEqual(str(UsdGeom.GetStageUpAxis(stage)),"Z")
            parts=[prim for prim in stage.Traverse() if prim.GetName().startswith("Part_")]
            self.assertEqual(len(parts),out["prims_authored"])
            self.assertTrue(all(prim.GetCustomDataByKey("dsiq_role") for prim in parts))
            collider=sum(prim.HasAPI(UsdPhysics.CollisionAPI) for prim in parts)
            self.assertEqual(collider,out["static_colliders"])
            materials=[prim for prim in stage.Traverse() if prim.IsA(UsdShade.Material)]
            self.assertEqual(len(materials),out["usd_preview_materials"])

    def test_cleanroom_and_warehouse_export_distinct_real_usd(self):
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        from dsiq_native.environments import ReferenceEnvironment
        from dsiq_native.environment_usd import export_reference_environment
        from pxr import Usd
        with TemporaryDirectory() as folder:
            with BulletPlayground(BulletSettings(payload_count=0)) as world:
                env=ReferenceEnvironment(world)
                counts=[]
                for preset in ("cleanroom_bay","logistics_bay"):
                    env.build(preset)
                    path=Path(folder)/(preset+".usda")
                    report=export_reference_environment(env.parts,path,preset)
                    counts.append(report["prims_authored"])
                    self.assertTrue(Usd.Stage.Open(str(path)))
                self.assertNotEqual(counts[0],counts[1])

if __name__=="__main__":
    unittest.main()
