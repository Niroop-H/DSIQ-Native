"""OpenUSD visual model is derived from verified URDF, animated by real Bullet."""
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

BOTH=(importlib.util.find_spec("pxr") is not None
      and importlib.util.find_spec("pybullet") is not None)


@unittest.skipUnless(BOTH,"Real Bullet and Pixar OpenUSD required")
class URDFVisualUSDTests(unittest.TestCase):
    def test_animated_source_geometry_and_bullet_link_position_match(self):
        from dsiq_native.machine_designer import MachineDesign,save_project
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        from dsiq_native.usd_urdf_visuals import record_urdf_visual_usd
        from pxr import Usd,UsdGeom
        with TemporaryDirectory() as folder:
            src=save_project(
                MachineDesign(asset_id="anim_source_urdf_gantry"),
                Path(folder)/"gantry"
            )
            out=Path(folder)/"source_visual.usda"
            with BulletPlayground(BulletSettings(payload_count=0,
                                                  gravity_m_s2=0.)) as world:
                robot=world.import_urdf_robot(src["manifest"],
                                              base_position_m=(-1.,1.3,0.))
                world.command_imported_joint(robot["asset_id"],"actuator_x",.25)
                report=record_urdf_visual_usd(
                    world,src["manifest"],out,frames=4,steps_per_frame=12
                )
                self.assertEqual(report["link_tracks"],5)
                self.assertGreaterEqual(report["visual_shapes"],8)
                self.assertFalse(report["converted_imported_meshes"])
                self.assertFalse(report["oem_calibration_verified"])
                stage=Usd.Stage.Open(str(out))
                prim=next(x for x in stage.Traverse()
                          if x.GetCustomDataByKey("dsiq_original_urdf_link_name")=="x_axis_carriage")
                attr=prim.GetAttribute("xformOp:translate")
                self.assertEqual(attr.GetTimeSamples(),[12.,24.,36.,48.])
                actual_idx=next(j["joint_index"] for j in robot["joints"]
                                if j["name"]=="actuator_x")
                observed=world.p.getLinkState(
                    robot["body_id"],actual_idx,computeForwardKinematics=True,
                    physicsClientId=world.client
                )[4]
                recorded=attr.Get(Usd.TimeCode(48))
                self.assertAlmostEqual(recorded[0],observed[0],places=7)
                self.assertAlmostEqual(recorded[1],observed[1],places=7)
                self.assertAlmostEqual(recorded[2],observed[2],places=7)
                geometries=[
                    x for x in stage.Traverse() if x.IsA(UsdGeom.Gprim)
                ]
                self.assertEqual(len(geometries),report["visual_shapes"])
                self.assertGreater(
                    sum(bool(x.GetAttribute("size")) for x in geometries),1
                )
                with self.assertRaises(FileExistsError):
                    record_urdf_visual_usd(world,src["manifest"],out,frames=1)

    def test_bad_capture_bounds_rejected_before_file_created(self):
        from dsiq_native.usd_urdf_visuals import record_urdf_visual_usd
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        with TemporaryDirectory() as folder:
            with BulletPlayground(BulletSettings(payload_count=0)) as world:
                for bad in (0,121):
                    with self.assertRaises(ValueError):
                        record_urdf_visual_usd(
                            world,"does-not-exist",Path(folder)/"bad.usda",
                            frames=bad
                        )
                self.assertFalse((Path(folder)/"bad.usda").exists())


if __name__=="__main__":
    unittest.main()
