"""Real Bullet scene editor hierarchy, transformations and revision history."""
import importlib.util
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

HAS_BULLET=importlib.util.find_spec("pybullet") is not None


@unittest.skipUnless(HAS_BULLET,"Bullet optional dependency required")
class NativeSceneEditorTests(unittest.TestCase):
    def setUp(self):
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        from dsiq_native.scene_editor import SceneEditor
        self.world=BulletPlayground(BulletSettings(payload_count=1))
        self.addCleanup(self.world.close)
        self.editor=SceneEditor(self.world)

    def test_real_payload_transform_undo_redo(self):
        initial=self.editor.entities()
        self.assertEqual(len(initial),3)
        self.assertFalse(next(x for x in initial if x["asset_id"]=="reference_6r")["editable"])
        before=next(x["position_m"] for x in initial if x["asset_id"]=="payload_000")
        move=self.editor.move("payload_000",(1.1,-.4,1.2))
        self.assertTrue(move["changed"])
        actual=next(x["position_m"] for x in self.editor.entities()
                    if x["asset_id"]=="payload_000")
        self.assertLess(math.dist(actual,(1.1,-.4,1.2)),1e-7)
        self.editor.undo()
        undo=next(x["position_m"] for x in self.editor.entities()
                  if x["asset_id"]=="payload_000")
        self.assertLess(math.dist(undo,before),1e-7)
        self.editor.redo()
        redo=next(x["position_m"] for x in self.editor.entities()
                  if x["asset_id"]=="payload_000")
        self.assertLess(math.dist(redo,(1.1,-.4,1.2)),1e-7)
        self.assertIn("NOT_PHYSICAL_MOTION",move["operation"])

    def test_rejects_locked_and_nonfinite_edits(self):
        with self.assertRaises(ValueError):
            self.editor.move("ground",(1.,0.,0.))
        with self.assertRaises(ValueError):
            self.editor.move("reference_6r",(1.,0.,0.))
        with self.assertRaises(ValueError):
            self.editor.move("no_such_component",(1.,0.,0.))
        with self.assertRaises(ValueError):
            self.editor.move("payload_000",(float("nan"),0.,0.))
        with self.assertRaises(ValueError):
            self.editor.move("payload_000",(1000.,0.,0.))
        with self.assertRaises(ValueError):
            self.editor.undo()

    def test_perspective_raycast_picks_real_viewport_entity(self):
        hit=self.editor.pick_viewport(
            eye_m=(-1.4,3.,1.),
            look_at_m=(-1.4,1.05,1.),
            pixel_xy=(79.5,59.5),
            image_wh=(160,120),
        )
        self.assertTrue(hit["hit"])
        self.assertEqual(hit["asset_id"],"payload_000")
        self.assertTrue(hit["editable"])
        self.assertEqual(hit["selection_type"],"GEOMETRY_RAYCAST_REAL_BULLET")
        self.assertTrue(hit["hit_position_m"])
        with self.assertRaises(ValueError):
            self.editor.pick_viewport((0,0,1),(0,0,0),(-1,0),(160,120))
        with self.assertRaises(ValueError):
            self.editor.pick_viewport((0,0,1),(0,0,1),(2,2),(160,120))

    def test_imported_robot_pose_can_be_edited_with_real_bullet(self):
        from dsiq_native.machine_designer import MachineDesign,save_project
        with TemporaryDirectory() as tmp:
            project=save_project(MachineDesign(asset_id="editor_user_gantry"),Path(tmp)/"gantry")
            imported=self.world.import_urdf_robot(project["manifest"],
                  base_position_m=(-.7,1.2,0.))
            self.assertEqual(imported["joint_count"],4)
            elements=self.editor.entities()
            self.assertTrue(any(x["asset_id"]=="editor_user_gantry" for x in elements))
            self.editor.move("editor_user_gantry",(.5,1.7,.05))
            pos=self.world.p.getBasePositionAndOrientation(
                imported["body_id"],physicsClientId=self.world.client
            )[0]
            self.assertAlmostEqual(pos[0],.5,places=7)
            self.assertAlmostEqual(pos[1],1.7,places=7)
            self.editor.undo()
            original=self.world.p.getBasePositionAndOrientation(
                imported["body_id"],physicsClientId=self.world.client
            )[0]
            self.assertAlmostEqual(original[0],-.7,places=7)
            self.assertEqual(self.editor.status()["undo_depth"],0)


if __name__=="__main__":
    unittest.main()
