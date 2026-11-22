"""Environment presets built from semantic, colliding facility structures."""
import importlib.util
import unittest

HAS_BULLET=importlib.util.find_spec("pybullet") is not None

@unittest.skipUnless(HAS_BULLET,"PyBullet optional dependency required")
class EnvironmentsTests(unittest.TestCase):
    def setUp(self):
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        from dsiq_native.environments import ReferenceEnvironment
        self.world=BulletPlayground(BulletSettings(payload_count=0))
        self.addCleanup(self.world.close)
        self.env=ReferenceEnvironment(self.world)

    def test_each_preset_is_structurally_distinct_and_semantic(self):
        kinds=[]
        for preset in ("robotics_cell","cleanroom_bay","logistics_bay"):
            report=self.env.build(preset)
            self.assertGreater(report["part_count"],35,preset)
            self.assertLessEqual(report["part_count"],200)
            self.assertEqual(len(report["semantic_part_names"]),
                             len(set(report["semantic_part_names"])))
            self.assertIn("not_verified_oem",report["fidelity"])
            kinds.append(set(report["semantic_roles"]))
        self.assertNotEqual(kinds[0],kinds[1])
        self.assertNotEqual(kinds[0],kinds[2])
        self.assertNotEqual(kinds[1],kinds[2])

    def test_environment_clear_does_not_delete_robot_or_floor(self):
        original=self.world.robot
        floor=self.world.ground
        scene=self.env.build("robotics_cell")
        self.assertGreater(scene["collision_parts"],10)
        body_ids=[p.body_id for p in self.env.parts]
        self.assertTrue(all(self.world.p.getBodyInfo(id,physicsClientId=self.world.client)
                            for id in body_ids))
        result=self.env.build("open")
        self.assertEqual(result["part_count"],0)
        self.assertEqual(self.world.robot,original)
        self.assertEqual(self.world.ground,floor)
        self.assertEqual(self.world.p.getNumJoints(original,physicsClientId=self.world.client),6)
        self.world.step(12)

    def test_invalid_preset_does_not_destroy_current_scene(self):
        self.env.build("cleanroom_bay")
        parts=len(self.env.parts)
        with self.assertRaises(ValueError):
            self.env.build("random_decorative_factory")
        self.assertEqual(len(self.env.parts),parts)

if __name__=="__main__":
    unittest.main()
