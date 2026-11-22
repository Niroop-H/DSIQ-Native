"""DSIQ designed XYZ gantry: real mechanical links, joint limits and Bullet tests."""
import importlib.util
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.machine_designer import (
    MachineDesign, build_urdf, load_design, save_project
)
from dsiq_native.urdf_bridge import inspect_urdf,validate_robot_manifest

class MachineDesignerTests(unittest.TestCase):
    def test_three_real_axis_parts_and_bill_of_materials(self):
        design=MachineDesign(asset_id="test_user_gantry",x_travel_m=.7,y_travel_m=.6,z_travel_m=.25)
        content=build_urdf(design)
        self.assertIn(b'actuator_x',content)
        self.assertIn(b'actuator_y',content)
        self.assertIn(b'actuator_z',content)
        self.assertIn(b'x_linear_bearing',content)
        self.assertIn(b'tool_mounting_plate',content)
        self.assertIn(b'z_guided_slide',content)

    def test_writes_editable_project_and_real_urdf(self):
        with TemporaryDirectory() as tmp:
            folder=Path(tmp)/"new_machine"
            export=save_project(MachineDesign(asset_id="gantry_fixture"),folder)
            self.assertTrue(Path(export["design_json"]).is_file())
            design=load_design(export["design_json"])
            self.assertEqual(design.asset_id,"gantry_fixture")
            _,robot=validate_robot_manifest(export["manifest"])
            self.assertEqual(robot.root_link,"fixed_structural_frame")
            self.assertEqual(len(robot.joints),4)
            self.assertEqual(sum(j.kind=="prismatic" for j in robot.joints),3)
            self.assertEqual(len(robot.links_without_inertial),0)
            self.assertEqual(set(j.name for j in robot.joints),{
                "actuator_x","actuator_y","actuator_z","flange_attachment"
            })
            self.assertGreater(Path(export["urdf"]).stat().st_size,1000)
            with self.assertRaises(FileExistsError):
                save_project(design,folder)

    def test_configuration_is_bounded_and_invalid_units_rejected(self):
        with self.assertRaises(ValueError):
            MachineDesign(z_travel_m=2.).validate()
        with self.assertRaises(ValueError):
            MachineDesign(clearance_height_m=.8,z_travel_m=.7).validate()
        with self.assertRaises(ValueError):
            MachineDesign(x_motor_effort_n=-1.).validate()
        with self.assertRaises(ValueError):
            MachineDesign(asset_id="../evil").validate()
        with self.assertRaises(ValueError):
            MachineDesign.from_dict({"schema":"dsiq-machine-v1",
                "mechanism":"xyz_gantry","parameters":{"unrecognized":1}})

    def test_json_editing_changes_real_mechanism_stroke(self):
        with TemporaryDirectory() as tmp:
            project=save_project(MachineDesign(x_travel_m=1.8),Path(tmp)/"machine")
            data=json.loads(Path(project["design_json"]).read_text())
            self.assertEqual(data["parameters"]["x_travel_m"],1.8)
            robot=inspect_urdf(project["urdf"])
            axis_x=next(j for j in robot.joints if j.name=="actuator_x")
            self.assertAlmostEqual(axis_x.upper,1.8)


@unittest.skipUnless(importlib.util.find_spec("pybullet"),"Bullet required")
class MachinePhysicsTests(unittest.TestCase):
    def test_user_gantry_is_simulated_in_actual_bullet_with_measured_motion(self):
        from dsiq_native.bullet_runtime import BulletPlayground, BulletSettings
        with TemporaryDirectory() as tmp:
            project=save_project(
                MachineDesign(asset_id="my_actual_assembly",x_travel_m=.6),
                Path(tmp)/"custom_gantry",
            )
            with BulletPlayground(BulletSettings(payload_count=0)) as world:
                loaded=world.import_urdf_robot(project["manifest"],
                                               base_position_m=(0.,1.2,0.))
                self.assertEqual(loaded["joint_count"],4)
                self.assertEqual(loaded["asset_class"],"test_fixture")
                world.command_imported_joint(
                    "my_actual_assembly","actuator_x",.25
                )
                world.command_imported_joint(
                    "my_actual_assembly","actuator_z",.12
                )
                result=world.step(480)
                imported=result["imported_robots"][0]["joint_states"]
                by_name={joint["name"]:joint["position"] for joint in imported}
                self.assertGreater(by_name["actuator_x"],.02)
                self.assertGreater(by_name["actuator_z"],.005)
                self.assertFalse(result["physically_qualified"])
                with self.assertRaises(ValueError):
                    world.command_imported_joint(
                        "my_actual_assembly","actuator_x",20.
                    )

if __name__=="__main__":
    unittest.main()
