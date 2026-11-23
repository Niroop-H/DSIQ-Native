"""An imported machine inventory may only report what the real URDF declares."""
from pathlib import Path
from tempfile import TemporaryDirectory
import json
import unittest
from dsiq_native.machine_designer import MachineDesign,save_project
from dsiq_native.machine_inventory import machine_inventory


class URDFMachineInventoryTests(unittest.TestCase):
    def test_actual_user_urdf_parts_and_inertia_inventory(self):
        with TemporaryDirectory() as tmp:
            project=save_project(
                MachineDesign(asset_id="independent_inventory_xyz"),
                Path(tmp)/"my_machine"
            )
            result=machine_inventory(project["manifest"])
            self.assertEqual(result["asset_id"],"independent_inventory_xyz")
            self.assertEqual(result["asset_class"],"test_fixture")
            self.assertEqual(result["link_count"],5)
            self.assertEqual(result["joint_count"],4)
            self.assertEqual(result["actuated_joint_count"],3)
            self.assertAlmostEqual(result["declared_total_mass_kg"],32.8)
            self.assertGreater(result["visual_subcomponents"],8)
            self.assertEqual(result["mesh_references"],0)
            self.assertTrue(result["rights_and_source_integrity_verified"])
            self.assertFalse(result["real_machine_dynamics_qualified"])
            self.assertFalse(result["real_oem_dimensions_qualified"])
            self.assertEqual(sum(j["joint_type"]=="prismatic"
                                 for j in result["joints"]),3)
            self.assertTrue(all(
                j["declared_effort"]>0 and j["declared_max_speed"]>0
                for j in result["joints"]
                if j["joint_type"]!="fixed"
            ))
            self.assertTrue(all(x["declared_inertia"] is not None
                                for x in result["links"]))

    def test_checksum_mismatch_rejects_inventory(self):
        with TemporaryDirectory() as tmp:
            project=save_project(
                MachineDesign(asset_id="tampered_inventory_xyz"),
                Path(tmp)/"test_machine"
            )
            with Path(project["urdf"]).open("ab") as f:
                f.write(b"\n<!-- unauthorized modification -->")
            with self.assertRaisesRegex(ValueError,"checksum"):
                machine_inventory(project["manifest"])


if __name__=="__main__":
    unittest.main()
