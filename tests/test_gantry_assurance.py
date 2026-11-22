"""Gantry motion and frame assurance, independently verified against actual Bullet."""
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.gantry_assurance import expected_tool_position,verify_gantry_in_bullet
from dsiq_native.machine_designer import MachineDesign,save_project

class GantryMathTests(unittest.TestCase):
    def test_expected_xyz_machine_tool_frames(self):
        spec=MachineDesign(x_travel_m=1.2,y_travel_m=.8,z_travel_m=.45,
                           clearance_height_m=1.5)
        base=expected_tool_position(spec,(0.,0.,0.))
        self.assertAlmostEqual(base[0],-.6)
        self.assertAlmostEqual(base[1],-.4)
        self.assertAlmostEqual(base[2],1.5-.45/2-.17)
        target=expected_tool_position(spec,(1.2,.8,.45))
        self.assertAlmostEqual(target[0],.6)
        self.assertAlmostEqual(target[1],.4)
        self.assertAlmostEqual(target[2],1.5-.45-.45/2-.17)

    def test_reference_motion_limits_reject_unsafe_requests(self):
        spec=MachineDesign()
        with self.assertRaises(ValueError):
            expected_tool_position(spec,(100.,0.,0.))
        with self.assertRaises(ValueError):
            expected_tool_position(spec,(-.01,0.,0.))

@unittest.skipUnless(importlib.util.find_spec("pybullet"),"Actual Bullet runtime required")
class GantryBulletTests(unittest.TestCase):
    def test_executed_urdf_joint_frames_match_independent_equations(self):
        with TemporaryDirectory() as tmp:
            save_project(
                MachineDesign(asset_id="assurance_gantry",
                    x_travel_m=.80,y_travel_m=.65,z_travel_m=.32),
                Path(tmp)/"gantry",
            )
            result=verify_gantry_in_bullet(Path(tmp)/"gantry")
            self.assertEqual(len(result["samples"]),3)
            self.assertTrue(result["passed"],result)
            self.assertLessEqual(result["max_error_m"],.0001)
            self.assertFalse(result["physically_qualified"])

    def test_invalid_tolerance_is_rejected(self):
        with self.assertRaises(ValueError):
            verify_gantry_in_bullet("/does/not/matter",tolerance_m=1.0)

if __name__=="__main__":
    unittest.main()
