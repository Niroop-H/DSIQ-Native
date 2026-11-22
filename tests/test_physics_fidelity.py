"""Isolated numerical physics acceptance tests on actual Bullet CPU builds."""
import importlib.util
import math
import unittest
from dsiq_native.physics_fidelity import (
    rigid_contact_refinement,
    sliding_friction_response,
    imported_motor_limits,
    run_extended_physics_suite,
)

HAS_BULLET = importlib.util.find_spec("pybullet") is not None


@unittest.skipUnless(HAS_BULLET,"Install optional PyBullet physics backend")
class PhysicsFidelityTests(unittest.TestCase):
    def test_contact_at_three_timesteps(self):
        case=rigid_contact_refinement()
        self.assertTrue(case.passed,case)
        samples=case.observed["samples"]
        self.assertEqual([x["hz"] for x in samples],[120,240,480])
        self.assertTrue(all(x["contacts"]>=1 for x in samples))
        self.assertTrue(all(math.isfinite(x["reported_normal_force_N"]) for x in samples))

    def test_material_friction_changes_actual_sliding_distance(self):
        case=sliding_friction_response()
        self.assertTrue(case.passed,case)
        self.assertGreater(case.observed["travel_reduction_m"],.08)
        self.assertGreater(case.observed["speed_reduction_m_s"],.05)

    def test_imported_joint_limits_actually_checked_and_motor_responds(self):
        case=imported_motor_limits()
        self.assertTrue(case.passed,case)
        self.assertTrue(case.observed["out_of_range_command_rejected"])
        self.assertLess(
            case.observed["position_m_after_60_steps"],
            case.tolerances["max_position_m_after_60_steps"]+.000001
        )

    def test_machine_readable_extended_suite(self):
        report=run_extended_physics_suite()
        self.assertEqual(report["case_count"],3)
        self.assertTrue(report["passed"],report)
        self.assertFalse(report["independent_hardware_validation"])
        self.assertTrue(all(
            set(("observed","reference","tolerances","passed")) <= set(item)
            for item in report["cases"]
        ))


if __name__=="__main__":
    unittest.main()
