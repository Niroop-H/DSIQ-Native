"""End-to-end core readiness must be auditable without claiming certification."""
import importlib.util
import unittest

BOTH=importlib.util.find_spec("pybullet") is not None and importlib.util.find_spec("pxr") is not None


@unittest.skipUnless(BOTH,"Actual Bullet and Pixar USD bindings required")
class ThreeCoreAcceptanceTests(unittest.TestCase):
    def test_every_defined_reference_core_gate_executes(self):
        from dsiq_native.core_readiness import inspect_core_readiness
        report=inspect_core_readiness()
        self.assertTrue(report["all_defined_software_gates_passed"],report)
        self.assertTrue(all(report["software_gates"].values()))
        self.assertEqual(report["physics"]["reference_case_count"],10)
        self.assertEqual(report["robotics"]["joint_count"],4)
        self.assertGreater(report["robotics"]["screened_samples"],10)
        self.assertEqual(report["openusd"]["time_sampled_joint_tracks"],2)
        self.assertEqual(len(report["openusd"]["keyframes"]),3)
        self.assertFalse(report["industrial_release_ready"])
        self.assertFalse(report["robotics"]["hardware_qualified"])
        self.assertGreater(len(report["remaining_blockers"]),5)


if __name__=="__main__":
    unittest.main()
