"""Conservation results must be measured, finite and timestep-sensitive."""
import importlib.util
import unittest

from dsiq_native.physics_conservation import (
    gravitational_energy_drift,force_free_linear_momentum,
    run_conservation_suite,
)

@unittest.skipUnless(importlib.util.find_spec("pybullet"),
                     "Actual Bullet backend needed")
class PhysicsConservationTests(unittest.TestCase):
    def test_gravitational_energy_drift_halves_on_step_refinement(self):
        case=gravitational_energy_drift()
        self.assertTrue(case.passed,case)
        samples=case.observed["samples"]
        self.assertEqual([x["frequency_hz"] for x in samples],[120,240,480])
        self.assertGreater(samples[0]["abs_error_J"],samples[1]["abs_error_J"])
        self.assertGreater(samples[1]["abs_error_J"],samples[2]["abs_error_J"])
        self.assertLess(samples[2]["abs_error_J"],.25)
        self.assertFalse(case.reference.get("contact_qualified",False))

    def test_linear_momentum_without_applied_force(self):
        case=force_free_linear_momentum()
        self.assertTrue(case.passed,case)
        self.assertLess(
            case.observed["absolute_momentum_error_kg_m_s"],1e-5,
        )

    def test_suite_is_quantified_not_certified(self):
        report=run_conservation_suite()
        self.assertEqual(report["case_count"],2)
        self.assertTrue(report["passed"],report)
        self.assertFalse(report["independent_physical_qualification"])
        self.assertTrue(all("tolerances" in r for r in report["cases"]))


if __name__=="__main__":
    unittest.main()
