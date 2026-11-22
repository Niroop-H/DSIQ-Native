"""Quantitative, isolated physics fixtures; must run in actual Bullet CI."""
import importlib.util
import unittest

from dsiq_native.physics_benchmarks import (
    free_fall, ground_contact, torque_and_inertia, repeatability, timestep_convergence, run_physics_suite,
)

@unittest.skipUnless(importlib.util.find_spec("pybullet"),"PyBullet optional dependency required")
class MeasuredPhysicsTests(unittest.TestCase):
    def test_free_fall_matches_analytic_reference_with_declared_tolerance(self):
        result=free_fall()
        self.assertTrue(result.passed, result)
        self.assertLess(result.observed["position_absolute_error_m"], .02)

    def test_contact_settles_with_observed_collision(self):
        result=ground_contact()
        self.assertTrue(result.passed, result)
        self.assertGreater(result.observed["contact_points"],0)

    def test_rotational_inertia_matches_analytic_reference(self):
        result=torque_and_inertia()
        self.assertTrue(result.passed, result)

    def test_repeated_clients_do_not_leak_state(self):
        result=repeatability()
        self.assertTrue(result.passed, result)

    def test_gravity_error_decreases_with_fixed_step_refinement(self):
        result=timestep_convergence()
        self.assertTrue(result.passed,result)
        rates=result.observed["observations"]
        self.assertEqual([x["hz"] for x in rates],[120,240,480])
        self.assertGreater(rates[0]["absolute_error_m"],
                           rates[1]["absolute_error_m"])
        self.assertGreater(rates[1]["absolute_error_m"],
                           rates[2]["absolute_error_m"])

    def test_suite_reports_actual_measured_failures_not_certification(self):
        data=run_physics_suite()
        self.assertEqual(data["case_count"],5)
        self.assertTrue(data["passed"],data)
        self.assertTrue(all("observed" in item for item in data["cases"]))
        self.assertIn("no OEM",data["claims"])

    def test_bounded_benchmark_length(self):
        with self.assertRaises(ValueError):
            free_fall(duration_steps=1000000)

if __name__=="__main__":
    unittest.main()
