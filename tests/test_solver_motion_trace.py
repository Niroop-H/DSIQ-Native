"""Real two-solver time series and defensive CLI input tests."""
from __future__ import annotations
import importlib.util
import unittest


class MotionTraceValidationTests(unittest.TestCase):
    def test_reject_unjustified_or_unsafe_test_parameters(self):
        from dsiq_native.solver_motion_trace import compare_motion_traces
        for request in (
            {"joint":"fabricated_axis"},
            {"target":float("nan")},
            {"steps":0},
            {"steps":True},
            {"every":0},
            {"tolerance_m":-.01},
        ):
            with self.subTest(request=request):
                with self.assertRaises(ValueError):
                    compare_motion_traces(**request)


@unittest.skipUnless(
    importlib.util.find_spec("mujoco") is not None and
    importlib.util.find_spec("pybullet") is not None,
    "True PyBullet and MuJoCo required",
)
class TrueMotionTraceAcceptance(unittest.TestCase):
    def test_source_bound_solver_time_series_records_real_feedback(self):
        from dsiq_native.solver_motion_trace import compare_motion_traces
        result=compare_motion_traces(steps=100,every=5)
        self.assertEqual(result["sample_every_steps"],5)
        self.assertEqual(result["steps"],100)
        self.assertEqual(result["joint"],"actuator_x")
        self.assertEqual(result["fixture"],"self_authored_xyz_gantry_not_oem")
        self.assertEqual(len(result["backend_measurements"]),2)
        self.assertGreater(result["trajectory_comparison"]["position_rmse_m"],0.)
        self.assertTrue(
            result["trajectory_comparison"]["within_final_tolerance"],result
        )
        self.assertLessEqual(
            result["trajectory_comparison"]["position_peak_difference_m"],.010,
            result["trajectory_comparison"]
        )
        self.assertTrue(
            result["all_source_speed_and_effort_envelopes_satisfied"],result
        )

        self.assertFalse(result["comparable_motor_control_laws_calibrated"])
        self.assertFalse(result["safety_certified"])
        self.assertTrue(all(
            len(solver["samples"])==20
            for solver in result["backend_measurements"].values()
        ))
        self.assertTrue(all(
            solver["source_lower_m"]==0. and solver["source_upper_m"]==1.2 and
            not solver["source_travel_bounds_violated"]
            for solver in result["backend_measurements"].values()
        ))
        for backend in result["backend_measurements"].values():
            self.assertTrue(all(
                a["step"]<b["step"] and
                a["simulated_time_s"]<b["simulated_time_s"]
                for a,b in zip(backend["samples"],backend["samples"][1:])
            ))
            self.assertLess(backend["max_position_above_target_m"],.20)

    def test_reject_source_specific_z_travel_overrange(self):
        from dsiq_native.solver_motion_trace import compare_motion_traces
        with self.assertRaisesRegex(ValueError,"verified source URDF"):
            compare_motion_traces(joint="actuator_z",target=.6)

if __name__=="__main__":
    unittest.main()
