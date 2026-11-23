"""True compiled Warp XYZ kernel numerical reference and explicit CUDA gate."""
from __future__ import annotations

import importlib.util
import unittest

from dsiq_native.gpu_vector_xyz import (
    benchmark_warp_xyz, source_verified_xyz_parameters,
)


class XYZWarpSourceContract(unittest.TestCase):
    def test_all_axes_and_subtree_mass_from_real_rights_gated_urdf(self):
        p=source_verified_xyz_parameters()
        self.assertEqual(p["source_asset_class"],"test_fixture")
        self.assertEqual(len(p["source_urdf_sha256"]),64)
        self.assertEqual([x["name"] for x in p["axes"]],
                         ["actuator_x","actuator_y","actuator_z"])
        self.assertEqual([x["upper_m"] for x in p["axes"]],
                         [1.2,.8,.45])
        self.assertEqual([x["max_speed_m_s"] for x in p["axes"]],
                         [.8,.65,.4])
        self.assertEqual([x["max_effort_n"] for x in p["axes"]],
                         [250.,160.,220.])
        for observed,target in zip(
            [x["effective_moving_mass_kg"] for x in p["axes"]],
            [12.8,6.8,2.8],
        ):
            self.assertAlmostEqual(observed,target)
        self.assertTrue(p["moving_subtree_mass_not_multibody_mass_matrix"])

    def test_reject_oversized_gpu_batches_and_invalid_device(self):
        for kwargs in (
            {"envs":0},{"envs":65537},{"envs":True},
            {"steps":0},{"steps":2401},{"seed":-1},{"steps":True},
            {"device":"gpu"},{"device":"cuda"},{"device":"cuda:-1"},
            {"memory_limit_mib":0},{"memory_limit_mib":False},
            {"envs":65536,"steps":2400},
        ):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(ValueError):
                    benchmark_warp_xyz(**{**{"envs":8,"steps":4,"device":"cpu"},**kwargs})


@unittest.skipUnless(
    importlib.util.find_spec("warp") is not None and
    importlib.util.find_spec("numpy") is not None,
    "Install actual NVIDIA Warp to compile 3-axis reference kernel",
)
class XYZWarpKernelConformance(unittest.TestCase):
    def test_compiled_3axis_kernel_on_cpu_oracle_matches_independent_scalar(self):
        r=benchmark_warp_xyz(envs=128,steps=24,device="cpu",
                             oracle_samples=8,seed=1007)
        self.assertTrue(r["independent_oracle_passed"],r)
        self.assertEqual(r["total_axis_updates"],128*3*24)
        self.assertEqual(r["axes_per_env"],3)
        self.assertEqual(r["execution_device"],"cpu")
        self.assertFalse(r["actual_cuda_kernel_executed"])
        self.assertFalse(r["cuda_memory_admission"]["cuda_admission_checked"])
        self.assertTrue(r["explicit_cpu_reference_mode"])
        self.assertTrue(r["warp_compiled_kernel_executed"])
        self.assertEqual(len(r["independent_oracle_samples"]),8)
        self.assertTrue(r["source_joint_travel_and_speed_checked_all_envs"])
        self.assertTrue(r["not_a_general_multibody_physics_engine"])
        self.assertTrue(r["not_qualified_for_hardware_robot_control"])
        self.assertLessEqual(r["independent_reference_max_position_error_m"],5e-4)
        self.assertLessEqual(r["independent_reference_max_velocity_error_m_s"],5e-4)
        self.assertGreater(r["synchronized_kernel_wall_s"],0)

    def test_repeated_source_states_are_deterministic_on_same_device(self):
        a=benchmark_warp_xyz(envs=32,steps=12,device="cpu",
                             oracle_samples=8,seed=42)
        b=benchmark_warp_xyz(envs=32,steps=12,device="cpu",
                             oracle_samples=8,seed=42)
        self.assertEqual(a["independent_oracle_samples"],
                         b["independent_oracle_samples"])

    def test_unavailable_gpu_is_not_silently_replaced_by_cpu(self):
        import warp
        if warp.is_cuda_available():
            self.skipTest("Actual GPU present; CUDA acceptance should execute")
        with self.assertRaisesRegex(RuntimeError,"CUDA GPU unavailable"):
            benchmark_warp_xyz(envs=8,steps=4,device="cuda:0")


if __name__=="__main__":
    unittest.main()
