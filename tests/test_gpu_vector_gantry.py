"""Numerical and safety contract for truly compiled Warp kernels.

This suite uses explicit Warp CPU only to check kernel semantics on
ordinary CI workers. CUDA execution is a separate, fail-closed real-device
acceptance gate; passing this suite alone is not GPU qualification.
"""
from __future__ import annotations

import importlib.util
import math
import unittest

from dsiq_native.gpu_vector_gantry import (
    _initial_conditions, _reference_step, benchmark_warp_reduced_order,
    inspect_warp_compute,load_verified_reference_axis,validate_batch_options,
)


class GPUComputeContractTests(unittest.TestCase):
    def test_real_source_axis_metadata_is_verified(self):
        source=load_verified_reference_axis()
        self.assertEqual(source["axis_name"],"actuator_x")
        self.assertEqual(source["source_asset_class"],"test_fixture")
        self.assertEqual(source["lower_m"],0.)
        self.assertEqual(source["upper_m"],1.2)
        self.assertEqual(source["max_speed_m_s"],.8)
        self.assertEqual(source["max_effort_n"],250.)
        self.assertAlmostEqual(source["effective_moving_mass_kg"],12.8)
        self.assertEqual(len(source["source_urdf_sha256"]),64)
        self.assertFalse(source["full_multibody_dynamics"])

    def test_bounded_parameters_reject_unsafe_or_excessive_work(self):
        good=dict(envs=128,steps=24,seed=1007,memory_limit_mib=64)
        validate_batch_options(**good)
        for change in (
            {"envs":0},{"envs":True},{"envs":262145},
            {"steps":0},{"steps":True},{"steps":2401},
            {"seed":-1},{"seed":2**32},{"seed":True},
            {"memory_limit_mib":2},{"memory_limit_mib":True},
            {"envs":262144,"steps":2400},
        ):
            with self.subTest(change=change):
                with self.assertRaises(ValueError):
                    validate_batch_options(**{**good,**change})
        for device in ("", "gpu", "CUDA", "cuda", "cuda:bad","cuda:-1",
                       "cuda://0", "cuda:00000000"):
            with self.subTest(device=device):
                with self.assertRaises(ValueError):
                    benchmark_warp_reduced_order(
                        envs=8,steps=3,device=device
                    )

    def test_independent_reference_model_preserves_limits(self):
        model=load_verified_reference_axis()
        q=.0; v=-.8
        for _ in range(2400):
            q,v=_reference_step(q,v,-10.,model,1/240)
            self.assertGreaterEqual(q,model["lower_m"])
            self.assertLessEqual(q,model["upper_m"])
            self.assertLessEqual(abs(v),model["max_speed_m_s"]+1e-12)
        self.assertTrue(math.isfinite(q))

    def test_initial_state_seed_is_reproducible(self):
        source=load_verified_reference_axis()
        a=_initial_conditions(100,source,1007)
        b=_initial_conditions(100,source,1007)
        c=_initial_conditions(100,source,1008)
        self.assertEqual(a,b)
        self.assertNotEqual(a,c)
        self.assertTrue(all(source["lower_m"]<=x<=source["upper_m"]
                            for seq in a for x in seq))

    def test_gpu_capability_reporting_does_not_claim_certification(self):
        report=inspect_warp_compute()
        self.assertFalse(report["gpu_compute_qualified"])
        self.assertIsInstance(report["cuda_available"],bool)


@unittest.skipUnless(importlib.util.find_spec("warp") is not None
                     and importlib.util.find_spec("numpy") is not None,
                     "Install optional Warp for real compiled CPU kernel tests")
class RealWarpKernelAcceptance(unittest.TestCase):
    def test_actual_warp_kernel_on_explicit_cpu_matches_scalar(self):
        r=benchmark_warp_reduced_order(
            envs=256,steps=48,seed=1007,
            device="cpu",oracle_samples=16,
        )
        self.assertEqual(r["total_actuator_updates"],256*48)
        self.assertTrue(r["independent_oracle_passed"],r)
        self.assertLessEqual(r["position_residual_max_m"],5e-4)
        self.assertLessEqual(r["velocity_residual_max_m_s"],5e-4)
        self.assertTrue(r["source_bounds_checked_for_all_environments"])
        self.assertFalse(r["execution_device_is_cuda"])
        self.assertFalse(r["actual_gpu_kernel_executed"])
        self.assertTrue(r["cpu_mode_is_explicit_semantic_oracle"])
        self.assertTrue(r["uses_real_warp_compiled_kernel"])
        self.assertFalse(r["hardware_qualified_on_requested_gpu"])
        self.assertFalse(r["has_full_multibody_or_contacts"])
        self.assertGreater(r["synchronized_kernel_wall_seconds"],0.)
        self.assertEqual(len(r["independent_scalar_samples"]),16)

    def test_repeatability_two_independent_kernel_runs(self):
        a=benchmark_warp_reduced_order(
            envs=32,steps=16,seed=42,device="cpu",
        )
        b=benchmark_warp_reduced_order(
            envs=32,steps=16,seed=42,device="cpu",
        )
        self.assertEqual(a["independent_scalar_samples"],
                         b["independent_scalar_samples"])
        self.assertTrue(a["independent_oracle_passed"])

    def test_explicit_cuda_request_fails_on_non_cuda_runners(self):
        from warp import is_cuda_available
        if is_cuda_available():
            self.skipTest("A real NVIDIA CUDA GPU exists: run GPU hardware acceptance")
        with self.assertRaisesRegex(RuntimeError,"CUDA GPU unavailable"):
            benchmark_warp_reduced_order(
                envs=16,steps=4,device="cuda:0",
            )


if __name__=="__main__":
    unittest.main()
