"""A real Bullet implementation of the pluggable physics world contract.

These tests forbid silently substituting Bullet for a requested unavailable
solver, and keep industrial-qualification claims off all reference results.
"""
from __future__ import annotations

import importlib.util
import unittest

from dsiq_native.physics_backend import (
    PhysicsBackendWorld, available_backend_capabilities, open_backend,
    run_backend_conformance,
)


class PhysicsBackendSpecificationTests(unittest.TestCase):
    def test_advertises_only_implemented_solver(self):
        caps=available_backend_capabilities()
        ids=[x["backend_id"] for x in caps]
        self.assertIn("bullet_cpu_direct_v1",ids)
        self.assertTrue(set(ids)<={"bullet_cpu_direct_v1","mujoco_cpu_direct_v1"})
        self.assertEqual(len(ids),len(set(ids)))
        self.assertFalse(caps[0]["supports_usd_physics_import"])
        self.assertFalse(caps[0]["supports_gpu_batched_worlds"])
        self.assertFalse(caps[0]["supports_hardware_actuation"])
        self.assertFalse(caps[0]["independently_qualified"])

    def test_does_not_fake_or_fallback_missing_solver(self):
        for unavailable in ("mujoco", "newton", "physx5", "", "panda3d"):
            with self.subTest(unavailable=unavailable):
                with self.assertRaisesRegex(ValueError,"unavailable"):
                    open_backend(unavailable)
                with self.assertRaisesRegex(ValueError,"unavailable"):
                    run_backend_conformance(unavailable)


@unittest.skipUnless(importlib.util.find_spec("pybullet") is not None,
                     "Install real CPU Bullet to execute solver conformance")
class RealBulletBackendTests(unittest.TestCase):
    def test_backend_implements_contract_and_refuses_use_after_close(self):
        backend=open_backend()
        self.assertIsInstance(backend,PhysicsBackendWorld)
        self.assertTrue(backend.capabilities.supports_measured_joint_feedback)
        self.assertEqual(backend.step(1)["step_index"],1)
        with self.assertRaisesRegex(ValueError,"not loaded"):
            backend.read_joint_states("not_loaded")
        with self.assertRaisesRegex(ValueError,"not loaded"):
            backend.read_contacts("not_loaded")
        backend.close()
        backend.close()
        with self.assertRaisesRegex(RuntimeError,"closed"):
            backend.step(1)
        with self.assertRaisesRegex(RuntimeError,"closed"):
            backend.import_verified_urdf("missing.json")

    def test_source_bound_two_world_actual_physics_conformance(self):
        report=run_backend_conformance()
        self.assertEqual(report["test_case_count"],2)
        self.assertTrue(report["passed"],report)
        self.assertLessEqual(report["repeatability_position_delta_m"],1e-6)
        self.assertTrue(all(t["disallowed_command_rejections"]==2
                            for t in report["tests"]))
        self.assertTrue(all(t["measured_x_position_m"]>.005
                            for t in report["tests"]))
        self.assertFalse(report["usd_physics_import_implemented"])
        self.assertFalse(report["other_physics_backends_tested"])
        self.assertFalse(report["industrial_safety_or_oem_calibration_qualified"])


if __name__=="__main__":
    unittest.main()
