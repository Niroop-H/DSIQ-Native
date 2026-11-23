"""Independent real MuJoCo CPU physics checks and matched solver fixture.

Only the opt-in MuJoCo CI runner installs mujoco. Other runners skip these
tests; the dedicated job must pass rather than treating the skip as green.
"""
from __future__ import annotations
import importlib.util
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

HAS_MUJOCO=importlib.util.find_spec("mujoco") is not None
HAS_BULLET=importlib.util.find_spec("pybullet") is not None


@unittest.skipUnless(HAS_MUJOCO and HAS_BULLET,
                     "Install actual MuJoCo and Bullet for two-solver acceptance")
class MuJoCoBackendAcceptance(unittest.TestCase):
    def test_source_verified_mujoco_actuator_and_contacts(self):
        from dsiq_native.physics_backend import open_backend, PhysicsBackendWorld
        from dsiq_native.machine_designer import MachineDesign, save_project
        with TemporaryDirectory() as root:
            data=save_project(
                MachineDesign(asset_id="mujoco_direct_trial"),
                Path(root)/"gantry"
            )
            with open_backend("mujoco_cpu_direct_v1") as backend:
                self.assertIsInstance(backend,PhysicsBackendWorld)
                self.assertEqual(backend.capabilities.backend_id,
                                 "mujoco_cpu_direct_v1")
                self.assertFalse(backend.capabilities.supports_gpu_batched_worlds)
                record=backend.import_verified_urdf(data["manifest"])
                asset=record["asset_id"]
                before=backend.read_joint_states(asset)
                names=[x["name"] for x in before["joints"]]
                self.assertIn("actuator_x",names)
                with self.assertRaises(ValueError):
                    backend.command_joint(asset,"unknown",.1)
                with self.assertRaises(ValueError):
                    backend.command_joint(asset,"actuator_x",999.)
                backend.command_joint(asset,"actuator_x",.20)
                clock=backend.step(100)
                self.assertEqual(clock["step_index"],100)
                states=backend.read_joint_states(asset)
                x=next(j for j in states["joints"] if j["name"]=="actuator_x")
                self.assertGreater(x["position"],.005)
                self.assertLess(x["position"],.30)
                self.assertEqual(x["unit"],"m")
                self.assertEqual(states["source_urdf_sha256"],
                                 record["source_urdf_sha256"])
                self.assertEqual(backend.read_contacts(asset)["step_index"],100)
                self.assertFalse(states["physically_calibrated"])
            with self.assertRaises(RuntimeError):
                backend.step(1)

    def test_matched_reference_mujoco_bullet_separate_real_solvers(self):
        from dsiq_native.physics_backend import run_cross_backend_comparison
        report=run_cross_backend_comparison()
        self.assertEqual(len(report["individual_backend_conformance"]),2)
        self.assertTrue(all(v["passed"] for v in report["individual_backend_conformance"]))
        self.assertTrue(report["passed"],report)
        oracle=report["analytic_unconstrained_freefall_comparison"]
        self.assertTrue(oracle["passed"],oracle)
        self.assertLessEqual(oracle["bullet_mujoco_position_residual_m"],.001)
        self.assertLessEqual(oracle["bullet_mujoco_velocity_residual_m_s"],.01)
        self.assertFalse(oracle["oem_or_robot_safety_qualified"])
        self.assertIn("controller_response_agrees_within_10mm",report)

        self.assertFalse(report["same_oem_calibration_or_solver_equivalence_proven"])
        self.assertFalse(report["hardware_safety_qualified"])

if __name__=="__main__":
    unittest.main()
