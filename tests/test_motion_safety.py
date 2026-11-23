"""Motion limits and simulated stop-latch are fail closed and NEVER safety rated."""
from __future__ import annotations
from dataclasses import replace
import math
import unittest

from dsiq_native.imported_trajectory import ImportedTrajectory
from dsiq_native.motion_safety import (
    audit_joint_motion, SimulationStopLatch, ACC_FACTOR, JERK_FACTOR,
)


class AnalyticImportedMotionTests(unittest.TestCase):
    def setUp(self):
        T=2.
        u=[i/100 for i in range(101)]
        states=tuple((.3*(10*x**3-15*x**4+6*x**5),)
                     for x in u)
        self.plan=ImportedTrajectory(
            "named_source_robot",("linear_actuator",),(1,),
            (0.,),(.3,),T,T/100,states,(1.,),
        )
        self.source={
            "schema":"dsiq-document-controlled-joint-motion-limits-v1",
            "asset_id":"named_source_robot",
            "joints":{"linear_actuator":{
                "joint_type":"prismatic",
                "max_speed":.4,
                "max_acceleration":.6,
                "max_jerk":3.,
            }},
        }

    def test_exact_quintic_derivative_extrema(self):
        out=audit_joint_motion(self.plan,self.source)
        self.assertTrue(out["all_documented_limits_passed"])
        self.assertTrue(out["quintic_analytical_maxima_used"])
        result=out["per_joint"]["linear_actuator"]["analytical_maxima"]
        self.assertAlmostEqual(result["speed"],.3*1.875/2.)
        self.assertAlmostEqual(result["acceleration"],.3*ACC_FACTOR/4.)
        self.assertAlmostEqual(result["jerk"],.3*JERK_FACTOR/8.)
        self.assertFalse(out["industrial_safety_certified"])
        self.assertFalse(out["torque_and_force_qualified"])
        self.assertFalse(out["physical_estop_validated"])

    def test_overspeed_or_jerk_failure_does_not_get_pass(self):
        self.source["joints"]["linear_actuator"]["max_jerk"]=1.
        out=audit_joint_motion(self.plan,self.source)
        self.assertFalse(out["all_documented_limits_passed"])
        self.assertFalse(out["per_joint"]["linear_actuator"]["passes_all_documented_limits"])

    def test_limit_document_cannot_be_reused_for_a_changed_urdf(self):
        with self.assertRaisesRegex(ValueError,"verified URDF SHA256"):
            audit_joint_motion(
                self.plan,self.source,
                expected_source_sha256="a"*64
            )
        self.source["source_urdf_sha256"]="a"*64
        result=audit_joint_motion(
            self.plan,self.source,expected_source_sha256="a"*64
        )
        self.assertTrue(result["source_urdf_sha256_bound_and_verified"])

    def test_missing_axis_or_bogus_urdf_speed_rejected(self):
        self.source["joints"]={}
        with self.assertRaisesRegex(ValueError,"every imported actuator"):
            audit_joint_motion(self.plan,self.source)
        self.source["joints"]["linear_actuator"]={
            "max_speed":999.,"max_acceleration":2.,"max_jerk":10.,
        }
        with self.assertRaisesRegex(ValueError,"source URDF"):
            audit_joint_motion(self.plan,self.source)

    def test_corrupted_plan_and_nan_limits_rejected(self):
        corrupt=replace(self.plan,samples=((0.,),(math.inf,)))
        with self.assertRaisesRegex(ValueError,"invalid"):
            audit_joint_motion(corrupt,self.source)
        self.source["joints"]["linear_actuator"]["max_acceleration"]=math.nan
        with self.assertRaisesRegex(ValueError,"positive finite"):
            audit_joint_motion(self.plan,self.source)


class SimulatedLatchTests(unittest.TestCase):
    def test_arming_requires_both_proofs_and_consumes_single_move(self):
        state=SimulationStopLatch()
        self.assertEqual(state.inspect()["software_state"],"STOPPED")
        self.assertFalse(state.inspect()["hardware_stop_function"])
        with self.assertRaises(RuntimeError):
            state.consume_single_move()
        state.arm(documented_limits_passed=True,collision_screen_passed=True)
        self.assertTrue(state.inspect()["motion_permitted_in_simulation"])
        self.assertTrue(state.consume_single_move())
        with self.assertRaises(RuntimeError):
            state.consume_single_move()

    def test_fault_is_latched_until_explicit_reset(self):
        state=SimulationStopLatch()
        with self.assertRaisesRegex(RuntimeError,"prerequisite"):
            state.arm(documented_limits_passed=True,collision_screen_passed=False)
        self.assertEqual(state.inspect()["software_state"],"FAULT")
        with self.assertRaisesRegex(RuntimeError,"reset"):
            state.arm(documented_limits_passed=True,collision_screen_passed=True)
        state.reset()
        state.arm(documented_limits_passed=True,collision_screen_passed=True)
        state.trip("Operator commanded simulated stop")
        self.assertFalse(state.inspect()["motion_permitted_in_simulation"])


if __name__=="__main__":
    unittest.main()
