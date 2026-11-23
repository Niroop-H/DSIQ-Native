"""External reference comparison utility must never fabricate physical validation."""
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.physical_validation import compare_measured_robot_motion


class ExternalMotionComparisonTests(unittest.TestCase):
    def setUp(self):
        self.temp=TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        self.sim=self.root/"simulator.json"
        self.observed=self.root/"physical.json"
        self.sim.write_text(json.dumps({
            "tracking":{"sampled_reports":[
                {"time_s":.1,"measured_joint_states":{"actuator_x":.1}},
                {"time_s":.2,"measured_joint_states":{"actuator_x":.2}},
                {"time_s":.3,"measured_joint_states":{"actuator_x":.3}},
            ]}
        }))
        self.observed.write_text(json.dumps({
            "schema":"dsiq-external-joint-measurements-v1",
            "equipment_identifier":"TEST_ONLY_SYNTHETIC_NOT_REAL_HARDWARE",
            "instrument_identifier":"TEST_ONLY_SYNTHETIC",
            "instrument_calibration_reference":"NO_ACTUAL_CALIBRATION_UNIT_TEST",
            "observations":[
                {"joint":"actuator_x","time_s":.1,"measured_position":.1005,"unit":"m"},
                {"joint":"actuator_x","time_s":.2,"measured_position":.20,"unit":"m"},
                {"joint":"actuator_x","time_s":.3,"measured_position":.3001,"unit":"m"},
            ],
        }))
    def test_interpolated_comparison_pass_does_not_certify_industry(self):
        result=compare_measured_robot_motion(
            self.sim,self.observed,tolerance=.002
        )
        self.assertTrue(result["all_operator_tolerances_met"])
        self.assertFalse(result["industrial_physics_calibrated"])
        self.assertFalse(result["measurement_source_independently_authenticated"])
        self.assertEqual(result["comparison"]["actuator_x"]["samples"],3)
        self.assertLess(result["comparison"]["actuator_x"]["maximum_absolute_error"],.002)

    def test_reference_residual_exceeding_tolerance_fails(self):
        doc=json.loads(self.observed.read_text())
        doc["observations"][1]["measured_position"]=.5
        self.observed.write_text(json.dumps(doc))
        result=compare_measured_robot_motion(
            self.sim,self.observed,tolerance=.002
        )
        self.assertFalse(result["all_operator_tolerances_met"])

    def test_uncertainty_guard_uses_conservative_residual_margin(self):
        doc=json.loads(self.observed.read_text())
        doc["joint_units"]={"actuator_x":"m"}
        for row in doc["observations"]:
            row["standard_uncertainty"]=.0001
        self.observed.write_text(json.dumps(doc))
        report=compare_measured_robot_motion(
            self.sim,self.observed,tolerance=.002,require_uncertainty=True,
        )
        self.assertTrue(report["all_conservative_uncertainty_bounds_met"])
        self.assertTrue(report["external_joint_unit_contract_present"])
        self.assertTrue(report["standard_uncertainty_complete"])
        self.assertFalse(report["measurement_clock_alignment_independently_verified"])
        self.assertFalse(report["industrial_physics_calibrated"])
        doc["observations"][0]["standard_uncertainty"]=.002
        self.observed.write_text(json.dumps(doc))
        failed=compare_measured_robot_motion(
            self.sim,self.observed,tolerance=.002,require_uncertainty=True,
        )
        self.assertTrue(failed["all_operator_tolerances_met"])
        self.assertFalse(failed["all_conservative_uncertainty_bounds_met"])

    def test_partial_uncertainty_and_mixed_units_rejected(self):
        doc=json.loads(self.observed.read_text())
        doc["observations"][0]["standard_uncertainty"]=.0001
        self.observed.write_text(json.dumps(doc))
        with self.assertRaisesRegex(ValueError,"Every measurement"):
            compare_measured_robot_motion(self.sim,self.observed)
        doc["observations"][0].pop("standard_uncertainty")
        doc["observations"][1]["unit"]="rad"
        self.observed.write_text(json.dumps(doc))
        with self.assertRaisesRegex(ValueError,"Mixed units"):
            compare_measured_robot_motion(self.sim,self.observed)

    def test_nonfinite_simulated_feedback_cannot_be_accepted(self):
        import math
        simulated=json.loads(self.sim.read_text())
        simulated["tracking"]["sampled_reports"][1]["measured_joint_states"]["actuator_x"]=math.nan
        self.sim.write_text(json.dumps(simulated))
        with self.assertRaisesRegex(ValueError,"Nonfinite"):
            compare_measured_robot_motion(self.sim,self.observed)

    def test_faulted_simulation_is_ineligible_for_calibration(self):
        simulated=json.loads(self.sim.read_text())
        simulated["tracking"]["runtime_monitor_passed"]=False
        self.sim.write_text(json.dumps(simulated))
        with self.assertRaisesRegex(ValueError,"Faulted"):
            compare_measured_robot_motion(self.sim,self.observed)

    def test_declared_unit_contract_must_match_each_measurement(self):
        doc=json.loads(self.observed.read_text())
        doc["joint_units"]={"actuator_x":"rad"}
        self.observed.write_text(json.dumps(doc))
        with self.assertRaisesRegex(ValueError,"source unit contract"):
            compare_measured_robot_motion(self.sim,self.observed)

    def test_strict_uncertainty_requires_original_instrument_data(self):
        with self.assertRaisesRegex(ValueError,"standard_uncertainty"):
            compare_measured_robot_motion(
                self.sim,self.observed,require_uncertainty=True,
            )

    def test_strict_urdf_source_hash_and_units_bind_measurement(self):
        sim=json.loads(self.sim.read_text())
        sim["tracking"]["source_urdf_sha256"]="c"*64
        sim["tracking"]["joint_units_from_loaded_urdf"]={"actuator_x":"m"}
        self.sim.write_text(json.dumps(sim))
        measured=json.loads(self.observed.read_text())
        measured["source_urdf_sha256"]="c"*64
        self.observed.write_text(json.dumps(measured))
        report=compare_measured_robot_motion(
            self.sim,self.observed,require_source_binding=True
        )
        self.assertTrue(report["simulation_source_sha256_matches_external_metadata"])
        self.assertTrue(report["simulation_urdf_joint_units_present"])
        measured["observations"][0]["unit"]="rad"
        self.observed.write_text(json.dumps(measured))
        with self.assertRaisesRegex(ValueError,"loaded source URDF"):
            compare_measured_robot_motion(self.sim,self.observed)
        measured["observations"][0]["unit"]="m"
        measured["source_urdf_sha256"]="d"*64
        self.observed.write_text(json.dumps(measured))
        with self.assertRaisesRegex(ValueError,"does not match"):
            compare_measured_robot_motion(self.sim,self.observed)

    def test_strict_source_identity_requires_source_metadata(self):
        with self.assertRaisesRegex(ValueError,"source URDF SHA256"):
            compare_measured_robot_motion(
                self.sim,self.observed,require_source_binding=True
            )

    def test_bad_external_records_rejected(self):
        doc=json.loads(self.observed.read_text())
        doc["observations"][0]["joint"]="not_same_mechanism"
        self.observed.write_text(json.dumps(doc))
        with self.assertRaisesRegex(ValueError,"joint name"):
            compare_measured_robot_motion(self.sim,self.observed)
        with self.assertRaises(ValueError):
            compare_measured_robot_motion(self.sim,self.observed,tolerance=-1)

if __name__=="__main__":
    unittest.main()
