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
