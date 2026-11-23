"""End-to-end launch audit refuses false clearance; motion gate uses Bullet."""
from __future__ import annotations

from contextlib import redirect_stdout,redirect_stderr
from io import StringIO
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.cli import main
from dsiq_native.launch_evidence import SCHEMA


def invoke(*items):
    out=StringIO()
    err=StringIO()
    with redirect_stdout(out),redirect_stderr(err):
        status=main([str(x) for x in items])
    return status,json.loads((out.getvalue() or err.getvalue()).strip())


class LaunchEvidenceCLIContracts(unittest.TestCase):
    def test_no_evidence_never_qualifies_for_industrial_release(self):
        with TemporaryDirectory() as tmp:
            source=Path(tmp)/"empty-inventory.json"
            source.write_text(json.dumps({"schema":SCHEMA,"evidence":{}}))
            status,report=invoke("launch-evidence-audit",source)
            self.assertEqual(status,3)
            self.assertEqual(report["missing_or_failed_categories"],8)
            self.assertFalse(report["production_or_industrial_launch_authorized"])


@unittest.skipUnless(importlib.util.find_spec("pybullet"),
                     "Actual source robot Bullet runtime needed")
class SourceJointMotionGateCLI(unittest.TestCase):
    def test_reviewed_kinematic_limits_must_be_complete_and_checked(self):
        from dsiq_native.machine_designer import MachineDesign,save_project
        from dsiq_native.bullet_runtime import BulletPlayground,BulletSettings
        with TemporaryDirectory() as tmp:
            root=Path(tmp)
            source=save_project(
                MachineDesign(asset_id="test_limit_gantry"),
                root/"gantry"
            )
            command=["robot-urdf-motion",source["manifest"],
                     "--target","actuator_x=0.03",
                     "--duration","2",
                     "--require-reviewed-limits"]
            status,report=invoke(*command)
            self.assertEqual(status,2)
            self.assertIn("--motion-limit-file",report["error"])
            with BulletPlayground(BulletSettings(payload_count=0)) as world:
                robot=world.import_urdf_robot(source["manifest"])
                joints={
                    j["name"]:{
                        "joint_type":"prismatic",
                        "max_speed":j["max_velocity"],
                        "max_acceleration":10.,
                        "max_jerk":100.,
                    }
                    for j in robot["joints"] if j["lower"] is not None
                }
            file=root/"reviewed-kinematic-limits.json"
            file.write_text(json.dumps({
                "schema":"dsiq-document-controlled-joint-motion-limits-v1",
                "asset_id":"test_limit_gantry",
                "joints":joints,
            }))
            status,report=invoke(*command,"--motion-limit-file",file)
            self.assertEqual(status,0,report)
            self.assertTrue(report["documented_kinematic_gate_used"])
            self.assertTrue(report["kinematic_motion_limit_audit"]["all_documented_limits_passed"])
            self.assertFalse(report["physical_estop_present"])
            self.assertFalse(report["industrial_safety_certified"])


if __name__=="__main__":
    unittest.main()
