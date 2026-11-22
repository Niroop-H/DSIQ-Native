"""Safe ROS URDF read and actual Bullet import acceptance fixtures.

No fixture is represented as an OEM machine or production-qualified dynamics.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import importlib.util
import unittest

from dsiq_native.urdf_bridge import inspect_urdf, validate_robot_manifest


TEST_URDF = """<?xml version="1.0"?>
<robot name="reference_hinge">
  <link name="base">
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
    <visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>
    <collision><geometry><box size="0.2 0.2 0.2"/></geometry></collision>
  </link>
  <link name="arm">
    <inertial>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.03" ixy="0" ixz="0" iyy="0.03" iyz="0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry><box size="0.1 0.1 0.4"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" rpy="0 0 0"/>
      <geometry><box size="0.1 0.1 0.4"/></geometry>
    </collision>
  </link>
  <joint name="pivot" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="30" velocity="2"/>
  </joint>
</robot>
"""


class UrdfValidationTests(unittest.TestCase):
    def setUp(self):
        self.folder = TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root = Path(self.folder.name)
        self.source = self.root / "fixture.urdf"
        self.source.write_text(TEST_URDF, encoding="utf-8")
        (self.root / "rights.txt").write_text(
            "This fixture is original test data, not real industrial CAD.",
            encoding="utf-8",
        )
        self.manifest = {
            "schema": "dsiq-robot-urdf-v1",
            "asset_class": "test_fixture",
            "asset_id": "test_hinge",
            "manufacturer": "Test Fixture Author",
            "model": "Synthetic Reference Hinge (not OEM)",
            "revision": "1",
            "source_uri": "local://self-authored-fixture",
            "license": {"usage_rights": "owned", "evidence_file": "rights.txt"},
            "geometry": {
                "urdf_file": "fixture.urdf",
                "sha256": hashlib.sha256(self.source.read_bytes()).hexdigest(),
            },
        }
        self.manifest_path = self.root / "robot.manifest.json"
        self.save()

    def save(self):
        self.manifest_path.write_text(json.dumps(self.manifest), encoding="utf-8")

    def test_independent_urdf_tree_and_inertia_analysis(self):
        robot = inspect_urdf(self.source)
        self.assertEqual(robot.robot_name, "reference_hinge")
        self.assertEqual(robot.root_link, "base")
        self.assertEqual(len(robot.links), 2)
        self.assertEqual(len(robot.joints), 1)
        self.assertEqual(robot.joints[0].name, "pivot")
        self.assertEqual(robot.joints[0].kind, "revolute")
        self.assertAlmostEqual(robot.joints[0].lower, -1.57)
        self.assertEqual(len(robot.links_without_inertial), 0)
        self.assertFalse(robot.as_dict()["physically_qualified"])

    def test_authorized_fixture_manifest_can_be_inspected(self):
        manifest, robot = validate_robot_manifest(self.manifest_path)
        self.assertEqual(manifest["asset_class"], "test_fixture")
        self.assertEqual(robot.robot_name, "reference_hinge")
        self.assertFalse(robot.as_dict()["physically_qualified"])

    def test_checksum_and_provenance_gate(self):
        self.manifest["geometry"]["sha256"] = "0"*64
        self.save()
        with self.assertRaisesRegex(ValueError, "checksum"):
            validate_robot_manifest(self.manifest_path)

    def test_missing_license_evidence_is_fatal(self):
        (self.root / "rights.txt").unlink()
        with self.assertRaisesRegex(ValueError, "License"):
            validate_robot_manifest(self.manifest_path)

    def test_reject_parent_escape_and_remote_uris(self):
        self.manifest["geometry"]["urdf_file"] = "../unauthorized.urdf"
        self.save()
        with self.assertRaises(ValueError):
            validate_robot_manifest(self.manifest_path)
        self.manifest["geometry"]["urdf_file"] = "https://bad.example/arm.urdf"
        self.save()
        with self.assertRaises(ValueError):
            validate_robot_manifest(self.manifest_path)

    def test_xacro_or_xml_entity_not_executed(self):
        self.source.write_text(
            '<!DOCTYPE robot [<!ENTITY x SYSTEM "file:///etc/passwd">]>'
            '<robot name="unsafe"><link name="base"/></robot>',
            encoding="utf-8",
        )
        with self.assertRaisesRegex(ValueError, "prohibited"):
            inspect_urdf(self.source)

    def test_bad_joint_tree_rejected(self):
        text = TEST_URDF.replace(
            '<child link="arm"/>', '<child link="missing"/>',
        )
        self.source.write_text(text, encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "parent/child"):
            inspect_urdf(self.source)

    def test_unphysical_inertia_tensor_is_rejected_before_bullet(self):
        source=TEST_URDF.replace('ixy="0" ixz="0" iyy="0.1"',
                                 'ixy="1.0" ixz="0" iyy="0.1"')
        self.source.write_text(source,encoding="utf-8")
        with self.assertRaisesRegex(ValueError,"positive-definite"):
            inspect_urdf(self.source)

    def test_unphysical_inertia_triangle_inequality_is_rejected(self):
        source=TEST_URDF.replace('izz="0.1"','izz="10.0"')
        self.source.write_text(source,encoding="utf-8")
        with self.assertRaisesRegex(ValueError,"inertia moment"):
            inspect_urdf(self.source)

    def test_joint_without_physical_motor_effort_is_rejected(self):
        source=TEST_URDF.replace('effort="30"','')
        self.source.write_text(source,encoding="utf-8")
        with self.assertRaisesRegex(ValueError,"missing required motor effort"):
            inspect_urdf(self.source)

    def test_visual_oem_tag_requires_real_mesh_and_review(self):
        self.manifest["asset_class"] = "oem_verified"
        self.save()
        with self.assertRaisesRegex(ValueError, "mesh"):
            validate_robot_manifest(self.manifest_path)

    def test_visual_review_does_not_imply_physical_certification(self):
        self.source.write_text(TEST_URDF.replace(
            '<visual><geometry><box size="0.2 0.2 0.2"/></geometry></visual>',
            '<visual><geometry><mesh filename="base.obj"/></geometry></visual>',
        ), encoding="utf-8")
        (self.root / "base.obj").write_text(
            "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n", encoding="utf-8",
        )
        (self.root / "review.txt").write_text("Review fixture only", encoding="utf-8")
        self.manifest["asset_class"] = "engineering_reference"
        self.manifest["visual_review"] = {
            "approved": True, "reviewer": "Unit Test",
            "reviewed_against": "local://test-fixture-only",
            "evidence_file": "review.txt",
        }
        self.manifest["geometry"]["sha256"] = hashlib.sha256(self.source.read_bytes()).hexdigest()
        self.save()
        _, inspection = validate_robot_manifest(self.manifest_path)
        self.assertEqual(len(inspection.visual_mesh_paths), 1)
        self.assertFalse(inspection.as_dict()["physically_qualified"])


@unittest.skipUnless(importlib.util.find_spec("pybullet"), "Bullet backend required")
class ActualBulletUrdfTests(UrdfValidationTests):
    def test_real_import_controls_revolute_and_reports_motion(self):
        from dsiq_native.bullet_runtime import BulletPlayground, BulletSettings
        with BulletPlayground(BulletSettings(payload_count=0)) as world:
            import_report = world.import_urdf_robot(str(self.manifest_path))
            self.assertEqual(import_report["asset_id"], "test_hinge")
            self.assertEqual(import_report["joint_count"], 1)
            self.assertFalse(import_report["physically_qualified"])
            command = world.command_imported_joint("test_hinge", "pivot", 0.4)
            self.assertEqual(command["units"], "radians")
            with self.assertRaises(ValueError):
                world.command_imported_joint("test_hinge","pivot",3.0)
            with self.assertRaises(FileExistsError):
                world.import_urdf_robot(str(self.manifest_path))
            data = world.step(200)
            self.assertEqual(len(data["imported_robots"]), 1)
            state = data["imported_robots"][0]["joint_states"][0]
            self.assertGreater(state["position"], .05)
            self.assertFalse(data["physically_qualified"])


if __name__ == "__main__":
    unittest.main()
