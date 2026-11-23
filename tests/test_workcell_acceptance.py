"""Every workcell is evaluated with the same standards, not one showcase demo."""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.machine_designer import MachineDesign, save_project
from dsiq_native.workcell_acceptance import (
    CATALOG_SCHEMA, GATES, MAX_CATALOG, create_workcell,
    inspect_workcell, audit_workcell, audit_catalog, create_catalog,
)

HAS_BULLET=importlib.util.find_spec("pybullet") is not None


class WorkcellProjectContracts(unittest.TestCase):
    def setUp(self):
        self.directory=TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root=Path(self.directory.name)
        self.project1=self._build("assembly_cell_alpha")
        self.project2=self._build("assembly_cell_beta")
        self.catalog=self.root/"catalog.json"
        entries=[
            {"file":p.name,"sha256":hashlib.sha256(p.read_bytes()).hexdigest()}
            for p in (self.project1,self.project2)
        ]
        self.catalog.write_text(json.dumps({
            "schema":CATALOG_SCHEMA,"workcells":entries,
        }),encoding="utf-8")

    def _build(self,name):
        machine=save_project(
            MachineDesign(asset_id=name+"_gantry"),
            self.root/name,
        )
        file=self.root/(name+".workcell.json")
        create_workcell(file,workcell_id=name,name="Physical XYZ assembly testing "+name,
                        manifest=machine["manifest"])
        return file

    def test_two_distinct_workcells_share_all_mandatory_gates(self):
        for project in (self.project1,self.project2):
            report=inspect_workcell(project)
            self.assertEqual(report["status"],"INCOMPLETE")
            self.assertFalse(report["certified"])
            self.assertFalse(report["passed_engineering_gate"])
            self.assertEqual(report["required_gates"],list(GATES))
            self.assertTrue(all("Missing independent acceptance evidence" in x
                                for x in report["blockers"] if "acceptance evidence" in x))
            self.assertEqual(len(report["equipment"]),1)
            self.assertTrue(report["equipment"][0]["source_integrity_verified"])
            self.assertGreater(report["equipment"][0]["actuated_joint_count"],0)

    def test_catalog_checks_every_workcell_not_first_only(self):
        data=audit_catalog(self.catalog)
        self.assertEqual(data["total"],2)
        self.assertEqual(data["rejected"],0)
        self.assertEqual(data["incomplete"],2)
        self.assertFalse(data["all_industrially_certified"])
        self.assertTrue(data["all_source_integrity_valid"])
        self.assertFalse(data["runtime_checks_executed"])
        self.assertIsNone(data["all_runtime_checks_passed"])

    def test_urdf_manifest_hash_tampering_is_rejected(self):
        source=json.loads(self.project1.read_text())
        source["equipment"][0]["manifest_sha256"]="0"*64
        self.project1.write_text(json.dumps(source),encoding="utf-8")
        r=inspect_workcell(self.project1)
        self.assertEqual(r["status"],"REJECTED")
        self.assertTrue(r["errors"])
        with self.assertRaisesRegex(ValueError,"checksum"):
            audit_catalog(self.catalog)

    def test_evidence_file_integrity_is_not_external_certification(self):
        project=json.loads(self.project1.read_text())
        for gate in GATES:
            evidence=self.root/(gate+".txt")
            evidence.write_text("Author declaration; not independently verified",
                                encoding="utf-8")
            project["acceptance_evidence"][gate]={
                "file":evidence.name,
                "sha256":hashlib.sha256(evidence.read_bytes()).hexdigest(),
            }
        self.project1.write_text(json.dumps(project),encoding="utf-8")
        r=inspect_workcell(self.project1)
        self.assertEqual(r["status"],"INCOMPLETE")
        self.assertFalse(r["certified"])
        self.assertTrue(all(v=="FILE_HASH_VERIFIED_ONLY"
                            for v in r["evidence"].values()))

    def test_nonportable_manifests_and_parent_paths_rejected(self):
        file=self.root/"unsafe.workcell.json"
        with self.assertRaises(FileNotFoundError):
            create_workcell(file,workcell_id="test_escape_cell",
                name="unsafe",manifest=Path(self.directory.name).parent/"secret.json")
        document=json.loads(self.project2.read_text())
        document["equipment"][0]["manifest"]="../external.json"
        self.project2.write_text(json.dumps(document),encoding="utf-8")
        self.assertEqual(inspect_workcell(self.project2)["status"],"REJECTED")

    def test_missing_roles_does_not_pass_as_professional_workcell(self):
        document=json.loads(self.project2.read_text())
        document["interface"]["required_sensors"]=[]
        self.project2.write_text(json.dumps(document),encoding="utf-8")
        r=inspect_workcell(self.project2)
        self.assertFalse(r["passed_engineering_gate"])
        for role in ("tooling","fixture","safety"):
            self.assertIn("Missing required workcell role: "+role,r["blockers"])
        self.assertTrue(any("sensor" in b.lower() for b in r["blockers"]))

    def test_build_catalog_from_all_portable_workcells(self):
        target=self.root/"generated.catalog.json"
        output=create_catalog(target,[self.project1,self.project2])
        self.assertEqual(output["total"],2)
        self.assertFalse(output["every_workcell_certified"])
        self.assertEqual(audit_catalog(target)["total"],2)
        with self.assertRaises(FileExistsError):
            create_catalog(target,[self.project1])

    def test_catalog_requires_each_identity_and_hash(self):
        doc=json.loads(self.catalog.read_text())
        doc["workcells"][1]["sha256"]="0"*64
        self.catalog.write_text(json.dumps(doc),encoding="utf-8")
        with self.assertRaisesRegex(ValueError,"checksum"):
            audit_catalog(self.catalog)

    def test_duplicate_workcell_id_is_rejected_in_catalog(self):
        file=self.root/"clone.workcell.json"
        file.write_bytes(self.project1.read_bytes())
        entries=json.loads(self.catalog.read_text())
        entries["workcells"][1]={
            "file":file.name,"sha256":hashlib.sha256(file.read_bytes()).hexdigest()
        }
        self.catalog.write_text(json.dumps(entries),encoding="utf-8")
        with self.assertRaisesRegex(ValueError,"Duplicate workcell ID"):
            audit_catalog(self.catalog)

    def test_cannot_overwrite_an_existing_workcell(self):
        source=self.root/"assembly_cell_alpha"/"machine.manifest.json"
        with self.assertRaises(FileExistsError):
            create_workcell(self.project1,workcell_id="overwrite_attempt",
                            name="attempt",manifest=source)


@unittest.skipUnless(HAS_BULLET,"Actual Bullet optional physics backend required")
class WorkcellDynamicEvidenceTests(WorkcellProjectContracts):
    def test_actual_multicell_batch_measures_each_bullet_workcell(self):
        result=audit_catalog(self.catalog,simulate=True,steps=60)
        self.assertEqual(result["total"],2)
        self.assertTrue(result["all_runtime_checks_passed"],result)
        self.assertFalse(result["all_industrially_certified"])
        for workcell in result["results"]:
            self.assertTrue(workcell["runtime_executed"])
            self.assertTrue(workcell["runtime"]["finite_joint_feedback"])
            self.assertEqual(workcell["runtime"]["lidar_rays_measured"],24)
            self.assertGreaterEqual(len(workcell["runtime"]["assemblies"]),1)
            self.assertFalse(workcell["runtime"]["physics_validated_for_industry"])
            for machine in workcell["runtime"]["assemblies"]:
                self.assertGreater(len(machine["joints_commanded"]),0)
                for joint in machine["joints_commanded"]:
                    self.assertIn("observed",joint)

    def test_rejected_source_refuses_physics_execution(self):
        document=json.loads(self.project1.read_text())
        document["equipment"][0]["manifest_sha256"]="0"*64
        self.project1.write_text(json.dumps(document),encoding="utf-8")
        result=audit_workcell(self.project1,steps=4)
        self.assertFalse(result["runtime_executed"])
        self.assertFalse(result["runtime_passed"])
        self.assertTrue(result["errors"])


if __name__=="__main__":
    unittest.main()
