"""Hashes can be verified; independent safety approval cannot be self-awarded."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.launch_evidence import audit_launch_evidence,REQUIREMENTS,SCHEMA


class LaunchEvidenceTests(unittest.TestCase):
    def test_empty_inventory_is_explicitly_blocked(self):
        with TemporaryDirectory() as temp:
            path=Path(temp)/"gate.json"
            path.write_text(json.dumps({"schema":SCHEMA,"evidence":{}}))
            report=audit_launch_evidence(path)
            self.assertEqual(report["category_count"],8)
            self.assertEqual(report["file_integrity_categories_verified"],0)
            self.assertEqual(report["missing_or_failed_categories"],8)
            self.assertFalse(report["production_or_industrial_launch_authorized"])

    def test_even_every_matching_hash_cannot_authenticate_customer_safety(self):
        with TemporaryDirectory() as temp:
            root=Path(temp)
            case=root/"evidence.txt"
            case.write_text("UNIT TEST DATA IS NOT OEM VALIDATION")
            sha=hashlib.sha256(case.read_bytes()).hexdigest()
            catalog={
                "schema":SCHEMA,
                "evidence":{
                    name:{"attachments":[{"file":"evidence.txt","sha256":sha}]}
                    for name in REQUIREMENTS
                },
            }
            path=root/"gate.json"
            path.write_text(json.dumps(catalog))
            result=audit_launch_evidence(path)
            self.assertTrue(result["all_source_files_integrity_verified"])
            self.assertEqual(result["file_integrity_categories_verified"],8)
            self.assertFalse(result["independent_technical_assessment_completed"])
            self.assertFalse(result["production_or_industrial_launch_authorized"])
            self.assertEqual(result["release_gate"],"BLOCKED_PENDING_INDEPENDENT_EVIDENCE_AND_SIGNOFF")
            case.write_text("TAMPERED ENGINEERING MEASUREMENT")
            invalid=audit_launch_evidence(path)
            self.assertFalse(invalid["all_source_files_integrity_verified"])
            self.assertEqual(invalid["file_integrity_categories_verified"],0)

    def test_path_traversal_and_unlisted_category_rejected(self):
        with TemporaryDirectory() as temp:
            root=Path(temp)
            path=root/"gate.json"
            path.write_text(json.dumps({
                "schema":SCHEMA,
                "evidence":{"BAD_CATEGORY":{}},
            }))
            with self.assertRaisesRegex(ValueError,"Unknown"):
                audit_launch_evidence(path)
            path.write_text(json.dumps({
                "schema":SCHEMA,
                "evidence":{"P0_ROBOT_SAFETY":{
                    "attachments":[{"file":"../outside.txt","sha256":"0"*64}]
                }},
            }))
            with self.assertRaisesRegex(ValueError,"traversal"):
                audit_launch_evidence(path)

if __name__=="__main__":
    unittest.main()
