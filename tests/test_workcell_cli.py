"""End-to-end CLI workcell policy and simulation, without any special named model."""
from __future__ import annotations

import contextlib
import hashlib
from io import StringIO
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import importlib.util
import unittest

from dsiq_native.cli import main
from dsiq_native.machine_designer import MachineDesign, save_project


def invoke(*args):
    output,error=StringIO(),StringIO()
    with contextlib.redirect_stdout(output),contextlib.redirect_stderr(error):
        code=main([str(x) for x in args])
    data=json.loads(output.getvalue() or error.getvalue())
    return code,data


class WorkcellCLIContracts(unittest.TestCase):
    def setUp(self):
        self.folder=TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        self.root=Path(self.folder.name)
        self.m1=save_project(MachineDesign(asset_id="catalog_robot_alpha"),
                             self.root/"machine_alpha")
        self.m2=save_project(MachineDesign(asset_id="catalog_robot_beta"),
                             self.root/"machine_beta")
        self.first=self.root/"alpha.workcell.json"
        self.second=self.root/"beta.workcell.json"
        for file,manifest,name in (
            (self.first,self.m1["manifest"],"robotic_cell_alpha"),
            (self.second,self.m2["manifest"],"robotic_cell_beta"),
        ):
            code,data=invoke("workcell-new",file,"--id",name,
                             "--name",name,"--manifest",manifest)
            self.assertEqual(code,0,data)
            self.assertEqual(data["status"],"INCOMPLETE")
        self.catalog=self.root/"workcells.catalog.json"
        self.catalog.write_text(json.dumps({
            "schema":"dsiq-workcell-catalog-v1",
            "workcells":[{
                "file":p.name,"sha256":hashlib.sha256(p.read_bytes()).hexdigest(),
            } for p in (self.first,self.second)],
        }),encoding="utf-8")

    def test_two_workcells_audited_without_special_case(self):
        code,data=invoke("workcell-catalog",self.catalog)
        self.assertEqual(code,0,data)
        self.assertEqual(data["total"],2)
        self.assertEqual(data["incomplete"],2)
        self.assertFalse(data["all_industrially_certified"])

    def test_build_catalog_via_cli(self):
        output=self.root/"new.catalog.json"
        code,result=invoke("workcell-catalog-build",output,self.first,self.second)
        self.assertEqual(code,0,result)
        self.assertEqual(result["total"],2)
        code,verified=invoke("workcell-catalog",output)
        self.assertEqual(code,0,verified)
        self.assertEqual(verified["total"],2)

    def test_workcell_inspect_returns_explicit_missing_evidence(self):
        code,data=invoke("workcell-inspect",self.first)
        self.assertEqual(code,0,data)
        self.assertEqual(data["status"],"INCOMPLETE")
        self.assertFalse(data["certified"])
        self.assertIn("geometry_cad",data["required_gates"])
        self.assertTrue(any("safety" in item.lower() for item in data["blockers"]))

    def test_edit_tamper_fails_catalog_gate(self):
        self.second.write_text(self.second.read_text().replace(
            '"revision": "draft-1"','"revision": "tampered"'
        ),encoding="utf-8")
        code,data=invoke("workcell-catalog",self.catalog)
        self.assertNotEqual(code,0)
        self.assertFalse(data.get("ok",True))


@unittest.skipUnless(importlib.util.find_spec("pybullet"),
                     "Actual Bullet optional physics backend required")
class LiveWorkcellCLITest(WorkcellCLIContracts):
    def test_runtime_checks_each_workcell_assembly(self):
        code,data=invoke("workcell-catalog",self.catalog,
                         "--simulate","--steps","24")
        self.assertEqual(code,0,data)
        self.assertTrue(data["all_runtime_checks_passed"],data)
        self.assertEqual(len(data["results"]),2)
        for case in data["results"]:
            self.assertTrue(case["runtime_executed"])
            self.assertTrue(case["runtime"]["finite_joint_feedback"])
            self.assertFalse(case["certified"])


if __name__=="__main__":
    unittest.main()
