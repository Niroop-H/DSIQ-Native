"""End-to-end user-defined machine and environment CLI integration."""
import contextlib
from io import StringIO
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import importlib.util
import unittest

from dsiq_native.cli import main

def call(*args):
    stdout,stderr=StringIO(),StringIO()
    with contextlib.redirect_stdout(stdout),contextlib.redirect_stderr(stderr):
        exit_code=main(list(map(str,args)))
    value=stdout.getvalue() if stdout.getvalue() else stderr.getvalue()
    return exit_code,json.loads(value)

class MachineCliContractTests(unittest.TestCase):
    def test_user_authored_machine_project_creation_and_rebuild(self):
        with TemporaryDirectory() as tmp:
            folder=Path(tmp)/"xyz"
            code,data=call("machine-create",folder,"--name","robotic_inspection_gantry",
                           "--x-travel","1.5","--y-travel","0.5","--z-travel","0.3")
            self.assertEqual(code,0,data)
            self.assertTrue(Path(data["urdf"]).exists())
            self.assertEqual(data["fidelity"],"parametric_reference_not_oem")
            with self.assertRaises(AssertionError):
                self.assertEqual(data["fidelity"],"oem_certified")
            rebuild=Path(tmp)/"rebuild"
            code,out=call("machine-build",data["design_json"],rebuild)
            self.assertEqual(code,0,out)
            self.assertTrue(Path(out["manifest"]).exists())
            code,error=call("machine-create",folder,"--name","same_id_again")
            self.assertEqual(code,2)
            self.assertFalse(error["ok"])

@unittest.skipUnless(importlib.util.find_spec("pybullet"),"Bullet required")
class RealEnvironmentCliTests(unittest.TestCase):
    def test_real_semantic_scene_and_png_preview(self):
        with TemporaryDirectory() as tmp:
            path=Path(tmp)/"world.png"
            code,result=call("environment-render","robotics_cell",
                "--steps","12","--png",path)
            self.assertEqual(code,0,result)
            self.assertTrue(path.is_file())
            self.assertGreater(result["environment"]["collision_parts"],20)
            self.assertEqual(result["image"]["renderer"],"PYBULLET_TINY_RENDERER_CPU")

    def test_reference_physics_verification_cli(self):
        code,result=call("physics-verify")
        self.assertEqual(code,0,result)
        self.assertTrue(result["passed"])
        self.assertEqual(result["case_count"],4)

if __name__=="__main__":
    unittest.main()
