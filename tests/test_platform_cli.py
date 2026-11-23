"""System-wide CLI checks for source-backed data and viewport metrics."""
import contextlib
from io import StringIO
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from dsiq_native.cli import main
from dsiq_native.machine_designer import MachineDesign,save_project

HAS_BULLET=importlib.util.find_spec("pybullet") is not None


def call(*parts):
    out=StringIO()
    err=StringIO()
    with contextlib.redirect_stdout(out),contextlib.redirect_stderr(err):
        code=main([str(v) for v in parts])
    return code,json.loads(out.getvalue() or err.getvalue())


class InventoryCLIContracts(unittest.TestCase):
    def test_machine_inventory_from_real_generated_urdf(self):
        with TemporaryDirectory() as tmp:
            machine=save_project(
                MachineDesign(asset_id="cli_inventory_machine"),
                Path(tmp)/"test_machine",
            )
            code,output=call("machine-inventory",machine["manifest"])
            self.assertEqual(code,0,output)
            self.assertEqual(output["link_count"],5)
            self.assertEqual(output["actuated_joint_count"],3)
            self.assertAlmostEqual(output["declared_total_mass_kg"],32.8)
            self.assertFalse(output["real_oem_dimensions_qualified"])


@unittest.skipUnless(HAS_BULLET,"Measured Bullet data required")
class GraphicsAndCOCOIntegrationCLI(unittest.TestCase):
    def test_bounded_renderer_benchmark_command(self):
        code,report=call("render-benchmark","--frames","1",
                         "--width","64","--height","48")
        self.assertEqual(code,0,report)
        self.assertEqual(len(report["reports"]),3)
        self.assertIsNone(report["rtx_gpu_fps"])
        self.assertFalse(report["qualifies_rtx3050"])

    def test_dataset_generate_then_coco_with_verified_annotations(self):
        with TemporaryDirectory() as tmp:
            folder=Path(tmp)/"data"
            coco=Path(tmp)/"annotations.json"
            code,info=call("dataset-generate",folder,
                           "--frames","2","--seed","11","--width","64",
                           "--height","48","--boxes","1","--steps","8")
            self.assertEqual(code,0,info)
            code,converted=call("dataset-coco",folder,coco,"--seed","42")
            self.assertEqual(code,0,converted)
            self.assertEqual(converted["train_images"],1)
            self.assertEqual(converted["validation_images"],1)
            self.assertEqual(converted["train_validation_overlap"],0)
            self.assertTrue(coco.is_file())
            report=json.loads(coco.read_text())
            self.assertFalse(report["physical_sensor_calibrated"])
            self.assertFalse(report["real_world_training_qualified"])


if __name__=="__main__":
    unittest.main()
