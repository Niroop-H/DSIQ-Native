"""Recorded Bullet synthetic dataset: actual image/depth, checksums and reproducible inputs."""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from dsiq_native.synthetic_dataset import (
    DatasetConfig, generate_dataset, validate_dataset,
)

HAS_PHYSICS=importlib.util.find_spec("pybullet") is not None


class SyntheticDatasetContractTests(unittest.TestCase):
    def test_resource_caps_are_enforced(self):
        for bad in (
            {"frames":1000},
            {"frames":0},
            {"width":8000},
            {"height":800},
            {"seed":-1},
            {"boxes":20},
            {"settle_steps":100000},
            {"width":320,"height":241},
        ):
            with self.subTest(bad=bad),self.assertRaises(ValueError):
                DatasetConfig(**bad).validate()


@unittest.skipUnless(HAS_PHYSICS,"Optional Bullet backend required")
class BulletDatasetIntegrationTests(unittest.TestCase):
    def test_real_rgbd_depth_segmentation_records_and_integrity(self):
        import numpy as np
        with TemporaryDirectory() as tmp:
            path=Path(tmp)/"dataset"
            config=DatasetConfig(frames=2,seed=42,width=64,height=48,
                                 boxes=2,settle_steps=20)
            result=generate_dataset(path,config)
            self.assertEqual(result["frame_count"],2)
            self.assertFalse(result["calibrated"])
            verified=validate_dataset(path)
            self.assertTrue(verified["valid"])
            self.assertEqual(verified["files_verified"],4)
            manifest=json.loads((path/"dataset.json").read_text())
            self.assertEqual(len(manifest["frames"]),2)
            self.assertEqual(manifest["renderer"],"bullet_tiny_renderer_cpu")
            first=manifest["frames"][0]
            self.assertEqual(len(first["payloads"]),2)
            self.assertEqual(first["camera"]["width"],64)
            self.assertEqual(len(first["body_categories"]),4)
            self.assertEqual(len(first["measured_joint_rad"]),6)
            self.assertEqual((path/"frame_0000.png").read_bytes()[:8],
                             b"\x89PNG\r\n\x1a\n")
            with np.load(path/"frame_0000.npz") as data:
                self.assertEqual(data["depth_m"].shape,(48,64))
                self.assertEqual(data["segmentation"].shape,(48,64))
                self.assertTrue(np.isfinite(data["depth_m"]).all())
            with self.assertRaises(FileExistsError):
                generate_dataset(path,config)

    def test_seed_repeats_scenario_parameters_not_claimed_exact_pixels(self):
        with TemporaryDirectory() as tmp:
            config=DatasetConfig(frames=1,seed=99,width=64,height=48,
                                 boxes=1,settle_steps=5)
            a=Path(tmp)/"a";b=Path(tmp)/"b"
            generate_dataset(a,config)
            generate_dataset(b,config)
            ma=json.loads((a/"dataset.json").read_text())["frames"][0]
            mb=json.loads((b/"dataset.json").read_text())["frames"][0]
            self.assertEqual(ma["joint_target_rad"],mb["joint_target_rad"])
            self.assertEqual(ma["camera_eye_xyz_m"],mb["camera_eye_xyz_m"])
            self.assertEqual(ma["payloads"][0]["initial_xyz_m"],
                             mb["payloads"][0]["initial_xyz_m"])
            self.assertFalse(json.loads((a/"dataset.json").read_text())["optical_calibrated"])

    def test_dataset_integrity_detects_modified_capture(self):
        with TemporaryDirectory() as tmp:
            folder=Path(tmp)/"capture"
            generate_dataset(folder,DatasetConfig(frames=1,width=64,height=48,
                              boxes=1,settle_steps=2))
            with (folder/"frame_0000.png").open("ab") as f:
                f.write(b"tamper")
            with self.assertRaisesRegex(ValueError,"integrity"):
                validate_dataset(folder)


if __name__=="__main__":
    unittest.main()
