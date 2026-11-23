"""COCO boxes are measured from real Bullet pixels, not inferred 3D boxes."""
from __future__ import annotations
import importlib.util
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from dsiq_native.dataset_coco import export_coco
from dsiq_native.synthetic_dataset import DatasetConfig,generate_dataset

HAS_BULLET=importlib.util.find_spec("pybullet") is not None


class CocoArgumentTests(unittest.TestCase):
    def test_rejects_invalid_fraction_seed(self):
        for value in (float("nan"),-1.,.6,0.):
            with self.subTest(fraction=value),self.assertRaises(ValueError):
                export_coco("missing","missing.json",validation_fraction=value)
        for seed in (-1,2**32,1.2,True):
            with self.subTest(seed=seed),self.assertRaises(ValueError):
                export_coco("missing","missing.json",seed=seed)


@unittest.skipUnless(HAS_BULLET,"Actual Bullet and NumPy dependencies needed")
class MeasuredCOCOTests(unittest.TestCase):
    def test_validated_bbox_export_has_disjoint_splits_and_real_mask_pixels(self):
        import numpy as np
        with TemporaryDirectory() as tmp:
            source=Path(tmp)/"dataset"
            generate_dataset(source,DatasetConfig(
                frames=4,seed=1007,width=64,height=48,boxes=2,settle_steps=8
            ))
            out=Path(tmp)/"coco.json"
            report=export_coco(source,out,seed=42)
            self.assertTrue(report["source_verified"])
            self.assertEqual(report["train_images"],3)
            self.assertEqual(report["validation_images"],1)
            self.assertEqual(report["train_validation_overlap"],0)
            pack=json.loads(out.read_text())
            train={image["id"] for image in pack["train"]["images"]}
            valid={image["id"] for image in pack["val"]["images"]}
            self.assertFalse(train & valid)
            self.assertEqual(len(train|valid),4)
            expected_categories={x["id"] for x in pack["train"]["categories"]}
            actual_boxes=pack["train"]["annotations"]+pack["val"]["annotations"]
            self.assertGreater(len(actual_boxes),0)
            for label in actual_boxes:
                self.assertIn(label["category_id"],expected_categories)
                self.assertGreater(label["area"],0)
                frame_idx=label["image_id"]-1
                with np.load(source/f"frame_{frame_idx:04d}.npz",
                             allow_pickle=False) as values:
                    seg=np.asarray(values["segmentation"],dtype=np.int64)
                mask=np.where(seg>=0,seg & ((1<<24)-1),-1)==label["dsiq_body_uid"]
                ys,xs=np.nonzero(mask)
                self.assertEqual(int(xs.size),label["area"])
                xmin,ymin,width,height=label["bbox"]
                self.assertEqual(
                    [xmin,ymin,width,height],
                    [int(xs.min()),int(ys.min()),
                     int(xs.max()-xs.min()+1),int(ys.max()-ys.min()+1)]
                )
                self.assertEqual(label["segmentation"],[])
            with self.assertRaises(FileExistsError):
                export_coco(source,out)
            second=Path(tmp)/"second.json"
            report2=export_coco(source,second,seed=42)
            self.assertEqual(json.loads(second.read_text())["train"],
                             pack["train"])
            self.assertEqual(report2["train_images"],report["train_images"])

    def test_detects_tampered_source_images_before_export(self):
        with TemporaryDirectory() as tmp:
            source=Path(tmp)/"dataset"
            generate_dataset(source,DatasetConfig(
                frames=2,width=64,height=48,boxes=1,settle_steps=5
            ))
            with (source/"frame_0000.png").open("ab") as f:
                f.write(b"tampering")
            with self.assertRaisesRegex(ValueError,"integrity"):
                export_coco(source,Path(tmp)/"out.json")


if __name__=="__main__":
    unittest.main()
