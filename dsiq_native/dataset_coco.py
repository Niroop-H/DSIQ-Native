"""Convert REAL Bullet segmentation pixels into a bounded COCO detection dataset.

This does not hallucinate annotated machines: all bounding boxes and pixel
areas derive from verified captured segmentation NPZ data. COCO labels remain
reference-simulation labels, never calibrated real-world ground truth.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import random

from .synthetic_dataset import validate_dataset

MASK24=(1<<24)-1


def export_coco(source: str | Path, output: str | Path, *,
                validation_fraction: float = .25, seed: int = 1007) -> dict:
    if type(seed) is not int or not 0 <= seed <= 2**32-1:
        raise ValueError("COCO split seed must be in unsigned 32-bit range")
    if (not isinstance(validation_fraction,(int,float)) or
            isinstance(validation_fraction,bool) or
            not math.isfinite(validation_fraction) or
            not .05 <= validation_fraction <= .5):
        raise ValueError("Validation fraction must be between 0.05 and 0.5")
    root=Path(source).resolve()
    out=Path(output).resolve()
    if out.exists():
        raise FileExistsError("COCO output already exists; refusing to overwrite")
    dataset=validate_dataset(root)
    if dataset["frame_count"] < 2:
        raise ValueError("A train/validation split needs at least two captured frames")
    data=json.loads((root/"dataset.json").read_text(encoding="utf-8"))
    if data.get("optical_calibrated") or data.get("physical_calibrated"):
        raise ValueError("This converter accepts explicit uncalibrated reference datasets")
    import numpy as np
    categories=sorted({
        category for frame in data["frames"]
        for category in frame["body_categories"].values()
        if category!="ground"
    })
    if not categories:
        raise ValueError("No non-ground body categories found")
    ids={name:i+1 for i,name in enumerate(categories)}
    shuffled=list(range(dataset["frame_count"]))
    random.Random(seed).shuffle(shuffled)
    validation_count=max(1,min(len(shuffled)-1,
        int(round(len(shuffled)*validation_fraction))))
    validation=set(shuffled[:validation_count])
    results={}
    for split in ("train","val"):
        images=[]
        annotations=[]
        next_id=1
        for index,frame in enumerate(data["frames"]):
            if (index in validation)!=(split=="val"):
                continue
            filename=frame["rgb_png"]
            with np.load(root/frame["depth_npz"],allow_pickle=False) as arrays:
                segmentation=np.asarray(arrays["segmentation"],dtype=np.int64)
                depth=np.asarray(arrays["depth_m"],dtype=np.float64)
            width=frame["camera"]["width"]
            height=frame["camera"]["height"]
            if segmentation.shape!=(height,width) or depth.shape!=(height,width):
                raise ValueError("Captured sensor array dimensions contradict camera metadata")
            if not np.isfinite(depth).all():
                raise ValueError("Captured depth has nonfinite values")
            image_id=index+1
            images.append({
                "id":image_id,
                "file_name":filename,
                "width":width,
                "height":height,
                "dsiq_sha256":frame["rgb_sha256"],
                "dsiq_simulation_step":frame["simulation_step_index"],
            })
            body_ids=np.where(segmentation>=0,segmentation & MASK24,-1)
            for raw,category in sorted(frame["body_categories"].items(),
                                       key=lambda v:int(v[0])):
                if category=="ground":
                    continue
                try:
                    body=int(raw)
                except ValueError as exc:
                    raise ValueError("Dataset contains non-integer Bullet body ID") from exc
                if body<0 or body>MASK24 or category not in ids:
                    raise ValueError("Unrecognized segmentation body/category contract")
                ys,xs=np.nonzero(body_ids==body)
                if xs.size==0:
                    continue  # physically occluded or outside camera field of view
                xmin=int(xs.min()); ymin=int(ys.min())
                xmax=int(xs.max())+1; ymax=int(ys.max())+1
                annotations.append({
                    "id":next_id,
                    "image_id":image_id,
                    "category_id":ids[category],
                    "bbox":[xmin,ymin,xmax-xmin,ymax-ymin],
                    "area":int(xs.size),
                    "iscrowd":0,
                    "segmentation":[],
                    "dsiq_body_uid":body,
                    "dsiq_annotation_origin":"real_bullet_segmentation_pixels",
                })
                next_id+=1
        results[split]={
            "info":{
                "description":"DSIQ measured Bullet TinyRenderer reference data",
                "calibrated_sensor":False,
                "source_renderer":"bullet_tiny_renderer_cpu",
                "split_seed":seed,
                "pixel_bbox_ground_truth_only_for_simulator":True,
                "annotation_type":"bbox_only_segmentation_polygons_not_provided",
            },
            "images":images,
            "annotations":annotations,
            "categories":[{"id":i,"name":name,"supercategory":"DSIQ reference"}
                          for name,i in ids.items()],
        }
    out.parent.mkdir(parents=True,exist_ok=True)
    # Write one self-contained JSON with source image references; no
    # unnecessary image duplication or excessive dataset disk usage.
    bundle={
        "schema":"dsiq-coco-detection-export-v1",
        "original_dataset":str(root),
        "integrity_validated":True,
        "seed":seed,
        "train":results["train"],
        "val":results["val"],
        "physical_sensor_calibrated":False,
        "real_world_training_qualified":False,
    }
    out.write_text(json.dumps(bundle,indent=2,allow_nan=False)+"\n",encoding="utf8")
    return {
        "output":str(out),
        "source_verified":True,
        "train_images":len(results["train"]["images"]),
        "validation_images":len(results["val"]["images"]),
        "train_bboxes":len(results["train"]["annotations"]),
        "validation_bboxes":len(results["val"]["annotations"]),
        "categories":list(ids),
        "train_validation_overlap":0,
        "calibrated":False,
        "bbox_source":"measured_bullet_segmentation",
    }
