"""Bounded, reproducible Bullet synthetic RGB-D dataset capture.

Isaac Sim/Replicator gap: this is a CPU prototype of a real
simulation-to-dataset workflow, not a claim of RTX photorealism or
calibrated optics. Each sample runs genuine Bullet physics and TinyRenderer.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import random

from .bullet_runtime import BulletPlayground, BulletSettings, PHYSICS_DT
from .sensors import BulletSensors


@dataclass(frozen=True)
class DatasetConfig:
    frames: int = 8
    seed: int = 1007
    width: int = 160
    height: int = 120
    boxes: int = 3
    settle_steps: int = 72

    def validate(self) -> None:
        for name, value, low, high in (
            ("frames", self.frames, 1, 64),
            ("seed", self.seed, 0, 2**32-1),
            ("width", self.width, 64, 320),
            ("height", self.height, 48, 240),
            ("boxes", self.boxes, 1, 12),
            ("settle_steps", self.settle_steps, 1, 960),
        ):
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f"{name} must be an integer in [{low},{high}]")
        if self.width * self.height > 76800:
            raise ValueError("Dataset capture exceeds 76,800-pixel low-memory budget")


def _hash_file(path: Path) -> str:
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b""):
            sha.update(chunk)
    return sha.hexdigest()


def generate_dataset(directory: str | Path, config: DatasetConfig) -> dict:
    """Write real per-frame images/depth/segmentation and labels.

    Output directory must be NEW or empty; existing frame files are never
    silently overwritten. Randomizes initial body poses/sizes/masses,
    camera placement and joint targets with recorded seeded inputs.

    The random seed controls *scenario parameters*; it is not a promise of
    bitwise-identical physics pixels across Bullet builds or CPU models.
    """
    config.validate()
    folder = Path(directory).resolve()
    if folder.exists() and (not folder.is_dir() or any(folder.iterdir())):
        raise FileExistsError("Dataset output directory must be new or empty")
    folder.mkdir(parents=True, exist_ok=True)
    rnd = random.Random(config.seed)
    records = []
    for index in range(config.frames):
        # Use an isolated physics client per scenario. One frame is in
        # memory at a time: no unbounded multi-camera GPU allocations.
        with BulletPlayground(BulletSettings(payload_count=0)) as world:
            scenario = []
            for obj in range(config.boxes):
                side = round(rnd.uniform(.08, .24), 5)
                mass = round(rnd.uniform(.2, 3.0), 5)
                position = (
                    round(rnd.uniform(-1.2, 1.2), 5),
                    round(rnd.uniform(.8, 1.8), 5),
                    round(rnd.uniform(.55, 1.5), 5),
                )
                body = world.spawn_box(
                    mass_kg=mass, side_m=side, position_m=position
                )
                # Real Bullet materials on actually simulated rigid bodies.
                friction = round(rnd.uniform(.25,.90),5)
                restitution = round(rnd.uniform(0., .25),5)
                world.p.changeDynamics(
                    body, -1, lateralFriction=friction,
                    restitution=restitution,
                    physicsClientId=world.client,
                )
                scenario.append({
                    "body_id":body, "class_name":"reference_payload",
                    "side_m":side, "mass_kg":mass,
                    "initial_xyz_m":list(position),
                    "friction_coefficient":friction,
                    "restitution":restitution,
                })
            joints = (
                round(rnd.uniform(-.30,.30),5),
                round(rnd.uniform(-.30,.10),5),
                round(rnd.uniform(-.05,.30),5),
                round(rnd.uniform(-.20,.20),5),
                round(rnd.uniform(-.15,.15),5),
                round(rnd.uniform(-.10,.10),5),
            )
            world.command_joints(joints)
            result = world.step(config.settle_steps)
            eye = (
                round(rnd.uniform(2.3,3.3),5),
                round(rnd.uniform(-3.5,-2.5),5),
                round(rnd.uniform(1.6,2.5),5),
            )
            aim = (
                round(rnd.uniform(-.15,.15),5),
                round(rnd.uniform(-.1,.2),5),
                round(rnd.uniform(.6,.9),5),
            )
            # Physically measured scene -> actual software renderer.
            frame = BulletSensors(world).camera_rgbd(
                eye_m=eye,look_at_m=aim,
                width=config.width,height=config.height,
            )
            rgb = folder / f"frame_{index:04d}.png"
            depth = folder / f"frame_{index:04d}.npz"
            frame.save_png(rgb)
            frame.save_depth_npz(depth)
            state = result
            observed_bodies = [
                {
                    **obj,
                    "observed_xyz_m":state["payload_positions_m"][i],
                } for i,obj in enumerate(scenario)
            ]
            # Segmentation entries are Bullet's actual body/link integers.
            # Per-class labels are explicit metadata, not invented masks.
            records.append({
                "frame_index":index,
                "rgb_png":rgb.name,
                "rgb_sha256":_hash_file(rgb),
                "depth_npz":depth.name,
                "depth_sha256":_hash_file(depth),
                "camera_eye_xyz_m":list(eye),
                "camera_aim_xyz_m":list(aim),
                "camera":frame.metadata(),
                "simulation_step_index":world.step_index,
                "simulated_time_s":world.step_index*PHYSICS_DT,
                "joint_target_rad":list(joints),
                "measured_joint_rad":state["joint_radians"],
                "body_categories":{
                    str(world.ground):"ground",
                    str(world.robot):"reference_6r_robot",
                    **{str(obj["body_id"]):"reference_payload" for obj in scenario},
                },
                "payloads":observed_bodies,
                "physics_contact_count":state["num_contacts"],
            })
    manifest = {
        "schema":"dsiq-synthetic-bullet-v1",
        "engine":"pybullet_cpp_cpu",
        "renderer":"bullet_tiny_renderer_cpu",
        "config":asdict(config),
        "frames":records,
        "units":{"position":"meters","time":"seconds","joint_angle":"radians"},
        "pixel_labels":"bullet_body_and_link_segmentation_ids_not_ground_truth_semantics",
        "optical_calibrated":False,
        "physical_calibrated":False,
        "randomization":"seeded_camera_pose_payload_material_mass_geometry_and_joint_commands",
        "reproducibility":"seeded_inputs_only_cross_platform_exact_pixels_not_guaranteed",
        "use":"research_reference_dataset_not_real_sensor_or_oem_data",
    }
    target = folder / "dataset.json"
    target.write_text(json.dumps(manifest,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    return {
        "output":str(folder),
        "manifest":str(target),
        "frame_count":len(records),
        "engine":manifest["engine"],
        "renderer":manifest["renderer"],
        "seed":config.seed,
        "calibrated":False,
        "max_pixels_per_frame":config.width*config.height,
    }


def validate_dataset(directory: str | Path) -> dict:
    """Check the manifest and each captured frame for exact integrity."""
    folder = Path(directory).resolve()
    manifest_file = folder / "dataset.json"
    if not manifest_file.is_file() or manifest_file.stat().st_size > 4*1024*1024:
        raise ValueError("Missing or oversized DSIQ synthetic manifest")
    data = json.loads(manifest_file.read_text(encoding="utf-8"))
    if data.get("schema") != "dsiq-synthetic-bullet-v1":
        raise ValueError("Unknown synthetic dataset schema")
    config = DatasetConfig(**data["config"])
    config.validate()
    frames = data.get("frames")
    if not isinstance(frames,list) or len(frames)!=config.frames:
        raise ValueError("Frame count and declared configuration differ")
    checked = 0
    for index,entry in enumerate(frames):
        if entry.get("frame_index") != index:
            raise ValueError(f"Frame ordering mismatch at index {index}")
        for field,sha_field in (("rgb_png","rgb_sha256"),("depth_npz","depth_sha256")):
            expected=f"frame_{index:04d}."+("png" if field=="rgb_png" else "npz")
            if entry.get(field)!=expected:
                raise ValueError("Dataset image paths must match canonical local names")
            path=folder/expected
            if not path.is_file() or _hash_file(path)!=entry.get(sha_field):
                raise ValueError(f"Dataset integrity failure: {expected}")
            checked+=1
    return {
        "valid":True, "frame_count":config.frames,
        "files_verified":checked,
        "seed":config.seed, "calibrated":False,
    }
