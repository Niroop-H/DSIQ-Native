"""Actual Bullet geometric ray and software RGB-D sensors for robotics workcells.

Uses Bullet collision queries and TinyRenderer in DIRECT mode; no fabricated
telemetry, no RTX or optical/sensor-noise calibration claims. Depth is metric
camera Z distance reconstructed from the OpenGL depth buffer.
"""
from __future__ import annotations

from dataclasses import dataclass
import binascii
import math
from pathlib import Path
import struct
from typing import Sequence
import zlib

from .bullet_runtime import BulletPlayground


def _finite_xyz(values: Sequence[float], name: str) -> tuple[float, float, float]:
    if len(values) != 3:
        raise ValueError(f"{name} must contain three coordinates")
    v = tuple(float(x) for x in values)
    if any(not math.isfinite(x) or abs(x) > 100 for x in v):
        raise ValueError(f"{name} must contain finite coordinates within +/-100m")
    return v  # type: ignore[return-value]


@dataclass(frozen=True)
class CameraFrame:
    width: int
    height: int
    rgba: bytes
    metric_depth_m: tuple[float, ...]
    segmentation: tuple[int, ...]
    near_m: float
    far_m: float
    renderer: str = "PYBULLET_TINY_RENDERER_CPU"

    def metadata(self) -> dict:
        return {
            "width": self.width,
            "height": self.height,
            "near_m": self.near_m,
            "far_m": self.far_m,
            "depth_min_m": min(self.metric_depth_m),
            "depth_max_m": max(self.metric_depth_m),
            "renderer": self.renderer,
            "sensor_calibrated": False,
            "depth_units": "meters_along_camera_optical_axis",
            "contains_metric_depth": True,
            "contains_segmentation": True,
        }

    def save_png(self, file: str | Path) -> str:
        """Save actual 8-bit RGBA pixels to a standards-compliant PNG, no Pillow."""
        path = Path(file).resolve()
        if path.exists():
            raise FileExistsError(f"Refusing to overwrite image: {path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        def chunk(tag: bytes, payload: bytes) -> bytes:
            return (struct.pack("!I", len(payload)) + tag + payload +
                    struct.pack("!I", binascii.crc32(tag + payload) & 0xffffffff))
        rows = bytearray()
        width_bytes = self.width * 4
        for y in range(self.height):
            rows.extend(b"\x00")  # PNG scanline filter: None
            rows.extend(self.rgba[y*width_bytes:(y+1)*width_bytes])
        png = (
            b"\x89PNG\r\n\x1a\n" +
            chunk(b"IHDR", struct.pack("!2I5B", self.width, self.height, 8, 6, 0, 0, 0)) +
            chunk(b"IDAT", zlib.compress(bytes(rows), 6)) +
            chunk(b"IEND", b"")
        )
        path.write_bytes(png)
        return str(path)

    def save_depth_npz(self, file: str | Path) -> str:
        """Preserve measured linear-depth array and segmentation in NumPy NPZ."""
        try:
            import numpy as np
        except ImportError as exc:
            raise RuntimeError("NumPy required to save metric depth arrays") from exc
        path = Path(file).resolve()
        if path.exists():
            raise FileExistsError(f"Refusing to overwrite depth archive: {path}")
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            depth_m=np.array(self.metric_depth_m, dtype=np.float32).reshape(self.height, self.width),
            segmentation=np.array(self.segmentation, dtype=np.int32).reshape(self.height, self.width),
            renderer=np.array(self.renderer),
            near_m=np.float32(self.near_m),
            far_m=np.float32(self.far_m),
        )
        return str(path)


class BulletSensors:
    def __init__(self, world: BulletPlayground):
        self.world = world

    def lidar_2d(
        self, *, origin_m: Sequence[float] = (0.,0.,1.),
        count: int = 72, max_range_m: float = 5.0,
        start_angle_rad: float = 0.0,
    ) -> dict:
        """Idealized horizontal 360-degree geometric LiDAR.

        Real Bullet ray hits, not a physical optical sensor or calibrated
        scanner. Hit classification is by Bullet body ID only.
        """
        origin = _finite_xyz(origin_m, "origin_m")
        if type(count) is not int or not 4 <= count <= 360:
            raise ValueError("LiDAR count must be an integer in [4,360]")
        if (not isinstance(max_range_m, (float,int)) or
                not math.isfinite(max_range_m) or not 0.1 <= max_range_m <= 50.):
            raise ValueError("LiDAR max_range_m must be between 0.1 and 50m")
        if not math.isfinite(start_angle_rad):
            raise ValueError("LiDAR start angle must be finite")
        origins = [origin] * count
        endpoints = []
        for i in range(count):
            angle = start_angle_rad + 2 * math.pi * i / count
            endpoints.append((
                origin[0] + max_range_m * math.cos(angle),
                origin[1] + max_range_m * math.sin(angle),
                origin[2],
            ))
        contacts = self.world.p.rayTestBatch(
            origins, endpoints, physicsClientId=self.world.client,
        )
        measurements = []
        for i, hit in enumerate(contacts):
            body_id = int(hit[0])
            hit_fraction = float(hit[2])
            if not 0. <= hit_fraction <= 1.:
                raise RuntimeError("Bullet ray cast returned invalid hit fraction")
            measurements.append({
                "angle_rad": start_angle_rad + 2 * math.pi * i / count,
                "range_m": float(max_range_m * hit_fraction),
                "hit": body_id >= 0,
                "body_id": body_id,
                "hit_position_m": list(hit[3]) if body_id >= 0 else None,
                "hit_normal": list(hit[4]) if body_id >= 0 else None,
            })
        return {
            "sensor_type": "BULLET_IDEAL_GEOMETRIC_LIDAR_2D",
            "origin_m": list(origin),
            "max_range_m": max_range_m,
            "scan_count": count,
            "timestamp_sim_seconds": self.world.step_index / 240.,
            "measurements": measurements,
            "calibrated": False,
            "noise_model": "none",
        }

    def camera_rgbd(
        self, *, eye_m: Sequence[float] = (2.8,-3.0,1.9),
        look_at_m: Sequence[float] = (0.,0.,0.5),
        width: int = 160, height: int = 120,
        near_m: float = .05, far_m: float = 20.,
        fov_degrees: float = 60.,
    ) -> CameraFrame:
        """TinyRenderer CPU RGB, metric depth and body segmentation arrays."""
        eye = _finite_xyz(eye_m, "eye_m")
        target = _finite_xyz(look_at_m, "look_at_m")
        if math.dist(eye, target) <= .01:
            raise ValueError("Camera eye and target must be distinct")
        if type(width) is not int or type(height) is not int:
            raise ValueError("Image dimensions must be integer")
        if not 32 <= width <= 640 or not 32 <= height <= 480 or width*height > 200000:
            raise ValueError("Camera resolution exceeds constrained CPU sensor budget")
        if not (math.isfinite(near_m) and math.isfinite(far_m)
                and 0.01 <= near_m < far_m <= 100.0):
            raise ValueError("Invalid near/far clipping planes")
        if not math.isfinite(fov_degrees) or not 15. <= fov_degrees <= 120.:
            raise ValueError("Camera FOV must be in [15,120] degrees")
        p = self.world.p
        view = p.computeViewMatrix(eye, target, [0.,0.,1.])
        proj = p.computeProjectionMatrixFOV(
            fov=fov_degrees, aspect=width/height,
            nearVal=near_m, farVal=far_m,
        )
        data = p.getCameraImage(
            width, height,
            viewMatrix=view, projectionMatrix=proj,
            renderer=p.ER_TINY_RENDERER,
            physicsClientId=self.world.client,
        )
        if int(data[0]) != width or int(data[1]) != height:
            raise RuntimeError("Bullet camera dimensions do not match request")
        rgba = data[2].tobytes() if hasattr(data[2], "tobytes") else bytes(data[2])
        if len(rgba) != width * height * 4:
            raise RuntimeError("Unexpected Bullet RGBA buffer length")
        depth = data[3]
        seg = data[4]
        import numpy as np
        raw = np.asarray(depth, dtype=np.float64).reshape(-1)
        segmentation = tuple(int(x) for x in np.asarray(seg, dtype=np.int32).reshape(-1))
        if len(raw) != width * height or len(segmentation) != width * height:
            raise RuntimeError("Bullet depth/segmentation buffers have wrong size")
        metric = far_m*near_m / (far_m - (far_m-near_m)*raw)
        if not np.isfinite(metric).all():
            raise RuntimeError("Bullet depth image contains nonfinite values")
        return CameraFrame(
            width, height, rgba, tuple(float(x) for x in metric),
            segmentation, near_m, far_m,
        )

    def robot_link_velocity(self, link_index: int = 5) -> dict:
        """Actual Bullet link kinematics, not a calibrated inertial measurement."""
        if type(link_index) is not int or not 0 <= link_index < 6:
            raise ValueError("Robot link index must be in [0,5]")
        p = self.world.p
        values = p.getLinkState(
            self.world.robot, link_index,
            computeLinkVelocity=True,
            computeForwardKinematics=True,
            physicsClientId=self.world.client,
        )
        return {
            "sensor_type": "BULLET_LINK_KINEMATICS",
            "link_index": link_index,
            "linear_velocity_m_s": [float(x) for x in values[6]],
            "angular_velocity_rad_s": [float(x) for x in values[7]],
            "timestamp_sim_seconds": self.world.step_index / 240.,
            "calibrated": False,
            "note": "Link velocity is not a calibrated IMU with acceleration/bias/noise.",
        }
