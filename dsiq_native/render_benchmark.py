"""Measured CPU viewport/render-workload profiling across the real 3D presets.

No generated GPU/RTX frame times: all timings are measured Bullet
TinyRenderer CPU on the machine executing the command. Not a
photorealistic quality evaluation or portable FPS guarantee.
"""
from __future__ import annotations

import hashlib
import math
import statistics
import time
from dataclasses import dataclass

from .bullet_runtime import BulletPlayground,BulletSettings
from .environments import PRESETS,ReferenceEnvironment
from .sensors import BulletSensors


def benchmark_render_presets(*,frames: int = 3,width: int = 160,
                             height: int = 120, presets: tuple[str,...] = (
                                 "robotics_cell","cleanroom_bay","logistics_bay"
                             )) -> dict:
    if type(frames) is not int or not 1<=frames<=12:
        raise ValueError("frames must be 1..12")
    if (type(width) is not int or type(height) is not int or
        not 64<=width<=320 or not 48<=height<=240 or width*height>76800):
        raise ValueError("Viewport profiling requires <= 76,800 pixels")
    if (not isinstance(presets,tuple) or not 1<=len(presets)<=len(PRESETS) or
        len(set(presets))!=len(presets) or
        any(x not in PRESETS for x in presets)):
        raise ValueError("Presets must be distinct known reference environments")
    reports=[]
    for preset in presets:
        with BulletPlayground(BulletSettings(payload_count=1)) as world:
            environment=ReferenceEnvironment(world)
            scene=environment.build(preset)
            sensors=BulletSensors(world)
            captures=[]
            for index in range(frames):
                step_start=time.perf_counter()
                world.step(4)
                solve_ms=(time.perf_counter()-step_start)*1000
                t0=time.perf_counter()
                image=sensors.camera_rgbd(width=width,height=height)
                render_ms=(time.perf_counter()-t0)*1000
                rgb=image.rgba
                if len(rgb)!=width*height*4:
                    raise RuntimeError("CPU renderer produced invalid pixel buffer")
                colors=len({
                    rgb[i:i+3] for i in range(0,len(rgb),4)
                })
                if not 1<=colors<=width*height:
                    raise RuntimeError("CPU image color statistics invalid")
                captures.append({
                    "index":index,"physics_4_steps_ms":solve_ms,
                    "renderer_frame_ms":render_ms,
                    "rgba_sha256":hashlib.sha256(rgb).hexdigest(),
                    "distinct_rgb_colors":colors,
                    "depth_min_m":min(image.metric_depth_m),
                    "depth_max_m":max(image.metric_depth_m),
                })
            render_sorted=sorted(x["renderer_frame_ms"] for x in captures)
            median=statistics.median(render_sorted)
            p95=render_sorted[math.ceil(.95*len(render_sorted))-1]
            reports.append({
                "preset":preset,
                "environment":scene,
                "captured_frames":len(captures),
                "image_width":width,"image_height":height,
                "render_p50_ms":median,
                "render_p95_ms":p95,
                "observed_median_frame_rate_hz":1000./median if median>0 else None,
                "frames":captures,
                "renderer":"BULLET_TINY_RENDERER_CPU",
                "gpu_allocations_measured":False,
                "photorealistic":False,
            })
    return {
        "schema":"dsiq-environment-render-benchmark-v1",
        "engine":"pybullet_cpu",
        "renderer":"BULLET_TINY_RENDERER_CPU",
        "reports":reports,
        "hardware_specific_measurement":True,
        "fps_guaranteed":False,
        "rtx_gpu_fps":None,
        "qualifies_rtx3050":False,
        "note":"Measured current runner CPU-only timings; no GPU/RTX qualification.",
    }
