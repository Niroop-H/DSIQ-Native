# DSIQ Native — Exact source-geometry GPU buffer sharing

**Implementation:** `dsiq_native/render_geometry_cache.py` and `dsiq_native/gpu_studio.py`. **Status:** Actual Panda3D OpenGL rendering, verified independent Bullet robots; **not** a Hydra, Vulkan, RTX, physical machine, or industrial performance certification.

## What has changed

The previous native editor created a new full Panda3D vertex/index buffer for *every* CAD visual of *every* imported robot, even when several independent machines had precisely identical visual geometry.

Now, an imported CAD visual is eligible for **immutable GPU draw-geometry buffer sharing** based on a SHA-256 content fingerprint of the actual **post-URDF visual-transform vertices**, triangle topology/order, and RGBA material values. Names and machine asset IDs do not affect render identity; differences of vertex coordinates, triangle indices/order or RGBA prevent sharing.

Each imported machine retains a distinct source-verified URDF manifest, separate original Bullet rigid/articulated body, own link nodes and parent transforms. No part is removed, no geometry is simplified or merged, and no simulation state is reused. A distinct `GeomNode` is created for each rendered visual, referring to the same underlying immutable Panda3D `Geom` when its exact content matches an existing buffer.

The integrated Bullet workcell currently enforces **four imported robots maximum**. This benchmark deliberately honors that bound; exceeding it fails closed rather than quietly increasing the robot limit. Larger graphics-only scene scaling is a separate future stress-test target.

## Test on a real display or runner

Install graphics and robotics dependencies:

```powershell
python -m pip install -e ".[gpu]"
python -m dsiq_native gpu-instancing-benchmark --copies 4 --frames 120 --width 1280 --height 720 --screenshot instanced-source-gantries.png > instancing-report.json
```

A successful report is derived from actual GUI-rasterized OpenGL frames. It contains source-authored gantry instances (not OEM equipment), distinct imported Bullet body count, source triangle instances, number of unique retained `Geom` buffers, reused triangle instances, actual frame timing/FPS, and OpenGL driver where available. If hardware permits, the existing `gpu-benchmark --require-nvidia --min-fps 30` remains the separate genuine on-device test.

**CI:** `.github/workflows/ci.yml` runs a Mesa/Xvfb scene containing four separate validated reference gantries, asserts four **distinct Bullet bodies**, verifies precise GPU buffer reuse and saves a real framebuffer capture and JSON timing report in the `dsiq-native-opengl-software-ci` artifact.

## Separate large graphics-only stress path

The integrated real Bullet workcell remains capped at **four imported robots**. For larger rendering loads, DSIQ now offers a **distinct graphics-only benchmark** that creates many static visual references from one source-verified gantry at a captured real Bullet link pose:

```powershell
python -m dsiq_native gpu-visual-load-benchmark --copies 256 --frames 48 --width 1280 --height 720 --screenshot visual-load.png > gpu-visual-load.json
```

The above is a suggested on-device test, not a published FPS result. The new command supports **1–512 extra static CAD copies**, additionally bounded to 500,000 scene triangles. Its expanded Mesa software-OpenGL CI fixture uses **300 static visual copies** plus one real Bullet prototype: **301 visual assemblies, 111,972 source-authored triangle instances**, **one physics body**, no extra actuators. The CI gate requires an actual GL framebuffer screenshot, positive observed FPS, p50/p95/p99 wall-clock frame intervals and at least 100,000 rendered reference-triangle instances. These are low-complexity repeated gantry visuals, not a 100k-triangle unique industrial CAD assembly. Every static reference keeps a frozen snapshot of verified source-geometry link transforms and identical render data; it has no independent physical dynamics or sensors.

The benchmark reports actual graphics-frame timings, authored scene triangle instances, unique draw-geometry buffers, and clear separate counts for reference copies and actual Bullet bodies. This is a **graphics resource stress test**, not a simulated industrial production line.

## What metrics mean

- `total_authored_triangle_instances`: actual triangles authored in the live render graph, including all machines.
- `unique_geom_buffer_triangles`: triangle content stored only once in our Panda3D Geom object cache.
- `reused_triangle_instances`: the difference between total draw instances and unique cached buffer content, **not** a count of triangles culled or removed.
- `unique_geom_buffer_objects`: number of unique content hashes held in the current editor session.
- `verified_source_visual_nodes`: count of independent rendered CAD visual parts, whether or not a buffer is shared.
- `observed_fps`, `interframe_p50_ms`, `interframe_p95_ms`, `interframe_p99_ms`: actual measured frame loop intervals in **this renderer/session**, not GPU hardware timestamp queries or a fabricated RTX card specification. A short benchmark is not a long-run stability or 4GB VRAM qualification.

**Not yet measured:** process-specific GPU VRAM saved, GPU vertex residency, draw calls reduced, multi-GPU performance, real high-density factory CAD rendering or RTX3050 4GiB physical hardware performance. Panda3D may make driver-level copies that are not reported by this application. Buffer sharing alone does **not** implement scene batching or frustum/occlusion culling.

**Security/accuracy caveat:** SHA-256 identifies precisely matching render content, not lawful usage rights or independent OEM dimensional/physical accuracy. Imported source geometry is validated and rights-labelled by the existing URDF pipeline *before* any GPU buffer is created.

## Next rendering-engine acceptance work

1. Establish repeated measured per-process GPU memory probes on an actual RTX3050 and second vendor GPU.
2. Add spatial culling, payload-aware streaming, explicit LOD rules and lossless reauthoring boundaries.
3. Prototype an independent Vulkan/Hydra delegate with material, shadow and selection validation before replacing any working OpenGL editor.
4. Measure p50/p95/p99 render frame times on legal multi-hundred-thousand-triangle engineering assets, with driver and hardware identifiers.
5. Do not infer full Omniverse feature parity from repeated reference gantries or synthetic stress scenes.
