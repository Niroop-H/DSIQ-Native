# DSIQ Native — Exact source-geometry GPU buffer sharing

**Implementation:** `dsiq_native/render_geometry_cache.py` and `dsiq_native/gpu_studio.py`. **Status:** Actual Panda3D OpenGL rendering, verified independent Bullet robots; **not** a Hydra, Vulkan, RTX, physical machine, or industrial performance certification.

## What has changed

The previous native editor created a new full Panda3D vertex/index buffer for *every* CAD visual of *every* imported robot, even when several independent machines had precisely identical visual geometry.

Now, an imported CAD visual is eligible for **immutable GPU draw-geometry buffer sharing** based on a SHA-256 content fingerprint of the actual **post-URDF visual-transform vertices**, triangle topology/order, and RGBA material values. Names and machine asset IDs do not affect render identity; differences of vertex coordinates, triangle indices/order or RGBA prevent sharing.

Each imported machine retains a distinct source-verified URDF manifest, separate original Bullet rigid/articulated body, own link nodes and parent transforms. No part is removed, no geometry is simplified or merged, and no simulation state is reused. A distinct `GeomNode` is created for each rendered visual, referring to the same underlying immutable Panda3D `Geom` when its exact content matches an existing buffer.

## Test on a real display or runner

Install graphics and robotics dependencies:

```powershell
python -m pip install -e ".[gpu]"
python -m dsiq_native gpu-instancing-benchmark --copies 6 --frames 120 --width 1280 --height 720 --screenshot instanced-source-gantries.png > instancing-report.json
```

A successful report is derived from actual GUI-rasterized OpenGL frames. It contains source-authored gantry instances (not OEM equipment), distinct imported Bullet body count, source triangle instances, number of unique retained `Geom` buffers, reused triangle instances, actual frame timing/FPS, and OpenGL driver where available. If hardware permits, the existing `gpu-benchmark --require-nvidia --min-fps 30` remains the separate genuine on-device test.

**CI:** `.github/workflows/ci.yml` runs a Mesa/Xvfb scene containing six separate validated reference gantries, asserts six **distinct Bullet bodies**, verifies precise GPU buffer reuse and saves a real framebuffer capture and JSON timing report in the `dsiq-native-opengl-software-ci` artifact.

## What metrics mean

- `total_authored_triangle_instances`: actual triangles authored in the live render graph, including all machines.
- `unique_geom_buffer_triangles`: triangle content stored only once in our Panda3D Geom object cache.
- `reused_triangle_instances`: the difference between total draw instances and unique cached buffer content, **not** a count of triangles culled or removed.
- `unique_geom_buffer_objects`: number of unique content hashes held in the current editor session.
- `verified_source_visual_nodes`: count of independent rendered CAD visual parts, whether or not a buffer is shared.
- `observed_fps`, `interframe_p50_ms`, `interframe_p95_ms`: measured times of **this renderer/session**, not a fabricated RTX card specification.

**Not yet measured:** process-specific GPU VRAM saved, GPU vertex residency, draw calls reduced, multi-GPU performance, real high-density factory CAD rendering or RTX3050 4GiB physical hardware performance. Panda3D may make driver-level copies that are not reported by this application. Buffer sharing alone does **not** implement scene batching or frustum/occlusion culling.

**Security/accuracy caveat:** SHA-256 identifies precisely matching render content, not lawful usage rights or independent OEM dimensional/physical accuracy. Imported source geometry is validated and rights-labelled by the existing URDF pipeline *before* any GPU buffer is created.

## Next rendering-engine acceptance work

1. Establish repeated measured per-process GPU memory probes on an actual RTX3050 and second vendor GPU.
2. Add spatial culling, payload-aware streaming, explicit LOD rules and lossless reauthoring boundaries.
3. Prototype an independent Vulkan/Hydra delegate with material, shadow and selection validation before replacing any working OpenGL editor.
4. Measure p50/p95/p99 render frame times on legal multi-hundred-thousand-triangle engineering assets, with driver and hardware identifiers.
5. Do not infer full Omniverse feature parity from repeated reference gantries or synthetic stress scenes.
