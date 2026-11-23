# DSIQ Native — Comprehensive Omniverse-Class Capability Program

**Program instruction:** pursue *every major NVIDIA Omniverse capability family*, across parallel engineering tracks. This updates the earlier **narrow-wedge** strategic advice; auditability and vendor portability are **advantages**, not a ceiling on functionality. The documented [source review](OMNIVERSE_COMPETITION_EXECUTION_PLAN.md) remains useful for technical sequencing, acceptance methods and independent research.

**Reality:** the table describes a forward engineering target. It does not state current equivalence to NVIDIA software or that future GPU performance is already achieved.

## Program-wide architecture

- **OpenUSD scene authority:** composed layers, references, payloads, variants and stable asset provenance, with read-only validation before any import or optimization.
- **Independent backend interfaces:** scene/physics/renderer/sensors/services; no secretly substituting approximations when a user requests a different solver.
- **One native shell, many integrations:** fast standalone Windows app and headless engine, plus optional web review, Kit bridge and plugins.
- **Measured acceptance:** physically meaningful tests, matched hardware/scene comparisons, security and licensing, clear provenance and fidelity labels.
- **Flexible hardware:** 4 GiB VRAM/16 GiB RAM budget is an important entry tier, but advanced GPUs and multi-GPU servers must be future supported performance tiers.

## Parallel workstream matrix

| Feature family | Verified / bounded implementation | Full program target and next independently testable milestone |
| --- | --- | --- |
| 1. OpenUSD core | Stage, layers, references, variants, telemetry, scene profile | Payload streaming, composition correctness, purpose/visibility, instancing, collections, schema support; large multi-layer fixture |
| 2. Asset validation and provenance | Source hash/rights checks, read-only USD mesh/physics/material validator and resolved local dependency-hash gate | Versioned pluggable validation rules, missing-asset path checks, remote namespace resolver; adversarial USD corpus |
| 3. High-end renderer | Real Panda3D OpenGL editor/TinyRenderer CPU; exact source buffer reuse; separately verified 4-robot render and 301-visual, one-robot stress fixture with actual GL timing/p99; not qualified hardware | Hydra/Vulkan raster PBR, progressive LOD/streaming, high-quality materials/shadows, ray/path tracing, physically measured device throughput |
| 4. Physics engines | C++ Bullet via PyBullet and optional real MuJoCo CPU adapter; independent solver reports | Newton, PhysX, GPU-batched solver, deformables/particles, stable friction/contact parity; matched numerical benchmarks |
| 5. Robotics and articulated machines | Source-verified URDF and real CPU motor actuation, preflight limits, motion analysis | Collision-checked articulated planning, robot controllers and hardware-safe interfaces; real OEM measurements |
| 6. Physical AI | Seeded Bullet reference policy and basic randomized environments | GPU/vector RL, imitation learning, policy export, task learning, sim-to-real benchmarks with held-out calibration |
| 7. Sensors | Bullet RGB-D, idealized LiDAR, link-COM inertial probe with actual solver velocity and finite-difference force, read-only ROS 2 transport | Render-consistent cameras, radar, LiDAR, IMU and calibration/noise/time-sync; reproducible sensor residuals |
| 8. Synthetic data | Seeded CPU generator, COCO export and SHA checks | Camera/depth/normals/semantic instance AOVs, material and lighting randomization graph, KITTI/YOLO/BOP export |
| 9. CAD / interop | Rights-checked URDF, geometry meshes and USD | STEP/IGES kernel, Blender/glTF/FBX, authoring constraints, CAD tessellation accuracy and assembly round trip |
| 10. Material and lighting | Basic native GPU-lit source geometry | USDShade/MaterialX, calibrated PBR, texture pipeline, IBL and consistent renderer output |
| 11. Native authoring UI | Actual editor hierarchy, move, undo/redo, metric grid snap | Gizmos, parametric constraints/mates, layers, timeline, inspector, multiselection, docks, reparenting |
| 12. Extension platform | Python package/CLI and optional Kit extension | Documented plug-in lifecycle, versioned SDK, sandbox/permission contract and extension marketplace |
| 13. Collaboration | Local layered workspaces | Live two-user layer delta sync, conflict resolution, access policy, branch/session history and versioned storage |
| 14. Streaming / thin clients | Existing native desktop and CLI | Remote interactive render sessions, authenticated WebRTC/WebGPU review, headless session lifecycle |
| 15. Industrial digital twins | Evidence-qualified workcell schemas and source motion recording | OPC UA/MQTT, calibrated machine-specific HMI/process models, time-aligned telemetry, stale data handling |
| 16. AI agents and scene workflows | Python APIs available for future automation | Constrained MCP tools, explainable scene authoring, safe execution approvals and reproducible agent benchmarks |
| 17. System optimisation | Measured OpenGL p50/p95/p99 wall frames, real-source Panda3D vertex/index buffer deduplication, reference-only CAD stress harness and read-only USD reuse profiler | USD deferred loading, process GPU VRAM/RAM attribution, culling, LOD, multigpu portability, back-pressure and matched target-hardware FPS/VRAM/RAM |
| 18. Enterprise/security/release | SBOM, trust-gated updates, unsigned Windows previews, vulnerability audit | Signed production releases, third-party licence clearance, secure asset services, identity, audit and support |
| 19. SimReady / content fidelity | Reference gantry, environment fixtures, source import | Real licensed multi-machine content library with calibrated materials/colliders, provenance and external acceptance |
| 20. Multi-device computing | CPU baseline, one OpenGL graphics path | Vulkan-class AMD/Intel/NVIDIA GPU compatibility, RTX accelerated path and HPC scale; device matrix verified on hardware |

No entry above should be interpreted as claiming every NVIDIA implementation detail is public, reproducible, legally reusable or technically achievable on the RTX3050 minimum configuration.

## Development policy: parallel builds, gated integration

All workstreams may build prototypes concurrently and publish testable branches of knowledge. **Repository implementation remains on `main` as instructed.** A capability only becomes an "implemented" row after an actual integration test demonstrates it. Production-grade and physically qualified statuses require separate acceptance.

Milestone gate A: source validation, real MuJoCo/Bullet, metric grid authoring and automated full UI video evidence.

Milestone gate B: USD layer source dependency/payload rule plugin and a large-stage native memory benchmark.

Milestone gate C: a genuine Vulkan/Hydra render proof with captured GPU identification and stress tests on the desired RTX3050.

Milestone gate D: solver expansion, sensor AOVs, ROS 2 and reproducible AI benchmark; preserve control-safety separation.

Milestone gate E: collaborative storage, streaming, hardware/process bindings, independent industrial pilot, security/licensing and signed releases.

We develop all categories, but do not remove passing existing functionality to display incomplete features as shipped. Benchmarks and numerical residuals must include failures. **There is no claimed NVIDIA feature parity or newly awarded industrial qualification score as of this document.**
