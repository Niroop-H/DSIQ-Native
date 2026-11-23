# DSIQ Native — Auditable OpenUSD Engineering Workcells

**Devashri S Industries · MAH Quantum**

**Status:** Foundation / pre-alpha, not industrial-safety certified. DSIQ Native
is an independent, Windows-first, **vendor-neutral** toolchain for source-
verified 3D machine assets, OpenUSD scene composition, measurable reference
physics and a lightweight native workcell editor. It is **not yet equivalent**
to NVIDIA Omniverse, Isaac Sim or Isaac Lab. See the
[qualification gates](docs/100_POINT_QUALIFICATION_PLAN.md).

## Start in five minutes (Python 3.11)

Windows PowerShell:

```powershell
git clone https://github.com/Niroop-H/DSIQ-Native.git
cd DSIQ-Native
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".[cad]"
python -m dsiq_native hardware
python -m dsiq_native machine-create build/reference-gantry --name demo_gantry
python -m dsiq_native machine-verify build/reference-gantry
python -m dsiq_native physics-backend-check
python -m dsiq_native new-stage build/first-scene.usda
python -m dsiq_native inspect build/first-scene.usda
```

The gantry above is a **self-authored engineering reference fixture**, not
an OEM-calibrated production machine. The backend check runs measured Bullet
physics in two separate worlds; it does not assess a real robot.

### Launch the native OpenGL editor

On a workstation with a working OpenGL driver:

```powershell
python -m pip install -e ".[gpu]"
python -m dsiq_native gpu-studio --manifest build/reference-gantry/machine.manifest.json
```

This is a real Panda3D raster viewport and Bullet simulation. It is **not**
a Vulkan Hydra delegate, RTX path tracer or a GPU-accelerated physics solver.
The [real UI video evidence](https://github.com/Niroop-H/DSIQ-Native/actions/workflows/ui-video.yml)
uses Mesa software OpenGL in CI, not an RTX3050 performance test. The optional
Kit extension is an interoperability adapter, not the main runtime.

## Working capability vs. evidence still needed

| Area | Implemented baseline | Explicit gaps |
| --- | --- | --- |
| Source geometry | Rights/hash-gated URDF, USD, source meshes; real CAD OpenGL draw path | OEM-scale CAD translators, licensed machinery data, calibrated fidelity |
| OpenUSD | Metric Z-up stages, references, layers, variants, measured telemetry; read-only reuse profile | Payload streaming, general-purpose asset rule plugins, Hydra rendering |
| Robotics | Bullet CPU physics, imported URDF actuators, bounded quintic motion, discrete collision checks, solver telemetry | Independent safety, CCD, OEM drive calibration, industrial control |
| Physics backends | Source-verified real Bullet and optional real MuJoCo CPU adapters; isolated-world conformance and measured source-matched comparison | Newton, PhysX, full USD Physics import, physical calibration, GPU batches |
| Editor | Native OpenGL/Panda3D window, hierarchy, move, undo/redo, 1/5/25 mm edit-grid snapping | Scalable PBR, mechanical mates/gizmos, RTX3050 hardware qualification |
| Sensors/data | Bullet RGB-D/LiDAR, actual link-derived ideal IMU-like kinematics and ROS 2 read-only transport | Calibrated optics/IMU bias, uncertainty, synchronized real hardware |
| Supply chain | Secure dependency audit, offline signed-update verification, SBOM, CI preview binaries | Independent licence review, publisher signing, validated installers |
| Industrial acceptance | Evidence inventory and qualification checklists | External machine calibration, risk assessment, customer pilot |

**Engineering focus:** one verified workcell, an open library-first pipeline
(import → validate → simulate → render → share), and reproducible benchmarks
on the target **16 GiB RAM / RTX 3050 (4 GiB VRAM)** budget. Proprietary
asset ownership and external commissioning cannot be generated from code.
The prior 42/100 figure is only a provisional assessment, not a new score.

## Parallel full-feature engineering program

**Program target:** comprehensive Omniverse-class capability coverage, not a limited-feature niche. Parallel workstreams cover OpenUSD, renderer, physics, robotics, Physical AI, calibrated sensors, synthetic data, CAD, materials, UI, plugins, collaboration, streaming, operational digital twins, automation, optimisation and enterprise distribution. The current platform is not yet at feature parity.

[View the 20-track master implementation and acceptance matrix](docs/OMNIVERSE_FULL_PARITY_MASTER_MATRIX.md).

[Read the independently executed 240 Hz Bullet/MuJoCo motor-trace evidence and strict 10 mm acceptance gates](docs/2026_11_SOLVER_MOTION_ACCEPTANCE.md).

To run the optional **real MuJoCo-vs-Bullet** CPU comparison (not a safety or cross-solver equivalence certificate):

```powershell
python -m pip install -e ".[mujoco]"
python -m dsiq_native physics-backend-check --backend mujoco_cpu_direct_v1
python -m dsiq_native physics-cross-backend-check --tolerance 0.01
```

The **10 mm final-position** screening threshold is a software comparison gate for the self-authored reference gantry, **not an OEM actuator tolerance**. The latest source-bounded 240 Hz trace measured 0.255 mm final disagreement, 9.41 mm maximum sampled position disagreement, and both solvers within the source effort and speed envelopes for this case. The controllers remain non-OEM. Use `physics-motion-trace --steps 100 --sample-every 5 --tolerance .01 --require-match` to reproduce the measured acceptance. Different simulation/control laws can produce discrepancies on other machines.

To validate a real authored OpenUSD stage before loading heavy payloads:

```powershell
python -m dsiq_native usd-validate build/first-scene.usda
```

Validation is bounded, read-only and checks default-time composed geometry; deferred payload contents, external rights and independent physical fidelity require further checks.

## OpenUSD portable dependency and reference sensor checks

```powershell
python -m dsiq_native usd-dependency-audit build/first-scene.usda --asset-root build
python -m dsiq_native imu-sample --steps 24 --link 5
```

The dependency auditor uses genuine Pixar OpenUSD dependency resolution and records approved local file hashes. It does **not** sandbox external USD resolver plug-ins, does not validate USDZ internal contents and cannot prove asset ownership. The inertial probe reads real Bullet link kinematics and finite differences across physics steps; it is **not** a calibrated hardware IMU.

See [USD dependency audit](docs/USD_DEPENDENCY_AUDIT.md) and [CPU inertial probe](docs/SENSOR_INERTIAL_REFERENCE.md).

## Large-scene OpenGL diagnostics — genuine frames, no fake factory physics

```powershell
python -m pip install -e ".[gpu]"
python -m dsiq_native gpu-instancing-benchmark --copies 4 --frames 120 --screenshot four-robots.png
python -m dsiq_native gpu-visual-load-benchmark --copies 300 --frames 120 --screenshot visual-stress.png
```

The first benchmark tests shared immutable render buffers across **four independently simulated, source-verified Bullet gantries** (the supported workcell limit). The second draws **301 gantry visuals** but runs **only one real Bullet body**—300 are explicitly frozen graphics-only source references for raster-performance testing. The latter processes 111,972 source triangle instances; they are repeated, not unique OEM industrial CAD. Both report real OpenGL renderer, FPS, p50/p95/p99 timings and resource reuse; neither claims measured per-process VRAM savings or RTX/Hydra/Vulkan implementation.

See [exact rendering resource reuse and limits](docs/NATIVE_OPENGL_GEOMETRY_REUSE.md). Hardware-specific acceptance on a real RTX 3050 remains outstanding.

## Developer checks

```powershell
python -m unittest discover -s tests -v
python -m dsiq_native physics-verify
python -m dsiq_native physics-backend-check
python -m dsiq_native core-readiness
python -m dsiq_native usd-scene-profile build/first-scene.usda
```

Some integration tests need optional dependencies, Xvfb/ROS 2, or a graphics
device. CI separates these gates so skipped optional tests cannot be mistaken
for complete hardware qualification.

## Project documents

- [Phase-by-phase Omniverse-competition execution plan](docs/OMNIVERSE_COMPETITION_EXECUTION_PLAN.md): the roadmap, exit criteria and architectural tradeoffs.
- [Industrial 100-point qualification gates](docs/100_POINT_QUALIFICATION_PLAN.md): external acceptance evidence and eight blocked categories.
- [Physics contract and conformance](docs/PHYSICS_BACKEND_CONTRACT.md): the current Bullet adapter and criteria for adding new solvers.
- [OpenUSD workspaces](docs/USD_COMPOSITION.md), [GPU CAD editor](docs/NATIVE_GPU_CAD.md), [real robot motion](docs/URDF_ROBOTS.md), [ROS 2 bridge](docs/ROS2_BRIDGE.md).
- [Previous full README / development milestones](docs/README_HISTORY_2026.md): preserved unabridged; useful for historical experiments.

## Distribution and contribution status

No top-level `LICENSE` grant has been selected by the owner yet.
**Do not assume this repository is open-source or that its code/assets can
be copied or redistributed.** A deliberate owner licence decision, dependency
licence review and third-party notices must precede commercial distribution.
Consult [CONTRIBUTING.md](CONTRIBUTING.md) before proposing material changes.

**Repository scope:** `Niroop-H/DSIQ-Native` only. The separate DSIQ
Domain Expansion Web Edition is not modified by work in this repository.
