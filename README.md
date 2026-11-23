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
| Physics backends | Source-verified Bullet world adapter, conformance contract and repeatability evidence | MuJoCo, Newton, PhysX backends, cross-solver comparisons, GPU batches |
| Editor | Native real OpenGL/Panda3D window, selection, hierarchy, move, undo/redo | Scalable PBR renderer, mechanical mates/gizmos, 3050 hardware qualification |
| Sensors/data | Bullet RGB-D/LiDAR reference and ROS 2 read-only transport | Calibrated optics, sensor uncertainty, safely commissioned controllers |
| Supply chain | Secure dependency audit, offline signed-update verification, SBOM, CI preview binaries | Independent licence review, publisher signing, validated installers |
| Industrial acceptance | Evidence inventory and qualification checklists | External machine calibration, risk assessment, customer pilot |

**Engineering focus:** one verified workcell, an open library-first pipeline
(import → validate → simulate → render → share), and reproducible benchmarks
on the target **16 GiB RAM / RTX 3050 (4 GiB VRAM)** budget. Proprietary
asset ownership and external commissioning cannot be generated from code.
The prior 42/100 figure is only a provisional assessment, not a new score.

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
