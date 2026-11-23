# DSIQ Native — Independent GPU CAD Workcell Studio

## What is actually implemented

DSIQ now has **two distinct render paths**:

1. The original Bullet TinyRenderer CPU viewport in the Tk engineering
   studio, suitable for headless sensor data and CPU-only systems.
2. An optional **Panda3D/OpenGL** native rasterized 3D CAD viewport
   with real shader lighting, a hierarchy, and linked Bullet simulation.

The GPU viewport is not NVIDIA Kit, PhysX, RTX ray tracing, material
path tracing, exact OEM geometry, or a qualified VR/AR environment.

### Run on Windows (Python development build)

    git pull origin main
    python -m pip install -e ".[gpu]"
    python -m dsiq_native gpu-studio

An empty CAD window opens. Choose **Import licensed URDF** or
**Open workcell** and select a rights-checked DSIQ manifest / project.
For a fully source-backed visual, the URDF may contain box, cylinder,
sphere, and local STL/OBJ/DAE mesh assets.

**Important:** every external visual mesh requires a SHA-256 lock in
the DSIQ manifest under geometry.mesh_sha256:

    python -m dsiq_native cad-mesh-lock path/to/machine.manifest.json

This command **only prints suggested hashes**; the user must review
license/source evidence and commit the hashes to the manifest explicitly.
Do not import an unknown manufacturer's CAD without permission.
Provenance strings and checksums are necessary, not sufficient proof of
manufacturer dimensional precision or rights.

### Editor controls

- Mouse left-drag: orbit the camera; mouse wheel: zoom.
- Right-click: select underlying **Bullet collision geometry** when
  the physical proxy matches an imported asset.
- Hierarchy panel: imported assets and URDF links, with source name and
  live position; click an asset to select it.
- Arrow keys: translate selected machine base X/Y in **25 mm increments**.
- Page Up / Page Down: Z translation; Ctrl+Z / Ctrl+Y: actual
  Bullet base-transform undo/redo.
- Space: pause/resume real Bullet simulation; Ctrl+O: import new URDF.
- Top buttons: import source CAD, load an engineering workcell, undo/redo.

Layout edits are **simulation authoring teleports**, not motor
commands. Do not treat them as collision-free robot trajectories.
The editor does not yet have mouse transform gizmos, detailed mechanical
mates/constraints, topology editing, PBR material authoring, scene
streaming, or photorealistic ray tracing.

### Source CAD to animated OpenUSD

    python -m dsiq_native cad-usd-record path/to/machine.manifest.json source-motion.usda --frames 12 --joint actuator_x --target 0.25

This pipeline reads the exact locked source mesh triangles and visual
geometry, applies URDF relative transforms and material color, then
authors actual Bullet link-world poses into time-sampled metric/Z-up
OpenUSD. Unsupported source geometry fails rather than being silently
substituted with a fake block.

Triangulation does not imply that source CAD has been independently
qualified or that rendering is visually identical to the OEM software.

### Real OpenGL performance evidence (not simulated FPS)

    python -m dsiq_native gpu-benchmark --frames 120 --width 1280 --height 720 --screenshot actual_gpu.png > gpu-report.json

Reports actual GL frame timing, viewport resolution, drawn triangles,
OpenGL driver identity where available, and nvidia-smi free/total VRAM
before/after where an NVIDIA device is present. VRAM samples cover
**global device usage**, not precisely the process's peak allocations.
A CI virtual-display Mesa benchmark validates GL operation but is NOT
proof of an RTX 3050 meeting local performance targets.

To require an observed minimum of 30 FPS on a verified NVIDIA graphics
path (both GPU identity and GL vendor):

    python -m dsiq_native gpu-benchmark --frames 120 --min-fps 30 --require-nvidia --manifest path/to/rights-checked-machine.manifest.json > local-RTX-report.json

For RTX 3050 qualification, test the actual intended 4/6/8 GiB card,
16 GiB RAM, current drivers, a demanding rights-cleared workcell, and
a long stability soak with independently measured process VRAM.
Do not infer these from hosted CI.

## Robotics learning, ROS and physical evidence

    python -m dsiq_native robot-vector-benchmark --envs 4 --steps 30
    python -m dsiq_native ros2-preview --lidar-rays 36
    python -m dsiq_native physical-compare bullet-report.json external-joint-observations.json --tolerance 0.005

The vector benchmark steps **separate real CPU Bullet clients** and
reports observed solver throughput, not simulated Isaac Lab GPU
vectorization. ROS2 Jazzy CI includes both captured-message DDS
delivery and a separate full running DSIQ ros2-stream publisher
subscription test. Neither validates physical robot control.

The physical comparator requires user-supplied instrument/equipment
identity, calibration reference, units, timestamps, and at least
three observations per joint. It will show residuals even if the
model performs poorly; it NEVER pretends the observations were
obtained independently or grants safety certification.

## Windows distribution and release gates

A separate **unsigned Windows GPU portable preview** is built using
PyInstaller on CI, along with a CPU-only preview. The GPU executable
has GL runtime dependencies; Windows CI verifies CLI operation, while
the Linux Xvfb Mesa job runs the native GPU graphics acceptance test.
GPU hardware qualification must run locally on supported GPUs.

    python -m dsiq_native release-sbom components.spdx.json
    python -m dsiq_native release-attest DSIQ-Native-GPU-Preview.zip gpu.attestation.json
    python -m dsiq_native release-verify DSIQ-Native-GPU-Preview.zip gpu.attestation.json

SPDX is an installed-Python-component inventory only. Neither checks
complete binary/DLL dependencies nor proves that all components are
licensed for distribution.

The Windows production gate is deliberately **fail closed**:

    pwsh -File scripts/windows_production_gate.ps1 -Executable .\DSIQ-Native-GPU.exe

An unsigned binary is blocked. A commercially released machine-safety
tool needs valid Authenticode signed executables/installers, an authorized
publisher identity, legal and security review, authenticated physical
validation, actual hardware benchmarks, and independent customer pilots.
No certificate, machine data, or endorsement is fabricated in this project.

## Remaining competitive blockers

- Continuous swept-collision certificate / qualified self collision and
  torque/load-limited planning with independent real-world validation.
- Mesh material UV/texture fidelity, accurate assembly mates and moving
  constraints, and OEM supplier digital twin content.
- GPU material lighting pipeline beyond raster Phong shading, real-time
  PBR, texture streaming, occlusion/LOD and validated large factory assets.
- RTX 3050 device-specific long-run FPS/VRAM/stability evidence.
- Physical robot ROS 2 control validation, sensor calibration,
  serious ML/RL policy training and real-world transfer benchmarks.
- Signed installer, complete security audit, approved licenses,
  commercial update channel and customer acceptance.

**Status: useful working OpenGL/CAD and physics integration prototype.
Not NVIDIA Omniverse/Isaac Sim parity or certified industrial software.**
