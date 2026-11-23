# DSIQ Native — Platform Readiness and Release Gates

**Date:** 2026-11-23. **Current product stage:** pre-alpha/reference engineering.
The goal is to compete through independently validated professional
workcells—not by claiming 100% precision merely because a demo renders.

Each software feature must have reproducible tests, and each real machine
must have external reference data and engineering sign-off. No source
file, successful CI test, or self-declared evidence automatically grants
industrial qualification.

| Engineering area | Newest implemented verification | Hard release blockers |
|---|---|---|
| Physics | Real Bullet contact, friction, energy/momentum, timestep experiments, sampled collision screening | Independent material/contact/torque calibration, swept CCD and other domain physics |
| Robotics | 6R FK/IK, verified general URDF motors, multi-axis trajectories, collision preflight | Operational torque/acceleration qualification, self-collision, dynamic safety, OEM robot calibration |
| OpenUSD | Metric layers/references/variants, real link transform telemetry, temporal data readback | Full CAD geometry/pose binding, robust large-scene asset portability, manufacturer source qualification |
| Sensors | Real Bullet CPU image/depth/segmentation and geometric LiDAR | Physical optical noise/bias calibration, synchronized realistic camera/range sensing |
| ROS 2 | Read-only JointState, LaserScan, Clock and now measured parent-relative TF contract; optional live publisher | Real DDS interoperability, controller tests, validated URDF/TF frame matching |
| Machine development | Editable physical 3-axis gantry, generic rights-verified source URDF parts and inertia inventory | Large licensed OEM CAD/machine library, realistic actuators, measured masses, bill-of-materials provenance |
| Environments | Physics-colliding robotics/cleanroom/logistics reference scenes and USD material metadata | Industrial facility measured layouts, real detailed HVAC/safety, scalable scene management |
| Graphics | Tk live CPU renderer, free camera, object picking; CPU frame/FPS/physics benchmark | Hardware-qualified GPU real-time PBR/RTX renderer with textures, streaming, denoising |
| Physical AI | Bounded seeded Bullet dataset capture, actual-segmentation COCO detection bbox export | Robot learning and RL engine, policy evaluation, calibrated training distributions, real domain tests |
| Commercial | CI-built unsigned Windows portable standalone CPU executable with frozen simulation smoke test | Signed installer and Store review, reproducible SBOM, security assessment, GPU profiling, customer pilots |

## Cross-platform verification

Run the repository's independent Python and platform tests, including:

    python -m dsiq_native core-readiness
    python -m dsiq_native physics-extended
    python -m dsiq_native physics-conservation
    python -m dsiq_native render-benchmark --frames 2 --width 64 --height 48
    python -m dsiq_native ros2-preview --lidar-rays 24
    python -m dsiq_native machine-inventory PATH_TO_MACHINE_MANIFEST.json
    python -m dsiq_native dataset-generate sample-data --frames 4 --width 64 --height 48
    python -m dsiq_native dataset-coco sample-data sample-coco.json

The dataset exporter verifies original image/depth hashes, extracts
bounding boxes directly from actual Bullet segmentation pixels and
performs deterministic train/validation splitting. It does NOT have a
training framework or a scientifically calibrated physical sensor.

## Windows portable preview

The Windows CI job installs PyInstaller, packages the CPU robotics
command-line application into a directory-based Windows executable,
and starts that executable to verify robot forward kinematics and
a real Bullet physics run. It creates a zip and SHA-256 digest as
short-lived GitHub Actions artifacts.

Limitations: unsigned preview; no Windows code signing/MSIX, installer,
auto-update, licensing/anti-tamper verification, ROS 2 runtime, NVIDIA
Kit/RTX or guaranteed full Pixar USD integration in the frozen image.
Production redistribution requires licensing review of each bundled
dependency and suitable legal/security approval.

**Source repository is not a calibrated industrial design package.**
Do not deploy the reference robots in an operational safety system.

## Practical next engineering priorities

1. Model-accurate, continuous collision and dynamics qualification
   for verified articulated robot families and licensed machine CAD.
2. Real native GPU renderer implementation and measured RTX 3050 VRAM
   and latency, with fallback to CPU mode.
3. Published ROS 2 end-to-end middleware tests and URDF/TF frame validation.
4. Physically calibrated sensor and domain-randomization benchmarks.
5. Additional workcell families with exact provenance and per-model
   independent CAD, safety, power and performance acceptance.
6. Industrial distributions with signing, update channels, SBOM,
   vulnerability scanning, documented license compliance and pilots.

**A completed task is represented as IMPLEMENTED + TESTED within its
declared scope, not COMPLETE FOR ALL INDUSTRIAL USE CASES.**
