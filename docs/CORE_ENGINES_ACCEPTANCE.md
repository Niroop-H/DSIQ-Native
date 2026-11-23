# DSIQ Native — Core Physics, Imported Robotics and OpenUSD Readiness

**Release-blocking pillars:** Physics numerical behaviour, general imported
robot actuation, OpenUSD time-sampled interoperability. No engine can
truthfully be declared "100% complete" for every industry or physical regime.
This document states reproducible, finite acceptance gates for the
capabilities actually implemented.

## 1. Physics: conservation measurements

Run:

    python -m pip install -e ".[robotics]"
    python -m dsiq_native physics-verify
    python -m dsiq_native physics-extended
    python -m dsiq_native physics-conservation

The physics-conservation command performs TWO independent reference
experiments with the actual Bullet solver:

- Gravitational energy residual of an unforced, undamped, contact-free
  box at 120/240/480 Hz. It reports kinetic and gravitational potential
  energy in joules, absolute residual, relative error and whether
  residual decreases as the timestep is refined. Semi-implicit integration
  is NOT exactly energy conserving and is not advertised as such.
- Linear momentum conservation of a freely moving rigid mass with
  gravity, drag and contact removed. It reports measured momentum
  and absolute difference from the initial momentum.

These tests provide evidence about isolated reference systems only.
They DO NOT validate collision-contact accuracy, fluid/FEM physics,
industrial compliance, hardware safety or equivalence with PhysX.

## 2. General URDF multi-axis motion, not only our custom 6R

Every rights-checked imported URDF with supported revolute/prismatic
motors now supports a joint-space quintic path including all movable
joints, holding untargeted axes and using each actuator's declared
URDF velocity limit. It rejects unknown/fixed joints, out-of-range
goals, nonfinite values, inadequate speed and insufficient duration.
It reads the current measured Bullet state and aborts execution if
that start state has changed.

    python -m dsiq_native robot-urdf-motion machine/machine.manifest.json --target actuator_x=0.3 --target actuator_y=0.2 --record-every 12

Execution uses actual Bullet motor controls and records measured
feedback, requested state, max/p95 tracking error, travel-limit
exceedances and observed contact counts.

The CLI also fails closed if actual Bullet per-sample geometry screening
finds an obstacle along the requested joint trajectory. It snapshots
and restores the solver world before live execution, and reports
which sample obstructed the path. This is NOT continuous swept
collision detection and cannot certify motion safety between samples
or self-collisions without a qualified self-collision model.

LIMITATION: This is velocity-bounded reference motion, NOT a
collision-free, torque/acceleration qualified industrial trajectory.
Real robot acceleration, torque/load, swept collision, joint stiffness,
encoder calibration and physical controller acceptance remain necessary.

## 3. OpenUSD: actual solver animation and articulation feedback

DSIQ records time-sampled OpenUSD animation directly from the real
Bullet solver. Stage units are meters and Z-up. Every keyframe
includes measured reference and imported-URDF robot base and link
world-space transforms, joint positions and sandbox payload positions.

    python -m dsiq_native bullet-usd-record actual_motion.usda --frames 16 --steps-per-frame 12 --urdf-manifest machine/machine.manifest.json --joint actuator_x --target 0.35
    python -m dsiq_native usd-telemetry-inspect actual_motion.usda

Time codes are aligned with the 240 Hz fixed Bullet timestep. The file
can be inspected by standard Pixar USD clients and compatible viewers.
It is measured transform/joint telemetry: NOT manufacturer geometry,
a full scene renderer or PhysX/RTX.

An existing scene and these measurements can be visually combined
once accurate local-parent transform conversion and exact model
matching are implemented. The current archive does not pretend
that world-space link frames are safe local animation offsets.

## 4. Remaining industrial qualification requirements

- Measured link masses/inertias/friction, visual and collision CAD,
  dimensional tolerances and calibration for every actual machine.
- Independently qualified swept collision planning, torque and
  actuator limits, safety constraints and real-world benchmarks.
- Domain-specific physical engines beyond rigid bodies where needed:
  deformables, advanced materials, fluids and manufacturing processes.
- USD joint hierarchies, calibrated visual/material asset binding,
  animation roundtrip and robust dependency portability.
- Workstation profiling on 16 GiB RAM and RTX 3050, calibrated renderer
  and sensors, real hardware acceptance and operational risk review.
- Failing cases and uncertainty metrics, not only passing green tests.

All implemented features require CI tests on Windows/Linux and the
OpenUSD/Bullet integration suite. Passing those tests is not a
certificate covering all physical phenomena, robot families or USD files.


## Unified all-three-core acceptance command

    python -m dsiq_native core-readiness

This executes all ten currently defined isolated physics cases,
verified non-OEM imported-gantry multi-axis trajectory planning and
tracking, real Bullet discrete obstacle preflight, genuine OpenUSD
layer composition and actual time-sampled robot motion inspection.
It writes a machine-readable report containing all individual
software gate booleans and unresolved industrial blockers.

The all_defined_software_gates_passed property is a narrowly bounded
regression signal. The industrial_release_ready property remains FALSE
until external machine measurements, physical validation, production
safety and hardware qualification have been obtained. A green CI test
is not a replacement for those domain-specific proofs.


## URDF rigid-body inertia validation improvement

URDF link inertia matrices are required to be symmetric positive-definite
AND physically feasible in their **principal moment coordinate system**.
The importer now computes all three eigenvalues of each real symmetric 3x3
tensor (without requiring a Python numerical package in the contracts job)
and checks that no principal moment exceeds the sum of the other two.
This rejects otherwise-positive-definite off-diagonal matrices that pass
diagonal-only moment tests yet cannot describe a physical rigid body.

This is an input feasibility test; it does not measure real equipment
mass, center of gravity or principal inertia values, which still require
source measurements and independent engineering review.
