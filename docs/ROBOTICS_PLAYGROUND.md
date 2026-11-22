# DSIQ Native — Robotics Playground v1

**Implemented:** OpenUSD reference playground generation, six-revolute-joint robot
with a real analytic forward-kinematics and Jacobian implementation, position
inverse kinematics, USD joint motor-drive targets, explicitly authored rigid-body
schemas, box payloads with collision and mass, a ground collider, and Kit-native
control UI with PURE BLACK (#000000) / LIGHT (#FFFFFF) modes.

**Not implemented/qualified:** independently validated mass and collision
properties for any OEM robot, production articulation stability, physically
calibrated sensors, real robot tool electronics, obstacle avoidance, torque
controllers, RTX3050 frame-rate acceptance, or a distributable standalone GUI.

## Zero-cost headless workflow (Windows PowerShell)

```powershell
git clone https://github.com/Niroop-H/DSIQ-Native.git
cd DSIQ-Native
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".[usd]"

# Build a metric Z-up robotics scene
python -m dsiq_native playground examples\robotics.usda --boxes 5

# Inspect USD-authored bodies and joints
python -m dsiq_native playground-report examples\robotics.usda

# Query analytical kinematics (inputs: degrees; output: meters, radians)
python -m dsiq_native robot-fk 0 -20 40 10 0 0
python -m dsiq_native robot-ik 0.7 0.2 0.7

# Set actual USD angular joint drive targets (degrees), then open in Kit
python -m dsiq_native robot-drive examples\robotics.usda 0 -20 40 10 0 0

# Add an actual rigid sandbox cube, with explicit mass
python -m dsiq_native spawn-box examples\robotics.usda --mass 2 --side .14 --xyz 0.2 1.0 1.2

python -m unittest discover -s tests -v
```

To reset an existing scene's playground while retaining any scene assets
outside `/World/Playground`:

```powershell
python -m dsiq_native playground examples\robotics.usda --reset --boxes 2
```

## Reference-6R model

Original mechanical teaching model, **not** a UR5, Panda, KR, IRB, or other OEM
machine. It is described by six serial revolute axes (Z,Y,Y,X,Y,X),
six link offsets, declared joint limits, link mass surrogates, and a tool-flange
offset. All link masses and diagonal inertia tensors are **reference estimates
for simplified colliders**, not measured properties.

| Joint | Axis | Home offset (m) | Angular limits (deg) |
|---|---|---|---|
| 1 Base azimuth | Z | (0,0,0.28) | -175 to 175 |
| 2 Shoulder | Y | (0,0,0.22) | -125 to 125 |
| 3 Elbow | Y | (0.42,0,0) | -145 to 145 |
| 4 Wrist roll | X | (0.38,0,0) | -180 to 180 |
| 5 Wrist pitch | Y | (0.16,0,0) | -120 to 120 |
| 6 Tool roll | X | (0.12,0,0) | -180 to 180 |

Home end-effector (zero angles): **(1.20, 0, 0.50) meters**.

The kinematic solver uses 3D Rodrigues transforms, analytical
`axis × (tool - joint-origin)` Jacobian, and damped least-squares iterative
IK with angle limits, maximum iteration count and convergence residuals.
This makes it useful for reference controls and test benches. It does **not**
solve end-effector orientation, self-collision, reach envelopes, singular
robustness at every target, obstacle avoidance or industrial path planning.

## OpenUSD physics semantics

- Independent sibling rigid bodies for links; USD Physics articulation root.
- Proper per-link collision shapes, masses, box-derived inertia tensors,
  gravity, angular joint limits and `DriveAPI` angular targets.
- Ground collider and up to 40 individually named test payloads.
- Exactly metric (metersPerUnit=1), Z-up stage; mismatched stages are rejected.
- Creation never silently deletes existing user work. Reset is explicit.
- Display solids are *reference mechanical geometry*, not OEM scanned,
  vendor-certified or dimensionally calibrated CAD.

`UsdPhysics` is scene **schema**, not a solver. For physical movement a
compatible Kit host must have `omni.physx` enabled and Kit's timeline running.
A matching USD stage opening correctly does not prove collision, constraint
or robotic fidelity. PhysX stepping and GPU behavior require acceptance tests.

## Native Kit UI features

Open a stage in Kit, enable `dsiq.studio`, then use the DSIQ Native panel:
- Pure black (#000000) and light (#FFFFFF) palettes; data entry state retained
  while switching. This styles DSIQ panels only, not the whole NVIDIA app.
- Inspect prims and active playground, save non-anonymous USD stage.
- Create/reset playground with explicit destructive reset label.
- Play/pause/stop Kit's real timeline and report whether PhysX is enabled.
- Spawn physical payloads with input mass, side and XYZ.
- Edit six joint angular target positions in degrees with physical limit checks.
- Home robot, specify Cartesian XYZ and solve/apply position-only IK.
- Adjust USD gravity; every invalid action produces explicit error messages.

## Numerical acceptance

Independent tests check analytic FK, right-handed orthonormal rotation,
Jacobian against finite differences, joint-limit enforcement, reachable IK,
unreachable-target failure, API input validation, USD on-disk scene composition,
body and collider counts, actual joint-drive target persistence, reset
idempotence, 40-body cap, and CLI read/write behavior.

**Next evidence gates:** Kit launch and PhysX step results on supported GPUs;
reference dynamics checks; CPU fallback physics; RTX3050 4/6/8 GiB measurements;
real licensed robot CAD, sensor calibration, collision reference tests and
comprehensive automation of the GUI.

## Why not claim a complete Isaac Sim competitor now?

Isaac Sim's official minimum is 32 GiB physical RAM and 16 GiB GPU VRAM, with
a higher-class RTX GPU than 3050. DSIQ's low-memory target is narrower. We can
offer useful robotics authoring and analytical tooling under those constraints;
we must not market full sensor/RTX/robot-learning parity without working
benchmarks on the target hardware.

NVIDIA Kit/Physics documentation:
- https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/rigid_bodies.html
- https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/simulation_control/simulation_control.html
- https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html
