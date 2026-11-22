# DSIQ Native — Local CPU Bullet Robotics Playground

**What it is:** a second, independently executable physics backend for
an explicitly authored *reference six-axis manipulator*, plus rigid payload
bodies, collisions and measured joint motion. It uses Bullet's C++ physics
solver through the Python API and can render via Bullet's own OpenGL GUI.

**What it is not:** NVIDIA's PhysX or RTX engine, a licensed manufacturer
robot CAD assembly, an ISO-certified controller, a calibrated digital twin,
or evidence of industry-ready sensor/contact fidelity.

## Windows and Linux dependency selection

For Windows x86-64, the Python extra uses the separately maintained
`pybullet-arm64==3.2.8` *drop-in* distribution (its compiled Windows wheel
imports as `pybullet`). For Linux it uses upstream `pybullet==3.2.7`.
This decision avoids requiring a full local C++ build of PyBullet on Windows.
The Windows fork is third-party software and its licensing, provenance and
security require review before commercial packaging. Platform CI tests
the actual installed binaries, not just Python syntax.

## Why an additional backend?

The RTX 3050 + 16 GiB physical RAM target is below current Omniverse Kit and
Isaac Sim minimums. Running Kit for authoring/visualization remains experimental.
Bullet **DIRECT** mode runs CPU physics without GPU requirements. Bullet **GUI**
mode offers a real local OpenGL robot playground, with the familiar orbit,
pan and zoom controls and live six-joint sliders, though its render quality
is not equivalent to RTX path tracing.

Bullet and OpenUSD remain separate backends for now: do not claim their
dynamics are bitwise/physically identical. Cross-engine dynamic validation
and USD-to-Bullet asset transfer must be independently measured.

## Run now (Windows PowerShell)

```powershell
git clone https://github.com/Niroop-H/DSIQ-Native.git
cd DSIQ-Native
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".[physics]"

# CPU-only simulation, no Kit or display:
python -m dsiq_native bullet-run --steps 480 --boxes 5

# Save a reproducible machine-readable physics report:
python -m dsiq_native bullet-run --steps 240 --boxes 3 --json-out bullet-report.json

# Native 3D Bullet playground with joint control sliders:
python -m dsiq_native bullet-run --gui --boxes 5
```

If you also want the OpenUSD schema editor, install optional USD support:

```powershell
python -m pip install -e ".[robotics]"
python -m dsiq_native playground robot-workcell.usda --boxes 5
```

## Real physics features

- Real Bullet multibody with six independently actuated revolute joints.
- True joint positions/velocities, motor position commands and fixed 240 Hz
  time integration in CPU Bullet (the physical solver is C++).
- Configurable gravity, contact/friction behavior, payload count and mass.
- Colliding scene floor, world-fixed robot base, per-link simplified collision
  shapes, no opaque "pretend" simulation animation.
- Independent analytic FK and Bullet joint-frame position comparison on
  measured joint angles.
- Reading contact counts, payload positions and world-frame end-effector pose.
- Explicit per-world physics client IDs; separate simulations cannot interfere.
- All user-controlled motor requests validated against six mechanical limits.
- Report records simulated time, fixed timestep, actual joint state, reference
  difference, contacts, simulation engine and nonqualification status.

## Precision boundaries

`BulletSettings` uses fixed dt=1/240 seconds, 100 solver iterations by
default, and target joint motors. This is **not** proof of exact physical
accuracy: friction, contact manifolds, motor saturation, inertia and numeric
integration remain approximations. The collision bodies are simple reference
geometry with declared approximate link masses rather than measured OEM data.

A trajectory obtained from Bullet cannot automatically be transferred to a
real robot. Dynamics need independent empirical tests, controller tuning,
measured inertias, emergency limits and collision safety acceptance.

## System memory

Most constrained scenes here are simple and CPU-resident. Actual memory
consumption is to be measured on the user machine, especially if more bodies
or external CAD objects are added. The 16 GiB RAM target is a policy/acceptance
constraint, not a hard per-process allocator in this initial runtime.

The Bullet native GUI uses OpenGL, **not RTX**. Light/dark DSIQ Studio themes
are supplied through the optional NVIDIA Kit plugin; Bullet's built-in debug
window theme is determined by Bullet.

## Acceptance tests

GitHub Actions runs actual Bullet simulation in **DIRECT** mode on Windows
and Linux. Tests cover six-link construction, home and several off-home
kinematic poses against independent analytic FK, gravity and falling payload
motion, joint motor actuation, invalid-parameter rejection, max-payload limits,
and physics-world isolation.

Passing these tests demonstrates the reference simulation works on those CI
runners. It does not establish actual RTX 3050 GUI support, validated contact
precision, production timing guarantees or OEM manipulator equivalence.

Official Bullet API:
https://github.com/bulletphysics/bullet3

## Real robot source interoperability

A rights-verified user-owned URDF can now be inspected and loaded into
Bullet for physically executed joint commands (not manufacturer calibration).
See [URDF Robot Imports](URDF_ROBOTS.md). DSIQ does not distribute OEM CAD.
