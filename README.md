# DSIQ Native — OpenUSD Industrial Engineering Studio

**Devashri S Industries · MAH Quantum**

DSIQ Native is a new, independent Windows-first industrial scene-authoring, simulation, and robotics engineering project. It targets measured, resource-aware operation on **16 GiB system RAM and an NVIDIA GeForce RTX 3050**, including configurations with **4 GiB GPU VRAM**. This hardware is **below NVIDIA's published minimum for current Omniverse Kit and Isaac Sim releases**. Running the Kit-based GUI or RTX rendering on such a machine is *experimental, not guaranteed*. Headless tools are the guaranteed development baseline only after their tests pass.

> **Status: foundation / pre-alpha.** This repository is not yet equivalent to NVIDIA Omniverse, Isaac Sim, or Isaac Lab. It does not contain 606 validated physical machines, shipping RTX rendering, an OEM CAD library, or commissioned control/robot models. The code, tests and acceptance gates below are starting infrastructure, not evidence of parity.

The existing [DSIQ Domain Expansion](https://github.com/Niroop-H/DSIQ-Domain-Expansion) remains the **separate Web Edition** and is not modified by this project.

## Architectural principles

1. **OpenUSD is authoritative**: metric units, Z-up, explicit references/layers, consistent machine and sensor identities. Never approximate a customer's CAD into generic boxes while claiming geometric equivalence.
2. **Separate fidelity levels**: (a) authored visual reference, (b) numerically executable model, (c) calibrated / independently accepted industrial twin. The UI and APIs must never silently promote a model between levels.
3. **Use real assets with provenance**: manufacturer/model/revision, license evidence, source file hashes, assembly hierarchy, LOD, collision source, measured inertias, material evidence and domain-specific acceptance records. If missing, report **NOT READY** rather than render a counterfeit model.
4. **Rendering and physics are independent**: GPU backends may be paused, disabled, or quality-reduced without changing physical model coefficients or integrator settings. Accuracy is assessed using external reference datasets and reproducible numerical tests.
5. **16 GiB RAM, RTX 3050 is the design target, not a guarantee**: select capability tiers from *measured free* GPU memory and RAM; refuse oversize scenes before exhausting memory; record observed peak memory, p95 step time, asset provenance, and rendering metrics. GPU driver/Kit allocations remain outside DSIQ's full control.
6. **Native differentiation**: actionable equipment metadata, engineering evidence, asset validation, digital twin integrations and useful operator workflows—not simply copying NVIDIA's interface.

## Foundation and current implementation

- Standard-library, platform-aware NVIDIA GPU and RAM inspection with conservative, *estimated* application-asset budgets and clear warnings.
- An OEM/asset manifest validation gate: SHA-256 checks, source provenance, licensing evidence, exact CAD files and physics-evidence classification; missing files fail closed.
- Headless OpenUSD scene creation, inspection, and reference attachment via Pixar's `pxr` Python bindings (requires `usd-core` or a compatible Kit Python environment).
- An optional **Kit SDK Python extension** with active stage diagnostics, robotics playground controls, gravity, motor drive targets and theme switching, It runs *inside a separately installed NVIDIA Kit Base Editor*; it does **not** ship NVIDIA's proprietary SDK binaries.
- Python unit/integration tests and GitHub CI scaffolding.
- Competitive-feature, hardware and engineering-acceptance specifications.

**Not yet built/qualified:** standalone RTX rendering integration, validated PhysX articulation simulation on the RTX 3050, MJCF import, production-qualified ROS 2 bridges, industry-calibrated robot dynamics, RTX-accurate RGB/depth sensors, realistic sensor noise calibration, detailed approved OEM CAD libraries, and production Windows installers. Actual CPU Bullet RGB-D/LiDAR and verified local URDF inspection/import exist, but they are not RTX or physically calibrated replacements. These are explicit milestones, not hidden stubs labeled complete.

## Extended contact physics and read-only ROS 2 integration

DSIQ now has additional numeric acceptance experiments for
**resting contact at 120/240/480 Hz**, friction-sensitive real Bullet
sliding dynamics, and URDF prismatic-joint velocity/effort limits.
Measurements include observed errors and thresholds rather than claiming
perfect physics.

A new ROS 2 preview builds actual `JointState`, `LaserScan` and
simulation-clock message fields from Bullet's measured joint states and
raycast sensor results. An **optional ROS 2 rclpy publisher** can send
the measurements on isolated `/dsiq/` topics when an independently
installed ROS 2 environment is available. It is strictly read-only and
has no physical actuator-command channel.

~~~powershell
python -m pip install -e ".[physics]"
python -m dsiq_native physics-extended
python -m dsiq_native ros2-preview --steps-per-frame 12 --lidar-rays 36
# Requires a separate, functioning ROS 2 Python runtime:
python -m dsiq_native ros2-stream --frames 100
~~~

See [Extended Physics Evidence](docs/EXTENDED_PHYSICS.md) and
[ROS 2 Read-Only Bridge](docs/ROS2_BRIDGE.md). CI tests actual Bullet
measurements and offline ROS field contracts; live ROS 2 DDS middleware
is **not** exercised or claimed validated by ordinary CI.

## Isaac Sim gap: physics-backed synthetic datasets

A bounded CPU Bullet dataset generator now records *actual* simulation
state, RGBA image pixels, metric depth, segmentation IDs, randomized
physics inputs, camera parameters, and SHA-256 integrity records. This
is original research-grade prototype output, **not** NVIDIA RTX
Replicator, calibrated sensors or real manufacturer ground truth.

~~~powershell
python -m pip install -e ".[physics]"
python -m dsiq_native dataset-generate output-data --frames 8 --seed 1007
python -m dsiq_native dataset-verify output-data
~~~

See [Synthetic Perception Data](docs/SYNTHETIC_DATA.md).

## Omniverse gap: layered OpenUSD asset assembly

DSIQ can now create a persistent layered project with editable equipment
references and separate position/rotation layout opinions. It accepts
only SHA-256-and-rights-verified real USD sources containing valid mesh
geometry and can author actual USD representation variants from two
explicitly supplied sources, without manufacturing fake detailed CAD.

~~~powershell
python -m pip install -e ".[usd]"
python -m dsiq_native usd-workspace-new my-project
python -m dsiq_native usd-workspace-import my-project assets/robot/asset.manifest.json
python -m dsiq_native usd-workspace-place my-project my_robot --xyz 0 0 1 --yaw 45
python -m dsiq_native usd-workspace-inspect my-project
~~~

See [OpenUSD Assembly and Representation Layers](docs/USD_COMPOSITION.md).
These commands do not constitute a full Omniverse Composer UI or
multiplayer real-time collaboration.

## Physics Solver Convergence Tests

The physics verification suite now compares falling-body solutions at
**120, 240 and 480 Hz** against an independent analytical reference,
requiring measured error to decrease as the timestep is refined.
The result reports error magnitudes and ratios instead of treating
one successful gravity test as proof of universal precision.
This is **not** a contact, deformable-body or industrial dynamics certification.

~~~powershell
python -m dsiq_native physics-verify
~~~

## Inertia and Actuator Import Safety

Imported URDF links now require a physically feasible positive-definite
inertia tensor and consistency checks on inertia moments. Movable joints
require explicit positive effort and speed limits before import. These
are minimum numerical safeguards, not evidence of calibrated OEM dynamics.

## Physics Assurance + User Machine Lab + Free Camera

New development tools support measurable physical reference experiments,
actual user-editable XYZ gantry construction, semantically organized facility
backgrounds and 6DOF camera navigation. All references are labeled honestly.

~~~powershell
python -m pip install -e ".[robotics]"
python -m dsiq_native physics-verify
python -m dsiq_native machine-create user-gantry --name user_xyz_gantry --x-travel 1.2
python -m dsiq_native bullet-urdf user-gantry/machine.manifest.json --joint actuator_x --target 0.3 --steps 480
python -m dsiq_native machine-verify user-gantry
python -m dsiq_native environment-render robotics_cell --png reference.png
python -m dsiq_native environment-usd cleanroom_bay cleanroom.usda
python -m dsiq_native studio --theme black
~~~

The desktop now has free-fly WASD/Q/E navigation, selectable guarded robotics
cell, cleanroom and warehouse handling environments, a machine designer and
physics verification. Facility layouts include actual semantic structural
objects and physical collision where appropriate; they do not impersonate
licensed industrial machines or photorealistic environments.

USD exports include reference materials and collision schemas. Actual
RTX/Kit rendering, detailed OEM CAD and production physics qualifications
still require dedicated development and RTX 3050 hardware acceptance.

See **[Physics, Machine Design and Free Camera](docs/ENGINEERING_SCENES_AND_PHYSICS.md)**.

## Reference Robot Trajectory Programming

The Reference6R can now generate **joint-limit-aware quintic trajectories**,
using analytically checked speed and acceleration bounds at fixed
1/240-second simulation resolution. These trajectories can be executed
using real Bullet motor control, with measured joint tracking residuals
and contact counts recorded rather than claiming perfect motion.

~~~powershell
python -m dsiq_native robot-plan --start 0 0 0 0 0 0 --goal 10 -20 25 5 -8 4
python -m dsiq_native bullet-trajectory --goal 10 -20 25 5 -8 4
~~~

The planner is not collision-free, torque-certified or OEM-calibrated;
its limits are documented DSIQ reference values only.
See **[Trajectory Programming and Tracking](docs/TRAJECTORY_PLANNER.md)**.

## Licensed industrial URDF robot support

DSIQ now inspects actual local **URDF robot assemblies**, validates links,
joint topology, inertial properties, mesh references, source SHA-256 and
license evidence, and can import approved source robots into real Bullet
physics for joint actuation. The native desktop File menu includes
**Import verified URDF robot** and **Control imported robot joint**.

~~~powershell
python -m dsiq_native urdf-inspect path/to/robot.urdf
python -m dsiq_native bullet-urdf path/to/robot.manifest.json --joint elbow --target 0.3 --steps 480
~~~

Physical geometry must come from a genuine, legally usable source.
Test fixtures are identified as such; no industrial model is fabricated
or presented as manufacturer-certified. ROS package:// URIs, Xacro and
untrusted plugins are rejected until an audited import pipeline exists.
See **[Verified URDF imports](docs/URDF_ROBOTS.md)**.

## NEW — Standalone DSIQ Native Desktop Robotics Studio

The project now also contains a self-contained, DSIQ-branded native desktop
control interface with **PURE BLACK (#000000)** and **LIGHT (#FFFFFF)** themes.
Unlike the experimental Omniverse Kit interface, it uses local C++ Bullet
physics and a software-rendered 3D camera, making it a credible lower-RAM
development path on the RTX 3050 system.

The GUI has actual orbit/zoom viewport navigation, Play/Pause/Step, dynamic
payload creation, a six-joint robot control panel, position-only Cartesian IK,
LiDAR scanning, PNG screenshots, reference OpenUSD scene export and real
physics-state JSON reporting. This is **not RTX visualization or calibrated
OEM dynamics**, and the GUI requires on-device validation beyond headless CI.

~~~powershell
python -m pip install -e ".[physics]"
python -m dsiq_native studio --theme black --boxes 4
# Or:
python -m dsiq_native studio --theme light --boxes 4
~~~

See **[Standalone Desktop Studio](docs/DESKTOP_STUDIO.md)**.

## CPU Robotics Sensor Suite

The constrained-memory Bullet backend now also exposes **geometric LiDAR,
RGB-D camera and joint-link velocity** data from the actual physics world.
Outputs include real ray hits, measured depth buffers and segmentation IDs;
color PNG and depth NPZ exports are available without NVIDIA Kit.

```powershell
python -m dsiq_native bullet-run --steps 240 --lidar-rays 36
python -m dsiq_native bullet-run --steps 120 --camera-png rgb.png --depth-npz depth.npz
```

This is low-cost, idealized CPU sensing—not calibrated real-world camera or
LiDAR response and not NVIDIA RTX sensor rendering.

See **[Bullet CPU Sensors](docs/CPU_SENSORS.md)**.

## NEW — Local CPU Bullet Robotics Runtime (RTX 3050 fallback)

DSIQ now also has a **real C++ Bullet-backed physics engine** accessible through
PyBullet. It runs actual rigid-body contacts, gravity, six revolute joints,
motor control and payload dynamics **without Kit or NVIDIA RTX**. A separate
native Bullet OpenGL GUI provides interactive joint sliders and scene orbit
controls; this is a lower-fidelity viewer, not a fake RTX implementation.

```powershell
python -m pip install -e ".[physics]"
python -m dsiq_native bullet-run --steps 480 --boxes 5
python -m dsiq_native bullet-run --gui --boxes 5
```

The engine reports real joint feedback and compares tool positions to an
independent analytic kinematic model. It uses simplified and explicitly
identified reference geometry—not OEM assets or physically calibrated robot
inertias. Validation runs on CPU in Windows/Linux CI.

See **[CPU Bullet Playground](docs/CPU_PLAYGROUND.md)**.

## Robotics Playground — development milestone

The robotics module now includes a meaningful, original six-axis reference arm with
analytic FK/Jacobian and position-only IK, real USD revolute joints and angular
motor-drive schemas, test payloads and a configurable physics scene. The Kit
extension has user controls for creating/resetting the playground, adding physical
sandbox objects, editing robot joint targets, Cartesian IK, gravity, play/pause/stop,
saving USD, and **pure-black (#000000) / light (#FFFFFF) themes**.

These are verified through source and headless unit/integration tests, not a claim
that PhysX, OEM dynamics or RTX rendering has passed tests on an RTX 3050.
The model uses transparent simplified reference collision geometry, **not** a
misleading imitation of manufacturer CAD.

See **[Robotics Playground — Controls and validation](docs/ROBOTICS_PLAYGROUND.md)**.

```powershell
python -m dsiq_native playground examples/robotics.usda --boxes 5
python -m dsiq_native playground-report examples/robotics.usda
python -m dsiq_native robot-fk 0 -20 40 0 0 0
python -m dsiq_native robot-ik 0.7 0.2 0.7
python -m dsiq_native robot-drive examples/robotics.usda 0 -20 40 0 0 0
```

## Use the functional headless tools

Python 3.11+ recommended; PowerShell examples:

```powershell
git clone https://github.com/Niroop-H/DSIQ-Native.git
cd DSIQ-Native
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".[usd]"
python -m dsiq_native hardware
python -m dsiq_native new-stage examples/local-scene.usda
python -m dsiq_native inspect examples/local-scene.usda
python -m unittest discover -s tests -v
```

To reference a **real, licensed OpenUSD asset**, first prepare its `dsiq-asset-v1` manifest with a real source geometry file and actual SHA-256; validate before attachment:

```powershell
python -m dsiq_native validate-asset path\to\asset.manifest.json
python -m dsiq_native attach examples/local-scene.usda path\to\asset.manifest.json
```

No fabricated high-fidelity CAD assets or manufacturer names are included.

## Try the optional NVIDIA Kit-based scene inspector

Use NVIDIA's official [Kit App Template](https://github.com/NVIDIA-Omniverse/kit-app-template) or a compatible **Kit Base Editor** installation and enable the extension in `source/extensions/dsiq.studio`. See [Kit integration notes](docs/KIT_INTEGRATION.md). Do not assume Kit or Isaac Sim will load on a 4 GiB RTX 3050 without device tests.

## Project status and acceptance criteria

See [Feature parity matrix](docs/COMPETITIVE_MATRIX.md), [Hardware contract](docs/HARDWARE_CONTRACT.md) and [Engineering standards](docs/ENGINEERING_STANDARDS.md). The acceptance program includes 4/6/8 GiB GPU variants, real CAD assemblies, fixed robotic reference trajectories, reproducibility checks, GPU traces, and a strict no-fabricated-assets policy.

**Third-party software and trademarks**: OpenUSD is an independent open project; NVIDIA Omniverse, Kit, RTX, PhysX, and Isaac Sim are NVIDIA products. Their availability, binaries, licensing and redistribution are governed by their respective owners' terms. DSIQ is not affiliated with or endorsed by NVIDIA.

Official documentation:
- https://docs.omniverse.nvidia.com/dev-overview/latest/common/technical-requirements.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/requirements.html
- https://docs.omniverse.nvidia.com/dev-overview/latest/common/NVIDIA_Omniverse_License_Agreement.html
