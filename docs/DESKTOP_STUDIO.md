# DSIQ Native Desktop Robotics Studio

**Development preview.** A genuine standalone DSIQ desktop UI using Python
Tkinter, the C++ Bullet physics engine, and Bullet TinyRenderer. It does not
need NVIDIA Kit, RTX, CUDA, or a high-end GPU to run the reference-robot
playground. **Its software raster renderer is not an RTX renderer.**

## Run on Windows with 16 GiB RAM and an RTX 3050

Install Python 3.11 x64 with Tcl/Tk:

~~~powershell
git clone https://github.com/Niroop-H/DSIQ-Native.git
cd DSIQ-Native
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e ".[physics]"

# Pure-black desktop:
python -m dsiq_native studio --theme black --boxes 4

# White desktop:
python -m dsiq_native studio --theme light --boxes 2
~~~

The Windows physics extra uses independently maintained pybullet-arm64 3.2.8
as a drop-in compiled Windows distribution because upstream PyBullet does
not have official CPython 3.11 Windows wheels. This third-party fork
requires source/license/security review before commercial distribution.
Linux uses upstream PyBullet 3.2.7.

## Actual controls

- Scene: reset reference workcell; export OpenUSD reference schemas if
  Pixar USD is installed.
- Simulation: Play, Pause, Step, fixed 1/240 s integrator in Bullet.
- Payloads: specify mass, dimensions, XYZ and spawn real colliding bodies.
- Reference robot: six independent joint-angle sliders with limits,
  actuated motors, home and position-based inverse kinematics.
- Sensors: live TinyRenderer color image and 72-ray geometric LiDAR,
  screenshot PNG export and physics-state JSON export.
- Navigation: drag to orbit, wheel to zoom, Frame All.
- Theme: PURE BLACK #000000 or LIGHT #FFFFFF applied to the DSIQ UI.
- Live diagnostics: actual solver time, body/contact count and analytic
  FK residual against measured Bullet joint state.

## Execution design

~~~text
DSIQ Native Tkinter Window
  ├── OpenUSD scene-schema authoring and reference-scene export
  ├── 6R reference robot / motor controls
  ├── C++ Bullet rigid-body physics (fixed 240 Hz simulation step)
  ├── Bullet TinyRenderer camera (CPU 3D viewport)
  ├── LiDAR / RGB-D / segmentation / joint-state sensor APIs
  └── state export, physics tests and error messages
~~~

All visuals in the live camera come from Bullet's current simulated
bodies, not fabricated screenshots.

## Important fidelity distinctions

- Reference robot uses simplified collision geometry and estimated
  inertial parameters. No manufacturer/OEM certification.
- Cartesian IK solves position only, not orientation or collision avoidance.
- GUI is software-rasterized CPU, not full NVIDIA RTX/path tracing.
- Exported USD contains a valid *reference* scene and joint motor targets,
  not an exact snapshot of Bullet's current dynamics.
- Does not yet offer full CAD import into the live Bullet viewport,
  real ROS 2 robotics, hardware commissioning, multi-modal calibrated
  sensors or industrial photometric correctness.
- An RTX 3050 does not become officially supported for current Isaac Sim
  merely because this separate CPU backend runs.
- Native GUI window requires actual user-device acceptance; GitHub CI runs
  physics and sensors headlessly and validates desktop configuration
  and camera geometry, but not native mouse/viewport operation.

## Performance admission

A typical CPU-rendered reference scene is relatively small; no general
16 GiB system RAM ceiling can be guaranteed without measurement.
The application limits sensor resolution to 200,000 pixels and software
refresh rate to at most 20 Hz. Real GPU/CPU memory, frame time, physics
error and stability acceptance is still required.

This is the first **standalone DSIQ-branded usable reference-robot studio**,
not a complete replacement for Omniverse or Isaac Sim.
