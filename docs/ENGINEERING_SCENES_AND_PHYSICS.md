# DSIQ Native — Physics, Machine Design, Environments and Free Camera

Status: **engineering development preview**. There is no claim of full
NVIDIA Omniverse or Isaac Sim parity on the specified 16 GiB system RAM
and RTX 3050 workstation.

## Quantitative physics

Install Python 3.11 and the optional robotics dependencies:

~~~powershell
python -m pip install -e ".[robotics]"
python -m dsiq_native physics-verify
~~~

The suite executes real Bullet numerical experiments:
- Free fall, compared with analytic gravitational displacement/velocity.
- Settling rigid-body contact, compared with a measured rest height.
- Constant torque on a rigid body, compared with torque/inertia dynamics.
- Independent-client repeatability with a fixed simulation timestep.

Each result contains observed values, analytic references, explicit
tolerances and pass/fail data. Passing these individual checks does NOT
prove physically perfect simulation, real equipment safety, deformable
physics, fluid dynamics or parity with NVIDIA PhysX.

## Create and test actual articulated machines

An editable XYZ gantry can be authored with user-specific dimensions,
masses, three actuators, travel limits, structural uprights, rail
carriages, a sensor/tool flange and joint drive limits.

~~~powershell
python -m dsiq_native machine-create my-gantry --name user_xyz_gantry --x-travel 1.2 --y-travel 0.8 --z-travel 0.4 --height 1.5

# Edit my-gantry/machine.design.json and rebuild into a NEW empty folder.
python -m dsiq_native machine-build my-gantry/machine.design.json my-gantry-v2

# Inspect actual joint limits and execute a 350 mm X-axis command.
python -m dsiq_native urdf-inspect my-gantry-v2/machine.urdf
python -m dsiq_native bullet-urdf my-gantry-v2/machine.manifest.json --joint actuator_x --target 0.35 --steps 480
~~~

Generated files include editable JSON, valid URDF, a SHA-256 provenance
manifest and a source-rights file. These are original mechanically useful
**parametric reference parts**, not OEM machine CAD. Real manufacturer
geometry still requires licensed CAD sources, visual QA, true measured
inertias, collisions and independent physical validation.

Imported URDF motor force/velocity is now taken from the source joint
metadata, not an arbitrary fixed force.

## Semantic architectural environments

Three reference backgrounds, with meaningful named parts and collisions:
- robotics_cell: safety guarding, frames, human access opening, work
  table, cable bridge and hazard floor lines;
- cleanroom_bay: floor modules, HEPA cassette references, equipment
  footprint, ventilation/utility chases and glazed partitions;
- logistics_bay: loading racks, pallet bays, static rollers and
  segregated travel lanes.

These are detailed functional architecture references, NOT fake
manufacturer-specific machinery. Static rollers do not run a conveyor.

~~~powershell
python -m dsiq_native environment-render robotics_cell --png robotics.png
python -m dsiq_native environment-render cleanroom_bay --png cleanroom.png
python -m dsiq_native environment-render logistics_bay --png logistics.png
~~~

Actual OpenUSD exports include metric Z-up transforms, component
names, semantic roles, static collision APIs, and USD Preview Surface
metallic/roughness/color/opacity materials:

~~~powershell
python -m dsiq_native environment-usd robotics_cell robotics.usda
python -m dsiq_native environment-usd cleanroom_bay cleanroom.usda
python -m dsiq_native environment-usd logistics_bay logistics.usda
~~~

OpenUSD metadata can be used by compatible renderers, but the low-memory
viewer uses Bullet TinyRenderer CPU. Material descriptors are NOT
photorealistic RTX lighting. Professional texture and asset pipelines
remain an explicit acceptance requirement.

## Interactive desktop and six-degree free movement

~~~powershell
python -m dsiq_native studio --theme black
python -m dsiq_native studio --theme light
~~~

The desktop View menu toggles orbit and free-fly camera, and
performance/balanced/detailed CPU viewport rendering quality.

Click inside the viewport to give it keyboard focus. In Free mode:
- W/S: forward/backward along the camera's heading.
- A/D: strafe left/right.
- Q/E: descend/ascend in the world Z axis.
- Shift + movement: sprint.
- Drag: camera yaw/pitch orientation.
- Mouse wheel: move forward/backward.
- Frame All: return to the reference scene.

The Environments menu switches scene layout. The Machine Lab menu
creates a parameterized machine, rebuilds an edited design, imports a
verified URDF and controls imported joints. The Verification menu runs
measured physics acceptance fixtures. Camera movement never changes
simulated physics state.

Viewport profiles are 192x144 at 16 FPS, 480x360 at 12 FPS and
640x300 at 8 FPS, all *software CPU rendering*. They are not quality
guarantees for arbitrary industrial scenes.

## Production criteria

- Independent physical reference measurements, uncertainty budgets,
  timestep convergence and collision/contact stress tests.
- Legally sourced, highly detailed real equipment CAD and material
  texture maps, with proper geometry, semantics, sensors and part motion.
- Photorealistic native renderer and actual RTX GPU acceptance data.
- Robust Windows packaging, asset import isolation and project workflow.
- No silent fallback from validated physics to demonstrative animation.
- No default write access to real PLCs or physical robot controllers.
- Measured RAM/VRAM on actual 16 GiB/RTX 3050 machines.

This engineering milestone improves the independent platform without
misrepresenting prototype assets or renderer capabilities.
