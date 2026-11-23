# DSIQ Native — Pluggable Physics Backend Contract v1

The repository roadmap prioritizes a **solver-neutral library boundary** before adding extra physics engines. The first shipped implementation was a real isolated **Bullet CPU DIRECT** world. A second, optional real **MuJoCo CPU** solver adapter is now implemented and independently exercised. Newton, PhysX and GPU tensor backends are not implemented or claimed.

## Python contract

Module: `dsiq_native.physics_backend`

- `PhysicsBackendWorld` (runtime-checkable protocol): import verified URDF, command one source actuator, explicitly step a world, read measured joint feedback and discrete contacts, close world.
- `BackendCapabilities`: explicit solver ID, timestep, source formats and unsupported features. USD Physics import, GPU batches, hardware actuation and industrial qualification are currently **false**.
- `BulletPhysicsBackend`: each adapter owns one isolated PyBullet DIRECT client and a source-verified URDF manifest; no fake import/solver substitution.
- `open_backend(name)`: accepts `bullet_cpu_direct_v1` or `mujoco_cpu_direct_v1` (MuJoCo bindings required); all other names fail closed.
- `run_backend_conformance(name)`: creates an explicitly labelled, self-authored gantry fixture, verifies the source SHA-256, motor limit rejection, measured motor response, fixed-step clock and two-world repeatability.

## CLI smoke acceptance

```powershell
python -m pip install -e ".[physics]"
python -m dsiq_native physics-backend-check
python -m unittest discover -s tests -p test_physics_backend.py -v
```

These commands run actual Bullet simulation and return a machine-readable JSON report. The Bullet conformance test is required in both `bullet-linux` and `bullet-windows` CI jobs. An independent `mujoco-conformance-ubuntu` job installs real native MuJoCo, exercises the complete contract, and archives a real Bullet/MuJoCo source-matched comparison. Preserve all test artifacts and solver versions for a meaningful comparison later.

## Second solver: real MuJoCo CPU (not a Bullet shim)

```powershell
python -m pip install -e ".[mujoco]"
python -m dsiq_native physics-backend-check --backend mujoco_cpu_direct_v1
python -m dsiq_native physics-cross-backend-check --tolerance 0.01
```

The MuJoCo adapter compiles the same source-verified URDF with the native MuJoCo loader, reads independent `MjData` positions, velocities and contacts, and applies a documented effort-limited PD reference drive. The Bullet adapter continues to run independent `pybullet` simulation. The automated comparison reports the absolute measured X-axis residual after 100 steps and reproducibility across two isolated worlds per solver.

**Important:** The reference controllers are not OEM calibrated; success within the **10 mm final-position and sampled peak-position** software screening gates does not establish numerical solver equivalence, comprehensive closed-loop stability, or OEM machine validation. A separate actual 100-step, 240 Hz trace checks peak speed, effort and exact URDF travel envelope. Measured: 0.255 mm final and 9.41 mm peak position differences on the current reference gantry. Full trace evidence is archived by CI. The MuJoCo adapter currently supports one fixed-base URDF per world. USD Physics import, PhysX, Newton and GPU batching remain unsupported.

## First adapter's source boundary

- Source geometry is checked by the existing `validate_robot_manifest` pipeline before loading. The loaded robot's SHA-256 is repeated in joint feedback.
- The reference six-axis manipulator in BulletPlayground is not an imported OEM robot.
- A controller command only affects a named imported robot; its URDF speed/effort/travel constraints are enforced.
- All reports explicitly identify solver-derived observations and disclaim OEM calibration, continuous swept collision qualification and physical E-stop protection.
- A world cannot be reused after closing; multiple clients must never share implicit global PyBullet state.

## Required before adding further solvers

1. A documented and distributable dependency licence/build recipe for the new engine.
2. A real asset loader for the declared source format, with matching units and source identity.
3. Fixed time stepping; named joints; source motor ranges; explicit collision/contact semantics; stable reset and destruction.
4. Two-solver conformance using identical physical properties, calibrated reference parameters, deterministic trials and comparable error metrics. **Matching one fixture does not certify general equivalence.**
5. CI on the actual supported platforms; do not silently select Bullet when another backend fails.
6. A separate, carefully scoped USD Physics mapping adapter. Do not simply label a URDF loader as USD Physics compatibility.

Future work should move the existing high-level trajectory runner and sensor pipelines behind this adapter incrementally, keeping the prior direct Bullet implementation as a reference oracle until contract parity passes.

## Safety and numerical limits

This is **a software integration interface**, not a physical safety or control standard. Torque, damping, friction, contact, integration stability and controller timing remain solver- and asset-dependent. A 100/100 industrial qualification requires independent evidence on real equipment.
