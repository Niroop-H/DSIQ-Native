# Engineering quality policy

## No generic shapes masquerading as industrial equipment

Each *detailed* asset must have approved real CAD/OpenUSD geometry (or explicitly identified, accurately authored reference geometry), manufacturer/model/revision, lawful usage rights, geometry hash, exact scale/axis, meaningful mechanical subassemblies and documented visual acceptance references. Include verified LOD, collision geometry, material evidence, actuated joints, wiring/pipe routes where actually relevant. Missing equipment detail is shown as NOT AVAILABLE; do not invent it.

The initial DSIQ manifest validator checks local integrity, provenance metadata and evidence presence. It is **not** visual inspection, industrial acceptance or proof of CAD accuracy. Imported USD must be parsed and its declared subassemblies actually found.

## No claim of perfect physics without experiments

Every mechanical model needs independent analytic/experimental test fixtures with units, tolerance, instrument uncertainty, solver settings, timestep convergence and error curves. Validate contacts, joint limits, friction, sustained constraints, collision dynamics, energy behavior, robot trajectories and sensor timing where supported. Fluids, deformables, thermals and semiconductor processes need *separate* verified models.

Reproducibility does not prove physical validity. A valid USD PhysicsScene does not imply a physical solver has run. A passing rigid-body fixture is not evidence of accurate robot assembly or machine commissioning. Never expose an uncommissioned virtual PLC as authorized hardware control.

## Architecture

- OpenUSD scene data: non-destructive references, layers, variants, units, semantic identity.
- Rendering: separate GPU-specific backend and quality profiles.
- Physics: separate fixed-step solver, validated contact/collision/inertia and physical control.
- Robotics: separate links/joints, calibrated actuators, controllers, trajectory tracking and ROS bridges.
- Sensors: timestamped RGB/depth/LiDAR/IMU/force simulation with calibrated noise.
- Industrial: machine-specific control logic and measured operating envelopes.
- Assurance: model and output provenance, numerical acceptance, real hardware benchmarks.

Never ship proprietary OEM CAD, credential files or unlicensed materials in a public repository. Review NVIDIA runtime redistribution terms separately.

## Definition of done

No feature is designated production-ready without passing code CI, native runtime tests, representative RTX 3050 hardware acceptance, visual reference comparison, numerical reference residual limits, security review, documentation and relevant real-world domain acceptance.
