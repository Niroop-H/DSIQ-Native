# Head-to-head feature matrix and development gates

**Objective:** compete in industrial authoring, interoperability and robotics. **Status:** pre-alpha foundation, NOT current Omniverse/Isaac Sim parity.

| Capability | Omniverse / Isaac-class target | Current DSIQ Native | Required gate |
|---|---|---|---|
| OpenUSD | Layers, references, payloads, variants, semantics | Metric stage creation, real asset attachment, new layered assembly workspace, external references, representation variants and separate pose edits | Layered roundtrip, performance, safe collaboration and full schema conformance |
| Native editor | Scene tree, property tools, undo/redo, multi-view | Functional Bullet hierarchy/transform inspector, layout Undo/Redo, workcell project loading and revision export; Kit extension experimental | 3D transform gizmos, full CAD assembly constraints, GPU renderer, hardware qualification |
| RTX renderer | Physically based lighting, path tracing, denoising | Not implemented independently; Kit host may supply renderer | RTX 3050 real-device frame and VRAM evidence |
| Materials/CAD | Real assemblies, calibrated PBR, CAD translation | USD files plus provenance/rights gate only | Manufacturer- or expert-approved CAD and materials |
| Rigid/soft/fluid physics | PhysX/advanced mechanics | Actual CPU Bullet rigid bodies, contacts, articulated reference robot, USD PhysicsScene schemas; PhysX runtime not qualified | Independent numeric / experimental benchmarks, GPU PhysX, soft/fluid models |
| Robots and control | Articulations, ROS2, IK, dynamics, motion | Actual 6R FK/Jacobian/IK, USD joints, Bullet motors, verified URDF import; read-only ROS2 message preview and optional publisher | Calibrated OEM robot, collision planning, tested ROS2 DDS/TF and controllers |
| Sensors | RTX RGB/depth, semantic, LiDAR, contact, IMU | CPU Bullet RGB-D, segmentation and geometric LiDAR + link velocities; no RTX sensing | GPU sensor ground truth, optical/noise calibration, time sync |
| Isaac-style learning | Massive GPU RL, domain randomization | New seeded CPU Bullet RGB-D/segmentation dataset workflow; actual RL training still absent | RTX photorealism, calibrated sensors, policy learning, tensorized parallel simulation |
| Industrial twins | Equipment semantics, processes, live telemetry | Asset manifest and authoring foundation | Commissioned data bindings and domain validation |
| 16GiB RAM / RTX3050 | Selective graceful operation with constrained scene | Advisory GPU budget; standalone CPU software-rendered studio, headless Windows/Linux CI; RTX3050 GUI untested | Actual workstation RAM traces, GUI/fps/accuracy acceptance |
| Shipping | Signed, supported Windows installer | Not implemented | Clean-room install, license/redistribution review |

## Every workcell policy and editor progress

- A versioned workcell schema now defines **required mechanical roles**
  and **ten evidence categories for each individual workcell**.
- Workcell catalogs hash all referenced projects and can run sequential
  real Bullet physics audits for every entry, not only a selected demo.
- A native scene hierarchy editor can inspect actual Bullet entities,
  move imported assemblies in authoring mode and Undo/Redo real
  scene-transform changes. A workcell project can be loaded
  transactionally and saved as a new revision.
- This is a significant **authoring and quality-gate foundation**, but
  not a complete professional multi-viewport CAD modeler. There are
  no edit gizmos, precise CAD assembly mates, RTX renderer or signed
  external certification.
- Missing tooling, fixtures, safety equipment and independent evidence
  remain blocking results; none are silently fabricated.

## New tested/reference integration boundary

- Three additional isolated physical measurements: contact stability across
  timesteps, friction-sensitive sliding, and imported actuator speed/travel
  constraints. These are **not** certified industrial physics.
- Read-only ROS 2 JointState/LaserScan/Clock conversions from actual
  Bullet state; optional live rclpy publisher if ROS 2 is separately
  installed. CI exercises offline contracts; real DDS middleware,
  TF trees and controller interoperability are not yet qualified.
- No ROS subscribers or industrial actuation paths have been enabled.
- Remaining gaps include ROS2 TF/URDF compatibility, actual ROS2 runtime
  acceptance on targeted platforms, calibrated contact friction/CCD,
  hardware-in-the-loop trials, and independently measured graphics budgets.

## Gap-closing work initiated

- **OpenUSD workflow:** real authored composition projects use stronger
  layout and weaker equipment layers, verified external geometry
  references, user-edited transforms, and source/low-memory USD variants.
  This is not Omniverse's complete collaborative layer stack.
- **Synthetic data:** real CPU Bullet RGB-D/segmentation capture, randomized
  starting conditions and physical parameters, immutable checksummed
  image/depth files, and dataset integrity verification.
  No RTX optics, learned policies, or calibrated sensor behavior yet.
- **Remaining P0/P1:** real OEM CAD assemblies, higher-fidelity renderers,
  measured GPU budgets on RTX 3050, ROS 2 lifecycle integration, calibrated
  physics, continuous collision safety and hardware acceptance.
- **Remaining P2:** Isaac Lab-style batched robotics learning, multiphysics,
  cloud collaboration, large-scene streaming, secure commercial installer.

## Most recent verified engineering additions

- Quantitative Bullet reference experiments: gravity, resting contacts,
  torque/inertia and repeatability with machine-readable tolerances.
- User-authored XYZ gantry project generator: editable dimensions, actual
  URDF structural assembly, three prismatic motors, bounded source
  effort/speed, SHA-256 manifest and real Bullet actuation.
- Named robotics cell, cleanroom and logistics reference architecture,
  with solid collision classifications, equipment/safety semantics
  and a bounded CPU software renderer.
- Native 6DOF free-flight camera with keyboard and mouse controls,
  integrated into the black/light desktop.
- OpenUSD facility export with metric transforms, semantic roles,
  static collision schemas and PBR Preview Surface material inputs.
- These do NOT constitute manufacturer-exact CAD, RTX visuals, calibrated
  physics, offline robot path safety or complete Isaac Sim parity.

## Milestone sequence

M0 — **Now:** establish verifiable OpenUSD stage, robust import/rights gate, hardware budget, Kit diagnostic extension and CI. These have code but execution on target GPU is not yet established.

M1 — **Native viewport:** Install official Kit SDK externally, launch an actual DSIQ-branded app, test on RTX3050. Implement low-memory scene loading, stage authoring, undo/redo, materials, real CAD assembler, asset browser.

M2 — **Real robotics workcell:** PhysX or another genuinely native solver, articulation/joint models, measured contacts, URDF/MJCF ingestion, reference robot CAD with lawful asset provenance, a robot test suite.

M3 — **RTX/sensors:** Physically derived materials, selective RTX, scene streaming, LOD/instancing, calibrated camera/depth/sensor stack, captured benchmarks on RTX3050.

M4 — **Industrial domain engines:** Specific machines and industry process models with manufacturer/academic acceptance evidence and failure-analysis metrics; never label every machine 'validated' based on a generic process loop.

M5 — **Commercial release:** CI/CD, signing, install/update, protection boundaries, usability studies, customer pilot qualification and independent audit.

Any claim of head-to-head superiority needs matched benchmark scenes, shared fidelity criteria, resource-normalized costs, numerical error reporting and transparent limitations.
