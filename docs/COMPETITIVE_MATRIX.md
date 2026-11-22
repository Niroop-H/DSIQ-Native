# Head-to-head feature matrix and development gates

**Objective:** compete in industrial authoring, interoperability and robotics. **Status:** pre-alpha foundation, NOT current Omniverse/Isaac Sim parity.

| Capability | Omniverse / Isaac-class target | Current DSIQ Native | Required gate |
|---|---|---|---|
| OpenUSD | Layers, references, payloads, variants, semantics | Metric stage creation, inspect and referenced real asset attachment | Real USD roundtrip, schema conformance |
| Native editor | Scene tree, property tools, undo/redo, multi-view | Working standalone Tk CPU robotics studio with live viewport and black/light themes; optional Kit extension unverified on target GPU | Full editor, undo/redo, native Kit RTX device qualification |
| RTX renderer | Physically based lighting, path tracing, denoising | Not implemented independently; Kit host may supply renderer | RTX 3050 real-device frame and VRAM evidence |
| Materials/CAD | Real assemblies, calibrated PBR, CAD translation | USD files plus provenance/rights gate only | Manufacturer- or expert-approved CAD and materials |
| Rigid/soft/fluid physics | PhysX/advanced mechanics | Actual CPU Bullet rigid bodies, contacts, articulated reference robot, USD PhysicsScene schemas; PhysX runtime not qualified | Independent numeric / experimental benchmarks, GPU PhysX, soft/fluid models |
| Robots and control | Articulations, ROS2, IK, dynamics, motion | Actual 6R FK/Jacobian/IK, USD joints, Bullet motors, verified URDF inspection/import; ROS2 not yet implemented | Real calibrated OEM robot, trajectory tests, collision planning, ROS2 |
| Sensors | RTX RGB/depth, semantic, LiDAR, contact, IMU | CPU Bullet RGB-D, segmentation and geometric LiDAR + link velocities; no RTX sensing | GPU sensor ground truth, optical/noise calibration, time sync |
| Isaac-style learning | Massive GPU RL, domain randomization | Not implemented | Policies, repeatability and throughput tests |
| Industrial twins | Equipment semantics, processes, live telemetry | Asset manifest and authoring foundation | Commissioned data bindings and domain validation |
| 16GiB RAM / RTX3050 | Selective graceful operation with constrained scene | Advisory GPU budget; standalone CPU software-rendered studio, headless Windows/Linux CI; RTX3050 GUI untested | Actual workstation RAM traces, GUI/fps/accuracy acceptance |
| Shipping | Signed, supported Windows installer | Not implemented | Clean-room install, license/redistribution review |

## Milestone sequence

M0 — **Now:** establish verifiable OpenUSD stage, robust import/rights gate, hardware budget, Kit diagnostic extension and CI. These have code but execution on target GPU is not yet established.

M1 — **Native viewport:** Install official Kit SDK externally, launch an actual DSIQ-branded app, test on RTX3050. Implement low-memory scene loading, stage authoring, undo/redo, materials, real CAD assembler, asset browser.

M2 — **Real robotics workcell:** PhysX or another genuinely native solver, articulation/joint models, measured contacts, URDF/MJCF ingestion, reference robot CAD with lawful asset provenance, a robot test suite.

M3 — **RTX/sensors:** Physically derived materials, selective RTX, scene streaming, LOD/instancing, calibrated camera/depth/sensor stack, captured benchmarks on RTX3050.

M4 — **Industrial domain engines:** Specific machines and industry process models with manufacturer/academic acceptance evidence and failure-analysis metrics; never label every machine 'validated' based on a generic process loop.

M5 — **Commercial release:** CI/CD, signing, install/update, protection boundaries, usability studies, customer pilot qualification and independent audit.

Any claim of head-to-head superiority needs matched benchmark scenes, shared fidelity criteria, resource-normalized costs, numerical error reporting and transparent limitations.
