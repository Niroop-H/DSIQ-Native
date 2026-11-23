# DSIQ Native — Evidence-first Omniverse competition plan

**Basis:** User-supplied *DSIQ Native: Review, and a Roadmap to Compete with NVIDIA Omniverse*, dated 23 November 2026, originally reviewing commit 853d6f3. This plan converts its engineering recommendations into measurable acceptance gates rather than claiming general competitive parity.

## Full-feature program direction — user-confirmed

DSIQ Native targets **comprehensive feature parity across NVIDIA Omniverse capability families**, not only a small subset of auditable tooling. Engineering tracks are developed in parallel while release and qualification gates remain evidence-driven. Source-verified assets and vendor portability remain product advantages, not reasons to exclude rendering, robotics, AI, sensors, collaboration or advanced infrastructure.

See the 20-family parallel execution and acceptance matrix: [OMNIVERSE_FULL_PARITY_MASTER_MATRIX.md](OMNIVERSE_FULL_PARITY_MASTER_MATRIX.md).

## The differentiated product

A comprehensive Omniverse-class engineering platform built around portable OpenUSD, modular physics/rendering/AI services, auditable assets, local-to-cluster execution, and built-in qualification evidence.

This is an engineering target, **not** a claim of current feature parity or equal performance across NVIDIA, AMD and Intel GPUs. Demonstrate it through matched scene, numerical fidelity, hardware usage and release metrics, including failures.

## Design constraints to preserve

- Keep legal asset origin, binary hashes, source units and fidelity labels tied to each real machine; never quietly replace a missing OEM model with a box.
- OpenUSD owns scene composition (metric, Z-up, references, sublayers, variants and animation); avoid proprietary scene-format lock-in.
- Separate the rendered visual, executable numerical model and independently calibrated physical twin.
- Make scene editing, physics, sensors, graphics and data useful as independently installable modules. Kit is an optional interoperability bridge only.
- Work toward **one finished workcell pipeline** before expanding to additional industrial domains.

## Phase 0 — foundation, in progress

| Workstream | Implemented or identified | Exit evidence |
| --- | --- | --- |
| Documentation | A concise five-minute root README with full original history preserved | First-time engineer installs and creates/tests one local workcell |
| Physics | An isolated real Bullet adapter under a solver-agnostic API; a named backend selector and two-world conformance runner | Linux and Windows conformance runs produce actual solver state, source hash and repeatability |
| Packaging risk | Windows dependence on separately published pybullet-arm64 3.2.8 | Publisher/source/wheel provenance, licence and bundled DLL review before commercial build |
| Licensing | No LICENSE grant currently tracked, no implicit open-source rights | MAH Quantum owner selects a deliberate licence, then add notices and dependency attribution |
| App architecture | CLI and desktop module still need decomposition | Incremental behavior-preserving extractions and verified existing GUI workflow |

## Phase 1 — prove the evidence wedge

1. Ship the OpenUSD/asset validator as a clean library/CLI with pluggable rule sets, mesh sanity, materials, physics schema and missing reference checks.
2. Expand the physics protocol for actual USD Physics mapping: articulation state, contacts and physically stable timestep. Do not falsely advertise USD support before implemented.
3. Integrate at least one second genuine solver (MuJoCo if the current licensing/build situation supports it) and execute matched fixtures against real Bullet.
4. Release benchmark scene hashes, environment metadata, residuals, failure logs and reproducible scripts.

**Exit:** An independent engineer adopts the standalone validator or conformance suite without adopting the whole studio.

## Phase 2 — actual portable advanced viewport

- Validate a portable Vulkan + Hydra render delegate design. First implement correct raster PBR, materials, shadows and stable picking; later investigate ray tracing and CPU fallback.
- Use authored payloads, instancing rules and LOD rather than silently merging incompatible machine geometry.
- Measure FPS, frame time distribution, peak GPU VRAM, process RAM, scene-load latency and device errors on 4/6/8 GiB tiers.
- Do not remove the already-working Panda3D native editor until replacement capability and evidence exist.

**Exit:** Interactive multi-hundred-thousand-triangle OpenUSD engineering scene on the target RTX 3050/16 GiB device with publishable traces.

## Phase 3 — robotics and physical-AI fidelity

- Add source-verified MJCF/SDF imports and reliable solver conformance as a gate for MuJoCo/Newton, not merely an option in the UI.
- Calibrated noise optics/IMU/LiDAR and time-sync semantics; sensor outputs must correspond to the scene being rendered.
- Reproducible randomization, annotation formats and seed-controlled training benchmark with held-out tasks.
- ROS 2 read-only telemetry first; any external physical actuation requires independent commissioned protective systems.

**Exit:** A third party reproduces a real policy benchmark and can identify measured sim-to-real limitations.

## Phase 4 — collaborative engineering and operations

- Self-hosted USD layer-sync with revision conflict detection, permissions, locking, and user-visible resolution.
- Read-only OPC UA/MQTT bindings to source-authenticated measurements and typed USD process attributes.
- Verify independent two-user editing and one genuine source-fed twin; fail visibly on stale measurements.

**Exit:** Two authorized users can safely edit a common stage; one calibrated source feeds a real operational twin.

## Phase 5 — commercialization and independent qualification

- Signed Windows distribution (MSIX/MSI as appropriate), SBOM/licensing, deterministic release evidence and protected updates.
- Real OEM source rights, measured machine parameters and instrumentation uncertainty, separate safety review, and customer acceptance.
- Record all eight GitHub P0/P1 qualification gates. The existing **42/100 is a provisional judgment**; only a new evidence-backed assessment may alter it.

**Exit:** A bounded, independently accepted workcell deployment. This does not assert universal industrial safety or NVIDIA parity.

## Parallel execution and integration gates

The phase labels remain verification milestones, **not a requirement to wait before prototyping later capability families**. Vulkan/Hydra, Physical AI, sensors, collaboration and streaming may be developed concurrently when their acceptance boundaries are explicit; integration into the supported native application requires reproducible test evidence.

## Explicit stop rules

Do not simultaneously claim unfinished physics solvers, renderers or application frameworks are shipping production features. Do not market a Kit extension as independent rendering. Do not infer physical safety from software stop latches or discrete collision sampling. Do not claim universal Vulkan support or hardware performance from Mesa CI. Do not add more domains before one machine's data-to-screen-to-simulation workflow is proven.

See the qualification plan, source URDF documentation, OpenUSD composition design and GitHub Actions evidence. This roadmap provides relative priority and acceptance evidence, not promised delivery dates.
