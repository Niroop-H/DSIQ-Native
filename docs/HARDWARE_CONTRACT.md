# Hardware contract — RTX 3050 / 16 GiB system RAM

**Design target, not measured support.** DDR5 system RAM and GPU VRAM are separate. RTX 3050 laptop/desktop variants may have 4, 6, or 8 GiB VRAM; DSIQ detects the actual device through nvidia-smi.

Current NVIDIA published minimums (November 2026):
- Omniverse Kit: RTX 3070, 16 GiB RAM.
- Isaac Sim 6.0: RTX 4080 with 16 GiB VRAM, 32 GiB RAM.

Therefore **RTX 3050 is below both published GPU minimums**. DSIQ headless OpenUSD authoring can be developed independently, but Kit/RTX startup and native GUI responsiveness are experimental until measured. Device-specific failure must be visible, not disguised.

## Implemented advisory budget

The device module reports actual NVIDIA total/free VRAM, driver version and physical RAM. It reserves at least the greater of 1 GiB or one-third of GPU VRAM and budgets at most half of GPU VRAM for **DSIQ-owned asset caches**, subject to currently free memory. If less than 512 MiB can be reserved for owned assets, or RAM is below 12 GiB, it recommends headless-only mode.

This is NOT a guaranteed GPU cap: Kit, OS, driver, denoisers, acceleration structures, render targets and physics use additional memory. Accurate VRAM residency and hard admission checks are future work.

## Quality-preserving fallback rules

- GPU ray/path tracing, sensor count, image resolution and visible geometry may be reduced or disabled. Never quietly change mass, contact material, collision surfaces, integrator coefficients or controller timing to gain FPS.
- Separate visual CAD meshes from validated collision/robot kinematic representations. A cheaper visual mesh is not permission to replace precision collision geometry without evidence.
- Stream referenced USD assets; instance repeated geometry; retain OEM geometry and materials on disk; simplify display only with labeled, reviewed LODs.
- Stop or refuse a scene if it cannot meet the RAM/VRAM budget. Prefer an explicit unsupported result over a silent fallback to fictitious physics.

## Required acceptance tests (none yet completed on RTX 3050)

Test actual 4/6/8 GiB RTX 3050 SKUs where available, each with 16 GiB physical RAM and a real licensed industrial CAD scene. Record driver, scene hash, peak system RAM, peak/free GPU VRAM, geometry size, frame p50/p95, physics p95, numerical reference residuals, sensor accuracy and crash/OOM behavior.

Initial requirements: no OOM, graceful refusal of scenes that exceed resources, accurate replay of approved physics fixtures and full error reporting. Do not publish 30/60 FPS claims without independent real-device measurements.

Sources:
- https://docs.omniverse.nvidia.com/dev-overview/latest/common/technical-requirements.html
- https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/requirements.html
