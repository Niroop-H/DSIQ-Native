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

## What this initial commit implements

- Standard-library, platform-aware NVIDIA GPU and RAM inspection with conservative, *estimated* application-asset budgets and clear warnings.
- An OEM/asset manifest validation gate: SHA-256 checks, source provenance, licensing evidence, exact CAD files and physics-evidence classification; missing files fail closed.
- Headless OpenUSD scene creation, inspection, and reference attachment via Pixar's `pxr` Python bindings (requires `usd-core` or a compatible Kit Python environment).
- An optional **Kit SDK Python extension** displaying real properties of the active USD stage. It runs *inside a separately installed NVIDIA Kit Base Editor*; it does **not** ship NVIDIA's proprietary SDK binaries.
- Python unit/integration tests and GitHub CI scaffolding.
- Competitive-feature, hardware and engineering-acceptance specifications.

**Not yet built:** proprietary/native RTX renderer integration, PhysX simulation graphs, URDF/MJCF import, ROS 2 bridges, validated robot dynamics, RGB/depth/RTX sensor stack, detailed manufacturer-approved CAD models, production Windows packaging. These are explicit milestones, not hidden stubs labeled complete.

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
