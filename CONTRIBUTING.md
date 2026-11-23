# Contributing to DSIQ Native

DSIQ Native is an engineering pre-alpha project maintained under MAH Quantum / Devashri S Industries direction.

**Rights notice:** A repository-level outbound software licence has not yet been selected. Public access to source code does not by itself authorize copying, modification or redistribution. Before accepting externally authored code or commercial distribution, the owner must formally select a licence and approve contributor terms. Please open an issue to discuss the intended change before submitting patches; do not assume rights to third-party CAD, equipment files, pretrained models or proprietary datasets.

## Development workflow

Use Python 3.11 and isolate dependencies in a virtual environment. Make changes only to the `Niroop-H/DSIQ-Native` repository; preserve its documented headless, Windows and native OpenGL pathways.

- Define a specific engineering claim and explicit acceptance threshold before implementing new code.
- Separate idealized fixtures, user-supplied source assets and independently calibrated equipment.
- Include deterministic unit/regression tests, measured runtime tests where appropriate, and negative tests for unsafe or malicious inputs.
- Run `python -m unittest discover -s tests -v` and the affected optional integration suites.
- For motion/physics changes, retain real source URDF limits and explicit fixed-step clocks. A simulated stop is not an E-stop.
- For USD changes, preserve authoring composition, geometry identity, upstream source rights and material/physics semantics. Do not auto-merge collisions or manufacture fictitious OEM assets.
- Add no runtime SDK/vendor dependency without a licence/provenance review and predictable failure modes.
- Document unsupported devices, skipped test categories and external evidence requirements; do not inflate industrial readiness scores.

## Acceptance

Required CI must pass on the supported platforms, and the evidence should name its commit SHA. External customers and safety-critical users still require physical calibration and an independent risk/safety process.

## Issue topics before release

- Select and document repository software licensing and inbound contribution policy.
- Review transitive licence compatibility and binary provenance of third-party physics/graphics packages.
- Establish benchmark scenes, matching hardware and measurable acceptance gates for alternate solvers and renderers.
