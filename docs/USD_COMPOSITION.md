# DSIQ Native — Layered OpenUSD assembly workspace

This closes part of the gap with NVIDIA Omniverse's OpenUSD composition
workflows. It is a genuine layered USD project, not a substitute for
NVIDIA Kit rendering, physics or industrial CAD translation.

## Project structure

    workspace.json
    project.usda
    layers/equipment.usda
    layers/layout.usda

The OpenUSD root loads the layout layer stronger than equipment.
Verified equipment is authored as external USD references; transforms
are authored separately. Sources remain referenced rather than
flattened into boxes. The scene is canonical Z-up, 1 meter/unit.

## Usage

~~~powershell
python -m pip install -e ".[usd]"
python -m dsiq_native usd-workspace-new my-usd-project

# Real file must have DSIQ asset manifest, source USD and rights evidence:
python -m dsiq_native usd-workspace-import my-usd-project assets/robot/asset.manifest.json

# Two real USD geometry sources can become source/low-memory variants:
python -m dsiq_native usd-workspace-import my-usd-project assets/source/asset.manifest.json --low-memory-manifest assets/low/asset.manifest.json

# Authored workspace properties are written into dedicated USD layers:
python -m dsiq_native usd-workspace-place my-usd-project machine_id --xyz 1.0 -0.5 0.7 --yaw 30
python -m dsiq_native usd-workspace-variant my-usd-project machine_id low_memory
python -m dsiq_native usd-workspace-inspect my-usd-project
~~~

## Asset checks

Every imported source must pass provenance/rights evidence and SHA-256
checks, canonical units/up-axis verification, declared real
subassembly paths, and actual triangulable USD mesh topology.
The low-memory asset must have no more triangles than the source.

The authoring limit is 32 scene asset placements and at most 250,000
estimated triangles per representation. This is a conservative
admission policy, not a measured system or GPU RAM guarantee.

A test fixture is not represented as production/OEM equipment.
Valid file checks do not establish physical/dimensional correctness.

## What is not solved

No multiuser collaboration, cloud synchronization, advanced assembly
editing, general CAD conversion, independent RTX rendering, real-time
PhysX simulation or manufacturer-approved CAD libraries are included
with this module. The USD workspace can still be opened by compatible
external tools and future DSIQ backends.

Tests author actual OpenUSD fixture meshes and exercise persistent
sublayer composition, independently authored transforms, representation
switching, geometry visibility and invalid-source rejection.
