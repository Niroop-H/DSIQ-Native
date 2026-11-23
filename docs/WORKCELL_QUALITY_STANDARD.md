# DSIQ Native — Universal Robotics Workcell Quality Standard

## Product rule: every workcell, not one demonstration

DSIQ must support multiple independently useful robotics workcells.
**Every workcell goes through exactly the same versioned validation**.
No model may bypass required evidence because it is familiar, popular
or visually impressive.

A passing checksum or software test is NOT proof of industrial accuracy.
This tool deliberately never returns an industrial-certification status.
The available reports are REJECTED, INCOMPLETE, and in future situations
PENDING_INDEPENDENT_REVIEW. Evidence is not assumed to be correct merely
because a file exists.

## Required content of EVERY workcell

The DSIQ workcell file includes stable identity and revision, SI units,
an engineering environment, and 1–4 verified URDF assemblies in the
constrained current CPU runtime. Each assembly declares its role, local
source manifest, manifest SHA-256 and metric XYZ base pose.

All workcells must include actual **robot, tooling, fixture and safety**
roles. The robot must have at least one real articulated joint. The
simulator must identify a supported sensor contract. Invented sensor
names and duplicate equipment IDs are rejected.

The ten mandatory engineering evidence categories are:

| Gate | Engineering requirement |
|---|---|
| geometry_cad | Real dimensions and assembly hierarchy; rights and geometry verification |
| inertial_parameters | Measured masses, centers of mass, inertia and uncertainty |
| mechanism_kinematics | Verified frames, joint limits and kinematic calibration |
| contact_dynamics | Contact, friction, restitution and load-case validation |
| actuator_controls | Actuator ratings, control limits, measured response and safety |
| sensors | Calibrated cameras, LiDAR, force/torque and synchronization |
| safety | Guards, interlocks, E-stop, risk analysis and expert sign-off |
| rendering_materials | Accurate CAD visualization, PBR materials, measured references |
| resource_performance | Actual CPU/GPU memory, FPS and solver timing |
| reproducibility | Repeatable benchmarks, versions and reproducible evidence |

A gate may refer to an evidence file with a SHA-256 checksum. The
validator confirms that its file is present and unmodified. It marks
this as FILE_HASH_VERIFIED_ONLY; it never assumes a self-submitted
file establishes independent technical review or certification.

## Create, inspect, simulate

Assume an existing verified URDF machine is under the workcell
project folder, e.g. machine/machine.manifest.json.

~~~powershell
python -m pip install -e ".[robotics]"
python -m dsiq_native workcell-new cell_a.workcell.json --id cell_a_robotics --name "Robotics assembly cell A" --manifest machine/machine.manifest.json
python -m dsiq_native workcell-inspect cell_a.workcell.json
python -m dsiq_native workcell-audit cell_a.workcell.json --steps 120
~~~

A reference workcell can run physics while still being INCOMPLETE.
Missing safety equipment, real tooling, evidence or source validation
are never silently filled in with dummy boxes or claimed correct.

## Apply the same checks to MANY workcells

~~~powershell
python -m dsiq_native workcell-catalog-build my-workcells.json cell_a.workcell.json cell_b.workcell.json cell_c.workcell.json
python -m dsiq_native workcell-catalog my-workcells.json
python -m dsiq_native workcell-catalog my-workcells.json --simulate --steps 120
~~~

A catalog checks every workcell, not just the first entry. It stores
SHA-256 hashes of each file, checks unique IDs and rejects changed
files until the changes have been explicitly versioned. The current
constrained runner supports 64 workcells per catalog batch and loads
one workcell at a time to bound memory. Multiple catalogs can be
used for larger robotics programs.

## Native editor

Run with the black or light theme:

~~~powershell
python -m dsiq_native studio --theme black
~~~

The scene hierarchy inspector displays actual Bullet machine bodies
and payloads, provenance class and XYZ transforms. Editable assets
support real simulation-world position changes, Undo and Redo.
These are **authoring teleports**, not commanded robot movements.

Workcells menu:
- Open a provenance-checked workcell project transactionally.
- Audit the loaded workcell using a separate Bullet physics client.
- Save a new authored layout revision without overwriting the original.
- Open the scene tree and transform inspector.

This implements a functional editor foundation. It is not yet a full
professional CAD modeling tool with viewport gizmos, assembly mates,
feature modeling, multi-user locking or RTX rendering.

## Real constraints and missing qualification

- A workcell is limited to four imported URDF assemblies in the
  current CPU engine. This is a runtime resource bound, not a claim
  that industrial facilities have only four pieces of equipment.
- Registered workcells are not automatically OEM-accurate. Every
  real machine needs lawful detailed assets, expert review, measured
  dynamics and physical experiments.
- A green CI build proves the tested functions run on CI systems.
  It does not prove manufacturer physics, machine safety, RTX 3050
  performance or real robot commissioning.
- No physical robot control is enabled by this workcell framework.
- A future industrial qualification system needs independent signed
  reviewers and per-model numerical acceptance criteria, not merely
  more JSON documents.

The policy applies to **every supported workcell** and is designed to
reject unsupported claims rather than rewarding a polished demo.
