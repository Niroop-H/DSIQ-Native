# DSIQ Native — Verified URDF Robot Imports

This module moves DSIQ beyond one fixed reference manipulator by enabling
a real **user-supplied, licensed ROS URDF robot** to be inspected and loaded
into Bullet physics. Its links, joints, collision geometry and limits
come from the actual URDF, not from a fabricated generic CAD model.

It does **not** supply or impersonate any OEM geometry.

## Headless inspection

~~~powershell
python -m pip install -e ".[physics]"

# Parse actual URDF frames, inertial parameters and joint topology.
python -m dsiq_native urdf-inspect path\to\robot.urdf
~~~

The inspector checks:
- Up to 64 actual named links and 63 joints.
- Unique, connected acyclic parent/child tree with exactly one root.
- Fixed, revolute, continuous and prismatic joint semantics.
- Finite joint axes, origins, limits and dimensions in SI units.
- Local file references for STL, OBJ and DAE geometry (max 80 MiB/file).
- Mass and inertia tensor presence, with missing-data warnings.
- No XML DTD/entities, Xacro execution, plugin directives or remote assets.
- Constrained URDF size (2 MiB) and asset path traversal safeguards.

## Only verified asset manifests may execute

Add a dsiq-robot-urdf-v1 manifest *alongside a real robot URDF*, with:
- Exact model/manufacturer/revision and source/provenance URI.
- Original URDF SHA-256.
- Local file path for independently verifiable license/usage rights.
- asset_class: test_fixture, engineering_reference or oem_verified.
- For engineering/OEM class, real mesh files and an approved visual-review
  evidence document. A test fixture may use primitives, but must say so.

Example template (placeholder values are NOT valid until replaced with actual
evidence and a real file hash):

~~~json
{
  "schema": "dsiq-robot-urdf-v1",
  "asset_class": "oem_verified",
  "asset_id": "approved_robot_identifier",
  "manufacturer": "Actual manufacturer",
  "model": "Actual model name",
  "revision": "Actual revision",
  "source_uri": "actual source URL or internal reference",
  "license": {
    "usage_rights": "licensed",
    "evidence_file": "license-rights.txt"
  },
  "geometry": {
    "urdf_file": "approved-robot.urdf",
    "sha256": "ACTUAL_64_CHARACTER_SHA256"
  },
  "visual_review": {
    "approved": true,
    "reviewer": "Verified reviewer",
    "reviewed_against": "Verifiable actual reference",
    "evidence_file": "visual-acceptance.txt"
  }
}
~~~

On Windows, run Get-FileHash to obtain the actual URDF digest. The
manifest alone is not OEM or physical certification. It is a **provenance
and integrity gate**. Actual shape and machine acceptance remain external.

~~~powershell
python -m dsiq_native bullet-urdf my-robot.manifest.json --steps 240

# Apply a real imported actuated joint command.
# Target is radians for revolute joints, meters for prismatic joints:
python -m dsiq_native bullet-urdf my-robot.manifest.json --joint elbow --target 0.3 --steps 480
~~~

The runtime loads the URDF with Bullet's native loadURDF parser, with fixed
base, actual per-link geometry, declared inertias, joints and collision data.
Joint drive targets are bounded to their declared physical range; true
fixed and unsupported joints reject motor commands. Reports include actual
solver feedback for imported robot joint positions.

## Not supported yet

- ROS package:// URIs, Xacro macros and distant mesh references must be
  resolved in a separate reviewed offline build pipeline. The code rejects
  them rather than executing/importing them unsafely.
- Not a substitute for manufacturer robot-controller HMI or certified robot
  motion planning.
- No manufacturer-grade model is shipped; obtain legitimate OEM/public
  assets and licensing evidence separately.
- Collision shapes in URDF may be simplified, not exact machine hardware.
- Physics is numerical approximation. Success loading a URDF does not
  validate its masses, joint constraints, collision surface or gearbox.
- Malicious native mesh importers require stronger operating-system
  sandboxing. Only import files from trusted, authorized sources.
- The original Reference6R robot remains in the playground for comparison.
  Imported robots are separate Bullet bodies and use the original source
  geometry and joint frames.

## Acceptance program

- Rejection of XML entities and executable plugin elements.
- No parent traversal/external mesh loads.
- Verified SHA-256 + rights evidence before execution.
- Graph acyclicity and bounded joint count.
- Real Bullet load and motor movement for an expressly labeled
  non-OEM unit-test hinge.
- No manufactured sensor/physics qualification status.

This is functional import infrastructure to accept accurate external
robot equipment when its actual legal geometry and measurements are
available.
