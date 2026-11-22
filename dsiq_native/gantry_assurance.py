"""Independent kinematics checks for genuine user-authored XYZ gantry URDF.

Verify assembly coordinate frames in the *actual* Bullet importer against
closed-form machine geometry at several joint positions. This prevents
silently flipped axes or incorrect tool transforms. It is NOT dynamic
torque, contact or manufacturer calibration.
"""
from __future__ import annotations
import math
from pathlib import Path
from typing import Sequence

from .machine_designer import MachineDesign, load_design
from .urdf_bridge import validate_robot_manifest


def expected_tool_position(
    spec: MachineDesign,
    joint_positions_m: Sequence[float],
    base_position_m: Sequence[float] = (0.,0.,0.),
) -> tuple[float,float,float]:
    spec.validate()
    if len(joint_positions_m)!=3 or len(base_position_m)!=3:
        raise ValueError("XYZ gantry requires three actuator and three base coordinates")
    values=tuple(float(v) for v in joint_positions_m)
    base=tuple(float(v) for v in base_position_m)
    if any(not math.isfinite(x) for x in (*values,*base)):
        raise ValueError("Gantry coordinate values must be finite")
    for v, limit in zip(values,(spec.x_travel_m,spec.y_travel_m,spec.z_travel_m)):
        if not 0.<=v<=limit:
            raise ValueError("Requested gantry actuator outside configured stroke")
    x,y,z=values
    return (
        base[0]-spec.x_travel_m/2+x,
        base[1]-spec.y_travel_m/2+y,
        base[2]+spec.clearance_height_m-z-spec.z_travel_m/2-.17,
    )


def verify_gantry_in_bullet(
    project_directory: str | Path, *, tolerance_m: float = .0001
) -> dict:
    if not math.isfinite(tolerance_m) or not 1e-8<=tolerance_m<=.01:
        raise ValueError("Gantry kinematic tolerance out of range")
    folder=Path(project_directory).resolve()
    spec=load_design(folder/"machine.design.json")
    manifest,inspection=validate_robot_manifest(folder/"machine.manifest.json")
    if manifest["asset_id"]!=spec.asset_id:
        raise ValueError("Machine project URDF and design asset IDs differ")
    names={j.name for j in inspection.joints}
    required={"actuator_x","actuator_y","actuator_z","flange_attachment"}
    if names!=required:
        raise ValueError("Gantry URDF joint set does not match machine specification")
    from .bullet_runtime import BulletPlayground,BulletSettings
    test_positions=[
        (0.,0.,0.),
        (spec.x_travel_m*.20,spec.y_travel_m*.45,spec.z_travel_m*.65),
        (spec.x_travel_m*.85,spec.y_travel_m*.75,spec.z_travel_m*.15),
    ]
    samples=[]
    with BulletPlayground(BulletSettings(payload_count=0,gravity_m_s2=0.)) as world:
        loaded=world.import_urdf_robot(
            str(folder/"machine.manifest.json"),
            base_position_m=(0.,2.0,0.),
        )
        joints={j["name"]:j["joint_index"] for j in loaded["joints"]}
        body=loaded["body_id"]
        for xyz in test_positions:
            for name,q in zip(("actuator_x","actuator_y","actuator_z"),xyz):
                world.p.resetJointState(
                    body,joints[name],q,targetVelocity=0.,
                    physicsClientId=world.client,
                )
            flange_index=joints["flange_attachment"]
            pose=world.p.getLinkState(
                body,flange_index,computeForwardKinematics=True,
                physicsClientId=world.client,
            )
            observed=tuple(float(v) for v in pose[4])
            analytic=expected_tool_position(
                spec,xyz,base_position_m=(0.,2.0,0.)
            )
            error=math.dist(observed,analytic)
            samples.append({
                "commanded_actuators_m":list(xyz),
                "observed_Bullet_flange_m":list(observed),
                "analytic_flange_m":list(analytic),
                "absolute_position_error_m":error,
                "passed":error<=tolerance_m,
            })
    return {
        "asset_id":spec.asset_id,
        "mechanism":"3_axis_prismatic_inspection_gantry",
        "test_kind":"STATIC_KINEMATIC_GEOMETRY_NOT_DYNAMIC_CALIBRATION",
        "tolerance_m":tolerance_m,
        "samples":samples,
        "max_error_m":max(a["absolute_position_error_m"] for a in samples),
        "passed":all(a["passed"] for a in samples),
        "physically_qualified":False,
        "limits":"Kinematic frame agreement alone does not validate mass, contact, "
                 "motor response, machine rigidity, safety or actual CAD dimensions.",
    }
