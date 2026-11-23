"""Unified, independently executable acceptance gate for DSIQ's three core APIs.

The gate tests actual CPU physics experiments, a source-verified imported
robot with joint-motion and discrete collision checks, and OpenUSD layer
plus measured animation roundtrip. Passing checks a DELIMITED software
contract, not all physical science or industrial machine readiness.
"""
from __future__ import annotations

import math
from pathlib import Path
from tempfile import TemporaryDirectory

from .bullet_runtime import BulletPlayground,BulletSettings
from .physics_benchmarks import run_physics_suite
from .physics_conservation import run_conservation_suite
from .physics_fidelity import run_extended_physics_suite
from .machine_designer import MachineDesign,save_project
from .imported_trajectory import (
    plan_imported_trajectory,execute_imported_trajectory
)
from .motion_collision import screen_imported_path
from .usd_telemetry import record_bullet_usd,inspect_bullet_usd
from .usd_composition import new_workspace,inspect_workspace


def inspect_core_readiness() -> dict:
    base=run_physics_suite()
    extended=run_extended_physics_suite()
    conserved=run_conservation_suite()
    physics_ok=base["passed"] and extended["passed"] and conserved["passed"]
    with TemporaryDirectory() as temp:
        root=Path(temp)
        project=save_project(
            MachineDesign(asset_id="core_acceptance_reference_gantry"),
            root/"reference_mechanism",
        )
        layer=new_workspace(root/"usd_workspace")
        workspace=inspect_workspace(root/"usd_workspace")
        with BulletPlayground(BulletSettings(
            payload_count=0,gravity_m_s2=0.
        )) as world:
            imported=world.import_urdf_robot(
                project["manifest"],base_position_m=(-1.,1.3,0.),
            )
            plan=plan_imported_trajectory(
                world,imported["asset_id"],
                {"actuator_x":.20,"actuator_y":.10},
            )
            clearance=screen_imported_path(
                world,plan,clearance_m=.003
            )
            # This run refuses to execute after any sampled collision.
            if not clearance["collision_screen_passed"]:
                raise RuntimeError(
                    "Core accepted reference mechanism unexpectedly "
                    "collides in its robot joint test plan"
                )
            tracking=execute_imported_trajectory(
                world,plan,record_every=24,
            )
            scene=root/"measured_kinematics.usda"
            capture=record_bullet_usd(
                world,scene,frames=3,steps_per_frame=8,
            )
        observed=inspect_bullet_usd(scene)
        robotics_ok=(
            len(plan.names)==3 and len(plan.samples)>2
            and clearance["collision_screen_passed"]
            and math.isfinite(tracking["max_joint_tracking_error"])
            and tracking["final_actual"]["actuator_x"]>.01
            and tracking["final_actual"]["actuator_y"]>.005
            and not tracking["joint_limit_violations"]
        )
        openusd_ok=(
            workspace["asset_count"]==0
            and len(workspace["layer_stack"])==2
            and capture["captured_frames"]==3
            and observed["pose_tracks"]>=12
            and observed["joint_tracks"]>=2
            and len(observed["keyframe_times"])==3
            and observed["meters_per_unit"]==1.0
            and observed["up_axis"]=="Z"
        )
        checks={
            "rigid_body_benchmark_suite":base["passed"],
            "contact_friction_motor_suite":extended["passed"],
            "conservation_refinement_suite":conserved["passed"],
            "verified_imported_robot_motion":robotics_ok,
            "discrete_collision_preflight":clearance["collision_screen_passed"],
            "layered_openusd_authoring":len(workspace["layer_stack"])==2,
            "measured_openusd_animation_roundtrip":openusd_ok,
        }
        return {
            "schema":"dsiq-three-core-acceptance-v1",
            "all_defined_software_gates_passed":all(checks.values()),
            "software_gates":checks,
            "physics":{
                "reference_case_count":base["case_count"]+
                    extended["case_count"]+conserved["case_count"],
                "passed":physics_ok,
                "engine":"pybullet_cpu",
            },
            "robotics":{
                "imported_asset_id":imported["asset_id"],
                "joint_count":imported["joint_count"],
                "motor_plan_samples":len(plan.samples),
                "screened_samples":clearance["evaluated_samples"],
                "max_tracking_error":tracking["max_joint_tracking_error"],
                "source_velocity_limits_obeyed_analytically":True,
                "continuous_collision_qualified":False,
                "hardware_qualified":False,
            },
            "openusd":{
                "workspace_layers":workspace["layer_stack"],
                "time_sampled_pose_tracks":observed["pose_tracks"],
                "time_sampled_joint_tracks":observed["joint_tracks"],
                "keyframes":observed["keyframe_times"],
                "has_manufacturer_geometry":False,
                "renderer_qualified":False,
            },
            "industrial_release_ready":False,
            "remaining_blockers":[
                "independent_machine_CAD_and_inertial_calibration",
                "continuous_collision_detection_and_self_collision_certification",
                "torque_and_acceleration_constrained_planning",
                "deformable_fluid_and_domain_specific_physics_if_required",
                "fully_qualified_visual_openusd_assembly_motion_binding",
                "calibrated_physical_sensor_and_real_ROS2_interoperability",
                "RTX_3050_16GB_RAM_hardware_performance_qualification",
                "external_operational_safety_signoff",
            ],
            "interpretation":"Passing refers to THESE narrow integration tests, "
                "not every physics model, robot family or OpenUSD feature.",
        }
