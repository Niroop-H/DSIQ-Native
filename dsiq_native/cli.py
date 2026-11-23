"""DSIQ Native: auditable commands for OpenUSD, robotics and hardware.

Meaningful subcommands run headlessly without Kit or a proprietary renderer.
All results are machine-readable; validation and unsupported actions fail closed.
"""
from __future__ import annotations

import argparse
import json
import math
import sys

from .asset_quality import validate_asset
from .hardware import inspect_device
from .usd_stage import _existing_layer, _load, attach_asset, inspect_stage, new_stage
from .playground import (
    PlaygroundSettings, create_playground, playground_report, set_reference_joint_targets,
    spawn_box,
)
from .robotics import forward_kinematics, inverse_kinematics


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="dsiq-native",
        description="Native OpenUSD/robotics development platform (pre-alpha)",
    )
    sub = p.add_subparsers(dest="command", required=True)

    sub.add_parser("hardware", help="Detect NVIDIA GPU, VRAM and physical system RAM")
    asset = sub.add_parser("validate-asset", help="Verify actual geometry, hash and rights evidence")
    asset.add_argument("manifest")

    create = sub.add_parser("new-stage", help="Create metric Z-up USD without fabricated machinery")
    create.add_argument("scene")

    inspect = sub.add_parser("inspect", help="Inspect USD topology and physics-schema counts")
    inspect.add_argument("scene")
    inspect.add_argument("--max-prims", type=int, default=20000)

    attach = sub.add_parser("attach", help="Reference verified real USD assets")
    attach.add_argument("scene")
    attach.add_argument("manifest")

    playground = sub.add_parser("playground", help="Create an actual USD robotics playground")
    playground.add_argument("scene")
    playground.add_argument("--boxes", type=int, default=5)
    playground.add_argument("--payload-mass", type=float, default=1.0)
    playground.add_argument("--gravity", type=float, default=9.81)
    playground.add_argument("--no-robot", action="store_true")
    playground.add_argument(
        "--reset", action="store_true",
        help="Explicitly replace only /World/Playground in an existing USD stage",
    )

    box = sub.add_parser("spawn-box", help="Add one physical sandbox payload to the USD stage")
    box.add_argument("scene")
    box.add_argument("--xyz", type=float, nargs=3, default=(0.0, 1.0, 1.2))
    box.add_argument("--side", type=float, default=0.16)
    box.add_argument("--mass", type=float, default=1.0)

    report = sub.add_parser("playground-report", help="Report actual robot joint/collider schema")
    report.add_argument("scene")

    fk = sub.add_parser("robot-fk", help="Reference6R forward kinematics (degrees)")
    fk.add_argument("joint_degrees", type=float, nargs=6)

    ik = sub.add_parser("robot-ik", help="Reference6R position-only IK (meters)")
    ik.add_argument("xyz_m", type=float, nargs=3)
    ik.add_argument("--tolerance", type=float, default=0.001)
    ik.add_argument("--max-iterations", type=int, default=250)

    drive = sub.add_parser("robot-drive", help="Write six angular drive targets into a USD playground")
    drive.add_argument("scene")
    drive.add_argument("joint_degrees", type=float, nargs=6)
    bullet = sub.add_parser(
        "bullet-run",
        help="Run actual CPU Bullet robotics physics; --gui opens native 3D playground"
    )
    bullet.add_argument("--gui", action="store_true")
    bullet.add_argument("--steps", type=int, default=480)
    bullet.add_argument("--boxes", type=int, default=4)
    bullet.add_argument("--gravity", type=float, default=9.81)
    bullet.add_argument("--joint-degrees", type=float, nargs=6, default=[0.]*6)
    bullet.add_argument("--json-out", type=str, default=None)
    bullet.add_argument("--lidar-rays", type=int, default=0,
                        help="Collect ideal geometric Bullet LiDAR rays (4-360)")
    bullet.add_argument("--camera-png", type=str, default=None,
                        help="Save actual TinyRenderer CPU RGBA camera PNG")
    bullet.add_argument("--depth-npz", type=str, default=None,
                        help="Save metric depth and segmentation arrays")
    studio = sub.add_parser(
        "studio", help="Launch DSIQ-branded native CPU robotics GUI; no NVIDIA Kit needed"
    )
    studio.add_argument("--theme", choices=("black", "light"), default="black")
    studio.add_argument("--boxes", type=int, default=4)
    studio.add_argument("--width", type=int, default=480)
    studio.add_argument("--height", type=int, default=360)
    studio.add_argument("--fps", type=int, default=12)
    read_urdf = sub.add_parser(
        "urdf-inspect", help="Inspect real local URDF topology, geometry and joint limits"
    )
    read_urdf.add_argument("urdf_file")
    imported = sub.add_parser(
        "bullet-urdf", help="Load a rights-verified URDF into actual Bullet physics and run it"
    )
    imported.add_argument("manifest", help="Local dsiq-robot-urdf-v1 provenance manifest")
    imported.add_argument("--steps", type=int, default=240)
    imported.add_argument("--joint", type=str, default=None)
    imported.add_argument(
        "--target", type=float, default=None,
        help="Position target in radians (revolute) or meters (prismatic)"
    )
    plan = sub.add_parser(
        "robot-plan", help="Design bounded six-joint quintic trajectory (angles in degrees)"
    )
    plan.add_argument("--start", nargs=6, type=float, required=True)
    plan.add_argument("--goal", nargs=6, type=float, required=True)
    plan.add_argument("--duration", type=float, default=None)
    tracking = sub.add_parser(
        "bullet-trajectory", help="Actually run planned Reference6R joint trajectory in Bullet"
    )
    tracking.add_argument("--goal", nargs=6, type=float, required=True)
    tracking.add_argument("--duration", type=float, default=None)
    tracking.add_argument("--record-every", type=int, default=20)
    sub.add_parser(
        "physics-verify",
        help="Run actual Bullet gravity, contact, torque and repeatability acceptance fixtures"
    )
    make = sub.add_parser("machine-create",help="Design a functional parametric XYZ gantry")
    make.add_argument("output_directory")
    make.add_argument("--name",default="dsiq_xyz_inspection_gantry")
    make.add_argument("--x-travel",type=float,default=1.2)
    make.add_argument("--y-travel",type=float,default=.8)
    make.add_argument("--z-travel",type=float,default=.45)
    make.add_argument("--height",type=float,default=1.3)
    build = sub.add_parser("machine-build",help="Rebuild URDF from an edited machine design JSON")
    build.add_argument("design_json")
    build.add_argument("output_directory")
    environment = sub.add_parser(
        "environment-render",
        help="Render real Bullet semantic facility reference background into PNG"
    )
    environment.add_argument("preset",choices=["open","robotics_cell","cleanroom_bay","logistics_bay"])
    environment.add_argument("--steps",type=int,default=12)
    environment.add_argument("--png",type=str,default=None)
    env_usd = sub.add_parser(
        "environment-usd",
        help="Export semantic reference facility to real OpenUSD with collision and PBR"
    )
    env_usd.add_argument("preset",choices=["open","robotics_cell","cleanroom_bay","logistics_bay"])
    env_usd.add_argument("scene_file")
    mv = sub.add_parser(
        "machine-verify",
        help="Independently check 3-axis machine URDF link frames against actual Bullet FK"
    )
    mv.add_argument("project_directory")
    mv.add_argument("--tolerance",type=float,default=.0001)
    sd = sub.add_parser(
        "dataset-generate", help="Generate measured CPU Bullet RGB-D/segmentation dataset"
    )
    sd.add_argument("output_directory")
    sd.add_argument("--frames",type=int,default=8)
    sd.add_argument("--seed",type=int,default=1007)
    sd.add_argument("--width",type=int,default=160)
    sd.add_argument("--height",type=int,default=120)
    sd.add_argument("--boxes",type=int,default=3)
    sd.add_argument("--steps",type=int,default=72)
    sv = sub.add_parser(
        "dataset-verify",help="Verify every synthetic image and depth file SHA-256"
    )
    sv.add_argument("directory")
    wc=sub.add_parser("usd-workspace-new",
                      help="Create layered OpenUSD project (equipment and layout sublayers)")
    wc.add_argument("project_directory")
    wi=sub.add_parser("usd-workspace-import",
                      help="Import verified real USD mesh asset with optional LOD variant")
    wi.add_argument("project_directory")
    wi.add_argument("asset_manifest")
    wi.add_argument("--low-memory-manifest",default=None)
    wp=sub.add_parser("usd-workspace-place",
                      help="Author asset pose in independent OpenUSD layout layer")
    wp.add_argument("project_directory")
    wp.add_argument("asset_id")
    wp.add_argument("--xyz",nargs=3,type=float,required=True)
    wp.add_argument("--yaw",type=float,default=0.)
    ws=sub.add_parser("usd-workspace-variant",
                      help="Select actual source or low-memory USD representation")
    ws.add_argument("project_directory")
    ws.add_argument("asset_id")
    ws.add_argument("choice",choices=("source","low_memory"))
    wl=sub.add_parser("usd-workspace-inspect",
                      help="Inspect composed USD layers, actual mesh references and assets")
    wl.add_argument("project_directory")
    sub.add_parser(
        "physics-extended",
        help="Measure contact step stability, friction and actual URDF motor limits"
    )
    rp=sub.add_parser(
        "ros2-preview",help="Capture actual Bullet JointState/LaserScan/Clock fields without ROS 2"
    )
    rp.add_argument("--steps-per-frame",type=int,default=12)
    rp.add_argument("--lidar-rays",type=int,default=36)
    rp.add_argument("--range-m",type=float,default=5.)
    rs=sub.add_parser(
        "ros2-stream",help="Read-only live ROS 2 publishers (requires external rclpy)"
    )
    rs.add_argument("--frames",type=int,default=50)
    rs.add_argument("--steps-per-frame",type=int,default=12)
    rs.add_argument("--lidar-rays",type=int,default=36)
    rs.add_argument("--range-m",type=float,default=5.)
    wnew=sub.add_parser(
        "workcell-new",
        help="Create portable reference workcell project from a rights-verified URDF"
    )
    wnew.add_argument("project_file")
    wnew.add_argument("--id",required=True)
    wnew.add_argument("--name",required=True)
    wnew.add_argument("--manifest",required=True)
    wnew.add_argument("--environment",default="robotics_cell",
                      choices=("open","robotics_cell","cleanroom_bay","logistics_bay"))
    winsp=sub.add_parser(
        "workcell-inspect",help="Apply the same ten engineering evidence gates to one workcell"
    )
    winsp.add_argument("project_file")
    waudit=sub.add_parser(
        "workcell-audit",help="Inspect workcell and actually simulate all of its URDF assemblies"
    )
    waudit.add_argument("project_file")
    waudit.add_argument("--steps",type=int,default=120)
    catalog_new=sub.add_parser(
        "workcell-catalog-build",
        help="Build a portable registry of ALL supplied workcell project files"
    )
    catalog_new.add_argument("catalog_file")
    catalog_new.add_argument("workcells",nargs="+")
    catalog=sub.add_parser(
        "workcell-catalog",help="Audit every workcell in a hashed multi-workcell catalog"
    )
    catalog.add_argument("catalog_file")
    catalog.add_argument("--simulate",action="store_true")
    catalog.add_argument("--steps",type=int,default=120)
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        command = args.command
        code = 0
        if command == "workcell-new":
            from .workcell_acceptance import create_workcell
            result=create_workcell(
                args.project_file,workcell_id=args.id,name=args.name,
                manifest=args.manifest,environment=args.environment,
            )
        elif command == "workcell-inspect":
            from .workcell_acceptance import inspect_workcell
            result=inspect_workcell(args.project_file)
            code=0 if not result["errors"] else 3
        elif command == "workcell-audit":
            from .workcell_acceptance import audit_workcell
            result=audit_workcell(args.project_file,steps=args.steps)
            code=0 if result["runtime_passed"] else 3
        elif command == "workcell-catalog-build":
            from .workcell_acceptance import create_catalog
            result=create_catalog(args.catalog_file,args.workcells)
        elif command == "workcell-catalog":
            from .workcell_acceptance import audit_catalog
            result=audit_catalog(
                args.catalog_file,simulate=args.simulate,steps=args.steps
            )
            code=(
                0 if (
                    result["all_source_integrity_valid"] and
                    (not args.simulate or result["all_runtime_checks_passed"])
                ) else 3
            )
        elif command == "physics-extended":
            from .physics_fidelity import run_extended_physics_suite
            result=run_extended_physics_suite()
            code=0 if result["passed"] else 3
        elif command == "ros2-preview":
            from .ros2_bridge import RosBridgeSettings,preview_ros_contract
            result=preview_ros_contract(RosBridgeSettings(
                frames=1,steps_per_frame=args.steps_per_frame,
                lidar_rays=args.lidar_rays,scan_range_m=args.range_m,
            ))
        elif command == "ros2-stream":
            from .ros2_bridge import RosBridgeSettings,publish_ros2
            result=publish_ros2(RosBridgeSettings(
                frames=args.frames,steps_per_frame=args.steps_per_frame,
                lidar_rays=args.lidar_rays,scan_range_m=args.range_m,
            ))
        elif command == "usd-workspace-new":
            from .usd_composition import new_workspace
            result=new_workspace(args.project_directory)
        elif command == "usd-workspace-import":
            from .usd_composition import add_verified_asset
            result=add_verified_asset(
                args.project_directory,args.asset_manifest,
                alternative_manifest=args.low_memory_manifest,
            )
        elif command == "usd-workspace-place":
            from .usd_composition import place_asset
            result=place_asset(
                args.project_directory,args.asset_id,args.xyz,
                rotation_z_deg=args.yaw,
            )
        elif command == "usd-workspace-variant":
            from .usd_composition import set_representation
            result=set_representation(
                args.project_directory,args.asset_id,args.choice
            )
        elif command == "usd-workspace-inspect":
            from .usd_composition import inspect_workspace
            result=inspect_workspace(args.project_directory)
        elif command == "dataset-generate":
            from .synthetic_dataset import DatasetConfig, generate_dataset
            result=generate_dataset(
                args.output_directory,
                DatasetConfig(
                    frames=args.frames,seed=args.seed,
                    width=args.width,height=args.height,
                    boxes=args.boxes,settle_steps=args.steps,
                ),
            )
        elif command == "dataset-verify":
            from .synthetic_dataset import validate_dataset
            result=validate_dataset(args.directory)
        elif command == "hardware":
            result = inspect_device().as_dict()
        elif command == "validate-asset":
            check = validate_asset(args.manifest)
            result = check.as_dict()
            code = 0 if check.valid_for_import else 2
        elif command == "new-stage":
            result = new_stage(args.scene)
        elif command == "inspect":
            if args.max_prims <= 0:
                raise ValueError("--max-prims must be positive")
            result = inspect_stage(args.scene, max_prims=args.max_prims)
        elif command == "attach":
            result = attach_asset(args.scene, args.manifest)
        elif command == "playground":
            from pathlib import Path
            path = Path(args.scene)
            if not path.exists():
                new_stage(path)
            stage = _load(_existing_layer(path))
            result = create_playground(
                stage,
                PlaygroundSettings(
                    gravity_m_s2=args.gravity,
                    payload_count=args.boxes,
                    payload_mass_kg=args.payload_mass,
                    include_reference_robot=not args.no_robot,
                ),
                replace=args.reset,
            )
            stage.GetRootLayer().Save()
            result["scene"] = str(path.resolve())
        elif command == "spawn-box":
            stage = _load(_existing_layer(args.scene))
            path = spawn_box(
                stage, mass_kg=args.mass,
                side_m=args.side, position_m=args.xyz,
            )
            stage.GetRootLayer().Save()
            result = {"spawned_prim": path, "geometry": "SANDBOX_PRIMITIVE"}
        elif command == "playground-report":
            stage = _load(_existing_layer(args.scene))
            result = playground_report(stage)
        elif command == "robot-fk":
            pose = forward_kinematics([math.radians(q) for q in args.joint_degrees])
            result = pose.as_dict()
            result["joint_degrees"] = args.joint_degrees
        elif command == "robot-ik":
            result = inverse_kinematics(
                args.xyz_m, tolerance_m=args.tolerance,
                max_iterations=args.max_iterations,
            ).as_dict()
            code = 0 if result["converged"] else 3
        elif command == "machine-verify":
            from .gantry_assurance import verify_gantry_in_bullet
            result=verify_gantry_in_bullet(
                args.project_directory,tolerance_m=args.tolerance
            )
            code=0 if result["passed"] else 3
        elif command == "environment-usd":
            from .bullet_runtime import BulletPlayground, BulletSettings
            from .environments import ReferenceEnvironment
            from .environment_usd import export_reference_environment
            with BulletPlayground(BulletSettings(payload_count=0)) as world:
                env=ReferenceEnvironment(world)
                env.build(args.preset)
                result=export_reference_environment(
                    env.parts,args.scene_file,args.preset
                )
        elif command == "environment-render":
            from .bullet_runtime import BulletPlayground, BulletSettings
            from .environments import ReferenceEnvironment
            if not 1 <= args.steps <= 10000:
                raise ValueError("--steps must be in [1,10000]")
            with BulletPlayground(BulletSettings(payload_count=0)) as world:
                env=ReferenceEnvironment(world)
                result={
                    "environment":env.build(args.preset),
                    "physics":world.step(args.steps),
                }
                if args.png:
                    from .sensors import BulletSensors
                    frame=BulletSensors(world).camera_rgbd(
                        eye_m=(3.5,-4.,2.6),look_at_m=(0.,0.,1.15),
                        width=480,height=360,
                    )
                    result["image"]={
                        **frame.metadata(),"file":frame.save_png(args.png)
                    }
        elif command == "physics-verify":
            from .physics_benchmarks import run_physics_suite
            result = run_physics_suite()
            code = 0 if result["passed"] else 3
        elif command == "machine-create":
            from .machine_designer import MachineDesign, save_project
            spec = MachineDesign(
                asset_id=args.name, x_travel_m=args.x_travel,
                y_travel_m=args.y_travel, z_travel_m=args.z_travel,
                clearance_height_m=args.height,
            )
            result = save_project(spec, args.output_directory)
        elif command == "machine-build":
            from .machine_designer import load_design, save_project
            result = save_project(load_design(args.design_json), args.output_directory)
        elif command == "robot-plan":
            from .trajectory import plan_joint_trajectory
            path = plan_joint_trajectory(
                [math.radians(v) for v in args.start],
                [math.radians(v) for v in args.goal],
                duration_s=args.duration,
            )
            result = path.summary()
            result["start_degrees"] = args.start
            result["goal_degrees"] = args.goal
        elif command == "bullet-trajectory":
            from .trajectory import plan_joint_trajectory, replay_in_bullet
            from .bullet_runtime import BulletPlayground, BulletSettings
            goal = [math.radians(v) for v in args.goal]
            plan = plan_joint_trajectory(
                (0.,)*6, goal, duration_s=args.duration,
            )
            with BulletPlayground(BulletSettings(payload_count=0)) as world:
                result = {
                    "plan": plan.summary(),
                    "tracking": replay_in_bullet(
                        world, plan, record_every=args.record_every,
                    ),
                }
        elif command == "urdf-inspect":
            from .urdf_bridge import inspect_urdf
            result = inspect_urdf(args.urdf_file).as_dict()
        elif command == "bullet-urdf":
            from .bullet_runtime import BulletPlayground, BulletSettings
            if not 1 <= args.steps <= 50000:
                raise ValueError("--steps must be between 1 and 50000")
            if (args.joint is None) != (args.target is None):
                raise ValueError("--joint and --target must be supplied together")
            with BulletPlayground(BulletSettings(payload_count=0)) as world:
                import_result = world.import_urdf_robot(args.manifest)
                if args.joint is not None:
                    world.command_imported_joint(
                        import_result["asset_id"], args.joint, args.target
                    )
                report = world.step(args.steps)
                result = {
                    "import": import_result,
                    "runtime": report,
                    "note": "True Bullet simulation with user-authored URDF; "
                            "no OEM/physics calibration is inferred.",
                }
        elif command == "studio":
            from .desktop_studio import launch_studio
            result = launch_studio(
                theme=args.theme, boxes=args.boxes,
                camera_width=args.width, camera_height=args.height,
                fps_limit=args.fps,
            )
        elif command == "bullet-run":
            from pathlib import Path
            from .bullet_runtime import BulletPlayground, BulletSettings
            if not 1 <= args.steps <= 50000:
                raise ValueError("--steps must be between 1 and 50,000")
            if args.gui and (args.lidar_rays or args.camera_png or args.depth_npz):
                raise ValueError("Sensor exports require --gui omitted (headless DIRECT mode)")
            with BulletPlayground(BulletSettings(
                gui=args.gui, gravity_m_s2=args.gravity, payload_count=args.boxes
            )) as world:
                world.command_joints([math.radians(q) for q in args.joint_degrees])
                if args.gui:
                    world.run_interactive()
                    result = {
                        "engine": "pybullet_cpu",
                        "gui_closed": True,
                        "note": "Local native Bullet window closed. No RTX/Kit required.",
                    }
                else:
                    result = world.step(args.steps)
                    if args.lidar_rays or args.camera_png or args.depth_npz:
                        from .sensors import BulletSensors
                        sensors = BulletSensors(world)
                        if args.lidar_rays:
                            result["lidar"] = sensors.lidar_2d(count=args.lidar_rays)
                        if args.camera_png or args.depth_npz:
                            frame = sensors.camera_rgbd()
                            result["camera"] = frame.metadata()
                            if args.camera_png:
                                result["camera"]["rgb_png"] = frame.save_png(args.camera_png)
                            if args.depth_npz:
                                result["camera"]["depth_npz"] = frame.save_depth_npz(args.depth_npz)
            if args.json_out:
                target = Path(args.json_out).resolve()
                if target.exists():
                    raise FileExistsError(f"Refusing to overwrite existing report: {target}")
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
                result["report_file"] = str(target)
        elif command == "robot-drive":
            stage = _load(_existing_layer(args.scene))
            result = {"target_degrees": set_reference_joint_targets(
                stage, [math.radians(q) for q in args.joint_degrees]
            )}
            stage.GetRootLayer().Save()
        else:
            raise ValueError(f"Unknown subcommand: {command}")
    except (OSError, RuntimeError, ValueError, FileExistsError, ArithmeticError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, indent=2), file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return code
