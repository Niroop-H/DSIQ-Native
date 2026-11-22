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
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        command = args.command
        code = 0
        if command == "hardware":
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
