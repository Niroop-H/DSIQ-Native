"""DSIQ Native command line. All results are machine-readable and errors fail loudly."""
from __future__ import annotations

import argparse
import json
import sys

from .asset_quality import validate_asset
from .hardware import inspect_device
from .usd_stage import attach_asset, inspect_stage, new_stage


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="dsiq-native",
        description="Native OpenUSD asset and hardware inspection (pre-alpha; no OEM certification)",
    )
    sub = p.add_subparsers(dest="command", required=True)
    sub.add_parser("hardware", help="Read actual NVIDIA VRAM and physical system RAM")
    asset = sub.add_parser("validate-asset", help="Verify local OEM asset manifest and evidence")
    asset.add_argument("manifest")
    create = sub.add_parser("new-stage", help="Create metric Z-up USD without fake geometry")
    create.add_argument("scene")
    inspect = sub.add_parser("inspect", help="Inspect USD topology/physics schema")
    inspect.add_argument("scene")
    inspect.add_argument("--max-prims", type=int, default=20000)
    attach = sub.add_parser("attach", help="Reference real on-disk USD asset into canonical scene")
    attach.add_argument("scene")
    attach.add_argument("manifest")
    return p


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "hardware":
            result = inspect_device().as_dict()
            code = 0
        elif args.command == "validate-asset":
            validation = validate_asset(args.manifest)
            result = validation.as_dict()
            code = 0 if validation.valid_for_import else 2
        elif args.command == "new-stage":
            result = new_stage(args.scene)
            code = 0
        elif args.command == "inspect":
            if args.max_prims <= 0:
                raise ValueError("--max-prims must be positive")
            result = inspect_stage(args.scene, max_prims=args.max_prims)
            code = 0
        elif args.command == "attach":
            result = attach_asset(args.scene, args.manifest)
            code = 0
        else:
            raise ValueError("Unknown subcommand")
    except (OSError, RuntimeError, ValueError, FileExistsError) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, indent=2), file=sys.stderr)
        return 2
    print(json.dumps(result, indent=2, sort_keys=True))
    return code
