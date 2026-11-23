"""PyInstaller executable entry point for DSIQ Native CPU/robotics CLI.

This script is intentionally NOT a claim that the frozen executable
includes NVIDIA Kit, RTX/PhysX, ROS 2, or the optional Pixar USD binding.
"""
from dsiq_native.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
