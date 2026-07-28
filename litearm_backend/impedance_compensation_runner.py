#!/usr/bin/env python3
"""Compatibility entry point for the LiteArm Python impedance script."""

import argparse
import os
import sys


BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPT_RUNNER = os.path.join(BACKEND_DIR, "run_script.py")
DEFAULT_CONFIG = os.path.join(
    BACKEND_DIR, "robot_param", "litearm_arms.yaml"
)
SCRIPT_NAME = "dual_arm_impedance_compensation.py"


def main():
    parser = argparse.ArgumentParser(
        description="Run LiteArm dual-arm impedance control in Python"
    )
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--left-target", nargs=7, type=float)
    parser.add_argument("--right-target", nargs=7, type=float)
    args = parser.parse_args()

    command = [
        sys.executable,
        SCRIPT_RUNNER,
        "--config",
        os.path.abspath(args.config),
        SCRIPT_NAME,
    ]
    if args.left_target is not None:
        command.extend(["--left-target", *[str(value) for value in args.left_target]])
    if args.right_target is not None:
        command.extend(["--right-target", *[str(value) for value in args.right_target]])

    os.execv(sys.executable, command)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
