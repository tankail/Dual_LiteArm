#!/usr/bin/env python3
"""Compatibility entry point for the LiteArm Python gravity script."""

import argparse
import os
import sys


BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPT_RUNNER = os.path.join(BACKEND_DIR, "run_script.py")
DEFAULT_CONFIG = os.path.join(
    BACKEND_DIR, "robot_param", "litearm_arms.yaml"
)
SCRIPT_NAME = "2_gravity_compensation_control.py"


def main():
    parser = argparse.ArgumentParser(
        description="Run LiteArm dual-arm gravity compensation in Python"
    )
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    args = parser.parse_args()

    if args.side != "both":
        print(
            "This dual-arm entry point controls both arms; "
            "use the dual-arm script directly.",
            file=sys.stderr,
        )
        return 2

    os.execv(
        sys.executable,
        [
            sys.executable,
            SCRIPT_RUNNER,
            "--config",
            os.path.abspath(args.config),
            SCRIPT_NAME,
        ],
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
