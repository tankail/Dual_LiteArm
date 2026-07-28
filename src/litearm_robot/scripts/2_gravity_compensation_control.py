#!/usr/bin/env python3
"""Run the validated C++ LiteArm gravity compensation example."""

import argparse

from litearm_demo_common import add_side_arg, run_gravity_process


def main():
    parser = argparse.ArgumentParser(
        description="Start LiteArm gravity compensation using the C++ examples."
    )
    add_side_arg(parser)
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Run duration in seconds. Default: run until Ctrl+C.",
    )
    args = parser.parse_args()
    run_gravity_process(side=args.side, duration=args.duration)


if __name__ == "__main__":
    main()
