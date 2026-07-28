#!/usr/bin/env python3
"""Joint interpolation demo with a non-zero speed limit."""

import argparse
import numpy as np

from litearm_demo_common import add_common_args, add_side_arg, arm_positions, interpolate_arm, open_robot


def main():
    parser = argparse.ArgumentParser(description="Non-zero-velocity joint interpolation.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--offset", type=float, default=0.15)
    parser.add_argument("--velocity", type=float, default=0.4)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        q0 = arm_positions(robot, args.side)
        q1 = q0.copy()
        q1[1:3] += args.offset
        print(f"[LiteArmDemo] start={np.round(q0, 4).tolist()} target={np.round(q1, 4).tolist()}")
        if not args.execute:
            print("[LiteArmDemo] dry run. Add --execute to move.")
            return
        interpolate_arm(robot, args.side, q0, q1, args.duration, velocity=args.velocity)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
