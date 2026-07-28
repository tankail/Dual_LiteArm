#!/usr/bin/env python3
"""Joint-space moveJ-style interpolation demo for LiteArm."""

import argparse
import time
import numpy as np

from litearm_demo_common import (
    add_common_args,
    add_side_arg,
    arm_positions,
    open_robot,
    send_arm_posvel,
)


def move_j(robot, side, start, target, duration, velocity):
    steps = max(1, int(duration * 100))
    for i in range(steps + 1):
        alpha = i / steps
        # Cubic time scaling avoids a hard position jump at both ends.
        s = alpha * alpha * (3.0 - 2.0 * alpha)
        q = (1.0 - s) * start + s * target
        send_arm_posvel(robot, side, q, velocity=velocity)
        time.sleep(duration / steps)


def main():
    parser = argparse.ArgumentParser(description="MoveJ-style joint interpolation.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--offset", type=float, default=0.15)
    parser.add_argument("--velocity", type=float, default=0.4)
    parser.add_argument("--execute", action="store_true", help="Actually move the arm.")
    args = parser.parse_args()

    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        start = arm_positions(robot, args.side)
        target = start.copy()
        target[1:3] += args.offset
        print(f"[LiteArmDemo] start={np.round(start, 4).tolist()}")
        print(f"[LiteArmDemo] target={np.round(target, 4).tolist()}")
        if not args.execute:
            print("[LiteArmDemo] dry run. Add --execute to move.")
            return
        move_j(robot, args.side, start, target, args.duration, args.velocity)
        time.sleep(1.0)
        move_j(robot, args.side, target, start, args.duration, args.velocity)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
