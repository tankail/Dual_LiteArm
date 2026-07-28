#!/usr/bin/env python3
"""Validate LiteArm IK by solving the current FK pose."""

import argparse
import numpy as np

from litearm_demo_common import (
    add_common_args,
    add_side_arg,
    arm_positions,
    forward_kinematics,
    inverse_kinematics,
    open_robot,
)


def main():
    parser = argparse.ArgumentParser(description="LiteArm inverse kinematics validation.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    args = parser.parse_args()

    robot = None
    try:
        robot, cfg = open_robot(args.config, init_board=not args.no_init)
        q = arm_positions(robot, args.side)
        pos, rot = forward_kinematics(cfg, args.side, q)
        solved = inverse_kinematics(cfg, args.side, pos, rot, q)
        print(f"[LiteArmDemo] current q={np.round(q, 6).tolist()}")
        if solved is None:
            print("[LiteArmDemo] IK failed")
            return
        print(f"[LiteArmDemo] solved q={np.round(solved, 6).tolist()}")
        print(f"[LiteArmDemo] max joint error={np.max(np.abs(q - solved)):.6f} rad")
    finally:
        if robot is not None:
            robot.close_all()


if __name__ == "__main__":
    main()
