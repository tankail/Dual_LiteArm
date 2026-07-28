#!/usr/bin/env python3
"""Cartesian straight-line position move using FK and IK."""

import argparse
import numpy as np

from litearm_demo_common import (
    add_common_args,
    add_side_arg,
    arm_positions,
    cartesian_target_from_current,
    inverse_kinematics,
    interpolate_arm,
    open_robot,
)


def main():
    parser = argparse.ArgumentParser(description="LiteArm Cartesian position move.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--dx", type=float, default=0.02)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    robot = None
    try:
        robot, cfg = open_robot(args.config, init_board=not args.no_init)
        q0 = arm_positions(robot, args.side)
        target_pos, target_rot = cartesian_target_from_current(
            cfg, args.side, q0, [args.dx, args.dy, args.dz]
        )
        q1 = inverse_kinematics(cfg, args.side, target_pos, target_rot, q0)
        print(f"[LiteArmDemo] target xyz={np.round(target_pos, 5).tolist()}")
        print(f"[LiteArmDemo] target q={None if q1 is None else np.round(q1, 5).tolist()}")
        if q1 is None:
            return
        if not args.execute:
            print("[LiteArmDemo] dry run. Add --execute to move.")
            return
        interpolate_arm(robot, args.side, q0, q1, args.duration)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
