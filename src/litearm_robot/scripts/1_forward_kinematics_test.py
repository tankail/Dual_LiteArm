#!/usr/bin/env python3
"""Print LiteArm end-effector pose from current joint positions."""

import argparse
import time
import numpy as np

from litearm_demo_common import (
    add_common_args,
    add_side_arg,
    arm_positions,
    forward_kinematics,
    open_robot,
)


def main():
    parser = argparse.ArgumentParser(description="LiteArm forward kinematics test.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--once", action="store_true")
    args = parser.parse_args()

    robot = None
    try:
        robot, cfg = open_robot(args.config, init_board=not args.no_init)
        while True:
            q = arm_positions(robot, args.side)
            pos, rot = forward_kinematics(cfg, args.side, q)
            print(f"[LiteArmDemo] {args.side} q={np.round(q, 4).tolist()}")
            print(f"[LiteArmDemo] position(m)={np.round(pos, 5).tolist()}")
            print(f"[LiteArmDemo] rotation=\n{np.array2string(rot, precision=4, suppress_small=True)}")
            if args.once:
                break
            time.sleep(1.0 / max(args.rate, 0.1))
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.close_all()


if __name__ == "__main__":
    main()
