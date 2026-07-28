#!/usr/bin/env python3
"""Gravity and friction compensation using the LiteArm Pinocchio model."""

import argparse
import time
import numpy as np

from litearm_demo_common import (
    add_common_args,
    add_side_arg,
    arm_positions,
    arm_velocities,
    friction_torque,
    gravity_torque,
    open_robot,
    send_arm_torque,
)


def main():
    parser = argparse.ArgumentParser(description="Gravity + friction compensation demo.")
    add_common_args(parser)
    add_side_arg(parser)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    if not args.execute:
        print("[LiteArmDemo] dry run. Add --execute to start torque control.")
        return

    fc = np.array([0.20, 0.15, 0.15, 0.15, 0.04, 0.04, 0.04])
    fv = np.array([0.06, 0.06, 0.06, 0.03, 0.02, 0.02, 0.02])
    robot = None
    try:
        robot, cfg = open_robot(args.config, init_board=not args.no_init)
        end = None if args.duration is None else time.time() + max(0.0, args.duration)
        while end is None or time.time() < end:
            sides = ("left", "right") if args.side == "both" else (args.side,)
            for side in sides:
                q = arm_positions(robot, side)
                v = arm_velocities(robot, side)
                torque = gravity_torque(cfg, side, q) + friction_torque(v, fc, fv)
                send_arm_torque(robot, side, torque)
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
