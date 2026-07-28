#!/usr/bin/env python3
"""Print FK while running Pinocchio gravity feed-forward."""

import argparse
import time
import numpy as np

from litearm_demo_common import (
    add_common_args,
    add_side_arg,
    arm_positions,
    forward_kinematics,
    gravity_torque,
    open_robot,
    send_arm_torque,
)


def main():
    parser = argparse.ArgumentParser(description="Gravity compensation with FK telemetry.")
    add_common_args(parser)
    add_side_arg(parser)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    if not args.execute:
        print("[LiteArmDemo] dry run. Add --execute to start torque control.")
        return

    robot = None
    try:
        robot, cfg = open_robot(args.config, init_board=not args.no_init)
        end = None if args.duration is None else time.time() + max(0.0, args.duration)
        while end is None or time.time() < end:
            sides = ("left", "right") if args.side == "both" else (args.side,)
            for side in sides:
                q = arm_positions(robot, side)
                pos, _rot = forward_kinematics(cfg, side, q)
                tau = gravity_torque(cfg, side, q)
                print(f"\r[{side}] xyz={np.round(pos, 4).tolist()} "
                      f"tau={np.round(tau, 3).tolist()}   ", end="", flush=True)
                send_arm_torque(robot, side, tau)
            time.sleep(0.01)
        print()
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
