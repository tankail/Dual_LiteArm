#!/usr/bin/env python3
"""Joint PD control demo for LiteArm."""

import argparse
import time
import numpy as np

from litearm_demo_common import (
    add_common_args,
    add_side_arg,
    arm_positions,
    send_arm_pvkd,
    open_robot,
)


def main():
    parser = argparse.ArgumentParser(description="Run a joint PD position demo.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--offset", type=float, default=0.15, help="Offset for joints 2 and 3 in rad.")
    parser.add_argument("--kp", type=float, default=8.0)
    parser.add_argument("--kd", type=float, default=0.35)
    parser.add_argument("--execute", action="store_true", help="Send control commands.")
    args = parser.parse_args()

    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        q0 = arm_positions(robot, args.side)
        target = q0.copy()
        target[1:3] += args.offset
        print(f"[LiteArmDemo] current={np.round(q0, 4).tolist()}")
        print(f"[LiteArmDemo] target={np.round(target, 4).tolist()}")
        if not args.execute:
            print("[LiteArmDemo] dry run. Add --execute to control the arm.")
            return

        kp = [args.kp] * 7
        kd = [args.kd] * 7
        end = time.time() + max(0.0, args.duration)
        while time.time() < end:
            send_arm_pvkd(robot, args.side, target, kp, kd)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
