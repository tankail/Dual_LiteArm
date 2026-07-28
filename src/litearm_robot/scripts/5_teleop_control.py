#!/usr/bin/env python3
"""Mirror one LiteArm arm to the other arm in joint space."""

import argparse
import time
import numpy as np

from litearm_demo_common import add_common_args, arm_positions, open_robot, send_arm_posvel


def main():
    parser = argparse.ArgumentParser(description="Joint-space left/right teleoperation demo.")
    add_common_args(parser)
    parser.add_argument("--leader", choices=("left", "right"), default="left")
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    follower = "right" if args.leader == "left" else "left"
    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        print(f"[LiteArmDemo] leader={args.leader}, follower={follower}")
        if not args.execute:
            print("[LiteArmDemo] dry run. Add --execute to mirror motion.")
            return
        end = None if args.duration is None else time.time() + max(0.0, args.duration)
        while end is None or time.time() < end:
            q_leader = arm_positions(robot, args.leader)
            q_target = args.scale * q_leader
            send_arm_posvel(robot, follower, q_target, velocity=0.5)
            print(f"\r[LiteArmDemo] {args.leader}={np.round(q_leader, 3).tolist()}", end="", flush=True)
            time.sleep(0.01)
        print()
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] teleoperation interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
