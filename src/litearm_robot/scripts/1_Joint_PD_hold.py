#!/usr/bin/env python3
"""Hold the current LiteArm joint positions with explicit KP/KD control."""

import argparse
import time

from litearm_demo_common import add_common_args, open_robot, current_positions


def main():
    parser = argparse.ArgumentParser(description="Hold current positions with MODE_POS_VEL_KP_KD.")
    add_common_args(parser)
    parser.add_argument("--duration", type=float, default=5.0, help="Hold duration in seconds.")
    parser.add_argument("--kp", type=float, default=8.0, help="Position gain.")
    parser.add_argument("--kd", type=float, default=0.35, help="Velocity gain.")
    args = parser.parse_args()

    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        pos = current_positions(robot)
        cmd = {gid: (p, 0.0, args.kp, args.kd) for gid, p in pos.items()}
        print(f"[LiteArmDemo] holding {len(cmd)} motors for {args.duration:.1f}s")

        end = time.time() + max(0.0, args.duration)
        while time.time() < end:
            robot.set_all_pos_vel_kp_kd(cmd)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            try:
                robot.stop_all()
            finally:
                robot.close_all()


if __name__ == "__main__":
    main()
