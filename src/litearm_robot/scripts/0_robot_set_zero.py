#!/usr/bin/env python3
"""Reset motor zero positions, matching Panthera 0_robot_set_zero.py."""

import argparse
import time

from litearm_demo_common import add_common_args, open_robot, print_state_table, read_states


def main():
    parser = argparse.ArgumentParser(description="Reset all LiteArm motor zero positions.")
    add_common_args(parser)
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually reset the motor zero positions.",
    )
    parser.add_argument("--watch", action="store_true", help="Print states after reset.")
    args = parser.parse_args()

    if not args.execute:
        print("[LiteArmDemo] dry run. Add --execute to reset motor zero positions.")
        return

    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        print("[LiteArmDemo] resetting zero positions...")
        robot.reset_zero_all()
        print("[LiteArmDemo] reset-zero command sent.")
        if args.watch:
            while True:
                print_state_table(read_states(robot))
                time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.close_all()


if __name__ == "__main__":
    main()
