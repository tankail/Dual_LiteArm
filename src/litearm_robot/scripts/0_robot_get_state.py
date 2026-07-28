#!/usr/bin/env python3
"""Print LiteArm joint states, similar to Panthera 0_robot_get_state.py."""

import argparse
import time

from litearm_demo_common import (
    add_common_args,
    add_side_arg,
    ARM_ALL_IDS,
    open_robot,
    positive_float,
    print_state_table,
    read_states,
)


def main():
    parser = argparse.ArgumentParser(description="Read and print LiteArm joint states.")
    add_common_args(parser)
    add_side_arg(parser)
    parser.add_argument("--rate", type=positive_float, default=2.0, help="Print rate in Hz.")
    parser.add_argument("--once", action="store_true", help="Read once and exit.")
    args = parser.parse_args()

    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        motor_ids = ARM_ALL_IDS[args.side]
        interval = 1.0 / args.rate
        while True:
            states = read_states(robot)
            print_state_table(states, motor_ids)
            if args.once:
                break
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.close_all()


if __name__ == "__main__":
    main()
