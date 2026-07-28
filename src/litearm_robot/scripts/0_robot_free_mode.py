#!/usr/bin/env python3
"""Put LiteArm motors into free mode and keep the script alive."""

import argparse
import time

from litearm_demo_common import add_common_args, open_robot


def main():
    parser = argparse.ArgumentParser(description="Set all configured LiteArm motors to free mode.")
    add_common_args(parser)
    parser.add_argument("--duration", type=float, default=None, help="Seconds to keep running.")
    args = parser.parse_args()

    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        robot.set_all_free_mode()
        print("[LiteArmDemo] free mode enabled. Press Ctrl+C to stop.")

        if args.duration is None:
            while True:
                time.sleep(0.5)
        else:
            end = time.time() + max(0.0, args.duration)
            while time.time() < end:
                robot.set_all_free_mode()
                time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.close_all()


if __name__ == "__main__":
    main()
