#!/usr/bin/env python3
"""Small sinusoidal joint trajectory demo."""

import argparse
import time
import numpy as np

from litearm_demo_common import add_common_args, add_side_arg, arm_positions, open_robot, send_arm_posvel


def main():
    parser = argparse.ArgumentParser(description="Sinusoidal joint trajectory.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--amplitude", type=float, default=0.1)
    parser.add_argument("--frequency", type=float, default=0.1)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        q0 = arm_positions(robot, args.side)
        if not args.execute:
            print(f"[LiteArmDemo] dry run around q={np.round(q0, 4).tolist()}. Add --execute.")
            return
        end = time.time() + max(0.0, args.duration)
        while time.time() < end:
            t = time.time()
            q = q0.copy()
            q[1] += args.amplitude * np.sin(2 * np.pi * args.frequency * t)
            send_arm_posvel(robot, args.side, q, velocity=max(0.2, args.amplitude * 4))
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
