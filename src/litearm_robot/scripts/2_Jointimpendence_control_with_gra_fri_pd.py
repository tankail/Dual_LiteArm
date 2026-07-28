#!/usr/bin/env python3
"""Joint impedance control with gravity and friction feed-forward."""

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
    parser = argparse.ArgumentParser(description="Joint impedance + gravity + friction demo.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--kp", type=float, default=3.0)
    parser.add_argument("--kd", type=float, default=0.25)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    fc = np.array([0.20, 0.15, 0.15, 0.15, 0.04, 0.04, 0.04])
    fv = np.array([0.06, 0.06, 0.06, 0.03, 0.02, 0.02, 0.02])
    robot = None
    try:
        robot, cfg = open_robot(args.config, init_board=not args.no_init)
        q0 = arm_positions(robot, args.side)
        q_des = q0.copy()
        q_des[1:3] += 0.1
        if not args.execute:
            print("[LiteArmDemo] dry run. Add --execute to start torque control.")
            return

        kp = np.ones(7) * args.kp
        kd = np.ones(7) * args.kd
        end = time.time() + max(0.0, args.duration)
        while time.time() < end:
            q = arm_positions(robot, args.side)
            v = arm_velocities(robot, args.side)
            torque = (
                kp * (q_des - q)
                - kd * v
                + gravity_torque(cfg, args.side, q)
                + friction_torque(v, fc, fv)
            )
            send_arm_torque(robot, args.side, torque)
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
