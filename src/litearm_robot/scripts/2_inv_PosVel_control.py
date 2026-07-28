#!/usr/bin/env python3
"""Cartesian target to IK, then joint PosVel execution."""

import argparse
import numpy as np

from litearm_demo_common import (
    add_common_args,
    add_side_arg,
    arm_positions,
    cartesian_target_from_current,
    inverse_kinematics,
    interpolate_arm,
    open_robot,
)


def main():
    parser = argparse.ArgumentParser(description="LiteArm inverse PosVel control demo.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--dx", type=float, default=0.02)
    parser.add_argument("--dy", type=float, default=0.0)
    parser.add_argument("--dz", type=float, default=0.0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    robot = None
    try:
        robot, cfg = open_robot(args.config, init_board=not args.no_init)
        q0 = arm_positions(robot, args.side)
        pos, rot = cartesian_target_from_current(
            cfg, args.side, q0, [args.dx, args.dy, args.dz]
        )
        q_target = inverse_kinematics(cfg, args.side, pos, rot, q0)
        print(f"[LiteArmDemo] target position={np.round(pos, 5).tolist()}")
        if q_target is None:
            print("[LiteArmDemo] IK failed; no command sent.")
            return
        print(f"[LiteArmDemo] target q={np.round(q_target, 5).tolist()}")
        if not args.execute:
            print("[LiteArmDemo] dry run. Add --execute to move.")
            return
        interpolate_arm(robot, args.side, q0, q_target, args.duration)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
