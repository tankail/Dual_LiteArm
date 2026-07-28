#!/usr/bin/env python3
"""Cartesian orientation move using FK and IK."""

import argparse
import numpy as np

from litearm_demo_common import (
    add_common_args,
    add_side_arg,
    arm_positions,
    cartesian_target_from_current,
    forward_kinematics,
    inverse_kinematics,
    interpolate_arm,
    open_robot,
    rotation_matrix_x,
    rotation_matrix_y,
    rotation_matrix_z,
)


def main():
    parser = argparse.ArgumentParser(description="LiteArm Cartesian orientation move.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--axis", choices=("x", "y", "z"), default="y")
    parser.add_argument("--angle-deg", type=float, default=15.0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    rotations = {"x": rotation_matrix_x, "y": rotation_matrix_y, "z": rotation_matrix_z}
    robot = None
    try:
        robot, cfg = open_robot(args.config, init_board=not args.no_init)
        q0 = arm_positions(robot, args.side)
        pos, rot = forward_kinematics(cfg, args.side, q0)
        target_rot = rot @ rotations[args.axis](np.deg2rad(args.angle_deg))
        q1 = inverse_kinematics(cfg, args.side, pos, target_rot, q0)
        print(f"[LiteArmDemo] target rotation axis={args.axis}, angle={args.angle_deg} deg")
        print(f"[LiteArmDemo] target q={None if q1 is None else np.round(q1, 5).tolist()}")
        if q1 is None:
            return
        if not args.execute:
            print("[LiteArmDemo] dry run. Add --execute to move.")
            return
        interpolate_arm(robot, args.side, q0, q1, args.duration)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
