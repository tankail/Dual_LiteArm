#!/usr/bin/env python3
"""Small joint-space PosVel demo for LiteArm."""

import argparse
import time

from litearm_demo_common import (
    ARM_IDS,
    add_common_args,
    add_side_arg,
    current_positions,
    hold_current_position_cmd,
    open_robot,
)


def main():
    parser = argparse.ArgumentParser(description="Move one LiteArm joint by a small relative offset.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--joint", type=int, default=2, help="Joint index inside the selected arm, 1-7.")
    parser.add_argument("--amplitude", type=float, default=0.15, help="Relative motion in rad.")
    parser.add_argument("--velocity", type=float, default=0.4, help="Command velocity in rad/s.")
    parser.add_argument("--max-torque", type=float, default=None, help="Override max torque for every motor.")
    parser.add_argument("--hold", type=float, default=1.5, help="Hold time at each point.")
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually send motion commands. Without this flag the script only prints the planned command.",
    )
    args = parser.parse_args()

    if args.side == "both":
        raise SystemExit("--side both is not supported for this single-joint demo")
    if args.joint < 1 or args.joint > 7:
        raise SystemExit("--joint must be in 1..7")

    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        base_pos = current_positions(robot)
        gid = ARM_IDS[args.side][args.joint - 1]
        if gid not in base_pos:
            raise RuntimeError(f"cannot read current position for global motor id {gid}")

        targets = [
            base_pos[gid] + args.amplitude,
            base_pos[gid] - args.amplitude,
            base_pos[gid],
        ]
        print(
            f"[LiteArmDemo] side={args.side}, joint={args.joint}, gid={gid}, "
            f"current={base_pos[gid]:.4f}, targets={[round(v, 4) for v in targets]}"
        )
        if not args.execute:
            print("[LiteArmDemo] dry run only. Re-run with --execute to move.")
            return

        cmd = hold_current_position_cmd(robot, velocity=args.velocity, max_torque=args.max_torque)
        for target in targets:
            cmd[gid] = (
                target,
                float(args.velocity),
                float(args.max_torque) if args.max_torque is not None else 8.0,
            )
            robot.set_all_pos_vel_max_torque(cmd)
            time.sleep(max(0.0, args.hold))
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
