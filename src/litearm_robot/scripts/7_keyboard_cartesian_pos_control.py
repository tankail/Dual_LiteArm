#!/usr/bin/env python3
"""Keyboard Cartesian position control using Pinocchio IK."""

import argparse
import threading
import time
import numpy as np

from litearm_demo_common import (
    add_common_args,
    add_side_arg,
    arm_positions,
    forward_kinematics,
    inverse_kinematics,
    open_robot,
    rotation_matrix_x,
    rotation_matrix_y,
    rotation_matrix_z,
    send_arm_posvel,
)


def main():
    parser = argparse.ArgumentParser(description="Keyboard Cartesian position control.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--step-mm", type=float, default=5.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    try:
        from pynput import keyboard
    except Exception as exc:
        raise SystemExit(f"keyboard input unavailable: {exc}")

    robot = None
    running = True
    changed = threading.Event()
    try:
        robot, cfg = open_robot(args.config, init_board=not args.no_init)
        q = arm_positions(robot, args.side)
        position, rotation = forward_kinematics(cfg, args.side, q)
        print("[LiteArmDemo] W/S X, A/D Y, Q/E Z, 1/2 Rx, 3/4 Ry, 5/6 Rz, ESC quit")
        if not args.execute:
            print("[LiteArmDemo] dry run only. Add --execute for motor commands.")

        step = args.step_mm / 1000.0

        def on_press(key):
            nonlocal position, rotation, running
            char = getattr(key, "char", None)
            if key == keyboard.Key.esc:
                running = False
                return False
            if char in ("w", "W"): position[0] += step
            elif char in ("s", "S"): position[0] -= step
            elif char in ("a", "A"): position[1] += step
            elif char in ("d", "D"): position[1] -= step
            elif char in ("q", "Q"): position[2] += step
            elif char in ("e", "E"): position[2] -= step
            elif char == "1": rotation = rotation @ rotation_matrix_x(0.03)
            elif char == "2": rotation = rotation @ rotation_matrix_x(-0.03)
            elif char == "3": rotation = rotation @ rotation_matrix_y(0.03)
            elif char == "4": rotation = rotation @ rotation_matrix_y(-0.03)
            elif char == "5": rotation = rotation @ rotation_matrix_z(0.03)
            elif char == "6": rotation = rotation @ rotation_matrix_z(-0.03)
            else: return
            changed.set()

        listener = keyboard.Listener(on_press=on_press)
        listener.start()
        while running:
            if changed.is_set():
                changed.clear()
                q_target = inverse_kinematics(cfg, args.side, position, rotation, q)
                if q_target is not None:
                    q = q_target
                    if args.execute:
                        send_arm_posvel(robot, args.side, q, velocity=0.3)
                    print(f"[LiteArmDemo] xyz={np.round(position, 4).tolist()} q={np.round(q, 3).tolist()}")
            time.sleep(0.01)
        listener.stop()
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
