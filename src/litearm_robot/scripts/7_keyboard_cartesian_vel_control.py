#!/usr/bin/env python3
"""Keyboard Cartesian velocity control using a damped Jacobian inverse."""

import argparse
import time
import threading
import numpy as np

from litearm_demo_common import (
    add_common_args,
    add_side_arg,
    arm_positions,
    damped_pseudoinverse,
    jacobian,
    open_robot,
    send_arm_posvel,
)


def main():
    parser = argparse.ArgumentParser(description="Keyboard Cartesian velocity control.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--linear-speed", type=float, default=0.03)
    parser.add_argument("--angular-speed", type=float, default=0.3)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    try:
        from pynput import keyboard
    except Exception as exc:
        raise SystemExit(f"keyboard input unavailable: {exc}")

    robot = None
    running = True
    pressed = set()
    lock = threading.Lock()
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        print("[LiteArmDemo] hold W/S X, A/D Y, Q/E Z, 1/2 Rx, 3/4 Ry, 5/6 Rz, ESC quit")
        if not args.execute:
            print("[LiteArmDemo] dry run only. Add --execute for motor commands.")

        def on_press(key):
            nonlocal running
            if key == keyboard.Key.esc:
                running = False
                return False
            char = getattr(key, "char", None)
            if char:
                with lock:
                    pressed.add(char.lower())

        def on_release(key):
            char = getattr(key, "char", None)
            if char:
                with lock:
                    pressed.discard(char.lower())

        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        dt = 0.01
        while running:
            with lock:
                keys = set(pressed)
            twist = np.zeros(6)
            if "w" in keys: twist[0] += args.linear_speed
            if "s" in keys: twist[0] -= args.linear_speed
            if "a" in keys: twist[1] += args.linear_speed
            if "d" in keys: twist[1] -= args.linear_speed
            if "q" in keys: twist[2] += args.linear_speed
            if "e" in keys: twist[2] -= args.linear_speed
            if "1" in keys: twist[3] += args.angular_speed
            if "2" in keys: twist[3] -= args.angular_speed
            if "3" in keys: twist[4] += args.angular_speed
            if "4" in keys: twist[4] -= args.angular_speed
            if "5" in keys: twist[5] += args.angular_speed
            if "6" in keys: twist[5] -= args.angular_speed

            q = arm_positions(robot, args.side)
            dq = damped_pseudoinverse(jacobian(_cfg, args.side, q), 0.04) @ twist
            q_target = q + dt * np.clip(dq, -1.0, 1.0)
            if args.execute:
                send_arm_posvel(robot, args.side, q_target, velocity=0.3)
            time.sleep(dt)
        listener.stop()
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
