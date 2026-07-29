#!/usr/bin/env python3
"""Read both LiteArm joint states while applying gravity compensation."""

import argparse
import signal
import time

import numpy as np

from litearm_control import DEFAULT_CONFIG, DualLiteArmPython


keep_running = True


def _signal_handler(_signum, _frame):
    global keep_running
    keep_running = False
    print("\nInterrupted; exiting state demo.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read left and right LiteArm joint states."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG,
        help="Dual-arm backend YAML configuration.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=2.0,
        help="Display refresh rate in Hz.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Read and print one state sample, then exit.",
    )
    parser.add_argument(
        "--control-rate",
        type=float,
        default=200.0,
        help="Gravity compensation update rate in Hz (default: 200).",
    )
    parser.add_argument(
        "--no-gravity-comp",
        action="store_true",
        help="Only read state; do not send gravity compensation torque.",
    )
    return parser.parse_args()


def _print_arm(robot, side):
    q = robot.arm_q(side)
    print(
        f"[{side}] q(rad): "
        f"[{', '.join(f'{value:.4f}' for value in q)}]"
    )
    print(
        f"[{side}] q(deg): "
        f"[{', '.join(f'{value:.2f}' for value in np.degrees(q))}]"
    )


def print_robot_state(robot):
    _print_arm(robot, "left")
    _print_arm(robot, "right")


def main():
    global keep_running
    args = parse_args()
    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")
    if args.control_rate <= 0.0:
        raise SystemExit("--control-rate must be positive")

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    robot = None
    try:
        robot = DualLiteArmPython(args.config)
        robot.open()
        gravity_enabled = not args.no_gravity_comp
        print(
            "LiteArm state reader started. "
            f"Gravity compensation: {'ON' if gravity_enabled else 'OFF'}."
        )
        if gravity_enabled:
            print(f"Gravity compensation control rate: {args.control_rate:.1f} Hz")

        control_period = 1.0 / args.control_rate
        print_period = 1.0 / args.rate
        next_control_tick = time.monotonic()
        next_print_tick = next_control_tick
        while keep_running:
            next_control_tick += control_period
            robot.read_state()
            robot.validate_arms()

            if gravity_enabled:
                left_gravity = robot.compute_gravity("left")
                right_gravity = robot.compute_gravity("right")
                robot.send_mit_torque(
                    left_gravity,
                    right_gravity,
                )

            now = time.monotonic()
            if now >= next_print_tick:
                print_robot_state(robot)
                next_print_tick += print_period

                if args.once:
                    break
                if time.monotonic() > next_print_tick + print_period:
                    next_print_tick = time.monotonic() + print_period

            sleep_time = next_control_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            elif time.monotonic() > next_control_tick + control_period:
                next_control_tick = time.monotonic()
    finally:
        if robot is not None:
            robot.close(stop=not args.no_gravity_comp)
        print("LiteArm state reader stopped.")


if __name__ == "__main__":
    main()
