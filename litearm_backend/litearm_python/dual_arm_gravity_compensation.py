#!/usr/bin/env python3
"""Dual LiteArm gravity compensation in Python.

This is the Python equivalent of dual_arm_gravity_compensation.cpp.
It sends MIT torque commands unless --dry-run is used.
"""

import argparse
import signal
import time

from litearm_control import DEFAULT_CONFIG, DualLiteArmPython, format_vector


keep_running = True


def _signal_handler(_signum, _frame):
    global keep_running
    keep_running = False
    print("\nInterrupted; stopping gravity compensation.")


def parse_args():
    parser = argparse.ArgumentParser(description="LiteArm dual-arm gravity compensation")
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="dual-arm backend YAML")
    parser.add_argument("--rate", type=float, default=200.0, help="control rate in Hz")
    parser.add_argument("--print-interval", type=float, default=0.5, help="print interval")
    parser.add_argument("--dry-run", action="store_true", help="print torques only")
    return parser.parse_args()


def main():
    global keep_running
    args = parse_args()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    robot = DualLiteArmPython(args.config)
    robot.open()
    robot.print_summary()
    print("\nDual-arm gravity compensation")
    print("tau = G(q)")
    print(f"mode: {'DRY-RUN' if args.dry_run else 'MIT torque output'}")

    next_tick = time.monotonic()
    period = 1.0 / max(args.rate, 1.0)
    last_print = 0.0
    loop_count = 0

    try:
        while keep_running:
            next_tick += period
            robot.read_state()
            robot.validate_arms()

            left_tau = robot.compute_gravity("left")
            right_tau = robot.compute_gravity("right")
            if not args.dry_run:
                robot.send_mit_torque(left_tau, right_tau)

            now = time.monotonic()
            if now - last_print >= args.print_interval:
                print(f"\n--- gravity loop #{loop_count} ---")
                print(f"[left] q(rad):  {format_vector(robot.arm_q('left'))}")
                print(f"[left] tau(Nm): {format_vector(left_tau)}")
                print(f"[right] q(rad): {format_vector(robot.arm_q('right'))}")
                print(f"[right] tau(Nm): {format_vector(right_tau)}")
                last_print = now

            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            elif time.monotonic() > next_tick + period:
                next_tick = time.monotonic()
            loop_count += 1
    finally:
        robot.close(stop=not args.dry_run)
        print("Gravity compensation stopped.")


if __name__ == "__main__":
    main()
