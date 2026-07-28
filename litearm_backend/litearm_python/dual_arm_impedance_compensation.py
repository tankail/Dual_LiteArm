#!/usr/bin/env python3
"""Dual LiteArm joint-space impedance control in Python.

The torque calculation is the same one used by dual_arm_impedance_calc.py:

    tau = G(q) + Kp * (q_target - q) - Kd * dq

The clipped result is sent directly as the MIT torque command unless
--dry-run is used.
"""

import argparse
import signal
import sys
import time

import numpy as np

from litearm_control import DEFAULT_CONFIG, DualLiteArmPython, format_vector, valid_vector


keep_running = True


def _signal_handler(_signum, _frame):
    global keep_running
    keep_running = False
    print("\nInterrupted; stopping impedance control.")


def _target_argument(values, name):
    if values is None:
        return None
    target = np.array(values, dtype=np.float64)
    if not valid_vector(target, 7):
        raise ValueError(f"{name} requires 7 valid joint positions in radians")
    return target[:7].copy()


def parse_args():
    parser = argparse.ArgumentParser(description="LiteArm dual-arm impedance control")
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="dual-arm backend YAML")
    parser.add_argument("--rate", type=float, default=200.0, help="control rate in Hz")
    parser.add_argument("--print-interval", type=float, default=0.5, help="print interval")
    parser.add_argument("--left-target", nargs=7, type=float, default=None)
    parser.add_argument("--right-target", nargs=7, type=float, default=None)
    parser.add_argument("--dry-run", action="store_true", help="print torques only")
    return parser.parse_args()


def main():
    global keep_running
    args = parse_args()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    robot = None
    try:
        robot = DualLiteArmPython(args.config)

        robot.open()
        robot.read_state(wait_after_request=0.1)
        robot.validate_arms()
        startup_state = robot.current_arm_targets()
        targets = {
            "left": startup_state["left"].copy(),
            "right": startup_state["right"].copy(),
        }

        left_target = _target_argument(args.left_target, "--left-target")
        right_target = _target_argument(args.right_target, "--right-target")
        if left_target is not None:
            targets["left"] = left_target
        if right_target is not None:
            targets["right"] = right_target

        robot.print_summary()
        print("\nDual-arm joint-space impedance control")
        print("tau = G(q) + Kp * (q_target - q) - Kd * dq")
        print(
            f"impedance Kp: {format_vector(robot.dynamics['left'].kp)}"
        )
        print(
            f"impedance Kd: {format_vector(robot.dynamics['left'].kd)}"
        )
        print(
            "torque limit: "
            f"{format_vector(robot.dynamics['left'].torque_limit)}"
        )
        print("feedback guard: disabled")
        print(f"mode: {'DRY-RUN' if args.dry_run else 'MIT torque output'}")
        print(f"[left] q_target(rad):  {format_vector(targets['left'])}")
        print(f"[right] q_target(rad): {format_vector(targets['right'])}")

        next_tick = time.monotonic()
        period = 1.0 / max(args.rate, 1.0)
        last_print = 0.0
        loop_count = 0

        while keep_running:
            next_tick += period
            robot.read_state()
            robot.validate_arms()
            left_q = robot.arm_q("left")
            left_dq = robot.arm_dq("left")
            right_q = robot.arm_q("right")
            right_dq = robot.arm_dq("right")

            left_terms = robot.compute_impedance(
                "left", targets["left"], q=left_q, dq=left_dq
            )
            right_terms = robot.compute_impedance(
                "right", targets["right"], q=right_q, dq=right_dq
            )
            # Send exactly the same clipped tau produced by the calculator.
            left_torque = left_terms.clipped
            right_torque = right_terms.clipped
            if not args.dry_run:
                robot.send_mit_torque(left_torque, right_torque)

            now = time.monotonic()
            if now - last_print >= args.print_interval:
                print(f"\n--- impedance loop #{loop_count} ---")
                print(f"[left] q(rad):  {format_vector(left_q)}")
                print(f"[left] tau(Nm): {format_vector(left_torque)}")
                print(f"[right] q(rad): {format_vector(right_q)}")
                print(f"[right] tau(Nm): {format_vector(right_torque)}")
                last_print = now

            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            elif time.monotonic() > next_tick + period:
                next_tick = time.monotonic()
            loop_count += 1
    except Exception as exc:
        print(f"\n[Error] impedance control stopped: {exc}", file=sys.stderr)
        return 1
    finally:
        if robot is not None:
            try:
                robot.close(stop=not args.dry_run)
            except Exception as close_exc:
                print(f"[Safety] failed to close robot ports: {close_exc}", file=sys.stderr)
            print("Impedance control stopped.")


if __name__ == "__main__":
    raise SystemExit(main())
