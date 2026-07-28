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

from litearm_control import (
    DEFAULT_CONFIG,
    DualLiteArmPython,
    format_vector,
    valid_vector,
)


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


def _print_table(side, q, dq, target, terms):
    print(f"\n[{side}] tau = G(q) + Kp(q_target - q) - Kd*dq")
    print("+------+----------+----------+----------+----------+----------+----------+----------+")
    print("| joint| q(deg)   | target   | error    | dq       | G(Nm)    | PD(Nm)   | tau(Nm)  |")
    print("+------+----------+----------+----------+----------+----------+----------+----------+")
    for i in range(7):
        error = target[i] - q[i]
        print(
            f"| j{i + 1:<4}"
            f"| {q[i] * 180.0 / np.pi:8.2f} "
            f"| {target[i]:8.4f} "
            f"| {error:8.4f} "
            f"| {dq[i]:8.4f} "
            f"| {terms.gravity[i]:8.4f} "
            f"| {terms.pd[i]:8.4f} "
            f"| {terms.clipped[i]:8.4f} |"
        )
    print("+------+----------+----------+----------+----------+----------+----------+----------+")
    print(
        f"| |tau raw| sum = {abs(terms.total).sum():.4f} Nm, "
        f"|tau clipped| sum = {abs(terms.clipped).sum():.4f} Nm |"
    )


def parse_args():
    parser = argparse.ArgumentParser(description="LiteArm dual-arm impedance control")
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="dual-arm backend YAML")
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="terminal sample/print rate in Hz (default: 1)",
    )
    parser.add_argument(
        "--print-interval",
        type=float,
        default=None,
        help="legacy print interval in seconds; overrides --rate",
    )
    parser.add_argument(
        "--control-rate",
        type=float,
        default=200.0,
        help="MIT torque control rate in Hz (default: 200)",
    )
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
        print("feedback: direct q/dq with joint-limit and jump fault protection")
        print(f"mode: {'DRY-RUN' if args.dry_run else 'MIT torque output'}")
        if args.print_interval is not None:
            print_period = max(float(args.print_interval), 0.001)
            output_rate = 1.0 / print_period
        else:
            output_rate = max(float(args.rate), 0.1)
            print_period = 1.0 / output_rate
        control_rate = max(float(args.control_rate), 1.0)
        print(f"control rate: {control_rate:.2f} Hz")
        print(f"terminal sample/print rate: {output_rate:.2f} Hz")
        print(f"[left] q_target(rad):  {format_vector(targets['left'])}")
        print(f"[right] q_target(rad): {format_vector(targets['right'])}")

        control_period = 1.0 / control_rate
        next_control_tick = time.monotonic()
        next_print_tick = next_control_tick
        sample = 0

        while keep_running:
            next_control_tick += control_period
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
            if now >= next_print_tick:
                sample += 1
                print(f"\n========== sample #{sample} ==========")
                _print_table(
                    "left",
                    left_q,
                    left_dq,
                    targets["left"],
                    left_terms,
                )
                _print_table(
                    "right",
                    right_q,
                    right_dq,
                    targets["right"],
                    right_terms,
                )
                next_print_tick += print_period
                if time.monotonic() > next_print_tick + print_period:
                    next_print_tick = time.monotonic() + print_period

            sleep_time = next_control_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            elif time.monotonic() > next_control_tick + control_period:
                next_control_tick = time.monotonic()
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
