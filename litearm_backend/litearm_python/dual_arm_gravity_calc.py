#!/usr/bin/env python3
"""Print dual LiteArm gravity compensation torques without driving motors."""

import argparse
import signal
import time

from litearm_control import DEFAULT_CONFIG, DualLiteArmPython


keep_running = True


def _signal_handler(_signum, _frame):
    global keep_running
    keep_running = False
    print("\nInterrupted; exiting gravity calculator.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="LiteArm dual-arm gravity torque calculator"
    )
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
        "--state-rate",
        type=float,
        default=200.0,
        help="state read/validation rate in Hz (default: 200)",
    )
    parser.add_argument("--samples", type=int, default=0, help="0 means run until Ctrl+C")
    return parser.parse_args()


def _gravity_terms(robot, side, q):
    raw = robot.dynamics[side].gravity(q)
    clipped = robot.dynamics[side].clip(raw)
    return raw, clipped


def _print_table(side, q, dq, raw, clipped):
    print(f"\n[{side}] tau = G(q)")
    print("+------+----------+----------+----------+----------+")
    print("| joint| q(deg)   | q(rad)   | dq       | G(Nm)    |")
    print("+------+----------+----------+----------+----------+")
    for i in range(7):
        print(
            f"| j{i + 1:<4}"
            f"| {q[i] * 180.0 / 3.141592653589793:8.2f} "
            f"| {q[i]:8.4f} "
            f"| {dq[i]:8.4f} "
            f"| {clipped[i]:8.4f} |"
        )
    print("+------+----------+----------+----------+----------+")
    print(
        f"| |G raw| sum = {abs(raw).sum():.4f} Nm, "
        f"|G clipped| sum = {abs(clipped).sum():.4f} Nm |"
    )


def main():
    global keep_running
    args = parse_args()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    robot = DualLiteArmPython(args.config)
    robot.open()
    robot.read_state(wait_after_request=0.1)
    robot.validate_arms()

    robot.print_summary()
    print("\nDual-arm gravity torque calculator")
    print("tau = G(q)")
    print("No MIT torque output is sent.")
    if args.print_interval is not None:
        print_period = max(float(args.print_interval), 0.001)
        output_rate = 1.0 / print_period
    else:
        output_rate = max(float(args.rate), 0.1)
        print_period = 1.0 / output_rate
    state_rate = max(float(args.state_rate), output_rate, 1.0)
    state_period = 1.0 / state_rate
    print(f"state read/validation rate: {state_rate:.2f} Hz")
    print(f"terminal sample/print rate: {output_rate:.2f} Hz")

    sample = 0
    next_state_tick = time.monotonic()
    next_print_tick = next_state_tick
    try:
        while keep_running:
            next_state_tick += state_period
            robot.read_state()
            robot.validate_arms()

            now = time.monotonic()
            if now >= next_print_tick:
                sample += 1
                left_q = robot.arm_q("left")
                left_dq = robot.arm_dq("left")
                right_q = robot.arm_q("right")
                right_dq = robot.arm_dq("right")
                left_raw, left_clipped = _gravity_terms(robot, "left", left_q)
                right_raw, right_clipped = _gravity_terms(robot, "right", right_q)

                print(f"\n========== sample #{sample} ==========")
                _print_table("left", left_q, left_dq, left_raw, left_clipped)
                _print_table("right", right_q, right_dq, right_raw, right_clipped)

                next_print_tick += print_period
                if time.monotonic() > next_print_tick + print_period:
                    next_print_tick = time.monotonic() + print_period

                if args.samples > 0 and sample >= args.samples:
                    break

            sleep_time = next_state_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            elif time.monotonic() > next_state_tick + state_period:
                next_state_tick = time.monotonic()
    finally:
        robot.close(stop=False)
        print("Gravity calculator stopped.")


if __name__ == "__main__":
    main()
