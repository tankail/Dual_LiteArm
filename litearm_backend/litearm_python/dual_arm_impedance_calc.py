#!/usr/bin/env python3
"""Print dual LiteArm impedance torque terms without driving motors."""

import argparse
import signal
import time

from litearm_control import DEFAULT_CONFIG, DualLiteArmPython, format_vector


keep_running = True


def _signal_handler(_signum, _frame):
    global keep_running
    keep_running = False
    print("\nInterrupted; exiting impedance calculator.")


def parse_args():
    parser = argparse.ArgumentParser(description="LiteArm dual-arm impedance torque calculator")
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
    parser.add_argument("--samples", type=int, default=0, help="0 means run until Ctrl+C")
    return parser.parse_args()


def _print_table(side, q, dq, target, terms):
    print(f"\n[{side}] tau = G(q) + Kp(q_target - q) - Kd*dq")
    print("+------+----------+----------+----------+----------+----------+----------+----------+")
    print("| joint| q(deg)   | target   | error    | dq       | G(Nm)    | PD(Nm)   | tau(Nm)  |")
    print("+------+----------+----------+----------+----------+----------+----------+----------+")
    for i in range(7):
        error = target[i] - q[i]
        print(
            f"| j{i + 1:<4}"
            f"| {q[i] * 180.0 / 3.141592653589793:8.2f} "
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


def main():
    global keep_running
    args = parse_args()
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    robot = DualLiteArmPython(args.config)
    robot.open()
    robot.read_state(wait_after_request=0.1)
    robot.validate_arms()

    targets = robot.current_arm_targets()
    robot.print_summary()
    print("\nDual-arm impedance torque calculator")
    print("No MIT torque output is sent.")
    if args.print_interval is not None:
        period = max(float(args.print_interval), 0.001)
        output_rate = 1.0 / period
    else:
        output_rate = max(float(args.rate), 0.1)
        period = 1.0 / output_rate
    print(f"terminal sample/print rate: {output_rate:.2f} Hz")
    print(f"[left] q_target(rad):  {format_vector(targets['left'], 4)}")
    print(f"[right] q_target(rad): {format_vector(targets['right'], 4)}")

    sample = 0
    next_tick = time.monotonic()
    try:
        while keep_running:
            next_tick += period
            sample += 1
            robot.read_state()
            robot.validate_arms()
            left_q = robot.arm_q("left")
            left_dq = robot.arm_dq("left")
            right_q = robot.arm_q("right")
            right_dq = robot.arm_dq("right")
            left_terms = robot.compute_impedance("left", targets["left"], left_q, left_dq)
            right_terms = robot.compute_impedance("right", targets["right"], right_q, right_dq)

            print(f"\n========== sample #{sample} ==========")
            _print_table("left", left_q, left_dq, targets["left"], left_terms)
            _print_table("right", right_q, right_dq, targets["right"], right_terms)

            if args.samples > 0 and sample >= args.samples:
                break

            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            elif time.monotonic() > next_tick + period:
                # Do not accumulate delay when terminal output or serial
                # communication takes longer than the requested period.
                next_tick = time.monotonic()
    finally:
        robot.close(stop=False)
        print("Impedance calculator stopped.")


if __name__ == "__main__":
    main()
