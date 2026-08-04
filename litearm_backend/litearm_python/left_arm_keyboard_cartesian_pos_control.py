#!/usr/bin/env python3
"""Keyboard Cartesian position control for the left LiteArm."""

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np

from litearm_control import DEFAULT_CONFIG, DualLiteArmPython


# litearm_demo_common.py lives in the same folder (self-contained).
DEMO_SCRIPT_DIR = Path(__file__).resolve().parent
if str(DEMO_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(DEMO_SCRIPT_DIR))

from litearm_demo_common import (  # noqa: E402
    forward_kinematics,
    inverse_kinematics,
    rotation_matrix_x,
    rotation_matrix_y,
    rotation_matrix_z,
)


SIDE = "left"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Keyboard Cartesian position control for the left LiteArm."
    )
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG,
        help="Dual-arm backend YAML configuration.",
    )
    parser.add_argument(
        "--step-mm",
        type=float,
        default=5.0,
        help="Cartesian translation increment for each key press.",
    )
    parser.add_argument(
        "--rotation-step",
        type=float,
        default=0.03,
        help="Rotation increment in radians for each key press.",
    )
    parser.add_argument(
        "--velocity",
        type=float,
        default=0.3,
        help="Joint position command velocity in rad/s.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=100.0,
        help="State and display loop rate in Hz.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Read state and solve IK, but do not send motor commands.",
    )
    return parser.parse_args()


def _send_arm_position(robot, q_target, velocity):
    if robot.manager is None:
        raise RuntimeError("controller is not open")

    torque_limits = robot.dynamics[SIDE].torque_limit
    command = {
        gid: (float(q_target[index]), float(velocity), float(torque_limits[index]))
        for index, gid in enumerate(robot.arm_ids(SIDE))
    }
    # Only selected-arm IDs are included. The other arm is left untouched.
    robot.manager.set_all_pos_vel_max_torque(command)


def _read_safe_state(robot, previous_q, previous_time):
    robot.read_state()
    q = robot.arm_q(SIDE)
    dq = robot.arm_dq(SIDE)

    if (
        not np.all(np.isfinite(q))
        or not np.all(np.isfinite(dq))
        or np.any(np.abs(q) >= 100.0)
    ):
        raise RuntimeError(f"invalid {SIDE} arm feedback: q={q}, dq={dq}")

    lower, upper = robot.joint_limits[SIDE]
    tolerance = robot.feedback_limit_tolerance_rad
    if np.any(q < lower - tolerance) or np.any(q > upper + tolerance):
        raise RuntimeError(f"{SIDE} arm feedback is outside joint limits: q={q}")

    now = time.monotonic()
    if previous_q is not None and previous_time is not None:
        dt = max(now - previous_time, 1.0e-3)
        allowed = (
            robot.feedback_jump_tolerance_rad
            + robot.feedback_jump_velocity_scale * np.abs(dq) * dt
        )
        delta = np.abs(q - previous_q)
        if np.any(delta > allowed):
            joint = int(np.argmax(delta - allowed))
            raise RuntimeError(
                f"{SIDE} j{joint + 1} feedback jump: "
                f"delta={delta[joint]:.4f} rad, "
                f"allowed={allowed[joint]:.4f} rad"
            )
    return q, dq, now


def main():
    args = parse_args()
    if args.step_mm <= 0.0:
        raise SystemExit("--step-mm must be positive")
    if args.rotation_step <= 0.0:
        raise SystemExit("--rotation-step must be positive")
    if args.velocity <= 0.0:
        raise SystemExit("--velocity must be positive")
    if args.rate <= 0.0:
        raise SystemExit("--rate must be positive")

    try:
        from pynput import keyboard
    except Exception as exc:
        raise SystemExit(f"keyboard input unavailable: {exc}") from exc

    robot = None
    listener = None
    running = True
    changed = threading.Event()
    lock = threading.Lock()

    try:
        robot = DualLiteArmPython(args.config)
        robot.open()
        q, _dq, state_time = _read_safe_state(robot, None, None)

        target_position, target_rotation = forward_kinematics(
            robot.config, SIDE, q
        )
        target_position = np.asarray(target_position, dtype=float)
        target_rotation = np.asarray(target_rotation, dtype=float)
        commanded_q = q.copy()

        print("=" * 64)
        print("LiteArm left-arm keyboard Cartesian position control")
        print("=" * 64)
        print(f"Config: {robot.config_path}")
        print(f"Initial xyz (m): {np.round(target_position, 5).tolist()}")
        print("W/S: X +/-   A/D: Y +/-   Q/E: Z +/-")
        print("1/2: rotate X +/-   3/4: rotate Y +/-   5/6: rotate Z +/-")
        print("ESC: quit")
        print(f"Mode: {'DRY-RUN' if args.dry_run else 'POSITION COMMAND'}")
        print("Only left-arm motor IDs 1-7 receive commands.")

        step = args.step_mm / 1000.0

        def on_press(key):
            nonlocal running, target_position, target_rotation
            if key == keyboard.Key.esc:
                running = False
                return False

            char = getattr(key, "char", None)
            if not char:
                return

            with lock:
                if char in ("w", "W"):
                    target_position[0] += step
                elif char in ("s", "S"):
                    target_position[0] -= step
                elif char in ("a", "A"):
                    target_position[1] += step
                elif char in ("d", "D"):
                    target_position[1] -= step
                elif char in ("q", "Q"):
                    target_position[2] += step
                elif char in ("e", "E"):
                    target_position[2] -= step
                elif char == "1":
                    target_rotation = target_rotation @ rotation_matrix_x(
                        args.rotation_step
                    )
                elif char == "2":
                    target_rotation = target_rotation @ rotation_matrix_x(
                        -args.rotation_step
                    )
                elif char == "3":
                    target_rotation = target_rotation @ rotation_matrix_y(
                        args.rotation_step
                    )
                elif char == "4":
                    target_rotation = target_rotation @ rotation_matrix_y(
                        -args.rotation_step
                    )
                elif char == "5":
                    target_rotation = target_rotation @ rotation_matrix_z(
                        args.rotation_step
                    )
                elif char == "6":
                    target_rotation = target_rotation @ rotation_matrix_z(
                        -args.rotation_step
                    )
                else:
                    return
                changed.set()

        listener = keyboard.Listener(on_press=on_press)
        listener.start()

        period = 1.0 / args.rate
        next_tick = time.monotonic()
        next_display = next_tick
        previous_q = q.copy()

        while running:
            next_tick += period
            q, _dq, state_time = _read_safe_state(
                robot, previous_q, state_time
            )
            previous_q = q.copy()

            if changed.is_set():
                with lock:
                    requested_position = target_position.copy()
                    requested_rotation = target_rotation.copy()
                    changed.clear()

                q_target = inverse_kinematics(
                    robot.config,
                    SIDE,
                    requested_position,
                    requested_rotation,
                    q,
                )
                if q_target is None:
                    print(
                        f"\nIK failed; target kept at "
                        f"{np.round(requested_position, 5).tolist()}"
                    )
                else:
                    commanded_q = np.asarray(q_target, dtype=float)
                    if not args.dry_run:
                        _send_arm_position(robot, commanded_q, args.velocity)
                    print(
                        f"\ncommand q={np.round(commanded_q, 4).tolist()} "
                        f"target xyz={np.round(requested_position, 5).tolist()}"
                    )

            now = time.monotonic()
            if now >= next_display:
                current_position, _rotation = forward_kinematics(
                    robot.config, SIDE, q
                )
                with lock:
                    display_target = target_position.copy()
                print(
                    f"\rtarget={np.round(display_target, 4).tolist()} "
                    f"current={np.round(current_position, 4).tolist()} "
                    f"q={np.round(q, 3).tolist()}",
                    end="",
                    flush=True,
                )
                next_display = now + 0.1

            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            elif time.monotonic() > next_tick + period:
                next_tick = time.monotonic()

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if listener is not None:
            listener.stop()
        if robot is not None:
            robot.close(stop=True)
        print("\nLeft-arm Cartesian controller stopped.")


if __name__ == "__main__":
    main()
