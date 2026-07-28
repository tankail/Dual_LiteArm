#!/usr/bin/env python3
"""Replay a LiteArm JSONL joint trajectory."""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from litearm_demo_common import add_common_args, add_side_arg, arm_positions, interpolate_arm, open_robot, send_arm_posvel


def load_samples(path):
    samples = []
    with Path(path).expanduser().open("r") as f:
        for line in f:
            if line.strip():
                samples.append(json.loads(line))
    if not samples:
        raise ValueError("trajectory file is empty")
    return samples


def main():
    parser = argparse.ArgumentParser(description="Replay a LiteArm JSONL trajectory.")
    add_common_args(parser)
    add_side_arg(parser)
    parser.add_argument("--input", required=True)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()

    samples = load_samples(args.input)
    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        first = np.asarray(samples[0]["positions"], dtype=float)
        current = arm_positions(robot, args.side)
        print(f"[LiteArmDemo] samples={len(samples)}, first={np.round(first, 4).tolist()}")
        if not args.execute:
            print("[LiteArmDemo] dry run. Add --execute to replay.")
            return

        # Move gently to the first recorded posture before time-based replay.
        interpolate_arm(robot, args.side, current, first, 2.0)
        start = float(samples[0].get("time", 0.0))
        wall_start = time.time()
        previous = first
        previous_time = float(samples[0].get("time", 0.0))
        for sample in samples[1:]:
            recorded_t = (float(sample.get("time", start)) - start) / max(args.scale, 1e-6)
            while time.time() - wall_start < recorded_t:
                time.sleep(0.001)
            q = np.asarray(sample["positions"], dtype=float)
            current_time = float(sample.get("time", previous_time))
            dt = max(current_time - previous_time, 0.005)
            send_arm_posvel(robot, args.side, q, velocity=max(0.2, float(np.max(np.abs(q - previous)) / dt)))
            previous = q
            previous_time = current_time
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] replay interrupted")
    finally:
        if robot is not None:
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
