#!/usr/bin/env python3
"""Record LiteArm joint state to JSONL."""

import argparse
import json
import time
from pathlib import Path

from litearm_demo_common import ARM_ALL_IDS, add_common_args, add_side_arg, open_robot, read_states


def main():
    parser = argparse.ArgumentParser(description="Record LiteArm positions, velocities and torques.")
    add_common_args(parser)
    add_side_arg(parser)
    parser.add_argument("--output", default="litearm_trajectory.jsonl")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--rate", type=float, default=100.0)
    args = parser.parse_args()

    output = Path(args.output).expanduser().resolve()
    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        ids = ARM_ALL_IDS[args.side]
        output.parent.mkdir(parents=True, exist_ok=True)
        end = time.time() + max(0.0, args.duration)
        with output.open("w") as f:
            print(f"[LiteArmDemo] recording to {output}")
            while time.time() < end:
                timestamp = time.time()
                states = read_states(robot, samples=1)
                sample = {
                    "time": timestamp,
                    "positions": [states[gid].pos for gid in ids],
                    "velocities": [states[gid].vel for gid in ids],
                    "torques": [states[gid].torque for gid in ids],
                }
                f.write(json.dumps(sample) + "\n")
                f.flush()
                time.sleep(1.0 / max(args.rate, 1.0))
        print("[LiteArmDemo] recording finished")
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] recording interrupted")
    finally:
        if robot is not None:
            robot.close_all()


if __name__ == "__main__":
    main()
