#!/usr/bin/env python3
"""Single-joint velocity control demo for LiteArm."""

import argparse
import math
import time

from litearm_demo_common import ARM_IDS, add_common_args, add_side_arg, open_robot, read_states


def main():
    parser = argparse.ArgumentParser(description="Alternate one joint between positive and negative velocity.")
    add_common_args(parser)
    add_side_arg(parser, default="left")
    parser.add_argument("--joint", type=int, default=1, choices=range(1, 8))
    parser.add_argument("--velocity", type=float, default=0.2)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--execute", action="store_true", help="Send velocity commands.")
    args = parser.parse_args()

    robot = None
    try:
        robot, _cfg = open_robot(args.config, init_board=not args.no_init)
        gid = ARM_IDS[args.side][args.joint - 1]
        if not args.execute:
            print(f"[LiteArmDemo] dry run. Would control global motor id {gid}. Add --execute.")
            return

        end = time.time() + max(0.0, args.duration)
        while time.time() < end:
            direction = 1.0 if (math.floor(time.time()) % 6) >= 3 else -1.0
            robot.set_all_velocity({gid: direction * args.velocity})
            states = read_states(robot, samples=1)
            st = states.get(gid)
            if st:
                print(f"\r[LiteArmDemo] gid={gid} target_vel={direction * args.velocity:+.3f} "
                      f"pos={st.pos:+.3f} vel={st.vel:+.3f}", end="", flush=True)
            time.sleep(0.05)
        print()
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] interrupted")
    finally:
        if robot is not None:
            robot.set_all_velocity({gid: 0.0} if "gid" in locals() else {})
            robot.stop_all()
            robot.close_all()


if __name__ == "__main__":
    main()
