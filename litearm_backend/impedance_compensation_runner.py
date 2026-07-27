#!/usr/bin/env python3
"""Run the C++ dual-arm joint-space impedance controller."""

import argparse
import os
import signal
import subprocess
import sys
import time


BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.normpath(os.path.join(BACKEND_DIR, ".."))
EXECUTABLE = os.path.join(
    ROOT_DIR, "install", "litearm_robot", "lib", "litearm_robot",
    "dual_arm_impedance_compensation",
)
LIB_DIR = os.path.join(ROOT_DIR, "install", "litearm_robot", "lib")
ROS_LIB_DIR = "/opt/ros/humble/lib"
CONFIG_DIR = os.path.join(ROOT_DIR, "src", "litearm_config", "robot_param")
URDF_DIR = os.path.join(ROOT_DIR, "src", "litearm_robot", "urdf")


def main():
    parser = argparse.ArgumentParser(
        description="LiteArm dual-arm joint-space impedance controller")
    parser.add_argument("--executable", default=EXECUTABLE)
    parser.add_argument(
        "--left-config",
        default=os.path.join(CONFIG_DIR, "litearm_left_arm.yaml"),
    )
    parser.add_argument(
        "--right-config",
        default=os.path.join(CONFIG_DIR, "litearm_right_arm.yaml"),
    )
    parser.add_argument(
        "--left-urdf",
        default=os.path.join(URDF_DIR, "LiteArm_A10_251224_left_arm.urdf"),
    )
    parser.add_argument(
        "--right-urdf",
        default=os.path.join(URDF_DIR, "LiteArm_A10_251224_right_arm.urdf"),
    )
    parser.add_argument("--left-target", nargs=7, type=float)
    parser.add_argument("--right-target", nargs=7, type=float)
    args = parser.parse_args()

    required = (
        ("executable", args.executable),
        ("left config", args.left_config),
        ("right config", args.right_config),
        ("left URDF", args.left_urdf),
        ("right URDF", args.right_urdf),
    )
    for label, path in required:
        if not os.path.exists(path):
            print(f"[ImpedanceRunner] {label} not found: {path}", file=sys.stderr)
            return 1

    env = os.environ.copy()
    ld_paths = [LIB_DIR]
    if os.path.isdir(ROS_LIB_DIR):
        ld_paths.append(ROS_LIB_DIR)
    if env.get("LD_LIBRARY_PATH"):
        ld_paths.append(env["LD_LIBRARY_PATH"])
    env["LD_LIBRARY_PATH"] = os.pathsep.join(ld_paths)

    command = [
        args.executable,
        args.left_config,
        args.right_config,
        args.left_urdf,
        args.right_urdf,
    ]
    if args.left_target is not None:
        command.extend(["--left-target", *[str(value) for value in args.left_target]])
    if args.right_target is not None:
        command.extend(["--right-target", *[str(value) for value in args.right_target]])
    print(
        "[ImpedanceRunner] starting: " + " ".join(command),
        flush=True,
    )
    child = None

    def stop_child(signum=None, frame=None):
        del signum, frame
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)

    signal.signal(signal.SIGINT, stop_child)
    signal.signal(signal.SIGTERM, stop_child)

    try:
        child = subprocess.Popen(
            command,
            cwd=ROOT_DIR,
            env=env,
        )
        return child.wait()
    except KeyboardInterrupt:
        stop_child()
        if child is not None:
            return child.wait()
        return 1
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            time.sleep(0.2)
            if child.poll() is None:
                child.kill()


if __name__ == "__main__":
    raise SystemExit(main())
