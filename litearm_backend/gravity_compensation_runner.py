#!/usr/bin/env python3
"""Run the known-good C++ LiteArm gravity compensation examples.

This wrapper does not compute or send torque itself. It starts the C++
left/right arm gravity compensation programs with the same config and URDF
paths used when running them manually.
"""

import argparse
import os
import signal
import subprocess
import sys
import time


BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.normpath(os.path.join(BACKEND_DIR, ".."))
EXEC_DIR = os.path.join(ROOT_DIR, "install", "litearm_robot", "lib", "litearm_robot")
LIB_DIR = os.path.join(ROOT_DIR, "install", "litearm_robot", "lib")
ROS_LIB_DIR = "/opt/ros/humble/lib"
CONFIG_DIR = os.path.join(ROOT_DIR, "src", "litearm_config", "robot_param")
URDF_DIR = os.path.join(ROOT_DIR, "src", "litearm_robot", "urdf")


DEFAULTS = {
    "dual": {
        "exe": os.path.join(EXEC_DIR, "dual_arm_gravity_compensation"),
    },
    "left": {
        "exe": os.path.join(EXEC_DIR, "left_arm_gravity_compensation"),
        "config": os.path.join(CONFIG_DIR, "litearm_left_arm.yaml"),
        "urdf": os.path.join(URDF_DIR, "LiteArm_A10_251224_left_arm.urdf"),
    },
    "right": {
        "exe": os.path.join(EXEC_DIR, "right_arm_gravity_compensation"),
        "config": os.path.join(CONFIG_DIR, "litearm_right_arm.yaml"),
        "urdf": os.path.join(URDF_DIR, "LiteArm_A10_251224_right_arm.urdf"),
    },
}


def _check_file(path, label, executable=False):
    if not os.path.exists(path):
        raise FileNotFoundError(f"{label} not found: {path}")
    if executable and not os.access(path, os.X_OK):
        raise PermissionError(f"{label} is not executable: {path}")


def _start_side(side, overrides):
    cfg = DEFAULTS[side].copy()
    cfg.update({k: v for k, v in overrides.items() if v})

    _check_file(cfg["exe"], f"{side} executable", executable=True)
    _check_file(cfg["config"], f"{side} config")
    _check_file(cfg["urdf"], f"{side} URDF")

    cmd = [cfg["exe"], cfg["config"], cfg["urdf"]]
    env = os.environ.copy()
    ld_paths = [LIB_DIR]
    if os.path.isdir(ROS_LIB_DIR):
        ld_paths.append(ROS_LIB_DIR)
    if env.get("LD_LIBRARY_PATH"):
        ld_paths.append(env["LD_LIBRARY_PATH"])
    env["LD_LIBRARY_PATH"] = os.pathsep.join(ld_paths)
    print(f"[GravityRunner] starting {side}: {' '.join(cmd)}", flush=True)
    return subprocess.Popen(cmd, cwd=ROOT_DIR, env=env)


def _start_dual(overrides):
    exe = overrides.get("dual_exe") or DEFAULTS["dual"]["exe"]
    _check_file(exe, "dual executable", executable=True)

    left_cfg = overrides.get("left_config") or DEFAULTS["left"]["config"]
    right_cfg = overrides.get("right_config") or DEFAULTS["right"]["config"]
    left_urdf = overrides.get("left_urdf") or DEFAULTS["left"]["urdf"]
    right_urdf = overrides.get("right_urdf") or DEFAULTS["right"]["urdf"]
    for label, path in (
        ("left config", left_cfg),
        ("right config", right_cfg),
        ("left URDF", left_urdf),
        ("right URDF", right_urdf),
    ):
        _check_file(path, label)

    env = os.environ.copy()
    ld_paths = [LIB_DIR]
    if os.path.isdir(ROS_LIB_DIR):
        ld_paths.append(ROS_LIB_DIR)
    if env.get("LD_LIBRARY_PATH"):
        ld_paths.append(env["LD_LIBRARY_PATH"])
    env["LD_LIBRARY_PATH"] = os.pathsep.join(ld_paths)

    cmd = [exe, left_cfg, right_cfg, left_urdf, right_urdf]
    print(f"[GravityRunner] starting dual: {' '.join(cmd)}", flush=True)
    return subprocess.Popen(cmd, cwd=ROOT_DIR, env=env)


def main():
    parser = argparse.ArgumentParser(description="LiteArm dual-arm gravity compensation runner")
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--dual-exe")
    parser.add_argument("--left-exe")
    parser.add_argument("--right-exe")
    parser.add_argument("--left-config")
    parser.add_argument("--right-config")
    parser.add_argument("--left-urdf")
    parser.add_argument("--right-urdf")
    args = parser.parse_args()

    children = []
    stopping = False

    def stop_children(signum=None, frame=None):
        nonlocal stopping
        stopping = True
        print("[GravityRunner] stopping children", flush=True)
        for proc in children:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
        deadline = time.time() + 3.0
        for proc in children:
            while proc.poll() is None and time.time() < deadline:
                time.sleep(0.05)
            if proc.poll() is None:
                proc.terminate()
        for proc in children:
            if proc.poll() is None:
                proc.kill()

    signal.signal(signal.SIGINT, stop_children)
    signal.signal(signal.SIGTERM, stop_children)

    try:
        if args.side == "both":
            dual_exe = args.dual_exe or DEFAULTS["dual"]["exe"]
            if os.path.exists(dual_exe):
                children.append(_start_dual({
                    "dual_exe": args.dual_exe,
                    "left_config": args.left_config,
                    "right_config": args.right_config,
                    "left_urdf": args.left_urdf,
                    "right_urdf": args.right_urdf,
                }))
            else:
                print("[GravityRunner] dual executable not found; falling back to two single-arm processes",
                      flush=True)

        if not children and args.side in ("left", "both"):
            children.append(_start_side("left", {
                "exe": args.left_exe,
                "config": args.left_config,
                "urdf": args.left_urdf,
            }))
        if not children and args.side in ("right", "both"):
            children.append(_start_side("right", {
                "exe": args.right_exe,
                "config": args.right_config,
                "urdf": args.right_urdf,
            }))

        while not stopping:
            for proc in children:
                code = proc.poll()
                if code is not None:
                    stop_children()
                    print(f"[GravityRunner] child exited with code {code}", flush=True)
                    return code if code else 0
            time.sleep(0.1)
    except Exception as exc:
        stop_children()
        print(f"[GravityRunner] error: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        stop_children()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
