#!/usr/bin/env python3
"""Panthera-style runner for LiteArm Python control examples.

The backend starts this file after releasing its serial handles. The selected
LiteArm script then owns the serial ports until it receives Ctrl+C/SIGTERM.
"""

import argparse
import os
import sys


BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
SCRIPT_DIR = os.path.abspath(
    os.environ.get(
        "LITEARM_SCRIPT_DIR",
        os.path.join(BACKEND_DIR, "litearm_python"),
    )
)
DEFAULT_CONFIG = os.path.join(
    BACKEND_DIR, "robot_param", "litearm_arms.yaml"
)
TEACH_DIR = os.path.join(BACKEND_DIR, "litearm_python")


def _resolve_script_path(script_name):
    name = (script_name or "").replace("\\", "/").lstrip("/")
    if not name.endswith(".py"):
        name += ".py"
    path = os.path.abspath(os.path.normpath(os.path.join(SCRIPT_DIR, name)))
    if os.path.commonpath([SCRIPT_DIR, path]) != SCRIPT_DIR:
        return None, name
    return path, name


def _available_scripts():
    if not os.path.isdir(SCRIPT_DIR):
        return []
    return sorted(
        filename
        for filename in os.listdir(SCRIPT_DIR)
        if (
            filename.endswith(".py")
            and not filename.startswith("__")
            and filename
            not in (
                "litearm_control.py",
                "litearm_demo_common.py",
                "motor_driver.py",
            )
        )
    )


def main():
    parser = argparse.ArgumentParser(description="LiteArm Python script runner")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("script", nargs="?")
    parser.add_argument(
        "script_args",
        nargs=argparse.REMAINDER,
        help="arguments passed to the selected script",
    )
    args = parser.parse_args()

    if not args.script:
        print("Available scripts:")
        for filename in _available_scripts():
            print(f"  {filename}")
        return 1

    script_path, script_name = _resolve_script_path(args.script)
    if not script_path or not os.path.isfile(script_path):
        print(f"Script not found: {script_path or script_name}", file=sys.stderr)
        return 1

    command = [
        sys.executable,
        script_path,
        "--config",
        os.path.abspath(args.config),
    ]
    command.extend(args.script_args)

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    python_paths = [SCRIPT_DIR, TEACH_DIR]
    if env.get("PYTHONPATH"):
        python_paths.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(python_paths)

    print(f"[ScriptRunner] mode script: {script_name}", flush=True)
    print(f"[ScriptRunner] config: {os.path.abspath(args.config)}", flush=True)
    print(f"[ScriptRunner] command: {' '.join(command)}", flush=True)

    # Replace the runner process so the backend's SIGINT reaches the actual
    # control script directly and its finally block can close the serial ports.
    os.execvpe(sys.executable, command, env)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
