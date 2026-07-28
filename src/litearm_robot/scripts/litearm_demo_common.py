#!/usr/bin/env python3
"""Shared helpers for LiteArm Python demo scripts."""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml
import numpy as np
from serial.tools import list_ports

try:
    import pinocchio as pin
except ImportError:
    pin = None


SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_PKG_DIR = SCRIPT_DIR.parent
SRC_DIR = ROBOT_PKG_DIR.parent
ROOT_DIR = SRC_DIR.parent
TEACH_DIR = ROBOT_PKG_DIR / "teach"
DEFAULT_CONFIG = ROOT_DIR / "litearm_backend" / "robot_param" / "litearm_arms.yaml"
EXEC_DIR = ROOT_DIR / "install" / "litearm_robot" / "lib" / "litearm_robot"
LITEARM_LIB_DIR = ROOT_DIR / "install" / "litearm_robot" / "lib"
ROS_LIB_DIR = Path("/opt/ros/humble/lib")

if str(TEACH_DIR) not in sys.path:
    sys.path.insert(0, str(TEACH_DIR))

from motor_driver import MultiMotorManager  # noqa: E402


ARM_IDS = {
    "left": list(range(1, 8)),
    "right": list(range(9, 16)),
    "both": list(range(1, 8)) + list(range(9, 16)),
}

ARM_ALL_IDS = {
    "left": list(range(1, 9)),
    "right": list(range(9, 17)),
    "both": list(range(1, 17)),
}

DEFAULT_MAX_TORQUE = {
    1: 15.0, 2: 25.0, 3: 25.0, 4: 15.0, 5: 6.0, 6: 6.0, 7: 4.0, 8: 2.0,
    9: 15.0, 10: 25.0, 11: 25.0, 12: 15.0, 13: 6.0, 14: 6.0, 15: 4.0, 16: 2.0,
}


def add_common_args(parser):
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="Backend-style LiteArm YAML config. Default: litearm_backend/robot_param/litearm_arms.yaml",
    )
    parser.add_argument(
        "--no-init",
        action="store_true",
        help="Open serial ports but skip board initialization.",
    )
    return parser


def add_side_arg(parser, default="both"):
    parser.add_argument(
        "--side",
        choices=("left", "right", "both"),
        default=default,
        help="Arm side to operate.",
    )
    return parser


def load_config(path):
    path = Path(path).expanduser().resolve()
    with path.open("r") as f:
        cfg = yaml.safe_load(f)
    cfg["_config_dir"] = str(path.parent)
    cfg["_config_path"] = str(path)
    return cfg


def build_port_motor_types(cfg):
    result = {}
    for port, values in cfg.get("motor_types", {}).items():
        if isinstance(values, list):
            result[port] = {i + 1: str(value) for i, value in enumerate(values)}
        elif isinstance(values, dict):
            result[port] = {int(k): str(v) for k, v in values.items()}
    return result


def _split_numeric_port(path):
    i = len(path)
    while i > 0 and path[i - 1].isdigit():
        i -= 1
    if i == len(path):
        return path, None
    return path[:i], int(path[i:])


def _serial_sort_key(path):
    prefix, num = _split_numeric_port(path)
    return (prefix, num if num is not None else 10**9, path)


def candidate_serial_ports(prefix):
    ports = []
    for info in list_ports.comports():
        device = getattr(info, "device", "")
        if not device.startswith(prefix):
            continue
        vid = getattr(info, "vid", None)
        pid = getattr(info, "pid", None)
        if pid is None or vid is None or (pid == 0xFFFF and vid in (0xCAF1, 0xCAE1)):
            ports.append(device)
    return sorted(set(ports), key=_serial_sort_key)


def resolve_serial_ports(cfg):
    serial_cfg = cfg.get("serial", {})
    auto_resolve = serial_cfg.get("auto_resolve", True)
    wait_timeout = float(serial_cfg.get("wait_timeout", 5.0))
    wait_interval = float(serial_cfg.get("wait_interval", 0.2))
    configured = list(cfg.get("ports", {}).items())
    if not configured:
        return cfg

    deadline = time.time() + wait_timeout
    last_detected = {}
    resolved = None

    while True:
        attempted = {}
        ok = True
        grouped = {}
        for port, _ids in configured:
            prefix, _num = _split_numeric_port(port)
            grouped.setdefault(prefix, []).append(port)

        for prefix, requested_ports in grouped.items():
            detected = candidate_serial_ports(prefix)
            last_detected[prefix] = detected
            exact_available = all(os.path.exists(p) for p in requested_ports)
            if exact_available:
                for p in requested_ports:
                    attempted[p] = p
                continue
            if auto_resolve and len(detected) >= len(requested_ports):
                for old, new in zip(requested_ports, detected):
                    attempted[old] = new
                continue
            ok = False

        if ok:
            resolved = attempted
            break
        if time.time() >= deadline:
            break
        time.sleep(wait_interval)

    if resolved is None:
        details = ", ".join(
            f"{prefix}: {ports or 'none'}" for prefix, ports in last_detected.items()
        )
        raise FileNotFoundError(
            f"No usable LiteArm serial ports. configured={list(cfg.get('ports', {}).keys())}; detected={details}"
        )

    for old, new in resolved.items():
        if old != new:
            print(f"[LiteArmDemo] serial remap: {old} -> {new}")

    cfg["ports"] = {resolved[old]: ids for old, ids in configured}
    cfg["motor_types"] = {
        resolved.get(old, old): types
        for old, types in cfg.get("motor_types", {}).items()
        if old in resolved
    }
    return cfg


def open_robot(config_path=DEFAULT_CONFIG, init_board=True):
    cfg = resolve_serial_ports(load_config(config_path))
    port_map = dict(cfg.get("ports", {}))
    port_types = build_port_motor_types(cfg)

    print(f"[LiteArmDemo] config: {cfg['_config_path']}")
    for port, ids in port_map.items():
        types = [port_types.get(port, {}).get(i, "NONE") for i in ids]
        print(f"[LiteArmDemo] {port}: ids={ids}, types={types}")

    robot = MultiMotorManager(port_map, port_types)
    robot.open_all()
    if init_board:
        robot.init_all()
    return robot, cfg


def read_states(robot, samples=3, delay=0.03):
    states = {}
    for _ in range(max(1, samples)):
        robot.request_all_states()
        time.sleep(delay)
        states = robot.get_all_states()
    return states


def current_positions(robot, motor_ids=None):
    states = read_states(robot)
    motor_ids = motor_ids or robot.global_ids
    return {gid: states[gid].pos for gid in motor_ids if gid in states and abs(states[gid].pos) < 100}


def print_state_table(states, motor_ids=None):
    ids = motor_ids or sorted(states)
    print("")
    print(" gid |    pos(rad) |  vel(rad/s) | torque(Nm) | mode | fault")
    print("-----+-------------+-------------+------------+------+------")
    for gid in ids:
        st = states.get(gid)
        if not st:
            continue
        print(
            f"{gid:4d} | {st.pos:11.4f} | {st.vel:11.4f} | "
            f"{st.torque:10.4f} | {st.mode:4d} | {st.fault:5d}"
        )


def hold_current_position_cmd(robot, velocity=0.5, max_torque=None):
    pos = current_positions(robot)
    cmd = {}
    for gid in robot.global_ids:
        if gid in pos:
            torque_limit = DEFAULT_MAX_TORQUE.get(gid, 5.0)
            if max_torque is not None:
                torque_limit = float(max_torque)
            cmd[gid] = (pos[gid], float(velocity), torque_limit)
    return cmd


def run_gravity_process(side="both", duration=None):
    env = os.environ.copy()
    ld_parts = [str(LITEARM_LIB_DIR), str(ROS_LIB_DIR), env.get("LD_LIBRARY_PATH", "")]
    env["LD_LIBRARY_PATH"] = ":".join(p for p in ld_parts if p)

    if side == "both":
        exe = EXEC_DIR / "dual_arm_gravity_compensation"
        cmd = [str(exe)]
    elif side == "left":
        exe = EXEC_DIR / "left_arm_gravity_compensation"
        cmd = [str(exe)]
    else:
        exe = EXEC_DIR / "right_arm_gravity_compensation"
        cmd = [str(exe)]

    if not exe.exists():
        raise FileNotFoundError(f"gravity executable not found: {exe}")

    print(f"[LiteArmDemo] starting gravity compensation: {' '.join(cmd)}")
    proc = subprocess.Popen(cmd, cwd=str(ROOT_DIR), env=env, start_new_session=True)
    try:
        if duration is None:
            while proc.poll() is None:
                time.sleep(0.2)
        else:
            end = time.time() + float(duration)
            while time.time() < end and proc.poll() is None:
                time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[LiteArmDemo] stopping gravity compensation")
    finally:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except Exception:
                proc.terminate()
            try:
                proc.wait(timeout=4.0)
            except Exception:
                proc.kill()


def positive_float(value):
    value = float(value)
    if value <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


_MODEL_CACHE = {}


def _urdf_path(cfg, side):
    key = "left_arm" if side == "left" else "right_arm"
    raw = cfg.get("urdf", {}).get(key, "")
    path = Path(raw)
    if not path.is_absolute():
        path = Path(cfg["_config_dir"]) / path
    return path.resolve()


def joint_names_for_side(cfg, side):
    return list(cfg["groups"][side]["joint_names"][:7])


def load_pinocchio_model(cfg, side):
    if pin is None:
        raise RuntimeError("Pinocchio is not installed in the current Python environment")
    cache_key = (str(_urdf_path(cfg, side)), side)
    if cache_key in _MODEL_CACHE:
        return _MODEL_CACHE[cache_key]

    urdf_path = _urdf_path(cfg, side)
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")
    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    frame_name = cfg["groups"][side].get(
        "end_effector_link", f"{'l' if side == 'left' else 'r'}_joint7_link"
    )
    frame_id = model.getFrameId(frame_name)
    if frame_id >= len(model.frames):
        raise RuntimeError(f"end-effector frame not found: {frame_name}")
    joint_ids = [model.getJointId(name) for name in joint_names_for_side(cfg, side)]
    if any(jid == 0 for jid in joint_ids):
        raise RuntimeError(f"joint names not found in URDF for {side}: {joint_names_for_side(cfg, side)}")
    result = (model, data, frame_id, joint_ids)
    _MODEL_CACHE[cache_key] = result
    return result


def _make_q(model, q_arm):
    q = np.zeros(model.nq)
    q[: min(len(q_arm), model.nq)] = np.asarray(q_arm, dtype=float)[: model.nq]
    return q


def forward_kinematics(cfg, side, q_arm):
    model, data, frame_id, _joint_ids = load_pinocchio_model(cfg, side)
    q = _make_q(model, q_arm)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    placement = data.oMf[frame_id]
    return placement.translation.copy(), placement.rotation.copy()


def jacobian(cfg, side, q_arm):
    model, data, frame_id, _joint_ids = load_pinocchio_model(cfg, side)
    q = _make_q(model, q_arm)
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    return pin.getFrameJacobian(
        model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    )[:, :7]


def gravity_torque(cfg, side, q_arm):
    model, data, _frame_id, _joint_ids = load_pinocchio_model(cfg, side)
    q = _make_q(model, q_arm)
    return np.asarray(
        pin.rnea(model, data, q, np.zeros(model.nv), np.zeros(model.nv))
    )[:7]


def rotation_matrix_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rotation_matrix_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rotation_matrix_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def damped_pseudoinverse(matrix, damping=0.03):
    matrix = np.asarray(matrix, dtype=float)
    rows, cols = matrix.shape
    if rows <= cols:
        return matrix.T @ np.linalg.inv(matrix @ matrix.T + damping**2 * np.eye(rows))
    return np.linalg.inv(matrix.T @ matrix + damping**2 * np.eye(cols)) @ matrix.T


def inverse_kinematics(
    cfg,
    side,
    target_position,
    target_rotation,
    q_initial,
    max_iterations=100,
    position_tolerance=1e-4,
    rotation_tolerance=1e-3,
):
    model, data, frame_id, _joint_ids = load_pinocchio_model(cfg, side)
    q_arm = np.asarray(q_initial, dtype=float).copy()[:7]
    target_position = np.asarray(target_position, dtype=float)
    target_rotation = np.asarray(target_rotation, dtype=float)

    for _ in range(max_iterations):
        q = _make_q(model, q_arm)
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        current = data.oMf[frame_id]
        position_error = target_position - current.translation
        rotation_error = pin.log3(target_rotation @ current.rotation.T)
        error = np.concatenate((position_error, rotation_error))
        if np.linalg.norm(position_error) < position_tolerance and np.linalg.norm(rotation_error) < rotation_tolerance:
            return q_arm

        pin.computeJointJacobians(model, data, q)
        pin.updateFramePlacements(model, data)
        j = pin.getFrameJacobian(
            model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )[:, :7]
        q_arm += 0.35 * damped_pseudoinverse(j, 0.04) @ error

        limits = cfg["groups"][side].get("joint_limits", {})
        lower = np.asarray(limits.get("lower", [-np.inf] * 8), dtype=float)[:7]
        upper = np.asarray(limits.get("upper", [np.inf] * 8), dtype=float)[:7]
        q_arm = np.clip(q_arm, lower, upper)
    return None


def arm_positions(robot, side):
    ids = ARM_IDS[side]
    states = read_states(robot)
    return np.asarray([states[gid].pos for gid in ids], dtype=float)


def arm_velocities(robot, side):
    ids = ARM_IDS[side]
    states = read_states(robot)
    return np.asarray([states[gid].vel for gid in ids], dtype=float)


def send_arm_posvel(robot, side, q_arm, velocity=0.5, max_torque=None):
    ids = ARM_IDS[side]
    cmd = hold_current_position_cmd(robot, velocity=velocity, max_torque=max_torque)
    for i, gid in enumerate(ids):
        limit = max_torque if max_torque is not None else DEFAULT_MAX_TORQUE[gid]
        cmd[gid] = (float(q_arm[i]), float(velocity), float(limit))
    robot.set_all_pos_vel_max_torque(cmd)


def send_arm_pvkd(robot, side, q_arm, kp, kd):
    ids = ARM_IDS[side]
    current = current_positions(robot)
    cmd = {}
    for gid in robot.global_ids:
        if gid in current:
            cmd[gid] = (current[gid], 0.0, 0.0, 0.0)
    for i, gid in enumerate(ids):
        cmd[gid] = (float(q_arm[i]), 0.0, float(kp[i]), float(kd[i]))
    robot.set_all_pos_vel_kp_kd(cmd)


def send_arm_torque(robot, side, torque, kp=None, kd=None):
    ids = ARM_IDS[side]
    current = current_positions(robot)
    if kp is None:
        kp = np.zeros(7)
    if kd is None:
        kd = np.zeros(7)
    cmd = {}
    for gid in robot.global_ids:
        if gid in current:
            cmd[gid] = (current[gid], 0.0, 0.0, 0.0, 0.0)
    for i, gid in enumerate(ids):
        cmd[gid] = (0.0, 0.0, float(torque[i]), float(kp[i]), float(kd[i]))
    robot.set_all_pos_vel_torque_kp_kd(cmd)


def friction_torque(velocity, fc, fv, threshold=0.02):
    velocity = np.asarray(velocity, dtype=float)
    fc = np.asarray(fc, dtype=float)
    fv = np.asarray(fv, dtype=float)
    coulomb = np.where(np.abs(velocity) > threshold, fc * np.sign(velocity), 0.0)
    return coulomb + fv * velocity


def interpolate_arm(robot, side, start, target, duration, velocity=0.4, zero_velocity=False):
    steps = max(1, int(float(duration) * 100))
    start = np.asarray(start, dtype=float)
    target = np.asarray(target, dtype=float)
    for i in range(steps + 1):
        alpha = i / steps
        s = alpha * alpha * (3.0 - 2.0 * alpha)
        q = (1.0 - s) * start + s * target
        send_arm_posvel(robot, side, q, velocity=velocity)
        time.sleep(float(duration) / steps)


def cartesian_target_from_current(cfg, side, q_arm, position_delta=None, rotation_delta=None):
    position, rotation = forward_kinematics(cfg, side, q_arm)
    if position_delta is not None:
        position = position + np.asarray(position_delta, dtype=float)
    if rotation_delta is not None:
        rotation = rotation @ np.asarray(rotation_delta, dtype=float)
    return position, rotation
