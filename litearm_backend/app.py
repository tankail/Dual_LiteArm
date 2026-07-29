#!/usr/bin/env python3
"""
LiteArm A10 Digital Twin Backend Server
========================================
Flask + SocketIO backend for dual-arm LiteArm A10 robot.
Drives 4 serial ports (left/right/waist/head) via motor_driver.py.
Supports live hardware mode and demo (simulated) mode.

Usage:
    python app.py --config robot_param/litearm_full.yaml --port 5001
    python app.py --demo --port 5001
"""

import sys
import os
import time
import math
import json
import yaml
import threading
import logging
import argparse
import subprocess
import signal
import numpy as np
from serial.tools import list_ports

# ── Path setup: find motor_driver.py ───────────────────────────
BACKEND_DIR = os.path.dirname(os.path.abspath(__file__))
TEACH_DIR = os.path.normpath(os.path.join(BACKEND_DIR, '..', 'src', 'litearm_robot', 'teach'))
if TEACH_DIR not in sys.path:
    sys.path.insert(0, TEACH_DIR)

from motor_driver import MultiMotorManager, MotorState, rad_to_deg, deg_to_rad

# ── Optional: Pinocchio for FK ─────────────────────────────────
try:
    import pinocchio as pin
    from scipy.spatial.transform import Rotation as R
    HAS_PINOCCHIO = True
except ImportError:
    HAS_PINOCCHIO = False
    print("[WARN] Pinocchio not available — FK disabled")

# ── Flask / SocketIO ───────────────────────────────────────────
from flask import Flask, jsonify, send_from_directory, request as flask_request
from flask_socketio import SocketIO, emit
from flask_cors import CORS

# Quiet Werkzeug
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__, static_folder='frontend/dist', static_url_path='')
CORS(app, origins="*")
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# ── Global State ───────────────────────────────────────────────
robot = None          # MultiMotorManager or DemoRobot
config = {}
targets = {}          # {global_id: target_position}
control_mode = "position"  # "position" | "free" | "gravity_comp" | "impedance"
demo_mode = False
running = False
state_lock = threading.Lock()
target_lock = threading.Lock()
impedance_lock = threading.Lock()
mode_transition_lock = threading.RLock()
serial_io_lock = threading.RLock()
impedance_targets = np.array([], dtype=np.float64)
impedance_kp = np.array([], dtype=np.float64)
impedance_kd = np.array([], dtype=np.float64)
impedance_torque_limits = np.array([], dtype=np.float64)
script_process = None
script_name = None
script_output_lines = []
script_output_lock = threading.Lock()
SCRIPT_OUTPUT_LIMIT = 1000
SCRIPT_DIR = os.path.abspath(os.environ.get(
    'LITEARM_SCRIPT_DIR',
    os.path.join(BACKEND_DIR, 'litearm_python')))
SCRIPT_RUNNER = os.path.join(BACKEND_DIR, 'run_script.py')


def init_arrays(motor_count):
    """Resize global arrays to match config."""
    global MOTOR_COUNT, positions, velocities, torques
    global impedance_targets, impedance_kp, impedance_kd, impedance_torque_limits
    MOTOR_COUNT = motor_count
    positions = np.zeros(MOTOR_COUNT, dtype=np.float64)
    velocities = np.zeros(MOTOR_COUNT, dtype=np.float64)
    torques = np.zeros(MOTOR_COUNT, dtype=np.float64)
    impedance_targets = np.zeros(MOTOR_COUNT, dtype=np.float64)
    impedance_kp = np.zeros(MOTOR_COUNT, dtype=np.float64)
    impedance_kd = np.zeros(MOTOR_COUNT, dtype=np.float64)
    impedance_torque_limits = np.zeros(MOTOR_COUNT, dtype=np.float64)

# Flat motor arrays — sized at startup from config
MOTOR_COUNT = 0
positions = np.array([], dtype=np.float64)
velocities = np.array([], dtype=np.float64)
torques = np.array([], dtype=np.float64)
connected = False
backend_serial_fault = False
backend_serial_fault_message = ""
gravity_process = None  # External Python gravity script process
gravity_serial_released = False
gravity_transitioning = False
impedance_process = None  # External Python joint-space impedance process
impedance_serial_released = False
impedance_transitioning = False
active_config_path = None


# ═══════════════════════════════════════════════════════════════
# Config Loader
# ═══════════════════════════════════════════════════════════════

def load_config(path):
    """Load and validate litearm_full.yaml."""
    with open(path, 'r') as f:
        cfg = yaml.safe_load(f)
    # Resolve relative paths
    cfg['_config_dir'] = os.path.dirname(os.path.abspath(path))
    for key in ('file_path', 'left_arm', 'right_arm'):
        urdf_rel = cfg.get('urdf', {}).get(key, '')
        if urdf_rel and not os.path.isabs(urdf_rel):
            cfg['urdf'][key] = os.path.normpath(
                os.path.join(cfg['_config_dir'], urdf_rel))
    return cfg


def build_joint_list(cfg):
    """Flatten groups → ordered joint list for API consumers."""
    joints = []
    for gname, ginfo in cfg['groups'].items():
        names = ginfo['joint_names']
        indices = ginfo['motor_indices']
        lo = ginfo.get('joint_limits', {}).get('lower', [-5]*len(names))
        hi = ginfo.get('joint_limits', {}).get('upper', [5]*len(names))
        gripper_idx = ginfo.get('gripper_index', None)
        for i, name in enumerate(names):
            gid = indices[i]
            kind = 'gripper' if (i == gripper_idx or 'gripper' in name.lower()) else 'joint'
            joints.append({
                'name': name,
                'group': gname,
                'index': gid - 1,     # 0-based index (Panthera-compatible)
                'global_id': gid,
                'min': lo[i] if i < len(lo) else -5.0,
                'max': hi[i] if i < len(hi) else 5.0,
                'kind': kind,
            })
    joints.sort(key=lambda j: j['global_id'])
    return joints


def _finite_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _clamp_joint_target(value, joint):
    number = _finite_float(value)
    if number is None:
        return None

    lo = _finite_float(joint.get('min', -5.0))
    hi = _finite_float(joint.get('max', 5.0))
    lo = -5.0 if lo is None else lo
    hi = 5.0 if hi is None else hi
    if hi < lo:
        lo, hi = hi, lo
    return max(lo, min(hi, number))


def build_target_updates_from_flat_payload(flat=None, gripper=None):
    """
    Convert frontend flat commands into {global_id: target}.

    Current frontend commands should send all 16 joints, including both
    grippers. Older clients sent only arm joints plus a separate gripper
    scalar; keep that format working without shifting right-arm IDs.
    """
    joints = build_joint_list(config)
    joints_by_gid = {j['global_id']: j for j in joints}
    updates = {}

    if flat is not None:
        if isinstance(flat, np.ndarray):
            flat = flat.tolist()
        if isinstance(flat, (list, tuple)):
            flat_list = list(flat)
            if len(flat_list) == MOTOR_COUNT:
                for i, pos in enumerate(flat_list[:MOTOR_COUNT]):
                    gid = i + 1
                    joint = joints_by_gid.get(gid, {'min': -5.0, 'max': 5.0})
                    target = _clamp_joint_target(pos, joint)
                    if target is not None:
                        updates[gid] = target
            else:
                arm_joints = [j for j in joints if j.get('kind') != 'gripper']
                for joint, pos in zip(arm_joints, flat_list):
                    target = _clamp_joint_target(pos, joint)
                    if target is not None:
                        updates[joint['global_id']] = target

    gripper_value = _finite_float(gripper)
    if gripper_value is not None:
        for joint in joints:
            if joint.get('kind') != 'gripper':
                continue
            target = _clamp_joint_target(gripper_value, joint)
            if target is not None:
                updates[joint['global_id']] = target

    return updates


def build_port_motor_types(cfg):
    """Normalize YAML motor type lists to {port: {local_id: type}}."""
    result = {}
    for port, values in cfg.get('motor_types', {}).items():
        if isinstance(values, list):
            result[port] = {i + 1: str(value) for i, value in enumerate(values)}
        elif isinstance(values, dict):
            result[port] = {int(k): str(v) for k, v in values.items()}
    return result


def _split_numeric_port(path):
    """Return (/dev/ttyACM, 0) for /dev/ttyACM0; otherwise (path, None)."""
    i = len(path)
    while i > 0 and path[i - 1].isdigit():
        i -= 1
    if i == len(path):
        return path, None
    return path[:i], int(path[i:])


def _serial_sort_key(path):
    prefix, num = _split_numeric_port(path)
    return (prefix, num if num is not None else 10**9, path)


def _candidate_serial_ports(prefix):
    """Match the C++ driver behavior: scan /dev/ttyACM* and sort numerically."""
    ports = []
    for info in list_ports.comports():
        device = getattr(info, 'device', '')
        if not device.startswith(prefix):
            continue
        vid = getattr(info, 'vid', None)
        pid = getattr(info, 'pid', None)
        # The C++ driver accepts LivelyBot USB boards with PID 0xffff
        # and VID 0xcaf1/0xcae1. If VID/PID is unavailable, keep the
        # device as a fallback because some kernels do not expose it here.
        if pid is None or vid is None or (pid == 0xffff and vid in (0xcaf1, 0xcae1)):
            ports.append(device)
    return sorted(set(ports), key=_serial_sort_key)


def resolve_serial_ports(cfg):
    """
    Keep YAML port order, but resolve missing /dev/ttyACM0 style names by
    scanning existing numbered ttyACM devices. This mirrors the C++ LiteArm
    code where serial_id=1 means the first detected matching ttyACM device.
    """
    serial_cfg = cfg.get('serial', {})
    auto_resolve = serial_cfg.get('auto_resolve', True)
    wait_timeout = float(serial_cfg.get('wait_timeout', 5.0))
    wait_interval = float(serial_cfg.get('wait_interval', 0.2))
    configured = list(cfg.get('ports', {}).items())
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
            detected = _candidate_serial_ports(prefix)
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
        details = ', '.join(
            f"{prefix}: {ports or 'none'}" for prefix, ports in last_detected.items())
        raise FileNotFoundError(
            "No usable LiteArm serial ports. "
            f"configured={list(cfg.get('ports', {}).keys())}; detected={details}")

    remapped = [(old, new) for old, new in resolved.items() if old != new]
    if remapped:
        print("   Serial ports remapped after scan:")
        for old, new in remapped:
            print(f"     {old} -> {new}")

    old_ports = cfg.get('ports', {})
    old_types = cfg.get('motor_types', {})
    cfg['ports'] = {resolved[old]: ids for old, ids in configured}
    cfg['motor_types'] = {
        resolved.get(old, old): types for old, types in old_types.items()
        if old in resolved
    }
    return cfg


# ═══════════════════════════════════════════════════════════════
# Demo Robot (simulated)
# ═══════════════════════════════════════════════════════════════

class DemoRobot:
    """Fake robot that generates sinusoidal motor states."""

    def __init__(self, joint_count):
        self.joint_count = joint_count
        self.t0 = time.time()
        self.phases = np.linspace(0, 2*np.pi, joint_count, endpoint=False)

    def request_all_states(self):
        pass  # no-op

    def get_all_states(self):
        t = time.time() - self.t0
        out = {}
        for gid in range(1, self.joint_count + 1):
            idx = gid - 1
            pos = 0.3 * math.sin(t * 0.5 + self.phases[idx])
            vel = 0.15 * math.cos(t * 0.5 + self.phases[idx])
            tor = 0.5 * math.sin(t * 0.7 + self.phases[idx])
            out[gid] = MotorState(pos=pos, vel=vel, torque=tor, mode=0, fault=0)
        return out

    def set_all_pos_vel_max_torque(self, gid_to_pvt):
        pass

    def set_all_free_mode(self):
        pass

    def set_all_pos_vel_kp_kd(self, gid_to_pvkd):
        pass

    def set_all_pos_vel_torque_kp_kd(self, gid_to_pvtkd):
        pass

    def stop_all(self):
        pass

    def close_all(self):
        pass

    def open_all(self):
        pass

    def init_all(self):
        pass

    @property
    def total_motors(self):
        return self.joint_count

    @property
    def global_ids(self):
        return list(range(1, self.joint_count + 1))


# ═══════════════════════════════════════════════════════════════
# Pinocchio FK Helper
# ═══════════════════════════════════════════════════════════════

class FKEngine:
    """Forward kinematics for left/right arms using Pinocchio."""

    def __init__(self, urdf_path, left_joint_names, right_joint_names,
                 left_ee, right_ee):
        self.left_joints = left_joint_names
        self.right_joints = right_joint_names
        self.left_ee = left_ee
        self.right_ee = right_ee
        self.ready = False

        if not HAS_PINOCCHIO or not os.path.exists(urdf_path):
            return

        try:
            self.model = pin.buildModelFromUrdf(urdf_path)
            self.data = self.model.createData()
            # Pre-cache joint indices
            self.left_joint_ids = []
            for name in left_joint_names:
                jid = self.model.getJointId(name)
                self.left_joint_ids.append(jid)
            self.right_joint_ids = []
            for name in right_joint_names:
                jid = self.model.getJointId(name)
                self.right_joint_ids.append(jid)
            self.left_ee_id = self.model.getFrameId(left_ee)
            self.right_ee_id = self.model.getFrameId(right_ee)
            self.ready = True
            print(f"[FK] URDF loaded: {self.model.nq} joint DOFs, "
                  f"left EE={left_ee}, right EE={right_ee}")
        except Exception as e:
            print(f"[FK] Failed to init: {e}")

    def compute(self, joint_positions, side='left'):
        """Compute FK for one arm. Returns {pos: [x,y,z], euler: [r,p,y]} or None."""
        if not self.ready:
            return None
        try:
            q = np.zeros(self.model.nq)
            joint_ids = self.left_joint_ids if side == 'left' else self.right_joint_ids
            for i, jid in enumerate(joint_ids):
                if i < len(joint_positions):
                    q[self.model.joints[jid].idx_q] = joint_positions[i]
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            ee_id = self.left_ee_id if side == 'left' else self.right_ee_id
            placement = self.data.oMf[ee_id]
            pos = placement.translation.copy()
            rot = R.from_matrix(placement.rotation.copy())
            euler = rot.as_euler('zyx', degrees=False)  # intrinsic ZYX
            return {
                'position': pos.tolist(),
                'euler': euler.tolist(),
            }
        except Exception as e:
            return None


# ═══════════════════════════════════════════════════════════════
# Backend Control Loop
# ═══════════════════════════════════════════════════════════════

def _drop_backend_robot(reason, current_robot=None):
    """Disable backend serial I/O after a real serial failure."""
    global robot, connected, backend_serial_fault, backend_serial_fault_message
    reason_text = str(reason)
    with serial_io_lock:
        target_robot = current_robot if current_robot is not None else robot
        if target_robot is not None:
            try:
                target_robot.close_all()
            except Exception:
                pass
        if current_robot is None or robot is target_robot:
            robot = None
        connected = False
        backend_serial_fault = True
        backend_serial_fault_message = reason_text
    print(f"[Serial] Backend serial disabled: {reason_text}")


def _read_states(wait_after_request=0.0):
    """Request and copy the latest motor states into the shared arrays."""
    global positions, velocities, torques
    with serial_io_lock:
        current_robot = robot
        if current_robot is None:
            return
        try:
            current_robot.request_all_states()
            if wait_after_request > 0.0:
                time.sleep(wait_after_request)
            states = current_robot.get_all_states()
        except Exception as exc:
            _drop_backend_robot(f"state read failed: {exc}", current_robot)
            raise
    with state_lock:
        for gid in range(1, MOTOR_COUNT + 1):
            st = states.get(gid)
            idx = gid - 1
            if st and abs(st.pos) < 100:
                positions[idx] = st.pos
                velocities[idx] = st.vel
                torques[idx] = st.torque


def _hold_current_positions(current=None):
    """Prevent a mode change from sending stale position targets."""
    if current is None:
        with state_lock:
            current = positions.copy()
    with target_lock:
        for i, value in enumerate(current, start=1):
            targets[i] = float(value)


def _backend_position_commands_allowed():
    """Return whether the backend is the current position-mode owner."""
    return (
        control_mode == 'position'
        and robot is not None
        and not gravity_serial_released
        and not impedance_serial_released
        and not gravity_transitioning
        and not impedance_transitioning
    )


def _position_target_updates_allowed():
    """Only accept position targets when the backend owns position control."""
    if demo_mode:
        return control_mode == 'position'
    return _backend_position_commands_allowed()


def _snapshot_live_positions():
    """
    Capture a verified, fresh position snapshot before a controller handoff.

    The backend must not enter an external controller using a stale position
    or the driver's invalid sentinel value. Two requests also give the
    asynchronous serial receive thread time to publish a fresh state.
    """
    if robot is None or demo_mode:
        with state_lock:
            return positions.copy()

    last_valid = None
    for _ in range(3):
        _read_states(wait_after_request=0.02)
        with state_lock:
            snapshot = positions.copy()
            valid = (
                len(snapshot) == MOTOR_COUNT
                and np.all(np.isfinite(snapshot))
                and np.all(np.abs(snapshot) < 100.0)
            )
        if valid:
            last_valid = snapshot
            break
        time.sleep(0.01)

    if last_valid is None:
        raise RuntimeError("无法读取有效的最新关节位置，拒绝切换控制模式")
    return last_valid


def _send_position_hold_before_handoff(current):
    """Refresh the last position command without sending STOP."""
    with serial_io_lock:
        current_robot = robot
        if current_robot is None or demo_mode:
            return

        max_torque = float(config.get('robot', {}).get('max_torque', 15.0))
        cmd = {
            gid: (float(current[gid - 1]), 0.0, max_torque)
            for gid in range(1, MOTOR_COUNT + 1)
        }
        # Send twice so the most recent command remains a position-hold command
        # while the Python serial handle is being released.
        try:
            current_robot.set_all_pos_vel_max_torque(cmd)
            time.sleep(0.005)
            current_robot.set_all_pos_vel_max_torque(cmd)
            time.sleep(0.005)
        except Exception as exc:
            _drop_backend_robot(f"position handoff failed: {exc}", current_robot)
            raise


def _arm_target_from_snapshot(snapshot, group_name):
    """Return the seven non-gripper joint positions for one arm."""
    group = config.get('groups', {}).get(group_name, {})
    motor_indices = group.get('motor_indices', [])
    gripper_index = group.get('gripper_index', None)
    values = []
    for local_index, gid in enumerate(motor_indices):
        if local_index == gripper_index:
            continue
        index = int(gid) - 1
        if 0 <= index < len(snapshot):
            values.append(float(snapshot[index]))
    if len(values) != 7:
        raise RuntimeError(
            f"{group_name} arm handoff requires 7 joint positions, got {len(values)}")
    return values


def _configure_impedance_defaults():
    """Load joint-space impedance gains and initialize targets from q."""
    global impedance_targets, impedance_kp, impedance_kd, impedance_torque_limits

    params = config.get('impedance', {})
    default_kp = [4.0, 8.0, 8.0, 3.0, 2.0, 1.0, 0.8, 0.0]
    default_kd = [0.6, 0.8, 0.8, 0.4, 0.25, 0.15, 0.1, 0.0]
    default_limit = [15.0, 25.0, 25.0, 15.0, 6.0, 6.0, 4.0, 2.0]
    kp_values = params.get('kp', default_kp)
    kd_values = params.get('kd', default_kd)
    limit_values = params.get('torque_limit', default_limit)

    def value_at(values, index, fallback):
        if isinstance(values, (int, float)):
            return float(values)
        if index < len(values):
            return float(values[index])
        return float(fallback)

    with state_lock:
        current = positions.copy()
    with impedance_lock:
        impedance_targets[:] = current
        impedance_kp[:] = 0.0
        impedance_kd[:] = 0.0
        impedance_torque_limits[:] = 0.0

        for group in config.get('groups', {}).values():
            for local_index, gid in enumerate(group.get('motor_indices', [])):
                index = gid - 1
                if index < 0 or index >= MOTOR_COUNT:
                    continue
                impedance_kp[index] = value_at(
                    kp_values, local_index, default_kp[-1])
                impedance_kd[index] = value_at(
                    kd_values, local_index, default_kd[-1])
                impedance_torque_limits[index] = value_at(
                    limit_values, local_index, default_limit[-1])


def _set_impedance_targets_from_positions():
    """Make entering impedance mode bumpless by holding the current pose."""
    with state_lock:
        current = positions.copy()
    with impedance_lock:
        impedance_targets[:] = current


def _gravity_runner_path():
    return SCRIPT_RUNNER


def _is_gravity_runner_alive():
    return gravity_process is not None and gravity_process.poll() is None


def _wait_process_exit(proc, timeout=4.0):
    """Wait for a child process without sending another control signal."""
    if proc is None:
        return True
    try:
        proc.wait(timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        return False
    except Exception:
        return proc.poll() is not None


def _terminate_process_group(proc, timeout=4.0):
    """Gracefully stop a mode process, escalating only if it is stuck."""
    if proc is None:
        return
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass

    if _wait_process_exit(proc, timeout):
        return

    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass
    _wait_process_exit(proc, timeout=0.5)


def _request_gravity_handoff(proc):
    """Ask Gravity to exit without STOP, then wait before port transfer."""
    if proc is None or proc.poll() is not None:
        return True
    try:
        os.killpg(proc.pid, signal.SIGUSR1)
    except Exception as exc:
        print(f"[Impedance] Gravity handoff signal failed: {exc}")
        return False

    # The old code waited 50 ms and then sent SIGINT through
    # _stop_external_gravity(). That could race the SIGUSR1 handler and make
    # Gravity execute robot.close(stop=True) during the handoff.
    if _wait_process_exit(proc, timeout=4.0):
        return True

    print("[Impedance] Gravity did not exit after handoff request; stopping it")
    _terminate_process_group(proc)
    return False


def _release_robot_for_external_gravity():
    global robot, gravity_serial_released
    global backend_serial_fault, backend_serial_fault_message
    with serial_io_lock:
        current_robot = robot
        if current_robot is None or demo_mode:
            if not demo_mode:
                gravity_serial_released = True
                backend_serial_fault = False
                backend_serial_fault_message = ""
            return
        # Do not send STOP here. STOP disables the motor torque before the
        # gravity script has opened the ports, which causes the arm to drop
        # when switching from position mode.
        try:
            current_robot.close_all()
        except Exception:
            pass
        # A closed manager must not remain visible to the broadcast/control
        # threads while the external Python script owns the serial ports.
        robot = None
        gravity_serial_released = True
        backend_serial_fault = False
        backend_serial_fault_message = ""


def _restore_robot_after_external_gravity():
    global robot, gravity_serial_released
    global backend_serial_fault, backend_serial_fault_message
    if demo_mode or not gravity_serial_released:
        return True
    if not active_config_path:
        gravity_serial_released = False
        return False
    with serial_io_lock:
        try:
            print("[Gravity] Re-opening backend serial connection")
            time.sleep(0.3)
            robot = init_robot(active_config_path)
            gravity_serial_released = False
            _read_states()
            backend_serial_fault = False
            backend_serial_fault_message = ""
            return True
        except Exception as exc:
            robot = None
            gravity_serial_released = False
            backend_serial_fault = True
            backend_serial_fault_message = str(exc)
            print(f"[Gravity] Failed to re-open backend serial connection: {exc}")
            return False


def _start_external_gravity():
    global gravity_process, control_mode, gravity_transitioning
    with mode_transition_lock:
        if demo_mode:
            return False, "demo mode does not use hardware gravity compensation"
        if _script_is_alive():
            return False, f"script already running: {script_name}"
        if _is_gravity_runner_alive():
            return True, None

        runner = _gravity_runner_path()
        if not os.path.exists(runner):
            return False, f"mode script runner not found: {runner}"
        if not active_config_path:
            return False, "active robot config is not available"

        gravity_transitioning = True

        try:
            current = _snapshot_live_positions()
            _hold_current_positions(current)
            _send_position_hold_before_handoff(current)
            control_mode = 'gravity_comp'
            _release_robot_for_external_gravity()
            gravity_process = subprocess.Popen(
                [
                    sys.executable,
                    runner,
                    '--config',
                    active_config_path,
                    '2_gravity_compensation_control.py',
                ],
                cwd=BACKEND_DIR,
                stdout=None,
                stderr=None,
                start_new_session=True,
            )
            time.sleep(0.3)
            if gravity_process.poll() is not None:
                code = gravity_process.returncode
                gravity_process = None
                restored = _restore_robot_after_external_gravity()
                control_mode = 'position' if restored else 'free'
                return False, f"gravity runner exited immediately with code {code}"
            print(
                "[Gravity] External Python script started, "
                f"pid={gravity_process.pid}, "
                "script=2_gravity_compensation_control.py"
            )
            return True, None
        except Exception as exc:
            gravity_process = None
            restored = _restore_robot_after_external_gravity()
            control_mode = 'position' if restored else 'free'
            return False, str(exc)
        finally:
            gravity_transitioning = False


def _stop_external_gravity(restore_robot=True):
    global gravity_process, gravity_transitioning, control_mode
    with mode_transition_lock:
        owns_transition = not gravity_transitioning
        if owns_transition:
            gravity_transitioning = True
        restored = True
        try:
            proc = gravity_process
            gravity_process = None
            _terminate_process_group(proc)
            if restore_robot:
                restored = _restore_robot_after_external_gravity()
            # Keep the controller in a neutral state until the caller has
            # explicitly selected the next mode. This prevents the control
            # loop from treating the stopping process as an unexpected exit.
            if restore_robot:
                control_mode = 'free'
            return restored
        finally:
            if owns_transition:
                gravity_transitioning = False


def _impedance_runner_path():
    return SCRIPT_RUNNER


def _is_impedance_runner_alive():
    return impedance_process is not None and impedance_process.poll() is None


def _release_robot_for_external_impedance():
    global robot, impedance_serial_released
    global backend_serial_fault, backend_serial_fault_message
    with serial_io_lock:
        current_robot = robot
        if current_robot is None or demo_mode:
            if not demo_mode:
                impedance_serial_released = True
                backend_serial_fault = False
                backend_serial_fault_message = ""
            return
        # Keep the last position-hold command alive until the Python script
        # takes the port. Sending STOP here makes impedance entry unsafe.
        try:
            current_robot.close_all()
        except Exception:
            pass
        # Do not let backend threads use a closed serial manager while the
        # external MIT controller owns the ports.
        robot = None
        impedance_serial_released = True
        backend_serial_fault = False
        backend_serial_fault_message = ""


def _restore_robot_after_external_impedance():
    global robot, impedance_serial_released
    global backend_serial_fault, backend_serial_fault_message
    if demo_mode or not impedance_serial_released:
        return True
    if not active_config_path:
        impedance_serial_released = False
        return False
    with serial_io_lock:
        try:
            print("[Impedance] Re-opening backend serial connection")
            time.sleep(0.3)
            robot = init_robot(active_config_path)
            impedance_serial_released = False
            _read_states()
            backend_serial_fault = False
            backend_serial_fault_message = ""
            return True
        except Exception as exc:
            robot = None
            impedance_serial_released = False
            backend_serial_fault = True
            backend_serial_fault_message = str(exc)
            print(f"[Impedance] Failed to re-open backend serial connection: {exc}")
            return False


def _start_external_impedance(skip_backend_handoff=False):
    global impedance_process, control_mode, impedance_transitioning
    global impedance_serial_released, gravity_serial_released
    with mode_transition_lock:
        if demo_mode:
            return False, "demo mode does not use hardware impedance control"
        if _script_is_alive():
            return False, f"script already running: {script_name}"
        if _is_impedance_runner_alive():
            return True, None

        runner = _impedance_runner_path()
        if not os.path.exists(runner):
            return False, f"mode script runner not found: {runner}"
        if not active_config_path:
            return False, "active robot config is not available"

        owns_transition = not impedance_transitioning
        if owns_transition:
            impedance_transitioning = True
        try:
            left_target = None
            right_target = None
            if skip_backend_handoff:
                # Gravity Python has just released the ports. Keep ownership
                # on the mode-script side and let impedance latch q_target
                # directly from its first state read.
                control_mode = 'impedance'
                impedance_serial_released = True
            else:
                # Capture q_target before releasing the backend serial handle
                # and pass it explicitly to the Python script.
                current = _snapshot_live_positions()
                left_target = _arm_target_from_snapshot(current, 'left')
                right_target = _arm_target_from_snapshot(current, 'right')
                with impedance_lock:
                    impedance_targets[:] = current
                _hold_current_positions(current)
                _send_position_hold_before_handoff(current)
                control_mode = 'impedance'
                _release_robot_for_external_impedance()
            command = [
                sys.executable,
                runner,
                '--config',
                active_config_path,
                'dual_arm_impedance_compensation.py',
            ]
            if left_target is not None and right_target is not None:
                command.extend([
                    '--left-target', *[f'{value:.12g}' for value in left_target],
                    '--right-target', *[f'{value:.12g}' for value in right_target],
                ])
            impedance_process = subprocess.Popen(
                command,
                cwd=BACKEND_DIR,
                stdout=None,
                stderr=None,
                start_new_session=True,
            )
            time.sleep(0.3)
            if impedance_process.poll() is not None:
                code = impedance_process.returncode
                impedance_process = None
                if skip_backend_handoff:
                    impedance_serial_released = False
                    restored = _restore_robot_after_external_gravity()
                else:
                    restored = _restore_robot_after_external_impedance()
                control_mode = 'position' if restored else 'free'
                return False, f"impedance runner exited immediately with code {code}"
            if skip_backend_handoff:
                gravity_serial_released = False
            print(
                "[Impedance] External Python script started, "
                f"pid={impedance_process.pid}, "
                + ("target latched in Python after Gravity handoff"
                   if skip_backend_handoff else "target latched from backend")
                + ", script=dual_arm_impedance_compensation.py"
            )
            return True, None
        except Exception as exc:
            impedance_process = None
            if skip_backend_handoff:
                impedance_serial_released = False
                restored = _restore_robot_after_external_gravity()
            else:
                restored = _restore_robot_after_external_impedance()
            control_mode = 'position' if restored else 'free'
            return False, str(exc)
        finally:
            if owns_transition:
                impedance_transitioning = False


def _switch_gravity_to_impedance():
    """Hand serial ownership directly from Gravity Python to Impedance Python."""
    global control_mode, impedance_transitioning, gravity_transitioning
    global gravity_process
    with mode_transition_lock:
        if not gravity_serial_released:
            return _start_external_impedance()

        impedance_transitioning = True
        gravity_transitioning = True
        try:
            print("[Impedance] Direct Gravity -> Impedance handoff")
            # Tell the Gravity script not to send STOP in its finally block.
            # The last gravity MIT command stays active until the Impedance
            # script opens the ports and sends its first torque command.
            proc = gravity_process
            handoff_ok = _request_gravity_handoff(proc)
            gravity_process = None
            if not handoff_ok:
                _restore_robot_after_external_gravity()
                control_mode = 'free'
                return False, "gravity handoff failed before impedance start"
            return _start_external_impedance(skip_backend_handoff=True)
        finally:
            gravity_transitioning = False
            impedance_transitioning = False


def _stop_external_impedance(restore_robot=True):
    global impedance_process, impedance_transitioning, control_mode
    with mode_transition_lock:
        owns_transition = not impedance_transitioning
        if owns_transition:
            impedance_transitioning = True
        restored = True
        try:
            proc = impedance_process
            impedance_process = None
            _terminate_process_group(proc)
            if restore_robot:
                restored = _restore_robot_after_external_impedance()
            if restore_robot:
                control_mode = 'free'
            return restored
        finally:
            if owns_transition:
                impedance_transitioning = False


def _script_is_alive():
    return script_process is not None and script_process.poll() is None


def _append_script_output(text):
    if text is None:
        return
    lines = str(text).splitlines()
    if not lines:
        return
    with script_output_lock:
        script_output_lines.extend(lines)
        del script_output_lines[:-SCRIPT_OUTPUT_LIMIT]


def _clear_script_output():
    with script_output_lock:
        script_output_lines.clear()


def _get_script_output():
    with script_output_lock:
        return list(script_output_lines)


def _discover_scripts():
    if not os.path.isdir(SCRIPT_DIR):
        return []
    scripts = []
    for filename in sorted(os.listdir(SCRIPT_DIR)):
        if not filename.endswith('.py') or filename.startswith('__'):
            continue
        if filename in ('litearm_demo_common.py', 'litearm_control.py'):
            continue
        full_path = os.path.join(SCRIPT_DIR, filename)
        if not os.path.isfile(full_path):
            continue
        scripts.append({
            'name': filename[:-3],
            'file': filename,
            'label': filename[:-3].replace('_', ' '),
        })
    return scripts


def _resolve_script_path(script):
    name = (script or '').replace('\\', '/').lstrip('/')
    if not name.endswith('.py'):
        name += '.py'
    path = os.path.abspath(os.path.normpath(os.path.join(SCRIPT_DIR, name)))
    root = os.path.abspath(SCRIPT_DIR)
    if os.path.commonpath([root, path]) != root:
        return None, name
    return path, name


def _script_needs_execute(filename):
    # LiteArm scripts expose their own arguments. Keep this hook for API
    # compatibility, but do not append Panthera's --execute flag.
    return False


def _watch_script_process(proc, name):
    global script_process, script_name, control_mode
    try:
        for line in iter(proc.stdout.readline, ''):
            _append_script_output(line)
        proc.stdout.close()
        return_code = proc.wait()
        _append_script_output(f"[ScriptRunner] exited with code {return_code}")
    except Exception as exc:
        _append_script_output(f"[ScriptRunner] watcher error: {exc}")
    finally:
            if script_process is proc:
                script_process = None
                script_name = None
                if gravity_serial_released and not _is_gravity_runner_alive():
                    restored = _restore_robot_after_external_gravity()
                    control_mode = 'position' if restored else 'free'
            socketio.emit('script_status', {
                'running': False,
                'current_script': None,
            })


def _start_script(script):
    global script_process, script_name, control_mode
    if demo_mode:
        return False, 'demo mode does not run hardware scripts'
    if _script_is_alive():
        return False, f'script already running: {script_name}'
    if _is_gravity_runner_alive():
        return False, 'gravity compensation is running; stop it first'
    if _is_impedance_runner_alive():
        return False, 'impedance control is running; stop it first'

    script_path, script_filename = _resolve_script_path(script)
    if not script_path or not os.path.isfile(script_path):
        return False, f'script not found: {script_filename}'
    if not active_config_path:
        return False, 'active robot config is not available'

    _clear_script_output()
    _append_script_output(f"[ScriptRunner] starting {script_filename}")
    try:
        _hold_current_positions()
        control_mode = 'script'
        _release_robot_for_external_gravity()

        cmd = [
            sys.executable,
            SCRIPT_RUNNER,
            '--config',
            active_config_path,
            script_filename,
        ]
        env = os.environ.copy()
        env['PYTHONUNBUFFERED'] = '1'
        env['PYTHONPATH'] = TEACH_DIR + os.pathsep + env.get('PYTHONPATH', '')
        script_process = subprocess.Popen(
            cmd,
            cwd=BACKEND_DIR,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        script_name = script_filename
        threading.Thread(
            target=_watch_script_process,
            args=(script_process, script_filename),
            daemon=True,
        ).start()
        time.sleep(0.2)
        if script_process.poll() is not None:
            return_code = script_process.returncode
            script_process = None
            script_name = None
            restored = _restore_robot_after_external_gravity()
            control_mode = 'position' if restored else 'free'
            return False, f'script exited immediately with code {return_code}'
        return True, None
    except Exception as exc:
        script_process = None
        script_name = None
        restored = _restore_robot_after_external_gravity()
        control_mode = 'position' if restored else 'free'
        return False, str(exc)


def _stop_script(restore_robot=True):
    global script_process, script_name, control_mode
    restored = True
    proc = script_process
    script_process = None
    script_name = None
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass
        try:
            proc.wait(timeout=4.0)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except Exception:
                proc.kill()
    if restore_robot:
        restored = _restore_robot_after_external_gravity()
        control_mode = 'position' if restored else 'free'
    return restored


def control_loop():
    """Send position or joint-space impedance commands at control rate."""
    global running, targets, control_mode
    loop_hz = config.get('control', {}).get('loop_hz', 100)
    gravity_loop_hz = config.get('control', {}).get('gravity_loop_hz', 200)
    impedance_loop_hz = config.get('control', {}).get('impedance_loop_hz', 200)
    print(f"[Control] Starting at {loop_hz} Hz "
          f"(gravity compensation: {gravity_loop_hz} Hz, "
          f"impedance: {impedance_loop_hz} Hz)")

    while running:
        t0 = time.time()
        try:
            if control_mode == 'free':
                pass

            elif control_mode == 'gravity_comp':
                # The external Python mode script owns the serial ports.
                if (gravity_serial_released and not gravity_transitioning
                        and not impedance_transitioning
                        and not _is_gravity_runner_alive()
                        and not _script_is_alive()):
                    print("[Gravity] External script stopped unexpectedly; restoring backend serial")
                    restored = _stop_external_gravity(restore_robot=True)
                    control_mode = 'free'
                pass

            elif control_mode == 'script':
                # External demo script owns the serial ports.
                pass

            elif control_mode == 'impedance':
                # The Python impedance script owns both serial ports.
                if (impedance_serial_released and not impedance_transitioning
                        and not _is_impedance_runner_alive()):
                    print("[Impedance] External script stopped unexpectedly; restoring backend serial")
                    restored = _stop_external_impedance(restore_robot=True)
                    control_mode = 'free'

            elif control_mode == 'position':
                # Check the mode and send under the same transition lock.
                # Otherwise a stale position command can race a mode handoff
                # between the condition check and the serial write.
                with mode_transition_lock:
                    if _backend_position_commands_allowed():
                        cmd = {}
                        default_vel = float(
                            config.get('control', {}).get(
                                'default_velocity', 0.5))
                        max_torque = float(
                            config.get('robot', {}).get(
                                'max_torque', 15.0))
                        with target_lock:
                            for gid, tgt in targets.items():
                                cmd[gid] = (
                                    float(tgt), default_vel, max_torque)
                        if cmd:
                            with serial_io_lock:
                                current_robot = robot
                                if current_robot is not None:
                                    try:
                                        current_robot.set_all_pos_vel_max_torque(
                                            cmd)
                                    except Exception as exc:
                                        _drop_backend_robot(
                                            f"position command failed: {exc}",
                                            current_robot)
        except Exception as e:
            print(f"[Control] Error: {e}")

        if control_mode == 'gravity_comp':
            active_hz = gravity_loop_hz
        elif control_mode == 'impedance':
            active_hz = impedance_loop_hz
        else:
            active_hz = loop_hz
        interval = 1.0 / max(float(active_hz), 1.0)
        elapsed = time.time() - t0
        if elapsed < interval:
            time.sleep(interval - elapsed)


# ═══════════════════════════════════════════════════════════════
# Broadcast Loop (30 Hz)
# ═══════════════════════════════════════════════════════════════

def state_broadcast_loop():
    """Read motor states and broadcast via WebSocket."""
    global positions, velocities, torques, connected
    broadcast_hz = config.get('control', {}).get('broadcast_hz', 30)
    interval = 1.0 / broadcast_hz
    print(f"[Broadcast] Starting at {broadcast_hz} Hz")

    # FK engine — only use revolute arm joints (exclude gripper)
    fk = None
    if config:
        urdf_path = config.get('urdf', {}).get('file_path', '')
        left_group = config['groups'].get('left', {})
        right_group = config['groups'].get('right', {})
        left_all = left_group.get('joint_names', [])
        right_all = right_group.get('joint_names', [])
        # Filter out gripper joints for FK
        left_arm_joints = [n for n in left_all if 'gripper' not in n.lower()]
        right_arm_joints = [n for n in right_all if 'gripper' not in n.lower()]
        left_ee = left_group.get('end_effector_link', '')
        right_ee = right_group.get('end_effector_link', '')
        if urdf_path and left_arm_joints and right_arm_joints:
            fk = FKEngine(urdf_path, left_arm_joints, right_arm_joints, left_ee, right_ee)

    while running:
        t0 = time.time()
        try:
            external_serial_owner = (
                gravity_serial_released or impedance_serial_released)
            if robot is not None or external_serial_owner:
                if robot is not None and control_mode not in (
                        'gravity_comp', 'impedance', 'script'):
                    _read_states()

                # Build state dict — per-group slices from config
                state = {
                    'positions': positions.tolist(),
                    'velocities': velocities.tolist(),
                    'torques': torques.tolist(),
                    'target_positions': (
                        impedance_targets.tolist()
                        if control_mode == 'impedance'
                        else [float(targets.get(i+1, 0)) for i in range(MOTOR_COUNT)]
                    ),
                    'impedance_target': impedance_targets.tolist(),
                    'impedance_kp': impedance_kp.tolist(),
                    'impedance_kd': impedance_kd.tolist(),
                    'control_mode': control_mode,
                    'connected': connected,
                    'backend_serial_fault': backend_serial_fault,
                    'backend_serial_fault_message': backend_serial_fault_message,
                    'timestamp': time.time(),
                }

                # ── Left arm: FK + group state ──
                left_group = config.get('groups', {}).get('left')
                if left_group:
                    left_ids = left_group['motor_indices']  # e.g. [1..8]
                    left_start = left_ids[0] - 1
                    left_end = left_ids[-1]
                    left_all = positions[left_start:left_end].tolist()
                    # FK uses arm-only joints (exclude gripper)
                    left_arm_joints_fk = [n for n in left_group.get('joint_names', []) if 'gripper' not in n.lower()]
                    left_fk_pos = left_all[:len(left_arm_joints_fk)]
                    left_fk = fk.compute(left_fk_pos, 'left') if fk else None
                    state['left'] = {'positions': left_all, 'fk': left_fk}
                    # Primary FK = left arm (Panthera-compatible)
                    state['forward_kinematics'] = left_fk
                    state['ee_position'] = left_fk['position'] if left_fk else [0, 0, 0]
                    state['ee_euler'] = left_fk['euler'] if left_fk else [0, 0, 0]
                    state['gripper_position'] = float(left_all[-1]) if len(left_all) == len(left_ids) else 0.0
                    state['left_gripper_position'] = state['gripper_position']
                else:
                    state['forward_kinematics'] = None
                    state['ee_position'] = [0, 0, 0]
                    state['ee_euler'] = [0, 0, 0]
                    state['gripper_position'] = 0.0
                    state['left_gripper_position'] = 0.0
                state['external_wrench'] = [0, 0, 0, 0, 0, 0]

                # ── Right arm: FK + group state ──
                right_group = config.get('groups', {}).get('right')
                if right_group:
                    right_ids = right_group['motor_indices']
                    right_start = right_ids[0] - 1
                    right_end = right_ids[-1]
                    right_all = positions[right_start:right_end].tolist()
                    right_arm_joints_fk = [n for n in right_group.get('joint_names', []) if 'gripper' not in n.lower()]
                    right_fk_pos = right_all[:len(right_arm_joints_fk)]
                    right_fk = fk.compute(right_fk_pos, 'right') if fk else None
                    state['right'] = {'positions': right_all, 'fk': right_fk}
                    state['right_gripper_position'] = float(right_all[-1]) if len(right_all) == len(right_ids) else 0.0

                # ── Optional groups (waist, head) ──
                for gname in ['waist', 'head']:
                    g = config.get('groups', {}).get(gname)
                    if g:
                        g_start = g['motor_indices'][0] - 1
                        g_end = g['motor_indices'][-1]
                        state[gname] = positions[g_start:g_end].tolist()

                socketio.emit('robot_state', state)
                connected = True

        except Exception as e:
            print(f"[Broadcast] Error: {e}")

        elapsed = time.time() - t0
        if elapsed < interval:
            time.sleep(interval - elapsed)


# ═══════════════════════════════════════════════════════════════
# REST API
# ═══════════════════════════════════════════════════════════════

@app.route('/')
def serve_index():
    return send_from_directory(app.static_folder, 'index.html')


@app.route('/api/config')
def api_config():
    joints = build_joint_list(config)
    groups = {}
    for gname, ginfo in config.get('groups', {}).items():
        groups[gname] = {
            'name': ginfo.get('name', gname),
            'joint_count': len(ginfo['joint_names']),
            'motor_indices': ginfo['motor_indices'],
            'joint_limits': ginfo.get('joint_limits', {}),
        }
    return jsonify({
        'robot_name': config.get('robot', {}).get('name', 'LiteArm-A10'),
        'demo_mode': demo_mode,
        'connected': connected,
        'backend_serial_fault': backend_serial_fault,
        'backend_serial_fault_message': backend_serial_fault_message,
        'control_mode': control_mode,
        'joints': joints,
        'groups': groups,
        'control': config.get('control', {}),
        'impedance': {
            'target': impedance_targets.tolist(),
            'kp': impedance_kp.tolist(),
            'kd': impedance_kd.tolist(),
            'torque_limit': impedance_torque_limits.tolist(),
        },
        'end_effector_link': config.get('groups', {}).get('left', {}).get('end_effector_link', ''),
        'end_effector_offset': 0.07,
    })


@app.route('/api/status')
def api_status():
    with state_lock:
        return jsonify({
            'positions': positions.tolist(),
            'velocities': velocities.tolist(),
            'torques': torques.tolist(),
            'control_mode': control_mode,
            'connected': connected,
            'backend_serial_fault': backend_serial_fault,
            'backend_serial_fault_message': backend_serial_fault_message,
            'timestamp': time.time(),
        })


@app.route('/api/move', methods=['POST'])
def api_move():
    """Move one or more groups. Body: {groups: {left: [...], right: [...]}, velocity: 0.5}"""
    data = flask_request.get_json()
    if not data:
        return jsonify({'error': 'No JSON body'}), 400
    if not _position_target_updates_allowed():
        return jsonify({
            'error': f'position target update rejected in {control_mode} mode'
        }), 409

    velocity = float(data.get('velocity', config.get('control', {}).get('default_velocity', 0.5)))
    max_torque = float(data.get('max_torque', config.get('robot', {}).get('max_torque', 15.0)))

    groups_data = data.get('groups', {})
    if not groups_data:
        # Flat positions array for all motors
        updates = build_target_updates_from_flat_payload(
            data.get('positions', None),
            data.get('gripper', None))
        if updates:
            with target_lock:
                targets.update(updates)
        return jsonify({'ok': True, 'updated_count': len(updates)})

    # Group-based update
    with target_lock:
        for gname, gvals in groups_data.items():
            ginfo = config.get('groups', {}).get(gname)
            if ginfo is None:
                continue
            indices = ginfo['motor_indices']
            limits_lo = ginfo.get('joint_limits', {}).get('lower', [-5]*len(indices))
            limits_hi = ginfo.get('joint_limits', {}).get('upper', [5]*len(indices))

            if isinstance(gvals, list):
                for i, pos in enumerate(gvals):
                    if i >= len(indices):
                        break
                    lo = limits_lo[i] if i < len(limits_lo) else -5
                    hi = limits_hi[i] if i < len(limits_hi) else 5
                    targets[indices[i]] = max(lo, min(hi, float(pos)))
            else:
                # Single value for gripper
                gripper_index = int(ginfo.get(
                    'gripper_index',
                    len(indices) - 1 if indices else 0))
                gripper_index = max(0, min(gripper_index, len(indices) - 1))
                lo = limits_lo[gripper_index] if gripper_index < len(limits_lo) else 0
                hi = limits_hi[gripper_index] if gripper_index < len(limits_hi) else 5
                targets[indices[gripper_index]] = max(lo, min(hi, float(gvals)))

    return jsonify({'ok': True, 'targets_count': len(targets)})


@app.route('/api/home', methods=['POST'])
def api_home():
    """Home all joints to zero (smooth)."""
    if not _position_target_updates_allowed():
        return jsonify({
            'error': f'home rejected in {control_mode} mode'
        }), 409
    group = flask_request.get_json().get('group', None) if flask_request.is_json else None
    velocity = 0.3

    with target_lock:
        if group:
            ginfo = config.get('groups', {}).get(group)
            if ginfo:
                for idx in ginfo['motor_indices']:
                    targets[idx] = 0.0
        else:
            for i in range(1, MOTOR_COUNT + 1):
                targets[i] = 0.0
    return jsonify({'ok': True})


@app.route('/api/stop', methods=['POST'])
def api_stop():
    """Stop all motors (send stop command)."""
    global control_mode
    restored = True
    if _script_is_alive():
        restored = _stop_script(restore_robot=True) and restored
    if _is_gravity_runner_alive():
        restored = _stop_external_gravity(restore_robot=True) and restored
    if _is_impedance_runner_alive():
        restored = _stop_external_impedance(restore_robot=True) and restored
    control_mode = 'position' if restored else 'free'
    with serial_io_lock:
        current_robot = robot
        if current_robot is not None:
            try:
                current_robot.stop_all()
            except Exception as exc:
                _drop_backend_robot(f"stop command failed: {exc}", current_robot)
                control_mode = 'free'
    with target_lock:
        # Set targets to current positions
        with state_lock:
            for i in range(MOTOR_COUNT):
                targets[i+1] = float(positions[i])
    return jsonify({'ok': True})


@app.route('/api/set_mode', methods=['POST'])
def api_set_mode():
    global control_mode
    data = flask_request.get_json()
    mode = data.get('mode', 'position')
    if mode not in ('position', 'free', 'gravity_comp', 'impedance'):
        return jsonify({'error': f'Unknown mode: {mode}'}), 400
    if control_mode == 'script' and mode != 'script':
        _stop_script(restore_robot=True)
    if mode == 'gravity_comp':
        if control_mode == 'impedance':
            restored = _stop_external_impedance(restore_robot=True)
            if not restored:
                return jsonify({'error': 'failed to restore backend serial before gravity mode'}), 503
        ok, error = _start_external_gravity()
        if not ok:
            return jsonify({'error': error}), 503
    elif mode == 'impedance':
        if control_mode == 'gravity_comp':
            ok, error = _switch_gravity_to_impedance()
        else:
            ok, error = _start_external_impedance()
        if not ok:
            return jsonify({'error': error}), 503
    if control_mode == 'gravity_comp' and mode not in ('gravity_comp', 'impedance'):
        restored = _stop_external_gravity(restore_robot=True)
        if restored:
            _hold_current_positions()
        else:
            mode = 'free'
    if control_mode == 'impedance' and mode != 'impedance':
        restored = _stop_external_impedance(restore_robot=True)
        if restored:
            _hold_current_positions()
        else:
            mode = 'free'
    control_mode = mode
    if mode == 'free':
        with serial_io_lock:
            current_robot = robot
            if current_robot is not None:
                try:
                    current_robot.set_all_free_mode()
                except Exception as exc:
                    _drop_backend_robot(
                        f"free mode command failed: {exc}",
                        current_robot)
    socketio.emit('mode_changed', {'mode': mode})
    print(f"[Mode] → {mode}")
    return jsonify({'ok': True, 'mode': mode})


@app.route('/api/get_mode')
def api_get_mode():
    return jsonify({'mode': control_mode})


# ── URDF / Mesh file serving ──────────────────────────────────
ARM_DESC_DIR = os.path.normpath(os.path.join(
    BACKEND_DIR, '..', 'src', 'litearm_a10_251125'))


@app.route('/arm_description/<path:filepath>')
def serve_arm_file(filepath):
    """Serve URDF and mesh files for the 3D viewer."""
    return send_from_directory(ARM_DESC_DIR, filepath)


@app.route('/api/urdf_path')
def api_urdf_path():
    """Return the URDF file path for the frontend to load."""
    return jsonify({
        'urdf_url': '/arm_description/urdf/LiteArm_A10_251125.urdf',
        'mesh_package': 'package://litearm_a10_251125',
        'mesh_prefix': '/arm_description',
    })


@app.route('/api/arm_description_files')
def api_arm_description_files():
    """Return all URDF and mesh files for the Panthera-compatible frontend."""
    files = {}
    arm_dir = ARM_DESC_DIR
    for root, dirs, filenames in os.walk(arm_dir):
        for fn in filenames:
            full = os.path.join(root, fn)
            rel = os.path.relpath(full, arm_dir)
            files[rel] = f'/arm_description/{rel}'
    return jsonify({
        'success': True,
        'files': files,
        'base_url': '/arm_description',
        'robot_name': 'LiteArm_A10_251125',
    })


@app.route('/api/scripts')
def api_scripts():
    return jsonify({
        'scripts': _discover_scripts(),
        'scripts_count': len(_discover_scripts()),
        'scripts_dir': os.path.abspath(SCRIPT_DIR),
        'running': _script_is_alive(),
        'current_script': script_name,
    })


@app.route('/api/scripts/log')
def api_scripts_log():
    return '\n'.join(_get_script_output()) + '\n'


@app.route('/api/scripts/output')
def api_scripts_output():
    return jsonify({
        'running': _script_is_alive(),
        'current_script': script_name,
        'output': _get_script_output(),
    })


@app.route('/api/scripts/run', methods=['POST'])
def api_scripts_run():
    data = flask_request.get_json(silent=True) or {}
    ok, error = _start_script(data.get('script', ''))
    if not ok:
        return jsonify({'success': False, 'error': error}), 409
    socketio.emit('script_status', {
        'running': True,
        'current_script': script_name,
    })
    return jsonify({
        'success': True,
        'script': script_name,
        'pid': script_process.pid if script_process else None,
    })


@app.route('/api/scripts/stop', methods=['POST'])
def api_scripts_stop():
    _stop_script(restore_robot=True)
    socketio.emit('script_status', {
        'running': False,
        'current_script': None,
    })
    return jsonify({'success': True, 'status': 'stopped'})


# ═══════════════════════════════════════════════════════════════
# WebSocket Events
# ═══════════════════════════════════════════════════════════════

@socketio.on('connect')
def on_connect():
    print(f"Client connected (total: {len(socketio.server.manager.rooms)})")
    emit('config', {
        'robot_name': config.get('robot', {}).get('name', 'LiteArm-A10'),
        'demo_mode': demo_mode,
        'control_mode': control_mode,
        'connected': connected,
        'backend_serial_fault': backend_serial_fault,
        'backend_serial_fault_message': backend_serial_fault_message,
        'joints': build_joint_list(config),
        'groups': {g: {'name': gi.get('name', g),
                       'joint_count': len(gi['joint_names']),
                       'motor_indices': gi['motor_indices']}
                   for g, gi in config.get('groups', {}).items()},
        'control': config.get('control', {}),
        'impedance': {
            'target': impedance_targets.tolist(),
            'kp': impedance_kp.tolist(),
            'kd': impedance_kd.tolist(),
            'torque_limit': impedance_torque_limits.tolist(),
        },
    })


@socketio.on('disconnect')
def on_disconnect():
    print("Client disconnected")


@socketio.on('move_all')
def ws_move_all(data):
    if not _position_target_updates_allowed():
        print(f"[Move] ignored move_all in {control_mode} mode")
        socketio.emit('backend_error', {
            'message': f'位置目标更新已拒绝: 当前是 {control_mode} 模式'
        })
        return
    if isinstance(data, dict):
        updates = build_target_updates_from_flat_payload(
            data.get('positions', None),
            data.get('gripper', None))
        if not updates:
            return
        with target_lock:
            targets.update(updates)


@socketio.on('move_group')
def ws_move_group(data):
    if not _position_target_updates_allowed():
        print(f"[Move] ignored move_group in {control_mode} mode")
        socketio.emit('backend_error', {
            'message': f'位置目标更新已拒绝: 当前是 {control_mode} 模式'
        })
        return
    gname = data.get('group')
    positions_list = data.get('positions', [])
    ginfo = config.get('groups', {}).get(gname)
    if ginfo is None:
        return
    indices = ginfo['motor_indices']
    lo = ginfo.get('joint_limits', {}).get('lower', [-5]*len(indices))
    hi = ginfo.get('joint_limits', {}).get('upper', [5]*len(indices))
    with target_lock:
        for i, pos in enumerate(positions_list):
            if i >= len(indices):
                break
            l = lo[i] if i < len(lo) else -5
            h = hi[i] if i < len(hi) else 5
            targets[indices[i]] = max(l, min(h, float(pos)))


@socketio.on('home')
def ws_home(data=None):
    if not _position_target_updates_allowed():
        print(f"[Home] ignored in {control_mode} mode")
        socketio.emit('backend_error', {
            'message': f'回零已拒绝: 当前是 {control_mode} 模式'
        })
        return
    with target_lock:
        for i in range(1, MOTOR_COUNT + 1):
            targets[i] = 0.0


@socketio.on('reset_all')
def ws_reset_all(data=None):
    """Reset all joints to zero position."""
    if not _position_target_updates_allowed():
        print(f"[Reset] ignored in {control_mode} mode")
        socketio.emit('backend_error', {
            'message': f'复位已拒绝: 当前是 {control_mode} 模式'
        })
        return
    with target_lock:
        for i in range(1, MOTOR_COUNT + 1):
            targets[i] = 0.0
    print("[Reset] All targets → 0")


@socketio.on('stop')
def ws_stop(data=None):
    global control_mode
    restored = True
    if _script_is_alive():
        restored = _stop_script(restore_robot=True) and restored
    if _is_gravity_runner_alive():
        restored = _stop_external_gravity(restore_robot=True) and restored
    if _is_impedance_runner_alive():
        restored = _stop_external_impedance(restore_robot=True) and restored
    control_mode = 'position' if restored else 'free'
    with serial_io_lock:
        current_robot = robot
        if current_robot is not None:
            try:
                current_robot.stop_all()
            except Exception as exc:
                _drop_backend_robot(f"stop command failed: {exc}", current_robot)
                control_mode = 'free'


@socketio.on('gravity_comp')
def ws_gravity_comp(data=None):
    """Toggle gravity compensation mode."""
    global control_mode
    enable = data.get('enable', True) if isinstance(data, dict) else True
    if enable:
        if control_mode == 'impedance':
            restored = _stop_external_impedance(restore_robot=True)
            if not restored:
                print("[Gravity] Compensation rejected: failed to restore backend serial before gravity mode")
                socketio.emit('backend_error', {
                    'message': '重力补偿启动失败: 后端串口恢复失败'
                })
                return
        ok, error = _start_external_gravity()
        if not ok:
            print(f"[Gravity] Compensation rejected: {error}")
            socketio.emit('backend_error', {
                'message': f'重力补偿启动失败: {error}'
            })
            return
        control_mode = 'gravity_comp'
        print("[Gravity] Compensation ENABLED via Python script")
    else:
        restored = True
        if control_mode == 'gravity_comp':
            restored = _stop_external_gravity(restore_robot=True)
            if restored:
                _hold_current_positions()
        elif control_mode == 'impedance':
            restored = _stop_external_impedance(restore_robot=True)
            if restored:
                _hold_current_positions()
        control_mode = 'position' if restored else 'free'
        print("[Gravity] Compensation DISABLED → position mode")
    socketio.emit('mode_changed', {'mode': control_mode})


@socketio.on('set_mode')
def ws_set_mode(data):
    global control_mode
    mode = data.get('mode', 'position')
    if mode in ('position', 'free', 'gravity_comp', 'impedance'):
        if control_mode == 'script':
            _stop_script(restore_robot=True)
        if mode == 'gravity_comp':
            if control_mode == 'impedance':
                restored = _stop_external_impedance(restore_robot=True)
                if not restored:
                    print("[Gravity] Mode change rejected: failed to restore backend serial before gravity mode")
                    socketio.emit('backend_error', {
                        'message': '重力补偿启动失败: 后端串口恢复失败'
                    })
                    return
            ok, error = _start_external_gravity()
            if not ok:
                print(f"[Gravity] Mode change rejected: {error}")
                socketio.emit('backend_error', {
                    'message': f'重力补偿启动失败: {error}'
                })
                return
        elif mode == 'impedance':
            if control_mode == 'gravity_comp':
                ok, error = _switch_gravity_to_impedance()
            else:
                ok, error = _start_external_impedance()
            if not ok:
                print(f"[Impedance] Mode change rejected: {error}")
                socketio.emit('backend_error', {
                    'message': f'阻抗模式启动失败: {error}'
                })
                return
        if control_mode == 'gravity_comp' and mode not in ('gravity_comp', 'impedance'):
            restored = _stop_external_gravity(restore_robot=True)
            if restored:
                _hold_current_positions()
            else:
                mode = 'free'
        if control_mode == 'impedance' and mode != 'impedance':
            restored = _stop_external_impedance(restore_robot=True)
            if restored:
                _hold_current_positions()
            else:
                mode = 'free'
        control_mode = mode
        if mode == 'gravity_comp':
            print("[Gravity] Compensation ENABLED via Python script")
        elif mode == 'impedance':
            print("[Impedance] Joint-space G(q)+Kp(qd-q)-Kd*dq ENABLED")
        elif mode == 'free':
            with serial_io_lock:
                current_robot = robot
                if current_robot is not None:
                    try:
                        current_robot.set_all_free_mode()
                    except Exception as exc:
                        _drop_backend_robot(
                            f"free mode command failed: {exc}",
                            current_robot)
        socketio.emit('mode_changed', {'mode': mode})


# ═══════════════════════════════════════════════════════════════
# Initialization
# ═══════════════════════════════════════════════════════════════

def init_robot(cfg_path):
    """Initialize MultiMotorManager for live mode."""
    global config, targets, connected, positions, velocities, torques
    global backend_serial_fault, backend_serial_fault_message

    print("\n" + "=" * 50)
    print("LiteArm A10 Digital Twin Backend")
    print("=" * 50 + "\n")

    # Load config
    print(f"1. Loading config: {cfg_path}")
    cfg = load_config(cfg_path)
    cfg = resolve_serial_ports(cfg)
    config.update(cfg)
    print(f"   Robot: {cfg['robot']['name']}")
    print(f"   Groups: {list(cfg['groups'].keys())}")

    # Build port → motor map
    port_map = {}
    port_motor_types = build_port_motor_types(cfg)
    for port, local_ids in cfg['ports'].items():
        port_map[port] = local_ids
        print(f"   {port} → {len(local_ids)} motors, types="
              f"{[port_motor_types.get(port, {}).get(i, 'NONE') for i in local_ids]}")

    total_motors = sum(len(v) for v in port_map.values())
    print(f"\n2. Initializing {total_motors} motors on {len(port_map)} ports...")

    mgr = MultiMotorManager(port_map, port_motor_types)
    mgr.open_all()
    mgr.init_all()
    print(f"   Connected! Global IDs: {mgr.global_ids}")

    # Read initial state
    mgr.request_all_states()
    time.sleep(0.1)
    states = mgr.get_all_states()
    with state_lock:
        for gid in range(1, MOTOR_COUNT + 1):
            st = states.get(gid)
            if st and abs(st.pos) < 100:
                positions[gid-1] = st.pos
                velocities[gid-1] = st.vel
                torques[gid-1] = st.torque

    # Initialize targets to current positions
    with target_lock:
        for i in range(1, MOTOR_COUNT + 1):
            targets[i] = float(positions[i-1])
    _configure_impedance_defaults()

    connected = True
    backend_serial_fault = False
    backend_serial_fault_message = ""
    print(f"   Initialized. Positions sample: "
          f"{[f'{positions[i]:.3f}' for i in range(7)]}")

    return mgr


def init_demo():
    """Initialize demo (simulated) mode."""
    global config, targets, connected
    global backend_serial_fault, backend_serial_fault_message
    print("\n" + "=" * 50)
    print("LiteArm A10 Digital Twin Backend [DEMO MODE]")
    print("=" * 50 + "\n")
    print("No hardware — generating simulated motor states")

    connected = False
    backend_serial_fault = False
    backend_serial_fault_message = ""
    with target_lock:
        for i in range(1, MOTOR_COUNT + 1):
            targets[i] = 0.0
    _configure_impedance_defaults()
    return DemoRobot(MOTOR_COUNT)


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════

def main():
    global robot, config, demo_mode, running, active_config_path
    p = argparse.ArgumentParser(description='LiteArm A10 Backend')
    p.add_argument('--config', type=str,
                   default='robot_param/litearm_full.yaml',
                   help='Robot config YAML')
    p.add_argument('--demo', action='store_true', help='Demo mode (no hardware)')
    p.add_argument('--port', type=int, default=5001, help='Server port')
    args = p.parse_args()

    demo_mode = args.demo

    # Pre-load config so FK data is available in demo mode too
    cfg_path = os.path.join(BACKEND_DIR, args.config)
    if os.path.exists(cfg_path):
        config.update(load_config(cfg_path))
        active_config_path = cfg_path
    else:
        # Minimal fallback config
        config.update({
            'robot': {'name': 'LiteArm-A10', 'max_torque': 15.0},
            'control': {'loop_hz': 100, 'broadcast_hz': 30, 'default_velocity': 0.5},
            'groups': {},
            'ports': {},
        })

    # Compute motor count from config (max global_id across all groups)
    max_id = 0
    for ginfo in config.get('groups', {}).values():
        for gid in ginfo.get('motor_indices', []):
            max_id = max(max_id, gid)
    if max_id == 0:
        max_id = 20  # fallback
    init_arrays(max_id)

    # Init robot
    try:
        if demo_mode:
            robot = init_demo()
        else:
            robot = init_robot(cfg_path)
    except Exception as e:
        print(f"\nFailed to initialize robot: {e}")
        print("Falling back to DEMO mode")
        demo_mode = True
        robot = init_demo()

    # Start control + broadcast threads
    running = True
    control_thread = threading.Thread(target=control_loop, daemon=True)
    broadcast_thread = threading.Thread(target=state_broadcast_loop, daemon=True)
    control_thread.start()
    broadcast_thread.start()

    print(f"\n{'='*50}")
    print(f"  Backend API: http://localhost:{args.port}")
    print(f"  WebSocket:   ws://localhost:{args.port}")
    print(f"{'='*50}")
    print(f"\nServer starting on port {args.port}...")

    try:
        socketio.run(app, host='0.0.0.0', port=args.port,
                     allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        running = False
        if not demo_mode:
            try:
                _stop_script(restore_robot=False)
                _stop_external_gravity(restore_robot=False)
                _stop_external_impedance(restore_robot=False)
            except Exception:
                pass
        if robot is not None and not demo_mode:
            try:
                robot.stop_all()
                robot.close_all()
            except Exception:
                pass
        print("Done.")


if __name__ == '__main__':
    main()
