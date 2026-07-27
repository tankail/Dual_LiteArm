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
control_mode = "position"  # "position" | "free"
demo_mode = False
running = False
state_lock = threading.Lock()
target_lock = threading.Lock()


def init_arrays(motor_count):
    """Resize global arrays to match config."""
    global MOTOR_COUNT, positions, velocities, torques
    MOTOR_COUNT = motor_count
    positions = np.zeros(MOTOR_COUNT, dtype=np.float64)
    velocities = np.zeros(MOTOR_COUNT, dtype=np.float64)
    torques = np.zeros(MOTOR_COUNT, dtype=np.float64)

# Flat motor arrays — sized at startup from config
MOTOR_COUNT = 0
positions = np.array([], dtype=np.float64)
velocities = np.array([], dtype=np.float64)
torques = np.array([], dtype=np.float64)
connected = False
grav_engine = None  # GravityCompensationEngine, initialized after config loaded
gravity_process = None  # External C++ gravity runner process
gravity_serial_released = False
gravity_transitioning = False
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
# Gravity Compensation Engine
# ═══════════════════════════════════════════════════════════════

class GravityCompensationEngine:
    """Compute gravity torque via Pinocchio RNEA for left/right arms."""

    def __init__(self, left_urdf_path, right_urdf_path):
        self.ready = False
        self.models = {}
        self.datas = {}
        self.joint_maps = {}       # side → [joint_ids in order matching positions]
        self.tau_limits = {}       # side → torque limits per joint
        self.gravity_gains = {}    # side → gain per joint

        if not HAS_PINOCCHIO:
            return

        # Torque limits and gravity gains (same as C++ examples)
        tau_limit = [15.0, 25.0, 25.0, 15.0, 6.0, 6.0, 4.0]

        configs = {
            'left':  (left_urdf_path,  tau_limit, [0.85, 1.0, 1.0, 0.8, 1.0, 1.0, 1.0]),
            'right': (right_urdf_path, tau_limit, [1.0, 1.2, 1.0, 0.8, 1.0, 1.0, 1.0]),
        }

        for side, (urdf_path, tlim, ggain) in configs.items():
            if not urdf_path or not os.path.exists(urdf_path):
                print(f"[Gravity] {side} URDF not found: {urdf_path}")
                continue
            try:
                model = pin.buildModelFromUrdf(urdf_path)
                data = model.createData()
                self.models[side] = model
                self.datas[side] = data
                # Cache joint IDs for the first 7 revolute joints
                joint_ids = []
                for jname in model.names:
                    if model.getJointId(jname) < model.njoints and model.joints[model.getJointId(jname)].nq > 0:
                        joint_ids.append(model.getJointId(jname))
                self.joint_maps[side] = joint_ids[:7]
                self.tau_limits[side] = tlim[:len(joint_ids[:7])]
                self.gravity_gains[side] = ggain[:len(joint_ids[:7])]
                print(f"[Gravity] {side} arm loaded: {model.nq} DOF, {len(self.joint_maps[side])} joints")
            except Exception as e:
                print(f"[Gravity] {side} arm init failed: {e}")

        if set(self.models) == set(configs):
            self.ready = True
        else:
            print("[Gravity] disabled: both left and right arm models are required")

    def compute(self, joint_positions, side='left'):
        """Compute gravity torque for one arm via RNEA with v=0, a=0."""
        if not self.ready or side not in self.models:
            return None
        try:
            model = self.models[side]
            data = self.datas[side]
            joint_ids = self.joint_maps[side]
            n = len(joint_ids)

            q = np.zeros(model.nq)
            for i in range(min(n, len(joint_positions))):
                q[model.joints[joint_ids[i]].idx_q] = float(joint_positions[i])

            v = np.zeros(model.nv)
            a = np.zeros(model.nv)
            tau = pin.rnea(model, data, q, v, a)

            # Extract first n torques
            G = np.array([tau[model.joints[jid].idx_v] for jid in joint_ids])

            # Apply gain
            for i in range(n):
                G[i] *= self.gravity_gains[side][i]

            # Clip torque
            tlim = self.tau_limits[side]
            for i in range(n):
                G[i] = max(-tlim[i], min(tlim[i], G[i]))

            return G.tolist()
        except Exception as e:
            print(f"[Gravity] compute({side}) error: {e}")
            return None


# ═══════════════════════════════════════════════════════════════
# Control Loop (100 Hz)
# ═══════════════════════════════════════════════════════════════

def _get_gravity_engine():
    """Lazy-init gravity compensation engine from config URDFs."""
    global grav_engine
    if grav_engine is None and config:
        left_urdf = config.get('urdf', {}).get('left_arm', '')
        right_urdf = config.get('urdf', {}).get('right_arm', '')
        if left_urdf and right_urdf:
            grav_engine = GravityCompensationEngine(left_urdf, right_urdf)
    return grav_engine


def _read_states():
    """Request and copy the latest motor states into the shared arrays."""
    global positions, velocities, torques
    if robot is None:
        return
    robot.request_all_states()
    states = robot.get_all_states()
    with state_lock:
        for gid in range(1, MOTOR_COUNT + 1):
            st = states.get(gid)
            idx = gid - 1
            if st and abs(st.pos) < 100:
                positions[idx] = st.pos
                velocities[idx] = st.vel
                torques[idx] = st.torque


def _hold_current_positions():
    """Prevent a mode change from sending stale position targets."""
    with state_lock:
        current = positions.copy()
    with target_lock:
        for i, value in enumerate(current, start=1):
            targets[i] = float(value)


def _gravity_runner_path():
    return os.path.join(BACKEND_DIR, 'gravity_compensation_runner.py')


def _is_gravity_runner_alive():
    return gravity_process is not None and gravity_process.poll() is None


def _release_robot_for_external_gravity():
    global robot, gravity_serial_released
    if robot is None or demo_mode:
        return
    try:
        robot.stop_all()
    except Exception:
        pass
    try:
        robot.close_all()
    except Exception:
        pass
    gravity_serial_released = True


def _restore_robot_after_external_gravity():
    global robot, gravity_serial_released
    if demo_mode or not gravity_serial_released:
        return
    if not active_config_path:
        gravity_serial_released = False
        return
    try:
        print("[Gravity] Re-opening backend serial connection")
        robot = init_robot(active_config_path)
        gravity_serial_released = False
        _read_states()
        _hold_current_positions()
    except Exception as exc:
        gravity_serial_released = False
        print(f"[Gravity] Failed to re-open backend serial connection: {exc}")


def _start_external_gravity():
    global gravity_process, control_mode, gravity_transitioning
    if demo_mode:
        return False, "demo mode does not use external gravity compensation"
    if _is_gravity_runner_alive():
        return True, None

    runner = _gravity_runner_path()
    if not os.path.exists(runner):
        return False, f"gravity runner not found: {runner}"

    gravity_transitioning = True

    try:
        _hold_current_positions()
        control_mode = 'gravity_comp'
        _release_robot_for_external_gravity()
        gravity_process = subprocess.Popen(
            [sys.executable, runner, '--side', 'both'],
            cwd=BACKEND_DIR,
            stdout=None,
            stderr=None,
            start_new_session=True,
        )
        time.sleep(0.3)
        if gravity_process.poll() is not None:
            code = gravity_process.returncode
            gravity_process = None
            _restore_robot_after_external_gravity()
            control_mode = 'position'
            return False, f"gravity runner exited immediately with code {code}"
        print(f"[Gravity] External C++ runner started, pid={gravity_process.pid}")
        return True, None
    except Exception as exc:
        gravity_process = None
        _restore_robot_after_external_gravity()
        control_mode = 'position'
        return False, str(exc)
    finally:
        gravity_transitioning = False


def _stop_external_gravity(restore_robot=True):
    global gravity_process
    proc = gravity_process
    gravity_process = None
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
                try:
                    proc.kill()
                except Exception:
                    pass
    if restore_robot:
        _restore_robot_after_external_gravity()


def control_loop():
    """Send position commands to motors at control rate."""
    global running, targets, control_mode
    loop_hz = config.get('control', {}).get('loop_hz', 100)
    gravity_loop_hz = config.get('control', {}).get('gravity_loop_hz', 200)
    print(f"[Control] Starting at {loop_hz} Hz "
          f"(gravity compensation: {gravity_loop_hz} Hz)")

    while running:
        t0 = time.time()
        try:
            if control_mode == 'free':
                pass

            elif control_mode == 'gravity_comp':
                # External C++ examples own the serial ports in this mode.
                if (gravity_serial_released and not gravity_transitioning
                        and not _is_gravity_runner_alive()):
                    print("[Gravity] External runner stopped unexpectedly; returning to position mode")
                    _stop_external_gravity(restore_robot=True)
                    control_mode = 'position'
                pass

            elif control_mode == 'position' and targets:
                cmd = {}
                default_vel = float(config.get('control', {}).get('default_velocity', 0.5))
                max_torque = float(config.get('robot', {}).get('max_torque', 15.0))
                with target_lock:
                    for gid, tgt in targets.items():
                        cmd[gid] = (float(tgt), default_vel, max_torque)
                if cmd and robot is not None:
                    try:
                        robot.set_all_pos_vel_max_torque(cmd)
                    except Exception:
                        pass  # Serial errors are logged by motor_driver
        except Exception as e:
            print(f"[Control] Error: {e}")

        active_hz = gravity_loop_hz if control_mode == 'gravity_comp' else loop_hz
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
            if robot is not None:
                if control_mode != 'gravity_comp':
                    _read_states()

                # Build state dict — per-group slices from config
                state = {
                    'positions': positions.tolist(),
                    'velocities': velocities.tolist(),
                    'torques': torques.tolist(),
                    'target_positions': [float(targets.get(i+1, 0)) for i in range(MOTOR_COUNT)],
                    'control_mode': control_mode,
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
                else:
                    state['forward_kinematics'] = None
                    state['ee_position'] = [0, 0, 0]
                    state['ee_euler'] = [0, 0, 0]
                    state['gripper_position'] = 0.0
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
        'control_mode': control_mode,
        'joints': joints,
        'groups': groups,
        'control': config.get('control', {}),
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
            'timestamp': time.time(),
        })


@app.route('/api/move', methods=['POST'])
def api_move():
    """Move one or more groups. Body: {groups: {left: [...], right: [...]}, velocity: 0.5}"""
    data = flask_request.get_json()
    if not data:
        return jsonify({'error': 'No JSON body'}), 400

    velocity = float(data.get('velocity', config.get('control', {}).get('default_velocity', 0.5)))
    max_torque = float(data.get('max_torque', config.get('robot', {}).get('max_torque', 15.0)))

    groups_data = data.get('groups', {})
    if not groups_data:
        # Flat positions array for all motors
        flat = data.get('positions', [])
        if len(flat) == MOTOR_COUNT:
            with target_lock:
                for i, pos in enumerate(flat):
                    targets[i+1] = float(pos)
        return jsonify({'ok': True})

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
                lo = limits_lo[0] if limits_lo else 0
                hi = limits_hi[0] if limits_hi else 5
                targets[indices[0]] = max(lo, min(hi, float(gvals)))

    return jsonify({'ok': True, 'targets_count': len(targets)})


@app.route('/api/home', methods=['POST'])
def api_home():
    """Home all joints to zero (smooth)."""
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
    if robot is not None:
        try:
            robot.stop_all()
        except Exception:
            pass
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
    if mode not in ('position', 'free', 'gravity_comp'):
        return jsonify({'error': f'Unknown mode: {mode}'}), 400
    if mode == 'gravity_comp':
        ok, error = _start_external_gravity()
        if not ok:
            return jsonify({'error': error}), 503
    if control_mode == 'gravity_comp' and mode != 'gravity_comp':
        _stop_external_gravity(restore_robot=True)
        _hold_current_positions()
    control_mode = mode
    if mode == 'free' and robot is not None:
        try:
            robot.set_all_free_mode()
        except Exception:
            pass
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
        'joints': build_joint_list(config),
        'groups': {g: {'name': gi.get('name', g),
                       'joint_count': len(gi['joint_names']),
                       'motor_indices': gi['motor_indices']}
                   for g, gi in config.get('groups', {}).items()},
        'control': config.get('control', {}),
    })


@socketio.on('disconnect')
def on_disconnect():
    print("Client disconnected")


@socketio.on('move_all')
def ws_move_all(data):
    if isinstance(data, dict) and 'positions' in data:
        with target_lock:
            flat = data['positions']
            for i, pos in enumerate(flat[:MOTOR_COUNT]):
                targets[i+1] = float(pos)


@socketio.on('move_group')
def ws_move_group(data):
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
    with target_lock:
        for i in range(1, MOTOR_COUNT + 1):
            targets[i] = 0.0


@socketio.on('reset_all')
def ws_reset_all(data=None):
    """Reset all joints to zero position."""
    with target_lock:
        for i in range(1, MOTOR_COUNT + 1):
            targets[i] = 0.0
    print("[Reset] All targets → 0")


@socketio.on('stop')
def ws_stop(data=None):
    if robot is not None:
        try:
            robot.stop_all()
        except Exception:
            pass


@socketio.on('gravity_comp')
def ws_gravity_comp(data=None):
    """Toggle gravity compensation mode."""
    global control_mode
    enable = data.get('enable', True) if isinstance(data, dict) else True
    if enable:
        ok, error = _start_external_gravity()
        if not ok:
            print(f"[Gravity] Compensation rejected: {error}")
            socketio.emit('backend_error', {
                'message': f'重力补偿启动失败: {error}'
            })
            return
        control_mode = 'gravity_comp'
        print("[Gravity] Compensation ENABLED via C++ runner")
    else:
        if control_mode == 'gravity_comp':
            _stop_external_gravity(restore_robot=True)
            _hold_current_positions()
        control_mode = 'position'
        print("[Gravity] Compensation DISABLED → position mode")
    socketio.emit('mode_changed', {'mode': control_mode})


@socketio.on('set_mode')
def ws_set_mode(data):
    global control_mode
    mode = data.get('mode', 'position')
    if mode in ('position', 'free', 'gravity_comp'):
        if mode == 'gravity_comp':
            ok, error = _start_external_gravity()
            if not ok:
                print(f"[Gravity] Mode change rejected: {error}")
                socketio.emit('backend_error', {
                    'message': f'重力补偿启动失败: {error}'
                })
                return
        if control_mode == 'gravity_comp' and mode != 'gravity_comp':
            _stop_external_gravity(restore_robot=True)
            _hold_current_positions()
        control_mode = mode
        if mode == 'gravity_comp':
            print("[Gravity] Compensation ENABLED via C++ runner")
        elif mode == 'free' and robot is not None:
            try:
                robot.set_all_free_mode()
            except Exception:
                pass
        socketio.emit('mode_changed', {'mode': mode})


# ═══════════════════════════════════════════════════════════════
# Initialization
# ═══════════════════════════════════════════════════════════════

def init_robot(cfg_path):
    """Initialize MultiMotorManager for live mode."""
    global config, targets, connected, positions, velocities, torques

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

    connected = True
    print(f"   Initialized. Positions sample: "
          f"{[f'{positions[i]:.3f}' for i in range(7)]}")

    return mgr


def init_demo():
    """Initialize demo (simulated) mode."""
    global config, targets, connected
    print("\n" + "=" * 50)
    print("LiteArm A10 Digital Twin Backend [DEMO MODE]")
    print("=" * 50 + "\n")
    print("No hardware — generating simulated motor states")

    connected = False
    with target_lock:
        for i in range(1, MOTOR_COUNT + 1):
            targets[i] = 0.0
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
        if robot is not None and not demo_mode:
            try:
                _stop_external_gravity(restore_robot=False)
                robot.stop_all()
                robot.close_all()
            except Exception:
                pass
        print("Done.")


if __name__ == '__main__':
    main()
