#!/usr/bin/env python3
"""Shared Python control code for LiteArm dual-arm backend modes.

This module mirrors the verified C++ examples:
  gravity:   tau = G(q)
  impedance: tau = G(q) + Kp * (q_target - q) - Kd * dq

MIT commands are sent with the current position, vel=0, torque=tau, kp=0, kd=0.
"""

import copy
import os
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
import yaml
from serial.tools import list_ports

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover - depends on active conda env
    raise RuntimeError(
        "pinocchio is required. Run with the same conda env as backend.sh."
    ) from exc


BACKEND_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PROJECT_DIR = os.path.abspath(os.path.join(BACKEND_DIR, ".."))
TEACH_DIR = os.path.join(PROJECT_DIR, "src", "litearm_robot", "teach")
if TEACH_DIR not in sys.path:
    sys.path.insert(0, TEACH_DIR)

from motor_driver import MultiMotorManager  # noqa: E402


DEFAULT_CONFIG = os.path.join(BACKEND_DIR, "robot_param", "litearm_arms.yaml")

LEFT_GRAVITY_GAIN = np.array([0.85, 1.0, 1.0, 0.8, 1.0, 1.0, 1.0])
RIGHT_GRAVITY_GAIN = np.array([1.0, 1.2, 1.0, 0.8, 1.0, 1.0, 1.0])
DEFAULT_KP = np.array([6.0, 12.0, 12.0, 4.5, 3.0, 1.5, 1.2])
DEFAULT_KD = np.array([0.9, 1.2, 1.2, 0.6, 0.375, 0.225, 0.15])
DEFAULT_TORQUE_LIMIT = np.array([15.0, 25.0, 25.0, 15.0, 6.0, 6.0, 4.0])
DEFAULT_FEEDBACK_LIMIT_TOLERANCE_RAD = 0.05
DEFAULT_FEEDBACK_JUMP_TOLERANCE_RAD = 0.35
DEFAULT_FEEDBACK_JUMP_VELOCITY_SCALE = 4.0
DEFAULT_FEEDBACK_MAX_BAD_SAMPLES = 5


def _as_float_array(values: Iterable[float], size: int, name: str) -> np.ndarray:
    arr = np.array(list(values), dtype=np.float64)
    if arr.size < size:
        raise ValueError(f"{name} requires at least {size} values, got {arr.size}")
    return arr[:size].copy()


def _resolve_path(config_dir: str, path: str) -> str:
    if not path:
        return path
    if os.path.isabs(path):
        return os.path.normpath(path)
    return os.path.normpath(os.path.join(config_dir, path))


def load_config(config_path: str = DEFAULT_CONFIG) -> dict:
    config_path = os.path.abspath(config_path)
    with open(config_path, "r", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle)
    cfg["_config_path"] = config_path
    cfg["_config_dir"] = os.path.dirname(config_path)
    urdf = cfg.setdefault("urdf", {})
    for key in ("file_path", "left_arm", "right_arm"):
        if key in urdf:
            urdf[key] = _resolve_path(cfg["_config_dir"], urdf[key])
    return cfg


def _split_numeric_port(path: str) -> Tuple[str, Optional[int]]:
    i = len(path)
    while i > 0 and path[i - 1].isdigit():
        i -= 1
    if i == len(path):
        return path, None
    return path[:i], int(path[i:])


def _serial_sort_key(path: str) -> Tuple[str, int, str]:
    prefix, number = _split_numeric_port(path)
    return prefix, number if number is not None else 10**9, path


def _candidate_serial_ports(prefix: str) -> List[str]:
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


def resolve_serial_ports(cfg: dict) -> dict:
    """Resolve /dev/ttyACM* order the same way as the backend app."""
    cfg = copy.deepcopy(cfg)
    serial_cfg = cfg.get("serial", {})
    auto_resolve = bool(serial_cfg.get("auto_resolve", True))
    wait_timeout = float(serial_cfg.get("wait_timeout", 5.0))
    wait_interval = float(serial_cfg.get("wait_interval", 0.2))
    configured = list(cfg.get("ports", {}).items())
    if not configured:
        return cfg

    deadline = time.time() + wait_timeout
    last_detected: Dict[str, List[str]] = {}
    resolved = None

    while True:
        attempted = {}
        ok = True
        grouped: Dict[str, List[str]] = {}
        for port, _ids in configured:
            prefix, _number = _split_numeric_port(port)
            grouped.setdefault(prefix, []).append(port)

        for prefix, requested_ports in grouped.items():
            detected = _candidate_serial_ports(prefix)
            last_detected[prefix] = detected
            exact_available = all(os.path.exists(p) for p in requested_ports)
            if exact_available:
                for port in requested_ports:
                    attempted[port] = port
                continue
            if auto_resolve and len(detected) >= len(requested_ports):
                for old_port, new_port in zip(requested_ports, detected):
                    attempted[old_port] = new_port
                continue
            ok = False

        if ok:
            resolved = attempted
            break
        if time.time() >= deadline:
            details = ", ".join(
                f"{prefix}: {ports}" for prefix, ports in last_detected.items()
            )
            raise FileNotFoundError(
                f"serial ports not available; configured={list(cfg.get('ports', {}).keys())}; "
                f"detected={details}"
            )
        time.sleep(wait_interval)

    if any(old != new for old, new in resolved.items()):
        print("Serial ports remapped after scan:")
        for old, new in resolved.items():
            if old != new:
                print(f"  {old} -> {new}")

    cfg["ports"] = {resolved[old]: ids for old, ids in configured}
    if "motor_types" in cfg:
        cfg["motor_types"] = {
            resolved.get(old, old): values for old, values in cfg["motor_types"].items()
        }
    return cfg


def build_port_motor_types(cfg: dict) -> Dict[str, Dict[int, str]]:
    result: Dict[str, Dict[int, str]] = {}
    for port, values in cfg.get("motor_types", {}).items():
        if isinstance(values, list):
            result[port] = {i + 1: str(value) for i, value in enumerate(values)}
        elif isinstance(values, dict):
            result[port] = {int(k): str(v) for k, v in values.items()}
    return result


def arm_motor_ids(cfg: dict, group_name: str) -> List[int]:
    group = cfg.get("groups", {}).get(group_name, {})
    motor_indices = list(group.get("motor_indices", []))
    joint_names = list(group.get("joint_names", []))
    gripper_index = group.get("gripper_index")
    ids = []
    for local_index, gid in enumerate(motor_indices):
        name = joint_names[local_index] if local_index < len(joint_names) else ""
        if local_index == gripper_index or "gripper" in name.lower():
            continue
        ids.append(int(gid))
    if len(ids) != 7:
        raise ValueError(f"{group_name} arm requires 7 non-gripper joints, got {len(ids)}")
    return ids


def format_vector(values: Iterable[float], precision: int = 3) -> str:
    return "[" + ", ".join(f"{float(v):.{precision}f}" for v in values) + "]"


def valid_vector(values: Iterable[float], expected_size: int) -> bool:
    arr = np.array(list(values), dtype=np.float64)
    return (
        arr.size >= expected_size
        and bool(np.all(np.isfinite(arr[:expected_size])))
        and bool(np.all(np.abs(arr[:expected_size]) < 100.0))
    )


@dataclass
class TorqueTerms:
    gravity: np.ndarray
    pd: np.ndarray
    total: np.ndarray
    clipped: np.ndarray


class ArmDynamics:
    def __init__(
        self,
        side: str,
        urdf_path: str,
        gravity_gain: Iterable[float],
        kp: Iterable[float],
        kd: Iterable[float],
        torque_limit: Iterable[float],
        dof: int = 7,
    ):
        self.side = side
        self.dof = int(dof)
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.gravity_gain = _as_float_array(gravity_gain, self.dof, "gravity_gain")
        self.kp = _as_float_array(kp, self.dof, "kp")
        self.kd = _as_float_array(kd, self.dof, "kd")
        self.torque_limit = _as_float_array(torque_limit, self.dof, "torque_limit")
        if self.model.nq != self.dof:
            print(
                f"[{side}] warning: Pinocchio nq={self.model.nq}, expected dof={self.dof}"
            )

    def gravity(self, q: Iterable[float]) -> np.ndarray:
        q_values = _as_float_array(q, self.dof, "q")
        q_model = np.zeros(self.model.nq, dtype=np.float64)
        for i in range(min(self.model.nq, q_values.size)):
            q_model[i] = q_values[i]
        v_zero = np.zeros(self.model.nv, dtype=np.float64)
        a_zero = np.zeros(self.model.nv, dtype=np.float64)
        tau = np.asarray(pin.rnea(self.model, self.data, q_model, v_zero, a_zero))
        gravity = np.zeros(self.dof, dtype=np.float64)
        for i in range(min(self.dof, tau.size)):
            gravity[i] = tau[i]
        return gravity * self.gravity_gain

    def clip(self, torque: Iterable[float]) -> np.ndarray:
        torque_arr = _as_float_array(torque, self.dof, "torque")
        return np.clip(torque_arr, -self.torque_limit, self.torque_limit)

    def impedance_terms(
        self,
        q: Iterable[float],
        dq: Iterable[float],
        q_target: Iterable[float],
    ) -> TorqueTerms:
        q_arr = _as_float_array(q, self.dof, "q")
        dq_arr = _as_float_array(dq, self.dof, "dq")
        target_arr = _as_float_array(q_target, self.dof, "q_target")
        gravity = self.gravity(q_arr)
        pd = self.kp * (target_arr - q_arr) - self.kd * dq_arr
        total = gravity + pd
        return TorqueTerms(gravity=gravity, pd=pd, total=total, clipped=self.clip(total))


class DualLiteArmPython:
    """Dual-arm Python controller using the backend motor_driver."""

    def __init__(self, config_path: str = DEFAULT_CONFIG):
        self.config_path = os.path.abspath(config_path)
        self.config = load_config(self.config_path)
        self.manager: Optional[MultiMotorManager] = None
        self.left_ids = arm_motor_ids(self.config, "left")
        self.right_ids = arm_motor_ids(self.config, "right")
        self.joint_limits = {}
        for side in ("left", "right"):
            group = self.config.get("groups", {}).get(side, {})
            limits = group.get("joint_limits", {})
            lower = _as_float_array(
                limits.get("lower", [-100.0] * 7),
                7,
                f"{side}.joint_limits.lower",
            )
            upper = _as_float_array(
                limits.get("upper", [100.0] * 7),
                7,
                f"{side}.joint_limits.upper",
            )
            if np.any(lower >= upper):
                raise ValueError(f"{side}.joint_limits lower must be below upper")
            self.joint_limits[side] = (lower, upper)

        control_cfg = self.config.get("control", {})
        self.feedback_limit_tolerance_rad = float(
            control_cfg.get(
                "feedback_limit_tolerance_rad",
                DEFAULT_FEEDBACK_LIMIT_TOLERANCE_RAD,
            )
        )
        self.feedback_jump_tolerance_rad = float(
            control_cfg.get(
                "feedback_jump_tolerance_rad",
                DEFAULT_FEEDBACK_JUMP_TOLERANCE_RAD,
            )
        )
        self.feedback_jump_velocity_scale = float(
            control_cfg.get(
                "feedback_jump_velocity_scale",
                DEFAULT_FEEDBACK_JUMP_VELOCITY_SCALE,
            )
        )
        self.state_wait_after_request = float(
            control_cfg.get("state_wait_after_request", 0.001)
        )
        self.feedback_max_bad_samples = int(
            control_cfg.get(
                "feedback_max_bad_samples",
                DEFAULT_FEEDBACK_MAX_BAD_SAMPLES,
            )
        )
        if self.feedback_limit_tolerance_rad < 0.0:
            raise ValueError("feedback_limit_tolerance_rad must be non-negative")
        if self.feedback_jump_tolerance_rad <= 0.0:
            raise ValueError("feedback_jump_tolerance_rad must be positive")
        if self.feedback_jump_velocity_scale < 0.0:
            raise ValueError("feedback_jump_velocity_scale must be non-negative")
        if self.state_wait_after_request < 0.0:
            raise ValueError("state_wait_after_request must be non-negative")
        if self.feedback_max_bad_samples < 1:
            raise ValueError("feedback_max_bad_samples must be at least 1")
        self._feedback_prev_q: Optional[Dict[str, np.ndarray]] = None
        self._feedback_prev_dq: Optional[Dict[str, np.ndarray]] = None
        self._feedback_prev_time: Optional[float] = None
        self._feedback_fault: Optional[str] = None
        self._feedback_bad_counts = {
            side: np.zeros(7, dtype=np.int64) for side in ("left", "right")
        }
        self.motor_count = max(
            gid
            for group in self.config.get("groups", {}).values()
            for gid in group.get("motor_indices", [])
        )
        impedance_cfg = self.config.get("impedance", {})
        kp = impedance_cfg.get("kp", DEFAULT_KP.tolist())[:7]
        kd = impedance_cfg.get("kd", DEFAULT_KD.tolist())[:7]
        torque_limit = impedance_cfg.get("torque_limit", DEFAULT_TORQUE_LIMIT.tolist())[:7]
        self.dynamics = {
            "left": ArmDynamics(
                "left",
                self.config["urdf"]["left_arm"],
                LEFT_GRAVITY_GAIN,
                kp,
                kd,
                torque_limit,
            ),
            "right": ArmDynamics(
                "right",
                self.config["urdf"]["right_arm"],
                RIGHT_GRAVITY_GAIN,
                kp,
                kd,
                torque_limit,
            ),
        }
        self.positions = np.zeros(self.motor_count, dtype=np.float64)
        self.velocities = np.zeros(self.motor_count, dtype=np.float64)
        self.torques = np.zeros(self.motor_count, dtype=np.float64)

    def open(self) -> None:
        cfg = resolve_serial_ports(self.config)
        self.config = cfg
        port_motor_types = build_port_motor_types(cfg)
        self.manager = MultiMotorManager(cfg["ports"], port_motor_types)
        self._feedback_prev_q = None
        self._feedback_prev_dq = None
        self._feedback_prev_time = None
        self._feedback_fault = None
        self._feedback_bad_counts = {
            side: np.zeros(7, dtype=np.int64) for side in ("left", "right")
        }
        print(f"Opening {len(cfg['ports'])} serial ports for {self.motor_count} motors")
        self.manager.open_all()
        self.manager.init_all()
        self.read_state(wait_after_request=0.1)

    def close(self, stop: bool = False) -> None:
        if self.manager is None:
            return
        if stop:
            try:
                self.manager.stop_all()
            except Exception as exc:
                print(f"stop_all failed during close: {exc}")
        try:
            self.manager.close_all()
        finally:
            self.manager = None

    def read_state(
        self, wait_after_request: Optional[float] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        if self.manager is None:
            raise RuntimeError("controller is not open")
        if wait_after_request is None:
            wait_after_request = self.state_wait_after_request
        self.manager.request_all_states()
        if wait_after_request > 0.0:
            time.sleep(wait_after_request)
        states = self.manager.get_all_states()
        for gid in range(1, self.motor_count + 1):
            state = states.get(gid)
            if state is None:
                continue
            index = gid - 1
            if int(state.fault) != 0 and self._feedback_fault is None:
                self._feedback_fault = (
                    f"motor fault for global motor ID {gid}: "
                    f"fault=0x{int(state.fault):02x}, mode={int(state.mode)}"
                )
            if (
                np.isfinite(state.pos)
                and np.isfinite(state.vel)
                and np.isfinite(state.torque)
                and abs(state.pos) < 100.0
            ):
                self.positions[index] = state.pos
                self.velocities[index] = state.vel
                self.torques[index] = state.torque
            elif self._feedback_fault is None:
                self._feedback_fault = (
                    f"invalid feedback for global motor ID {gid}: "
                    f"pos={state.pos}, vel={state.vel}, torque={state.torque}"
                )
        return self.positions.copy(), self.velocities.copy()

    def arm_ids(self, side: str) -> List[int]:
        if side == "left":
            return self.left_ids
        if side == "right":
            return self.right_ids
        raise ValueError(f"unknown side: {side}")

    def arm_q(self, side: str) -> np.ndarray:
        return np.array([self.positions[gid - 1] for gid in self.arm_ids(side)])

    def arm_dq(self, side: str) -> np.ndarray:
        return np.array([self.velocities[gid - 1] for gid in self.arm_ids(side)])

    def validate_arms(self) -> None:
        if self._feedback_fault is not None:
            raise RuntimeError(self._feedback_fault)

        now = time.monotonic()
        jump_details: Dict[Tuple[str, int], Tuple[float, float, float, float]] = {}
        dt = None
        if self._feedback_prev_q is not None and self._feedback_prev_time is not None:
            dt = max(now - self._feedback_prev_time, 1.0e-3)

        for side in ("left", "right"):
            q = self.arm_q(side)
            dq = self.arm_dq(side)
            if not valid_vector(q, 7) or not valid_vector(dq, 7):
                raise RuntimeError(f"invalid {side} arm state")

            lower, upper = self.joint_limits[side]
            safe_lower = lower - self.feedback_limit_tolerance_rad
            safe_upper = upper + self.feedback_limit_tolerance_rad

            outside = (q < safe_lower) | (q > safe_upper)
            abnormal = np.zeros(7, dtype=bool)
            if dt is not None:
                delta = np.abs(q - self._feedback_prev_q[side])
                allowed = (
                    self.feedback_jump_tolerance_rad
                    + self.feedback_jump_velocity_scale * np.abs(dq) * dt
                )
                abnormal = delta > allowed
                for joint in np.flatnonzero(abnormal):
                    joint_index = int(joint)
                    estimated_dq = (
                        (q[joint_index] - self._feedback_prev_q[side][joint_index])
                        / dt
                    )
                    jump_details[(side, joint_index)] = (
                        float(delta[joint_index]),
                        float(dq[joint_index]),
                        float(allowed[joint_index]),
                        float(estimated_dq),
                    )

            bad = outside | abnormal
            self._feedback_bad_counts[side][~bad] = 0

            for joint in np.flatnonzero(bad):
                joint_index = int(joint)
                count = int(self._feedback_bad_counts[side][joint_index]) + 1
                self._feedback_bad_counts[side][joint_index] = count
                gid = self.arm_ids(side)[joint_index]

                if self._feedback_prev_q is None or self._feedback_prev_dq is None:
                    raise RuntimeError(
                        f"{side} j{joint_index + 1} invalid initial feedback: "
                        f"q={q[joint_index]:.4f} rad, global_id={gid}"
                    )

                if count >= self.feedback_max_bad_samples:
                    if outside[joint_index]:
                        raise RuntimeError(
                            f"{side} j{joint_index + 1} feedback out of limits "
                            f"for {count} consecutive samples: "
                            f"q={q[joint_index]:.4f} rad, allowed=["
                            f"{safe_lower[joint_index]:.4f}, "
                            f"{safe_upper[joint_index]:.4f}] rad, "
                            f"global_id={gid}"
                        )

                    (
                        delta_value,
                        dq_value,
                        allowed_value,
                        estimated_dq_value,
                    ) = jump_details[
                        (side, joint_index)
                    ]
                    raise RuntimeError(
                        f"{side} j{joint_index + 1} feedback jump persisted for "
                        f"{count} consecutive samples: "
                        f"previous={self._feedback_prev_q[side][joint_index]:.4f} rad, "
                        f"current={q[joint_index]:.4f} rad, "
                        f"delta={delta_value:.4f} rad, "
                        f"allowed={allowed_value:.4f} rad, "
                        f"dq_reported={dq_value:.4f} rad/s, "
                        f"dq_from_position={estimated_dq_value:.4f} rad/s, "
                        f"dt={dt:.4f} s, "
                        f"global_id={gid}"
                    )

                # A single bad encoder/CAN sample must not become the next
                # control state. Keep the last accepted position and velocity.
                self.positions[gid - 1] = self._feedback_prev_q[side][joint_index]
                self.velocities[gid - 1] = self._feedback_prev_dq[side][joint_index]
                if count == 1:
                    reason = "limit" if outside[joint_index] else "jump"
                    print(
                        f"[Feedback] holding {side} j{joint_index + 1} "
                        f"previous sample ({reason}, "
                        f"{count}/{self.feedback_max_bad_samples}, "
                        f"global_id={gid})"
                    )

        # Store the state after transient samples have been replaced. Bad
        # samples therefore cannot poison the baseline for the next cycle.
        self._feedback_prev_q = {
            side: self.arm_q(side).copy() for side in ("left", "right")
        }
        self._feedback_prev_dq = {
            side: self.arm_dq(side).copy() for side in ("left", "right")
        }
        self._feedback_prev_time = now

    def compute_gravity(self, side: str, q: Optional[Iterable[float]] = None) -> np.ndarray:
        q_values = self.arm_q(side) if q is None else q
        return self.dynamics[side].clip(self.dynamics[side].gravity(q_values))

    def compute_impedance(
        self,
        side: str,
        q_target: Iterable[float],
        q: Optional[Iterable[float]] = None,
        dq: Optional[Iterable[float]] = None,
    ) -> TorqueTerms:
        q_values = self.arm_q(side) if q is None else q
        dq_values = self.arm_dq(side) if dq is None else dq
        return self.dynamics[side].impedance_terms(q_values, dq_values, q_target)

    def current_arm_targets(self) -> Dict[str, np.ndarray]:
        return {
            "left": self.arm_q("left").copy(),
            "right": self.arm_q("right").copy(),
        }

    def send_mit_torque(
        self,
        left_torque: Iterable[float],
        right_torque: Iterable[float],
    ) -> None:
        if self.manager is None:
            raise RuntimeError("controller is not open")
        command = {}
        left_q = self.arm_q("left")
        right_q = self.arm_q("right")
        for gid, position, torque in zip(
            self.left_ids,
            left_q,
            _as_float_array(left_torque, 7, "left_torque"),
        ):
            command[gid] = (float(position), 0.0, float(torque), 0.0, 0.0)
        for gid, position, torque in zip(
            self.right_ids,
            right_q,
            _as_float_array(right_torque, 7, "right_torque"),
        ):
            command[gid] = (float(position), 0.0, float(torque), 0.0, 0.0)
        self.manager.set_all_pos_vel_torque_kp_kd(command)

    def print_summary(self) -> None:
        print(f"Config: {self.config_path}")
        print(f"Left arm URDF: {self.config['urdf']['left_arm']}")
        print(f"Right arm URDF: {self.config['urdf']['right_arm']}")
        print(f"Left arm IDs: {self.left_ids}")
        print(f"Right arm IDs: {self.right_ids}")
        for side in ("left", "right"):
            dyn = self.dynamics[side]
            print(f"[{side}] gravity gain: {format_vector(dyn.gravity_gain)}")
            print(f"[{side}] Kp: {format_vector(dyn.kp)}")
            print(f"[{side}] Kd: {format_vector(dyn.kd)}")
            print(f"[{side}] torque limit: {format_vector(dyn.torque_limit)}")
            lower, upper = self.joint_limits[side]
            print(f"[{side}] joint lower limit: {format_vector(lower)}")
            print(f"[{side}] joint upper limit: {format_vector(upper)}")
        print(
            "feedback safety: joint limits + jump detection "
            f"(limit_tol={self.feedback_limit_tolerance_rad:.3f} rad, "
            f"jump_base={self.feedback_jump_tolerance_rad:.3f} rad, "
            f"hold={self.feedback_max_bad_samples - 1} bad samples, "
            f"state_wait={self.state_wait_after_request * 1000.0:.1f} ms)"
        )
