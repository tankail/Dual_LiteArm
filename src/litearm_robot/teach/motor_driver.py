#!/usr/bin/env python3
"""
Livelybot 电机串口协议驱动 (纯Python)
协议参考: Panthera-HT_SDK / left_litearm 源码

支持多端口多电机, 全局ID自动编排:
  /dev/ttyACM0: 左手臂 8电机 → 全局ID 1-8
  /dev/ttyACM1: 右手臂 8电机 → 全局ID 9-16
  /dev/ttyACM2: 腰部    2电机 → 全局ID 17-18
  /dev/ttyACM3: 头部    2电机 → 全局ID 19-20
"""

import struct
import serial
import time
import threading
from typing import NamedTuple, Optional

# ======================== 协议常量 ========================
MODE_POSITION            = 0x80
MODE_VELOCITY            = 0x81
MODE_TORQUE              = 0x82
MODE_VOLTAGE             = 0x83
MODE_CURRENT             = 0x84
MODE_TIME_OUT            = 0x85
MODE_POS_VEL_TQE         = 0x90
MODE_POS_VEL_KP_KD       = 0x9E
MODE_POS_VEL_ACC         = 0xAD
MODE_POS_VEL_TQE_KP_KD_2 = 0xB0
MODE_NULL                = 0x00
MODE_RESET_ZERO          = 0x01
MODE_CONF_WRITE          = 0x02
MODE_STOP                = 0x03
MODE_BRAKE               = 0x04
MODE_SET_NUM             = 0x05
MODE_MOTOR_STATE         = 0x06
MODE_RESET               = 0x08
MODE_MOTOR_STATE2        = 0x0A
MODE_MOTOR_VERSION       = 0x0B
MODE_FUN_V               = 0x0C
MODE_FDCAN_RESET         = 0x0E
MODE_FDCAN_MOTOR_STATE   = 0x0F
MODE_FDCAN_MOTOR_STATE2  = 0x11

FRAME_HEADER   = 0xF7
MY_2PI         = 6.28318530717
SENTINEL_INT16 = -32768

def _ver(maj, mn, pat): return (maj << 12) | (mn << 4) | pat

# ======================== CRC 表 ========================
CRC8_TABLE = [
    0x00,0x5e,0xbc,0xe2,0x61,0x3f,0xdd,0x83,0xc2,0x9c,0x7e,0x20,0xa3,0xfd,0x1f,0x41,
    0x9d,0xc3,0x21,0x7f,0xfc,0xa2,0x40,0x1e,0x5f,0x01,0xe3,0xbd,0x3e,0x60,0x82,0xdc,
    0x23,0x7d,0x9f,0xc1,0x42,0x1c,0xfe,0xa0,0xe1,0xbf,0x5d,0x03,0x80,0xde,0x3c,0x62,
    0xbe,0xe0,0x02,0x5c,0xdf,0x81,0x63,0x3d,0x7c,0x22,0xc0,0x9e,0x1d,0x43,0xa1,0xff,
    0x46,0x18,0xfa,0xa4,0x27,0x79,0x9b,0xc5,0x84,0xda,0x38,0x66,0xe5,0xbb,0x59,0x07,
    0xdb,0x85,0x67,0x39,0xba,0xe4,0x06,0x58,0x19,0x47,0xa5,0xfb,0x78,0x26,0xc4,0x9a,
    0x65,0x3b,0xd9,0x87,0x04,0x5a,0xb8,0xe6,0xa7,0xf9,0x1b,0x45,0xc6,0x98,0x7a,0x24,
    0xf8,0xa6,0x44,0x1a,0x99,0xc7,0x25,0x7b,0x3a,0x64,0x86,0xd8,0x5b,0x05,0xe7,0xb9,
    0x8c,0xd2,0x30,0x6e,0xed,0xb3,0x51,0x0f,0x4e,0x10,0xf2,0xac,0x2f,0x71,0x93,0xcd,
    0x11,0x4f,0xad,0xf3,0x70,0x2e,0xcc,0x92,0xd3,0x8d,0x6f,0x31,0xb2,0xec,0x0e,0x50,
    0xaf,0xf1,0x13,0x4d,0xce,0x90,0x72,0x2c,0x6d,0x33,0xd1,0x8f,0x0c,0x52,0xb0,0xee,
    0x32,0x6c,0x8e,0xd0,0x53,0x0d,0xef,0xb1,0xf0,0xae,0x4c,0x12,0x91,0xcf,0x2d,0x73,
    0xca,0x94,0x76,0x28,0xab,0xf5,0x17,0x49,0x08,0x56,0xb4,0xea,0x69,0x37,0xd5,0x8b,
    0x57,0x09,0xeb,0xb5,0x36,0x68,0x8a,0xd4,0x95,0xcb,0x29,0x77,0xf4,0xaa,0x48,0x16,
    0xe9,0xb7,0x55,0x0b,0x88,0xd6,0x34,0x6a,0x2b,0x75,0x97,0xc9,0x4a,0x14,0xf6,0xa8,
    0x74,0x2a,0xc8,0x96,0x15,0x4b,0xa9,0xf7,0xb6,0xe8,0x0a,0x54,0xd7,0x89,0x6b,0x35,
]

CRC16_TABLE = [
    0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
    0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
    0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,
    0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
    0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,
    0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
    0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,
    0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
    0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,
    0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
    0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,
    0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
    0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,
    0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
    0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,
    0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
    0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,
    0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
    0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,
    0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
    0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,
    0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
    0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,
    0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
    0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,
    0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
    0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,
    0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
    0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,
    0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
    0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,
    0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78,
]


def crc8(data, init=0xFF):
    r = init
    for b in data: r = CRC8_TABLE[r ^ b]
    return r

def crc16(data, init=0xFFFF):
    r = init
    for b in data: r = (r >> 8) ^ CRC16_TABLE[(r ^ b) & 0xFF]
    return r


class MotorState(NamedTuple):
    pos: float; vel: float; torque: float; mode: int; fault: int


class MotorDriver:
    """单个串口上的电机驱动"""

    def __init__(self, port: str, motor_ids: list, baudrate=4000000):
        self.port = port
        self.motor_ids = sorted(motor_ids)
        self.id_max = max(motor_ids) if motor_ids else 1
        self.num_motors = len(motor_ids)
        self._lock = threading.Lock()
        self._ser = serial.Serial()
        self._ser.port = port
        self._ser.baudrate = baudrate
        self._ser.timeout = 0.1
        self._running = False
        self._rx_thread = None
        self._board_version = 0
        self._fun_v_confirmed = False
        self._motor_versions = {}
        # 本地ID → 状态
        self._states = {mid: MotorState(999,0,0,0,0) for mid in motor_ids}

    def open(self):
        print(f"  {self.port}: 打开串口...")
        self._ser.open()
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._running = True
        self._rx_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._rx_thread.start()
        time.sleep(0.05)  # 确保接收线程已启动
        print(f"  {self.port}: OK ({self.num_motors}个电机)")

    def init_board(self):
        print(f"  {self.port}: Step1 SET_NUM (告知通信板电机数量={self.num_motors})...")
        for i in range(250):  # 最多等 5 秒
            self._send_raw(MODE_SET_NUM, struct.pack('<BB', self.num_motors, 0))
            time.sleep(0.02)
            if self._board_version >= _ver(3,0,0): break
            if i % 50 == 49: print(f"    ... 等待通信板响应 ({i+1}/250)")
        if self._board_version:
            print(f"  {self.port}: 通信板 v{self._board_version>>12&0xF}.{(self._board_version>>4)&0xFF}.{self._board_version&0xF}")
        else:
            print(f"  {self.port}: 警告 - 未收到通信板版本, 跳过初始化")

        if self._board_version >= _ver(4,1,0):
            print(f"  {self.port}: Step2 FDCAN_RESET...")
            # 与C++对齐: 连发3次确保FDCAN总线可靠复位
            self._send_raw(MODE_FDCAN_RESET, b'\x7F')
            self._send_raw(MODE_FDCAN_RESET, b'\x7F')
            self._send_raw(MODE_FDCAN_RESET, b'\x7F')
            time.sleep(0.1)  # 等待FDCAN总线重新枚举电机

        if self._board_version:
            print(f"  {self.port}: Step3 检测电机固件版本...")
            self._motor_versions.clear()
            for i in range(20):
                self._send_raw(MODE_MOTOR_VERSION, b'\x7F'); time.sleep(0.1)
                if len(self._motor_versions) >= self.num_motors: break
            if self._motor_versions:
                for mid, v in self._motor_versions.items():
                    print(f"    ID={mid} v{v[0]}.{v[1]}.{v[2]}")
            else:
                print(f"    警告 - 未检测到电机")

            # 根据实际电机版本确定 fun_v
            if self._motor_versions:
                mv = min(_ver(v[0],v[1],v[2]) for v in self._motor_versions.values())
            else:
                mv = _ver(4,7,0)  # 默认假设新版
            if mv >= _ver(4,4,6):    fun_v = 5
            elif mv >= _ver(4,2,3):  fun_v = 4
            elif mv >= _ver(4,2,0):  fun_v = 2
            else:                    fun_v = 1
            print(f"  {self.port}: Step4 FUN_V={fun_v} (电机最低版本 v{mv>>12&0xF}.{(mv>>4)&0xFF}.{mv&0xF})")
            for _ in range(20):
                self._send_raw(MODE_FUN_V, struct.pack('<B', fun_v)); time.sleep(0.02)
                if self._fun_v_confirmed: break

        print(f"  {self.port}: 初始化完成")

    def close(self):
        self._running = False
        if self._rx_thread: self._rx_thread.join(timeout=0.5)
        if self._ser.is_open: self._ser.close()

    # ==================== 转换 ====================
    def _i16(self, v): return max(-32700, min(32700, int(v)))
    def _p2i(self, r): return self._i16(r / MY_2PI * 10000)
    def _v2i(self, r): return self._i16(r / MY_2PI * 4000)
    def _i2p(self, v): return v * MY_2PI / 10000.0
    def _i2v(self, v): return v * MY_2PI / 4000.0

    @staticmethod
    def _get_data_len(mode, id_max):
        m = {MODE_POSITION:2, MODE_VELOCITY:2, MODE_TORQUE:2,
             MODE_VOLTAGE:2, MODE_CURRENT:2, MODE_TIME_OUT:2,
             MODE_POS_VEL_TQE:6, MODE_POS_VEL_ACC:6,
             MODE_POS_VEL_KP_KD:8, MODE_POS_VEL_TQE_KP_KD_2:10}
        one = m.get(mode, 0); base = one * id_max
        fd = 56 if mode == MODE_POS_VEL_KP_KD else 60
        mul, rem = base // fd, base % fd
        if rem <= 6: r=rem
        elif rem <= 10: r=10
        elif rem <= 14: r=14
        elif rem <= 18: r=18
        elif rem <= 22: r=22
        elif rem <= 30: r=30
        elif rem <= 46: r=46
        else: r=fd
        return mul * fd + r

    def _send_raw(self, cmd, data):
        head = struct.pack('<B', FRAME_HEADER)
        cb = struct.pack('<B', cmd)
        lb = struct.pack('<H', len(data))
        c8 = struct.pack('<B', crc8(cb + lb))
        c16 = struct.pack('<H', crc16(data))
        with self._lock: self._ser.write(head + cb + lb + c8 + c16 + data)

    def _pad(self, cmd, payload):
        n = self._get_data_len(cmd, self.id_max) // 2
        buf = [SENTINEL_INT16] * n
        for i, v in enumerate(payload):
            if i < n: buf[i] = v
        return struct.pack('<' + 'h' * n, *buf)

    # ==================== 控制 (使用本地电机ID) ====================
    def set_pos_vel_max_torque(self, local_id_to_pvt: dict):
        """MODE_POS_VEL_TQE (0x90): 位置+速度+最大力矩, 电机内置PD"""
        payload = [SENTINEL_INT16] * (self.id_max * 3)
        for lid, (pos, vel, max_torque) in local_id_to_pvt.items():
            i = lid - 1
            payload[i*3+0] = self._p2i(pos)
            payload[i*3+1] = self._v2i(vel)
            payload[i*3+2] = self._i16(max_torque / 0.01)  # 力矩转换
        self._send_raw(MODE_POS_VEL_TQE, self._pad(MODE_POS_VEL_TQE, payload))

    def set_pos_vel_kp_kd(self, local_id_to_pvkd: dict):
        payload = [SENTINEL_INT16] * (self.id_max * 4)
        for lid, (pos, vel, kp, kd) in local_id_to_pvkd.items():
            i = lid - 1
            payload[i*4+0] = self._p2i(pos)
            payload[i*4+1] = self._v2i(vel)
            payload[i*4+2] = self._i16(kp * 10 * MY_2PI)
            payload[i*4+3] = self._i16(kd * 10 * MY_2PI)
        self._send_raw(MODE_POS_VEL_KP_KD, self._pad(MODE_POS_VEL_KP_KD, payload))

    def set_pos_vel_torque_kp_kd(self, local_id_to_pvtkd: dict):
        """MODE_POS_VEL_TQE_KP_KD_2 (0xB0): 位置+速度+前馈力矩+KP+KD"""
        payload = [SENTINEL_INT16] * (self.id_max * 5)
        for lid, (pos, vel, torque, kp, kd) in local_id_to_pvtkd.items():
            i = lid - 1
            payload[i*5+0] = self._p2i(pos)
            payload[i*5+1] = self._v2i(vel)
            payload[i*5+2] = self._i16(torque / 0.01)  # 力矩, 同 set_pos_vel_max_torque
            payload[i*5+3] = self._i16(kp * 10 * MY_2PI)
            payload[i*5+4] = self._i16(kd * 10 * MY_2PI)
        self._send_raw(MODE_POS_VEL_TQE_KP_KD_2, self._pad(MODE_POS_VEL_TQE_KP_KD_2, payload))

    def set_free_mode(self):
        self.set_pos_vel_kp_kd({mid:(0,0,0,0) for mid in self.motor_ids})

    def request_state(self):
        # 仅用 MODE_MOTOR_STATE2 (0x0A) — 电机固件 v4.7+ 全部支持
        self._send_raw(MODE_MOTOR_STATE2, b'\x7F')

    def set_stop(self):
        for _ in range(3): self._send_raw(MODE_STOP, b'\x7F')

    def get_local_state(self, local_id):
        return self._states.get(local_id, MotorState(999,0,0,0,0))

    def get_all_local_states(self):
        return dict(self._states)

    # ==================== 接收 ====================
    def _recv_loop(self):
        buf = bytearray()
        while self._running:
            try:
                if self._ser.in_waiting:
                    c = self._ser.read(self._ser.in_waiting)
                    if c: buf.extend(c)
                else: time.sleep(0.0005); continue
            except Exception: time.sleep(0.01); continue

            while len(buf) >= 7:
                idx = buf.find(FRAME_HEADER)
                if idx < 0: buf.clear(); break
                if idx > 0: del buf[:idx]
                if len(buf) < 7: break
                cmd = buf[1]; dlen = struct.unpack_from('<H',buf,2)[0]
                # 数据长度校验: 最大256字节(协议限制), 防止噪声数据导致死等
                if dlen > 256:
                    del buf[:1]; continue
                if crc8(bytes(buf[1:4])) != buf[4]: del buf[:1]; continue
                total = 7 + dlen
                if len(buf) < total: break
                if crc16(bytes(buf[7:total])) != struct.unpack_from('<H',buf,5)[0]:
                    del buf[:1]; continue
                self._parse(cmd, bytes(buf[7:total]))
                del buf[:total]

    def _parse(self, cmd, data):
        if cmd == MODE_SET_NUM and len(data) >= 4:
            self._board_version = _ver(data[2], data[3], data[4] if len(data)>=5 else 0)
        elif cmd == MODE_MOTOR_VERSION:
            for i in range(len(data)//4):
                off=i*4; mid=data[off]
                if mid in self._states:
                    self._motor_versions[mid] = (data[off+1],data[off+2],data[off+3])
        elif cmd == MODE_MOTOR_STATE:
            # 旧格式: id(1) + pos(2) + vel(2) + tqe(2) = 7 bytes
            for i in range(len(data)//7):
                off = i*7
                if off+7 > len(data): break
                mid = data[off]
                if mid in self._states:
                    p,v,t = struct.unpack_from('<hhh', data, off+1)
                    self._states[mid] = MotorState(pos=self._i2p(p), vel=self._i2v(v),
                                                   torque=float(t), mode=0, fault=0)
        elif cmd == MODE_FDCAN_MOTOR_STATE:
            # FDCAN头(3) + 电机状态(7 each): fault(1)+tx_err(1)+rx_err(1)
            offs = 3
            for i in range((len(data)-offs)//7):
                off = offs + i*7
                if off+7 > len(data): break
                mid = data[off]
                if mid in self._states:
                    p,v,t = struct.unpack_from('<hhh', data, off+1)
                    self._states[mid] = MotorState(pos=self._i2p(p), vel=self._i2v(v),
                                                   torque=float(t), mode=0, fault=0)
        elif cmd in (MODE_MOTOR_STATE2, MODE_FDCAN_MOTOR_STATE2):
            # MODE_MOTOR_STATE2: id(1)+mode(1)+fault(1)+pos(2)+vel(2)+tqe(2)=9
            # MODE_FDCAN_MOTOR_STATE2: FDCAN头(3)+电机状态(9 each)
            offs = 3 if cmd == MODE_FDCAN_MOTOR_STATE2 else 0
            for i in range((len(data)-offs)//9):
                off = offs + i*9
                if off+9 > len(data): break
                mid = data[off]
                if mid in self._states:
                    p,v,t = struct.unpack_from('<hhh',data,off+3)
                    self._states[mid] = MotorState(pos=self._i2p(p), vel=self._i2v(v),
                                                   torque=float(t), mode=data[off+1], fault=data[off+2])
        elif cmd == MODE_FUN_V:
            self._fun_v_confirmed = True


class MultiMotorManager:
    """
    多端口电机管理器 — 使用全局ID

    用法:
        mgr = MultiMotorManager({
            "/dev/ttyACM0": [1,2,3,4,5,6,7,8],   # 左手臂
            "/dev/ttyACM1": [1,2,3,4,5,6,7,8],   # 右手臂
            "/dev/ttyACM2": [1,2],                # 腰部
            "/dev/ttyACM3": [1,2],                # 头部
        })
        # 全局ID自动编排: ACM0→1-8 (左臂), ACM1→9-16 (右臂), ACM2→17-18 (腰), ACM3→19-20 (头)
    """

    def __init__(self, port_motor_map: dict):
        """
        port_motor_map: {port_path: [local_motor_id, ...]}
        按端口顺序分配全局ID
        """
        self.drivers = []
        self._gid_to_port = {}      # global_id → (driver_index, local_id)
        self._port_info = []        # [(port, driver)]
        self.total_motors = 0

        gid = 1
        for port, local_ids in port_motor_map.items():
            d = MotorDriver(port, list(local_ids))
            self.drivers.append(d)
            for lid in sorted(local_ids):
                self._gid_to_port[gid] = (len(self.drivers) - 1, lid)
                gid += 1
            self.total_motors = gid - 1
        self.global_ids = list(range(1, self.total_motors + 1))

    def _split_by_port(self, gid_to_val: dict) -> dict:
        """将 {global_id: value} 拆分为 {driver_index: {local_id: value}}"""
        by_drv = {}
        for gid, val in gid_to_val.items():
            info = self._gid_to_port.get(gid)
            if info is None: continue
            drv_idx, lid = info
            by_drv.setdefault(drv_idx, {})[lid] = val
        return by_drv

    # ==================== 初始化 ====================
    def open_all(self):
        for d in self.drivers: d.open()

    def init_all(self):
        for d in self.drivers: d.init_board()

    def close_all(self):
        for d in self.drivers: d.close()

    # ==================== 状态查询 ====================
    def request_all_states(self):
        for d in self.drivers: d.request_state()

    def get_all_states(self) -> dict:
        """返回 {global_id: MotorState}"""
        r = {}
        for gid, (drv_idx, lid) in self._gid_to_port.items():
            st = self.drivers[drv_idx].get_local_state(lid)
            r[gid] = st
        return r

    # ==================== 控制 (使用全局ID) ====================
    def set_all_free_mode(self):
        for d in self.drivers: d.set_free_mode()

    def set_all_pos_vel_max_torque(self, gid_to_pvt: dict):
        """MODE_POS_VEL_TQE: {global_id: (pos, vel, max_torque)}"""
        by_drv = self._split_by_port(gid_to_pvt)
        for drv_idx, lid_pvt in by_drv.items():
            self.drivers[drv_idx].set_pos_vel_max_torque(lid_pvt)

    def set_all_pos_vel_kp_kd(self, gid_to_pvkd: dict):
        """
        gid_to_pvkd: {global_id: (pos, vel, kp, kd)}
        """
        by_drv = self._split_by_port(gid_to_pvkd)
        for drv_idx, lid_pvkd in by_drv.items():
            self.drivers[drv_idx].set_pos_vel_kp_kd(lid_pvkd)

    def set_all_pos_vel_torque_kp_kd(self, gid_to_pvtkd: dict):
        """
        gid_to_pvtkd: {global_id: (pos, vel, torque, kp, kd)}
        MODE_POS_VEL_TQE_KP_KD_2 (0xB0) — feed-forward torque control
        """
        by_drv = self._split_by_port(gid_to_pvtkd)
        for drv_idx, lid_pvtkd in by_drv.items():
            self.drivers[drv_idx].set_pos_vel_torque_kp_kd(lid_pvtkd)

    def stop_all(self):
        for d in self.drivers: d.set_stop()


def rad_to_deg(r): return r * 180.0 / 3.141592653589793
def deg_to_rad(d): return d * 3.141592653589793 / 180.0
