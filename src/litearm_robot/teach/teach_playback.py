#!/usr/bin/env python3
"""
示教轨迹循环回放程序 — 来回跑 (ping-pong)

回放逻辑:
  起点 → 终点 → 反向播放 → 起点 → 正向播放 → ...  无限循环
  到达端点时不跳回, 而是原路反向, 平滑连续.

用法:
    python3 teach_playback.py trajectory_xxx.jsonl [选项]

选项:
    --kp 3.0       位置比例增益 (越大跟踪越紧, 过大振荡)
    --kd 0.3       速度阻尼增益
    --speed 1.0    播放速度倍率
    --loop 0       循环次数 (0=无限)

默认配置 (双臂+腰+头):
    /dev/ttyACM0 → 左手臂 8电机 (全局ID 1-8)
    /dev/ttyACM1 → 右手臂 8电机 (全局ID 9-16)
    /dev/ttyACM2 → 腰部    2电机 (全局ID 17-18)
    /dev/ttyACM3 → 头部    2电机 (全局ID 19-20)
"""

import sys
import time
import json
import argparse
import signal
from motor_driver import MultiMotorManager, rad_to_deg

# 默认端口与电机ID配置 (左臂ACM0 + 右臂ACM1 + 腰部ACM2 + 头部ACM3)
DEFAULT_PORTS = "/dev/ttyACM0,/dev/ttyACM1,/dev/ttyACM2,/dev/ttyACM3"
DEFAULT_MOTOR_IDS = "1,2,3,4,5,6,7,8;1,2,3,4,5,6,7,8;1,2;1,2"


def parse_motor_config(ports_str, ids_str):
    ports = [p.strip() for p in ports_str.split(",")]
    id_groups = [[int(x.strip()) for x in g.split(",")] for g in ids_str.split(";")]
    if len(ports) != len(id_groups):
        print(f"错误: ports({len(ports)}) 与 motor_id组({len(id_groups)}) 数量不一致")
        sys.exit(1)
    return {port: ids for port, ids in zip(ports, id_groups)}


def load_trajectory(filepath):
    records = []
    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    if len(records) < 2:
        print("错误: 轨迹至少需要2帧")
        sys.exit(1)
    return records


def detect_active_motors(records, global_ids):
    """从轨迹中检测哪些电机实际有数据 (pos != 999)"""
    active = set()
    for rec in records[:100]:  # 检查前100帧
        for gid in global_ids:
            v = rec.get(f"pos_{gid}", 999.0)
            if abs(v) < 100.0:  # 合理范围 (-100~100 rad)
                active.add(gid)
    return sorted(active)


def main():
    parser = argparse.ArgumentParser(description="示教轨迹循环回放 (双臂+腰+头, 来回跑)")
    parser.add_argument("trajectory", help="轨迹JSONL文件")
    parser.add_argument("--ports", default=DEFAULT_PORTS,
                        help="串口列表, 逗号分隔 (默认: /dev/ttyACM0,/dev/ttyACM1,/dev/ttyACM2,/dev/ttyACM3)")
    parser.add_argument("--motor_ids", default=DEFAULT_MOTOR_IDS,
                        help="每端口电机ID, 分号分隔各端口, 逗号分隔端口内电机")
    parser.add_argument("--loop", type=int, default=0, help="循环次数 (0=无限)")
    parser.add_argument("--kp", type=float, default=3.0, help="位置比例增益")
    parser.add_argument("--kd", type=float, default=0.3, help="速度阻尼增益")
    parser.add_argument("--speed", type=float, default=1.0, help="播放速度倍率")
    args = parser.parse_args()

    port_motor_map = parse_motor_config(args.ports, args.motor_ids)
    records = load_trajectory(args.trajectory)
    N = len(records)
    avg_dt = records[-1]["t"] / (N - 1) if N > 1 else 0.02

    # 初始化
    mgr = MultiMotorManager(port_motor_map)
    total = mgr.total_motors
    all_ids = mgr.global_ids

    # 自动检测活跃电机 (跳过断联的)
    active_ids = detect_active_motors(records, all_ids)
    if not active_ids:
        print("错误: 未检测到任何活跃电机 (所有pos=999)")
        sys.exit(1)

    print("=" * 60)
    print("示教轨迹循环回放 (双臂+腰+头, ping-pong)")
    print("=" * 60)
    print(f"  轨迹文件: {args.trajectory}")
    print(f"  帧数: {N},  总时长: {records[-1]['t']:.2f}s,  帧间隔: {avg_dt*1000:.1f}ms")
    print(f"  全局电机ID: 1-{total},  活跃: {active_ids} ({len(active_ids)}个)")
    if len(active_ids) < total:
        print(f"  跳过断联: {sorted(set(all_ids) - set(active_ids))}")
    print(f"  KP={args.kp},  KD={args.kd},  速度={args.speed}x")
    print(f"  循环: {'无限' if args.loop==0 else args.loop}次")

    print("\n初始化电机...")
    mgr.open_all()
    mgr.init_all()

    # 移到起点 (用 Joint_Pos_Vel 模式 = MODE_POS_VEL_TQE)
    print("移动到轨迹起点...")
    rec0 = records[0]
    mgr.set_all_pos_vel_max_torque({
        gid: (rec0.get(f"pos_{gid}", 0.0), 0.3, 15.0)
        for gid in active_ids
    })
    time.sleep(2.0)

    running = [True]
    def sig_handler(signum, frame):
        running[0] = False
        print("\n正在停止...")
    signal.signal(signal.SIGINT, sig_handler)

    print("\n开始回放! Ctrl+C 停止\n")

    # ==================== ping-pong 回放 ====================
    idx = 0
    direction = 1
    loop_count = 0
    max_loops = args.loop if args.loop > 0 else float("inf")
    speed = args.speed
    dt = avg_dt / speed
    elapsed = 0.0       # 当前段已播放的时间
    t0 = time.time()

    try:
        while running[0] and loop_count < max_loops:
            rec = records[idx]

            # 对齐 SDK: 用 MODE_POS_VEL_TQE (内置 PD + 力矩限制)
            mgr.set_all_pos_vel_max_torque({
                gid: (rec.get(f"pos_{gid}", 0.0), rec.get(f"vel_{gid}", 0.0), 15.0)
                for gid in active_ids
            })

            # 每10帧显示
            if idx % 10 == 0:
                arrow = "→" if direction > 0 else "←"
                parts = []
                for gid in active_ids:
                    d = rad_to_deg(rec.get(f"pos_{gid}", 0.0))
                    parts.append(f"{gid}:{d:+.0f}°")
                print(f"\r[{arrow}] {idx:4d}/{N} " + " ".join(parts), end="", flush=True)

            # 更新帧索引
            idx += direction
            if idx >= N:
                direction = -1; idx = N - 2
                elapsed = 0.0; t0 = time.time()  # 重置段计时
                loop_count += 1
                if loop_count < max_loops:
                    print(f"\n--- 往返#{loop_count} 反向 ---")
            elif idx < 0:
                direction = 1; idx = 1
                elapsed = 0.0; t0 = time.time()
                loop_count += 1
                if loop_count < max_loops:
                    print(f"\n--- 往返#{loop_count} 正向 ---")

            # 按固定帧间隔等待
            elapsed += dt
            target_time = t0 + elapsed
            while time.time() < target_time:
                time.sleep(0.0005)

    except KeyboardInterrupt:
        pass

    print("\n停止电机...")
    mgr.stop_all()
    mgr.close_all()
    print(f"回放结束, 共 {loop_count} 个往返")


if __name__ == "__main__":
    main()
