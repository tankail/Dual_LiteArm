#!/usr/bin/env python3
"""
示教录制程序 — 手动拖动电机, 记录位置轨迹

运行后全部电机进入自由模式(kp=kd=0), 可手动拖动.
按 Ctrl+C 停止录制, 轨迹自动保存为 .jsonl 文件.

用法:
    python3 teach_record.py
    python3 teach_record.py --output my_traj.jsonl --rate 50

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
from datetime import datetime
from motor_driver import MultiMotorManager, rad_to_deg

# 默认端口与电机ID配置 (左臂ACM0 + 右臂ACM1 + 腰部ACM2 + 头部ACM3)
DEFAULT_PORTS = "/dev/ttyACM0,/dev/ttyACM1,/dev/ttyACM2,/dev/ttyACM3"
DEFAULT_MOTOR_IDS = "1,2,3,4,5,6,7,8;1,2,3,4,5,6,7,8;1,2;1,2"


def parse_motor_config(ports_str, ids_str):
    """解析端口和电机ID配置"""
    ports = [p.strip() for p in ports_str.split(",")]
    id_groups = [[int(x.strip()) for x in g.split(",")] for g in ids_str.split(";")]
    if len(ports) != len(id_groups):
        print(f"错误: ports({len(ports)}) 与 motor_id组({len(id_groups)}) 数量不一致")
        sys.exit(1)
    port_map = {}
    for port, ids in zip(ports, id_groups):
        port_map[port] = ids
    return port_map


def main():
    parser = argparse.ArgumentParser(description="示教轨迹录制")
    parser.add_argument("--ports", default=DEFAULT_PORTS,
                        help="串口列表, 逗号分隔 (默认: /dev/ttyACM0,/dev/ttyACM1,/dev/ttyACM2,/dev/ttyACM3)")
    parser.add_argument("--motor_ids", default=DEFAULT_MOTOR_IDS,
                        help="每端口电机ID, 分号分隔各端口, 逗号分隔端口内电机")
    parser.add_argument("--output", default=None,
                        help="输出文件路径 (默认: trajectory_YYYYMMDD_HHMMSS.jsonl)")
    parser.add_argument("--rate", type=float, default=100.0,
                        help="录制频率 (Hz, 默认100)")
    parser.add_argument("--hold", type=float, default=0.0,
                        help="软保持增益 (kp), 默认0=自由模式. 建议 15-30 用于重力辅助")
    parser.add_argument("--hold_kd", type=float, default=2.0,
                        help="软保持阻尼 (kd), 默认2.0")
    args = parser.parse_args()

    port_motor_map = parse_motor_config(args.ports, args.motor_ids)

    if args.output:
        out_file = args.output
    else:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_file = f"trajectory_{ts}.jsonl"

    # 初始化
    mgr = MultiMotorManager(port_motor_map)
    total = mgr.total_motors

    print("=" * 60)
    print("示教轨迹录制 (双臂+腰+头)")
    print("=" * 60)
    for port, ids in port_motor_map.items():
        print(f"  {port} → {len(ids)}个电机 (本地ID: {ids})")
    print(f"  全局电机ID: 1-{total}")
    print(f"  录制频率: {args.rate} Hz")
    print(f"  输出文件: {out_file}")
    print()

    print("初始化电机...")
    mgr.open_all()
    mgr.init_all()

    mgr.set_all_free_mode()

    if args.hold > 0:
        # 软保持模式: 用当前姿态为基准, 加小 kp/kd, 手臂可拖动但不完全塌
        print(f"\n软保持模式 kp={args.hold}, kd={args.hold_kd} — 获取当前姿态...")
        mgr.request_all_states()
        time.sleep(0.1)
        states = mgr.get_all_states()
        # global_id → (pos, 0, kp, kd)
        targets = {}
        for gid in mgr.global_ids:
            st = states.get(gid)
            if st and abs(st.pos) < 100:
                targets[gid] = (st.pos, 0.0, args.hold, args.hold_kd)
        mgr.set_all_pos_vel_kp_kd(targets)
        print(f"  已在 {len(targets)} 个电机上设置软保持")
    else:
        print(f"\n全部 {total} 个电机已进入自由模式 (kp=kd=0), 可手动拖动")

    print()

    print("3秒后开始录制...")
    for i in range(3, 0, -1):
        print(f"  {i}...")
        time.sleep(1)
    print()

    # ==================== 录制循环 ====================
    interval = 1.0 / args.rate
    records = []
    start_time = time.time()
    running = True

    def sig_handler(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, sig_handler)

    print(f"正在录制 {total} 个电机! 拖动示教, 按 Ctrl+C 停止\n")

    # 先发一次查询预热
    mgr.request_all_states()
    time.sleep(0.05)

    try:
        while running:
            loop_start = time.time()

            # 先读上次请求的回复 (已由后台线程解析好), 再发新请求
            states = mgr.get_all_states()
            mgr.request_all_states()

            t = time.time() - start_time
            entry = {"t": round(t, 6)}
            for gid in range(1, total + 1):
                st = states.get(gid)
                if st:
                    entry[f"pos_{gid}"] = round(st.pos, 6)
                    entry[f"vel_{gid}"] = round(st.vel, 6)
            records.append(entry)

            # 每10帧显示一次
            if len(records) % 10 == 0:
                # 紧凑显示: 只显示前3个和后3个电机位置
                parts = []
                for gid in range(1, total + 1):
                    st = states.get(gid)
                    if st:
                        parts.append(f"{gid}:{rad_to_deg(st.pos):+.0f}°")
                line = " ".join(parts)
                print(f"\r[{len(records):6d}] {line}", end="", flush=True)

            elapsed = time.time() - loop_start
            if elapsed < interval:
                time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        pass

    if records:
        print(f"\n\n保存轨迹 ({len(records)} 帧) 到 {out_file} ...")
        with open(out_file, "w") as f:
            for entry in records:
                f.write(json.dumps(entry) + "\n")
        total_t = records[-1]["t"]
        print(f"完成! {len(records)} 帧, 时长 {total_t:.1f}s, "
              f"平均帧率 {len(records)/total_t:.0f}Hz")
        print(f"文件: {out_file}")
    else:
        print("\n未录制到数据")

    mgr.stop_all()
    mgr.close_all()


if __name__ == "__main__":
    main()
