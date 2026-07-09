#!/usr/bin/env python3
"""
合并两条轨迹: 躯干轨迹 + 头部/附加关节轨迹, 线性插值对齐时间轴.

用法:
    python3 merge_traj.py <body_traj.jsonl> <head_traj.jsonl> [output.jsonl]
"""
import json, sys

old_file = sys.argv[1] if len(sys.argv) > 1 else "trajectory_20260616_220111.jsonl"
new_file = sys.argv[2] if len(sys.argv) > 2 else "trajectory_20260621_161248.jsonl"
out_file = sys.argv[3] if len(sys.argv) > 3 else "trajectory_merged.jsonl"

with open(old_file) as f:
    old = [json.loads(l) for l in f if l.strip()]
with open(new_file) as f:
    new = [json.loads(l) for l in f if l.strip()]

# 从新文件中提取头部时间序列
head_t = [f['t'] for f in new]
head_19 = [f['pos_19'] for f in new]
head_20 = [f['pos_20'] for f in new]

def lerp(t_arr, v_arr, t):
    """线性插值"""
    if t <= t_arr[0]:
        return v_arr[0]
    if t >= t_arr[-1]:
        return v_arr[-1]
    for i in range(len(t_arr) - 1):
        if t_arr[i] <= t <= t_arr[i + 1]:
            frac = (t - t_arr[i]) / (t_arr[i + 1] - t_arr[i])
            return v_arr[i] + frac * (v_arr[i + 1] - v_arr[i])
    return v_arr[-1]

merged = []
for f_old in old:
    entry = {k: v for k, v in f_old.items()}
    t = f_old['t']
    # 头部时间循环 (22.4s 一循环)
    t_mod = t % head_t[-1] if head_t[-1] > 0 else t
    entry['pos_19'] = round(lerp(head_t, head_19, t_mod), 6)
    entry['vel_19'] = 0.0
    entry['pos_20'] = round(lerp(head_t, head_20, t_mod), 6)
    entry['vel_20'] = 0.0
    merged.append(entry)

with open(out_file, 'w') as f:
    for entry in merged:
        f.write(json.dumps(entry) + '\n')

print(f"合并完成: {len(merged)} 帧, {out_file}")
print(f"头部 19 范围: {min(head_19):.2f}~{max(head_19):.2f} rad")
print(f"头部 20 范围: {min(head_20):.2f}~{max(head_20):.2f} rad")
print(f"头部运动 {head_t[-1]:.1f}s 一循环")
