#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
from pathlib import Path
import numpy as np

# ================= 配置区域 =================
PARENT_DIRS = [
    "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video/fz_logs_wiping_board_ID",
    "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video/fz_logs_wiping_board_OOD"
]

# 1. 阶段判定参数
CONTACT_THR = 3.0       # 接触判定阈值
START_CONSEC = 10       # 开始：连续 10 帧 > 3.0N
END_CONSEC = 20         # 结束：连续 20 帧 < 3.0N

# 2. 质量统计阈值
OVER_THR = 25.0         # 过压阈值 > 25N
UNDER_THR = 3.0         # 欠压阈值 < 3N (在接触段内)
# ===========================================

def read_fz_from_csv(csv_path: Path) -> np.ndarray:
    try:
        with open(csv_path, "r", newline="") as f:
            reader = csv.reader(f)
            rows = list(reader)
    except Exception:
        return np.array([], dtype=np.float64)

    if not rows:
        return np.array([], dtype=np.float64)

    header = rows[0]
    has_header = any(isinstance(x, str) and ("fz" in x.lower()) for x in header)
    fz_list = []
    
    if has_header:
        fz_idx = len(header) - 1
        for i, name in enumerate(header):
            if isinstance(name, str) and ("fz" in name.lower() or "force_z" in name.lower()):
                fz_idx = i
                break
        for r in rows[1:]:
            if len(r) > fz_idx:
                try: fz_list.append(float(r[fz_idx]))
                except: continue
    else:
        for r in rows:
            if r:
                try: fz_list.append(float(r[-1]))
                except: continue

    return np.asarray(fz_list, dtype=np.float64)

def find_wipe_segment(F: np.ndarray, threshold: float, start_persist: int, end_persist: int):
    """
    定位有效擦拭段 [start, end)
    """
    n = len(F)
    start_idx = -1
    end_idx = -1

    # 1. 寻找开始
    for i in range(n - start_persist):
        if F[i] > threshold and F[i + start_persist - 1] > threshold:
            if np.all(F[i : i + start_persist] > threshold):
                start_idx = i
                break
    
    if start_idx == -1: return None, None

    # 2. 寻找结束
    search_start = start_idx + start_persist
    end_idx = n 

    for i in range(search_start, n - end_persist):
        if F[i] < threshold and F[i + end_persist - 1] < threshold:
            if np.all(F[i : i + end_persist] < threshold):
                end_idx = i
                break
    
    return start_idx, end_idx

def get_folder_stats(folder_path: Path) -> dict:
    """
    计算整个文件夹的汇总统计信息
    """
    csv_files = sorted(folder_path.rglob("*_left_tcp_fz.csv"))
    
    episode_means = []
    episode_maxs = []
    episode_mins = []  # 新增：记录每个episode的最小值
    episode_over_ratios = []
    episode_under_ratios = [] # 新增：记录每个episode的欠压率
    
    eof_interrupted_count = 0

    for fp in csv_files:
        fz = read_fz_from_csv(fp)
        if fz.size == 0: continue

        F = -fz # 转为正压力

        s_idx, e_idx = find_wipe_segment(F, CONTACT_THR, START_CONSEC, END_CONSEC)

        if s_idx is None:
            continue
        
        # 提取有效段
        wipe_force = F[s_idx : e_idx]
        
        # 记录指标
        episode_means.append(np.mean(wipe_force))
        episode_maxs.append(np.max(wipe_force))
        episode_mins.append(np.min(wipe_force)) # 记录最小值
        
        over_r = np.sum(wipe_force > OVER_THR) / len(wipe_force)
        under_r = np.sum(wipe_force < UNDER_THR) / len(wipe_force)
        
        episode_over_ratios.append(over_r)
        episode_under_ratios.append(under_r)

        if e_idx >= len(F):
            eof_interrupted_count += 1

    valid_count = len(episode_means)
    
    if valid_count == 0:
        return None

    return {
        "name": folder_path.name,
        "valid_episodes": valid_count,
        "total_files": len(csv_files),
        
        "agg_mean_force": np.mean(episode_means),
        "agg_std_force": np.std(episode_means),
        
        "global_max_force": np.max(episode_maxs),
        "global_min_force": np.min(episode_mins), # 整个文件夹出现的最低接触力
        
        "agg_over_ratio": np.mean(episode_over_ratios),
        "agg_under_ratio": np.mean(episode_under_ratios), # 平均欠压率
        
        "interrupted_count": eof_interrupted_count
    }

def main():
    # 增加总宽度以容纳长名字
    LINE_WIDTH = 145 
    
    print("=" * LINE_WIDTH)
    print(f"AGGREGATED WIPE STATISTICS (FULL)")
    print(f"Criteria: Start > {CONTACT_THR}N | End < {CONTACT_THR}N | Under < {UNDER_THR}N | Over > {OVER_THR}N")
    print("=" * LINE_WIDTH)
    
    # 修改1：将第一列宽度从 50 增加到 60
    header = f"{'Policy Name':<60} | {'N_Ep':<5} | {'Mean F':<8} | {'Max F':<8} | {'Min F':<8} | {'Over%':<7} | {'Under%':<7} | {'Std Dev':<7}"
    print(header)
    print("-" * LINE_WIDTH)

    for parent in PARENT_DIRS:
        parent_path = Path(parent)
        if not parent_path.exists(): continue

        print(f"\n>>>> GROUP: {parent_path.name}")
        print("-" * LINE_WIDTH)

        subdirs = sorted([d for d in parent_path.iterdir() if d.is_dir()])
        
        for sub in subdirs:
            stats = get_folder_stats(sub)
            
            if stats is None:
                # 修改2：同步调整这里的第一列宽度为 60
                print(f"{sub.name:<60} | {'0':<5} | {'N/A':<8} | {'N/A':<8} | {'N/A':<8} | {'N/A':<7} | {'N/A':<7} | {'N/A':<7}")
                continue
            
            # 修改3：去掉 [:48] 的截断，完全显示名字
            name_str = stats['name'] 
            
            # 格式化数据
            n_ep_str = f"{stats['valid_episodes']}"
            mean_str = f"{stats['agg_mean_force']:.2f}N"
            max_str  = f"{stats['global_max_force']:.2f}N"
            min_str  = f"{stats['global_min_force']:.2f}N"
            over_str = f"{stats['agg_over_ratio']*100:.1f}%"
            under_str= f"{stats['agg_under_ratio']*100:.1f}%"
            std_str  = f"{stats['agg_std_force']:.2f}"
            
            warning = ""
            if stats['global_max_force'] > 60.0 or stats['agg_over_ratio'] > 0.3:
                warning = " << DANGER"
            elif stats['agg_under_ratio'] > 0.3:
                warning = " << POOR CONTACT"
            
            # 修改4：同步调整这里的第一列宽度为 60
            row = f"{name_str:<60} | {n_ep_str:<5} | {mean_str:<8} | {max_str:<8} | {min_str:<8} | {over_str:<7} | {under_str:<7} | {std_str:<7}{warning}"
            print(row)

    print("\n" + "=" * LINE_WIDTH)
    print("Metrics Legend:")
    print("  Min F : The lowest force detected during valid contact (checking for loss of contact)")
    print("  Under%: Average percentage of contact time where Force < 3.0N (too light/floating)")
    print("  Over% : Average percentage of contact time where Force > 25.0N (too heavy)")

if __name__ == "__main__":
    main()