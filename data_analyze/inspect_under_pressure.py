#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import numpy as np
from pathlib import Path

# ================= 目标配置 =================
TARGET_FOLDER = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video/fz_logs_wiping_board_OOD/wmx_real_image_dp_absolute_12fps"

# 判定参数 (保持一致)
CONTACT_THR = 3.0
START_CONSEC = 10
END_CONSEC = 20
UNDER_THR = 3.0
# ===========================================

def read_fz_from_csv(csv_path: Path) -> np.ndarray:
    try:
        with open(csv_path, "r", newline="") as f:
            reader = csv.reader(f)
            rows = list(reader)
    except: return np.array([])
    if not rows: return np.array([])
    header = rows[0]
    fz_idx = len(header) - 1
    has_header = any("fz" in str(x).lower() for x in header)
    if has_header:
        for i, x in enumerate(header):
            if "fz" in str(x).lower():
                fz_idx = i; break
        data = []
        for r in rows[1:]:
            if len(r) > fz_idx:
                try: data.append(float(r[fz_idx]))
                except: continue
    else:
        data = []
        for r in rows:
            if r:
                try: data.append(float(r[-1]))
                except: continue
    return np.asarray(data, dtype=np.float64)

def find_wipe_segment(F: np.ndarray):
    n = len(F)
    start_idx = -1
    # 找开始
    for i in range(n - START_CONSEC):
        if np.all(F[i : i + START_CONSEC] > CONTACT_THR):
            start_idx = i
            break
    if start_idx == -1: return None, None
    # 找结束
    end_idx = n
    for i in range(start_idx + START_CONSEC, n - END_CONSEC):
        if np.all(F[i : i + END_CONSEC] < CONTACT_THR):
            end_idx = i
            break
    return start_idx, end_idx

def main():
    folder = Path(TARGET_FOLDER)
    if not folder.exists():
        print(f"Error: Folder not found: {folder}")
        return

    print(f"Inspecting: {folder.name}")
    print(f"Looking for UNDER-PRESSURE (< {UNDER_THR}N) frames inside valid wipe segments...\n")

    files = sorted(folder.rglob("*_left_tcp_fz.csv"))
    
    for fp in files:
        fz = read_fz_from_csv(fp)
        if fz.size == 0: continue
        
        F = -fz # 假设向下为负
        s, e = find_wipe_segment(F)
        
        if s is None:
            print(f"[SKIP] {fp.name} - No valid wipe detected.")
            continue

        # 提取有效段
        wipe_segment = F[s:e]
        
        # 找出欠压的索引 (相对于 wipe_segment)
        under_indices = np.where(wipe_segment < UNDER_THR)[0]
        
        print("="*80)
        print(f"FILE: {fp.name}")
        print(f"  Total Frames: {len(F)}")
        print(f"  Wipe Segment: Frame {s} -> {e} (Duration: {e-s} frames)")
        print(f"  Max Force   : {np.max(wipe_segment):.2f} N")
        print(f"  Min Force   : {np.min(wipe_segment):.2f} N")
        print("-" * 80)
        
        if len(under_indices) == 0:
            print("  Result: PERFECT CONTACT (No frames < 3.0N)")
        else:
            print(f"  Result: FOUND {len(under_indices)} UNDER-PRESSURE FRAMES ({len(under_indices)/len(wipe_segment)*100:.1f}%)")
            print("  Details:")
            
            # 将连续的帧合并显示
            if len(under_indices) > 0:
                groups = []
                # 简单的连续分组算法
                if len(under_indices) > 0:
                    current_group = [under_indices[0]]
                    for i in range(1, len(under_indices)):
                        if under_indices[i] == under_indices[i-1] + 1:
                            current_group.append(under_indices[i])
                        else:
                            groups.append(current_group)
                            current_group = [under_indices[i]]
                    groups.append(current_group)

                for grp in groups:
                    # 转换为全局帧号
                    g_start = s + grp[0]
                    g_end   = s + grp[-1]
                    
                    # 取这段的力值
                    vals = F[g_start : g_end+1]
                    min_v = np.min(vals)
                    avg_v = np.mean(vals)
                    
                    print(f"    - Frame {g_start:4d} to {g_end:4d} (Len {len(grp):3d}): Avg={avg_v:5.2f}N | Min={min_v:5.2f}N")

    print("\nDone.")

if __name__ == "__main__":
    main()