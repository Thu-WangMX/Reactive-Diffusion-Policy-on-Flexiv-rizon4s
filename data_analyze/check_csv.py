#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import os
import sys
from pathlib import Path
import numpy as np

# ================= 配置区域 =================
ROOTS = [
    "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video/fz_logs_wiping_board_ID",
    "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video/fz_logs_wiping_board_OOD"
]

# 阈值：如果全程最大压紧力小于 5N，则视为无效文件
INVALID_THR = 10.0 
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
    # 尝试智能寻找列索引
    has_header = any(isinstance(x, str) and ("fz" in x.lower()) for x in header)
    fz_list = []
    
    if has_header:
        fz_idx = len(header) - 1 # 默认最后一列
        for i, name in enumerate(header):
            if isinstance(name, str) and ("fz" in name.lower() or "force_z" in name.lower()):
                fz_idx = i
                break
        for r in rows[1:]:
            if len(r) > fz_idx:
                try: fz_list.append(float(r[fz_idx]))
                except: continue
    else:
        # 无表头则取最后一列
        for r in rows:
            if r:
                try: fz_list.append(float(r[-1]))
                except: continue

    return np.asarray(fz_list, dtype=np.float64)

def main():
    print("=" * 80)
    print(f"[CLEANUP TOOL] Criteria: Delete file if max(-Fz) < {INVALID_THR} N")
    print("=" * 80)

    files_to_delete = []

    for root_str in ROOTS:
        root_path = Path(root_str)
        if not root_path.exists():
            print(f"[Skipping] Path not found: {root_str}")
            continue

        print(f"\nScanning: {root_str} ...")
        csv_files = sorted(root_path.rglob("*_left_tcp_fz.csv"))
        
        for fp in csv_files:
            fz = read_fz_from_csv(fp)
            if fz.size == 0:
                print(f"  [Empty/Bad] {fp.name}")
                # 空文件也可以视为无效，视情况是否加入删除列表
                continue

            # 计算压紧力 F = -Fz (假设传感器受压为负值)
            # 如果您的传感器受压为正，请改为 F = fz
            F = -fz
            max_force = np.max(F)

            # 判断条件：最大力小于阈值
            if max_force < INVALID_THR:
                files_to_delete.append((fp, max_force))

    # --- 汇总结果 ---
    print("\n" + "=" * 80)
    print(f"SCAN COMPLETE. Found {len(files_to_delete)} invalid files (Max Force < {INVALID_THR}N).")
    
    if len(files_to_delete) == 0:
        print("Good news: All scanned files contain valid contact data.")
        return

    print("-" * 80)
    print("Files marked for deletion:")
    for fp, max_f in files_to_delete:
        print(f"  [Max: {max_f:5.2f}N] {fp}")
    print("-" * 80)

    # --- 删除确认环节 ---
    confirm = input(f"\n>>> WARNING: Are you sure you want to DELETE these {len(files_to_delete)} files? (yes/no): ").strip().lower()
    
    if confirm == "yes":
        deleted_count = 0
        for fp, _ in files_to_delete:
            try:
                os.remove(fp)
                print(f"Deleted: {fp.name}")
                deleted_count += 1
            except Exception as e:
                print(f"Failed to delete {fp.name}: {e}")
        print(f"\nSuccessfully deleted {deleted_count} files.")
    else:
        print("\nOperation cancelled. No files were deleted.")

if __name__ == "__main__":
    main()