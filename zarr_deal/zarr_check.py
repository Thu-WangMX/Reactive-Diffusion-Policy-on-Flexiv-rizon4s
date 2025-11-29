#!/usr/bin/env python
import numpy as np
import zarr
from pathlib import Path

# ===== 根据你的实际路径改这里 =====
ZARR_DIR = Path("/root/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_stream_downsample1_zarr")
ZARR_PATH = ZARR_DIR / "replay_buffer.zarr"
# =================================

print(f"Loading zarr from: {ZARR_PATH}")
root = zarr.open(ZARR_PATH, mode="r")
if "data" in root:
    data = root["data"]
else:
    data = root  # 以防结构稍微不一样

print("Available keys:", list(data.array_keys()))

# 这些是 float 数组，重点查 NaN / Inf
FLOAT_KEYS = [
    "action",
    "left_robot_gripper_width",
    "left_robot_q",
    "left_robot_tau",
    "left_robot_tau_ext",
    "left_robot_tcp_pose",
    "left_robot_tcp_vel",
    "left_robot_tcp_wrench",
    "target",
    "timestamp",
]

# 这些是 uint8 图像，NaN/Inf 不会有，但看一下 min/max
IMAGE_KEYS = [
    "external_img",
    "left_wrist_img",
]

def check_float_array(name, arr):
    print(f"\n==== [FLOAT] {name} ====")
    print("shape:", arr.shape, "dtype:", arr.dtype)

    has_nan = np.isnan(arr).any()
    has_inf = np.isinf(arr).any()
    print("has_nan:", has_nan)
    print("has_inf:", has_inf)

    # 为了防止全是 NaN/Inf，先屏蔽掉再算 min/max
    arr_finite = arr[np.isfinite(arr)]
    if arr_finite.size > 0:
        print("min (finite only):", arr_finite.min())
        print("max (finite only):", arr_finite.max())
    else:
        print("WARNING: all values are NaN/Inf!")

def check_image_array(name, arr):
    print(f"\n==== [IMAGE] {name} ====")
    print("shape:", arr.shape, "dtype:", arr.dtype)
    print("min:", arr.min(), "max:", arr.max())

# 逐个 key 检查
for k in FLOAT_KEYS:
    if k not in data.array_keys():
        print(f"\n[WARN] float key '{k}' not found in zarr, skip.")
        continue
    arr = data[k][:]
    if arr.dtype.kind not in ("f", "i", "u"):  # float / int / uint
        print(f"[WARN] '{k}' dtype={arr.dtype}, not numeric? skip detailed check.")
        continue
    check_float_array(k, arr)

for k in IMAGE_KEYS:
    if k not in data.array_keys():
        print(f"\n[WARN] image key '{k}' not found in zarr, skip.")
        continue
    arr = data[k][:]
    check_image_array(k, arr)

print("\n=== DONE sanity check ===")
