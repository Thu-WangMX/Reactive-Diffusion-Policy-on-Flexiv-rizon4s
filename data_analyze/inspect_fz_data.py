import numpy as np
import pandas as pd
import json
import os

# 定义文件路径
base_path = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video/fz_logs/wmx_real_image_dp_absolute_12fps/20260123_183409"
npy_path = os.path.join(base_path, "episode_000_left_tcp_fz.npy")
json_path = os.path.join(base_path, "episode_000_left_tcp_fz_meta.json")
csv_path = os.path.join(base_path, "episode_000_left_tcp_fz.csv")

print("="*40)
print("1. 检查 NPY 文件")
if os.path.exists(npy_path):
    try:
        data_npy = np.load(npy_path)
        print(f"Shape: {data_npy.shape}")
        print(f"Dtype: {data_npy.dtype}")
        print(f"前 5 个数据: {data_npy.flatten()[:5]}")
        print(f"最大值: {np.max(data_npy)}, 最小值: {np.min(data_npy)}")
    except Exception as e:
        print(f"读取失败: {e}")
else:
    print("文件不存在")

print("\n" + "="*40)
print("2. 检查 JSON 元数据")
if os.path.exists(json_path):
    try:
        with open(json_path, 'r') as f:
            meta = json.load(f)
        print(json.dumps(meta, indent=4, ensure_ascii=False))
    except Exception as e:
        print(f"读取失败: {e}")
else:
    print("文件不存在")

print("\n" + "="*40)
print("3. 检查 CSV 文件")
if os.path.exists(csv_path):
    try:
        # 尝试读取前几行，不假设有 header
        df = pd.read_csv(csv_path, header=None, nrows=5)
        print("CSV 前 5 行 (header=None):")
        print(df)
        print("-" * 20)
        # 再次尝试以第一行为 header 读取，看看是否合理
        df_head = pd.read_csv(csv_path, nrows=5)
        print("CSV 前 5 行 (header='infer'):")
        print(df_head.columns.tolist())
    except Exception as e:
        print(f"读取失败: {e}")
else:
    print("文件不存在")
print("="*40)