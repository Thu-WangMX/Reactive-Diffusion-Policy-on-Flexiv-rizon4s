import os
import sys
import torch
import numpy as np

# 尝试导入同目录下的 paep_dataset
# 如果报错，请确保此脚本和 paep_dataset.py 在同一目录，或者将上级目录加入 PYTHONPATH
try:
    from paep_dataset import PAEPFutureDataset
except ImportError:
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from paep_dataset import PAEPFutureDataset

# ================= 配置区域 =================
# 1. Zarr 路径 (参考自 make_dataset_split.py)
ZARR_PATH = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_usb/plug_in_usb_stream_downsample1_zarr/replay_buffer.zarr'

# 2. Split JSON 路径 (参考自 make_dataset_split.py)
SPLIT_JSON = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/config/paep_split_plugin_usb.json"

# 3. Phase 名称
# 注意：paep_dataset.py 默认是 ["approach", "progress", "done"] (3类)。
# 但 plug_in_usb 任务通常有 4 类。如果不指定，遇到 label=3 时会报错。
# 这里默认填入常用的 4 类，请根据实际情况修改。
PHASE_NAMES = ["approach", "search", "recovery", "insert"]
# ===========================================

def main():
    if not os.path.exists(ZARR_PATH):
        print(f"❌ 错误: Zarr 文件不存在: {ZARR_PATH}")
        return
    if not os.path.exists(SPLIT_JSON):
        print(f"❌ 错误: Split JSON 不存在: {SPLIT_JSON}")
        return

    print(f"数据源: {ZARR_PATH}")
    print(f"Split : {SPLIT_JSON}")
    print("-" * 60)
    print(f"{'Split Name':<10} | {'Phase Name':<15} | {'Count':<10} | {'Ratio':<10}")
    print("-" * 60)

    splits_to_check = ['train', 'val', 'test']

    for split in splits_to_check:
        try:
            # 实例化 Dataset
            # 我们只需要统计数量，不需要加载图像增强(use_img_aug=False)或计算归一化(compute_norm=False)
            # force_hist/delta 等参数设为最小有效值即可，只影响有效区间的边缘裁剪
            ds = PAEPFutureDataset(
                zarr_path=ZARR_PATH,
                split_json=SPLIT_JSON,
                split=split,
                force_hist=1,    # 最小占位
                delta=1,         # 最小占位
                future_k=1,      # 最小占位
                phase_names=PHASE_NAMES,
                compute_norm=False, # 关闭归一化计算，秒开
                use_img_aug=False,
                transition_sampling=False
            )
            
            # PAEPFutureDataset 在初始化时已经计算好了 class_counts
            counts = ds.class_counts
            total = sum(counts.values())

            # 打印结果
            first_line = True
            for name in PHASE_NAMES:
                count = counts.get(name, 0)
                ratio = (count / total * 100) if total > 0 else 0.0
                
                prefix = split.upper() if first_line else ""
                print(f"{prefix:<10} | {name:<15} | {count:<10} | {ratio:.2f}%")
                first_line = False
            
            print(f"{'':<10} | {'TOTAL':<15} | {total:<10} | 100.00%")
            print("-" * 60)

        except KeyError as e:
            # 比如 JSON 里没有 'test' 字段
            print(f"{split.upper():<10} | ⚠️  Skipped (KeyError: {e})")
            print("-" * 60)
        except Exception as e:
            print(f"{split.upper():<10} | ❌ Error: {e}")
            print("-" * 60)

if __name__ == "__main__":
    main()