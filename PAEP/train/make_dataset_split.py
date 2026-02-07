import json
import zarr
import numpy as np
import os

# ================= 配置区域 =================
# Zarr 文件路径 (请修改为你实际的 replay_buffer.zarr 路径)
ZARR_PATH = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_stream_downsample1_zarr/replay_buffer.zarr'

# 输出 JSON 文件名
OUT_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/config/paep_split_plugin_charger.json"

# 随机种子
SEED = 0

# 是否打乱顺序 (建议 True，除非你想按时间顺序切分)
SHUFFLE = True
# ===========================================

def main():
    # 1. 检查路径是否存在
    if not os.path.exists(ZARR_PATH):
        print(f"【错误】找不到路径: {ZARR_PATH}")
        return

    print(f"正在读取 Zarr: {ZARR_PATH}")
    
    # 2. 打开 Zarr 并读取 Episode 数量
    try:
        root = zarr.open_group(ZARR_PATH, mode="r")
        ends = np.array(root["meta/episode_ends"][:], dtype=np.int64)
        n_ep = len(ends)
        ep_ids = np.arange(n_ep, dtype=np.int64)
        print(f"总 Episode 数量: {n_ep}")
    except Exception as e:
        print(f"读取 Zarr 出错: {e}")
        return

    # 3. 打乱顺序
    if SHUFFLE:
        rng = np.random.default_rng(SEED)
        rng.shuffle(ep_ids)
        print(f"已打乱顺序 (Seed: {SEED})")

    # 4. 计算切分比例
    assert n_ep >= 3, f"need >=3 episodes, got {n_ep}"

    if n_ep == 50:
        n_train, n_val, n_test = 40, 5, 5
        print("检测到 50 条数据，使用硬编码切分: 40/5/5")
    else:
        n_train = int(round(0.8 * n_ep))
        n_val = int(round(0.1 * n_ep))
        n_test = n_ep - n_train - n_val
        
        # 保底逻辑：确保每个集至少有1条数据
        if n_val < 1: n_val = 1
        if n_test < 1: n_test = 1
        # 如果 val/test 抢占了名额，从 train 里扣除，但通常 train 应该够多
        if n_train < 1: n_train = n_ep - n_val - n_test
        
        print(f"按比例切分 (0.8/0.1/0.1): {n_train}/{n_val}/{n_test}")

    # 5. 执行切分
    train = ep_ids[:n_train].tolist()
    val   = ep_ids[n_train:n_train+n_val].tolist()
    test  = ep_ids[n_train+n_val:n_train+n_val+n_test].tolist()

    # 6. 保存结果
    out_data = {
        "train": train, 
        "val": val, 
        "test": test, 
        "num_episodes": int(n_ep)
    }
    
    with open(OUT_PATH, "w", encoding="utf-8") as f:
        json.dump(out_data, f, indent=2)

    print("-" * 30)
    print(f"成功写入文件: {OUT_PATH}")
    print(f"Train 数量: {len(train)}")
    print(f"Val   数量: {len(val)}")
    print(f"Test  数量: {len(test)}")
    print("-" * 30)
    print("Train IDs (前10个):", train[:10], "..." if len(train)>10 else "")
    print("Val IDs:", val)
    print("Test IDs:", test)

if __name__ == "__main__":
    main()