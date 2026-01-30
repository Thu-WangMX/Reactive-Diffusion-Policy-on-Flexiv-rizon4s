#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import zarr

def list_arrays(g: zarr.Group, prefix=""):
    """递归列出所有 array 的路径"""
    out = []
    for k, v in g.items():
        path = f"{prefix}/{k}" if prefix else k
        if isinstance(v, zarr.Array):
            out.append(path)
        elif isinstance(v, zarr.Group):
            out.extend(list_arrays(v, path))
    return out

def load_array(root: zarr.Group, path: str):
    """兼容 'a/b/c' 这种路径"""
    node = root
    parts = [p for p in path.split("/") if p]
    for p in parts[:-1]:
        node = node[p]
    return node[parts[-1]]

def basic_stats(x: np.ndarray, name: str, sample_n=200000):
    """对大数组做抽样统计，避免爆内存/太慢"""
    x_flat = x.reshape(-1)
    n = x_flat.size
    if n == 0:
        return f"[{name}] empty"

    if n > sample_n:
        idx = np.random.choice(n, size=sample_n, replace=False)
        s = x_flat[idx]
        sampled = True
    else:
        s = x_flat
        sampled = False

    # 尽量转成 float 做 min/max（int/uint 也 OK）
    s_float = s.astype(np.float32, copy=False) if s.dtype != np.float32 else s
    uniq = None
    # unique 只对“少量离散值”才有意义，太多就不算了
    if s.size <= 200000:
        u = np.unique(s)
        if u.size <= 50:
            uniq = u

    msg = []
    msg.append(f"[{name}] shape={x.shape} dtype={x.dtype} sampled={sampled} (n={n})")
    msg.append(f"  min={float(np.nanmin(s_float)):.6g} max={float(np.nanmax(s_float)):.6g} "
               f"mean={float(np.nanmean(s_float)):.6g}")
    if uniq is not None:
        msg.append(f"  unique({uniq.size})={uniq}")
    else:
        msg.append(f"  unique(>50 or skipped)")

    return "\n".join(msg)

def judge_binary_or_prob(x: np.ndarray, name: str):
    """
    判断：更像 binary(one-hot元素) / 概率 / 离散id
    """
    # 抽样
    x_flat = x.reshape(-1)
    if x_flat.size == 0:
        return f"[{name}] empty"

    sample_n = min(200000, x_flat.size)
    idx = np.random.choice(x_flat.size, size=sample_n, replace=False) if x_flat.size > sample_n else None
    s = x_flat[idx] if idx is not None else x_flat

    # 如果是浮点，看看是否接近 0/1
    if np.issubdtype(s.dtype, np.floating):
        s_float = s.astype(np.float32, copy=False)
        # 统计落在 {0,1} 的占比（允许极小数值误差）
        is_01 = np.isclose(s_float, 0.0, atol=1e-6) | np.isclose(s_float, 1.0, atol=1e-6)
        ratio_01 = float(np.mean(is_01))
        # 看看是否明显出现中间值
        has_mid = float(np.mean((s_float > 1e-3) & (s_float < 1 - 1e-3)))
        if ratio_01 > 0.999 and has_mid < 1e-4:
            return f"[{name}] ✅ 看起来是 0/1（二值或 one-hot 元素）(01_ratio={ratio_01:.4f})"
        else:
            # 再判断是否像概率（大多在[0,1]）
            in_01 = float(np.mean((s_float >= -1e-3) & (s_float <= 1 + 1e-3)))
            if in_01 > 0.99:
                return f"[{name}] ✅ 更像连续概率（float，主要在[0,1]，且不止0/1）(in01={in_01:.4f}, 01_ratio={ratio_01:.4f})"
            return f"[{name}] ⚠️ float，但不像标准概率/one-hot（可能是未归一化logit或别的量）"

    # 整型：更可能是离散 id 或二值 mask
    if np.issubdtype(s.dtype, np.integer):
        u = np.unique(s)
        if u.size <= 5 and set(u.tolist()).issubset({0, 1}):
            return f"[{name}] ✅ int 且只有0/1：二值（contact 很可能这样存）"
        else:
            # 可能是 phase_id
            return f"[{name}] ✅ int 且多取值：更像离散类别 id（phase_id 一类）unique_count~{u.size} (sample)"

    return f"[{name}] ⚠️ dtype={s.dtype} 不常见"

def judge_phase_format(x: np.ndarray, name="paep_phase"):
    """
    专门判断 phase：one-hot / 概率向量 / 离散id
    """
    shape = x.shape
    if x.ndim == 1:
        return f"[{name}] 1D：更像 phase_id（离散类别）"
    if x.ndim >= 2:
        C = shape[-1]
        # 抽样一些行看最后一维的性质
        x2 = x.reshape(-1, C)
        n = x2.shape[0]
        sample_n = min(50000, n)
        idx = np.random.choice(n, size=sample_n, replace=False) if n > sample_n else None
        s = x2[idx] if idx is not None else x2

        if np.issubdtype(s.dtype, np.floating):
            s_float = s.astype(np.float32, copy=False)
            row_sum = np.sum(s_float, axis=1)
            sum_close1 = float(np.mean(np.isclose(row_sum, 1.0, atol=1e-3)))
            # one-hot：每行最大值接近1，且非最大值接近0
            maxv = np.max(s_float, axis=1)
            # 统计“接近 one-hot”的比例
            near_one = np.isclose(maxv, 1.0, atol=1e-3)
            # 每行有多少个 > 0.5
            cnt_gt05 = np.sum(s_float > 0.5, axis=1)
            onehot_like = float(np.mean(near_one & (cnt_gt05 == 1) & (np.isclose(row_sum, 1.0, atol=1e-3))))

            if onehot_like > 0.95:
                return f"[{name}] ✅ 更像 one-hot（每行一个1，其余0）(onehot_like={onehot_like:.3f})"
            if sum_close1 > 0.95:
                return f"[{name}] ✅ 更像概率分布向量（每行和≈1，但不一定one-hot）(sum≈1 ratio={sum_close1:.3f})"
            # 也可能是 logits
            return f"[{name}] ⚠️ float 向量，但行和不≈1：可能是 logits 或未归一化分数 (sum≈1 ratio={sum_close1:.3f})"

        if np.issubdtype(s.dtype, np.integer):
            # 整型向量：有可能是 one-hot(0/1) 或多热
            u = np.unique(s)
            if u.size <= 5 and set(u.tolist()).issubset({0, 1}):
                # 检查每行是否恰好一个1
                row_sum = np.sum(s, axis=1)
                one1 = float(np.mean(row_sum == 1))
                if one1 > 0.95:
                    return f"[{name}] ✅ int one-hot（0/1，且每行恰好一个1）(ratio={one1:.3f})"
                return f"[{name}] ✅ int 0/1 向量，但不一定严格one-hot（可能多热或含padding）"
            return f"[{name}] ✅ int 向量（不常见），需要进一步看语义"

    return f"[{name}] ⚠️ 无法判断"

def main():
    base_dir = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr"
    zarr_path = base_dir
    if os.path.isdir(base_dir) and not base_dir.endswith(".zarr"):
        # 你的习惯一般是 dataset_dir/replay_buffer.zarr
        cand = os.path.join(base_dir, "replay_buffer.zarr")
        if os.path.exists(cand):
            zarr_path = cand

    print("[INFO] zarr_path =", zarr_path)
    root = zarr.open_group(zarr_path, mode="r")

    # 列出有哪些数组，方便你确认实际key叫啥
    all_arrays = list_arrays(root)
    print("[INFO] arrays in zarr (first 80):")
    for p in all_arrays[:80]:
        print("  -", p)
    if len(all_arrays) > 80:
        print(f"  ... ({len(all_arrays)} arrays total)")

    # 你关心的典型路径（按你之前写入习惯：data/paep_contact, data/paep_phase）
    candidates = [
        "data/paep_contact",
        "data/paep_phase",
        "data/paep_phase_id",
        "data/paep_phase_prob",
        "data/paep_contact_prob",
    ]

    # 找到实际存在的 key
    exist = []
    for c in candidates:
        try:
            _ = load_array(root, c)
            exist.append(c)
        except Exception:
            pass

    if not exist:
        print("[WARN] 没找到这些候选key：", candidates)
        print("       你从上面 arrays 列表里确认一下实际路径（可能叫 paep_event 或 paep_phase_logits 之类）")
        return

    for k in exist:
        arr = load_array(root, k)
        # 注意：zarr.Array 惰性加载，这里只读一小部分/抽样统计
        x = arr[:]
        print(basic_stats(x, k))
        print(judge_binary_or_prob(x, k))
        if "phase" in k and x.ndim >= 1:
            print(judge_phase_format(x, name=k))
        print("-" * 80)

if __name__ == "__main__":
    main()
