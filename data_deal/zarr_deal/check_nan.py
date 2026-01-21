import os, sys
import numpy as np
import zarr

ZARR_PATH = sys.argv[1]

def scan_array(arr, name, max_chunks=200):
    # 只抽样一些 chunk，不要全量扫（太慢）
    total_bad = 0
    checked = 0

    # 处理 zarr.Array / numpy
    if not hasattr(arr, "ndim"):
        return

    # 只扫 float 类数组（NaN/Inf 才有意义），但也顺便打印 dtype
    print(f"\n== {name} == shape={arr.shape} dtype={arr.dtype}")

    # 对于非浮点，主要看是否有异常范围（可选）
    is_float = np.issubdtype(arr.dtype, np.floating)

    # 生成一些切片索引（按第一维抽样）
    T = arr.shape[0] if arr.ndim >= 1 else 1
    idxs = np.linspace(0, max(T-1, 0), num=min(20, T), dtype=int) if T > 0 else [0]

    for i in idxs:
        x = arr[i]
        checked += 1
        if is_float:
            bad = ~np.isfinite(x)
            if bad.any():
                total_bad += bad.sum()
                print(f"[BAD] at {name}[{i}] bad_count={bad.sum()} "
                      f"min={np.nanmin(x)} max={np.nanmax(x)}")
        if checked >= max_chunks:
            break

    if is_float and total_bad == 0:
        print("OK: no NaN/Inf found in sampled slices.")

def main():
    root = zarr.open(os.path.join(ZARR_PATH, "replay_buffer.zarr"), mode="r")
    # 你们的键可能在 root['data'] 或 root['data/xxx']，先列一下
    print("Keys:", list(root.array_keys())[:50], "...")
    print("Groups:", list(root.group_keys())[:50], "...")

    # 常见结构：root['data'] 是 group
    data = root.get("data", root)

    # 遍历所有数组
    def walk(g, prefix=""):
        for k in g.array_keys():
            scan_array(g[k], prefix + k)
        for k in g.group_keys():
            walk(g[k], prefix + k + "/")

    walk(data, "data/")

if __name__ == "__main__":
    main()
