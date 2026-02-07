# stats_plugin_charger.py
# -*- coding: utf-8 -*-

import os
import json
from typing import Dict, Any, List, Tuple, Optional

import numpy as np
import zarr


# =========================
# 配置区：你只改这里
# =========================
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_charger/plug_in_charger_stream_downsample1_zarr/replay_buffer.zarr"

# 期望的 key（脚本会自动在 group 层级中寻找）
WRENCH_KEY = "left_robot_tcp_wrench"
POSE_KEY = "left_robot_tcp_pose"
EPISODE_ENDS_KEY = "meta/episode_ends"   # 期望存在

USE_ABS_FZ = True
PER_EPISODE = True

SAVE_JSON = True
OUT_JSON = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/stats_and_label/plug_in_charger/plug_in_charger_stats.json"
# =========================


def _percentiles(x: np.ndarray, ps=(0, 0.1, 1, 5, 10, 25, 50, 75, 90, 95, 98, 99, 99.5, 99.9, 100)) -> Dict[str, float]:
    x = x[np.isfinite(x)]
    if x.size == 0:
        return {f"p{p}": float("nan") for p in ps}
    qs = np.percentile(x, ps)
    return {f"p{ps[i]}": float(qs[i]) for i in range(len(ps))}


def _basic_stats(x: np.ndarray) -> Dict[str, Any]:
    x = x[np.isfinite(x)]
    if x.size == 0:
        return {"n": 0, "mean": float("nan"), "std": float("nan"),
                "min": float("nan"), "max": float("nan"), "percentiles": {}}
    return {
        "n": int(x.size),
        "mean": float(np.mean(x)),
        "std": float(np.std(x)),
        "min": float(np.min(x)),
        "max": float(np.max(x)),
        "percentiles": _percentiles(x),
    }


def _episode_slices(episode_ends: np.ndarray) -> List[Tuple[int, int]]:
    ends = episode_ends.astype(np.int64)
    starts = np.concatenate([[0], ends[:-1]])
    return list(zip(starts.tolist(), ends.tolist()))


def _group_has_keys(g: zarr.Group, keys: List[str]) -> bool:
    try:
        ak = set(g.array_keys())
    except Exception:
        return False
    return all(k in ak for k in keys)


def _find_group_with_keys(root: zarr.Group, keys: List[str], max_depth: int = 6) -> Tuple[zarr.Group, str]:
    """
    Search breadth-first for a group that contains all given array keys.
    Return (group, group_path_str).
    """
    queue: List[Tuple[zarr.Group, str, int]] = [(root, "/", 0)]
    visited = set()

    while queue:
        g, path, depth = queue.pop(0)
        if id(g) in visited:
            continue
        visited.add(id(g))

        if _group_has_keys(g, keys):
            return g, path

        if depth >= max_depth:
            continue

        try:
            subkeys = list(g.group_keys())
        except Exception:
            subkeys = []

        for sk in subkeys:
            try:
                child = g[sk]
                if isinstance(child, zarr.Group):
                    child_path = (path.rstrip("/") + "/" + sk).replace("//", "/")
                    queue.append((child, child_path, depth + 1))
            except Exception:
                continue

    raise KeyError(
        f"Cannot find any group containing keys={keys} under ZARR_PATH={ZARR_PATH}. "
        f"Root group_keys={list(root.group_keys())}, root array_keys={list(root.array_keys())}"
    )


def _try_read_episode_ends(root: zarr.Group) -> Optional[np.ndarray]:
    # prefer meta/episode_ends from root
    try:
        if "meta" in root.group_keys() and "episode_ends" in root["meta"].array_keys():
            return np.array(root["meta"]["episode_ends"][:], dtype=np.int64)
    except Exception:
        pass
    # or episode_ends at root
    try:
        if "episode_ends" in root.array_keys():
            return np.array(root["episode_ends"][:], dtype=np.int64)
    except Exception:
        pass
    return None


def main():
    if not os.path.exists(ZARR_PATH):
        raise FileNotFoundError(f"ZARR_PATH not found: {ZARR_PATH}")

    root = zarr.open(ZARR_PATH, mode="r")

    # 1) episode_ends 必须存在（你说确定有，那我们就强制读出来）
    episode_ends = _try_read_episode_ends(root)
    if episode_ends is None:
        raise KeyError(f"Missing {EPISODE_ENDS_KEY} (or root episode_ends) in {ZARR_PATH}")

    # 2) 自动定位包含 wrench/pose 的 group（解决你现在 root.array_keys() 为空的问题）
    data_group, data_group_path = _find_group_with_keys(root, [WRENCH_KEY, POSE_KEY])
    print(f"[OK] Using data group: {data_group_path}")
    print(f"[OK] data_group.array_keys() sample: {list(data_group.array_keys())[:30]}")

    wrench = np.array(data_group[WRENCH_KEY][:], dtype=np.float32)
    pose = np.array(data_group[POSE_KEY][:], dtype=np.float32)

    T = wrench.shape[0]
    if pose.shape[0] != T:
        raise RuntimeError(f"Length mismatch: wrench={T}, pose={pose.shape[0]}")

    Fx, Fy, Fz = wrench[:, 0], wrench[:, 1], wrench[:, 2]
    Mx, My, Mz = wrench[:, 3], wrench[:, 4], wrench[:, 5]

    F_norm = np.sqrt(Fx**2 + Fy**2 + Fz**2)
    M_norm = np.sqrt(Mx**2 + My**2 + Mz**2)
    W_norm = np.sqrt(F_norm**2 + M_norm**2)

    tcp_z = pose[:, 2]

    # frame-diff (不依赖 timestamp)
    dFz = np.concatenate([np.diff(Fz).astype(np.float32), [np.nan]]).astype(np.float32)
    dFn = np.concatenate([np.diff(F_norm).astype(np.float32), [np.nan]]).astype(np.float32)
    dz = np.concatenate([np.diff(tcp_z).astype(np.float32), [np.nan]]).astype(np.float32)

    Fz_use = np.abs(Fz) if USE_ABS_FZ else Fz

    stats: Dict[str, Any] = {
        "meta": {
            "zarr_path": ZARR_PATH,
            "data_group_path": data_group_path,
            "T": int(T),
            "wrench_key": WRENCH_KEY,
            "pose_key": POSE_KEY,
            "episode_ends_key": EPISODE_ENDS_KEY,
            "num_episodes": int(len(episode_ends)),
        },
        "global": {
            "Fx": _basic_stats(Fx),
            "Fy": _basic_stats(Fy),
            "Fz_signed": _basic_stats(Fz),
            "absFz" if USE_ABS_FZ else "Fz_use": _basic_stats(Fz_use),
            "F_norm": _basic_stats(F_norm),
            "Mx": _basic_stats(Mx),
            "My": _basic_stats(My),
            "Mz": _basic_stats(Mz),
            "M_norm": _basic_stats(M_norm),
            "W_norm": _basic_stats(W_norm),
            "tcp_z": _basic_stats(tcp_z),
            "dFz_frame": _basic_stats(dFz),
            "dF_norm_frame": _basic_stats(dFn),
            "dz_frame": _basic_stats(dz),
        }
    }

    if PER_EPISODE:
        slices = _episode_slices(episode_ends)
        ep_stats = []
        for i, (s, e) in enumerate(slices):
            if e - s < 5:
                continue
            ep_stats.append({
                "ep": i,
                "start": int(s),
                "end": int(e),
                "len": int(e - s),
                "absFz": _basic_stats(Fz_use[s:e]),
                "F_norm": _basic_stats(F_norm[s:e]),
                "M_norm": _basic_stats(M_norm[s:e]),
                "W_norm": _basic_stats(W_norm[s:e]),
                "tcp_z": _basic_stats(tcp_z[s:e]),
            })
        stats["episodes"] = {
            "episode_ends": episode_ends.astype(int).tolist(),
            "stats": ep_stats,
        }

    # 打印关键分位数
    g = stats["global"]
    key_absfz = "absFz" if USE_ABS_FZ else "Fz_use"
    print("\n=== Plug-in Charger Stats ===")
    print("zarr:", stats["meta"]["zarr_path"])
    print("data_group_path:", stats["meta"]["data_group_path"])
    print("T:", stats["meta"]["T"], "episodes:", stats["meta"]["num_episodes"])

    print("\n[Force magnitude]")
    print("  absFz   p95/p99/p99.5:", g[key_absfz]["percentiles"].get("p95"), g[key_absfz]["percentiles"].get("p99"), g[key_absfz]["percentiles"].get("p99.5"))
    print("  F_norm  p95/p99/p99.5:", g["F_norm"]["percentiles"].get("p95"), g["F_norm"]["percentiles"].get("p99"), g["F_norm"]["percentiles"].get("p99.5"))
    print("  M_norm  p95/p99/p99.5:", g["M_norm"]["percentiles"].get("p95"), g["M_norm"]["percentiles"].get("p99"), g["M_norm"]["percentiles"].get("p99.5"))
    print("  dF_norm_frame p99/p99.5:", g["dF_norm_frame"]["percentiles"].get("p99"), g["dF_norm_frame"]["percentiles"].get("p99.5"))

    print("\n[Z]")
    print("  tcp_z   p5/p50/p95:", g["tcp_z"]["percentiles"].get("p5"), g["tcp_z"]["percentiles"].get("p50"), g["tcp_z"]["percentiles"].get("p95"))
    print("  dz_frame p1/p99:", g["dz_frame"]["percentiles"].get("p1"), g["dz_frame"]["percentiles"].get("p99"))

    if SAVE_JSON:
        os.makedirs(os.path.dirname(OUT_JSON), exist_ok=True)
        with open(OUT_JSON, "w", encoding="utf-8") as f:
            json.dump(stats, f, ensure_ascii=False, indent=2)
        print("\nSaved:", OUT_JSON)


if __name__ == "__main__":
    main()
