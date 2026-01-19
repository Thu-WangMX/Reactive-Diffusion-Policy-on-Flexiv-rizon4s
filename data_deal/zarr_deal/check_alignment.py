#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
check_alignment.py

Usage:
  python check_alignment.py --zarr_path /path/to/replay_buffer.zarr --fps 24

What it checks:
  1) Where arrays live: root group or "data" subgroup
  2) timestamp exists, monotonicity, dt stats, and outliers wrt target dt=1/fps
  3) all arrays first-dim length match
  4) image repeated-frame tendency (mean abs pixel diff on sampled pairs)
  5) (optional) episode segmentation dt stats if episode_ends exists
"""

import argparse
import math
from typing import Dict, Tuple, Optional, List

import numpy as np
import zarr


IMG_KEYS_CANDIDATES = ["external_img", "left_wrist_img", "wrist_img", "rgb", "image"]
TS_KEYS_CANDIDATES = ["timestamp", "timestamps", "time", "t"]


def list_group_keys(g: zarr.hierarchy.Group) -> Tuple[List[str], List[str]]:
    """Return (subgroups, arrays) keys."""
    try:
        return list(g.group_keys()), list(g.array_keys())
    except Exception:
        # fallback
        subgroups = []
        arrays = []
        for k, v in g.items():
            if isinstance(v, zarr.hierarchy.Group):
                subgroups.append(k)
            else:
                arrays.append(k)
        return subgroups, arrays


def pick_data_group(root: zarr.hierarchy.Group) -> zarr.hierarchy.Group:
    """Use root['data'] if exists and looks like it contains arrays; else root."""
    subgroups, arrays = list_group_keys(root)
    if "data" in subgroups:
        g = root["data"]
        _, g_arrays = list_group_keys(g)
        if len(g_arrays) > 0:
            return g
    return root


def find_first_existing_key(g: zarr.hierarchy.Group, candidates: List[str]) -> Optional[str]:
    _, arrays = list_group_keys(g)
    s = set(arrays)
    for k in candidates:
        if k in s:
            return k
    return None


def check_lengths(g: zarr.hierarchy.Group) -> Dict[str, int]:
    """Return dict of array_key -> length (first dim) for arrays with ndim>=1."""
    _, arrays = list_group_keys(g)
    lens = {}
    for k in arrays:
        arr = g[k]
        if hasattr(arr, "shape") and len(arr.shape) >= 1:
            lens[k] = int(arr.shape[0])
    return lens


def dt_outliers(dt: np.ndarray, target_dt: float, low_ratio: float = 0.5, high_ratio: float = 1.5):
    """Indices where dt is too small or too large relative to target."""
    low = low_ratio * target_dt
    high = high_ratio * target_dt
    bad = np.where((dt < low) | (dt > high))[0]
    return bad, low, high


def mean_abs_pixel_diff_sampled(img_arr: np.ndarray, step: int = 100, max_pairs: int = 300) -> np.ndarray:
    """
    img_arr: (T,H,W,C) uint8
    returns diffs array (sampled mean abs diff between consecutive frames)
    """
    T = img_arr.shape[0]
    if T < 2:
        return np.array([], dtype=np.float32)

    idx = np.arange(0, T - 1, step, dtype=np.int64)
    if len(idx) > max_pairs:
        idx = idx[:max_pairs]

    diffs = []
    for i in idx:
        a = img_arr[i].astype(np.int16)
        b = img_arr[i + 1].astype(np.int16)
        diffs.append(np.mean(np.abs(a - b)))
    return np.asarray(diffs, dtype=np.float32)


def print_header(title: str):
    print("\n" + "=" * 80)
    print(title)
    print("=" * 80)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zarr_path", type=str, required=True, help="Path to replay_buffer.zarr")
    ap.add_argument("--fps", type=float, default=24.0, help="Expected FPS for the aligned timeline (default 24)")
    ap.add_argument("--outlier_low_ratio", type=float, default=0.5, help="dt < low_ratio*target_dt marked as outlier")
    ap.add_argument("--outlier_high_ratio", type=float, default=1.5, help="dt > high_ratio*target_dt marked as outlier")
    ap.add_argument("--img_step", type=int, default=100, help="Sampling step for image diff check")
    ap.add_argument("--max_img_pairs", type=int, default=300, help="Max sampled pairs for image diff check")
    ap.add_argument("--episode_ends_key", type=str, default=None,
                    help="Optional: episode_ends key path. e.g. 'meta/episode_ends' or 'episode_ends'")
    args = ap.parse_args()

    root = zarr.open_group(args.zarr_path, mode="r")
    subgroups, arrays = list_group_keys(root)

    print_header("Zarr structure")
    print("zarr_path:", args.zarr_path)
    print("root subgroups:", subgroups)
    print("root arrays:", arrays)

    g = pick_data_group(root)
    g_name = "data" if (isinstance(root.get("data", None), zarr.hierarchy.Group) and g is root["data"]) else "root"

    sg2, arr2 = list_group_keys(g)
    print("\nUsing group:", g_name)
    print("subgroups:", sg2)
    print("arrays:", arr2)

    # --- 1) length consistency ---
    print_header("Check length consistency (first dimension)")
    lens = check_lengths(g)
    if len(lens) == 0:
        print("No arrays with ndim>=1 found in selected group.")
        return

    # print a compact summary
    items = sorted(lens.items(), key=lambda x: x[0])
    for k, L in items:
        print(f"{k:<30} T={L}")

    lengths = np.array(list(lens.values()), dtype=np.int64)
    uniq = np.unique(lengths)
    if len(uniq) == 1:
        T = int(uniq[0])
        print("\n✅ All arrays share the same T =", T)
    else:
        print("\n❌ Not all arrays share the same length! Unique lengths:", uniq.tolist())
        # show offenders vs most common
        vals, counts = np.unique(lengths, return_counts=True)
        common = int(vals[np.argmax(counts)])
        print("Most common length:", common)
        offenders = [k for k, L in lens.items() if L != common]
        print("Offending keys:", offenders)
        # still continue with min length
        T = int(np.min(lengths))
        print("Proceeding with min T =", T, "(for checks below)")

    # --- 2) timestamp checks ---
    print_header("Timestamp checks")
    ts_key = find_first_existing_key(g, TS_KEYS_CANDIDATES)
    if ts_key is None:
        print("❌ No timestamp-like key found in selected group.")
        print("Tried candidates:", TS_KEYS_CANDIDATES)
        print("Available arrays:", arr2)
        print("\nHint: If timestamp is under another subgroup, pass --episode_ends_key or inspect subgroups.")
        return

    ts = np.asarray(g[ts_key][:], dtype=np.float64)
    if ts.shape[0] != T:
        print(f"⚠️ timestamp length {ts.shape[0]} != expected T {T}. Using min length for dt.")
        T2 = min(ts.shape[0], T)
        ts = ts[:T2]
    else:
        T2 = T

    dt = np.diff(ts)
    target_dt = 1.0 / float(args.fps)

    print("timestamp key:", ts_key)
    print("T used:", T2)
    print("target dt (1/fps):", target_dt)

    monotonic = bool(np.all(dt > 0))
    print("monotonic increasing:", monotonic)
    print("dt mean/std/min/max:",
          float(dt.mean()), float(dt.std()), float(dt.min()), float(dt.max()))

    bad_idx, low_thr, high_thr = dt_outliers(
        dt, target_dt,
        low_ratio=args.outlier_low_ratio,
        high_ratio=args.outlier_high_ratio
    )
    print(f"outlier thresholds: dt < {low_thr:.6f} or dt > {high_thr:.6f}")
    print("bad dt count:", int(len(bad_idx)))

    if len(bad_idx) > 0:
        print("first 30 bad indices:", bad_idx[:30].tolist())
        print("\nFirst few bad dt samples:")
        for i in bad_idx[:10]:
            print(f"  i={int(i):6d} dt={float(dt[i]):.6f}  ts[i]={float(ts[i]):.6f} ts[i+1]={float(ts[i+1]):.6f}")

    # --- 3) image repeated-frame checks ---
    print_header("Image repeated-frame checks (sampled mean abs pixel diff)")
    img_keys = [k for k in IMG_KEYS_CANDIDATES if k in set(arr2)]
    if len(img_keys) == 0:
        # try auto-detect by dtype+ndim
        candidates = []
        for k in arr2:
            arr = g[k]
            if hasattr(arr, "shape") and len(arr.shape) == 4 and arr.shape[-1] in (1, 3, 4):
                # likely images
                candidates.append(k)
        img_keys = candidates[:5]

    if len(img_keys) == 0:
        print("No image-like arrays found to check.")
    else:
        for k in img_keys:
            arr = g[k]
            if len(arr.shape) != 4:
                continue
            # sample read: we read the whole array only if not too huge; but your 33877*240*320*3 is big.
            # So do block reads by slicing needed indices only.
            print(f"\nKey: {k} shape={arr.shape} dtype={arr.dtype}")

            # We'll pull sampled pairs only to avoid loading everything.
            T_img = min(arr.shape[0], T2)
            step = args.img_step
            idx = np.arange(0, T_img - 1, step, dtype=np.int64)
            if len(idx) > args.max_img_pairs:
                idx = idx[:args.max_img_pairs]

            diffs = []
            for i in idx:
                a = arr[int(i)].astype(np.int16)
                b = arr[int(i + 1)].astype(np.int16)
                diffs.append(np.mean(np.abs(a - b)))
            diffs = np.asarray(diffs, dtype=np.float32)

            print("sample pairs:", len(diffs), "step:", step)
            print("mean/std/min/max diff:",
                  float(diffs.mean()), float(diffs.std()), float(diffs.min()), float(diffs.max()))

            # Heuristic warning: too many near-zero diffs suggests repeated frames.
            near_zero = float(np.mean(diffs < 1.0))  # threshold 1.0 intensity
            print("fraction(diff < 1.0):", near_zero)
            if near_zero > 0.3:
                print("⚠️ Many near-identical consecutive sampled frames. Could be repeated frames / low motion / stalled stream.")

    # --- 4) optional: episode ends checks ---
    print_header("Optional episode checks (if episode_ends exists)")
    ep_key = args.episode_ends_key

    def try_read_episode_ends(path_like: str) -> Optional[np.ndarray]:
        # allow "meta/episode_ends" style
        parts = [p for p in path_like.split("/") if p]
        node = root
        try:
            for p in parts[:-1]:
                node = node[p]
            arr = node[parts[-1]]
            return np.asarray(arr[:], dtype=np.int64)
        except Exception:
            return None

    episode_ends = None
    if ep_key is not None:
        episode_ends = try_read_episode_ends(ep_key)
    else:
        # try common locations
        for guess in ["episode_ends", "meta/episode_ends", "meta/episode_end", "episodes/episode_ends"]:
            episode_ends = try_read_episode_ends(guess)
            if episode_ends is not None:
                ep_key = guess
                break

    if episode_ends is None:
        print("No episode_ends found (this is OK).")
        print("If you have it, pass --episode_ends_key meta/episode_ends")
    else:
        # episode_ends stores end indices (exclusive or inclusive depends on your pipeline; we'll treat as end-exclusive if increasing)
        episode_ends = episode_ends.astype(np.int64)
        episode_ends = episode_ends[episode_ends > 0]
        episode_ends = episode_ends[episode_ends <= T2]
        episode_ends = np.unique(episode_ends)
        print("episode_ends key:", ep_key)
        print("num episodes:", len(episode_ends))
        # derive starts
        starts = np.concatenate([[0], episode_ends[:-1]])
        ends = episode_ends
        # per-episode dt stats
        for ei in range(min(5, len(ends))):
            s = int(starts[ei])
            e = int(ends[ei])
            if e - s < 2:
                continue
            dt_ep = np.diff(ts[s:e])
            print(f"Episode {ei}: [{s},{e}) len={e-s} dt_mean={float(dt_ep.mean()):.6f} dt_std={float(dt_ep.std()):.6f} dt_min={float(dt_ep.min()):.6f} dt_max={float(dt_ep.max()):.6f}")

        # quick global check: any episode boundary has huge dt?
        if len(ends) > 1:
            boundary_dt = ts[starts[1:]] - ts[ends[:-1]-1]  # approx gap between last frame and next episode first frame
            print("episode boundary gap mean/std/min/max:",
                  float(boundary_dt.mean()), float(boundary_dt.std()), float(boundary_dt.min()), float(boundary_dt.max()))

    print_header("Done")
    print("If dt is stable around 1/24 and image diffs are not near-zero too often, you can train PAEP directly.")


if __name__ == "__main__":
    main()
