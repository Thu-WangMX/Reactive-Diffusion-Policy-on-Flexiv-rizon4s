#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import numpy as np
import zarr

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zarr_path", required=True)
    ap.add_argument("--wrench_key", default="data/left_robot_tcp_wrench")
    ap.add_argument("--timestamp_key", default="data/timestamp")
    ap.add_argument("--episode_ends_key", default="meta/episode_ends")
    ap.add_argument("--fz_index", type=int, default=2)
    ap.add_argument("--use_abs", action="store_true")
    ap.add_argument("--print_per_episode", action="store_true")
    args = ap.parse_args()

    root = zarr.open(args.zarr_path, mode="r")
    wrench = np.asarray(root[args.wrench_key][:], dtype=np.float32)
    ts = np.asarray(root[args.timestamp_key][:], dtype=np.float32)
    ep_ends = np.asarray(root[args.episode_ends_key][:], dtype=np.int64)

    fz = wrench[:, args.fz_index]
    x = np.abs(fz) if args.use_abs else fz

    def q(a, ps):
        return np.percentile(a, ps).astype(np.float64)

    ps = [50, 75, 90, 95, 98, 99]
    qs = q(x, ps)
    print("=== Global Fz stats ===")
    print(f"N frames: {len(x)}  episodes: {len(ep_ends)}")
    print(f"min: {x.min():.3f}  max: {x.max():.3f}  mean: {x.mean():.3f}  std: {x.std():.3f}")
    for p, v in zip(ps, qs):
        print(f"p{p:02d}: {v:.3f}")

    # dt / hz
    dt = np.diff(ts.astype(np.float64))
    dt = dt[(dt > 1e-6) & (dt < 1.0)]
    if len(dt) > 0:
        med = np.median(dt)
        print(f"median dt: {med:.6f}s  hz~{1.0/med:.2f}")
    else:
        print("median dt: NA")

    if args.print_per_episode:
        print("\n=== Per-episode summary (p50/p95/max) ===")
        starts = np.concatenate([[0], ep_ends[:-1]])
        for i, (s, e) in enumerate(zip(starts, ep_ends)):
            xi = x[s:e]
            p50, p95 = np.percentile(xi, [50, 95])
            print(f"ep{i:03d} T={len(xi):4d}  p50={p50:6.2f} p95={p95:6.2f} max={xi.max():6.2f}")

if __name__ == "__main__":
    main()
