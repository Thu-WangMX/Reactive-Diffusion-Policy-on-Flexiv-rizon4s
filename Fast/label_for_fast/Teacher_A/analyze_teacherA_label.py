#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import argparse
import numpy as np

from reactive_diffusion_policy.common.replay_buffer import ReplayBuffer


ID_ROT6 = np.array([1, 0, 0, 0, 1, 0], dtype=np.float32)


def pct(a, q):
    return float(np.percentile(a, q))


def stats_1d(x):
    x = np.asarray(x).reshape(-1)
    return {
        "mean": float(x.mean()),
        "std": float(x.std()),
        "min": float(x.min()),
        "p1": pct(x, 1),
        "p5": pct(x, 5),
        "p50": pct(x, 50),
        "p95": pct(x, 95),
        "p99": pct(x, 99),
        "max": float(x.max()),
        "max_abs": float(np.max(np.abs(x))),
    }


def print_stats_block(name, x):
    s = stats_1d(x)
    print(f"\n== {name} ==")
    print(f"  n={len(np.asarray(x).reshape(-1))}")
    print(f"  mean={s['mean']:.6g}  std={s['std']:.6g}")
    print(f"  min={s['min']:.6g}  p1={s['p1']:.6g}  p5={s['p5']:.6g}  p50={s['p50']:.6g}  p95={s['p95']:.6g}  p99={s['p99']:.6g}  max={s['max']:.6g}")
    print(f"  max|x|={s['max_abs']:.6g}")


def analyze_one(name, r9, phase=None, contact=None, phase_val=None, contact_val=None):
    # optional filter
    mask = np.ones((r9.shape[0],), dtype=bool)
    if phase is not None and phase_val is not None:
        mask &= (phase == phase_val)
    if contact is not None and contact_val is not None:
        mask &= (contact == contact_val)

    rr = r9[mask]
    if rr.shape[0] == 0:
        print(f"\n[{name}] empty after filter, skip")
        return

    dp = rr[:, :3]
    rot6 = rr[:, 3:9]

    rot6_err = np.linalg.norm(rot6 - ID_ROT6[None, :], axis=1)
    dp_norm = np.linalg.norm(dp, axis=1)

    # “非零帧”阈值：基本等同你之前 B 的统计逻辑
    dp_nz = np.sum(dp_norm > 1e-12)
    rot_id_like = np.sum(rot6_err < 1e-6)

    print(f"\n------------------------------")
    print(f"[{name}] frames={rr.shape[0]}")
    print(f"  dp_nonzero_frames={dp_nz} ({100.0*dp_nz/rr.shape[0]:.3f}%)")
    print(f"  rot6_identity_like(||rot6-Id||<1e-6)={rot_id_like} ({100.0*rot_id_like/rr.shape[0]:.3f}%)")
    print(f"  rot6_err mean={rot6_err.mean():.4e}  p95={np.percentile(rot6_err,95):.4e}  max={rot6_err.max():.4e}")

    print_stats_block("dp_x", dp[:, 0])
    print_stats_block("dp_y", dp[:, 1])
    print_stats_block("dp_z", dp[:, 2])
    print_stats_block("||dp||", dp_norm)

    # rot6 dims
    for i in range(6):
        print_stats_block(f"rot6[{i}]", rot6[:, i])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ra_path", type=str, required=True, help="teacherA_fast_label_9d.npy")
    ap.add_argument("--zarr_path", type=str, default="", help="replay_buffer.zarr (optional, for phase/contact grouping)")
    ap.add_argument("--phase_key", type=str, default="paep_phase")
    ap.add_argument("--contact_key", type=str, default="paep_contact")
    args = ap.parse_args()

    rA = np.load(args.ra_path).astype(np.float32)
    assert rA.ndim == 2 and rA.shape[1] == 9, f"Expected (T,9), got {rA.shape}"

    print(f"[INFO] Loaded: {args.ra_path}")
    print(f"[INFO] Shape: {rA.shape}")

    phase = contact = None
    if args.zarr_path:
        rb = ReplayBuffer.copy_from_path(args.zarr_path, keys=[args.phase_key, args.contact_key])
        phase = np.asarray(rb[args.phase_key][:], dtype=np.int64).reshape(-1)
        contact = np.asarray(rb[args.contact_key][:], dtype=np.float32).reshape(-1)
        assert len(phase) == rA.shape[0] and len(contact) == rA.shape[0], "Length mismatch vs rA"
        print(f"[INFO] phase uniq: {np.unique(phase)}")
        print(f"[INFO] contact uniq: {np.unique(contact)}")

    # overall
    analyze_one("ALL", rA, phase=phase, contact=contact)

    # grouped
    if phase is not None and contact is not None:
        for p in [0, 1, 2]:
            analyze_one(f"phase={p}", rA, phase=phase, contact=contact, phase_val=p)
        for p in [0, 1, 2]:
            for c in [0.0, 1.0]:
                analyze_one(f"phase={p},contact={int(c)}", rA, phase=phase, contact=contact, phase_val=p, contact_val=c)


if __name__ == "__main__":
    main()
