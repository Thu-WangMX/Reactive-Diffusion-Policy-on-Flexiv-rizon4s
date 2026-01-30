#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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


def print_stats(name, x):
    s = stats_1d(x)
    print(f"\n== {name} ==")
    print(f"  n={len(np.asarray(x).reshape(-1))}")
    print(f"  mean={s['mean']:.6g}  std={s['std']:.6g}")
    print(f"  min={s['min']:.6g}  p1={s['p1']:.6g}  p5={s['p5']:.6g}  p50={s['p50']:.6g}  p95={s['p95']:.6g}  p99={s['p99']:.6g}  max={s['max']:.6g}")
    print(f"  max|x|={s['max_abs']:.6g}")


def analyze(name, r9, dz_clip=None, phase=None, contact=None, pval=None, cval=None):
    mask = np.ones((r9.shape[0],), dtype=bool)
    if phase is not None and pval is not None:
        mask &= (phase == pval)
    if contact is not None and cval is not None:
        mask &= (contact == cval)

    rr = r9[mask]
    if rr.shape[0] == 0:
        print(f"\n[{name}] empty, skip")
        return

    dp = rr[:, :3]
    rot6 = rr[:, 3:9]
    dp_norm = np.linalg.norm(dp, axis=1)
    rot6_err = np.linalg.norm(rot6 - ID_ROT6[None, :], axis=1)

    nz = int(np.sum(dp_norm > 1e-12))
    idlike = int(np.sum(rot6_err < 1e-6))

    print(f"\n------------------------------")
    print(f"[{name}] frames={rr.shape[0]}")
    print(f"  dp_nonzero_frames={nz} ({100.0*nz/rr.shape[0]:.3f}%)")
    print(f"  rot6_identity_like={idlike} ({100.0*idlike/rr.shape[0]:.3f}%)")
    print(f"  rot6_err mean={rot6_err.mean():.3e} p95={np.percentile(rot6_err,95):.3e} max={rot6_err.max():.3e}")

    print_stats("dp_x", dp[:, 0])
    print_stats("dp_y", dp[:, 1])
    print_stats("dp_z", dp[:, 2])
    print_stats("||dp||", dp_norm)

    if dz_clip is not None:
        # 近似 clip：|dp_z| 接近 dz_clip
        clip_frames = int(np.sum(np.abs(dp[:, 2]) >= (0.999 * dz_clip)))
        print(f"\n== Clip estimate ==")
        print(f"  dz_clip={dz_clip}  clip_frames~{clip_frames} ({100.0*clip_frames/rr.shape[0]:.3f}%)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rb_path", type=str, required=True, help="teacherB_fast_label_9d.npy")
    ap.add_argument("--zarr_path", type=str, required=True)
    ap.add_argument("--phase_key", type=str, default="paep_phase")
    ap.add_argument("--contact_key", type=str, default="paep_contact")
    ap.add_argument("--dz_clip", type=float, default=None, help="optional, for clip estimate")
    args = ap.parse_args()

    rB = np.load(args.rb_path).astype(np.float32)
    assert rB.ndim == 2 and rB.shape[1] == 9

    rb = ReplayBuffer.copy_from_path(args.zarr_path, keys=[args.phase_key, args.contact_key])
    phase = np.asarray(rb[args.phase_key][:], dtype=np.int64).reshape(-1)
    contact = np.asarray(rb[args.contact_key][:], dtype=np.float32).reshape(-1)

    assert len(phase) == rB.shape[0] and len(contact) == rB.shape[0]

    print("[INFO] phase uniq:", np.unique(phase))
    print("[INFO] contact uniq:", np.unique(contact))

    analyze("ALL", rB, dz_clip=args.dz_clip, phase=phase, contact=contact)
    for p in [0,1,2]:
        analyze(f"phase={p}", rB, dz_clip=args.dz_clip, phase=phase, contact=contact, pval=p)
    for p in [0,1,2]:
        for c in [0.0, 1.0]:
            analyze(f"phase={p},contact={int(c)}", rB, dz_clip=args.dz_clip, phase=phase, contact=contact, pval=p, cval=c)


if __name__ == "__main__":
    main()
