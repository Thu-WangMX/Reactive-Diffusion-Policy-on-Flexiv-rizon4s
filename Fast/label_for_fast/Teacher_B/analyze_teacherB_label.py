#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import json
import numpy as np

LABEL_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/label_for_fast/offline_wiping_board_teacherB_out/teacherB_fast_label_9d.npy"
META_PATH  = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/label_for_fast/offline_wiping_board_teacherB_out/teacherB_label_meta.json"

def _pct(x, q):
    return float(np.percentile(x, q))

def _safe_mean(x):
    return float(np.mean(x)) if x.size else float("nan")

def _safe_maxabs(x):
    return float(np.max(np.abs(x))) if x.size else float("nan")

def summarize_array(name, x):
    x = x.reshape(-1)
    print(f"\n== {name} ==")
    print(f"  n={x.size}")
    print(f"  mean={_safe_mean(x):.6g}  std={float(np.std(x)):.6g}")
    print(f"  min={float(np.min(x)):.6g}  p1={_pct(x,1):.6g}  p5={_pct(x,5):.6g}  p50={_pct(x,50):.6g}  p95={_pct(x,95):.6g}  p99={_pct(x,99):.6g}  max={float(np.max(x)):.6g}")
    print(f"  max|x|={_safe_maxabs(x):.6g}")

def main():
    assert os.path.isfile(LABEL_PATH), f"not found: {LABEL_PATH}"
    label = np.load(LABEL_PATH).astype(np.float32)  # (T,9)
    assert label.ndim == 2 and label.shape[1] == 9, f"expected (T,9), got {label.shape}"
    T = label.shape[0]

    print("[INFO] Loaded:", LABEL_PATH)
    print("[INFO] Shape:", label.shape)

    dp = label[:, :3]      # position residual (in "relative action" space of your action_utils)
    drot6 = label[:, 3:9]  # rot6d residual

    # ---- Identity check for rot6d part ----
    # You set identity as [1,0,0, 0,1,0]
    rot6_id = np.array([1,0,0, 0,1,0], dtype=np.float32)
    rot6_err = np.linalg.norm(drot6 - rot6_id[None, :], axis=1)

    # ---- zero/nonzero stats for dp ----
    dp_norm = np.linalg.norm(dp, axis=1)
    nonzero_mask = dp_norm > 1e-12

    print("\n== Basic ==")
    print(f"  T={T}")
    print(f"  dp_nonzero_frames={int(np.sum(nonzero_mask))}  ({100.0*np.mean(nonzero_mask):.3f}%)")
    print(f"  rot6_identity_like(frames with ||rot6-Id||<1e-6)={int(np.sum(rot6_err<1e-6))} ({100.0*np.mean(rot6_err<1e-6):.3f}%)")
    print(f"  rot6_err mean={float(np.mean(rot6_err)):.6g}  p95={_pct(rot6_err,95):.6g}  max={float(np.max(rot6_err)):.6g}")

    summarize_array("dp_x", dp[:,0])
    summarize_array("dp_y", dp[:,1])
    summarize_array("dp_z", dp[:,2])
    summarize_array("||dp||", dp_norm)

    # rot6d distribution (should be close to identity if you kept rotation unchanged)
    summarize_array("rot6[0]", drot6[:,0])
    summarize_array("rot6[1]", drot6[:,1])
    summarize_array("rot6[2]", drot6[:,2])
    summarize_array("rot6[3]", drot6[:,3])
    summarize_array("rot6[4]", drot6[:,4])
    summarize_array("rot6[5]", drot6[:,5])

    # ---- Optional: estimate dz_tool from dp magnitude (only meaningful if dp is pure tool-z projected) ----
    dz_clip = None
    comp = None
    if os.path.isfile(META_PATH):
        with open(META_PATH, "r") as f:
            meta = json.load(f)
        dz_clip = float(meta.get("dz_clip_m", 0.0))
        comp = float(meta.get("compliance_m_per_N", 0.0))
        print("\n== Meta ==")
        print(f"  dz_clip_m={dz_clip}")
        print(f"  compliance_m_per_N={comp}  (K ~= {1.0/comp if comp>0 else float('nan'):.6g} N/m)")

    # If your residual is basically dp_base = tool_z_in_base * dz_tool,
    # then ||dp|| ~= |dz_tool| (since tool_z_in_base is unit vector).
    # So we can use dp_norm as an estimate of |dz_tool|.
    if dz_clip is not None and dz_clip > 0:
        est_abs_dz = dp_norm
        clip_mask = est_abs_dz >= (dz_clip - 1e-6)
        print("\n== Estimated |dz_tool| stats (via ||dp||) ==")
        print(f"  clip_frames ~= {int(np.sum(clip_mask))} ({100.0*np.mean(clip_mask):.3f}%)")
        summarize_array("|dz_tool|~||dp||", est_abs_dz)

        # show top-20 largest steps
        topk = 20
        idx = np.argsort(-est_abs_dz)[:topk]
        print(f"\n== Top-{topk} frames by ||dp|| (approx |dz_tool|) ==")
        for i in idx:
            print(f"  t={int(i):6d}  ||dp||={float(est_abs_dz[i]):.6g}  dp={dp[i].tolist()}  rot6_err={float(rot6_err[i]):.3e}")

if __name__ == "__main__":
    main()
