#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
offline_sanity_fast.py

A stronger offline sanity checker for trained Fast residual model.

What it checks (val by default):
1) MAE metrics: all9/dp3/rot6, and active-only variants
2) Z-dim sanity: pred_z/gt_z distribution and abs error distribution
3) Correlation sanity: corr(z), corr(dp3_flat)
4) Outlier inspection: top-K worst frames by |z error| (with pred_z, gt_z, active flag)
5) Optional gate stats if aux["gate"] exists

Outputs:
- OUT_DIR/sanity_fast_report.json
- OUT_DIR/sanity_fast_npz.npz  (pred, gt, w_soft, m_hard, err_z, topk_idx, optional gate)
"""

import os
import json
import numpy as np
import torch
from torch.utils.data import DataLoader

from fast_dataset import FastResidualDataset, PHASE_NAMES, ACTIVE_PHASE_NAMES, HIST
from fast_model import FastResidualPolicy


# =========================
# CONFIG (edit here)
# =========================
CKPT_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/train_fast_wiping_v2_run1/ckpt_best.pt"
OUT_DIR   = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/train_fast_wiping_v2_run1"

SPLIT = "val"          # "train" or "val"
BATCH = 4096
NUM_WORKERS = 2
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

TOPK = 50              # number of worst frames to dump

# Wiping deployment usually ignores rot; this flag only affects extra sanity stats display (not MAE(all9))
ONLY_Z_DEPLOY_STYLE = True  # if True: also compute dp3/z metrics with pred_xy=0 for "deploy-like" sanity


def _np(x: torch.Tensor) -> np.ndarray:
    return x.detach().cpu().numpy()


def _safe_corr(a: np.ndarray, b: np.ndarray) -> float:
    a = a.reshape(-1).astype(np.float64)
    b = b.reshape(-1).astype(np.float64)
    if a.size < 2:
        return float("nan")
    if np.std(a) < 1e-12 or np.std(b) < 1e-12:
        return float("nan")
    return float(np.corrcoef(a, b)[0, 1])


def _stats_1d(x: np.ndarray) -> dict:
    x = np.asarray(x).reshape(-1).astype(np.float64)
    q = np.quantile(x, [0.0, 0.01, 0.05, 0.5, 0.95, 0.99, 1.0])
    return {
        "n": int(x.size),
        "mean": float(x.mean()),
        "std": float(x.std()),
        "min": float(q[0]),
        "q01": float(q[1]),
        "q05": float(q[2]),
        "median": float(q[3]),
        "q95": float(q[4]),
        "q99": float(q[5]),
        "max": float(q[6]),
    }


def _mae_block(pred: np.ndarray, gt: np.ndarray, mask: np.ndarray | None = None) -> dict:
    if mask is None:
        p = pred
        g = gt
    else:
        p = pred[mask]
        g = gt[mask]
    if p.shape[0] == 0:
        return {"all9": float("nan"), "dp3": float("nan"), "rot6": float("nan")}
    abs9 = np.abs(p - g)
    return {
        "all9": float(abs9.mean()),
        "dp3": float(abs9[:, :3].mean()),
        "rot6": float(abs9[:, 3:9].mean()),
    }


@torch.no_grad()
def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # ---------- dataset ----------
    ds = FastResidualDataset(split=SPLIT)
    dl = DataLoader(ds, batch_size=BATCH, shuffle=False, num_workers=NUM_WORKERS, pin_memory=True, drop_last=False)

    # ---------- model ----------
    ckpt = torch.load(CKPT_PATH, map_location="cpu")
    model = FastResidualPolicy(d_base=ds.base_dim, d_paep=ds.paep_dim).to(DEVICE)

    missing, unexpected = model.load_state_dict(ckpt["model"], strict=False)
    if len(missing) or len(unexpected):
        print("[WARN] load_state_dict(strict=False)")
        print("  missing:", missing)
        print("  unexpected:", unexpected)

    model.eval()

    # ---------- run ----------
    pred_list, gt_list, w_list, m_list = [], [], [], []
    gate_list = []

    for base_h, wrench_h, paep_h, y, w_soft, m_hard, _t in dl:
        base_h = base_h.to(DEVICE, non_blocking=True)
        wrench_h = wrench_h.to(DEVICE, non_blocking=True)
        paep_h = paep_h.to(DEVICE, non_blocking=True)
        y = y.to(DEVICE, non_blocking=True)

        pred, aux = model(base_h, wrench_h, paep_h)

        pred_list.append(_np(pred))
        gt_list.append(_np(y))
        w_list.append(_np(w_soft))
        m_list.append(_np(m_hard))

        if "gate" in aux:
            gate_list.append(_np(aux["gate"]))

    pred = np.concatenate(pred_list, axis=0)  # (N,9)
    gt = np.concatenate(gt_list, axis=0)      # (N,9)
    w_soft = np.concatenate(w_list, axis=0).reshape(-1)  # (N,)
    m_hard = np.concatenate(m_list, axis=0).reshape(-1)  # (N,)
    active = (m_hard > 0.5)

    N = int(pred.shape[0])
    Na = int(active.sum())

    # ---------- primary MAE ----------
    mae_all = _mae_block(pred, gt, None)
    mae_act = _mae_block(pred, gt, active)

    # ---------- z sanity ----------
    pred_z = pred[:, 2]
    gt_z = gt[:, 2]
    err_z = np.abs(pred_z - gt_z)

    z_stats = {
        "pred_z": _stats_1d(pred_z),
        "gt_z": _stats_1d(gt_z),
        "abs_err_z": _stats_1d(err_z),
        "pred_z_active": _stats_1d(pred_z[active]) if Na > 0 else None,
        "gt_z_active": _stats_1d(gt_z[active]) if Na > 0 else None,
        "abs_err_z_active": _stats_1d(err_z[active]) if Na > 0 else None,
    }

    # ---------- correlations ----------
    corr = {
        "z": _safe_corr(pred_z, gt_z),
        "z_active": _safe_corr(pred_z[active], gt_z[active]) if Na > 0 else float("nan"),
        "dp3_flat": _safe_corr(pred[:, :3], gt[:, :3]),
    }

    # ---------- deploy-like sanity (optional): assume you will zero xy/rot in deployment ----------
    deploy_like = None
    if ONLY_Z_DEPLOY_STYLE:
        pred_deploy = pred.copy()
        pred_deploy[:, 0] = 0.0
        pred_deploy[:, 1] = 0.0
        pred_deploy[:, 3:9] = 0.0
        deploy_like = {
            "mae_deploy_like_all": _mae_block(pred_deploy, gt, None),
            "mae_deploy_like_active": _mae_block(pred_deploy, gt, active),
            "abs_pred_xy_mean": np.abs(pred[:, :2]).mean(axis=0).tolist(),
            "abs_gt_xy_mean": np.abs(gt[:, :2]).mean(axis=0).tolist(),
        }

    # ---------- top-K worst frames by |z error| ----------
    topk = min(TOPK, N)
    topk_idx = np.argsort(-err_z)[:topk]
    worst = []
    for i in topk_idx:
        worst.append({
            "idx": int(i),
            "abs_err_z": float(err_z[i]),
            "pred_z": float(pred_z[i]),
            "gt_z": float(gt_z[i]),
            "active": bool(active[i]),
            "w_soft": float(w_soft[i]),
            "pred_dp3": pred[i, :3].astype(np.float64).tolist(),
            "gt_dp3": gt[i, :3].astype(np.float64).tolist(),
        })

    # ---------- gate stats (optional) ----------
    gate_stats = None
    gate = None
    if len(gate_list) > 0:
        gate = np.concatenate(gate_list, axis=0)  # (N,M)
        g = np.clip(gate, 1e-12, 1.0)
        ent = (-g * np.log(g)).sum(axis=1)
        gate_stats = {
            "gate_shape": list(gate.shape),
            "gate_mean": gate.mean(axis=0).tolist(),
            "gate_entropy_mean": float(ent.mean()),
            "gate_entropy_std": float(ent.std()),
        }

    report = {
        "ckpt_path": CKPT_PATH,
        "split": SPLIT,
        "phase_names": PHASE_NAMES,
        "active_phase_names": ACTIVE_PHASE_NAMES,
        "HIST": int(HIST),
        "N": N,
        "N_active": Na,
        "active_ratio": float(Na / max(1, N)),
        "wrench_clip_ratio_global": float(getattr(ds, "wrench_clip_ratio", float("nan"))),

        "mae_all": mae_all,
        "mae_active": mae_act,
        "z_stats": z_stats,
        "corr": corr,
        "deploy_like": deploy_like,
        "gate_stats": gate_stats,
        "worst_topk_by_abs_err_z": worst,
    }

    report_path = os.path.join(OUT_DIR, "sanity_fast_report.json")
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    print("[OK] wrote:", report_path)

    # save arrays for optional plotting/debugging later
    npz_path = os.path.join(OUT_DIR, "sanity_fast_npz.npz")
    save_dict = {
        "pred": pred.astype(np.float32),
        "gt": gt.astype(np.float32),
        "w_soft": w_soft.astype(np.float32),
        "m_hard": m_hard.astype(np.float32),
        "active": active.astype(np.float32),
        "err_z": err_z.astype(np.float32),
        "topk_idx": topk_idx.astype(np.int64),
    }
    if gate is not None:
        save_dict["gate"] = gate.astype(np.float32)
    np.savez_compressed(npz_path, **save_dict)
    print("[OK] wrote:", npz_path)

    # quick console summary
    print("\n=== Summary ===")
    print("MAE(all):", json.dumps(mae_all, indent=2))
    print("MAE(active):", json.dumps(mae_act, indent=2))
    print("corr:", json.dumps(corr, indent=2))
    print("z pred stats:", json.dumps(z_stats["pred_z"], indent=2))
    print("z gt   stats:", json.dumps(z_stats["gt_z"], indent=2))
    print("abs err z stats:", json.dumps(z_stats["abs_err_z"], indent=2))
    if Na > 0:
        print("abs err z ACTIVE stats:", json.dumps(z_stats["abs_err_z_active"], indent=2))
    print("Top-5 worst |err_z|:")
    for item in worst[:5]:
        print(item)


if __name__ == "__main__":
    main()
