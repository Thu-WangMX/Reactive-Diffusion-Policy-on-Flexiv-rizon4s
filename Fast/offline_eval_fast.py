#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
offline_eval_fast.py

Offline sanity-check for trained Fast model:
- Loads ckpt_best.pt
- Runs on FastResidualDataset(split="val")
- Reports:
  * mae dp3 / rot6 / all9 (overall + active-only)
  * delta_z stats (pred & gt): mean/std/min/max/quantiles
  * correlation of pred vs gt (dp3 + z)
  * magnitude ratio: |pred| / |gt| (sanity)
  * optional gate stats if NUM_SUBSPACE>1 and model returns aux["gate"]
- Saves:
  OUT_DIR/eval_fast_report.json
  OUT_DIR/eval_fast_npz.npz

No argparse. Edit CONFIG section.
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
OUT_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/train_fast_wiping_v2_run1"
SPLIT = "val"          # "val" or "train"
BATCH = 4096
NUM_WORKERS = 2
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# If you only care about z for wiping, you can set this True to zero-out other dims when computing extra sanity stats
ONLY_Z_SANITY = True


def _to_numpy(x: torch.Tensor) -> np.ndarray:
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
    x = x.reshape(-1).astype(np.float64)
    q = np.quantile(x, [0.0, 0.01, 0.05, 0.5, 0.95, 0.99, 1.0])
    return {
        "n": int(x.size),
        "mean": float(np.mean(x)),
        "std": float(np.std(x)),
        "min": float(q[0]),
        "q01": float(q[1]),
        "q05": float(q[2]),
        "median": float(q[3]),
        "q95": float(q[4]),
        "q99": float(q[5]),
        "max": float(q[6]),
    }


@torch.no_grad()
def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # -------- dataset / loader --------
    ds = FastResidualDataset(split=SPLIT)
    dl = DataLoader(ds, batch_size=BATCH, shuffle=False, num_workers=NUM_WORKERS, pin_memory=True, drop_last=False)

    # -------- model --------
    ckpt = torch.load(CKPT_PATH, map_location="cpu")
    model = FastResidualPolicy(d_base=ds.base_dim, d_paep=ds.paep_dim).to(DEVICE)

    missing, unexpected = model.load_state_dict(ckpt["model"], strict=False)
    if len(missing) > 0 or len(unexpected) > 0:
        print("[WARN] load_state_dict strict=False")
        print("  missing:", missing)
        print("  unexpected:", unexpected)

    model.eval()

    # -------- collect --------
    pred_list = []
    gt_list = []
    w_soft_list = []
    m_hard_list = []
    gate_list = []

    for base_h, wrench_h, paep_h, y, w_soft, m_hard, t_idx in dl:
        base_h = base_h.to(DEVICE, non_blocking=True)
        wrench_h = wrench_h.to(DEVICE, non_blocking=True)
        paep_h = paep_h.to(DEVICE, non_blocking=True)
        y = y.to(DEVICE, non_blocking=True)

        pred, aux = model(base_h, wrench_h, paep_h)

        pred_list.append(_to_numpy(pred))
        gt_list.append(_to_numpy(y))
        w_soft_list.append(_to_numpy(w_soft))
        m_hard_list.append(_to_numpy(m_hard))

        if "gate" in aux:
            gate_list.append(_to_numpy(aux["gate"]))

    pred = np.concatenate(pred_list, axis=0)   # (N,9)
    gt = np.concatenate(gt_list, axis=0)       # (N,9)
    w_soft = np.concatenate(w_soft_list, axis=0).reshape(-1)  # (N,)
    m_hard = np.concatenate(m_hard_list, axis=0).reshape(-1)  # (N,)

    active_mask = (m_hard > 0.5)
    N = int(pred.shape[0])
    Na = int(active_mask.sum())

    # -------- MAE metrics --------
    abs9 = np.abs(pred - gt)  # (N,9)
    mae_all9 = float(abs9.mean())
    mae_dp3 = float(abs9[:, :3].mean())
    mae_rot6 = float(abs9[:, 3:9].mean())

    if Na > 0:
        abs9_a = abs9[active_mask]
        mae_all9_a = float(abs9_a.mean())
        mae_dp3_a = float(abs9_a[:, :3].mean())
        mae_rot6_a = float(abs9_a[:, 3:9].mean())
    else:
        mae_all9_a = mae_dp3_a = mae_rot6_a = float("nan")

    # -------- z sanity --------
    pred_z = pred[:, 2]
    gt_z = gt[:, 2]

    pred_dp3 = pred[:, :3]
    gt_dp3 = gt[:, :3]

    # corr
    corr_z = _safe_corr(pred_z, gt_z)
    corr_dp3 = _safe_corr(pred_dp3.reshape(-1), gt_dp3.reshape(-1))

    # magnitude ratio (avoid divide-by-zero)
    eps = 1e-9
    mag_ratio_z = np.abs(pred_z) / (np.abs(gt_z) + eps)
    mag_ratio_dp3 = np.linalg.norm(pred_dp3, axis=1) / (np.linalg.norm(gt_dp3, axis=1) + eps)

    # optional: if you only want z, check x/y are small
    xy_abs_pred = np.abs(pred[:, :2]).mean(axis=0)  # mean |x|, mean |y|
    xy_abs_gt = np.abs(gt[:, :2]).mean(axis=0)

    # -------- gate stats --------
    gate_stats = None
    if len(gate_list) > 0:
        gate = np.concatenate(gate_list, axis=0)  # (N,M)
        gate_mean = gate.mean(axis=0)
        # entropy
        g = np.clip(gate, 1e-12, 1.0)
        ent = (-g * np.log(g)).sum(axis=1)  # (N,)
        gate_stats = {
            "gate_shape": list(gate.shape),
            "gate_mean": gate_mean.tolist(),
            "gate_entropy_mean": float(ent.mean()),
            "gate_entropy_std": float(ent.std()),
        }
    else:
        gate = None

    # -------- active-only extra stats --------
    if Na > 0:
        pred_z_a = pred_z[active_mask]
        gt_z_a = gt_z[active_mask]
        corr_z_a = _safe_corr(pred_z_a, gt_z_a)
    else:
        pred_z_a = gt_z_a = None
        corr_z_a = float("nan")

    # -------- report --------
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

        "mae": {
            "all9": mae_all9,
            "dp3": mae_dp3,
            "rot6": mae_rot6,
            "all9_active": mae_all9_a,
            "dp3_active": mae_dp3_a,
            "rot6_active": mae_rot6_a,
        },

        "z_stats": {
            "pred_z": _stats_1d(pred_z),
            "gt_z": _stats_1d(gt_z),
            "pred_z_active": _stats_1d(pred_z_a) if pred_z_a is not None else None,
            "gt_z_active": _stats_1d(gt_z_a) if gt_z_a is not None else None,
        },

        "corr": {
            "z": corr_z,
            "z_active": corr_z_a,
            "dp3_flat": corr_dp3,
        },

        "magnitude_ratio": {
            "z": _stats_1d(mag_ratio_z),
            "dp3_norm": _stats_1d(mag_ratio_dp3),
        },

        "xy_abs_mean": {
            "pred_xy": xy_abs_pred.tolist(),
            "gt_xy": xy_abs_gt.tolist(),
        },

        "gate_stats": gate_stats,
    }

    report_path = os.path.join(OUT_DIR, "eval_fast_report.json")
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    print("[OK] wrote:", report_path)
    print(json.dumps(report["mae"], indent=2))

    # save arrays for later plotting/debugging
    npz_path = os.path.join(OUT_DIR, "eval_fast_npz.npz")
    save_dict = {
        "pred": pred.astype(np.float32),
        "gt": gt.astype(np.float32),
        "w_soft": w_soft.astype(np.float32),
        "m_hard": m_hard.astype(np.float32),
    }
    if gate is not None:
        save_dict["gate"] = gate.astype(np.float32)
    np.savez_compressed(npz_path, **save_dict)
    print("[OK] wrote:", npz_path)


if __name__ == "__main__":
    main()
