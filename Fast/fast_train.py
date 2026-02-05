#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
fast_train.py (NO argparse)  ✅ wrench from ZARR

Inputs:
- teacherA_slow_exec_9d.npy  -> base_abs9d (T,9)
- teacherA_fast_label_9d.npy -> delta_rel_9d (T,9)  (EE-frame residual label)
- paep_prob_contact.npy + paep_prob_phase.npy -> paep_feat (T, 1+K)
- wrench6 from replay_buffer.zarr (T,6) raw

Key rule (铁律):
- fz_err_raw = Fz - fz_target
  >0 => force too small  => PRESS_DOWN => dz_ee should be POSITIVE
  <0 => force too large  => LIFT_UP    => dz_ee should be NEGATIVE
  =0 (deadband)          => HOLD       => don't enforce sign

We compute wrench mean/std on RAW wrench (train split), and optionally apply
normalized-space shift for model input:
    w_norm_z_in = w_norm_z - (fz_target / std_z)
so model "sees" error-to-target consistently with deploy runtime.
"""

import os
import json
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple, Dict, Any

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
from tqdm import tqdm
from loguru import logger

import zarr

from fast_model import FastResidualPolicy


# ============================================================
# CONFIG (只改这里)
# ============================================================

# ---- zarr ----
ZARR_PATH = "/ABS/PATH/TO/replay_buffer.zarr"  # TODO: 改成你的 zarr
WRENCH_KEY_CANDIDATES = [
    "data/left_robot_tcp_wrench"

]

# ---- teacher / paep npy (你现在就有这些) ----
BASE_ABS_9D_NPY = "Fast/label_for_fast/Teacher_A/offline_wiping_board_teacherA_out/teacherA_slow_exec_9d.npy"
DELTA_REL_9D_NPY = "Fast/label_for_fast/Teacher_A/offline_wiping_board_teacherA_out/teacherA_fast_label_9d.npy"
PAEP_CONTACT_NPY = "Fast/label_for_fast/offline_wiping_board_paep_prob_out/paep_prob_contact.npy"
PAEP_PHASE_NPY   = "Fast/label_for_fast/offline_wiping_board_paep_prob_out/paep_prob_phase.npy"

# ---- dataset windowing ----
HIST = 8
STRIDE = 1
VAL_RATIO = 0.10

# ---- wrench normalize/clip ----
WRENCH_CLIP_NORM = 6.0

# ---- force target & deadband ----
FZ_TARGET = -20.0
FZ_DEADBAND = 0.5

# ---- train ----
DEVICE = "cuda"
BATCH_SIZE = 256
EPOCHS = 20
LR = 3e-4
NUM_WORKERS = 4
GRAD_CLIP = 1.0

# ---- shift-to-target (你要的“铁律输入口径”) ----
APPLY_FZ_TARGET_SHIFT = True  # 强烈建议 True：train==deploy

# ---- sign loss (铁律约束) ----
LAMBDA_SIGN = 0.3
FZ_SCALE_N = 10.0     # N: tanh scale in raw-N semantics
DZ_SCALE_M = 0.003    # m: should match deploy max_delta_xyz
SIGN_MARGIN = 0.10

# ---- save ----
SAVE_DIR = "Fast/ckpts/wiping_fast_v1"
SAVE_NAME = "fast_residual_best"

# ============================================================


def _open_zarr_root(path: str):
    if not os.path.exists(path):
        raise FileNotFoundError(f"[FAST] ZARR_PATH not found: {path}")
    return zarr.open(path, mode="r")


def _find_key(root, candidates):
    for k in candidates:
        try:
            _ = root[k]
            return k
        except Exception:
            pass
    return None


class FastZarrDataset(Dataset):
    """
    Build training samples by indexing time t:
      base_hist (H,9), wrench_hist_norm (H,6), paep_hist (H,d), label (9)
    Also returns:
      w_soft (p_contact), m_hard (p_contact>0.5),
      fz_err_raw (deadbanded), fz_err_norm (deadbanded/std_z)
    """
    def __init__(
        self,
        base_abs9d: np.ndarray,      # (T,9)
        wrench6_raw: np.ndarray,     # (T,6)
        paep_feat: np.ndarray,       # (T,d)
        delta_rel9d: np.ndarray,     # (T,9)
        split: str,
        hist: int,
        stride: int,
        val_ratio: float,
        wrench_clip_norm: float,
        fz_target: float,
        fz_deadband: float,
        wrench_mean: Optional[np.ndarray] = None,
        wrench_std: Optional[np.ndarray] = None,
    ):
        assert split in ("train", "val")
        self.split = split
        self.hist = int(hist)
        self.stride = max(1, int(stride))
        self.val_ratio = float(val_ratio)
        self.wrench_clip_norm = float(wrench_clip_norm)
        self.fz_target = float(fz_target)
        self.fz_deadband = float(fz_deadband)

        # align length
        T = min(base_abs9d.shape[0], wrench6_raw.shape[0], paep_feat.shape[0], delta_rel9d.shape[0])
        self.base = base_abs9d[:T].astype(np.float32)
        self.wrench_raw = wrench6_raw[:T].astype(np.float32)
        self.paep = paep_feat[:T].astype(np.float32)
        self.y = delta_rel9d[:T].astype(np.float32)
        self.T = T

        # split indices (time split)
        n_val = max(1, int(self.T * self.val_ratio))
        if split == "val":
            idx = np.arange(self.T - n_val, self.T, dtype=np.int64)
        else:
            idx = np.arange(0, self.T - n_val, dtype=np.int64)
        idx = idx[::self.stride]
        self.idxs = idx

        # wrench norm stats must be RAW stats (NOT shifted)
        if wrench_mean is None or wrench_std is None:
            w = self.wrench_raw[self.idxs]
            self.wrench_mean = w.mean(axis=0).astype(np.float32)
            self.wrench_std = w.std(axis=0).astype(np.float32)
        else:
            self.wrench_mean = np.asarray(wrench_mean, dtype=np.float32).reshape(-1)[:6]
            self.wrench_std = np.asarray(wrench_std, dtype=np.float32).reshape(-1)[:6]
        self.wrench_std = np.maximum(self.wrench_std, 1e-6)

        self.d_base = int(self.base.shape[1])
        self.d_paep = int(self.paep.shape[1])

    def __len__(self):
        return int(self.idxs.shape[0])

    def _window(self, arr: np.ndarray, t: int, dim: int):
        t0 = max(0, t - self.hist + 1)
        x = arr[t0:t + 1]
        if x.shape[0] < self.hist:
            pad = np.repeat(x[:1], self.hist - x.shape[0], axis=0)
            x = np.concatenate([pad, x], axis=0)
        assert x.shape == (self.hist, dim)
        return x

    def __getitem__(self, i: int):
        t = int(self.idxs[i])

        base_h = self._window(self.base, t, self.d_base)          # (H,9)
        wrench_h_raw = self._window(self.wrench_raw, t, 6)        # (H,6)
        paep_h = self._window(self.paep, t, self.d_paep)          # (H,d)
        y = self.y[t]                                             # (9,)

        # normalize wrench (RAW stats)
        w_norm = (wrench_h_raw - self.wrench_mean[None]) / self.wrench_std[None]
        w_norm = np.clip(w_norm, -self.wrench_clip_norm, self.wrench_clip_norm).astype(np.float32)

        # gate weights from contact prob (assume paep_feat[:,0]=p_contact)
        pc = float(paep_h[-1, 0]) if self.d_paep >= 1 else 0.0
        pc = float(np.clip(pc, 0.0, 1.0))
        w_soft = np.array([pc], dtype=np.float32)
        m_hard = np.array([1.0 if pc > 0.5 else 0.0], dtype=np.float32)

        # raw / norm err (deadbanded in RAW N)
        fz = float(wrench_h_raw[-1, 2])
        fz_err_raw = float(fz - self.fz_target)
        if abs(fz_err_raw) < self.fz_deadband:
            fz_err_raw = 0.0
        fz_err_norm = float(fz_err_raw / float(self.wrench_std[2]))

        return (
            torch.from_numpy(base_h),                  # (H,9)
            torch.from_numpy(w_norm),                  # (H,6)
            torch.from_numpy(paep_h),                  # (H,d)
            torch.from_numpy(y),                       # (9,)
            torch.from_numpy(w_soft),                  # (1,)
            torch.from_numpy(m_hard),                  # (1,)
            torch.tensor(t, dtype=torch.int64),        # ()
            torch.tensor(fz_err_raw, dtype=torch.float32),   # ()
            torch.tensor(fz_err_norm, dtype=torch.float32),  # ()
        )


def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    device = DEVICE if DEVICE else ("cuda" if torch.cuda.is_available() else "cpu")

    # -------- load teacher/paep npy --------
    base_abs = np.load(BASE_ABS_9D_NPY).astype(np.float32)          # (T,9)
    delta_rel = np.load(DELTA_REL_9D_NPY).astype(np.float32)        # (T,9)
    pc = np.load(PAEP_CONTACT_NPY).astype(np.float32).reshape(-1, 1)
    pp = np.load(PAEP_PHASE_NPY).astype(np.float32)
    pp = pp.reshape(pp.shape[0], -1)
    paep_feat = np.concatenate([pc, pp], axis=1).astype(np.float32) # (T,1+K)

    # -------- load wrench from zarr --------
    root = _open_zarr_root(ZARR_PATH)
    wrench_key = _find_key(root, WRENCH_KEY_CANDIDATES)
    if wrench_key is None:
        # give user a concrete tree to pick from
        try:
            logger.error("[FAST] cannot find wrench key. Zarr tree:\n" + str(root.tree()))
        except Exception:
            logger.error("[FAST] cannot find wrench key and cannot print tree.")
        raise RuntimeError("Cannot find wrench key in zarr. Add correct key into WRENCH_KEY_CANDIDATES.")

    wrench = np.asarray(root[wrench_key][:], dtype=np.float32)
    wrench6 = wrench[:, :6].astype(np.float32)

    # -------- dataset --------
    ds_tr = FastZarrDataset(
        base_abs9d=base_abs,
        wrench6_raw=wrench6,
        paep_feat=paep_feat,
        delta_rel9d=delta_rel,
        split="train",
        hist=HIST,
        stride=STRIDE,
        val_ratio=VAL_RATIO,
        wrench_clip_norm=WRENCH_CLIP_NORM,
        fz_target=FZ_TARGET,
        fz_deadband=FZ_DEADBAND,
    )
    ds_va = FastZarrDataset(
        base_abs9d=base_abs,
        wrench6_raw=wrench6,
        paep_feat=paep_feat,
        delta_rel9d=delta_rel,
        split="val",
        hist=HIST,
        stride=STRIDE,
        val_ratio=VAL_RATIO,
        wrench_clip_norm=WRENCH_CLIP_NORM,
        fz_target=FZ_TARGET,
        fz_deadband=FZ_DEADBAND,
        wrench_mean=ds_tr.wrench_mean,
        wrench_std=ds_tr.wrench_std,
    )

    dl_tr = DataLoader(ds_tr, batch_size=BATCH_SIZE, shuffle=True, num_workers=NUM_WORKERS, pin_memory=True, drop_last=True)
    dl_va = DataLoader(ds_va, batch_size=BATCH_SIZE, shuffle=False, num_workers=max(1, NUM_WORKERS // 2), pin_memory=True, drop_last=False)

    # -------- model --------
    model = FastResidualPolicy(d_base=ds_tr.d_base, d_paep=ds_tr.d_paep).to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=LR)

    std_z = float(ds_tr.wrench_std[2])
    mean_z = float(ds_tr.wrench_mean[2])
    shift_norm_z = float(FZ_TARGET) / max(std_z, 1e-6)  # used as: w_norm_z_in = w_norm_z - shift_norm_z

    logger.info(f"[FAST][TRAIN] device={device}")
    logger.info(f"[FAST][TRAIN] zarr={ZARR_PATH} key={wrench_key}")
    logger.info(f"[FAST][TRAIN] T={ds_tr.T} hist={HIST} d_base={ds_tr.d_base} d_paep={ds_tr.d_paep}")
    logger.info(f"[FAST][TRAIN] wrench_mean_z={mean_z:.3f} wrench_std_z={std_z:.3f}")
    logger.info(f"[FAST][TRAIN] fz_target={FZ_TARGET:.3f} deadband={FZ_DEADBAND:.3f} apply_shift={APPLY_FZ_TARGET_SHIFT} shift_norm_z={shift_norm_z:.6f}")
    logger.info(f"[FAST][TRAIN] lambda_sign={LAMBDA_SIGN} fz_scale_N={FZ_SCALE_N} dz_scale_m={DZ_SCALE_M} margin={SIGN_MARGIN}")

    best_val = 1e18
    best_ckpt = os.path.join(SAVE_DIR, f"{SAVE_NAME}.pt")
    best_meta = os.path.join(SAVE_DIR, f"{SAVE_NAME}.meta.json")

    for ep in range(EPOCHS):
        # ==================== train ====================
        model.train()
        tr_loss = tr_l1 = tr_sign = tr_sign_acc = 0.0
        tr_n = 0

        pbar = tqdm(dl_tr, desc=f"Train ep {ep}", dynamic_ncols=True)
        for batch in pbar:
            base_h, w_norm, paep_h, y, w_soft, m_hard, t, fz_err_raw, fz_err_norm = batch
            base_h = base_h.to(device)
            w_norm = w_norm.to(device)
            paep_h = paep_h.to(device)
            y = y.to(device)
            w_soft = w_soft.to(device).reshape(-1)
            m_hard = m_hard.to(device).reshape(-1)
            fz_err_raw = fz_err_raw.to(device).reshape(-1)
            fz_err_norm = fz_err_norm.to(device).reshape(-1)

            # model input wrench: optional shift in normalized space (train==deploy)
            if APPLY_FZ_TARGET_SHIFT:
                w_in = w_norm.clone()
                w_in[:, :, 2] = w_in[:, :, 2] - shift_norm_z
            else:
                w_in = w_norm

            pred, aux = model(base_h, w_in, paep_h)

            # main regression (weighted by contact prob)
            l1_each = torch.abs(pred - y).mean(dim=1)
            loss_l1 = (w_soft * l1_each).mean()

            # sign loss with deadband-aware gating
            loss_sign = torch.zeros((), device=device)
            sign_acc = torch.zeros((), device=device)

            if LAMBDA_SIGN > 0:
                dz = pred[:, 2]

                active_err = (fz_err_raw.abs() > 0).float()
                # keep fz_scale in N semantics but use norm err for stability:
                fz_scale_norm = float(FZ_SCALE_N) / max(std_z, 1e-6)
                fz_err_n = torch.tanh(fz_err_norm / max(fz_scale_norm, 1e-6))
                dz_n = dz / float(DZ_SCALE_M)

                prod = dz_n * fz_err_n
                hinge = F.relu(float(SIGN_MARGIN) - prod)

                w = w_soft * active_err
                if w.sum() > 1e-6:
                    loss_sign = (w * hinge).sum() / (w.sum() + 1e-6)

                active = (w_soft > 0.2).float() * active_err
                if active.sum() > 0.5:
                    ok = ((dz * fz_err_raw) > 0).float()
                    sign_acc = (active * ok).sum() / (active.sum() + 1e-6)

            loss = loss_l1 + float(LAMBDA_SIGN) * loss_sign

            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), GRAD_CLIP)
            opt.step()

            B = int(pred.shape[0])
            tr_loss += float(loss.detach().cpu()) * B
            tr_l1 += float(loss_l1.detach().cpu()) * B
            tr_sign += float(loss_sign.detach().cpu()) * B
            tr_sign_acc += float(sign_acc.detach().cpu()) * B
            tr_n += B

            pbar.set_postfix({
                "loss": tr_loss / max(1, tr_n),
                "l1": tr_l1 / max(1, tr_n),
                "sign": tr_sign / max(1, tr_n),
                "sign_acc": tr_sign_acc / max(1, tr_n),
            })

        tr_loss /= max(1, tr_n)
        tr_l1 /= max(1, tr_n)
        tr_sign /= max(1, tr_n)
        tr_sign_acc /= max(1, tr_n)

        # ==================== val ====================
        model.eval()
        va_loss = va_l1 = va_sign = va_sign_acc = 0.0
        va_n = 0

        with torch.no_grad():
            for batch in tqdm(dl_va, desc=f"Val ep {ep}", dynamic_ncols=True):
                base_h, w_norm, paep_h, y, w_soft, m_hard, t, fz_err_raw, fz_err_norm = batch
                base_h = base_h.to(device)
                w_norm = w_norm.to(device)
                paep_h = paep_h.to(device)
                y = y.to(device)
                w_soft = w_soft.to(device).reshape(-1)
                fz_err_raw = fz_err_raw.to(device).reshape(-1)
                fz_err_norm = fz_err_norm.to(device).reshape(-1)

                if APPLY_FZ_TARGET_SHIFT:
                    w_in = w_norm.clone()
                    w_in[:, :, 2] = w_in[:, :, 2] - shift_norm_z
                else:
                    w_in = w_norm

                pred, aux = model(base_h, w_in, paep_h)

                l1_each = torch.abs(pred - y).mean(dim=1)
                loss_l1 = (w_soft * l1_each).mean()

                loss_sign = torch.zeros((), device=device)
                sign_acc = torch.zeros((), device=device)

                if LAMBDA_SIGN > 0:
                    dz = pred[:, 2]
                    active_err = (fz_err_raw.abs() > 0).float()

                    fz_scale_norm = float(FZ_SCALE_N) / max(std_z, 1e-6)
                    fz_err_n = torch.tanh(fz_err_norm / max(fz_scale_norm, 1e-6))
                    dz_n = dz / float(DZ_SCALE_M)

                    prod = dz_n * fz_err_n
                    hinge = F.relu(float(SIGN_MARGIN) - prod)

                    w = w_soft * active_err
                    if w.sum() > 1e-6:
                        loss_sign = (w * hinge).sum() / (w.sum() + 1e-6)

                    active = (w_soft > 0.2).float() * active_err
                    if active.sum() > 0.5:
                        ok = ((dz * fz_err_raw) > 0).float()
                        sign_acc = (active * ok).sum() / (active.sum() + 1e-6)

                loss = loss_l1 + float(LAMBDA_SIGN) * loss_sign

                B = int(pred.shape[0])
                va_loss += float(loss.detach().cpu()) * B
                va_l1 += float(loss_l1.detach().cpu()) * B
                va_sign += float(loss_sign.detach().cpu()) * B
                va_sign_acc += float(sign_acc.detach().cpu()) * B
                va_n += B

        va_loss /= max(1, va_n)
        va_l1 /= max(1, va_n)
        va_sign /= max(1, va_n)
        va_sign_acc /= max(1, va_n)

        logger.info(
            f"[EP {ep}] train: loss={tr_loss:.6f} l1={tr_l1:.6f} sign={tr_sign:.6f} sign_acc={tr_sign_acc:.3f} | "
            f"val: loss={va_loss:.6f} l1={va_l1:.6f} sign={va_sign:.6f} sign_acc={va_sign_acc:.3f}"
        )

        # ==================== save best ====================
        if va_loss < best_val:
            best_val = va_loss
            torch.save({"model": model.state_dict()}, best_ckpt)

            meta = {
                "HIST": int(HIST),
                "d_base": int(ds_tr.d_base),
                "d_paep": int(ds_tr.d_paep),
                "wrench_norm": {
                    "mean": [float(x) for x in ds_tr.wrench_mean.tolist()],
                    "std":  [float(x) for x in ds_tr.wrench_std.tolist()],
                },
                "fz_target": float(FZ_TARGET),
                "fz_deadband": float(FZ_DEADBAND),
                "apply_fz_target_shift_norm": bool(APPLY_FZ_TARGET_SHIFT),
                "fz_target_shift_norm_z": float(shift_norm_z),
                "lambda_sign": float(LAMBDA_SIGN),
                "fz_scale_N": float(FZ_SCALE_N),
                "dz_scale_m": float(DZ_SCALE_M),
                "sign_margin": float(SIGN_MARGIN),
            }
            with open(best_meta, "w") as f:
                json.dump(meta, f, indent=2)

            logger.info(f"[SAVE] best_ckpt={best_ckpt}")
            logger.info(f"[SAVE] best_meta={best_meta}")

    logger.info(f"[DONE] best_val={best_val:.6f}")


if __name__ == "__main__":
    main()
