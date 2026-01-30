#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
fast_dataset.py (v2)

Task-specific Fast residual dataset.
- Inputs (history window HIST):
  base_abs_9d(t): teacherA slow exec absolute pose (9D)
  wrench_hist(t): (HIST, 6) normalized + clipped
  paep_prob(t): p_contact (1) + p_phase (K) as soft semantic condition
  (optional) pose_err_9d(t): inv(base_abs) * meas_abs  (kept as interface, off by default)

- Targets:
  delta_rel_9d(t): TeacherB residual label (9D)

- Provides:
  sample_weight(t): soft weight = base + active_boost * [p_contact * sum(p_phase over active phases)]
  hard_active(t): (p_contact>=0.5) & (argmax(p_phase) in active phases)  (for stats/sampling)
"""

import os
import numpy as np
import torch
from torch.utils.data import Dataset

from reactive_diffusion_policy.common.replay_buffer import ReplayBuffer


# =========================
# Config (edit here)
# =========================
# --- data paths ---
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr"

# teacherB label (relative residual 9D)
TEACHERB_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/label_for_fast/offline_wiping_board_teacherB_out"
RB_LABEL_PATH = os.path.join(TEACHERB_DIR, "teacherB_fast_label_9d.npy")  # (T,9)

# teacherA slow exec abs 9D (as base input)
TEACHERA_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/label_for_fast/Teacher_A/offline_wiping_board_teacherA_out"
BASE_ABS_PATH = os.path.join(TEACHERA_DIR, "teacherA_slow_exec_9d.npy")   # (T,9)

# (optional) measured abs 9D for pose_err interface (keep for future; off for now)
USE_MEAS_ABS = False
MEAS_ABS_PATH = os.path.join(TEACHERA_DIR, "teacherA_meas_abs_9d.npy")    # (T,9) if you have it

# PAEP offline prob output dir (you already have)
PAEP_PROB_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/label_for_fast/offline_wiping_board_paep_prob_out"
PAEP_PHASE_PROB_PATH = os.path.join(PAEP_PROB_DIR, "paep_prob_phase.npy")     # (T,K)
PAEP_CONTACT_PROB_PATH = os.path.join(PAEP_PROB_DIR, "paep_prob_contact.npy") # (T,1)

# zarr key
KEY_WRENCH = "left_robot_tcp_wrench"   # (T,6)

# --- task phases (per-task config) ---
# wiping: 3 phases
PHASE_NAMES = ["approach", "progress", "done"]
ACTIVE_PHASE_NAMES = ["progress"]          # emphasize correction learning in these phases

# plug-in example (later you can switch):
# PHASE_NAMES = ["approach", "search", "retreat", "insert"]
# ACTIVE_PHASE_NAMES = ["search", "insert"]

# --- temporal window ---
HIST = 8
STRIDE = 1

# --- split ---
SEED = 42
VAL_RATIO = 0.1

# --- wrench normalize + clip ---
WRENCH_EPS = 1e-6
WRENCH_CLIP_NORM = 6.0   # clip AFTER normalization: clip(z, -c, c)

# --- weight / active definition ---
CONTACT_TH = 0.5  # hard-active threshold on p_contact


def _build_hist(x: np.ndarray, hist: int) -> np.ndarray:
    """
    Right-aligned history windows:
      x: (T,D) -> out: (T,hist,D)
    Pad by repeating first available sample.
    """
    assert x.ndim == 2
    T, D = x.shape
    out = np.zeros((T, hist, D), dtype=np.float32)
    for t in range(T):
        s = max(0, t - hist + 1)
        chunk = x[s:t+1]
        if chunk.shape[0] < hist:
            pad = np.repeat(chunk[:1], repeats=hist - chunk.shape[0], axis=0)
            chunk = np.concatenate([pad, chunk], axis=0)
        out[t] = chunk
    return out


def _infer_active_phase_indices(phase_names, active_phase_names):
    name2idx = {n: i for i, n in enumerate(phase_names)}
    idx = []
    for n in active_phase_names:
        if n not in name2idx:
            raise ValueError(f"ACTIVE_PHASE_NAMES contains '{n}', but not in PHASE_NAMES={phase_names}")
        idx.append(name2idx[n])
    return idx


class FastResidualDataset(Dataset):
    """
    Returns:
      base_hist:   (HIST, D_base)  where D_base = 9 (+9 if USE_MEAS_ABS)
      wrench_hist: (HIST, 6) normalized+clipped
      paep_hist:   (HIST, 1+K) [p_contact, p_phase(K)]
      y:           (9,)
      weight:      scalar float
      hard_active: scalar float {0,1}
      t:           int64
    """
    def __init__(self, split="train"):
        assert split in ["train", "val"]
        self.split = split

        self.phase_names = list(PHASE_NAMES)
        self.active_phase_idx = _infer_active_phase_indices(self.phase_names, ACTIVE_PHASE_NAMES)
        self.phase_dim = len(self.phase_names)

        # ---------- load labels / base ----------
        y = np.load(RB_LABEL_PATH).astype(np.float32)         # (T,9)
        base_abs = np.load(BASE_ABS_PATH).astype(np.float32)  # (T,9)
        assert y.shape[0] == base_abs.shape[0] and y.shape[1] == 9 and base_abs.shape[1] == 9
        T = int(y.shape[0])

        # optional measured abs pose
        if USE_MEAS_ABS:
            meas_abs = np.load(MEAS_ABS_PATH).astype(np.float32)
            assert meas_abs.shape == base_abs.shape, f"meas_abs shape {meas_abs.shape} != base_abs {base_abs.shape}"
        else:
            meas_abs = None

        # ---------- load PAEP probs ----------
        p_phase = np.load(PAEP_PHASE_PROB_PATH).astype(np.float32)        # (T,K)
        p_contact = np.load(PAEP_CONTACT_PROB_PATH).astype(np.float32)    # (T,1) or (T,)
        if p_contact.ndim == 1:
            p_contact = p_contact[:, None]
        assert p_phase.shape[0] == T and p_contact.shape[0] == T
        assert p_phase.shape[1] == self.phase_dim, \
            f"PAEP phase prob dim mismatch: got {p_phase.shape[1]}, expect {self.phase_dim} from PHASE_NAMES"

        # ---------- load wrench from zarr ----------
        rbz = ReplayBuffer.copy_from_path(ZARR_PATH, keys=[KEY_WRENCH])
        wrench = np.asarray(rbz[KEY_WRENCH][:, :6], dtype=np.float32)
        assert wrench.shape[0] == T and wrench.shape[1] == 6

        # ---------- compute wrench norm (global) ----------
        wrench_mean = wrench.mean(axis=0)
        wrench_std = wrench.std(axis=0)
        wrench_std = np.maximum(wrench_std, WRENCH_EPS)

        wrench_norm = (wrench - wrench_mean[None, :]) / wrench_std[None, :]
        # clip
        wrench_norm_clip = np.clip(wrench_norm, -WRENCH_CLIP_NORM, WRENCH_CLIP_NORM).astype(np.float32)

        # clip stats (for logging)
        clip_ratio = float(np.mean(np.abs(wrench_norm) > WRENCH_CLIP_NORM))

        # ---------- build base features ----------
        # base_abs_9d always included
        base_feat = base_abs.astype(np.float32)

        # pose_err interface (optional): inv(base_abs) * meas_abs
        # NOTE: keep interface only; for now if USE_MEAS_ABS=False -> not included
        if USE_MEAS_ABS:
            # minimal (safe) approx: use delta in xyz only; rot err left as zeros
            # (you can later replace with proper SE(3) relative computation for rot6d)
            pose_err = np.zeros_like(base_abs, dtype=np.float32)
            pose_err[:, :3] = (meas_abs[:, :3] - base_abs[:, :3])
            base_feat = np.concatenate([base_feat, pose_err], axis=1).astype(np.float32)  # (T,18)

        # ---------- build paep features per step ----------
        paep_feat = np.concatenate([p_contact, p_phase], axis=1).astype(np.float32)  # (T, 1+K)

        # ---------- compute weights / hard active ----------
        phase_pred = np.argmax(p_phase, axis=1).astype(np.int64)
        hard_active = (p_contact[:, 0] >= CONTACT_TH) & np.isin(phase_pred, np.array(self.active_phase_idx, dtype=np.int64))
        self.hard_active = hard_active.astype(np.float32)

        active_prob = p_contact[:, 0] * np.sum(p_phase[:, self.active_phase_idx], axis=1)  # soft in [0,1]
        self.active_prob = active_prob.astype(np.float32)

        # ---------- build history windows ----------
        self.base_hist = _build_hist(base_feat, HIST)                 # (T,HIST,D_base)
        self.wrench_hist = _build_hist(wrench_norm_clip, HIST)        # (T,HIST,6)
        self.paep_hist = _build_hist(paep_feat, HIST)                 # (T,HIST,1+K)

        self.y = y
        self.base_abs = base_abs
        self.T = T

        # store norm for train script (for reproducibility)
        self.wrench_mean = wrench_mean.astype(np.float32)
        self.wrench_std = wrench_std.astype(np.float32)
        self.wrench_clip_ratio = clip_ratio

        # ---------- indices / split ----------
        valid = np.arange(0, T, STRIDE, dtype=np.int64)
        rng = np.random.RandomState(SEED)
        perm = rng.permutation(valid)
        n_val = int(len(perm) * VAL_RATIO)
        val_idx = perm[:n_val]
        train_idx = perm[n_val:]
        self.indices = train_idx if split == "train" else val_idx

    @property
    def base_dim(self) -> int:
        return int(self.base_hist.shape[-1])

    @property
    def paep_dim(self) -> int:
        return int(self.paep_hist.shape[-1])

    def __len__(self):
        return int(self.indices.shape[0])

    def __getitem__(self, i: int):
        t = int(self.indices[i])
        base_h = self.base_hist[t]       # (HIST, D_base)
        wrench_h = self.wrench_hist[t]   # (HIST, 6)
        paep_h = self.paep_hist[t]       # (HIST, 1+K)
        y = self.y[t]                    # (9,)
        w = self.active_prob[t]          # [0,1] soft
        m = self.hard_active[t]          # {0,1}
        return (
            torch.from_numpy(base_h),
            torch.from_numpy(wrench_h),
            torch.from_numpy(paep_h),
            torch.from_numpy(y),
            torch.tensor(w, dtype=torch.float32),
            torch.tensor(m, dtype=torch.float32),
            torch.tensor(t, dtype=torch.int64),
        )
