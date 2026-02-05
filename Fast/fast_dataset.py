# fast_dataset.py
import json
import os
from typing import Optional, Sequence

import numpy as np
import torch
from torch.utils.data import Dataset


def _load_npy(x: str) -> np.ndarray:
    arr = np.load(x)
    if isinstance(arr, np.lib.npyio.NpzFile):
        k0 = list(arr.keys())[0]
        arr = arr[k0]
    return np.asarray(arr)


def _default_time_split(T: int, split: str) -> np.ndarray:
    # last 10% as val, rest as train
    n_val = max(1, int(0.1 * T))
    if split == "val":
        return np.arange(T - n_val, T, dtype=np.int64)
    return np.arange(0, T - n_val, dtype=np.int64)


def _parse_split_json(split_json: str, split: str, T: int) -> Optional[np.ndarray]:
    """
    Accept common formats:
    1) {"train": [...], "val": [...]}
    2) {"train_idx": [...], "val_idx": [...]}
    3) {"train": {"idx": [...]}, "val": {"idx": [...]}}
    If not recognized -> None.
    """
    if not split_json:
        return None
    if not os.path.exists(split_json):
        return None
    try:
        obj = json.load(open(split_json, "r"))
    except Exception:
        return None

    keys = [split, f"{split}_idx", f"{split}Idx", f"{split}_indices"]
    cand = None
    for k in keys:
        if k in obj:
            cand = obj[k]
            break
    if cand is None and (split in obj) and isinstance(obj[split], dict) and ("idx" in obj[split]):
        cand = obj[split]["idx"]
    if cand is None:
        return None

    try:
        idx = np.asarray(cand, dtype=np.int64).reshape(-1)
        idx = idx[(idx >= 0) & (idx < T)]
        if idx.size == 0:
            return None
        return np.unique(idx)
    except Exception:
        return None


class FastResidualDataset(Dataset):
    """
    FAST residual dataset.

    You want fz_target=-20N and “铁律”:
      fz_err_raw  = Fz - fz_target   (N)
      fz_err_norm = fz_err_raw / std_z  (std units)
    deadband is in RAW-N space: |fz_err_raw| < deadband -> 0

    IMPORTANT:
    - wrench_mean/std MUST be computed on RAW wrench (NOT shifted),
      because deploy/runtime expects raw stats + optional normalized-space z shift:
        w_norm_z -= fz_target/std_z
    """
    def __init__(
        self,
        base_abs_npy: str,
        wrench_npy: str,
        paep_npy: str,
        delta_rel_npy: str,
        split_json: str = "",
        split: str = "train",
        hist: int = 8,
        stride: int = 1,
        wrench_clip_norm: float = 6.0,
        wrench_mean: Optional[Sequence[float]] = None,
        wrench_std: Optional[Sequence[float]] = None,
        fz_target: float = -20.0,
        fz_deadband: float = 0.5,
    ):
        assert split in ("train", "val", "test"), f"split must be train/val/test, got {split}"
        self.split = split
        self.hist = int(hist)
        self.stride = max(1, int(stride))
        self.wrench_clip_norm = float(wrench_clip_norm)

        self.fz_target = float(fz_target)
        self.fz_deadband = float(fz_deadband)

        base = _load_npy(base_abs_npy).astype(np.float32)     # (T,9)
        wrench = _load_npy(wrench_npy).astype(np.float32)     # (T,6) raw
        paep = _load_npy(paep_npy).astype(np.float32)         # (T,d)
        y = _load_npy(delta_rel_npy).astype(np.float32)       # (T,9)

        assert base.ndim == 2 and base.shape[1] == 9, f"base_abs9d must be (T,9), got {base.shape}"
        assert wrench.ndim == 2 and wrench.shape[1] >= 6, f"wrench must be (T,6+), got {wrench.shape}"
        assert y.ndim == 2 and y.shape[1] == 9, f"delta_rel must be (T,9), got {y.shape}"
        assert base.shape[0] == wrench.shape[0] == paep.shape[0] == y.shape[0], "T mismatch among inputs"

        self.base = base
        self.wrench_raw = wrench[:, :6]
        self.paep = paep
        self.y = y
        self.T = int(base.shape[0])

        idx = _parse_split_json(split_json, split, self.T)
        if idx is None:
            idx = _default_time_split(self.T, "val" if split == "val" else "train")
        idx = idx[::self.stride]
        self.idxs = idx.astype(np.int64)

        # RAW-space mean/std (NO shift here)
        if (wrench_mean is None) or (wrench_std is None):
            w = self.wrench_raw[self.idxs]
            self.wrench_mean = w.mean(axis=0).astype(np.float32)
            self.wrench_std = w.std(axis=0).astype(np.float32)
        else:
            self.wrench_mean = np.asarray(wrench_mean, dtype=np.float32).reshape(-1)[:6]
            self.wrench_std = np.asarray(wrench_std, dtype=np.float32).reshape(-1)[:6]
        self.wrench_std = np.maximum(self.wrench_std, 1e-6)

        self.d_base = int(self.base.shape[1])
        self.d_paep = int(self.paep.shape[1])

    def __len__(self) -> int:
        return int(self.idxs.shape[0])

    def _window(self, arr: np.ndarray, t: int, dim: int) -> np.ndarray:
        t0 = max(0, t - self.hist + 1)
        x = arr[t0:t + 1]
        if x.shape[0] < self.hist:
            pad = np.repeat(x[:1], self.hist - x.shape[0], axis=0)
            x = np.concatenate([pad, x], axis=0)
        assert x.shape == (self.hist, dim), f"window expects {(self.hist, dim)}, got {x.shape}"
        return x

    def __getitem__(self, i: int):
        t = int(self.idxs[i])

        base_h = self._window(self.base, t, self.d_base)          # (H,9)
        wrench_h_raw = self._window(self.wrench_raw, t, 6)        # (H,6) raw
        paep_h = self._window(self.paep, t, self.d_paep)          # (H,d)

        w_norm = (wrench_h_raw - self.wrench_mean[None]) / self.wrench_std[None]
        w_norm = np.clip(w_norm, -self.wrench_clip_norm, self.wrench_clip_norm).astype(np.float32)

        y = self.y[t].astype(np.float32)                          # (9,)

        # default useful gate weights:
        pc = float(paep_h[-1, 0]) if (self.d_paep >= 1) else 0.0
        pc = float(np.clip(pc, 0.0, 1.0))
        w_soft = np.asarray([pc], dtype=np.float32)
        m_hard = np.asarray([1.0 if pc > 0.5 else 0.0], dtype=np.float32)

        # RAW + NORM err (deadband in RAW-N space)
        fz = float(wrench_h_raw[-1, 2])
        fz_err_raw = float(fz - self.fz_target)
        if abs(fz_err_raw) < self.fz_deadband:
            fz_err_raw = 0.0
        std_z = float(self.wrench_std[2])
        fz_err_norm = float(fz_err_raw / max(std_z, 1e-6))

        return (
            torch.from_numpy(base_h),                        # (H,9)
            torch.from_numpy(w_norm),                        # (H,6)
            torch.from_numpy(paep_h),                        # (H,d)
            torch.from_numpy(y),                             # (9,)
            torch.from_numpy(w_soft),                        # (1,)
            torch.from_numpy(m_hard),                        # (1,)
            torch.tensor(t, dtype=torch.int64),              # scalar
            torch.tensor(fz_err_raw, dtype=torch.float32),   # scalar (N)
            torch.tensor(fz_err_norm, dtype=torch.float32),  # scalar (std)
        )
