# PAEP/paep_train/paep_dataset.py
import json
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

import numpy as np
import torch
from torch.utils.data import Dataset

import zarr


EVENT_NAMES = ["idle", "approach", "under", "effective", "over", "retreat"]
NUM_EVENTS = len(EVENT_NAMES)


def load_split_ids(split_json: str, split: str) -> List[int]:
    with open(split_json, "r", encoding="utf-8") as f:
        obj = json.load(f)
    assert split in obj, f"split must be in {list(obj.keys())}"
    return list(map(int, obj[split]))


def _open_group(zarr_path: str):
    root = zarr.open(zarr_path, mode="r")
    # auto-detect whether keys are at root or under "data"
    if isinstance(root, zarr.hierarchy.Group):
        if "external_img" in root.array_keys():
            return root, "root"
        if "data" in root.group_keys():
            g = root["data"]
            if "external_img" in g.array_keys():
                return g, "data"
    raise KeyError("Cannot find arrays. Expect external_img at root or under group 'data'.")


def _episode_slices(episode_ends: np.ndarray) -> List[Tuple[int, int]]:
    # episode_ends: shape (E,), each is end index (exclusive)
    ends = episode_ends.astype(np.int64).tolist()
    slices = []
    s = 0
    for e in ends:
        slices.append((s, e))
        s = e
    return slices


@dataclass
class NormStats:
    wrench_mean: np.ndarray  # (6,)
    wrench_std: np.ndarray   # (6,)
    pose_mean: np.ndarray    # (9,)
    pose_std: np.ndarray     # (9,)

    def to_torch(self, device="cpu"):
        return {
            "wrench_mean": torch.tensor(self.wrench_mean, dtype=torch.float32, device=device),
            "wrench_std": torch.tensor(self.wrench_std, dtype=torch.float32, device=device),
            "pose_mean": torch.tensor(self.pose_mean, dtype=torch.float32, device=device),
            "pose_std": torch.tensor(self.pose_std, dtype=torch.float32, device=device),
        }


class PAEPFutureDataset(Dataset):
    """
    One sample = (I_ext[t], I_wrist[t], F_hist[t-L+1:t], pose[t]) -> y[t+delta]

    Implements:
      - transition oversampling (sample near event transitions with probability p)
      - class counts (for weights)
      - normalization stats (force/pose)
    """

    def __init__(
        self,
        zarr_path: str,
        split_json: str,
        split: str,
        force_hist: int,
        delta: int,
        seed: int = 0,
        # transition sampling
        transition_sampling: bool = False,
        transition_prob: float = 0.6,
        transition_window: int = 12,
        # normalization
        compute_norm: bool = True,
        norm_samples: int = 20000,
        provided_norm: Optional[NormStats] = None,
    ):
        super().__init__()
        self.rng = random.Random(seed + (0 if split == "train" else (123 if split == "val" else 456)))
        self.np_rng = np.random.default_rng(seed + (0 if split == "train" else (123 if split == "val" else 456)))

        self.group, self.group_name = _open_group(zarr_path)

        self.force_hist = int(force_hist)
        self.delta = int(delta)

        self.transition_sampling = bool(transition_sampling)
        self.transition_prob = float(transition_prob)
        self.transition_window = int(transition_window)

        # keys
        required = [
            "external_img",
            "left_wrist_img",
            "left_robot_tcp_wrench",
            "left_robot_tcp_pose",
            "paep_event",
        ]
        for k in required:
            if k not in self.group.array_keys():
                raise KeyError(f"Missing key '{k}' in group={self.group_name}")

        self.ext = self.group["external_img"]
        self.wrist = self.group["left_wrist_img"]
        self.wrench = self.group["left_robot_tcp_wrench"]
        self.pose = self.group["left_robot_tcp_pose"]
        self.event = self.group["paep_event"]

        # episodes
        # Prefer meta/episode_ends if exists at root; if group is "data", meta is sibling under root
        root = zarr.open(zarr_path, mode="r")
        if "meta" in root.group_keys() and "episode_ends" in root["meta"].array_keys():
            episode_ends = np.array(root["meta"]["episode_ends"][:], dtype=np.int64)
        elif "episode_ends" in self.group.array_keys():
            episode_ends = np.array(self.group["episode_ends"][:], dtype=np.int64)
        else:
            raise KeyError("Missing meta/episode_ends")

        self.episode_slices_all = _episode_slices(episode_ends)

        self.episode_ids = load_split_ids(split_json, split)
        self.episode_slices = [self.episode_slices_all[i] for i in self.episode_ids]

        # Precompute transition indices per episode (global frame indices)
        self.transition_indices: List[np.ndarray] = []
        self.valid_ranges: List[Tuple[int, int]] = []  # [lo, hi) valid t for sampling
        for (s, e) in self.episode_slices:
            # valid t must satisfy t-(L-1) >= s  and t+delta < e
            lo = s + (self.force_hist - 1)
            hi = e - self.delta
            if hi <= lo:
                self.valid_ranges.append((0, 0))
                self.transition_indices.append(np.zeros((0,), dtype=np.int64))
                continue

            y = np.array(self.event[s:e], dtype=np.int64)
            # transitions at local index i where y[i]!=y[i-1], i in [1..len-1]
            trans_local = np.nonzero(y[1:] != y[:-1])[0] + 1
            trans_global = trans_local + s

            # keep only transitions that can be sampled (within valid t)
            trans_global = trans_global[(trans_global >= lo) & (trans_global < hi)]
            self.transition_indices.append(trans_global.astype(np.int64))
            self.valid_ranges.append((lo, hi))

        # class counts for this split (for info)
        self.class_counts = self._count_classes()

        # normalization
        if provided_norm is not None:
            self.norm = provided_norm
        else:
            self.norm = None
            if compute_norm:
                self.norm = self._compute_norm(norm_samples)

        if self.norm is None:
            # fallback safe defaults
            self.norm = NormStats(
                wrench_mean=np.zeros((6,), np.float32),
                wrench_std=np.ones((6,), np.float32),
                pose_mean=np.zeros((9,), np.float32),
                pose_std=np.ones((9,), np.float32),
            )

    def _count_classes(self) -> Dict[str, int]:
        counts = np.zeros((NUM_EVENTS,), dtype=np.int64)
        for (s, e) in self.episode_slices:
            y = np.array(self.event[s:e], dtype=np.int64)
            for c in range(NUM_EVENTS):
                counts[c] += np.sum(y == c)
        return {EVENT_NAMES[i]: int(counts[i]) for i in range(NUM_EVENTS)}

    def _compute_norm(self, n: int) -> NormStats:
        # sample random frames from valid ranges
        wrench_samples = []
        pose_samples = []
        total_episodes = len(self.episode_slices)
        if total_episodes == 0:
            raise RuntimeError("Empty split.")
        for _ in range(n):
            ep_idx = self.rng.randrange(total_episodes)
            lo, hi = self.valid_ranges[ep_idx]
            if hi <= lo:
                continue
            t = self.rng.randrange(lo, hi)
            wrench_samples.append(np.array(self.wrench[t], dtype=np.float32))
            pose_samples.append(np.array(self.pose[t], dtype=np.float32))

        W = np.stack(wrench_samples, axis=0) if len(wrench_samples) else np.zeros((1, 6), np.float32)
        P = np.stack(pose_samples, axis=0) if len(pose_samples) else np.zeros((1, 9), np.float32)

        w_mean = W.mean(axis=0)
        w_std = W.std(axis=0) + 1e-6
        p_mean = P.mean(axis=0)
        p_std = P.std(axis=0) + 1e-6
        return NormStats(w_mean.astype(np.float32), w_std.astype(np.float32), p_mean.astype(np.float32), p_std.astype(np.float32))

    def __len__(self):
        # This dataset is used with "random sampling per step", so length is not meaningful.
        # We'll return a large number; training loop controls steps/epoch.
        return 10**9

    def _sample_t(self) -> int:
        # choose an episode first
        ep_i = self.rng.randrange(len(self.episode_slices))
        lo, hi = self.valid_ranges[ep_i]
        if hi <= lo:
            # fallback to another ep
            for _ in range(10):
                ep_i = self.rng.randrange(len(self.episode_slices))
                lo, hi = self.valid_ranges[ep_i]
                if hi > lo:
                    break
        if hi <= lo:
            raise RuntimeError("No valid sampling range (force_hist/delta too large for episodes).")

        if self.transition_sampling and (self.rng.random() < self.transition_prob) and (len(self.transition_indices[ep_i]) > 0):
            # sample near a transition
            t0 = int(self.transition_indices[ep_i][self.rng.randrange(len(self.transition_indices[ep_i]))])
            # uniform in [t0-w, t0+w]
            w = self.transition_window
            t = t0 + self.rng.randint(-w, w)
            t = max(lo, min(hi - 1, t))
            return t
        else:
            return self.rng.randrange(lo, hi)

    def __getitem__(self, idx: int):
        t = self._sample_t()

        # images: uint8 HWC
        ext = np.array(self.ext[t], dtype=np.uint8)
        wrist = np.array(self.wrist[t], dtype=np.uint8)

        # force history: [L,6] from t-L+1 .. t inclusive
        L = self.force_hist
        f0 = t - (L - 1)
        wrench_hist = np.array(self.wrench[f0:t+1], dtype=np.float32)  # (L,6)
        pose_t = np.array(self.pose[t], dtype=np.float32)  # (9,)

        y = int(self.event[t + self.delta])  # future label

        sample = {
            "ext_img": torch.from_numpy(ext).permute(2, 0, 1).float() / 255.0,     # (3,H,W)
            "wrist_img": torch.from_numpy(wrist).permute(2, 0, 1).float() / 255.0, # (3,H,W)
            "wrench_hist": torch.from_numpy(wrench_hist),                          # (L,6)
            "pose": torch.from_numpy(pose_t),                                      # (9,)
            "y": torch.tensor(y, dtype=torch.long),
        }
        return sample
