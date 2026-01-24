# paep_dataset.py
import json
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

import numpy as np
import torch
from torch.utils.data import Dataset
import zarr


# wiping board (new labels)
PHASE_NAMES = ["approach", "progress", "done"]
NUM_EVENTS = 3
EVENT_NAMES = PHASE_NAMES

CONTACT_NAMES = ["free", "contact"]


def load_split_ids(split_json: str, split: str) -> List[int]:
    with open(split_json, "r", encoding="utf-8") as f:
        obj = json.load(f)
    assert split in obj, f"split must be in {list(obj.keys())}"
    return list(map(int, obj[split]))


def _open_group(zarr_path: str):
    root = zarr.open(zarr_path, mode="r")
    if isinstance(root, zarr.hierarchy.Group):
        # arrays at root
        if "external_img" in root.array_keys():
            return root, "root"
        # arrays under data/
        if "data" in root.group_keys():
            g = root["data"]
            if "external_img" in g.array_keys():
                return g, "data"
    raise KeyError("Cannot find arrays. Expect external_img at root or under group 'data'.")


def _episode_slices(episode_ends: np.ndarray) -> List[Tuple[int, int]]:
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
    Sample at time t:

    Inputs:
      ext_img[t], wrist_img[t], wrench_hist[t-L+1:t], pose[t]

    Targets:
      y_phase = phase[t+delta]                    (single future frame)
      y_contact = OR(contact[t+1 .. t+K])         (future window OR)

    Notes:
      - We ensure sampling t does not cross episode boundary (by valid_ranges)
      - transition_sampling oversamples near phase transitions
    """

    def __init__(
        self,
        zarr_path: str,
        split_json: str,
        split: str,
        force_hist: int,
        delta: int,
        future_k: int = 12,  # 24Hz * 0.5s
        seed: int = 0,
        transition_sampling: bool = False,
        transition_prob: float = 0.6,
        transition_window: int = 12,
        compute_norm: bool = True,
        norm_samples: int = 20000,
        provided_norm: Optional[NormStats] = None,
    ):
        super().__init__()
        self.rng = random.Random(seed + (0 if split == "train" else (123 if split == "val" else 456)))

        self.group, self.group_name = _open_group(zarr_path)

        self.force_hist = int(force_hist)
        self.delta = int(delta)
        self.future_k = int(future_k)

        self.transition_sampling = bool(transition_sampling)
        self.transition_prob = float(transition_prob)
        self.transition_window = int(transition_window)

        required = [
            "external_img",
            "left_wrist_img",
            "left_robot_tcp_wrench",
            "left_robot_tcp_pose",
            "paep_contact",
            "paep_phase",
        ]
        for k in required:
            if k not in self.group.array_keys():
                raise KeyError(f"Missing key '{k}' in group={self.group_name}")

        self.ext = self.group["external_img"]
        self.wrist = self.group["left_wrist_img"]
        self.wrench = self.group["left_robot_tcp_wrench"]
        self.pose = self.group["left_robot_tcp_pose"]
        self.contact = self.group["paep_contact"]  # int8 0/1
        self.phase = self.group["paep_phase"]      # int8 0/1/2

        # episode ends
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

        # valid ranges + transitions per episode
        self.valid_ranges: List[Tuple[int, int]] = []
        self.transition_indices: List[np.ndarray] = []

        need_future = max(self.delta, self.future_k)

        for (s, e) in self.episode_slices:
            lo = s + (self.force_hist - 1)
            hi = e - need_future
            if hi <= lo:
                self.valid_ranges.append((0, 0))
                self.transition_indices.append(np.zeros((0,), dtype=np.int64))
                continue

            y = np.array(self.phase[s:e], dtype=np.int64)
            trans_local = np.nonzero(y[1:] != y[:-1])[0] + 1
            trans_global = trans_local + s
            trans_global = trans_global[(trans_global >= lo) & (trans_global < hi)]
            self.transition_indices.append(trans_global.astype(np.int64))
            self.valid_ranges.append((lo, hi))

        self.class_counts = self._count_phase()
        self.contact_counts = self._count_contact()

        # norm
        if provided_norm is not None:
            self.norm = provided_norm
        else:
            self.norm = None
            if compute_norm:
                self.norm = self._compute_norm(norm_samples)
        if self.norm is None:
            self.norm = NormStats(
                wrench_mean=np.zeros((6,), np.float32),
                wrench_std=np.ones((6,), np.float32),
                pose_mean=np.zeros((9,), np.float32),
                pose_std=np.ones((9,), np.float32),
            )

    def _count_phase(self) -> Dict[str, int]:
        counts = np.zeros((NUM_EVENTS,), dtype=np.int64)
        for (s, e) in self.episode_slices:
            y = np.array(self.phase[s:e], dtype=np.int64)
            for c in range(NUM_EVENTS):
                counts[c] += np.sum(y == c)
        return {EVENT_NAMES[i]: int(counts[i]) for i in range(NUM_EVENTS)}

    def _count_contact(self) -> Dict[str, int]:
        free_cnt = 0
        contact_cnt = 0
        for (s, e) in self.episode_slices:
            y = np.array(self.contact[s:e], dtype=np.int64)
            free_cnt += int(np.sum(y == 0))
            contact_cnt += int(np.sum(y == 1))
        return {"free": free_cnt, "contact": contact_cnt}

    def _compute_norm(self, n: int) -> NormStats:
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
        return NormStats(w_mean.astype(np.float32), w_std.astype(np.float32),
                         p_mean.astype(np.float32), p_std.astype(np.float32))

    def __len__(self):
        # 用 steps_per_epoch 控制训练步数
        return 10**9

    def _sample_t(self) -> int:
        ep_i = self.rng.randrange(len(self.episode_slices))
        lo, hi = self.valid_ranges[ep_i]
        if hi <= lo:
            for _ in range(20):
                ep_i = self.rng.randrange(len(self.episode_slices))
                lo, hi = self.valid_ranges[ep_i]
                if hi > lo:
                    break
        if hi <= lo:
            raise RuntimeError("No valid sampling range. Check force_hist/delta/future_k vs episode length.")

        if self.transition_sampling and (self.rng.random() < self.transition_prob) and (len(self.transition_indices[ep_i]) > 0):
            t0 = int(self.transition_indices[ep_i][self.rng.randrange(len(self.transition_indices[ep_i]))])
            w = self.transition_window
            t = t0 + self.rng.randint(-w, w)
            t = max(lo, min(hi - 1, t))
            return t

        return self.rng.randrange(lo, hi)

    def __getitem__(self, idx: int):
        t = self._sample_t()

        ext = np.array(self.ext[t], dtype=np.uint8)
        wrist = np.array(self.wrist[t], dtype=np.uint8)

        L = self.force_hist
        f0 = t - (L - 1)
        wrench_hist = np.array(self.wrench[f0:t + 1], dtype=np.float32)  # (L,6)
        pose_t = np.array(self.pose[t], dtype=np.float32)                # (9,)

        # targets
        y_phase = int(self.phase[t + self.delta])  # single-step future

        c_future = np.array(self.contact[t + 1: t + 1 + self.future_k], dtype=np.int64)  # length K
        y_contact = 1.0 if (c_future.max() > 0) else 0.0

        return {
            "ext_img": torch.from_numpy(ext).permute(2, 0, 1).float() / 255.0,
            "wrist_img": torch.from_numpy(wrist).permute(2, 0, 1).float() / 255.0,
            "wrench_hist": torch.from_numpy(wrench_hist),
            "pose": torch.from_numpy(pose_t),

            "y_phase": torch.tensor(y_phase, dtype=torch.long),
            "y_contact": torch.tensor([y_contact], dtype=torch.float32),  # (1,)
        }
