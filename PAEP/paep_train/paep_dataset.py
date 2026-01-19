# paep_dataset.py
import json
from dataclasses import dataclass
from typing import Optional, List, Dict, Tuple

import numpy as np
import zarr
import torch
from torch.utils.data import Dataset

EVENT_NAMES = ["idle", "approach", "under", "effective", "over", "retreat"]

def _has(root, path: str) -> bool:
    try:
        _ = root[path]
        return True
    except Exception:
        return False

def pick_data_group(root: zarr.hierarchy.Group):
    # Prefer data group if data/paep_event exists; else root
    if _has(root, "data/paep_event"):
        return root["data"], "data"
    return root, "root"

def load_split_ids(split_json: str, split: str) -> List[int]:
    with open(split_json, "r", encoding="utf-8") as f:
        sp = json.load(f)
    if split not in sp:
        raise KeyError(f"split '{split}' not found in {split_json}. keys={list(sp.keys())}")
    ids = [int(x) for x in sp[split]]
    return ids

def episode_ranges_from_ends(episode_ends: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    episode_ends = np.asarray(episode_ends, dtype=np.int64)
    starts = np.concatenate([np.array([0], dtype=np.int64), episode_ends[:-1]])
    return starts, episode_ends

def gather_ranges(starts: np.ndarray, ends: np.ndarray, episode_ids: List[int]) -> List[Tuple[int, int, int]]:
    """
    Return list of (eid, start, end) for given episode_ids.
    """
    out = []
    n_ep = len(ends)
    for eid in episode_ids:
        if 0 <= eid < n_ep:
            out.append((eid, int(starts[eid]), int(ends[eid])))
    return out

def compute_counts_from_ranges(label_arr, ranges, num_classes: int) -> np.ndarray:
    binc = np.zeros((num_classes,), dtype=np.int64)
    for _, s, e in ranges:
        y = np.asarray(label_arr[s:e], dtype=np.int64)
        binc += np.bincount(y, minlength=num_classes)
    return binc

def compute_mean_std_from_ranges(arr, ranges, max_samples: Optional[int] = None, seed: int = 0):
    """
    Compute mean/std over concatenated ranges; optionally subsample for speed.
    """
    xs = []
    for _, s, e in ranges:
        xs.append(np.asarray(arr[s:e], dtype=np.float32))
    x = np.concatenate(xs, axis=0)

    if max_samples is not None and x.shape[0] > max_samples:
        rng = np.random.default_rng(seed)
        idx = rng.choice(x.shape[0], size=int(max_samples), replace=False)
        x = x[idx]

    mean = x.mean(axis=0)
    std = x.std(axis=0) + 1e-6
    return mean, std

@dataclass
class NormStats:
    wrench_mean: np.ndarray
    wrench_std: np.ndarray
    pose_mean: np.ndarray
    pose_std: np.ndarray

class PAEPZarrDataset(Dataset):
    """
    Segment sampling within specified split episodes (no crossing episodes).
    Returns many-to-many segment.
    """
    def __init__(
        self,
        zarr_path: str,
        segment_len: int,
        samples_per_epoch: int,
        split_json: str,
        split: str,  # train/val/test
        norm_stats: Optional[NormStats] = None,  # if provided, use these; else compute (train)
        norm_from_split: str = "train",          # compute norm from which split when norm_stats None
        norm_max_samples: Optional[int] = None,  # subsample for stats if needed
        seed: int = 0,
    ):
        super().__init__()
        self.zarr_path = zarr_path
        self.root = zarr.open_group(zarr_path, mode="r")
        self.g, self.g_name = pick_data_group(self.root)

        # keys
        for k in ["external_img", "left_wrist_img", "left_robot_tcp_wrench", "left_robot_tcp_pose", "paep_event"]:
            if not _has(self.g, k):
                raise KeyError(f"Missing key '{k}' in group={self.g_name}")

        self.k_ext = "external_img"
        self.k_wrist = "left_wrist_img"
        self.k_wrench = "left_robot_tcp_wrench"
        self.k_pose = "left_robot_tcp_pose"
        self.k_event = "paep_event"

        # episode ranges
        if not _has(self.root, "meta/episode_ends"):
            raise KeyError("meta/episode_ends not found")
        episode_ends = np.asarray(self.root["meta/episode_ends"][:], dtype=np.int64)
        starts, ends = episode_ranges_from_ends(episode_ends)

        self.split = split
        self.episode_ids = load_split_ids(split_json, split)
        self.ranges = gather_ranges(starts, ends, self.episode_ids)

        self.segment_len = int(segment_len)
        self.samples_per_epoch = int(samples_per_epoch)

        # keep only episodes long enough
        self.ranges = [(eid, s, e) for (eid, s, e) in self.ranges if (e - s) >= self.segment_len]
        if len(self.ranges) == 0:
            raise RuntimeError(f"No episodes in split={split} are long enough for segment_len={segment_len}")

        # class counts for this split (used for weights in training; for val/test just informative)
        binc = compute_counts_from_ranges(self.g[self.k_event], self.ranges, num_classes=len(EVENT_NAMES))
        self.class_counts = {EVENT_NAMES[i]: int(binc[i]) for i in range(len(EVENT_NAMES))}

        # normalization stats: train computes; val/test should reuse train stats (from ckpt)
        if norm_stats is not None:
            self.norm = norm_stats
        else:
            # compute from norm_from_split episodes
            ids_for_norm = load_split_ids(split_json, norm_from_split)
            ranges_for_norm = gather_ranges(starts, ends, ids_for_norm)
            # filter by length >=1
            ranges_for_norm = [(eid, s, e) for (eid, s, e) in ranges_for_norm if (e - s) > 0]

            wrench_mean, wrench_std = compute_mean_std_from_ranges(
                self.g[self.k_wrench], ranges_for_norm, max_samples=norm_max_samples, seed=seed
            )
            pose_mean, pose_std = compute_mean_std_from_ranges(
                self.g[self.k_pose], ranges_for_norm, max_samples=norm_max_samples, seed=seed
            )
            self.norm = NormStats(wrench_mean, wrench_std, pose_mean, pose_std)

        # RNG
        self.rng = np.random.default_rng(seed)

    @staticmethod
    def img_to_tensor(x_u8: np.ndarray) -> torch.Tensor:
        # [S,H,W,3] uint8 -> [S,3,H,W] float32
        x = torch.from_numpy(x_u8).float() / 255.0
        return x.permute(0, 3, 1, 2).contiguous()

    def __len__(self):
        return self.samples_per_epoch

    def __getitem__(self, idx):
        # sample one episode range
        eid, s, e = self.ranges[self.rng.integers(0, len(self.ranges))]
        a = int(self.rng.integers(s, e - self.segment_len + 1))
        b = a + self.segment_len

        ext = np.asarray(self.g[self.k_ext][a:b], dtype=np.uint8)
        wrist = np.asarray(self.g[self.k_wrist][a:b], dtype=np.uint8)
        wrench = np.asarray(self.g[self.k_wrench][a:b], dtype=np.float32)
        pose = np.asarray(self.g[self.k_pose][a:b], dtype=np.float32)
        event = np.asarray(self.g[self.k_event][a:b], dtype=np.int64)

        # normalize
        wrench = (wrench - self.norm.wrench_mean) / self.norm.wrench_std
        pose = (pose - self.norm.pose_mean) / self.norm.pose_std

        return {
            "external_img": self.img_to_tensor(ext),
            "wrist_img": self.img_to_tensor(wrist),
            "wrench": torch.from_numpy(wrench),
            "tcp_pose": torch.from_numpy(pose),
            "event": torch.from_numpy(event),
            "episode_id": torch.tensor(eid, dtype=torch.int64),
            "start": torch.tensor(a, dtype=torch.int64),
        }
