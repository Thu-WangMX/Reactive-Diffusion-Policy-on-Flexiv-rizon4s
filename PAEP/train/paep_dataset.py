# paep_dataset.py
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

import numpy as np
import torch
from torch.utils.data import Dataset
import zarr
import json

from torchvision import transforms
import torchvision.transforms.functional as TF


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


DEFAULT_PHASE_NAMES = ["approach", "progress", "done"]
CONTACT_NAMES = ["free", "contact"]


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
      ext_img[t], left_wrist_img[t], right_wrist_img[t], wrench_hist[t-L+1:t], pose[t]

    Targets:
      y_phase   = phase[t+delta]             (single future frame)
      y_contact = OR(contact[t+1 .. t+K])    (future window OR)

    Notes:
      - Ensure sampling t does not cross episode boundary (valid_ranges)
      - transition_sampling oversamples near phase transitions
      - Optional image augmentation (train split only):
          * ColorJitter (shared across views)
          * Very mild affine (shared across views): small translate + small scale, no rotation
    """

    def __init__(
        self,
        zarr_path: str,
        split_json: str,
        split: str,
        force_hist: int,
        delta: int,
        future_k: int = 12,
        seed: int = 0,
        transition_sampling: bool = False,
        transition_prob: float = 0.6,
        transition_window: int = 12,
        compute_norm: bool = True,
        norm_samples: int = 20000,
        provided_norm: Optional[NormStats] = None,
        phase_names: Optional[List[str]] = None,
        # -------- augmentation knobs (safe defaults) --------
        use_img_aug: bool = True,
        # 建议把默认值改温和，避免忘传参踩坑
        jitter_brightness: float = 0.10,
        jitter_contrast: float = 0.15,
        jitter_saturation: float = 0.05,
        jitter_hue: float = 0.01,
        geom_translate: float = 0.01,
        geom_scale: float = 0.01,
        geom_prob: float = 0.10,
        color_prob: float = 0.40,
    ):
        super().__init__()
        self.split = str(split)
        self.rng = random.Random(seed + (0 if split == "train" else (123 if split == "val" else 456)))

        self.group, self.group_name = _open_group(zarr_path)

        self.force_hist = int(force_hist)
        self.delta = int(delta)
        self.future_k = int(future_k)

        self.transition_sampling = bool(transition_sampling)
        self.transition_prob = float(transition_prob)
        self.transition_window = int(transition_window)

        # phase names / num events
        if phase_names is None:
            phase_names = list(DEFAULT_PHASE_NAMES)
        assert isinstance(phase_names, (list, tuple)) and len(phase_names) > 0, "phase_names must be a non-empty list"
        self.phase_names = list(phase_names)
        self.num_events = int(len(self.phase_names))

        # ImageNet normalize (because you are using pretrained ResNet)
        self.normalize = transforms.Normalize(
            mean=[0.485, 0.456, 0.406],
            std=[0.229, 0.224, 0.225]
        )

        # augmentation config (train only)
        self.use_img_aug = bool(use_img_aug) and (self.split == "train")
        self.jitter_brightness = float(jitter_brightness)
        self.jitter_contrast = float(jitter_contrast)
        self.jitter_saturation = float(jitter_saturation)
        self.jitter_hue = float(jitter_hue)
        self.geom_translate = float(geom_translate)
        self.geom_scale = float(geom_scale)
        self.geom_prob = float(geom_prob)
        self.color_prob = float(color_prob)

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
        self.left_wrist = self.group["left_wrist_img"]
        self.has_right_wrist = ("right_wrist_img" in self.group.array_keys())
        self.right_wrist = self.group["right_wrist_img"] if self.has_right_wrist else None

        self.wrench = self.group["left_robot_tcp_wrench"]
        self.pose = self.group["left_robot_tcp_pose"]
        self.contact = self.group["paep_contact"]  # int8 0/1
        self.phase = self.group["paep_phase"]      # int8 0..num_events-1 (expected)

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
            ymax = int(y.max()) if y.size > 0 else 0
            if ymax >= self.num_events:
                raise ValueError(
                    f"[PAEPFutureDataset] paep_phase max={ymax} but num_events={self.num_events}. "
                    f"Check cfg.PHASE_NAMES vs dataset labels."
                )

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
        counts = np.zeros((self.num_events,), dtype=np.int64)
        for (s, e) in self.episode_slices:
            y = np.array(self.phase[s:e], dtype=np.int64)
            for c in range(self.num_events):
                counts[c] += np.sum(y == c)
        return {self.phase_names[i]: int(counts[i]) for i in range(self.num_events)}

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
        # controlled by steps_per_epoch in training
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

    # --------------------- augmentation helpers ---------------------

    def _apply_shared_affine(self, x: torch.Tensor, translate_px: Tuple[int, int], scale: float) -> torch.Tensor:
        # no rotation, no shear
        return TF.affine(
            x, angle=0.0, translate=list(translate_px), scale=float(scale),
            shear=[0.0, 0.0], interpolation=TF.InterpolationMode.BILINEAR,
            fill=0
        )


    def __getitem__(self, idx: int):
        t = self._sample_t()

        ext = np.array(self.ext[t], dtype=np.uint8)
        left_wrist = np.array(self.left_wrist[t], dtype=np.uint8)
        if self.has_right_wrist:
            right_wrist = np.array(self.right_wrist[t], dtype=np.uint8)
        else:
            right_wrist = left_wrist  # keep your compatibility behavior

        # zarr stores BGR -> convert to RGB
        ext = ext[..., ::-1].copy()
        left_wrist = left_wrist[..., ::-1].copy()
        right_wrist = right_wrist[..., ::-1].copy()


        L = self.force_hist
        f0 = t - (L - 1)
        wrench_hist = np.array(self.wrench[f0:t + 1], dtype=np.float32)  # (L,6)
        pose_t = np.array(self.pose[t], dtype=np.float32)                # (9,)

        # targets
        y_phase = int(self.phase[t + self.delta])  # single-step future
        c_future = np.array(self.contact[t + 1: t + 1 + self.future_k], dtype=np.int64)
        y_contact = 1.0 if (c_future.max() > 0) else 0.0

        # to float tensor in [0,1]
        ext_tensor = torch.from_numpy(ext).permute(2, 0, 1).float() / 255.0
        left_tensor = torch.from_numpy(left_wrist).permute(2, 0, 1).float() / 255.0
        right_tensor = torch.from_numpy(right_wrist).permute(2, 0, 1).float() / 255.0

        # augmentation (train only)
        if self.use_img_aug:
            # shared color jitter
            if self.rng.random() < self.color_prob:
                # generate one set of jitter params and apply to all three views
                # (implemented by applying same sampled factors via helper)
                # NOTE: helper samples internally using self.rng; we must sample once and reuse.
                # So we pre-sample the factors here:
                b = self.jitter_brightness
                c = self.jitter_contrast
                s = self.jitter_saturation
                h = self.jitter_hue

                # pre-sample factors
                bf = 1.0 + self.rng.uniform(-b, b) if b > 0 else 1.0
                cf = 1.0 + self.rng.uniform(-c, c) if c > 0 else 1.0
                sf = 1.0 + self.rng.uniform(-s, s) if s > 0 else 1.0
                hf = self.rng.uniform(-h, h) if h > 0 else 0.0

                def apply_jitter(x):
                    x = TF.adjust_brightness(x, bf)
                    x = TF.adjust_contrast(x, cf)
                    x = TF.adjust_saturation(x, sf)
                    if h > 0:
                        x = TF.adjust_hue(x, hf)
                    return x.clamp(0.0, 1.0)

                ext_tensor = apply_jitter(ext_tensor)
                left_tensor = apply_jitter(left_tensor)
                right_tensor = apply_jitter(right_tensor)

            # shared mild affine (translate + scale)
            if self.rng.random() < self.geom_prob:
                H, W = int(ext_tensor.shape[1]), int(ext_tensor.shape[2])
                max_dx = int(round(self.geom_translate * W))
                max_dy = int(round(self.geom_translate * H))
                dx = self.rng.randint(-max_dx, max_dx) if max_dx > 0 else 0
                dy = self.rng.randint(-max_dy, max_dy) if max_dy > 0 else 0
                scale = 1.0 + self.rng.uniform(-self.geom_scale, self.geom_scale) if self.geom_scale > 0 else 1.0

                ext_tensor = self._apply_shared_affine(ext_tensor, (dx, dy), scale)
                left_tensor = self._apply_shared_affine(left_tensor, (dx, dy), scale)
                right_tensor = self._apply_shared_affine(right_tensor, (dx, dy), scale)

        # ImageNet normalize
        ext_tensor = self.normalize(ext_tensor)
        left_tensor = self.normalize(left_tensor)
        right_tensor = self.normalize(right_tensor)

        return {
            "ext_img": ext_tensor,
            "left_wrist_img": left_tensor,
            "right_wrist_img": right_tensor,
            "wrench_hist": torch.from_numpy(wrench_hist),
            "pose": torch.from_numpy(pose_t),
            "y_phase": torch.tensor(y_phase, dtype=torch.long),
            "y_contact": torch.tensor([y_contact], dtype=torch.float32),
        }
