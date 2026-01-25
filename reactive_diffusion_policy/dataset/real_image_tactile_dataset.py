# real_image_tactile_dataset.py
from typing import Dict
import os
import copy
import tqdm
import numpy as np
import torch
from threadpoolctl import threadpool_limits

from reactive_diffusion_policy.common.pytorch_util import dict_apply
from reactive_diffusion_policy.dataset.base_dataset import BaseImageDataset
from reactive_diffusion_policy.model.common.normalizer import LinearNormalizer, SingleFieldLinearNormalizer
from reactive_diffusion_policy.common.replay_buffer import ReplayBuffer
from reactive_diffusion_policy.common.sampler import SequenceSampler, get_val_mask, downsample_mask
from reactive_diffusion_policy.common.normalize_util import get_image_range_normalizer, get_action_normalizer
from reactive_diffusion_policy.common.action_utils import (
    absolute_actions_to_relative_actions,
    get_inter_gripper_actions,
)
from reactive_diffusion_policy.real_world.real_world_transforms import RealWorldTransforms


def _take_first_k_timeordered(x: np.ndarray, k: int, downsample_ratio: int) -> np.ndarray:
    """
    Match original behavior: take first k, then downsample by ratio, keep time order.
    Equivalent to x[:k][::-ratio][::-1].
    """
    y = x if k is None else x[:k]
    r = int(downsample_ratio)
    if r <= 1:
        return y
    return y[::-r][::-1]


class RealImageTactileDataset(BaseImageDataset):
    """
    Real-world dataset with RGB + low-dim observations.

    PAEP-Force alignment (no SequenceSampler change):
    - Image/low-dim obs follow obs_temporal_downsample_ratio (e.g., r=2 => 12Hz).
    - Force history wrench_hist is ALWAYS dense (24Hz):
        For each sparse obs frame time index, we fetch the last `force_hist` raw wrench frames
        from replay_buffer directly, with correct episode-boundary padding.
    """

    def __init__(
        self,
        shape_meta: dict,
        dataset_path: str,
        horizon=1,
        pad_before=0,
        pad_after=0,
        n_obs_steps=None,
        obs_temporal_downsample_ratio=1,
        n_latency_steps=0,
        seed=42,
        val_ratio=0.0,
        max_train_episodes=None,
        delta_action=False,
        relative_action=False,
        relative_tcp_obs_for_relative_action=True,
        transform_params=None,
        # smoothing (legacy/optional)
        action_smoothing_alpha: float = 0.0,
        smooth_xyz_only: bool = False,
        smooth_rot_only: bool = False,
        smooth_rpy_only: bool = False,
        # wrench history
        add_wrench_hist: bool = True,
        force_hist: int = 48,
        wrench_key: str = "left_robot_tcp_wrench",
        wrench_hist_key: str = "wrench_hist",
        wrench_hist_pad_mode: str = "repeat_first",
        debug_wrench_hist: bool = False,
        # swallow any future args from hydra configs
        **kwargs,
    ):
        assert os.path.isdir(dataset_path)
        assert (not add_wrench_hist) or (n_obs_steps is not None), "add_wrench_hist=True requires n_obs_steps."

        # store
        self.action_smoothing_alpha = float(action_smoothing_alpha)
        self.smooth_xyz_only = bool(smooth_xyz_only)
        self.smooth_rot_only = bool(smooth_rot_only)
        self.smooth_rpy_only = bool(smooth_rpy_only)

        if len(kwargs) > 0:
            print(f"[RealImageTactileDataset] Ignored extra kwargs: {list(kwargs.keys())}")

        # ---- parse shape meta ----
        rgb_keys = []
        lowdim_keys = []
        obs_shape_meta = shape_meta["obs"]
        for key, attr in obs_shape_meta.items():
            t = attr.get("type", "low_dim")
            if t == "rgb":
                rgb_keys.append(key)
            elif t == "low_dim":
                lowdim_keys.append(key)

        extended_rgb_keys = []
        extended_lowdim_keys = []
        extended_obs_shape_meta = shape_meta.get("extended_obs", dict())
        for key, attr in extended_obs_shape_meta.items():
            t = attr.get("type", "low_dim")
            if t == "rgb":
                extended_rgb_keys.append(key)
            elif t == "low_dim":
                extended_lowdim_keys.append(key)

        # ---- load replay buffer ----
        zarr_path = os.path.join(dataset_path, "replay_buffer.zarr")
        zarr_load_keys = set(rgb_keys + lowdim_keys + extended_rgb_keys + extended_lowdim_keys + ["action"])
        zarr_load_keys = list(filter(lambda k: "wrt" not in k, zarr_load_keys))
        if add_wrench_hist and (wrench_key not in zarr_load_keys):
            zarr_load_keys.append(wrench_key)

        replay_buffer = ReplayBuffer.copy_from_path(zarr_path, keys=zarr_load_keys)

        # ---- optional: delta action ----
        if delta_action:
            actions = replay_buffer["action"][:]
            assert actions.shape[1] <= 3
            actions_diff = np.zeros_like(actions)
            episode_ends = replay_buffer.episode_ends[:]
            for i in range(len(episode_ends)):
                start = 0 if i == 0 else episode_ends[i - 1]
                end = episode_ends[i]
                actions_diff[start + 1 : end] = np.diff(actions[start:end], axis=0)
            replay_buffer["action"][:] = actions_diff

        # ---- transforms / flags ----
        self.relative_action = relative_action
        self.relative_tcp_obs_for_relative_action = relative_tcp_obs_for_relative_action
        self.transforms = RealWorldTransforms(option=transform_params)

        self.add_wrench_hist = bool(add_wrench_hist)
        self.force_hist = int(force_hist)
        self.wrench_key = wrench_key
        self.wrench_hist_key = wrench_hist_key
        self.wrench_hist_pad_mode = str(wrench_hist_pad_mode)
        self.debug_wrench_hist = bool(debug_wrench_hist)

        self.shape_meta = shape_meta
        self.rgb_keys = rgb_keys
        self.lowdim_keys = lowdim_keys
        self.extended_rgb_keys = extended_rgb_keys
        self.extended_lowdim_keys = extended_lowdim_keys

        self.horizon = int(horizon)
        self.n_latency_steps = int(n_latency_steps)
        self.n_obs_steps = int(n_obs_steps) if n_obs_steps is not None else None
        self.obs_downsample_ratio = int(obs_temporal_downsample_ratio)
        self.pad_before = int(pad_before)
        self.pad_after = int(pad_after)
        self.seed = int(seed)

        # ---- masks ----
        val_mask = get_val_mask(
            n_episodes=replay_buffer.n_episodes,
            val_ratio=val_ratio,
            seed=seed,
        )
        train_mask = ~val_mask
        train_mask = downsample_mask(mask=train_mask, max_n=max_train_episodes, seed=seed)

        self.val_mask = val_mask
        self.replay_buffer = replay_buffer
        self.episode_ends = replay_buffer.episode_ends[:]  # cache for fast episode-boundary queries

        # ---- key_first_k ----
        # IMPORTANT: do NOT set key_first_k for uint8 rgb keys if your SequenceSampler fills with NaN
        # (since you said you won't edit SequenceSampler). Keep key_first_k only for float low-dim if you want.
        key_first_k: Dict[str, int] = dict()
        if self.n_obs_steps is not None:
            # keep perf hint for lowdim only (float), avoid rgb(uint8)
            for key in (lowdim_keys):
                if key not in (extended_lowdim_keys):
                    key_first_k[key] = self.n_obs_steps * self.obs_downsample_ratio
        self.key_first_k = key_first_k

        # ---- samplers ----
        # Use ONE sampler for all keys used normally (exclude wrench_key here; we fetch wrench directly)
        sampler_keys = set(rgb_keys + lowdim_keys + extended_rgb_keys + extended_lowdim_keys + ["action"])
        sampler_keys = list(sampler_keys)

        self.sampler = SequenceSampler(
            replay_buffer=replay_buffer,
            sequence_length=self.horizon + self.n_latency_steps,
            pad_before=self.pad_before,
            pad_after=self.pad_after,
            episode_mask=train_mask,
            keys=sampler_keys,
            key_first_k=key_first_k,
        )

    def get_validation_dataset(self):
        val_set = copy.copy(self)

        sampler_keys = set(self.rgb_keys + self.lowdim_keys + self.extended_rgb_keys + self.extended_lowdim_keys + ["action"])
        sampler_keys = list(sampler_keys)

        val_set.sampler = SequenceSampler(
            replay_buffer=self.replay_buffer,
            sequence_length=self.horizon + self.n_latency_steps,
            pad_before=self.pad_before,
            pad_after=self.pad_after,
            episode_mask=self.val_mask,
            keys=sampler_keys,
            key_first_k=self.key_first_k,
        )
        val_set.val_mask = ~self.val_mask
        return val_set

    def get_normalizer(self, **kwargs) -> LinearNormalizer:
        normalizer = LinearNormalizer()

        # calculate inter-gripper relative obs for "wrt" keys if present
        if (
            "left_robot_wrt_right_robot_tcp_pose" in self.lowdim_keys
            or "right_robot_wrt_left_robot_tcp_pose" in self.lowdim_keys
        ):
            inter_gripper_data_dict = {
                key: []
                for key in self.lowdim_keys
                if ("robot_tcp_pose" in key and "wrt" in key)
            }
            for data in tqdm.tqdm(self, leave=False, desc="Calculating inter-gripper relative obs for normalizer"):
                for key in inter_gripper_data_dict.keys():
                    inter_gripper_data_dict[key].append(data["obs"][key])
            inter_gripper_data_dict = dict_apply(inter_gripper_data_dict, np.stack)

        # relative action/obs normalization
        if self.relative_action:
            relative_data_dict = {
                key: []
                for key in (self.lowdim_keys + ["action"])
                if (("robot_tcp_pose" in key and "wrt" not in key) or (key == "action"))
            }
            for data in tqdm.tqdm(self, leave=False, desc="Calculating relative action/obs for normalizer"):
                for key in relative_data_dict.keys():
                    if key == "action":
                        relative_data_dict[key].append(data[key])
                    else:
                        relative_data_dict[key].append(data["obs"][key])
            relative_data_dict = dict_apply(relative_data_dict, np.stack)

        # action
        if self.relative_action:
            action_all = relative_data_dict["action"]
        else:
            action_all = self.replay_buffer["action"][:, : self.shape_meta["action"]["shape"][0]]
        normalizer["action"] = get_action_normalizer(action_all)

        # obs (wrench_hist NOT normalized here)
        for key in list(set(self.lowdim_keys)):
            if self.relative_action and ("relative_data_dict" in locals()) and (key in relative_data_dict):
                normalizer[key] = get_action_normalizer(relative_data_dict[key])
            elif "robot_tcp_pose" in key and "wrt" in key:
                normalizer[key] = get_action_normalizer(inter_gripper_data_dict[key])
            elif "robot_tcp_pose" in key and "wrt" not in key:
                normalizer[key] = get_action_normalizer(
                    self.replay_buffer[key][:, : self.shape_meta["obs"][key]["shape"][0]]
                )
            else:
                normalizer[key] = SingleFieldLinearNormalizer.create_fit(
                    self.replay_buffer[key][:, : self.shape_meta["obs"][key]["shape"][0]]
                )

        for key in list(set(self.extended_lowdim_keys)):
            if key in self.lowdim_keys:
                assert (
                    self.shape_meta["extended_obs"][key]["shape"][0]
                    == self.shape_meta["obs"][key]["shape"][0]
                ), f"Extended obs {key} has different shape from obs {key}"
            else:
                if self.relative_action and ("relative_data_dict" in locals()) and (key in relative_data_dict):
                    normalizer[key] = get_action_normalizer(relative_data_dict[key])
                elif "robot_tcp_pose" in key and "wrt" in key:
                    normalizer[key] = get_action_normalizer(inter_gripper_data_dict[key])
                elif "robot_tcp_pose" in key and "wrt" not in key:
                    normalizer[key] = get_action_normalizer(
                        self.replay_buffer[key][:, : self.shape_meta["extended_obs"][key]["shape"][0]]
                    )
                else:
                    normalizer[key] = SingleFieldLinearNormalizer.create_fit(
                        self.replay_buffer[key][:, : self.shape_meta["extended_obs"][key]["shape"][0]]
                    )

        # image
        for key in list(set(self.rgb_keys + self.extended_rgb_keys)):
            normalizer[key] = get_image_range_normalizer()

        return normalizer

    def get_all_actions(self) -> torch.Tensor:
        return torch.from_numpy(self.replay_buffer["action"][:, : self.shape_meta["action"]["shape"][0]])

    def __len__(self):
        return len(self.sampler)

    def _episode_bounds_for_index(self, abs_idx: int):
        """
        Given an absolute replay_buffer index, return (ep_start, ep_end_exclusive).
        """
        # episode_ends: [end0, end1, ...] (exclusive)
        # find smallest end > abs_idx
        ep_id = int(np.searchsorted(self.episode_ends, abs_idx, side="right"))
        ep_start = 0 if ep_id == 0 else int(self.episode_ends[ep_id - 1])
        ep_end = int(self.episode_ends[ep_id])
        return ep_start, ep_end

    def _fetch_wrench_window(self, abs_end_idx: int) -> np.ndarray:
        """
        Fetch a (L,6) wrench window ending at abs_end_idx (inclusive), dense 24Hz.
        Pads on the LEFT at episode start using repeat_first or zeros.
        """
        L = int(self.force_hist)
        ep_start, ep_end = self._episode_bounds_for_index(abs_end_idx)

        # clamp end inside episode
        abs_end_idx = int(np.clip(abs_end_idx, ep_start, ep_end - 1))

        start_idx = abs_end_idx - (L - 1)
        fetch_start = max(ep_start, start_idx)

        seg = self.replay_buffer[self.wrench_key][fetch_start : abs_end_idx + 1, :6].astype(np.float32)
        k = seg.shape[0]

        out = np.zeros((L, 6), dtype=np.float32)
        out[-k:] = seg

        if k < L:
            pad_n = L - k
            if self.wrench_hist_pad_mode == "repeat_first":
                if k > 0:
                    out[:pad_n] = seg[:1]
                else:
                    # no data at all (should be rare)
                    out[:pad_n] = 0.0
            elif self.wrench_hist_pad_mode == "zeros":
                pass
            else:
                raise ValueError(f"Unknown wrench_hist_pad_mode={self.wrench_hist_pad_mode}")

        return out

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        threadpool_limits(1)

        data = self.sampler.sample_sequence(idx)
        # indices: (buffer_start_idx, buffer_end_idx, sample_start_idx, sample_end_idx)
        buffer_start_idx, buffer_end_idx, sample_start_idx, sample_end_idx = self.sampler.indices[idx]
        buffer_start_idx = int(buffer_start_idx)
        buffer_end_idx = int(buffer_end_idx)
        sample_start_idx = int(sample_start_idx)
        sample_end_idx = int(sample_end_idx)

        To = int(self.n_obs_steps) if self.n_obs_steps is not None else None
        ratio = int(self.obs_downsample_ratio)

        obs_dict: Dict[str, np.ndarray] = {}

        # --- RGB ---
        for key in self.rgb_keys:
            x = data[key]  # (T,H,W,C)
            x = _take_first_k_timeordered(x, To * ratio if To is not None else None, ratio)
            obs_dict[key] = np.moveaxis(x, -1, 1).astype(np.float32) / 255.0
            if key not in self.extended_rgb_keys:
                del data[key]

        # --- Low-dim (non-wrt) ---
        for key in self.lowdim_keys:
            if "wrt" in key:
                continue
            x = data[key][:, : self.shape_meta["obs"][key]["shape"][0]]
            x = _take_first_k_timeordered(x, To * ratio if To is not None else None, ratio)
            obs_dict[key] = x.astype(np.float32)
            if key not in self.extended_lowdim_keys:
                del data[key]

        # inter-gripper relative action
        obs_dict.update(get_inter_gripper_actions(obs_dict, self.lowdim_keys, self.transforms))
        for key in ["left_robot_wrt_right_robot_tcp_pose", "right_robot_wrt_left_robot_tcp_pose"]:
            if key in obs_dict:
                obs_dict[key] = obs_dict[key][:, : self.shape_meta["obs"][key]["shape"][0]].astype(np.float32)

        # --- Extended obs ---
        extended_obs_dict: Dict[str, np.ndarray] = {}
        for key in self.extended_rgb_keys:
            extended_obs_dict[key] = np.moveaxis(data[key], -1, 1).astype(np.float32) / 255.0
            del data[key]
        for key in self.extended_lowdim_keys:
            if "wrt" not in key:
                extended_obs_dict[key] = data[key][:, : self.shape_meta["extended_obs"][key]["shape"][0]].astype(np.float32)
                del data[key]

        # --- NEW: wrench_hist (dense 24Hz, aligned to sparse obs frames) ---
        if self.add_wrench_hist:
            assert To is not None, "n_obs_steps must be set when add_wrench_hist=True."

            # We align wrench windows to the SAME sparse time positions used by _take_first_k_timeordered.
            # In the padded sampler sequence, the sparse frames correspond to indices:
            #   end positions = [k-1-(To-1)*ratio, ..., k-1] within the first k = To*ratio steps.
            k = To * ratio
            end_positions = np.arange(k - 1 - (To - 1) * ratio, k, ratio)  # length To, increasing

            def pos_to_abs_idx(pos: int) -> int:
                # map a position in the padded sample [0, sequence_length-1] to absolute replay idx
                if pos < sample_start_idx:
                    return buffer_start_idx
                if pos >= sample_end_idx:
                    return buffer_end_idx - 1
                return buffer_start_idx + (pos - sample_start_idx)

            end_abs = [pos_to_abs_idx(int(p)) for p in end_positions]
            w_hist = np.stack([self._fetch_wrench_window(t) for t in end_abs], axis=0)  # (To,L,6)
            obs_dict[self.wrench_hist_key] = w_hist.astype(np.float32)

            if self.debug_wrench_hist:
                fz = w_hist[..., 2]
                print(
                    f"[wrench_hist] idx={idx} To={To} ratio={ratio} k={k} "
                    f"end_pos={end_positions.tolist()} end_abs={end_abs} "
                    f"fz(min,max,mean)=({fz.min():.3f},{fz.max():.3f},{fz.mean():.3f})"
                )

        # --- Action ---
        action = data["action"][:, : self.shape_meta["action"]["shape"][0]].astype(np.float32)
        if self.n_latency_steps > 0:
            action = action[self.n_latency_steps:]

        # relative action / obs
        if self.relative_action:
            base_absolute_action = np.concatenate(
                [
                    obs_dict["left_robot_tcp_pose"][-1] if "left_robot_tcp_pose" in obs_dict else np.array([]),
                    obs_dict["right_robot_tcp_pose"][-1] if "right_robot_tcp_pose" in obs_dict else np.array([]),
                ],
                axis=-1,
            )
            action = absolute_actions_to_relative_actions(action, base_absolute_action=base_absolute_action)

            if self.relative_tcp_obs_for_relative_action:
                for key in self.lowdim_keys:
                    if ("robot_tcp_pose" in key) and ("wrt" not in key) and (key in obs_dict):
                        obs_dict[key] = absolute_actions_to_relative_actions(
                            obs_dict[key], base_absolute_action=base_absolute_action
                        )

        torch_data = {
            "obs": dict_apply(obs_dict, torch.from_numpy),
            "action": torch.from_numpy(action),
            "extended_obs": dict_apply(extended_obs_dict, torch.from_numpy),
        }
        return torch_data
