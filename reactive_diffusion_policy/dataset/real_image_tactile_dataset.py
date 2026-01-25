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


def _build_wrench_hist(wrench_seq: np.ndarray, hist_len: int, pad_mode: str = "repeat_first") -> np.ndarray:
    """
    Build per-timestep wrench history windows.

    wrench_seq: (T, 6) float32
    returns: (T, L, 6), right-aligned history ending at each t (inclusive)
    """
    assert wrench_seq.ndim == 2 and wrench_seq.shape[1] == 6, f"wrench_seq must be (T,6), got {wrench_seq.shape}"
    T = wrench_seq.shape[0]
    L = int(hist_len)
    out = np.zeros((T, L, 6), dtype=np.float32)

    for t in range(T):
        s = max(0, t - L + 1)
        seg = wrench_seq[s : t + 1]  # (<=L,6)
        k = seg.shape[0]
        out[t, -k:] = seg
        if k < L:
            if pad_mode == "repeat_first":
                out[t, : L - k] = seg[:1]
            elif pad_mode == "zeros":
                pass
            else:
                raise ValueError(f"Unknown pad_mode={pad_mode}")
    return out


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

    Key design for PAEP-guided vision-force fusion:
    - Training horizon (e.g., 16) should NOT be increased.
    - wrench_hist needs a longer history window: (n_obs_steps + force_hist - 1).
    Therefore we use a dedicated `wrench_sampler` with a longer `sequence_length`,
    while keeping the original sampler (and training horizon) unchanged.
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
        # ✅ swallow any future args from hydra configs
        **kwargs,
    ):
        assert os.path.isdir(dataset_path)
        assert (not add_wrench_hist) or (n_obs_steps is not None), "add_wrench_hist=True requires n_obs_steps."

        # store (even if not used yet)
        self.action_smoothing_alpha = float(action_smoothing_alpha)
        self.smooth_xyz_only = bool(smooth_xyz_only)
        self.smooth_rot_only = bool(smooth_rot_only)
        self.smooth_rpy_only = bool(smooth_rpy_only)

        # optionally warn about unused kwargs (main process only)
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
        self.wrench_hist_pad_mode = wrench_hist_pad_mode
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

        # ---- key_first_k (keep original behavior for short obs) ----
        key_first_k: Dict[str, int] = dict()
        if self.n_obs_steps is not None:
            for key in (rgb_keys + lowdim_keys):
                if key not in (extended_rgb_keys + extended_lowdim_keys):
                    key_first_k[key] = self.n_obs_steps * self.obs_downsample_ratio
            if self.add_wrench_hist:
                long_k = (self.n_obs_steps + self.force_hist - 1) * self.obs_downsample_ratio
                key_first_k[self.wrench_key] = long_k
        self.key_first_k = key_first_k

        # ---- samplers ----
        self.sampler = SequenceSampler(
            replay_buffer=replay_buffer,
            sequence_length=self.horizon + self.n_latency_steps,
            pad_before=self.pad_before,
            pad_after=self.pad_after,
            episode_mask=train_mask,
            key_first_k=key_first_k,
        )

        self.wrench_sampler = None
        if self.add_wrench_hist:
            long_seq_len = max(
                self.horizon + self.n_latency_steps,
                (self.n_obs_steps + self.force_hist - 1) * self.obs_downsample_ratio,
            )
            self.wrench_sampler = SequenceSampler(
                replay_buffer=replay_buffer,
                sequence_length=long_seq_len,
                pad_before=self.pad_before,
                pad_after=self.pad_after,
                episode_mask=train_mask,
                key_first_k=key_first_k,
            )

    def get_validation_dataset(self):
        val_set = copy.copy(self)
        val_set.sampler = SequenceSampler(
            replay_buffer=self.replay_buffer,
            sequence_length=self.horizon + self.n_latency_steps,
            pad_before=self.pad_before,
            pad_after=self.pad_after,
            episode_mask=self.val_mask,
            key_first_k=self.key_first_k,
        )
        if self.add_wrench_hist:
            long_seq_len = max(
                self.horizon + self.n_latency_steps,
                (self.n_obs_steps + self.force_hist - 1) * self.obs_downsample_ratio,
            )
            val_set.wrench_sampler = SequenceSampler(
                replay_buffer=self.replay_buffer,
                sequence_length=long_seq_len,
                pad_before=self.pad_before,
                pad_after=self.pad_after,
                episode_mask=self.val_mask,
                key_first_k=self.key_first_k,
            )
        else:
            val_set.wrench_sampler = None
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

        # obs (note: wrench_hist is NOT in shape_meta, so not normalized here)
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

    # def __len__(self):
    #     return len(self.sampler)
    def __len__(self):
        if getattr(self, "add_wrench_hist", False) and hasattr(self, "wrench_sampler") and self.wrench_sampler is not None:
            return len(self.wrench_sampler)
        return len(self.sampler)

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        threadpool_limits(1)

        # short sequence for training (keeps horizon unchanged)
        data = self.sampler.sample_sequence(idx)

        To = int(self.n_obs_steps) if self.n_obs_steps is not None else None
        ratio = int(self.obs_downsample_ratio)

        obs_dict: Dict[str, np.ndarray] = {}

        # --- RGB ---
        for key in self.rgb_keys:
            x = data[key]
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

        # --- NEW: wrench_hist (dynamic) ---
        if self.add_wrench_hist:
            assert self.wrench_sampler is not None, "wrench_sampler must be initialized when add_wrench_hist=True."
            w_data = self.wrench_sampler.sample_sequence(idx)
            if self.wrench_key not in w_data:
                raise KeyError(
                    f"wrench_key '{self.wrench_key}' not found in wrench_sampler output. "
                    f"Ensure it exists in replay_buffer and zarr_load_keys."
                )

            long_To = To + self.force_hist - 1
            w_raw = w_data[self.wrench_key][:, :6]  # (T_long,6) but may be shorter near episode boundaries

            # downsample/time-order + truncate
            w_long = _take_first_k_timeordered(w_raw, long_To * ratio, ratio).astype(np.float32)

            # robust padding if still shorter than required
            if w_long.shape[0] < long_To:
                pad_n = long_To - w_long.shape[0]
                if w_long.shape[0] > 0:
                    pad_val = w_long[:1]
                else:
                    pad_val = np.zeros((1, 6), dtype=np.float32)
                w_long = np.concatenate([np.repeat(pad_val, pad_n, axis=0), w_long], axis=0)

            w_hist_full = _build_wrench_hist(
                wrench_seq=w_long, hist_len=self.force_hist, pad_mode=self.wrench_hist_pad_mode
            )
            obs_dict[self.wrench_hist_key] = w_hist_full[self.force_hist - 1 : self.force_hist - 1 + To]  # (To,L,6)

            if self.debug_wrench_hist:
                print(
                    f"[wrench_hist] idx={idx} raw_len={w_raw.shape[0]} w_long={w_long.shape[0]} "
                    f"hist_full={w_hist_full.shape[0]} final={obs_dict[self.wrench_hist_key].shape}"
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
