from typing import Dict, Optional
import torch
import numpy as np
import os
from threadpoolctl import threadpool_limits
import copy
import tqdm
from reactive_diffusion_policy.common.pytorch_util import dict_apply
from reactive_diffusion_policy.dataset.base_dataset import BaseImageDataset
from reactive_diffusion_policy.model.common.normalizer import LinearNormalizer, SingleFieldLinearNormalizer
from reactive_diffusion_policy.common.replay_buffer import ReplayBuffer
from reactive_diffusion_policy.common.sampler import (
    SequenceSampler, get_val_mask, downsample_mask
)
from reactive_diffusion_policy.common.normalize_util import (
    get_image_range_normalizer,
    get_action_normalizer
)
from reactive_diffusion_policy.common.action_utils import (
    absolute_actions_to_relative_actions,
    get_inter_gripper_actions
)
from reactive_diffusion_policy.real_world.real_world_transforms import RealWorldTransforms


def _build_wrench_hist(
    wrench_seq: np.ndarray,
    hist_len: int,
    pad_mode: str = "repeat_first"
) -> np.ndarray:
    """
    Build per-timestep wrench history windows.

    Args:
        wrench_seq: (T, 6) float32
        hist_len: L
        pad_mode: "repeat_first" (recommended) or "zeros"

    Returns:
        wrench_hist: (T, L, 6)
        - for each t, wrench_hist[t] contains wrench_seq[max(0,t-L+1):t+1] right-aligned.
        - left padding is either repeated first available sample or zeros.
    """
    assert wrench_seq.ndim == 2 and wrench_seq.shape[1] == 6, f"wrench_seq must be (T,6), got {wrench_seq.shape}"
    T = wrench_seq.shape[0]
    L = int(hist_len)
    out = np.zeros((T, L, 6), dtype=np.float32)

    for t in range(T):
        s = max(0, t - L + 1)
        seg = wrench_seq[s:t + 1]  # (<=L,6)
        k = seg.shape[0]
        out[t, -k:] = seg
        if k < L:
            if pad_mode == "repeat_first":
                out[t, :L - k] = seg[:1]  # repeat earliest available in this window
            elif pad_mode == "zeros":
                # already zeros
                pass
            else:
                raise ValueError(f"Unknown pad_mode={pad_mode}")
    return out


class RealImageTactileDataset(BaseImageDataset):
    def __init__(
        self,
        shape_meta: dict,
        dataset_path: str,
        horizon=1,
        pad_before=0,
        pad_after=0,
        n_obs_steps=None,
        obs_temporal_downsample_ratio=1,  # you will set 1 for 24Hz
        n_latency_steps=0,
        seed=42,
        val_ratio=0.0,
        max_train_episodes=None,
        delta_action=False,
        relative_action=False,
        relative_tcp_obs_for_relative_action=True,
        transform_params=None,
        # --- NEW: wrench history (dynamic, not stored in zarr) ---
        add_wrench_hist: bool = True,
        force_hist: int = 48,
        wrench_key: str = "left_robot_tcp_wrench",
        wrench_hist_key: str = "wrench_hist",
        wrench_hist_pad_mode: str = "repeat_first",
    ):
        assert os.path.isdir(dataset_path)

        rgb_keys = list()
        lowdim_keys = list()
        obs_shape_meta = shape_meta["obs"]
        for key, attr in obs_shape_meta.items():
            t = attr.get("type", "low_dim")
            if t == "rgb":
                rgb_keys.append(key)
            elif t == "low_dim":
                lowdim_keys.append(key)

        extended_rgb_keys = list()
        extended_lowdim_keys = list()
        extended_obs_shape_meta = shape_meta.get("extended_obs", dict())
        for key, attr in extended_obs_shape_meta.items():
            t = attr.get("type", "low_dim")
            if t == "rgb":
                extended_rgb_keys.append(key)
            elif t == "low_dim":
                extended_lowdim_keys.append(key)

        zarr_path = os.path.join(dataset_path, "replay_buffer.zarr")
        zarr_load_keys = set(rgb_keys + lowdim_keys + extended_rgb_keys + extended_lowdim_keys + ["action"])

        # NOTE: keep behavior consistent with original code
        zarr_load_keys = list(filter(lambda key: "wrt" not in key, zarr_load_keys))

        # If we want wrench_hist, we must have wrench_key loaded.
        # This makes the dataset robust even if user forgot to add wrench_key to shape_meta.
        if add_wrench_hist and (wrench_key not in zarr_load_keys):
            zarr_load_keys.append(wrench_key)

        replay_buffer = ReplayBuffer.copy_from_path(zarr_path, keys=zarr_load_keys)

        if delta_action:
            # replace action as relative to previous frame
            actions = replay_buffer["action"][:]
            # support positions only at this time
            assert actions.shape[1] <= 3
            actions_diff = np.zeros_like(actions)
            episode_ends = replay_buffer.episode_ends[:]
            for i in range(len(episode_ends)):
                start = 0
                if i > 0:
                    start = episode_ends[i - 1]
                end = episode_ends[i]
                actions_diff[start + 1 : end] = np.diff(actions[start:end], axis=0)
            replay_buffer["action"][:] = actions_diff

        self.relative_action = relative_action
        self.relative_tcp_obs_for_relative_action = relative_tcp_obs_for_relative_action
        self.transforms = RealWorldTransforms(option=transform_params)

        # NEW: wrench history config
        self.add_wrench_hist = add_wrench_hist
        self.force_hist = int(force_hist)
        self.wrench_key = wrench_key
        self.wrench_hist_key = wrench_hist_key
        self.wrench_hist_pad_mode = wrench_hist_pad_mode

        key_first_k = dict()
        if n_obs_steps is not None:
            # only take first k obs from images/lowdim (non-extended) keys
            for key in rgb_keys + lowdim_keys:
                if key not in extended_rgb_keys + extended_lowdim_keys:
                    key_first_k[key] = n_obs_steps * obs_temporal_downsample_ratio
        self.key_first_k = key_first_k

        self.seed = seed
        val_mask = get_val_mask(
            n_episodes=replay_buffer.n_episodes,
            val_ratio=val_ratio,
            seed=seed,
        )
        train_mask = ~val_mask
        train_mask = downsample_mask(mask=train_mask, max_n=max_train_episodes, seed=seed)

        sampler = SequenceSampler(
            replay_buffer=replay_buffer,
            sequence_length=horizon + n_latency_steps,
            pad_before=pad_before,
            pad_after=pad_after,
            episode_mask=train_mask,
            key_first_k=key_first_k,
        )

        self.replay_buffer = replay_buffer
        self.sampler = sampler
        self.shape_meta = shape_meta
        self.rgb_keys = rgb_keys
        self.lowdim_keys = lowdim_keys
        self.extended_rgb_keys = extended_rgb_keys
        self.extended_lowdim_keys = extended_lowdim_keys
        self.n_obs_steps = n_obs_steps
        self.obs_downsample_ratio = obs_temporal_downsample_ratio
        self.val_mask = val_mask
        self.horizon = horizon
        self.n_latency_steps = n_latency_steps
        self.pad_before = pad_before
        self.pad_after = pad_after

    def get_validation_dataset(self):
        val_set = copy.copy(self)
        val_set.sampler = SequenceSampler(
            replay_buffer=self.replay_buffer,
            sequence_length=self.horizon + self.n_latency_steps,
            pad_before=self.pad_before,
            pad_after=self.pad_after,
            episode_mask=self.val_mask,
        )
        val_set.val_mask = ~self.val_mask
        return val_set

    def get_normalizer(self, **kwargs) -> LinearNormalizer:
        normalizer = LinearNormalizer()

        # calculate relative action / obs
        if (
            "left_robot_wrt_right_robot_tcp_pose" in self.lowdim_keys
            or "right_robot_wrt_left_robot_tcp_pose" in self.lowdim_keys
        ):
            inter_gripper_data_dict = {
                key: list()
                for key in self.lowdim_keys
                if "robot_tcp_pose" in key and "wrt" in key
            }
            for data in tqdm.tqdm(self, leave=False, desc="Calculating inter-gripper relative obs for normalizer"):
                for key in inter_gripper_data_dict.keys():
                    inter_gripper_data_dict[key].append(data["obs"][key])
            inter_gripper_data_dict = dict_apply(inter_gripper_data_dict, np.stack)

        if self.relative_action:
            relative_data_dict = {
                key: list()
                for key in (self.lowdim_keys + ["action"])
                if (("robot_tcp_pose" in key and "wrt" not in key) or ("action" in key))
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

        # obs (NOTE: wrench_hist is NOT in shape_meta, so it won't be normalized here)
        for key in list(set(self.lowdim_keys)):
            if self.relative_action and key in relative_data_dict:
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
                if self.relative_action and key in relative_data_dict:
                    normalizer[key] = get_action_normalizer(relative_data_dict[key])
                elif "robot_tcp_pose" in key and "wrt" in key:
                    normalizer[key] = get_action_normalizer(inter_gripper_data_dict[key])
                elif "robot_tcp_pose" in key and "wrt" not in key:  # not used now
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

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        threadpool_limits(1)
        data = self.sampler.sample_sequence(idx)

        # only return first n_obs_steps of OBS (if provided)
        T_slice = slice(self.n_obs_steps)
        obs_downsample_ratio = self.obs_downsample_ratio

        obs_dict: Dict[str, np.ndarray] = dict()

        # --- RGB ---
        for key in self.rgb_keys:
            # T,H,W,C -> T,C,H,W and float32 in [0,1]
            obs_dict[key] = (
                np.moveaxis(data[key][T_slice][::-obs_downsample_ratio][::-1], -1, 1).astype(np.float32) / 255.0
            )
            if key not in self.rgb_keys:
                del data[key]

        # --- Low-dim ---
        for key in self.lowdim_keys:
            if "wrt" not in key:
                obs_dict[key] = (
                    data[key][:, : self.shape_meta["obs"][key]["shape"][0]][T_slice][::-obs_downsample_ratio][::-1]
                    .astype(np.float32)
                )
                if key not in self.extended_lowdim_keys:
                    del data[key]

        # inter-gripper relative action
        obs_dict.update(get_inter_gripper_actions(obs_dict, self.lowdim_keys, self.transforms))
        for key in ["left_robot_wrt_right_robot_tcp_pose", "right_robot_wrt_left_robot_tcp_pose"]:
            if key in obs_dict:
                obs_dict[key] = obs_dict[key][:, : self.shape_meta["obs"][key]["shape"][0]].astype(np.float32)

        # --- Extended obs ---
        extended_obs_dict: Dict[str, np.ndarray] = dict()
        for key in self.extended_rgb_keys:
            extended_obs_dict[key] = np.moveaxis(data[key], -1, 1).astype(np.float32) / 255.0
            del data[key]
        for key in self.extended_lowdim_keys:
            if "wrt" not in key:
                extended_obs_dict[key] = data[key][:, : self.shape_meta["extended_obs"][key]["shape"][0]].astype(np.float32)
                del data[key]

        # --- NEW: wrench_hist (dynamic) ---
        if self.add_wrench_hist:
            wrench_seq: Optional[np.ndarray] = None

            # prefer already-processed (downsampled, time-ordered) version if present
            if self.wrench_key in obs_dict:
                wrench_seq = obs_dict[self.wrench_key]
            elif self.wrench_key in data:
                # if wrench not declared in shape_meta but loaded via zarr_load_keys
                wrench_seq = (
                    data[self.wrench_key][:, :6][T_slice][::-obs_downsample_ratio][::-1].astype(np.float32)
                )

            if wrench_seq is None:
                raise KeyError(
                    f"add_wrench_hist=True but wrench_key '{self.wrench_key}' not found in obs_dict/data. "
                    f"Make sure zarr has '{self.wrench_key}' and it is loaded."
                )

            obs_dict[self.wrench_hist_key] = _build_wrench_hist(
                wrench_seq=wrench_seq, hist_len=self.force_hist, pad_mode=self.wrench_hist_pad_mode
            )

        # --- Action ---
        action = data["action"][:, : self.shape_meta["action"]["shape"][0]].astype(np.float32)
        if self.n_latency_steps > 0:
            action = action[self.n_latency_steps :]

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
                    if "robot_tcp_pose" in key and "wrt" not in key and key in obs_dict:
                        obs_dict[key] = absolute_actions_to_relative_actions(obs_dict[key], base_absolute_action=base_absolute_action)

        torch_data = {
            "obs": dict_apply(obs_dict, torch.from_numpy),
            "action": torch.from_numpy(action),
            "extended_obs": dict_apply(extended_obs_dict, torch.from_numpy),
        }
        return torch_data


def test():
    import hydra
    from hydra import initialize, compose
    from omegaconf import OmegaConf

    OmegaConf.register_new_resolver("eval", eval, replace=True)

    with initialize("../config"):
        cfg = hydra.compose(
            "train_diffusion_unet_real_image_workspace",
            overrides=["task=real_peel_image_gelsight_emb_absolute_12fps"],
        )
        OmegaConf.resolve(cfg)
        dataset = hydra.utils.instantiate(cfg.task.dataset)

    normalizer = dataset.get_normalizer()
    for i in range(min(5, len(dataset))):
        data = dataset[i]
        # print(data["obs"].keys())


if __name__ == "__main__":
    test()
