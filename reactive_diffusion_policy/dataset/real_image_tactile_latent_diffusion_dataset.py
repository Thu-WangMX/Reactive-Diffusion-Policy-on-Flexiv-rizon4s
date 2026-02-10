# import einops
# import numpy as np
# import tqdm
# from reactive_diffusion_policy.model.vae.model import VAE
# from reactive_diffusion_policy.dataset.real_image_tactile_dataset_v0 import RealImageTactileDataset
# from reactive_diffusion_policy.model.common.normalizer import LinearNormalizer, SingleFieldLinearNormalizer

# class RealImageTactileLatentDiffusionDataset(RealImageTactileDataset):
#     def __init__(self,
#                  at: VAE,
#                  use_latent_action_before_vq: bool,
#                  **kwargs):
#         super().__init__(**kwargs)
#         self.at = at
#         self.at.eval()
#         self.use_latent_action_before_vq = use_latent_action_before_vq

#     def get_normalizer(self, **kwargs) -> LinearNormalizer:
#         normalizer = super().get_normalizer(**kwargs)

#         latent_action_all = []

#         for data in tqdm.tqdm(self, leave=False, desc='Calculating latent action for normalizer'):
#             action = data['action'].to(self.at.device).unsqueeze(0)
#             action = normalizer['action'].normalize(action)
#             latent_action = self.at.encoder(
#                 self.at.preprocess(action / self.at.act_scale)
#             )
#             if self.at.use_vq:
#                 if not self.use_latent_action_before_vq:
#                     latent_action, _, _ = self.at.quant_state_with_vq(latent_action)
#             else:
#                 latent_action, _ = self.at.quant_state_without_vq(latent_action)
#             if self.at.use_conv_encoder:
#                 latent_action = einops.rearrange(latent_action, "N (T A) -> N T A", T=self.at.downsampled_input_h)
#             else:
#                 latent_action = einops.rearrange(latent_action, "N (T A) -> N T A", T=1)
#             latent_action_all.append(latent_action[0].cpu().detach().numpy())

#         latent_action_all = np.concatenate(latent_action_all, axis=0)

#         normalizer['latent_action'] = SingleFieldLinearNormalizer.create_fit(latent_action_all)

#         return normalizer
import einops
import numpy as np
import torch
import tqdm
from torch.utils.data import DataLoader

from reactive_diffusion_policy.model.vae.model import VAE
from reactive_diffusion_policy.dataset.real_image_tactile_dataset_v0 import RealImageTactileDataset
from reactive_diffusion_policy.model.common.normalizer import LinearNormalizer, SingleFieldLinearNormalizer
from reactive_diffusion_policy.common.pytorch_util import dict_apply
from reactive_diffusion_policy.common.action_utils import absolute_actions_to_relative_actions, get_inter_gripper_actions
from threadpoolctl import threadpool_limits


class RealImageTactileLatentDiffusionDataset(RealImageTactileDataset):
    def __init__(self,
                 at: VAE,
                 use_latent_action_before_vq: bool,
                 **kwargs):
        super().__init__(**kwargs)
        self.at = at
        self.at.eval()
        self.use_latent_action_before_vq = use_latent_action_before_vq
        self._mute_images = False

    def __getitem__(self, idx: int):
        """
        保持官方 __getitem__ 功能：
        - 采样 sequence
        - downsample + time reverse（官方写法是 [::-ratio][::-1]）
        - inter-gripper wrt 计算
        - relative_action / relative_tcp_obs_for_relative_action 处理

        但在 get_normalizer / latent-stats 阶段可开启 self._mute_images=True：
        - 不加载任何图像 key（省内存/加速）
        """
        if not getattr(self, "_mute_images", False):
            return super().__getitem__(idx)

        threadpool_limits(1)
        data = self.sampler.sample_sequence(idx)

        T_slice = slice(self.n_obs_steps)
        obs_downsample_ratio = self.obs_downsample_ratio

        # --------- 只构造 low-dim obs（不读图像）---------
        obs_dict = {}
        for key in self.lowdim_keys:
            if 'wrt' not in key:
                obs_dict[key] = data[key][:, :self.shape_meta['obs'][key]['shape'][0]][T_slice][::-obs_downsample_ratio][::-1].astype(np.float32)

        # inter-gripper relative obs（官方一致）
        obs_dict.update(get_inter_gripper_actions(obs_dict, self.lowdim_keys, self.transforms))
        for key in ['left_robot_wrt_right_robot_tcp_pose', 'right_robot_wrt_left_robot_tcp_pose']:
            if key in obs_dict:
                obs_dict[key] = obs_dict[key][:, :self.shape_meta['obs'][key]['shape'][0]].astype(np.float32)

        # --------- extended_obs：只构造 extended_lowdim（不读图像）---------
        extended_obs_dict = {}
        for key in self.extended_lowdim_keys:
            if 'wrt' not in key:
                extended_obs_dict[key] = data[key][:, :self.shape_meta['extended_obs'][key]['shape'][0]].astype(np.float32)

        # --------- action（官方一致）---------
        action = data['action'][:, :self.shape_meta['action']['shape'][0]].astype(np.float32)
        if self.n_latency_steps > 0:
            action = action[self.n_latency_steps:]

        # --------- relative_action（官方一致）---------
        if self.relative_action:
            base_absolute_action = np.concatenate([
                obs_dict['left_robot_tcp_pose'][-1] if 'left_robot_tcp_pose' in obs_dict else np.array([]),
                obs_dict['right_robot_tcp_pose'][-1] if 'right_robot_tcp_pose' in obs_dict else np.array([])
            ], axis=-1)
            action = absolute_actions_to_relative_actions(action, base_absolute_action=base_absolute_action)

            if self.relative_tcp_obs_for_relative_action:
                for key in self.lowdim_keys:
                    if 'robot_tcp_pose' in key and 'wrt' not in key:
                        obs_dict[key] = absolute_actions_to_relative_actions(obs_dict[key], base_absolute_action=base_absolute_action)

        torch_data = {
            'obs': dict_apply(obs_dict, torch.from_numpy),
            'action': torch.from_numpy(action),
            'extended_obs': dict_apply(extended_obs_dict, torch.from_numpy)
        }
        return torch_data

    def get_normalizer(self, **kwargs) -> LinearNormalizer:
        print("[Dataset] Starting Normalizer calculation (RDP-compatible)...")

        # 关键：不改 rgb_keys / extended_rgb_keys
        # 只是在 normalizer 计算期 “mute image loading”
        self._mute_images = True
        try:
            # 1) 先走官方 normalizer（会注册 action/lowdim/extended_lowdim + image range normalizer）
            normalizer = super().get_normalizer(**kwargs)

            # 2) 计算 latent_action normalizer（只需要 action，所以仍然 mute images）
            # device：不要用 self.at.parameters()（VAE 可能不是 nn.Module）；优先 encoder
            try:
                device = next(self.at.encoder.parameters()).device
            except Exception:
                device = torch.device("cpu")

            self.at.eval()
            latent_action_all = []

            loader = DataLoader(
                self,
                batch_size=256,
                shuffle=False,
                num_workers=0,
                pin_memory=False
            )

            print(f"[Dataset] Calculating Latent Actions on {device} (mute_images={self._mute_images})...")

            with torch.no_grad():
                for batch in tqdm.tqdm(loader, desc='Calculating Latent Actions'):
                    actions = batch['action'].to(device)
                    actions = normalizer['action'].normalize(actions)

                    act_scale = self.at.act_scale
                    if torch.is_tensor(act_scale):
                        act_scale = act_scale.to(device=actions.device, dtype=actions.dtype)

                    encoder_input = self.at.preprocess(actions / act_scale)
                    latent_action = self.at.encoder(encoder_input)

                    if self.at.use_vq:
                        if not self.use_latent_action_before_vq:
                            latent_action, _, _ = self.at.quant_state_with_vq(latent_action)
                    else:
                        latent_action, _ = self.at.quant_state_without_vq(latent_action)

                    T_out = self.at.downsampled_input_h if self.at.use_conv_encoder else 1
                    latent_action = einops.rearrange(latent_action, "N (T A) -> N T A", T=T_out)
                    latent_action_all.append(latent_action.cpu().numpy())

            if len(latent_action_all) > 0:
                latent_action_all = np.concatenate(latent_action_all, axis=0)
                normalizer['latent_action'] = SingleFieldLinearNormalizer.create_fit(latent_action_all)
                print(f"[Dataset] Latent stats calculated. Shape: {latent_action_all.shape}")
            else:
                print("[Warning] No latent actions calculated!")

            return normalizer

        finally:
            self._mute_images = False
            print("[Dataset] Restored image loading for training.")
