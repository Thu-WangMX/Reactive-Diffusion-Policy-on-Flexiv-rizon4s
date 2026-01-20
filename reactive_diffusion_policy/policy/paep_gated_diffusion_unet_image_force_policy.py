# reactive_diffusion_policy/policy/paep_gated_diffusion_unet_image_force_policy.py
from typing import Dict, Union, Optional, Sequence
import math

import torch
import torch.nn as nn
import torch.nn.functional as F
from einops import reduce
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler

from reactive_diffusion_policy.model.common.normalizer import LinearNormalizer
from reactive_diffusion_policy.policy.base_image_policy import BaseImagePolicy
from reactive_diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D
from reactive_diffusion_policy.model.diffusion.mask_generator import LowdimMaskGenerator
from reactive_diffusion_policy.model.vision.multi_image_obs_encoder import MultiImageObsEncoder
from reactive_diffusion_policy.model.vision.timm_obs_encoder import TimmObsEncoder
from reactive_diffusion_policy.common.pytorch_util import dict_apply

# ---- PAEP ----
# adjust import path to wherever you placed paep_model.py
from paep_model import PAEPFutureNet


def imagenet_normalize(x: torch.Tensor) -> torch.Tensor:
    """
    x: (B,3,H,W) or (3,H,W), float in [0,1]
    """
    mean = torch.tensor([0.485, 0.456, 0.406], device=x.device, dtype=x.dtype).view(-1, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225], device=x.device, dtype=x.dtype).view(-1, 1, 1)
    if x.dim() == 3:
        return (x - mean) / std
    elif x.dim() == 4:
        return (x - mean.unsqueeze(0)) / std.unsqueeze(0)
    else:
        raise ValueError(f"Unexpected x.dim={x.dim()}")
    


class HeadWiseCrossAttention(nn.Module):
    """
    Minimal multi-head cross-attn with per-head gating.
    Q: (B, Nq, D)   usually Nq=1
    KV: (B, Nk, D)
    gate_h: (B, H) in [0,1]
    """
    def __init__(self, d_model: int, n_heads: int, dropout: float = 0.0):
        super().__init__()
        assert d_model % n_heads == 0
        self.d_model = d_model
        self.n_heads = n_heads
        self.d_head = d_model // n_heads
        self.q_proj = nn.Linear(d_model, d_model)
        self.k_proj = nn.Linear(d_model, d_model)
        self.v_proj = nn.Linear(d_model, d_model)
        self.out_proj = nn.Linear(d_model, d_model)
        self.drop = nn.Dropout(dropout)

    def forward(self, q: torch.Tensor, kv: torch.Tensor, gate_h: torch.Tensor) -> torch.Tensor:
        B, Nq, D = q.shape
        _, Nk, _ = kv.shape
        H = self.n_heads
        dh = self.d_head

        qh = self.q_proj(q).view(B, Nq, H, dh).transpose(1, 2)     # (B,H,Nq,dh)
        kh = self.k_proj(kv).view(B, Nk, H, dh).transpose(1, 2)    # (B,H,Nk,dh)
        vh = self.v_proj(kv).view(B, Nk, H, dh).transpose(1, 2)    # (B,H,Nk,dh)

        attn = torch.matmul(qh, kh.transpose(-2, -1)) / math.sqrt(dh)  # (B,H,Nq,Nk)
        w = torch.softmax(attn, dim=-1)
        w = self.drop(w)

        out = torch.matmul(w, vh)  # (B,H,Nq,dh)

        # per-head gate
        out = out * gate_h[:, :, None, None]

        out = out.transpose(1, 2).contiguous().view(B, Nq, D)  # (B,Nq,D)
        return self.out_proj(out)


class DualGatedVisionForceFusion(nn.Module):
    """
    V_t (vision/proprio feature) is the anchor.
    F_hist_t -> force tokens.
    PAEP prob -> (g_contact scalar) and (g_head vector).
    """
    def __init__(
        self,
        d_model: int,
        n_heads: int = 8,
        force_token_dim: int = 6,
        force_tokens: int = 48,
        gate_hidden: int = 128,
        dropout: float = 0.0,
    ):
        super().__init__()
        self.d_model = d_model
        self.n_heads = n_heads
        self.force_tokens = force_tokens

        self.force_proj = nn.Linear(force_token_dim, d_model)
        self.cross_attn = HeadWiseCrossAttention(d_model=d_model, n_heads=n_heads, dropout=dropout)

        # head-wise gate from PAEP prob
        self.head_gate_mlp = nn.Sequential(
            nn.Linear(6, gate_hidden),
            nn.ReLU(inplace=True),
            nn.Linear(gate_hidden, n_heads),
        )

    def forward(
        self,
        v_feat: torch.Tensor,          # (B, D)
        wrench_hist: torch.Tensor,     # (B, L, 6)
        g_contact: torch.Tensor,       # (B,) scalar in [0,1]
        paep_prob: torch.Tensor,       # (B, 6)
    ) -> torch.Tensor:
        B, D = v_feat.shape
        assert wrench_hist.shape[1] == self.force_tokens, f"expect L={self.force_tokens}, got {wrench_hist.shape}"

        v_tok = v_feat.unsqueeze(1)                 # (B,1,D)
        f_tok = self.force_proj(wrench_hist)        # (B,L,D)

        g_head = torch.sigmoid(self.head_gate_mlp(paep_prob))   # (B,H)
        g_head = g_head * g_contact[:, None]                    # (B,H) dual-gate merge

        delta = self.cross_attn(v_tok, f_tok, g_head)           # (B,1,D)
        v_fused = v_tok + delta                                 # residual
        return v_fused.squeeze(1)                               # (B,D)


class PAEPGatedDiffusionUnetImageForcePolicy(BaseImagePolicy):
    def __init__(
        self,
        shape_meta: dict,
        noise_scheduler: DDPMScheduler,
        obs_encoder: Union[MultiImageObsEncoder, TimmObsEncoder],
        horizon,
        n_action_steps,
        n_obs_steps,
        # diffusion
        num_inference_steps=None,
        obs_as_global_cond=True,
        diffusion_step_embed_dim=256,
        down_dims=(256, 512, 1024),
        kernel_size=5,
        n_groups=8,
        cond_predict_scale=True,
        # fusion
        n_heads: int = 8,
        force_hist: int = 48,
        wrench_hist_key: str = "wrench_hist",
        pose_key: str = "left_robot_tcp_pose",
        # paep
        paep_ckpt: Optional[str] = None,   # torch.load ckpt that contains {"model":..., "norm":...}
        paep_img_size: int = 224,
        contact_class_ids: Sequence[int] = (2, 3, 4),  # approach, under, effective, over
        gmin: float = 0.05,
        freeze_paep: bool = True,
        **kwargs,
    ):
        super().__init__()
        # shapes
        action_shape = shape_meta["action"]["shape"]
        assert len(action_shape) == 1
        action_dim = action_shape[0]

        obs_feature_dim = obs_encoder.output_shape()[0]

        input_dim = action_dim + obs_feature_dim
        global_cond_dim = None
        if obs_as_global_cond:
            input_dim = action_dim
            global_cond_dim = obs_feature_dim * n_obs_steps

        self.model = ConditionalUnet1D(
            input_dim=input_dim,
            local_cond_dim=None,
            global_cond_dim=global_cond_dim,
            diffusion_step_embed_dim=diffusion_step_embed_dim,
            down_dims=down_dims,
            kernel_size=kernel_size,
            n_groups=n_groups,
            cond_predict_scale=cond_predict_scale,
        )

        self.obs_encoder = obs_encoder
        self.noise_scheduler = noise_scheduler
        self.mask_generator = LowdimMaskGenerator(
            action_dim=action_dim,
            obs_dim=0 if obs_as_global_cond else obs_feature_dim,
            max_n_obs_steps=n_obs_steps,
            fix_obs_steps=True,
            action_visible=False,
        )

        self.normalizer = LinearNormalizer()
        self.horizon = horizon
        self.obs_feature_dim = obs_feature_dim
        self.action_dim = action_dim
        self.n_action_steps = n_action_steps
        self.n_obs_steps = n_obs_steps
        self.obs_as_global_cond = obs_as_global_cond
        self.kwargs = kwargs

        if num_inference_steps is None:
            num_inference_steps = noise_scheduler.config.num_train_timesteps
        self.num_inference_steps = num_inference_steps

        # fusion module
        self.wrench_hist_key = wrench_hist_key
        self.pose_key = pose_key
        self.fusion = DualGatedVisionForceFusion(
            d_model=obs_feature_dim,
            n_heads=n_heads,
            force_token_dim=6,
            force_tokens=force_hist,
            gate_hidden=128,
            dropout=0.0,
        )

        # PAEP
        self.paep = PAEPFutureNet(num_events=6, img_pretrained=False)  # exact arch weights come from ckpt
        self.paep_img_size = paep_img_size
        self.contact_class_ids = tuple(int(x) for x in contact_class_ids)
        self.gmin = float(gmin)

        # norm buffers (filled after loading ckpt)
        self.register_buffer("_paep_wm", torch.zeros(6), persistent=False)
        self.register_buffer("_paep_ws", torch.ones(6), persistent=False)
        self.register_buffer("_paep_pm", torch.zeros(9), persistent=False)
        self.register_buffer("_paep_ps", torch.ones(9), persistent=False)

        if paep_ckpt is not None:
            ckpt = torch.load(paep_ckpt, map_location="cpu")
            self.paep.load_state_dict(ckpt["model"], strict=True)
            norm = ckpt.get("norm", None)
            if norm is not None:
                self._paep_wm.copy_(torch.tensor(norm["wrench_mean"], dtype=torch.float32))
                self._paep_ws.copy_(torch.tensor(norm["wrench_std"], dtype=torch.float32))
                self._paep_pm.copy_(torch.tensor(norm["pose_mean"], dtype=torch.float32))
                self._paep_ps.copy_(torch.tensor(norm["pose_std"], dtype=torch.float32))

        if freeze_paep:
            for p in self.paep.parameters():
                p.requires_grad_(False)
            self.paep.eval()

    def _paep_norm_dict(self, device, dtype):
        return {
            "wrench_mean": self._paep_wm.to(device=device, dtype=dtype),
            "wrench_std": self._paep_ws.to(device=device, dtype=dtype),
            "pose_mean": self._paep_pm.to(device=device, dtype=dtype),
            "pose_std": self._paep_ps.to(device=device, dtype=dtype),
        }
    
    def _normalize_obs_safe(self, obs_dict):
        wh = obs_dict.get(self.wrench_hist_key, None)
        obs_wo = {k: v for k, v in obs_dict.items() if k != self.wrench_hist_key}
        nobs = self.normalizer.normalize(obs_wo)
        if wh is not None:
            nobs[self.wrench_hist_key] = wh  # keep raw
        return nobs


    @torch.no_grad()
    def _run_paep(self, ext_img, wrist_img, wrench_hist, pose):
        """
        ext_img/wrist_img: (B,3,H,W) float in [0,1] (dataset gives /255)
        wrench_hist: (B,L,6)
        pose: (B,9)
        return prob: (B,6)
        """
        # match infer_stream preprocessing: imagenet normalize then PAEP resizes inside encoder
        ext_n = imagenet_normalize(ext_img)
        wrist_n = imagenet_normalize(wrist_img)
        norm = self._paep_norm_dict(device=ext_img.device, dtype=ext_img.dtype)

        logits = self.paep(
            ext_n, wrist_n, wrench_hist, pose,
            norm=norm,
            img_size=self.paep_img_size,
        )  # (B,6)
        prob = torch.softmax(logits, dim=-1)
        return prob

    def _compute_g_contact(self, prob: torch.Tensor) -> torch.Tensor:
        # prob: (B,6)
        g = prob[:, list(self.contact_class_ids)].sum(dim=-1)  # (B,)
        return torch.clamp(g, min=self.gmin, max=1.0)

    def _encode_fused_obs(self, nobs: Dict[str, torch.Tensor]) -> torch.Tensor:
        """
        Return fused obs features for first n_obs_steps:
          (B, n_obs_steps, D)
        """
        B = next(iter(nobs.values())).shape[0]
        To = self.n_obs_steps

        # ---- prepare per-step tensors ----
        # images
        ext = nobs["external_img"][:, :To]      # (B,To,3,H,W)
        wrist = nobs["left_wrist_img"][:, :To]  # (B,To,3,H,W)

        # wrench_hist
        wrench_hist = nobs[self.wrench_hist_key][:, :To]  # (B,To,L,6)
        pose = nobs[self.pose_key][:, :To]                # (B,To,9)

        # ---- obs_encoder features (DP-consistent) ----
        # IMPORTANT: do NOT pass wrench_hist to obs_encoder
        exclude = {self.wrench_hist_key}
        enc_nobs = {k: v[:, :To] for k, v in nobs.items() if (k not in exclude)}

        # reshape B,To -> (B*To, ...)
        flat_nobs = dict_apply(enc_nobs, lambda x: x.reshape(-1, *x.shape[2:]))
        v_feat = self.obs_encoder(flat_nobs)  # (B*To, D)

        # ---- PAEP gating + fusion ----
        flat_ext = ext.reshape(-1, *ext.shape[2:])
        flat_wrist = wrist.reshape(-1, *wrist.shape[2:])
        flat_wrench = wrench_hist.reshape(-1, *wrench_hist.shape[2:])  # (B*To,L,6)
        flat_pose = pose.reshape(-1, *pose.shape[2:])                  # (B*To,9)

        paep_prob = self._run_paep(flat_ext, flat_wrist, flat_wrench, flat_pose)  # (B*To,6)
        g_contact = self._compute_g_contact(paep_prob)                              # (B*To,)

        v_fused = self.fusion(
            v_feat=v_feat,
            wrench_hist=flat_wrench,
            g_contact=g_contact,
            paep_prob=paep_prob,
        )  # (B*To,D)

        return v_fused.reshape(B, To, -1)

    # ========= inference =========
    def conditional_sample(
        self,
        condition_data, condition_mask,
        local_cond=None, global_cond=None,
        generator=None,
        **kwargs,
    ):
        model = self.model
        scheduler = self.noise_scheduler

        trajectory = torch.randn(
            size=condition_data.shape,
            dtype=condition_data.dtype,
            device=condition_data.device,
            generator=generator,
        )

        scheduler.set_timesteps(self.num_inference_steps)

        for t in scheduler.timesteps:
            trajectory[condition_mask] = condition_data[condition_mask]
            model_output = model(trajectory, t, local_cond=local_cond, global_cond=global_cond)
            trajectory = scheduler.step(model_output, t, trajectory, generator=generator, **kwargs).prev_sample

        trajectory[condition_mask] = condition_data[condition_mask]
        return trajectory

    def predict_action(self, obs_dict: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        # normalize
        #nobs = self.normalizer.normalize(obs_dict)
        nobs = self._normalize_obs_safe(obs_dict)
        B = next(iter(nobs.values())).shape[0]
        To = self.n_obs_steps

        global_cond = None
        local_cond = None

        if self.obs_as_global_cond:
            fused = self._encode_fused_obs(nobs)          # (B,To,D)
            global_cond = fused.reshape(B, -1)            # (B, To*D)
        else:
            raise NotImplementedError("Keep obs_as_global_cond=True for now (DP-consistent).")

        nactions = torch.zeros((B, self.horizon, self.action_dim), device=global_cond.device, dtype=global_cond.dtype)
        cond_data = nactions.detach()
        trajectory = cond_data

        condition_mask = self.mask_generator(trajectory.shape)
        nsample = self.conditional_sample(
            condition_data=cond_data,
            condition_mask=condition_mask,
            local_cond=local_cond,
            global_cond=global_cond,
            **self.kwargs,
        )

        naction_pred = nsample[:, : self.n_action_steps]  # (B,Ta,Da)
        action_pred = self.normalizer["action"].unnormalize(naction_pred)

        return {"action": action_pred}

    # ========= training =========
    def compute_loss(self, batch) -> torch.Tensor:
        obs = batch["obs"]
        action = batch["action"]

        #nobs = self.normalizer.normalize(obs)
        nobs = self._normalize_obs_safe(obs)
        nactions = self.normalizer["action"].normalize(action)

        B = action.shape[0]
        horizon = action.shape[1]

        global_cond = None
        local_cond = None

        if self.obs_as_global_cond:
            fused = self._encode_fused_obs(nobs)          # (B,To,D)
            global_cond = fused.reshape(B, -1)
            trajectory = nactions
        else:
            raise NotImplementedError("Keep obs_as_global_cond=True for now (DP-consistent).")

        condition_mask = self.mask_generator(trajectory.shape)
        noise = torch.randn(trajectory.shape, device=trajectory.device)
        bsz = trajectory.shape[0]
        timesteps = torch.randint(
            0, self.noise_scheduler.config.num_train_timesteps,
            (bsz,), device=trajectory.device
        ).long()

        noisy_trajectory = self.noise_scheduler.add_noise(trajectory, noise, timesteps)

        loss_mask = ~condition_mask
        noisy_trajectory[condition_mask] = trajectory[condition_mask]

        pred = self.model(noisy_trajectory, timesteps, local_cond=local_cond, global_cond=global_cond)

        pred_type = self.noise_scheduler.config.prediction_type
        if pred_type == "epsilon":
            target = noise
        elif pred_type == "sample":
            target = trajectory
        else:
            raise ValueError(f"Unsupported prediction type {pred_type}")

        loss = F.mse_loss(pred, target, reduction="none")
        loss = loss * loss_mask.type(loss.dtype)
        loss = reduce(loss, "b ... -> b (...)", "mean").mean()
        return loss

    def forward(self, batch):
        return self.compute_loss(batch)
