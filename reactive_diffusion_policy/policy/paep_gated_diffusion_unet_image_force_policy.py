# ============================================================
# PAEP-Gated Diffusion Unet Image+Force Policy (FULL, FIXED + W&B DEBUG)
# - Fix crash: NEVER use `if k in self.normalizer` (LinearNormalizer lacks __contains__)
# - Fix: set_normalizer() now .to(device)
# - Fix: compute_loss() restores scheduler.prediction_type + mask + masked MSE reduction
# - Fix: PAEP input restores ImageNet normalize + uses ckpt norm dict (as in v1 that ran)
# - W&B logging:
#   * every `log_wandb_every` steps: g_contact mean/min/max, paep entropy/maxprob, argmax dist (bar)
#   * cross-attn/force injection stats: delta_norm_mean, inj_ratio_mean, etc.
#   * IMPORTANT PERF FIX: NO .item() in forward() (avoid GPU sync each step)
# ============================================================

from typing import Dict, Union, Optional, Sequence, Tuple
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

# ---- PAEP robust import ----
import importlib.util
from pathlib import Path


def _load_paep_future_net():
    """
    Robustly load PAEPFutureNet regardless of PYTHONPATH / package layout.
    We search repo_root/PAEP for a python file that defines PAEPFutureNet.
    """
    repo_root = Path(__file__).resolve().parents[2]
    paep_dir = repo_root / "PAEP"

    candidates = [
        paep_dir / "paep_model.py",
        paep_dir / "models" / "paep_model.py",
    ]

    if paep_dir.exists():
        for p in paep_dir.rglob("*.py"):
            if p.name.startswith("__"):
                continue
            try:
                txt = p.read_text(encoding="utf-8", errors="ignore")
            except Exception:
                continue
            if "PAEPFutureNet" in txt:
                candidates.append(p)

    for p in candidates:
        if not p.exists():
            continue
        try:
            spec = importlib.util.spec_from_file_location("paep_dyn", str(p))
            mod = importlib.util.module_from_spec(spec)
            assert spec and spec.loader
            spec.loader.exec_module(mod)
            if hasattr(mod, "PAEPFutureNet"):
                print(f"[PAEP] Loaded PAEPFutureNet from: {p}")
                return getattr(mod, "PAEPFutureNet"), str(p)
        except Exception:
            continue

    raise ModuleNotFoundError(
        f"Cannot find PAEPFutureNet under {paep_dir}. "
        f"Make sure PAEPFutureNet is defined in some .py file under PAEP/."
    )


PAEPFutureNet, _PAEP_SRC = _load_paep_future_net()


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


# ============================================================
# Head-wise cross attention with per-head gating
# ============================================================
class HeadWiseCrossAttention(nn.Module):
    """
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

        # debug cache (tensors only; no .item() here!)
        self._last_attn_debug = None

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

        # per-head gate inside attention
        out = out * gate_h[:, :, None, None]

        out = out.transpose(1, 2).contiguous().view(B, Nq, D)  # (B,Nq,D)
        y = self.out_proj(out)

        # ---- debug (tensors only, no sync) ----
        with torch.no_grad():
            # gate stats
            self._last_attn_debug = {
                "g_head_mean": gate_h.mean().detach(),
                "g_head_min": gate_h.min().detach(),
                "g_head_max": gate_h.max().detach(),
                # attention sharpness: max weight and entropy over Nk
                "attn_max_mean": w.max(dim=-1).values.mean().detach(),  # (B,H,Nq)->scalar
                "attn_entropy_mean": (-(w.clamp_min(1e-9).log() * w).sum(dim=-1)).mean().detach(),
            }

        return y


# ============================================================
# Dual-gated vision-force fusion
#   - head gate inside attn
#   - contact gate scales whole residual
# ============================================================
class DualGatedVisionForceFusion(nn.Module):
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

        self.force_proj = nn.Sequential(
            nn.Linear(force_token_dim, d_model),
            nn.ReLU(inplace=True),
            nn.Linear(d_model, d_model),
        )

        self.cross_attn = HeadWiseCrossAttention(d_model=d_model, n_heads=n_heads, dropout=dropout)

        self.head_gate_mlp = nn.Sequential(
            nn.Linear(6, gate_hidden),
            nn.ReLU(inplace=True),
            nn.Linear(gate_hidden, n_heads),
        )

        # debug cache (tensors only)
        self._last_fusion_debug = None

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

        delta = self.cross_attn(v_tok, f_tok, g_head)           # (B,1,D)
        v_fused = v_tok + (g_contact[:, None, None] * delta)    # scalar contact switch

        # ---- debug (tensors only, no .item() / no sync) ----
        with torch.no_grad():
            dn = delta.squeeze(1).norm(dim=-1)                  # (B,)
            vn = v_feat.norm(dim=-1)                            # (B,)
            inj = (g_contact * dn)                              # (B,)
            self._last_fusion_debug = {
                "delta_norm_mean": dn.mean().detach(),
                "v_norm_mean": vn.mean().detach(),
                "inj_norm_mean": inj.mean().detach(),
                "inj_ratio_mean": (inj / (vn + 1e-6)).mean().detach(),
            }

        return v_fused.squeeze(1)                               # (B,D)


# ============================================================
# Policy
# ============================================================
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
        gripper_key: str = "left_robot_gripper_width",
        zero_proprio_in_obs_encoder: bool = True,
        # paep
        paep_ckpt: Optional[str] = None,
        paep_img_size: int = 224,
        contact_class_ids: Sequence[int] = (2, 3, 4),  # under/effective/over
        gmin: float = 0.05,
        freeze_paep: bool = True,
        # logging
        log_wandb_every: int = 30,   # recommended default (per-epoch a few logs)
        **kwargs,
    ):
        super().__init__()
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
        self._extra_step_log = None
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

        # ---- logging ----
        self.log_wandb_every = int(log_wandb_every)
        self._train_step = 0
        self._last_debug = None

        # fusion + proprio
        self.wrench_hist_key = wrench_hist_key
        self.pose_key = pose_key
        self.gripper_key = gripper_key
        self.zero_proprio_in_obs_encoder = bool(zero_proprio_in_obs_encoder)

        # proprioception: tcp_pose(9) + gripper_width(1) = 10D
        self.proprio_dim = 10
        self.proprio_mlp = nn.Sequential(
            nn.Linear(self.proprio_dim, obs_feature_dim),
            nn.ReLU(inplace=True),
            nn.Linear(obs_feature_dim, obs_feature_dim),
        )

        self.fusion = DualGatedVisionForceFusion(
            d_model=obs_feature_dim,
            n_heads=n_heads,
            force_token_dim=6,
            force_tokens=force_hist,
            gate_hidden=128,
            dropout=0.0,
        )

        # PAEP
        self.paep = PAEPFutureNet(num_events=6, img_pretrained=False)
        self.paep_img_size = int(paep_img_size)
        self.contact_class_ids = tuple(int(x) for x in contact_class_ids)
        self.gmin = float(gmin)

        self.register_buffer("_paep_wm", torch.zeros(6), persistent=False)
        self.register_buffer("_paep_ws", torch.ones(6), persistent=False)
        self.register_buffer("_paep_pm", torch.zeros(9), persistent=False)
        self.register_buffer("_paep_ps", torch.ones(9), persistent=False)

        if paep_ckpt is not None:
            ckpt = torch.load(paep_ckpt, map_location="cpu")
            self.paep.load_state_dict(ckpt["model"], strict=True)
            norm = ckpt.get("norm", None)
            if norm is not None:
                self._paep_wm.copy_(torch.as_tensor(norm["wrench_mean"]).float())
                self._paep_ws.copy_(torch.as_tensor(norm["wrench_std"]).float())
                self._paep_pm.copy_(torch.as_tensor(norm["pose_mean"]).float())
                self._paep_ps.copy_(torch.as_tensor(norm["pose_std"]).float())

        if freeze_paep:
            for p in self.paep.parameters():
                p.requires_grad_(False)
            self.paep.eval()

    # ------------------------------------------------------------
    # Normalizer helpers (CRASH FIX: never use `k in self.normalizer`)
    # ------------------------------------------------------------
    def _has_norm_key(self, k: str) -> bool:
        return isinstance(k, str) and hasattr(self.normalizer, "params_dict") and (k in self.normalizer.params_dict)

    def set_normalizer(self, normalizer):
        if normalizer is None:
            return
        if isinstance(normalizer, LinearNormalizer):
            self.normalizer.load_state_dict(normalizer.state_dict())
        elif isinstance(normalizer, dict):
            self.normalizer.load_state_dict(normalizer)
        else:
            try:
                self.normalizer.load_state_dict(normalizer.state_dict())
            except Exception:
                self.normalizer = normalizer

        self.normalizer.to(next(self.parameters()).device)

    # ------------------------------------------------------------
    # obs split: raw vs dp-normalized
    # ------------------------------------------------------------
    def _split_raw_and_dp_obs(
        self, obs: Dict[str, torch.Tensor]
    ) -> Tuple[Dict[str, torch.Tensor], Dict[str, torch.Tensor]]:
        obs_raw: Dict[str, torch.Tensor] = {}
        obs_dp: Dict[str, torch.Tensor] = {}

        for k, v in obs.items():
            # RAW
            if torch.is_tensor(v) and v.dtype == torch.uint8:
                obs_raw[k] = v.float() / 255.0
            else:
                obs_raw[k] = v

            # DP-normalized
            if isinstance(k, str) and self._has_norm_key(k):
                obs_dp[k] = self.normalizer[k].normalize(v)
            else:
                obs_dp[k] = v if v.dtype != torch.uint8 else (v.float() / 255.0)

        return obs_raw, obs_dp

    # ------------------------------------------------------------
    # PAEP helpers
    # ------------------------------------------------------------
    def _paep_norm_dict(self, device, dtype):
        return {
            "wrench_mean": self._paep_wm.to(device=device, dtype=dtype),
            "wrench_std": self._paep_ws.to(device=device, dtype=dtype),
            "pose_mean": self._paep_pm.to(device=device, dtype=dtype),
            "pose_std": self._paep_ps.to(device=device, dtype=dtype),
        }

    @torch.no_grad()
    def _run_paep(
        self,
        ext_img: torch.Tensor,       # (B,3,H,W) float in [0,1]
        wrist_img: torch.Tensor,     # (B,3,H,W) float in [0,1]
        wrench_hist: torch.Tensor,   # (B,L,6) RAW
        pose: torch.Tensor,          # (B,9) RAW
    ) -> torch.Tensor:
        if ext_img.shape[-1] != self.paep_img_size or ext_img.shape[-2] != self.paep_img_size:
            ext_img = F.interpolate(ext_img, size=(self.paep_img_size, self.paep_img_size),
                                    mode="bilinear", align_corners=False)
        if wrist_img.shape[-1] != self.paep_img_size or wrist_img.shape[-2] != self.paep_img_size:
            wrist_img = F.interpolate(wrist_img, size=(self.paep_img_size, self.paep_img_size),
                                      mode="bilinear", align_corners=False)

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
        g = prob[:, list(self.contact_class_ids)].sum(dim=-1)  # (B,)
        return torch.clamp(g, min=self.gmin, max=1.0)

    # ------------------------------------------------------------
    # Encode fused obs
    # ------------------------------------------------------------
    def _encode_fused_obs(self, obs_raw: Dict[str, torch.Tensor], obs_dp: Dict[str, torch.Tensor]) -> torch.Tensor:
        any_v = next(v for v in obs_dp.values() if torch.is_tensor(v))
        B = any_v.shape[0]
        To = self.n_obs_steps

        ext_raw = obs_raw["external_img"][:, :To]
        wrist_raw = obs_raw["left_wrist_img"][:, :To]
        wrench_hist_raw = obs_raw[self.wrench_hist_key][:, :To]
        pose_raw = obs_raw[self.pose_key][:, :To]

        exclude = {self.wrench_hist_key}
        enc_nobs = {k: v[:, :To] for k, v in obs_dp.items() if (k not in exclude)}

        if self.zero_proprio_in_obs_encoder:
            if self.pose_key in enc_nobs:
                enc_nobs[self.pose_key] = torch.zeros_like(enc_nobs[self.pose_key])
            if self.gripper_key in enc_nobs:
                enc_nobs[self.gripper_key] = torch.zeros_like(enc_nobs[self.gripper_key])

        flat_nobs = dict_apply(enc_nobs, lambda x: x.reshape(-1, *x.shape[2:]))
        v_feat = self.obs_encoder(flat_nobs)  # (B*To, D)

        flat_ext = ext_raw.reshape(-1, *ext_raw.shape[2:])
        flat_wrist = wrist_raw.reshape(-1, *wrist_raw.shape[2:])
        flat_wrench = wrench_hist_raw.reshape(-1, *wrench_hist_raw.shape[2:])
        flat_pose = pose_raw.reshape(-1, *pose_raw.shape[2:])

        paep_prob = self._run_paep(flat_ext, flat_wrist, flat_wrench, flat_pose)  # (B*To,6)
        g_contact = self._compute_g_contact(paep_prob)                             # (B*To,)

        with torch.no_grad():
            self._last_debug = {
                "g_contact": g_contact.detach(),
                "paep_prob": paep_prob.detach(),
            }

        v_fused = self.fusion(
            v_feat=v_feat,
            wrench_hist=flat_wrench,
            g_contact=g_contact,
            paep_prob=paep_prob,
        )  # (B*To,D)

        if (self.pose_key not in obs_dp) or (self.gripper_key not in obs_dp):
            raise KeyError(
                f"Missing proprio keys in obs_dp. Need '{self.pose_key}' and '{self.gripper_key}'. "
                f"Got keys={list(obs_dp.keys())}"
            )

        proprio = torch.cat(
            [obs_dp[self.pose_key][:, :To], obs_dp[self.gripper_key][:, :To]],
            dim=-1
        )  # (B,To,10)
        proprio_flat = proprio.reshape(-1, proprio.shape[-1])
        proprio_emb = self.proprio_mlp(proprio_flat)  # (B*To,D)

        fused = v_fused + proprio_emb
        return fused.reshape(B, To, -1)

    # ------------------------------------------------------------
    # Diffusion sampling
    # ------------------------------------------------------------
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

    # ------------------------------------------------------------
    # Inference
    # ------------------------------------------------------------
    def predict_action(self, obs_dict: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        obs_raw, obs_dp = self._split_raw_and_dp_obs(obs_dict)
        any_v = next(v for v in obs_dp.values() if torch.is_tensor(v))
        B = any_v.shape[0]

        if not self.obs_as_global_cond:
            raise NotImplementedError("Keep obs_as_global_cond=True for now (DP-consistent).")

        fused = self._encode_fused_obs(obs_raw, obs_dp)   # (B,To,D)
        global_cond = fused.reshape(B, -1)

        cond_data = torch.zeros((B, self.horizon, self.action_dim),
                                device=global_cond.device, dtype=global_cond.dtype)
        condition_mask = self.mask_generator(cond_data.shape).to(cond_data.device)

        nsample = self.conditional_sample(
            condition_data=cond_data,
            condition_mask=condition_mask,
            local_cond=None,
            global_cond=global_cond,
            **self.kwargs,
        )

        # action_pred_full = self.normalizer["action"].unnormalize(nsample)
        # action_executed = self.normalizer["action"].unnormalize(nsample[:, : self.n_action_steps])

        # return {"action": action_executed, "action_pred": action_pred_full}
        action_pred_full = self.normalizer["action"].unnormalize(nsample)
        action_executed = self.normalizer["action"].unnormalize(nsample[:, : self.n_action_steps])

        out = {"action": action_executed, "action_pred": action_pred_full}

        # ---------------- PAEP debug export ----------------
        # self._last_debug is set in _encode_fused_obs(): (B*To, ...)
        dbg = getattr(self, "_last_debug", None)
        if isinstance(dbg, dict) and ("paep_prob" in dbg) and ("g_contact" in dbg):
            To = int(self.n_obs_steps)
            paep_prob = dbg["paep_prob"]      # (B*To, 6)
            g_contact = dbg["g_contact"]      # (B*To,)

            # reshape back to (B, To, ...)
            paep_prob = paep_prob.reshape(B, To, -1)
            g_contact = g_contact.reshape(B, To)

            # take the last obs step as "current" event
            paep_prob_last = paep_prob[:, -1, :]         # (B,6)
            g_contact_last = g_contact[:, -1]            # (B,)

            paep_event_id = torch.argmax(paep_prob_last, dim=-1)     # (B,)
            paep_maxprob = torch.max(paep_prob_last, dim=-1).values  # (B,)

            # keep tensors (runner会detach->cpu->numpy)
            out.update({
                "paep_prob": paep_prob_last,        # (B,6)
                "paep_event_id": paep_event_id,     # (B,)
                "paep_maxprob": paep_maxprob,       # (B,)
                "paep_g_contact": g_contact_last,   # (B,)
            })
        # ---------------------------------------------------
        return out


    # ------------------------------------------------------------
    # Training loss + W&B log
    # ------------------------------------------------------------
    
    def compute_loss(self, batch) -> torch.Tensor:
        obs = batch["obs"]
        action = batch["action"]

        obs_raw, obs_dp = self._split_raw_and_dp_obs(obs)
        nactions = self.normalizer["action"].normalize(action)
        B = nactions.shape[0]

        if not self.obs_as_global_cond:
            raise NotImplementedError("obs_as_global_cond=False not supported for this policy.")

        fused = self._encode_fused_obs(obs_raw, obs_dp)
        global_cond = fused.reshape(B, -1)

        trajectory = nactions
        noise = torch.randn_like(trajectory)
        timesteps = torch.randint(
            0, self.noise_scheduler.config.num_train_timesteps,
            (B,), device=trajectory.device
        ).long()

        noisy_trajectory = self.noise_scheduler.add_noise(trajectory, noise, timesteps)

        condition_mask = self.mask_generator(trajectory.shape).to(trajectory.device)
        noisy_trajectory[condition_mask] = trajectory[condition_mask]

        pred = self.model(noisy_trajectory, timesteps, local_cond=None, global_cond=global_cond)

        pred_type = self.noise_scheduler.config.prediction_type
        if pred_type == "epsilon":
            target = noise
        elif pred_type == "sample":
            target = trajectory
        else:
            raise ValueError(f"Unsupported prediction type {pred_type}")

        loss_mask = ~condition_mask
        loss = F.mse_loss(pred, target, reduction="none")
        loss = loss * loss_mask.type(loss.dtype)
        loss = reduce(loss, "b ... -> b (...)", "mean").mean()

        # ---------------------------
        # ✅ extra logs (JSON only)
        #   - do NOT call wandb.* here
        #   - do NOT specify step here
        #   - workspace will merge into step_log and log with self.global_step
        # ---------------------------
        self._train_step += 1

        is_main = (not torch.distributed.is_available()) or \
                (not torch.distributed.is_initialized()) or \
                (torch.distributed.get_rank() == 0)

        if is_main and (self.log_wandb_every > 0) and (self._train_step % self.log_wandb_every == 0) and (self._last_debug is not None):
            dbg = self._last_debug
            gc = dbg["g_contact"]  # (B*To,)
            pp = dbg["paep_prob"]  # (B*To,6)

            with torch.no_grad():
                paep_entropy = (-(pp * (pp.clamp_min(1e-9)).log()).sum(dim=-1)).mean()
                paep_maxprob_mean = pp.max(dim=-1).values.mean()

                # argmax distribution (store as list[float], JSON-safe)
                argmax = pp.argmax(dim=-1)
                counts = torch.bincount(argmax, minlength=pp.shape[-1]).float()
                counts = (counts / counts.sum().clamp_min(1.0))

            log_dict = {
                # 注意：这里不要放 wandb.Table / wandb.plot
                "debug/g_contact_mean": float(gc.mean().detach().cpu()),
                "debug/g_contact_min": float(gc.min().detach().cpu()),
                "debug/g_contact_max": float(gc.max().detach().cpu()),
                "debug/paep_entropy": float(paep_entropy.detach().cpu()),
                "debug/paep_maxprob_mean": float(paep_maxprob_mean.detach().cpu()),
                "debug/paep_argmax_ratio": counts.detach().cpu().tolist(),  # JSON list
            }

            # fusion debug (JSON-safe scalars)
            fd = getattr(self.fusion, "_last_fusion_debug", None)
            if isinstance(fd, dict):
                for k in ["delta_norm_mean", "inj_norm_mean", "inj_ratio_mean", "v_norm_mean"]:
                    if k in fd:
                        log_dict[f"debug/fusion_{k}"] = float(fd[k].detach().cpu())

            # attn debug (JSON-safe scalars)
            ad = getattr(self.fusion.cross_attn, "_last_attn_debug", None)
            if isinstance(ad, dict):
                for k in ["g_head_mean", "g_head_min", "g_head_max", "attn_max_mean", "attn_entropy_mean"]:
                    if k in ad:
                        log_dict[f"debug/attn_{k}"] = float(ad[k].detach().cpu())

            # 可选：把 loss 也塞进 extra（不强制）
            log_dict["debug/loss_train"] = float(loss.detach().cpu())

            # merge point for workspace
            self._extra_step_log = log_dict

        return loss

    def forward(self, batch, **kwargs):
        return self.compute_loss(batch, **kwargs)
