# reactive_diffusion_policy/policy/paep_gated_diffusion_unet_image_force_policy.py
from typing import Dict, Union, Optional, Sequence, Tuple
import math
import importlib.util
from pathlib import Path

import torch
import torch.nn as nn
import torch.nn.functional as F
from einops import reduce
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler

from reactive_diffusion_policy.model.common.normalizer import LinearNormalizer
from reactive_diffusion_policy.policy.base_image_policy import BaseImagePolicy
from reactive_diffusion_policy.model.diffusion.conditional_unet1d import ConditionalUnet1D
from reactive_diffusion_policy.model.diffusion.mask_generator import LowdimMaskGenerator
from reactive_diffusion_policy.common.pytorch_util import dict_apply


# ----------------------------
# Robust PAEP import
# ----------------------------
def _load_paep_future_net():
    repo_root = Path(__file__).resolve().parents[2]
    paep_dir = repo_root / "PAEP"
    candidates = [
        paep_dir / "paep_train" / "paep_model.py",
        paep_dir / "paep_model.py",
        paep_dir / "models" / "paep_model.py",
    ]
    if paep_dir.exists():
        for p in paep_dir.rglob("paep_model.py"):
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
                return getattr(mod, "PAEPFutureNet")
        except Exception:
            continue

    raise ModuleNotFoundError("Cannot find PAEPFutureNet under repo/PAEP/**/paep_model.py")


PAEPFutureNet = _load_paep_future_net()


def imagenet_normalize(x: torch.Tensor) -> torch.Tensor:
    mean = torch.tensor([0.485, 0.456, 0.406], device=x.device, dtype=x.dtype).view(-1, 1, 1)
    std = torch.tensor([0.229, 0.224, 0.225], device=x.device, dtype=x.dtype).view(-1, 1, 1)
    if x.dim() == 3:
        return (x - mean) / std
    if x.dim() == 4:
        return (x - mean.unsqueeze(0)) / std.unsqueeze(0)
    raise ValueError(f"Unexpected x.dim={x.dim()}")


# ============================================================
# Force TCN token encoder: (B,L,6) -> (B,L,D)
#   + returns force_global = mean(L)
# ============================================================
class ForceTCNTokenEncoder(nn.Module):
    def __init__(
        self,
        in_dim: int = 6,
        d_model: int = 256,
        hidden: int = 256,
        kernel_size: int = 5,
        num_blocks: int = 4,
        dropout: float = 0.0,
    ):
        super().__init__()
        self.in_proj = nn.Conv1d(in_dim, hidden, kernel_size=1)
        blocks = []
        for i in range(num_blocks):
            dilation = 2 ** i
            pad = (kernel_size - 1) * dilation
            blocks.append(nn.Sequential(
                nn.Conv1d(hidden, hidden, kernel_size, padding=pad, dilation=dilation),
                nn.ReLU(inplace=True),
                nn.Dropout(dropout),
                nn.Conv1d(hidden, hidden, kernel_size=1),
                nn.ReLU(inplace=True),
            ))
        self.blocks = nn.ModuleList(blocks)
        self.out_proj = nn.Linear(hidden, d_model)
        self.res_scale = 0.1

    def forward(self, x: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        # x: (B,L,6)
        x = x.transpose(1, 2)      # (B,6,L)
        h = self.in_proj(x)        # (B,H,L)
        for blk in self.blocks:
            y = blk(h)
            if y.shape[-1] != h.shape[-1]:
                y = y[..., -h.shape[-1]:]
            h = h + self.res_scale * y
        h = h.transpose(1, 2)      # (B,L,H)
        tokens = self.out_proj(h)  # (B,L,D)
        force_global = tokens.mean(dim=1)  # (B,D)
        return tokens, force_global


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
        out = out * gate_h[:, :, None, None]

        out = out.transpose(1, 2).contiguous().view(B, Nq, D)
        y = self.out_proj(out)

        with torch.no_grad():
            self._last_attn_debug = {
                "g_head_mean": gate_h.mean().detach(),
                "g_head_min": gate_h.min().detach(),
                "g_head_max": gate_h.max().detach(),
                "attn_max_mean": w.max(dim=-1).values.mean().detach(),
                "attn_entropy_mean": (-(w.clamp_min(1e-9).log() * w).sum(dim=-1)).mean().detach(),
            }

        return y


# ============================================================
# Dual-gated fusion (NEW PAEP gates) + low-cost force_global usage
#   - contact gate scales whole residual: g_contact = p_contact
#   - phase (and contact) -> head gate
#   - force_global -> mild amplitude modulation (scalar ~1.0)
# ============================================================
class DualGatedVisionForceFusion(nn.Module):
    def __init__(
        self,
        d_model: int,
        n_heads: int = 8,
        gate_hidden: int = 128,
        dropout: float = 0.0,
        use_contact_in_head_gate: bool = True,
    ):
        super().__init__()
        self.d_model = d_model
        self.n_heads = n_heads
        self.use_contact_in_head_gate = bool(use_contact_in_head_gate)

        self.cross_attn = HeadWiseCrossAttention(d_model=d_model, n_heads=n_heads, dropout=dropout)

        head_gate_in = 4 if self.use_contact_in_head_gate else 3  # [p_phase(3), p_contact(1)]
        self.head_gate_mlp = nn.Sequential(
            nn.Linear(head_gate_in, gate_hidden),
            nn.ReLU(inplace=True),
            nn.Linear(gate_hidden, n_heads),
        )

        self.amp_mlp = nn.Sequential(
            nn.Linear(d_model, gate_hidden),
            nn.ReLU(inplace=True),
            nn.Linear(gate_hidden, 1),
        )

        self._last_fusion_debug = None

    def forward(
        self,
        v_feat: torch.Tensor,          # (B,D)
        force_tokens: torch.Tensor,    # (B,L,D)
        force_global: torch.Tensor,    # (B,D)
        g_contact: torch.Tensor,       # (B,) in [0,1]
        p_phase: torch.Tensor,         # (B,3)
        p_contact: torch.Tensor,       # (B,1)
    ) -> torch.Tensor:
        v_tok = v_feat.unsqueeze(1)  # (B,1,D)

        if self.use_contact_in_head_gate:
            hg_in = torch.cat([p_phase, p_contact], dim=-1)  # (B,4)
        else:
            hg_in = p_phase  # (B,3)

        g_head = torch.sigmoid(self.head_gate_mlp(hg_in))  # (B,H)

        delta = self.cross_attn(v_tok, force_tokens, g_head)  # (B,1,D)

        amp = 1.0 + 0.1 * torch.tanh(self.amp_mlp(force_global)).squeeze(-1)  # (B,)
        inj = (g_contact * amp).clamp(0.0, 1.5)

        v_fused = v_tok + inj[:, None, None] * delta

        with torch.no_grad():
            dn = delta.squeeze(1).norm(dim=-1)
            vn = v_feat.norm(dim=-1)
            self._last_fusion_debug = {
                "delta_norm_mean": dn.mean().detach(),
                "v_norm_mean": vn.mean().detach(),
                "inj_mean": inj.mean().detach(),
                "inj_ratio_mean": ((inj * dn) / (vn + 1e-6)).mean().detach(),
                "amp_mean": amp.mean().detach(),
            }

        return v_fused.squeeze(1)


# ============================================================
# Policy
# ============================================================
class PAEPGatedDiffusionUnetImageForcePolicy(BaseImagePolicy):
    """
    - Vision: clean obs_encoder -> (B*To, Dv)
    - Force:  ForceTCNTokenEncoder -> tokens + global -> fuse with vision
    - Proprio: MLP (tcp_pose+gripper_width=10D) -> (B*To, Dp)
    - PAEP: NEW (p_contact, p_phase) used for dual-gated fusion
    - Optional: PAEP embedding concatenated into global_cond
    """
    def __init__(
        self,
        shape_meta: dict,
        noise_scheduler: DDPMScheduler,
        obs_encoder: nn.Module,  # should be "clean" vision-only encoder
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
        wrench_key: str = "left_robot_tcp_wrench",
        wrench_hist_key: str = "wrench_hist",
        pose_key: str = "left_robot_tcp_pose",
        gripper_key: str = "left_robot_gripper_width",
        # proprio
        proprio_embed_dim: Optional[int] = None,      # default = vision_dim
        # PAEP
        paep_ckpt: Optional[str] = None,
        paep_img_size: int = 224,
        gmin: float = 0.05,
        freeze_paep: bool = True,
        # NEW: whether concatenate PAEP embedding into global_cond
        use_paep_cond: bool = True,
        paep_cond_dim: int = 32,                     # embedding dim appended per step
        paep_cond_use_contact: bool = False,          # embedding uses [p_phase, p_contact] or only p_phase
        head_gate_use_contact: bool = True,
        # logging
        log_wandb_every: int = 30,
        **kwargs,
    ):
        super().__init__()
        action_shape = shape_meta["action"]["shape"]
        assert len(action_shape) == 1
        action_dim = action_shape[0]

        # obs_encoder output must be pure vision
        vision_dim = int(obs_encoder.output_shape()[0])
        self.vision_dim = vision_dim

        if proprio_embed_dim is None:
            proprio_embed_dim = vision_dim
        self.proprio_embed_dim = int(proprio_embed_dim)

        self.use_paep_cond = bool(use_paep_cond)
        self.paep_cond_dim = int(paep_cond_dim) if self.use_paep_cond else 0
        self.paep_cond_use_contact = bool(paep_cond_use_contact)

        cond_step_dim = self.vision_dim + self.proprio_embed_dim + self.paep_cond_dim
        self.cond_step_dim = int(cond_step_dim)

        input_dim = int(action_dim)
        global_cond_dim = self.cond_step_dim * int(n_obs_steps) if obs_as_global_cond else None

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
            action_dim=input_dim,
            obs_dim=0,
            max_n_obs_steps=int(n_obs_steps),
            fix_obs_steps=True,
            action_visible=False,
        )

        self.normalizer = LinearNormalizer()
        self.horizon = int(horizon)
        self.action_dim = int(action_dim)
        self.n_action_steps = int(n_action_steps)
        self.n_obs_steps = int(n_obs_steps)
        self.obs_as_global_cond = bool(obs_as_global_cond)
        self.kwargs = kwargs

        if num_inference_steps is None:
            num_inference_steps = noise_scheduler.config.num_train_timesteps
        self.num_inference_steps = int(num_inference_steps)

        # keys
        self.shape_meta = shape_meta
        self.wrench_key = wrench_key
        self.wrench_hist_key = wrench_hist_key
        self.pose_key = pose_key
        self.gripper_key = gripper_key

        self.rgb_keys = sorted([k for k, v in shape_meta["obs"].items() if v.get("type", "low_dim") == "rgb"])

        # proprio: tcp_pose(9)+gripper_width(1)=10
        self.proprio_dim = 10
        self.proprio_mlp = nn.Sequential(
            nn.Linear(self.proprio_dim, self.proprio_embed_dim),
            nn.ReLU(inplace=True),
            nn.Linear(self.proprio_embed_dim, self.proprio_embed_dim),
        )

        # force encoder
        self.force_tcn = ForceTCNTokenEncoder(
            in_dim=6,
            d_model=self.vision_dim,
            hidden=self.vision_dim,
            kernel_size=5,
            num_blocks=4,
            dropout=0.0,
        )
        self.force_hist = int(force_hist)

        # fusion (dual gate)
        self.fusion = DualGatedVisionForceFusion(
            d_model=self.vision_dim,
            n_heads=int(n_heads),
            gate_hidden=128,
            dropout=0.0,
            use_contact_in_head_gate=bool(head_gate_use_contact),
        )

        # PAEP
        self.paep = PAEPFutureNet(num_events=3, img_pretrained=False)
        self.paep_img_size = int(paep_img_size)
        self.gmin = float(gmin)

        # PAEP norm buffers (for PAEP internal normalization)
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

        # PAEP embedding (optional)
        if self.use_paep_cond:
            in_dim = 4 if self.paep_cond_use_contact else 3
            self.paep_cond_mlp = nn.Sequential(
                nn.Linear(in_dim, 64),
                nn.ReLU(inplace=True),
                nn.Linear(64, self.paep_cond_dim),
            )

        # logging
        self.log_wandb_every = int(log_wandb_every)
        self._train_step = 0
        self._last_debug = None
        self._extra_step_log = None

    # ----------------------------
    # normalizer helper
    # ----------------------------
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

    def _split_raw_and_dp_obs(
        self, obs: Dict[str, torch.Tensor]
    ) -> Tuple[Dict[str, torch.Tensor], Dict[str, torch.Tensor]]:
        obs_raw: Dict[str, torch.Tensor] = {}
        obs_dp: Dict[str, torch.Tensor] = {}
        for k, v in obs.items():
            if torch.is_tensor(v) and v.dtype == torch.uint8:
                obs_raw[k] = v.float() / 255.0
            else:
                obs_raw[k] = v

            if isinstance(k, str) and self._has_norm_key(k):
                obs_dp[k] = self.normalizer[k].normalize(v)
            else:
                obs_dp[k] = v if v.dtype != torch.uint8 else (v.float() / 255.0)
        return obs_raw, obs_dp

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
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        if ext_img.shape[-1] != self.paep_img_size or ext_img.shape[-2] != self.paep_img_size:
            ext_img = F.interpolate(ext_img, size=(self.paep_img_size, self.paep_img_size),
                                    mode="bilinear", align_corners=False)
        if wrist_img.shape[-1] != self.paep_img_size or wrist_img.shape[-2] != self.paep_img_size:
            wrist_img = F.interpolate(wrist_img, size=(self.paep_img_size, self.paep_img_size),
                                      mode="bilinear", align_corners=False)

        ext_n = imagenet_normalize(ext_img)
        wrist_n = imagenet_normalize(wrist_img)

        out = self.paep(
            ext_n, wrist_n, wrench_hist, pose,
            norm=self._paep_norm_dict(device=ext_img.device, dtype=ext_img.dtype),
            img_size=self.paep_img_size,
        )
        # out: {"p_phase":(B,3), "p_contact":(B,1)}
        p_phase = out["p_phase"]
        p_contact = out["p_contact"]
        return p_phase, p_contact

    def _compute_g_contact(self, p_contact: torch.Tensor) -> torch.Tensor:
        g = p_contact.squeeze(-1)  # (B,)
        return torch.clamp(g, min=self.gmin, max=1.0)

    # ------------------------------------------------------------
    # Encode fused obs: vision+force first, then concat proprio (+ optional paep_emb)
    # ------------------------------------------------------------
    def _encode_fused_obs(self, obs_raw: Dict[str, torch.Tensor], obs_dp: Dict[str, torch.Tensor]) -> torch.Tensor:
        any_v = next(v for v in obs_dp.values() if torch.is_tensor(v))
        B = any_v.shape[0]
        To = self.n_obs_steps

        # PAEP inputs (RAW)
        ext_raw = obs_raw["external_img"][:, :To]      # (B,To,3,H,W)
        wrist_raw = obs_raw["left_wrist_img"][:, :To]  # (B,To,3,H,W)
        wrench_hist_raw = obs_raw[self.wrench_hist_key][:, :To]  # (B,To,L,6)
        pose_raw = obs_raw[self.pose_key][:, :To]                # (B,To,9)

        # vision encoder inputs: ONLY rgb keys (DP-normalized / float)
        enc_nobs = {k: obs_dp[k][:, :To] for k in self.rgb_keys if k in obs_dp}
        flat_rgb = dict_apply(enc_nobs, lambda x: x.reshape(-1, *x.shape[2:]))  # (B*To,...)
        v_feat = self.obs_encoder(flat_rgb)  # (B*To,Dv)

        # flatten PAEP inputs
        flat_ext = ext_raw.reshape(-1, *ext_raw.shape[2:])
        flat_wrist = wrist_raw.reshape(-1, *wrist_raw.shape[2:])
        flat_wrench_raw = wrench_hist_raw.reshape(-1, *wrench_hist_raw.shape[2:])  # (B*To,L,6)
        flat_pose = pose_raw.reshape(-1, *pose_raw.shape[2:])                      # (B*To,9)

        p_phase, p_contact = self._run_paep(flat_ext, flat_wrist, flat_wrench_raw, flat_pose)
        g_contact = self._compute_g_contact(p_contact)

        # wrench_hist for TCN (prefer DP-normalized by wrench_key stats if available)
        flat_wrench_tcn = flat_wrench_raw
        if self._has_norm_key(self.wrench_key):
            try:
                flat_wrench_tcn = self.normalizer[self.wrench_key].normalize(flat_wrench_raw)
            except Exception:
                flat_wrench_tcn = flat_wrench_raw

        force_tokens, force_global = self.force_tcn(flat_wrench_tcn)  # (B*To,L,Dv), (B*To,Dv)

        # fusion
        v_fused = self.fusion(
            v_feat=v_feat,
            force_tokens=force_tokens,
            force_global=force_global,
            g_contact=g_contact,
            p_phase=p_phase,
            p_contact=p_contact,
        )  # (B*To,Dv)

        # proprio
        if (self.pose_key not in obs_dp) or (self.gripper_key not in obs_dp):
            raise KeyError(
                f"Missing proprio keys in obs_dp. Need '{self.pose_key}' and '{self.gripper_key}'. "
                f"Got keys={list(obs_dp.keys())}"
            )
        proprio = torch.cat([obs_dp[self.pose_key][:, :To], obs_dp[self.gripper_key][:, :To]], dim=-1)  # (B,To,10)
        proprio_flat = proprio.reshape(-1, proprio.shape[-1])
        proprio_emb = self.proprio_mlp(proprio_flat)  # (B*To,Dp)

        # optional PAEP embedding appended to cond (detach to prevent any accidental coupling)
        cond_parts = [v_fused, proprio_emb]
        if self.use_paep_cond:
            if self.paep_cond_use_contact:
                pe_in = torch.cat([p_phase, p_contact], dim=-1)
            else:
                pe_in = p_phase
            paep_emb = self.paep_cond_mlp(pe_in.detach())
            cond_parts.append(paep_emb)

        fused_step = torch.cat(cond_parts, dim=-1)  # (B*To, cond_step_dim)
        fused_step = fused_step.reshape(B, To, -1)  # (B,To,cond_step_dim)

        # debug stash (for logging / rollout export)
        with torch.no_grad():
            self._last_debug = {
                "p_phase": p_phase.detach(),
                "p_contact": p_contact.detach(),
                "g_contact": g_contact.detach(),
            }

        return fused_step

    # ------------------------------------------------------------
    # Diffusion sampling
    # ------------------------------------------------------------
    def conditional_sample(self, condition_data, condition_mask, local_cond=None, global_cond=None, generator=None, **kwargs):
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
            raise NotImplementedError("Keep obs_as_global_cond=True for now.")

        fused = self._encode_fused_obs(obs_raw, obs_dp)  # (B,To,cond_step_dim)
        global_cond = fused.reshape(B, -1)

        cond_data = torch.zeros((B, self.horizon, self.action_dim), device=global_cond.device, dtype=global_cond.dtype)
        condition_mask = self.mask_generator(cond_data.shape).to(cond_data.device)

        nsample = self.conditional_sample(
            condition_data=cond_data,
            condition_mask=condition_mask,
            local_cond=None,
            global_cond=global_cond,
            **self.kwargs,
        )

        action_pred_full = self.normalizer["action"].unnormalize(nsample)
        action_executed = self.normalizer["action"].unnormalize(nsample[:, : self.n_action_steps])

        out = {"action": action_executed, "action_pred": action_pred_full}

        # export PAEP debug for last obs step
        dbg = getattr(self, "_last_debug", None)
        if isinstance(dbg, dict) and ("p_phase" in dbg):
            To = int(self.n_obs_steps)
            p_phase = dbg["p_phase"].reshape(B, To, -1)[:, -1, :]      # (B,3)
            p_contact = dbg["p_contact"].reshape(B, To, 1)[:, -1, :]   # (B,1)
            g_contact = dbg["g_contact"].reshape(B, To)[:, -1]         # (B,)

            out.update({
                "paep_p_phase": p_phase,
                "paep_p_contact": p_contact,
                "paep_g_contact": g_contact,
                "paep_phase_id": torch.argmax(p_phase, dim=-1),
                "paep_phase_conf": torch.max(p_phase, dim=-1).values,
            })

        return out

    # ------------------------------------------------------------
    # Training loss
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

        # optional debug logs (workspace can read self._extra_step_log)
        self._train_step += 1
        is_main = (not torch.distributed.is_available()) or \
                  (not torch.distributed.is_initialized()) or \
                  (torch.distributed.get_rank() == 0)

        if is_main and (self.log_wandb_every > 0) and (self._train_step % self.log_wandb_every == 0) and (self._last_debug is not None):
            dbg = self._last_debug
            pc = dbg["p_contact"]
            pp = dbg["p_phase"]
            gc = dbg["g_contact"]

            with torch.no_grad():
                phase_entropy = (-(pp * (pp.clamp_min(1e-9)).log()).sum(dim=-1)).mean()
                phase_conf = pp.max(dim=-1).values.mean()
                contact_mean = pc.mean()
                contact_min = pc.min()
                contact_max = pc.max()
                g_mean = gc.mean()

            log_dict = {
                "debug/loss_train": float(loss.detach().cpu()),
                "debug/paep_contact_mean": float(contact_mean.detach().cpu()),
                "debug/paep_contact_min": float(contact_min.detach().cpu()),
                "debug/paep_contact_max": float(contact_max.detach().cpu()),
                "debug/paep_phase_entropy": float(phase_entropy.detach().cpu()),
                "debug/paep_phase_conf": float(phase_conf.detach().cpu()),
                "debug/g_contact_mean": float(g_mean.detach().cpu()),
            }

            fd = getattr(self.fusion, "_last_fusion_debug", None)
            if isinstance(fd, dict):
                for k in ["delta_norm_mean", "inj_mean", "inj_ratio_mean", "v_norm_mean", "amp_mean"]:
                    if k in fd:
                        log_dict[f"debug/fusion_{k}"] = float(fd[k].detach().cpu())

            ad = getattr(self.fusion.cross_attn, "_last_attn_debug", None)
            if isinstance(ad, dict):
                for k in ["g_head_mean", "g_head_min", "g_head_max", "attn_max_mean", "attn_entropy_mean"]:
                    if k in ad:
                        log_dict[f"debug/attn_{k}"] = float(ad[k].detach().cpu())

            self._extra_step_log = log_dict

        return loss

    def forward(self, batch, **kwargs):
        return self.compute_loss(batch, **kwargs)
