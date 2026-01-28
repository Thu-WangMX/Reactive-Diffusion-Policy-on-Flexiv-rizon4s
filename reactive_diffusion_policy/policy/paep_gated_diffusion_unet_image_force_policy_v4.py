# reactive_diffusion_policy/policy/paep_gated_diffusion_unet_image_force_policy_v4.py
import torch.distributed as dist
from typing import Dict, Tuple, Optional
import importlib.util
from pathlib import Path
import inspect

from loguru import logger
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
from reactive_diffusion_policy.model.force.force_encoder import ForceTCNTokenEncoder
from reactive_diffusion_policy.model.force.dual_gated_v_f_fusion_v4 import DualGatedVisionForceFusion


# ----------------------------
# Robust PAEP import (same spirit as v3)
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


class PAEPGatedDiffusionUnetImageForcePolicy(BaseImagePolicy):
    """
    v4 (DP-aligned behavior, with your custom modules):

    - Vision: clean obs_encoder -> (B*To, Dv)
    - Force:  ForceTCNTokenEncoder -> tokens + global -> orthogonal residual fusion into vision feature
    - Proprio: DP-style direct concat (normalized tcp_pose(9)+gripper_width(1)=10D)
    - PAEP:
        * used for fusion gating (p_phase, p_contact -> g_contact)
        * optionally appended into diffusion global_condition:
            - use_paep_cond=False  -> no PAEP appended
            - use_paep_cond=True & use_phase_emb=False -> raw 4D = [p_phase(3), p_contact(1)]
            - use_paep_cond=True & use_phase_emb=True  -> embed to phase_emb_dim
    """

    def __init__(
        self,
        shape_meta: dict,
        noise_scheduler: DDPMScheduler,
        obs_encoder: nn.Module,
        horizon: int,
        n_action_steps: int,
        n_obs_steps: int,
        # diffusion
        num_inference_steps: Optional[int] = None,
        obs_as_global_cond: bool = True,
        diffusion_step_embed_dim: int = 256,
        down_dims=(256, 512, 1024),
        kernel_size: int = 5,
        n_groups: int = 8,
        cond_predict_scale: bool = True,
        # fusion
        n_heads: int = 8,
        force_hist: int = 48,
        # keys
        wrench_key: str = "left_robot_tcp_wrench",
        wrench_hist_key: str = "wrench_hist",
        pose_key: str = "left_robot_tcp_pose",
        gripper_key: str = "left_robot_gripper_width",
        # PAEP
        paep_ckpt: Optional[str] = None,
        paep_img_size: int = 224,
        gmin: float = 0.0,
        freeze_paep: bool = True,
        # ---- PAEP as diffusion condition (clean semantics) ----
        use_paep_cond: bool = True,
        use_phase_emb: bool = False,          # False -> raw 4D; True -> embedding
        phase_emb_dim: int = 16,
        phase_emb_use_contact: bool = False,
        phase_emb_use_logprob: bool = True,   # use log(p_phase) for richer signal
        # fusion head-wise gate uses contact or not (your story choice)
        head_gate_use_contact: bool = False,
        # fusion (orthogonal residual)
        fusion_alpha: float = 0.05,
        fusion_learnable_alpha: bool = False,
        fusion_alpha_max: float = 0.2,
        fusion_inj_ratio_cap: float | None = None,
        fusion_enable_debug: bool = True,
        # logging
        log_wandb_every: int = 30,
        # parameters passed to scheduler.step (DP style)
        **kwargs,
    ):
        super().__init__()

        # ---- shapes ----
        action_shape = shape_meta["action"]["shape"]
        assert len(action_shape) == 1
        action_dim = int(action_shape[0])

        vision_dim = int(obs_encoder.output_shape()[0])
        self.vision_dim = vision_dim

        # DP-style proprio: 9+1 = 10
        self.proprio_dim = 10

        self.use_paep_cond = bool(use_paep_cond)
        self.use_phase_emb = bool(use_phase_emb)
        self.phase_emb_dim = int(phase_emb_dim)
        self.phase_emb_use_contact = bool(phase_emb_use_contact)
        self.phase_emb_use_logprob = bool(phase_emb_use_logprob)

        if not self.use_paep_cond:
            paep_step_dim = 0
        else:
            paep_step_dim = self.phase_emb_dim if self.use_phase_emb else 4  # raw4 = 3phase + 1contact
        self.paep_step_dim = int(paep_step_dim)

        self.cond_step_dim = int(self.vision_dim + self.proprio_dim + self.paep_step_dim)

        # ---- diffusion model (align DP: global_cond_dim = obs_feature_dim * n_obs_steps) ----
        global_cond_dim = None
        if obs_as_global_cond:
            global_cond_dim = self.cond_step_dim * int(n_obs_steps)

        self.model = ConditionalUnet1D(
            input_dim=action_dim,
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
            obs_dim=0 if obs_as_global_cond else self.cond_step_dim,
            max_n_obs_steps=int(n_obs_steps),
            fix_obs_steps=True,
            action_visible=False,
        )

        self.normalizer = LinearNormalizer()
        self.horizon = int(horizon)
        self.action_dim = int(action_dim)
        self.obs_feature_dim = int(self.cond_step_dim)  # for DP-like semantics
        self.n_action_steps = int(n_action_steps)
        self.n_obs_steps = int(n_obs_steps)
        self.obs_as_global_cond = bool(obs_as_global_cond)
        self.kwargs = kwargs

        if num_inference_steps is None:
            num_inference_steps = noise_scheduler.config.num_train_timesteps
        self.num_inference_steps = int(num_inference_steps)

        # ---- keys ----
        self.pose_key = str(pose_key)
        self.gripper_key = str(gripper_key)
        self.wrench_key = str(wrench_key)
        self.wrench_hist_key = str(wrench_hist_key)

        # rgb keys (same idea as your v4: only feed rgb to obs_encoder)
        self.rgb_keys = sorted(
            [k for k, v in shape_meta["obs"].items() if v.get("type", "low_dim") == "rgb"]
        )

        # ---- force encoder ----
        self.force_tcn = ForceTCNTokenEncoder(
            in_dim=6,
            d_model=self.vision_dim,
            hidden=self.vision_dim,
            kernel_size=5,
            num_blocks=4,
            dropout=0.0,
        )
        self.force_hist = int(force_hist)

        # ---- fusion module (orthogonal residual) ----
        self.fusion = DualGatedVisionForceFusion(
            d_model=self.vision_dim,
            n_heads=int(n_heads),
            gate_hidden=128,
            dropout=0.0,
            use_contact_in_head_gate=bool(head_gate_use_contact),
            use_ln=True,
            ln_out=True,
            eps=1e-6,
            alpha=float(fusion_alpha),
            learnable_alpha=bool(fusion_learnable_alpha),
            alpha_max=float(fusion_alpha_max),
            inj_ratio_cap=fusion_inj_ratio_cap,
            enable_debug=bool(fusion_enable_debug),
        )

        # ---- PAEP ----
        self.paep = PAEPFutureNet(num_events=3, img_pretrained=False)
        self.paep_img_size = int(paep_img_size)
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

        # ---- PAEP embedding (optional) ----
        if self.use_paep_cond and self.use_phase_emb:
            # input dim:
            #   phase: 3 (or log-prob 3)
            #   + contact(1) if phase_emb_use_contact
            emb_in_dim = 3 + (1 if self.phase_emb_use_contact else 0)
            self.phase_emb_mlp = nn.Sequential(
                nn.Linear(emb_in_dim, 128),
                nn.ReLU(inplace=True),
                nn.Linear(128, self.phase_emb_dim),
            )
        else:
            self.phase_emb_mlp = None

        # logs / debug
        self.log_wandb_every = int(log_wandb_every)
        self._train_step = 0
        self._last_debug = None
        self._extra_step_log = None

    def _to_float(self, x):
        try:
            if torch.is_tensor(x):
                return float(x.detach().item())
            return float(x)
        except Exception:
            return None

    def _push_extra_log(self, d: dict, prefix: str = ""):
        if not hasattr(self, "_extra_step_log") or self._extra_step_log is None:
            self._extra_step_log = {}
        for k, v in (d or {}).items():
            fv = self._to_float(v)
            if fv is not None:
                self._extra_step_log[prefix + k] = fv


    # ----------------------------
    # normalizer helper (DP-compatible, plus dict support like v3)
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
            # fallback: try dp-style
            try:
                self.normalizer.load_state_dict(normalizer.state_dict())
            except Exception:
                self.normalizer = normalizer
        self.normalizer.to(next(self.parameters()).device)

    # ----------------------------
    # v3 helper: split raw & dp obs
    # ----------------------------
    def _split_raw_and_dp_obs(
        self, obs: Dict[str, torch.Tensor]
    ) -> Tuple[Dict[str, torch.Tensor], Dict[str, torch.Tensor]]:
        obs_raw: Dict[str, torch.Tensor] = {}
        obs_dp: Dict[str, torch.Tensor] = {}
        for k, v in obs.items():
            # raw: uint8 -> float/255 (for PAEP)
            if torch.is_tensor(v) and isinstance(k, str) and (k in self.rgb_keys):
                if v.dtype == torch.uint8:
                    obs_raw[k] = v.float() / 255.0
                else:
                    vv = v.float()
                    # 若值域像 0~255（max > 1.5），则缩放到 0~1
                    obs_raw[k] = vv / 255.0 if vv.detach().max() > 1.5 else vv
            else:
                obs_raw[k] = v


            # dp: use normalizer if key exists, else keep float/255 for uint8
            # dp:
            #   - rgb: 强制保持 [0,1]（因为 encoder 里还会 imagenet_norm）
            #   - 其它 low_dim: 走 normalizer（如果存在）
            if isinstance(k, str) and (k in self.rgb_keys):
                obs_dp[k] = obs_raw[k]  # [0,1]，不再做 2x-1
            elif isinstance(k, str) and self._has_norm_key(k):
                obs_dp[k] = self.normalizer[k].normalize(v)
            else:
                obs_dp[k] = v if (torch.is_tensor(v) and v.dtype != torch.uint8) else (v.float() / 255.0)

        return obs_raw, obs_dp

    # ----------------------------
    # PAEP forward
    # ----------------------------
    @torch.no_grad()
    def _run_paep(self, ext_img, wrist_img, wrench_hist, pose):
        # 1) resize to paep_img_size
        if ext_img.shape[-1] != self.paep_img_size or ext_img.shape[-2] != self.paep_img_size:
            ext_img = F.interpolate(ext_img, size=(self.paep_img_size, self.paep_img_size),
                                    mode="bilinear", align_corners=False)
        if wrist_img.shape[-1] != self.paep_img_size or wrist_img.shape[-2] != self.paep_img_size:
            wrist_img = F.interpolate(wrist_img, size=(self.paep_img_size, self.paep_img_size),
                                    mode="bilinear", align_corners=False)

        # 2) normalize wrench/pose using ckpt stats
        w = (wrench_hist - self._paep_wm[None, None, :]) / (self._paep_ws[None, None, :] + 1e-6)
        p = (pose - self._paep_pm[None, :]) / (self._paep_ps[None, :] + 1e-6)

        # 3) pass img_size only if supported
        import inspect
        sig = inspect.signature(self.paep.forward)
        if "img_size" in sig.parameters:
            out = self.paep(ext_img, wrist_img, w, p, img_size=self.paep_img_size)
        else:
            out = self.paep(ext_img, wrist_img, w, p)
            
        import torch.distributed as dist
        # def _rank0():
        #     return (not dist.is_available()) or (not dist.is_initialized()) or dist.get_rank() == 0

        # if _rank0() and (not hasattr(self, "_paep_dbg_once")):
        #     self._paep_dbg_once = True
        #     for kk, vv in out.items():
        #         if torch.is_tensor(vv):
        #             print(
        #                 f"[DBG][paep_out] {kk} "
        #                 f"shape={tuple(vv.shape)} "
        #                 f"min={vv.min().item():.4f} max={vv.max().item():.4f} mean={vv.mean().item():.4f}"
        #             )
        # ================================================

        return out["p_phase"], out["p_contact"]


    def _compute_g_contact(self, p_contact: torch.Tensor) -> torch.Tensor:
        g = p_contact.squeeze(-1)  # (B,)
        return torch.clamp(g, min=self.gmin, max=1.0)

    # ------------------------------------------------------------
    # Encode fused obs: vision+force first, then concat proprio (+ optional paep)
    # ------------------------------------------------------------
    def _encode_fused_obs(self, obs_raw: dict, obs_dp: dict) -> torch.Tensor:
        # infer B, To
        any_v = next(v for v in obs_dp.values() if torch.is_tensor(v))
        B = any_v.shape[0]
        To = self.n_obs_steps

        # ---- PAEP raw inputs (you used these keys in v4) ----
        # NOTE: ensure your dataset provides these keys; otherwise change here.
        ext_raw = obs_raw["external_img"][:, :To]         # (B,To,3,H,W)
        wrist_raw = obs_raw["left_wrist_img"][:, :To]     # (B,To,3,H,W)
        wrench_hist_raw = obs_raw[self.wrench_hist_key][:, :To]  # (B,To,L,6)
        pose_raw = obs_raw[self.pose_key][:, :To]               # (B,To,9)

        # ---- vision encoder inputs (DP-normalized / float) ----
        enc_nobs = {k: obs_dp[k][:, :To] for k in self.rgb_keys if k in obs_dp}
        flat_rgb = dict_apply(enc_nobs, lambda x: x.reshape(-1, *x.shape[2:]))  # (B*To,...)


        import torch.distributed as dist

        # def _rank0():
        #     return (not dist.is_available()) or (not dist.is_initialized()) or dist.get_rank() == 0

        # # 在 v_feat = self.obs_encoder(flat_rgb) 之前插入：
        # if _rank0():
        #     # 取一个 key 看范围即可
        #     k0 = self.rgb_keys[0] if len(self.rgb_keys) > 0 else None
        #     if k0 is not None and k0 in enc_nobs:
        #         x = enc_nobs[k0]  # (B,To,3,H,W)
        #         x0 = x[:, 0]      # (B,3,H,W)
        #         print(f"[DBG][obs_encoder_in] key={k0} min={x0.min().item():.4f} max={x0.max().item():.4f} mean={x0.mean().item():.4f} std={x0.std().item():.4f}")

        v_feat = self.obs_encoder(flat_rgb)  # (B*To,Dv)

        # flatten PAEP inputs
        flat_ext = ext_raw.reshape(-1, *ext_raw.shape[2:])
        flat_wrist = wrist_raw.reshape(-1, *wrist_raw.shape[2:])
        flat_wrench_raw = wrench_hist_raw.reshape(-1, *wrench_hist_raw.shape[2:])  # (B*To,L,6)
        flat_pose = pose_raw.reshape(-1, *pose_raw.shape[2:])                      # (B*To,9)

        # PAEP
        p_phase, p_contact = self._run_paep(flat_ext, flat_wrist, flat_wrench_raw, flat_pose)
        g_contact = self._compute_g_contact(p_contact)

        # if _rank0():
        #     pc = p_contact
        #     print(f"[DBG][paep] p_contact min={pc.min().item():.4f} max={pc.max().item():.4f} mean={pc.mean().item():.4f}")
        #     pp = p_phase
        #     print(f"[DBG][paep] p_phase mean={pp.mean().item():.4f} (should be ~1/3 if uniform)")

        # wrench_hist for TCN (normalize by wrench_key if exists)
        flat_wrench_tcn = flat_wrench_raw
        if self._has_norm_key(self.wrench_key):
            try:
                # normalize expects (..,6) — LinearNormalizer should handle
                flat_wrench_tcn = self.normalizer[self.wrench_key].normalize(flat_wrench_raw)
            except Exception:
                flat_wrench_tcn = flat_wrench_raw

        # if _rank0():
        #     raw_std = flat_wrench_raw.std().item()
        #     raw_mean = flat_wrench_raw.mean().item()
        #     tcn_std = flat_wrench_tcn.std().item()
        #     tcn_mean = flat_wrench_tcn.mean().item()
        #     has = self._has_norm_key(self.wrench_key)
        #     print(f"[DBG][wrench_hist] has_norm({self.wrench_key})={has} raw(mean,std)=({raw_mean:.4f},{raw_std:.4f})  normed(mean,std)=({tcn_mean:.4f},{tcn_std:.4f})")


        force_tokens, force_global = self.force_tcn(flat_wrench_tcn)  # (B*To,L,Dv), (B*To,Dv)

        # fusion (orthogonal residual)
        v_fused = self.fusion(
            v_feat=v_feat,
            force_tokens=force_tokens,
            force_global=force_global,
            g_contact=g_contact,
            p_phase=p_phase,
            p_contact=p_contact,
        )  # (B*To,Dv)

        # ---- (A) PAEP stats ----
        with torch.no_grad():
            # p_contact: (B*To,1), p_phase:(B*To,3), g_contact:(B*To,)
            pc = p_contact
            pp = p_phase
            # entropy of phase prob
            phase_ent = (-(pp.clamp_min(1e-9).log() * pp).sum(dim=-1)).mean()

            self._push_extra_log({
                "paep/p_contact_mean": pc.mean(),
                "paep/p_contact_min": pc.min(),
                "paep/p_contact_max": pc.max(),
                "paep/phase_entropy": phase_ent,
                "paep/g_contact_mean": g_contact.mean(),
                "paep/g_contact_min": g_contact.min(),
                "paep/g_contact_max": g_contact.max(),
            })

        # ---- (B) attention stats (always available) ----
        attn_dbg = getattr(self.fusion.cross_attn, "_last_attn_debug", None)
        if isinstance(attn_dbg, dict):
            self._push_extra_log(attn_dbg, prefix="attn/")

        # ---- (C) fusion stats (need enable_debug=True) ----
        fus_dbg = getattr(self.fusion, "_last_fusion_debug", None)
        if isinstance(fus_dbg, dict):
            self._push_extra_log(fus_dbg, prefix="fusion/")

        # ---- (D) optional: force token norms ----
        with torch.no_grad():
            self._push_extra_log({
                "force/tokens_norm_mean": force_tokens.norm(dim=-1).mean(),
                "force/global_norm_mean": force_global.norm(dim=-1).mean(),
            })


        # proprio (DP-style direct concat)
        if (self.pose_key not in obs_dp) or (self.gripper_key not in obs_dp):
            raise KeyError(
                f"Missing proprio keys in obs_dp. Need '{self.pose_key}' and '{self.gripper_key}'. "
                f"Got keys={list(obs_dp.keys())}"
            )
        proprio = torch.cat(
            [obs_dp[self.pose_key][:, :To], obs_dp[self.gripper_key][:, :To]],
            dim=-1
        )  # (B,To,10)
        proprio_flat = proprio.reshape(-1, proprio.shape[-1])  # (B*To,10)

        # optional PAEP cond
        cond_parts = [v_fused, proprio_flat]
        if self.use_paep_cond:
            if self.use_phase_emb:
                # emb input: phase probs (or logprobs) + optional contact
                phase_in = p_phase
                if self.phase_emb_use_logprob:
                    phase_in = torch.log(torch.clamp(phase_in, min=1e-6))
                if self.phase_emb_use_contact:
                    emb_in = torch.cat([phase_in, p_contact], dim=-1)
                else:
                    emb_in = phase_in
                paep_vec = self.phase_emb_mlp(emb_in.detach())
            else:
                # raw 4D: [p_phase(3), p_contact(1)]
                paep_vec = torch.cat([p_phase, p_contact], dim=-1).detach()
            cond_parts.append(paep_vec)

        fused_step = torch.cat(cond_parts, dim=-1)         # (B*To, cond_step_dim)
        fused_step = fused_step.reshape(B, To, -1)         # (B,To,cond_step_dim)

        # debug stash
        with torch.no_grad():
            self._last_debug = {
                "p_phase": p_phase.detach(),
                "p_contact": p_contact.detach(),
                "g_contact": g_contact.detach(),
            }

        return fused_step

    # ------------------------------------------------------------
    # Diffusion sampling (DP-style, plus safe step-kwargs filtering)
    # ------------------------------------------------------------
    def conditional_sample(
        self,
        condition_data,
        condition_mask,
        local_cond=None,
        global_cond=None,
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

        # be robust if someone passes extra kwargs
        step_sig = inspect.signature(scheduler.step)
        step_kwargs = {k: v for k, v in kwargs.items() if k in step_sig.parameters}

        for t in scheduler.timesteps:
            trajectory[condition_mask] = condition_data[condition_mask]
            model_output = model(trajectory, t, local_cond=local_cond, global_cond=global_cond)
            trajectory = scheduler.step(
                model_output, t, trajectory, generator=generator, **step_kwargs
            ).prev_sample

        trajectory[condition_mask] = condition_data[condition_mask]
        return trajectory

    # ------------------------------------------------------------
    # Policy API (DP-aligned behavior)
    # ------------------------------------------------------------
    def predict_action(self, obs_dict: Dict[str, torch.Tensor], generator=None) -> Dict[str, torch.Tensor]:
        """
        DP-aligned:
          - output action is unnormalized
          - action slice: start = To-1, take n_action_steps
        """
        assert "past_action" not in obs_dict, "past_action not implemented"

        # split raw & dp obs
        obs_raw, obs_dp = self._split_raw_and_dp_obs(obs_dict)

        value = next(iter(obs_dp.values()))
        B = value.shape[0]
        T = self.horizon
        Da = self.action_dim
        To = self.n_obs_steps

        device = self.device
        dtype = self.dtype

        # global condition
        local_cond = None
        global_cond = None
        if self.obs_as_global_cond:
            fused_step = self._encode_fused_obs(obs_raw=obs_raw, obs_dp=obs_dp)  # (B,To,cond_step_dim)
            global_cond = fused_step.reshape(B, -1)
            cond_data = torch.zeros(size=(B, T, Da), device=device, dtype=dtype)
            cond_mask = torch.zeros_like(cond_data, dtype=torch.bool)
        else:
            raise NotImplementedError("obs_as_global_cond=False is not supported in this v4 variant")

        nsample = self.conditional_sample(
            cond_data,
            cond_mask,
            local_cond=local_cond,
            global_cond=global_cond,
            generator=generator,
            **self.kwargs,
        )

        # unnormalize prediction
        naction_pred = nsample[..., :Da]
        action_pred = self.normalizer["action"].unnormalize(naction_pred)

        # get action (DP rule)
        start = To - 1
        end = start + self.n_action_steps
        action = action_pred[:, start:end]

        return {
            "action": action,
            "action_pred": action_pred,
        }

    def compute_loss(self, batch: dict) -> torch.Tensor:

        """
        DP-aligned loss:
          - uses mask_generator for inpainting mask
          - supports prediction_type: epsilon / sample
          - masked mse loss on ~condition_mask
        """
        assert "valid_mask" not in batch

        obs = batch["obs"]
        obs_raw, obs_dp = self._split_raw_and_dp_obs(obs)

        nactions = self.normalizer["action"].normalize(batch["action"])
        
        # def _rank0():
        #     return (not dist.is_available()) or (not dist.is_initialized()) or dist.get_rank() == 0
        # if _rank0():
        #     a_raw = batch["action"]
        #     a_n = nactions
        #     print(f"[DBG][action_norm] raw(mean,std)=({a_raw.mean().item():.4f},{a_raw.std().item():.4f})  normed(mean,std)=({a_n.mean().item():.4f},{a_n.std().item():.4f})")

        batch_size = nactions.shape[0]
        horizon = nactions.shape[1]

        local_cond = None
        global_cond = None

        trajectory = nactions
        cond_data = trajectory

        if self.obs_as_global_cond:
            fused_step = self._encode_fused_obs(obs_raw=obs_raw, obs_dp=obs_dp)  # (B,To,cond_step_dim)
            global_cond = fused_step.reshape(batch_size, -1)
        else:
            raise NotImplementedError("obs_as_global_cond=False is not supported in this v4 variant")

        # generate inpainting mask (DP style)
        condition_mask = self.mask_generator(trajectory.shape)

        # noise & timestep
        noise = torch.randn(trajectory.shape, device=trajectory.device)
        bsz = trajectory.shape[0]
        timesteps = torch.randint(
            0,
            self.noise_scheduler.config.num_train_timesteps,
            (bsz,),
            device=trajectory.device,
        ).long()

        noisy_trajectory = self.noise_scheduler.add_noise(trajectory, noise, timesteps)

        # loss mask
        loss_mask = ~condition_mask

        # apply conditioning
        noisy_trajectory[condition_mask] = cond_data[condition_mask]

        # predict
        pred = self.model(
            noisy_trajectory,
            timesteps,
            local_cond=local_cond,
            global_cond=global_cond,
        )

        pred_type = self.noise_scheduler.config.prediction_type
        if pred_type == "epsilon":
            target = noise
        elif pred_type == "sample":
            target = trajectory
        else:
            raise ValueError(f"Unsupported prediction type {pred_type}")

        loss = F.mse_loss(pred, target, reduction="none")
        loss = loss * loss_mask.type(loss.dtype)
        loss = reduce(loss, "b ... -> b (...) ", "mean")
        loss = loss.mean()

       
        # optional debug log stash (won't affect DP training loop unless you use it)
        self._train_step += 1
        if isinstance(self._last_debug, dict):
            dbg = self._last_debug
            try:
                # 使用 _push_extra_log 追加数据，而不是覆盖整个字典
                self._push_extra_log({
                    "debug/g_contact_mean": dbg["g_contact"].mean(),
                    "debug/g_contact_min": dbg["g_contact"].min(),
                    "debug/g_contact_max": dbg["g_contact"].max(),
                    "debug/p_contact_mean": dbg["p_contact"].mean(),
                    "debug/p_contact_min": dbg["p_contact"].min(),
                    "debug/p_contact_max": dbg["p_contact"].max(),
                })
            except Exception:
                pass

        return loss

    def forward(self, batch):
        return self.compute_loss(batch)

