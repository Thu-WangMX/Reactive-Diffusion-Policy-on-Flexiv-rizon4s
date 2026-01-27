# 核心PyTorch基础导入
import torch
import torch.nn as nn
import math  # 若HeadWiseCrossAttention在同一文件，需保留（其内部用到math.sqrt）
from typing import Optional, Tuple  # 可选：扩展类型注解用，非必需但推荐

# 可选优化：导入Tensor类型让注解更简洁（推荐）
from torch import Tensor
from reactive_diffusion_policy.model.attention.headwise_cro_attention import HeadWiseCrossAttention 


class DualGatedVisionForceFusion(nn.Module):
    """
    Dual-gated fusion = (1) head-wise gate inside cross-attn + (2) contact gate on injection.
    Now includes a *norm-ratio injection limiter*:
      - We interpret inj as "target relative strength" (ratio of ||injected|| to ||vision||)
      - Hard cap r: realized injected norm ratio <= r (per-sample)
    """

    def __init__(
        self,
        d_model: int,
        n_heads: int = 8,
        gate_hidden: int = 128,
        dropout: float = 0.0,
        use_contact_in_head_gate: bool = False,
        # LN switches
        use_ln: bool = True,
        ln_out: bool = True,
        # NEW: norm-ratio injection cap
        inj_ratio_cap: float = 0.5,          # r: injected norm <= r * vision norm
        inj_ratio_floor: float = 0.0,        # optional: floor (usually 0)
        eps: float = 1e-6,
        # Debug
        enable_debug: bool = True,
    ):
        super().__init__()
        self.d_model = int(d_model)
        self.n_heads = int(n_heads)
        self.use_contact_in_head_gate = bool(use_contact_in_head_gate)
        self.use_ln = bool(use_ln)
        self.use_ln_out = bool(ln_out)

        self.inj_ratio_cap = float(inj_ratio_cap)
        self.inj_ratio_floor = float(inj_ratio_floor)
        self.eps = float(eps)
        self.enable_debug = bool(enable_debug)

        self.cross_attn = HeadWiseCrossAttention(d_model=d_model, n_heads=n_heads, dropout=dropout)

        head_gate_in = 4 if self.use_contact_in_head_gate else 3
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

        self._dbg_step = 0
        self._dbg_every = 30 
  
        # ---- LayerNorms ----
        if self.use_ln:
            self.ln_v = nn.LayerNorm(d_model)
            self.ln_force = nn.LayerNorm(d_model)
            self.ln_force_global = nn.LayerNorm(d_model)
            self.ln_delta = nn.LayerNorm(d_model)
            self.ln_out = nn.LayerNorm(d_model) if self.use_ln_out else nn.Identity()
        else:
            self.ln_v = nn.Identity()
            self.ln_force = nn.Identity()
            self.ln_force_global = nn.Identity()
            self.ln_delta = nn.Identity()
            self.ln_out = nn.Identity()

        self._last_fusion_debug = None

        # ---- debug file state (lazy init) ----
        self._dbg_log_path = None

    def forward(
        self,
        v_feat: torch.Tensor,          # (B,D)
        force_tokens: torch.Tensor,    # (B,L,D)
        force_global: torch.Tensor,    # (B,D)
        g_contact: torch.Tensor,       # (B,)
        p_phase: torch.Tensor,         # (B,3)
        p_contact: torch.Tensor,       # (B,1)
    ) -> torch.Tensor:
        """
        Returns: (B,D)
        """
        B = v_feat.shape[0]
        eps = self.eps

        # ---- vision token ----
        v_tok = v_feat.unsqueeze(1)  # (B,1,D)

        # ---- pre-norm inputs to cross-attn ----
        v_tok_n = self.ln_v(v_tok)                      # (B,1,D)
        force_tokens_n = self.ln_force(force_tokens)    # (B,L,D)

        # ---- head-wise gate (gate-2) ----
        if self.use_contact_in_head_gate:
            hg_in = torch.cat([p_phase, p_contact], dim=-1)  # (B,4)
        else:
            hg_in = p_phase                                  # (B,3)
        g_head = torch.sigmoid(self.head_gate_mlp(hg_in))    # (B,H)

        # ---- cross-attn delta ----
        delta = self.cross_attn(v_tok_n, force_tokens_n, g_head)  # (B,1,D)
        delta = self.ln_delta(delta)                               # stabilize direction/scale in feature space

        # ---- contact gate + amp (gate-1) ----
        # NOTE: we keep a small amp range; you can widen later if needed.
        force_global_n = self.ln_force_global(force_global)
        amp = 1.0 + 0.1 * torch.tanh(self.amp_mlp(force_global_n)).squeeze(-1)  # (B,)
        inj_raw = (g_contact * amp)                                             # (B,)

        # =====================================================================
        # NEW: Norm-ratio injection limiter (core)
        # inj_raw is treated as "target ratio": ||injected|| / ||vision||
        # We enforce realized_ratio <= inj_ratio_cap by scaling delta accordingly.
        # =====================================================================

        # Token-wise norms
        v_norm = v_tok.norm(dim=-1, keepdim=True).clamp_min(eps)     # (B,1,1)
        d_norm = delta.norm(dim=-1, keepdim=True).clamp_min(eps)     # (B,1,1)

        # clamp target ratio
        r_cap = self.inj_ratio_cap
        r_floor = self.inj_ratio_floor
        effective_ratio = torch.clamp(inj_raw, min=r_floor, max=r_cap)  # (B,)

        # scale so that ||scale*delta|| = effective_ratio * ||v||
        # scale shape: (B,1,1)
        scale = (effective_ratio[:, None, None] * (v_norm / d_norm))

        # fused
        injected = scale * delta
        v_fused = v_tok + injected

        # post norm
        v_fused = self.ln_out(v_fused)

        # ---- debug ----
        if self.enable_debug:
            with torch.no_grad():
                self._dbg_step += 1

                # squeeze norms
                vn = v_norm.squeeze(-1).squeeze(-1)  # (B,)
                dn = d_norm.squeeze(-1).squeeze(-1)  # (B,)

                inj_norm = injected.norm(dim=-1).squeeze(1)  # (B,)
                realized_ratio = inj_norm / (vn + eps)       # (B,)

                # extra norms/ratios you asked for
                fused_norm = v_fused.norm(dim=-1).squeeze(1)     # (B,)
                delta_over_v = dn / (vn + eps)                   # (B,)
                fused_over_v = fused_norm / (vn + eps)           # (B,)

                # head gate stats
                gh = g_head  # (B,H)

                # phase/contact summaries
                phase_id = torch.argmax(p_phase, dim=-1) if p_phase.ndim == 2 else None  # (B,)

                # delta alignment with v
                v_dir = v_tok.squeeze(1)
                d_dir = delta.squeeze(1)
                cos_vd = torch.sum(
                    v_dir * d_dir, dim=-1
                ) / (v_dir.norm(dim=-1) * d_dir.norm(dim=-1) + eps)  # (B,)

                # aggregate force_global stats
                fgn = force_global_n.norm(dim=-1)
                fg_mu = force_global_n.mean(dim=-1)
                fg_std = force_global_n.std(dim=-1, unbiased=False)

                # scale stats
                sc = scale.squeeze(1).squeeze(1)  # (B,)

                # ---- write CSV every N steps (timestamped file, fixed dir) ----
                if (self._dbg_step % self._dbg_every) == 0:
                    import os, time
                    dbg_dir = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video/PAEP_v2/fusion_debug"
                    os.makedirs(dbg_dir, exist_ok=True)

                    # lazy init file
                    if (not hasattr(self, "_dbg_log_path")) or (self._dbg_log_path is None):
                        ts = time.strftime("%Y%m%d_%H%M%S")
                        self._dbg_log_path = os.path.join(dbg_dir, f"fusion_debug_{ts}.csv")
                        with open(self._dbg_log_path, "w", buffering=1) as f:
                            f.write(
                                "step,B,cap,floor,"
                                "inj_raw_mean,inj_raw_max,effective_ratio_mean,effective_ratio_max,"
                                "v_norm_mean,v_norm_max,delta_norm_mean,delta_norm_max,"
                                "inj_norm_mean,inj_norm_max,fused_norm_mean,fused_norm_max,"
                                "delta_over_v_mean,delta_over_v_max,inj_over_v_mean,inj_over_v_max,"
                                "fused_over_v_mean,fused_over_v_max,"
                                "g_contact_mean,g_contact_max,amp_mean,amp_max,"
                                "g_head_mean,g_head_min,g_head_max,"
                                "p_contact_mean,p_contact_max,"
                                "cos_vd_mean,cos_vd_min,cos_vd_max,"
                                "force_global_n_norm_mean,force_global_n_mu_mean,force_global_n_std_mean,"
                                "scale_mean,scale_min,scale_max,"
                                "phase0_frac,phase1_frac,phase2_frac\n"
                            )

                    # phase fractions
                    if phase_id is not None:
                        frac0 = (phase_id == 0).float().mean().item()
                        frac1 = (phase_id == 1).float().mean().item()
                        frac2 = (phase_id == 2).float().mean().item()
                    else:
                        frac0 = frac1 = frac2 = float("nan")

                    # write one row
                    with open(self._dbg_log_path, "a", buffering=1) as f:
                        f.write(
                            f"{self._dbg_step},{B},{self.inj_ratio_cap:.6f},{self.inj_ratio_floor:.6f},"
                            f"{inj_raw.mean().item():.6f},{inj_raw.max().item():.6f},"
                            f"{effective_ratio.mean().item():.6f},{effective_ratio.max().item():.6f},"
                            f"{vn.mean().item():.6f},{vn.max().item():.6f},"
                            f"{dn.mean().item():.6f},{dn.max().item():.6f},"
                            f"{inj_norm.mean().item():.6f},{inj_norm.max().item():.6f},"
                            f"{fused_norm.mean().item():.6f},{fused_norm.max().item():.6f},"
                            f"{delta_over_v.mean().item():.6f},{delta_over_v.max().item():.6f},"
                            f"{realized_ratio.mean().item():.6f},{realized_ratio.max().item():.6f},"
                            f"{fused_over_v.mean().item():.6f},{fused_over_v.max().item():.6f},"
                            f"{g_contact.mean().item():.6f},{g_contact.max().item():.6f},"
                            f"{amp.mean().item():.6f},{amp.max().item():.6f},"
                            f"{gh.mean().item():.6f},{gh.min().item():.6f},{gh.max().item():.6f},"
                            f"{p_contact.mean().item():.6f},{p_contact.max().item():.6f},"
                            f"{cos_vd.mean().item():.6f},{cos_vd.min().item():.6f},{cos_vd.max().item():.6f},"
                            f"{fgn.mean().item():.6f},{fg_mu.mean().item():.6f},{fg_std.mean().item():.6f},"
                            f"{sc.mean().item():.6f},{sc.min().item():.6f},{sc.max().item():.6f},"
                            f"{frac0:.6f},{frac1:.6f},{frac2:.6f}\n"
                        )

                # keep original debug dict (and extend)
                self._last_fusion_debug = {
                    # norms
                    "v_norm_mean": vn.mean().detach(),
                    "v_norm_max": vn.max().detach(),
                    "delta_norm_mean": dn.mean().detach(),
                    "delta_norm_max": dn.max().detach(),
                    "inj_norm_mean": inj_norm.mean().detach(),
                    "inj_norm_max": inj_norm.max().detach(),
                    "fused_norm_mean": fused_norm.mean().detach(),
                    "fused_norm_max": fused_norm.max().detach(),

                    # ratios
                    "inj_raw_mean": inj_raw.mean().detach(),
                    "inj_raw_max": inj_raw.max().detach(),
                    "effective_ratio_mean": effective_ratio.mean().detach(),
                    "effective_ratio_max": effective_ratio.max().detach(),
                    "realized_ratio_mean": realized_ratio.mean().detach(),
                    "realized_ratio_max": realized_ratio.max().detach(),
                    "delta_over_v_mean": delta_over_v.mean().detach(),
                    "delta_over_v_max": delta_over_v.max().detach(),
                    "fused_over_v_mean": fused_over_v.mean().detach(),
                    "fused_over_v_max": fused_over_v.max().detach(),
                    "inj_ratio_cap": torch.tensor(self.inj_ratio_cap, device=vn.device),

                    # scale (coefficient applied to delta)
                    "scale_mean": sc.mean().detach(),
                    "scale_max": sc.max().detach(),
                    "scale_min": sc.min().detach(),

                    # gates
                    "g_contact_mean": g_contact.mean().detach(),
                    "g_contact_max": g_contact.max().detach(),
                    "amp_mean": amp.mean().detach(),
                    "amp_max": amp.max().detach(),
                    "g_head_mean": gh.mean().detach(),
                    "g_head_min": gh.min().detach(),
                    "g_head_max": gh.max().detach(),

                    # direction/alignment indicators
                    "cos_v_delta_mean": cos_vd.mean().detach(),
                    "cos_v_delta_min": cos_vd.min().detach(),
                    "cos_v_delta_max": cos_vd.max().detach(),

                    # force_global_n stats
                    "force_global_n_norm_mean": fgn.mean().detach(),
                    "force_global_n_mu_mean": fg_mu.mean().detach(),
                    "force_global_n_std_mean": fg_std.mean().detach(),

                    # phase/contact quick sanity
                    "p_contact_mean": p_contact.mean().detach(),
                    "p_contact_max": p_contact.max().detach(),
                }

                if phase_id is not None:
                    # fraction per phase id (0/1/2)
                    for k in [0, 1, 2]:
                        self._last_fusion_debug[f"phase_id_eq_{k}_frac"] = (phase_id == k).float().mean().detach()

        return v_fused.squeeze(1)
