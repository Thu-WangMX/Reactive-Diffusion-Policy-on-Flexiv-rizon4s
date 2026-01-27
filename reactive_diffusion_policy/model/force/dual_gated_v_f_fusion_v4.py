# Core PyTorch imports
import torch
import torch.nn as nn
from torch import Tensor

from reactive_diffusion_policy.model.attention.headwise_cro_attention import HeadWiseCrossAttention


class DualGatedVisionForceFusion(nn.Module):
    """
    Dual-Gated Vision-Force Fusion (Orthogonal Residual)

    - Gate-1 (contact gate): controls whether/how much force can affect vision (per-sample scalar).
    - Gate-2 (head-wise gate): phase(-and optionally contact)-conditioned gating over attention heads.
    - Orthogonal residual: remove the component of delta parallel to the vision token direction to reduce
      "visual pollution" (i.e., force pulling vision along its dominant direction).

    Form:
        v_fused = LN_out( v + scale * unit( delta_perp ) )

    where:
        scale = alpha * g_contact (optionally bounded by inj_ratio_cap w.r.t. ||v||)
        delta = CrossAttn(Q=v, K/V=force_tokens) with head gating
        delta_perp = delta - proj_v(delta)

    Notes:
    - This module is designed to be stable and easy to tune:
        * alpha: single global gain (fixed or learnable-bounded)
        * g_contact: PAEP contact probability (should allow 0)
        * inj_ratio_cap: optional safety cap on injected_norm / vision_norm
    """

    def __init__(
        self,
        d_model: int,
        n_heads: int = 8,
        gate_hidden: int = 128,
        dropout: float = 0.0,
        use_contact_in_head_gate: bool = False,
        # Norm switches
        use_ln: bool = True,
        ln_out: bool = True,
        eps: float = 1e-6,
        # Residual gain
        alpha: float = 0.05,
        learnable_alpha: bool = False,
        alpha_max: float = 0.2,
        # Optional safety: injected_norm <= inj_ratio_cap * vision_norm
        inj_ratio_cap: float | None = None,
        # Debug (no file I/O)
        enable_debug: bool = False,
    ):
        super().__init__()
        self.d_model = int(d_model)
        self.n_heads = int(n_heads)
        self.use_contact_in_head_gate = bool(use_contact_in_head_gate)

        self.use_ln = bool(use_ln)
        self.use_ln_out = bool(ln_out)
        self.eps = float(eps)

        self.enable_debug = bool(enable_debug)
        self._last_fusion_debug = None

        # attention
        self.cross_attn = HeadWiseCrossAttention(d_model=d_model, n_heads=n_heads, dropout=dropout)

        # head-wise gate (phase -> heads), optionally include p_contact
        head_gate_in = 4 if self.use_contact_in_head_gate else 3
        self.head_gate_mlp = nn.Sequential(
            nn.Linear(head_gate_in, gate_hidden),
            nn.ReLU(inplace=True),
            nn.Linear(gate_hidden, n_heads),
        )

        # alpha: single knob, fixed or learnable-bounded
        self.learnable_alpha = bool(learnable_alpha)
        self.alpha_max = float(alpha_max)
        if self.learnable_alpha:
            # initialize near provided alpha
            a0 = float(alpha)
            a0 = max(0.0, min(a0, self.alpha_max))
            # inverse-sigmoid for a0/alpha_max, avoid inf
            x = a0 / max(self.alpha_max, 1e-6)
            x = min(max(x, 1e-4), 1 - 1e-4)
            init = torch.log(torch.tensor(x / (1 - x)))
            self._alpha_logit = nn.Parameter(init.clone().detach())
        else:
            self.register_buffer("_alpha_const", torch.tensor(float(alpha)), persistent=False)

        self.inj_ratio_cap = inj_ratio_cap if inj_ratio_cap is None else float(inj_ratio_cap)

        # ---- LayerNorms ----
        if self.use_ln:
            self.ln_v = nn.LayerNorm(d_model)
            self.ln_force = nn.LayerNorm(d_model)
            self.ln_delta = nn.LayerNorm(d_model)
            self.ln_out = nn.LayerNorm(d_model) if self.use_ln_out else nn.Identity()
        else:
            self.ln_v = nn.Identity()
            self.ln_force = nn.Identity()
            self.ln_delta = nn.Identity()
            self.ln_out = nn.Identity()

    def _get_alpha(self) -> Tensor:
        if self.learnable_alpha:
            return self.alpha_max * torch.sigmoid(self._alpha_logit)  # scalar tensor
        return self._alpha_const  # scalar tensor

    @staticmethod
    def _unit(x: Tensor, eps: float) -> Tensor:
        return x / x.norm(dim=-1, keepdim=True).clamp_min(eps)

    def forward(
        self,
        v_feat: Tensor,          # (B,D)
        force_tokens: Tensor,    # (B,L,D)
        force_global: Tensor,    # (B,D) - kept for API compatibility (not used here)
        g_contact: Tensor,       # (B,)
        p_phase: Tensor,         # (B,3)
        p_contact: Tensor,       # (B,1)
    ) -> Tensor:
        """
        Returns:
            v_fused: (B,D)
        """
        B = v_feat.shape[0]
        eps = self.eps

        # vision token
        v_tok = v_feat.unsqueeze(1)  # (B,1,D)

        # pre-norm for attention
        v_q = self.ln_v(v_tok)                  # (B,1,D)
        f_kv = self.ln_force(force_tokens)      # (B,L,D)

        # head-wise gate (Gate-2)
        if self.use_contact_in_head_gate:
            hg_in = torch.cat([p_phase, p_contact], dim=-1)  # (B,4)
        else:
            hg_in = p_phase                                  # (B,3)
        g_head = torch.sigmoid(self.head_gate_mlp(hg_in))     # (B,H)

        # cross-attn residual (in normalized feature space)
        delta = self.cross_attn(v_q, f_kv, g_head)            # (B,1,D)
        delta = self.ln_delta(delta)

        # orthogonal residual: remove component parallel to v_q
        # proj = (<delta, v_q> / <v_q, v_q>) * v_q
        dot = (delta * v_q).sum(dim=-1, keepdim=True)                         # (B,1,1)
        denom = (v_q * v_q).sum(dim=-1, keepdim=True).clamp_min(eps)          # (B,1,1)
        proj = dot / denom * v_q                                              # (B,1,D)
        delta_perp = delta - proj                                             # (B,1,D)

        # unit direction (safe, scale is controlled only by alpha*g_contact)
        d_hat = self._unit(delta_perp, eps=eps)

        # Gate-1: contact strength (should allow 0)
        alpha = self._get_alpha()                                             # scalar
        scale = (alpha * g_contact).view(B, 1, 1)                              # (B,1,1)

        # optional safety cap on injected_norm / vision_norm
        if self.inj_ratio_cap is not None:
            v_norm = v_q.norm(dim=-1, keepdim=True).clamp_min(eps)            # (B,1,1)
            max_scale = self.inj_ratio_cap * v_norm                           # (B,1,1)
            scale = torch.minimum(scale, max_scale)

        v_fused = v_q + scale * d_hat                                         # (B,1,D)
        v_fused = self.ln_out(v_fused)

        if self.enable_debug:
            with torch.no_grad():
                vn = v_q.norm(dim=-1).squeeze(1)                              # (B,)
                inj = (scale * d_hat).norm(dim=-1).squeeze(1)                 # (B,)
                cos_vd = (v_q * delta).sum(dim=-1).squeeze(1) / (
                    v_q.norm(dim=-1).squeeze(1) * delta.norm(dim=-1).squeeze(1) + eps
                )
                self._last_fusion_debug = {
                    "alpha": float(alpha.detach().cpu().item()),
                    "g_contact_mean": float(g_contact.mean().detach().cpu().item()),
                    "v_norm_mean": float(vn.mean().detach().cpu().item()),
                    "inj_norm_mean": float(inj.mean().detach().cpu().item()),
                    "inj_over_v_mean": float((inj / (vn + eps)).mean().detach().cpu().item()),
                    "cos_vd_mean": float(cos_vd.mean().detach().cpu().item()),
                    "g_head_mean": float(g_head.mean().detach().cpu().item()),
                }

        return v_fused.squeeze(1)
