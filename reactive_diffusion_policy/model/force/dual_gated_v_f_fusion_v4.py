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
        alpha: float = 0.05,#TODO
        learnable_alpha: bool = False,
        alpha_max: float = 0.2,
        # Optional safety: injected_norm <= inj_ratio_cap * vision_norm
        inj_ratio_cap: float | None = None,
        num_events: int = 4,
        # Debug (no file I/O)
        enable_debug: bool = True,#TODO
    ):
        super().__init__()
        self.d_model = int(d_model)
        self.n_heads = int(n_heads)
        self.use_contact_in_head_gate = bool(use_contact_in_head_gate)
        self.num_events = int(num_events)
        self.use_ln = bool(use_ln)
        self.use_ln_out = bool(ln_out)
        self.eps = float(eps)

        self.enable_debug = bool(enable_debug)
        self._last_fusion_debug = None

        # attention
        self.cross_attn = HeadWiseCrossAttention(
            d_model=d_model, n_heads=n_heads, dropout=dropout, attn_temperature=1.5
        )


        # head-wise gate (phase -> heads), optionally include p_contact

        head_gate_in = int(self.num_events) + (1 if self.use_contact_in_head_gate else 0)

        self.head_gate_mlp = nn.Sequential(
            nn.Linear(head_gate_in, gate_hidden),
            nn.ReLU(inplace=True),
            nn.Linear(gate_hidden, n_heads),
        )


        # alpha: single knob, fixed or learnable-bounded，alpha 值永远在 $[0, \text{alpha\_max}]$ 之间
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
        v_tok = v_feat.unsqueeze(1)  # (B,1,D)#在第几个维度前插入一个新的维度

        # pre-norm for attention
        v_q = self.ln_v(v_tok)                  # (B,1,D)
        #f_kv = self.ln_force(force_tokens)      # (B,L,D)
        f_kv = self.ln_force(force_tokens)      # (B,L,D)

        # -------- force token safety clip (prevents rare explosions) --------
        # clip by per-token L2 norm: ||token|| <= force_tok_norm_cap
        force_tok_norm_cap = 50.0  # 推荐先用 30~80 之间试；你现在爆到1e4，50已经能挡掉绝大多数异常
        tok_norm = f_kv.norm(dim=-1, keepdim=True).clamp_min(self.eps)   # (B,L,1)
        scale_clip = torch.clamp(force_tok_norm_cap / tok_norm, max=1.0) # (B,L,1)
        f_kv = f_kv * scale_clip


        # head-wise gate (Gate-2)
        if self.use_contact_in_head_gate:
            hg_in = torch.cat([p_phase, p_contact], dim=-1)  # (B, num_events+1)
        else:
            hg_in = p_phase                                   # (B, num_events)
                        
        g_head = torch.sigmoid(self.head_gate_mlp(hg_in))     # (B,H) 其中 $B$ 是 Batch Size，$H$ 是 Head Number

        # cross-attn residual (in normalized feature space)
        delta = self.cross_attn(v_q, f_kv, g_head)            # (B,1,D)
        delta = self.ln_delta(delta)

        # orthogonal residual: remove component parallel to RAW vision token (pre-LN)
        # proj = (<delta, v_ref> / <v_ref, v_ref>) * v_ref
        v_ref = v_tok  # raw vision token, shape (B,1,D)
        dot = (delta * v_ref).sum(dim=-1, keepdim=True)                        # (B,1,1)
        denom = (v_ref * v_ref).sum(dim=-1, keepdim=True).clamp_min(eps)       # (B,1,1)
        proj = dot / denom * v_ref                                             # (B,1,D)
        delta_perp = delta - proj                                              # (B,1,D)
                                          # (B,1,D)

        # unit direction (safe, scale is controlled only by alpha*g_contact)
        d_hat = self._unit(delta_perp, eps=eps) #单位化

        # Gate-1: contact strength (should allow 0)
        alpha = self._get_alpha()                                             # scalar
        scale = (alpha * g_contact).view(B, 1, 1)                              # (B,1,1)

        # optional safety cap on injected_norm / vision_norm
        if self.inj_ratio_cap is not None:
            # use RAW vision token norm (pre-LN) for a more stable physical reference
            v_raw_norm = v_tok.norm(dim=-1, keepdim=True).clamp_min(eps)      # (B,1,1)
            max_scale = self.inj_ratio_cap * v_raw_norm                       # (B,1,1)
            scale = torch.minimum(scale, max_scale)

        v_fused = v_q + scale * d_hat                                         # (B,1,D)
        v_fused = self.ln_out(v_fused)


        if self.enable_debug:
            with torch.no_grad():
                # ---- norms ----
                vn = v_tok.norm(dim=-1).squeeze(1)                       # (B,)
                dn = delta.norm(dim=-1).squeeze(1)                          # (B,)
                inj_vec = (scale * d_hat).squeeze(1)                        # (B,D)
                inj = inj_vec.norm(dim=-1)                                  # (B,)

                # ---- scales ----
                raw_scale = (alpha * g_contact).view(B)                     # (B,) 未cap
                sc = scale.view(B)                                          # (B,) cap后真实scale

                # ---- ratios ----
                eps_ = eps
                delta_over_v = dn / (vn + eps_)
                inj_over_v   = inj / (vn + eps_)

                # ---- cos(v_ref,delta) ----
                v_ref = v_tok  # raw vision token (pre-LN) for interpretability
                denom = (v_ref.norm(dim=-1).squeeze(1).clamp_min(eps_) *
                         delta.norm(dim=-1).squeeze(1).clamp_min(eps_))
                cos_vd = (v_ref * delta).sum(dim=-1).squeeze(1) / (denom + eps_)


                # ---- phase/contact stats ----
                # p_phase: (B,3), p_contact: (B,1)
                # NOTE: 现在 p_phase 实际是 (B, num_events)，这里保留原注释不删
                phase_mean = p_phase.mean(dim=0)  # (num_events,)
                pc = p_contact.view(B)

                # 为了不破坏你原来的 debug key（p_phase0/1/2），这里保留 p0/p1/p2
                # 如果 num_events < 3（极少见），用 nan 占位，避免越界
                p0 = phase_mean[0] if phase_mean.numel() > 0 else torch.tensor(float("nan"), device=phase_mean.device)
                p1 = phase_mean[1] if phase_mean.numel() > 1 else torch.tensor(float("nan"), device=phase_mean.device)
                p2 = phase_mean[2] if phase_mean.numel() > 2 else torch.tensor(float("nan"), device=phase_mean.device)

                # ---- effective/realized ratio（解释：cap导致的缩放衰减比例）----
                # 1.0 = 没触发cap；<1 = 触发cap
                eff_ratio = sc / (raw_scale.abs() + eps_)

                self._last_fusion_debug = {
                    # ==========================================
                    # 1. 基础模长 (Norms) - 监控数值稳定性
                    # ==========================================
                    # 视觉特征原本的模长（基准）
                    "v_norm_mean": float(vn.mean().cpu().item()),
                    "v_norm_max":  float(vn.max().cpu().item()),
                    
                    # CrossAttn 算出来的原始力特征模长（在正交化和缩放之前）
                    "delta_norm_mean": float(dn.mean().cpu().item()),
                    
                    # 最终注入到视觉中的残差模长 (After Orthogonal & Scaling)
                    "inj_norm_mean": float(inj.mean().cpu().item()),
                    "inj_norm_max":  float(inj.max().cpu().item()),

                    # ==========================================
                    # 2. 几何关系 (Geometry) - 监控正交化效果
                    # ==========================================
                    # 视觉和原始力特征的余弦相似度。
                    # 接近 1.0/-1.0 说明力特征和视觉方向平行（冗余，会被正交化切掉）；
                    # 接近 0 说明力特征本身就是正交的（包含了视觉没有的新信息）。
                    "cos_v_delta_mean": float(cos_vd.mean().cpu().item()),
                    "cos_v_delta_min":  float(cos_vd.min().cpu().item()),
                    "cos_v_delta_max":  float(cos_vd.max().cpu().item()),

                    # ==========================================
                    # 3. 注入强度比例 (Injection Ratios) - 核心监控指标
                    # ==========================================
                    # 原始力特征相对于视觉特征的大小 (Before processing)
                    "delta_over_v_mean": float(delta_over_v.mean().cpu().item()),
                    
                    # [关键] 最终注入的力特征相对于视觉特征的大小 (After all processing)
                    # 如果此值过大(>0.5)，可能会破坏视觉特征；过小(<0.01)说明力没用上。
                    "inj_over_v_mean": float(inj_over_v.mean().cpu().item()),

                    # Safety Cap 生效比例：(实际Scale / 理论Scale)。
                    # 1.0 表示未触发截断；0.5 表示因为太强被强制砍半。
                    "effective_ratio_mean": float(eff_ratio.mean().cpu().item()),

                    # ==========================================
                    # 4. 门控与缩放 (Gating & Scaling)
                    # ==========================================
                    # 全局增益参数 alpha
                    "alpha": float(alpha.detach().cpu().item()),
                    
                    # 理论缩放系数 (alpha * g_contact)，未经过 Safety Cap
                    "raw_scale_mean": float(raw_scale.mean().cpu().item()),
                    
                    # 实际缩放系数 (Final Scale)，经过了 Safety Cap
                    "scale_mean": float(sc.mean().cpu().item()),
                    "scale_max":  float(sc.max().cpu().item()),

                    # 接触门控值 (Input g_contact)，通常 0 或 1
                    "g_contact_mean": float(g_contact.mean().cpu().item()),
                    
                    # Head-wise Gate 平均开启程度 (0~1)，监控有多少 Attention Heads 处于活跃状态
                    "g_head_mean": float(g_head.mean().detach().cpu().item()),

                    # ==========================================
                    # 5. 状态统计 (Phase Stats) - 监控输入分布
                    # ==========================================
                    # 输入的接触概率均值
                    "p_contact_mean": float(pc.mean().cpu().item()),
                    
                    # 各个相位的平均概率 (Phase 0, 1, 2)
                    "p_phase0_mean": float(p0.cpu().item()),
                    "p_phase1_mean": float(p1.cpu().item()),
                    "p_phase2_mean": float(p2.cpu().item()),
                }

                # 如果 num_events > 3（比如 plug-in charger 有 recovery），额外补齐 p_phase3_mean / p_phase4_mean ...
                # 不覆盖你原来的 key，只补充新增的 key
                for i in range(3, int(self.num_events)):
                    self._last_fusion_debug[f"p_phase{i}_mean"] = float(phase_mean[i].cpu().item())


        return v_fused.squeeze(1) #移除第1维（大小为1的维度）
