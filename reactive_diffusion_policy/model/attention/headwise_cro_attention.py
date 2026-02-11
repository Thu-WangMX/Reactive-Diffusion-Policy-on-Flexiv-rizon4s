# 核心PyTorch基础导入
import torch
import torch.nn as nn
import math  # 用于计算 sqrt(dh)
from typing import Optional  # 可选：若需要扩展类型注解，非必需但推荐

# 可选优化：导入Tensor类型让注解更简洁（推荐）
from torch import Tensor


# ============================================================
# Head-wise cross attention with per-head gating
# ============================================================
class HeadWiseCrossAttention(nn.Module):
    """
    Q: (B, Nq, D)   usually Nq=1
    KV: (B, Nk, D)
    gate_h: (B, H) in [0,1]
    """
    def __init__(self, d_model: int, n_heads: int, dropout: float = 0.0, attn_temperature: float = 1.0):

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
        self.attn_temperature = float(attn_temperature)

        self._last_attn_debug = None

    def forward(self, q: torch.Tensor, kv: torch.Tensor, gate_h: torch.Tensor) -> torch.Tensor:
        B, Nq, D = q.shape
        _, Nk, _ = kv.shape
        H = self.n_heads
        dh = self.d_head

        qh = self.q_proj(q).view(B, Nq, H, dh).transpose(1, 2)     # (B,H,Nq,dh)
        kh = self.k_proj(kv).view(B, Nk, H, dh).transpose(1, 2)    # (B,H,Nk,dh)
        vh = self.v_proj(kv).view(B, Nk, H, dh).transpose(1, 2)    # (B,H,Nk,dh)

        temp = max(self.attn_temperature, 1e-6)
        attn = torch.matmul(qh, kh.transpose(-2, -1)) / (math.sqrt(dh) * temp)
        attn = attn.clamp(min=-30.0, max=30.0)
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