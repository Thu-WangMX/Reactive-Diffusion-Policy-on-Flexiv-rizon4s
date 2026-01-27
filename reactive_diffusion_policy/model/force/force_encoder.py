# 核心PyTorch基础导入
import torch
import torch.nn as nn
from typing import Tuple  # 用于类型注解中的Tuple

# 可选：如果需要类型检查/提示更严格，可添加（非必需但推荐）
from torch import Tensor

# ============================================================
# Force TCN token encoder: (B,L,6) -> (B,L,D) 给policy用的而不是给paep  
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