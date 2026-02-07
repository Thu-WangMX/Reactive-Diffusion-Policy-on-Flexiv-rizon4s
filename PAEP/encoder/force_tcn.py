import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Literal

class CausalConv1d(nn.Module):
    """Causal conv: left padding only."""
    def __init__(self, in_ch: int, out_ch: int, kernel_size: int, dilation: int = 1):
        super().__init__()
        self.kernel_size = int(kernel_size)
        self.dilation = int(dilation)
        self.conv = nn.Conv1d(in_ch, out_ch, kernel_size=self.kernel_size, dilation=self.dilation, padding=0)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (B, C, L)
        pad_left = (self.kernel_size - 1) * self.dilation
        x = F.pad(x, (pad_left, 0))
        return self.conv(x)


class TCNBlock(nn.Module):
    """(CausalConv->ReLU->CausalConv->ReLU) + residual"""
    def __init__(self, ch: int, kernel_size: int, dilation: int, dropout: float = 0.0):
        super().__init__()
        self.conv1 = CausalConv1d(ch, ch, kernel_size=kernel_size, dilation=dilation)
        self.conv2 = CausalConv1d(ch, ch, kernel_size=kernel_size, dilation=dilation)
        self.act = nn.ReLU(inplace=True)
        self.drop = nn.Dropout(dropout) if dropout and dropout > 0 else nn.Identity()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        h = self.act(self.conv1(x))
        h = self.drop(h)
        h = self.act(self.conv2(h))
        h = self.drop(h)
        return x + h


class ForceEncoderTCN(nn.Module):
    """
    TCN = 1D Conv + Causal Padding + Dilation + Residual Connection
    input:  (B, L, 6)
    output: (B, out_dim)
    """
    def __init__(
        self,
        in_dim: int = 6,
        channels: int = 256,
        out_dim: int = 256,
        kernel_size: int = 5,
        num_blocks: int = 4,
        dropout: float = 0.0,
        pool: Literal["mean", "last"] = "mean",
    ):
        super().__init__()
        self.pool = pool
        self.in_proj = nn.Conv1d(in_dim, channels, kernel_size=1)
        blocks = []
        for i in range(num_blocks):
            d = 2 ** i
            blocks.append(TCNBlock(channels, kernel_size=kernel_size, dilation=d, dropout=dropout))
        self.blocks = nn.Sequential(*blocks)
        self.out_proj = nn.Linear(channels, out_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (B, L, 6)
        x = x.transpose(1, 2)       # (B, 6, L)
        x = self.in_proj(x)         # (B, C, L)
        x = self.blocks(x)          # (B, C, L)
        x = x.transpose(1, 2)       # (B, L, C)

        if self.pool == "last":
            feat = x[:, -1]         # (B, C)
        else:
            feat = x.mean(dim=1)    # (B, C) 更适合 “未来窗口 gate”

        return self.out_proj(feat)  # (B, out_dim)