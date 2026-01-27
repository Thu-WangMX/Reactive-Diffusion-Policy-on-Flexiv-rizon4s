# paep_model.py
from typing import Optional, Dict, Literal

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18, ResNet18_Weights



# =========================
# Force Encoders
# =========================
class ForceEncoderGRU(nn.Module):
    """
    1D CNN + GRU
    input:  (B, L, 6)
    output: (B, H)
    """
    def __init__(self, in_dim=6, conv_channels=128, force_k=25, gru_hidden=256):
        super().__init__()
        self.conv = nn.Sequential(
            nn.Conv1d(in_dim, conv_channels, kernel_size=force_k, padding=force_k // 2),
            nn.ReLU(inplace=True),
            nn.Conv1d(conv_channels, conv_channels, kernel_size=force_k, padding=force_k // 2),
            nn.ReLU(inplace=True),
        )
        self.gru = nn.GRU(input_size=conv_channels, hidden_size=gru_hidden, batch_first=True)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x.transpose(1, 2)   # (B, 6, L)
        x = self.conv(x)        # (B, C, L)
        x = x.transpose(1, 2)   # (B, L, C)
        out, _ = self.gru(x)    # (B, L, H)
        return out[:, -1]       # (B, H)


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


# =========================
# Vision / Proprio Encoders
# =========================
class VisionEncoder(nn.Module):
    def __init__(self, pretrained: bool = True, out_dim: int = 256):
        super().__init__()
        if pretrained:
            backbone = resnet18(weights=ResNet18_Weights.DEFAULT)
        else:
            backbone = resnet18(weights=None)
        backbone.fc = nn.Identity()
        self.backbone = backbone   # outputs 512
        self.proj = nn.Linear(512, out_dim)

    def forward(self, x: torch.Tensor, img_size: int = 224) -> torch.Tensor:
        if x.shape[-1] != img_size or x.shape[-2] != img_size:
            x = F.interpolate(x, size=(img_size, img_size), mode="bilinear", align_corners=False)
        feat = self.backbone(x)    # (B,512)
        return self.proj(feat)     # (B,out_dim)


class ProprioEncoder(nn.Module):
    def __init__(self, in_dim=9, out_dim=128):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(in_dim, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, out_dim),
            nn.ReLU(inplace=True),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.mlp(x)


# =========================
# PAEP Network (Dual Heads)
# =========================
class PAEPFutureNet(nn.Module):
    """
    输入: ext_img, wrist_img, wrench_hist(B,L,6), pose(B,9)
    输出(默认 forward): 概率分布
      - p_contact: (B,1) = sigmoid(logit_contact)
      - p_phase:   (B,P) = softmax(logits_phase)

    训练用: forward_logits() 返回 logits
      - logit_contact: (B,1)
      - logits_phase:  (B,P)
    """
    def __init__(
        self,
        num_events: int = 3,
        img_pretrained: bool = True,
        img_feat_dim: int = 256,
        # force encoder switch
        force_encoder: str = "tcn",   # "gru" | "tcn"
        force_k: int = 25,            # GRU conv kernel
        force_feat_dim: int = 256,
        # TCN params
        tcn_channels: int = 256,
        tcn_kernel: int = 5,
        tcn_blocks: int = 4,
        tcn_dropout: float = 0.0,
        tcn_pool: str = "mean",       # "mean" | "last"
        # proprio + fusion
        proprio_dim: int = 128,
        fusion_hidden: int = 256,
    ):
        super().__init__()
        self.ext_enc = VisionEncoder(pretrained=img_pretrained, out_dim=img_feat_dim)
        self.wrist_enc = VisionEncoder(pretrained=img_pretrained, out_dim=img_feat_dim)

        fe = str(force_encoder).lower()
        self.force_encoder_type = fe
        if fe == "tcn":
            self.force_enc = ForceEncoderTCN(
                in_dim=6,
                channels=tcn_channels,
                out_dim=force_feat_dim,
                kernel_size=tcn_kernel,
                num_blocks=tcn_blocks,
                dropout=tcn_dropout,
                pool=tcn_pool,
            )
        else:
            self.force_encoder_type = "gru"
            self.force_enc = ForceEncoderGRU(
                in_dim=6,
                conv_channels=128,
                force_k=force_k,
                gru_hidden=force_feat_dim,
            )

        self.prop_enc = ProprioEncoder(in_dim=9, out_dim=proprio_dim)

        in_dim = img_feat_dim * 2 + force_feat_dim + proprio_dim
        self.fusion = nn.Sequential(
            nn.Linear(in_dim, fusion_hidden),
            nn.ReLU(inplace=True),
            nn.Linear(fusion_hidden, fusion_hidden),
            nn.ReLU(inplace=True),
        )

        self.phase_head = nn.Linear(fusion_hidden, num_events)  # (B,P)
        self.contact_head = nn.Linear(fusion_hidden, 1)         # (B,1)

    def forward_logits(
        self,
        ext_img: torch.Tensor,
        wrist_img: torch.Tensor,
        wrench_hist: torch.Tensor,
        pose: torch.Tensor,
        norm: Optional[Dict[str, torch.Tensor]] = None,
        img_size: int = 224,
    ):
        # normalization (same as old)
        if norm is not None:
            wm, ws = norm["wrench_mean"], norm["wrench_std"]
            pm, ps = norm["pose_mean"], norm["pose_std"]
            wrench_hist = (wrench_hist - wm[None, None, :]) / ws[None, None, :]
            pose = (pose - pm[None, :]) / ps[None, :]

        v1 = self.ext_enc(ext_img, img_size=img_size)
        v2 = self.wrist_enc(wrist_img, img_size=img_size)
        f = self.force_enc(wrench_hist)
        p = self.prop_enc(pose)

        z = torch.cat([v1, v2, f, p], dim=-1)
        h = self.fusion(z)

        logits_phase = self.phase_head(h)     # (B,P)
        logit_contact = self.contact_head(h)  # (B,1)
        return logits_phase, logit_contact

    def forward(
        self,
        ext_img: torch.Tensor,
        wrist_img: torch.Tensor,
        wrench_hist: torch.Tensor,
        pose: torch.Tensor,
        norm: Optional[Dict[str, torch.Tensor]] = None,
        img_size: int = 224,
    ):
        logits_phase, logit_contact = self.forward_logits(
            ext_img, wrist_img, wrench_hist, pose, norm=norm, img_size=img_size
        )
        p_phase = torch.softmax(logits_phase, dim=-1)   # (B,P)
        p_contact = torch.sigmoid(logit_contact)        # (B,1)
        return {"p_phase": p_phase, "p_contact": p_contact}

    def set_vision_trainable(self, trainable: bool):
        for p in self.ext_enc.backbone.parameters():
            p.requires_grad = trainable
        for p in self.wrist_enc.backbone.parameters():
            p.requires_grad = trainable
