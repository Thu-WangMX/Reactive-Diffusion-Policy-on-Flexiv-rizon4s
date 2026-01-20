# PAEP/paep_train/paep_model.py
from typing import Optional, Dict

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18, ResNet18_Weights


class ForceEncoder(nn.Module):
    # input: (B, L, 6)
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
        # x: (B, L, 6)
        x = x.transpose(1, 2)           # (B, 6, L)
        x = self.conv(x)                # (B, C, L)
        x = x.transpose(1, 2)           # (B, L, C)
        out, _ = self.gru(x)            # (B, L, H)
        return out[:, -1]               # (B, H)


class VisionEncoder(nn.Module):
    def __init__(self, pretrained: bool = True, out_dim: int = 256):
        super().__init__()
        if pretrained:
            backbone = resnet18(weights=ResNet18_Weights.DEFAULT)
        else:
            backbone = resnet18(weights=None)

        # remove fc
        backbone.fc = nn.Identity()
        self.backbone = backbone  # outputs 512
        self.proj = nn.Linear(512, out_dim)

    def forward(self, x: torch.Tensor, img_size: int = 224) -> torch.Tensor:
        # x: (B,3,H,W) float in [0,1]
        if x.shape[-1] != img_size or x.shape[-2] != img_size:
            x = F.interpolate(x, size=(img_size, img_size), mode="bilinear", align_corners=False)
        feat = self.backbone(x)     # (B,512)
        return self.proj(feat)      # (B,out_dim)


class ProprioEncoder(nn.Module):
    def __init__(self, in_dim=9, out_dim=128):
        super().__init__()
        self.mlp = nn.Sequential(
            nn.Linear(in_dim, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, out_dim),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        return self.mlp(x)


class PAEPFutureNet(nn.Module):
    """
    Single-step:
      (I_ext[t], I_wrist[t], F_hist[t-L+1:t], pose[t]) -> logits over events at t+delta
    """
    def __init__(
        self,
        num_events: int = 6,
        img_pretrained: bool = True,
        img_feat_dim: int = 256,
        force_k: int = 25,
        force_feat_dim: int = 256,
        proprio_dim: int = 128,
        fusion_hidden: int = 256,
    ):
        super().__init__()
        self.ext_enc = VisionEncoder(pretrained=img_pretrained, out_dim=img_feat_dim)
        self.wrist_enc = VisionEncoder(pretrained=img_pretrained, out_dim=img_feat_dim)

        self.force_enc = ForceEncoder(in_dim=6, conv_channels=128, force_k=force_k, gru_hidden=force_feat_dim)
        self.prop_enc = ProprioEncoder(in_dim=9, out_dim=proprio_dim)

        in_dim = img_feat_dim * 2 + force_feat_dim + proprio_dim
        self.fusion = nn.Sequential(
            nn.Linear(in_dim, fusion_hidden),
            nn.ReLU(inplace=True),
            nn.Linear(fusion_hidden, fusion_hidden),
            nn.ReLU(inplace=True),
        )
        self.head = nn.Linear(fusion_hidden, num_events)

    def forward(
        self,
        ext_img: torch.Tensor,
        wrist_img: torch.Tensor,
        wrench_hist: torch.Tensor,
        pose: torch.Tensor,
        norm: Optional[Dict[str, torch.Tensor]] = None,
        img_size: int = 224,
    ) -> torch.Tensor:
        # ext_img, wrist_img: (B,3,H,W) float
        # wrench_hist: (B,L,6) float
        # pose: (B,9)
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
        logits = self.head(h)
        return logits

    def set_vision_trainable(self, trainable: bool):
        for p in self.ext_enc.backbone.parameters():
            p.requires_grad = trainable
        for p in self.wrist_enc.backbone.parameters():
            p.requires_grad = trainable
