# PAEP: Phase-Aware Event Prediction for Contact-Rich Manipulation 主网络
from typing import Optional, Dict, Literal

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18, ResNet18_Weights
from PAEP.encoder.force_gru import ForceEncoderGRU
from PAEP.encoder.force_tcn import ForceEncoderTCN
from PAEP.encoder.proprio_mlp import ProprioEncoder
from PAEP.encoder.resnet import VisionEncoder   


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
        tcn_kernel: int = 7,
        tcn_blocks: int = 4,
        tcn_dropout: float = 0.0,
        tcn_pool: str = "mean",       # "mean" | "last"
        # proprio + fusion
        proprio_dim: int = 128,
        fusion_hidden: int = 256,
    ):
        super().__init__()
        self.ext_enc = VisionEncoder(pretrained=img_pretrained, out_dim=img_feat_dim)
        self.left_wrist_enc = VisionEncoder(pretrained=img_pretrained, out_dim=img_feat_dim)
        self.right_wrist_enc = VisionEncoder(pretrained=img_pretrained, out_dim=img_feat_dim)


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

        in_dim = img_feat_dim * 3 + force_feat_dim + proprio_dim
        self.fusion = nn.Sequential(
            nn.Linear(in_dim, fusion_hidden),
            nn.ReLU(inplace=True),
            nn.Linear(fusion_hidden, fusion_hidden),
            nn.ReLU(inplace=True),
        )

        self.phase_head = nn.Linear(fusion_hidden, num_events)  # (B,P)
        self.contact_head = nn.Linear(fusion_hidden, 1)         # (B,1)

    def forward_logits( #返回的是未经过归一化处理的原始预测分数（Logits）,数值可以是任意实数，训练用，数值稳定性更好
        self,
        ext_img: torch.Tensor,
        left_wrist_img: torch.Tensor,
        right_wrist_img: torch.Tensor,
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

        v_ext = self.ext_enc(ext_img, img_size=img_size)
        v_lw  = self.left_wrist_enc(left_wrist_img, img_size=img_size)
        v_rw  = self.right_wrist_enc(right_wrist_img, img_size=img_size)
        f = self.force_enc(wrench_hist)
        p = self.prop_enc(pose)

        z = torch.cat([v_ext, v_lw, v_rw, f, p], dim=-1)
        h = self.fusion(z)

        logits_phase = self.phase_head(h)     # (B,P)
        logit_contact = self.contact_head(h)  # (B,1)
        return logits_phase, logit_contact

    def forward( #返回的是经过激活函数处理后的概率值，以字典形式封装，推理用
        self,
        ext_img: torch.Tensor,
        left_wrist_img: torch.Tensor,
        right_wrist_img: Optional[torch.Tensor] = None,
        wrench_hist: Optional[torch.Tensor] = None,
        pose: Optional[torch.Tensor] = None,
        norm: Optional[Dict[str, torch.Tensor]] = None,
        img_size: int = 224,
    ):
        # 兼容旧接口：如果没给 right_wrist_img，就用 left_wrist_img 复制一份
        if right_wrist_img is None:
            right_wrist_img = left_wrist_img

        logits_phase, logit_contact = self.forward_logits(
            ext_img, left_wrist_img, right_wrist_img, wrench_hist, pose, norm=norm, img_size=img_size
        )
        p_phase = torch.softmax(logits_phase, dim=-1)
        p_contact = torch.sigmoid(logit_contact)
        return {"p_phase": p_phase, "p_contact": p_contact}

    def set_vision_trainable(self, trainable: bool, layer4_only: bool = False):
        encs = [self.ext_enc, self.left_wrist_enc, self.right_wrist_enc]
        for enc in encs:
            # first freeze/unfreeze all backbone
            for p in enc.backbone.parameters():
                p.requires_grad = bool(trainable) and (not layer4_only)

            # if only unfreeze layer4
            if bool(trainable) and bool(layer4_only):
                if hasattr(enc.backbone, "layer4"):
                    for p in enc.backbone.layer4.parameters():
                        p.requires_grad = True

