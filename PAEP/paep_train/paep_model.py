# paep_model.py
import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18

def make_resnet18(pretrained: bool = False):
    m = resnet18(weights=("IMAGENET1K_V1" if pretrained else None))
    m.fc = nn.Identity()
    return m, 512

class CausalConv1d(nn.Module):
    def __init__(self, in_ch, out_ch, k, dilation=1):
        super().__init__()
        self.k = int(k)
        self.dilation = int(dilation)
        self.conv = nn.Conv1d(in_ch, out_ch, kernel_size=self.k, dilation=self.dilation)

    def forward(self, x):
        # x: [B,C,T]
        pad = (self.k - 1) * self.dilation
        x = F.pad(x, (pad, 0))
        return self.conv(x)

class ForceEncoder(nn.Module):
    def __init__(self, in_dim=6, conv_dim=128, k=25, gru_hidden=256):
        super().__init__()
        self.cconv = CausalConv1d(in_dim, conv_dim, k)
        self.gru = nn.GRU(conv_dim, gru_hidden, batch_first=True)

    def forward(self, wrench):
        # wrench: [B,S,6]
        x = wrench.transpose(1, 2)         # [B,6,S]
        x = self.cconv(x)                  # [B,conv,S]
        x = x.transpose(1, 2).contiguous() # [B,S,conv]
        y, _ = self.gru(x)                 # [B,S,H]
        return y

class PAEPNet(nn.Module):
    def __init__(
        self,
        num_events=6,
        img_pretrained=False,
        img_feat_dim=256,
        force_k=25,
        force_hidden=256,
        proprio_dim=128,
        fusion_hidden=256,
        use_fusion_gru=True,
    ):
        super().__init__()
        self.ext_backbone, ext_dim = make_resnet18(pretrained=img_pretrained)
        self.wrist_backbone, wrist_dim = make_resnet18(pretrained=img_pretrained)

        self.ext_proj = nn.Linear(ext_dim, img_feat_dim)
        self.wrist_proj = nn.Linear(wrist_dim, img_feat_dim)

        self.force_enc = ForceEncoder(in_dim=6, conv_dim=128, k=force_k, gru_hidden=force_hidden)

        self.proprio = nn.Sequential(
            nn.Linear(9, 128),
            nn.ReLU(inplace=True),
            nn.Linear(128, proprio_dim),
            nn.ReLU(inplace=True),
        )

        fused_dim = img_feat_dim * 2 + force_hidden + proprio_dim
        self.use_fusion_gru = bool(use_fusion_gru)
        if self.use_fusion_gru:
            self.fusion_gru = nn.GRU(fused_dim, fusion_hidden, batch_first=True)
            head_in = fusion_hidden
        else:
            head_in = fused_dim

        self.head = nn.Linear(head_in, num_events)

    def _encode_img(self, backbone, proj, x):
        # x: [B,S,3,H,W]
        B, S, C, H, W = x.shape
        x = x.reshape(B * S, C, H, W)
        feat = backbone(x)                 # [B*S,512]
        feat = F.relu(proj(feat), inplace=True)
        return feat.reshape(B, S, -1)

    def forward(self, ext_img, wrist_img, wrench, tcp_pose):
        ext = self._encode_img(self.ext_backbone, self.ext_proj, ext_img)
        wrist = self._encode_img(self.wrist_backbone, self.wrist_proj, wrist_img)
        f = self.force_enc(wrench)
        p = self.proprio(tcp_pose)

        fused = torch.cat([ext, wrist, f, p], dim=-1)
        if self.use_fusion_gru:
            fused, _ = self.fusion_gru(fused)
        logits = self.head(fused)
        return logits

class PAEPStreamer(nn.Module):
    """
    Online step-by-step inference maintaining GRU hidden states.
    """
    def __init__(self, net: PAEPNet):
        super().__init__()
        self.net = net
        self.reset()

    def reset(self):
        self.h_force = None
        self.h_fusion = None

    @torch.no_grad()
    def step(self, ext_img_1, wrist_img_1, wrench_1, tcp_pose_1):
        net = self.net

        # images
        ext = net._encode_img(net.ext_backbone, net.ext_proj, ext_img_1)      # [B,1,D]
        wrist = net._encode_img(net.wrist_backbone, net.wrist_proj, wrist_img_1)

        # force (manual with hidden)
        x = wrench_1.transpose(1, 2)               # [B,6,1]
        x = net.force_enc.cconv(x)                 # [B,conv,1]
        x = x.transpose(1, 2).contiguous()         # [B,1,conv]
        y, self.h_force = net.force_enc.gru(x, self.h_force)  # [B,1,H]

        p = net.proprio(tcp_pose_1)                # [B,1,P]
        fused = torch.cat([ext, wrist, y, p], dim=-1)

        if net.use_fusion_gru:
            fused, self.h_fusion = net.fusion_gru(fused, self.h_fusion)

        logits = net.head(fused)                   # [B,1,C]
        return logits
