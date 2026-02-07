import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18
from torchvision.models.resnet import ResNet18_Weights

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