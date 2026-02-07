import torch
import torch.nn as nn

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