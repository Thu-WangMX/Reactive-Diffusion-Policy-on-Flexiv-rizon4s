#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
fast_model.py (v2)

GRU-based residual policy with pluggable ForceEncoder:
- force_encoder = "raw": use wrench_hist directly (normalized+clipped already in dataset)
- force_encoder = "tcn": 1D conv over time to produce per-timestep force features

Also reserves multi-subspace interface:
- NUM_SUBSPACE = 1 by default (single head)
- if NUM_SUBSPACE > 1: gate + multi-head residual, weighted sum
"""

import torch
import torch.nn as nn
import torch.nn.functional as F


# =========================
# Config (edit here)
# =========================
FORCE_ENCODER = "raw"   # "raw" or "tcn"
TCN_CHANNELS = 64
TCN_LAYERS = 3
TCN_KERNEL = 5
TCN_DROPOUT = 0.0

GRU_HIDDEN = 128
GRU_LAYERS = 1
MLP_HIDDEN = 128
DROPOUT = 0.0

OUT_DIM = 9

# multi-subspace reserve
NUM_SUBSPACE = 1              # set >1 later to enable gating
GATE_TEMPERATURE = 1.0        # softmax temperature


class ForceTCN(nn.Module):
    """
    Simple temporal conv that preserves length:
      in:  (B, H, 6)
      out: (B, H, C)
    """
    def __init__(self, in_ch=6, channels=64, layers=3, kernel=5, dropout=0.0):
        super().__init__()
        assert kernel % 2 == 1, "Use odd kernel to preserve length with symmetric padding"
        blocks = []
        ch_in = in_ch
        for _ in range(layers):
            pad = kernel // 2
            blocks.append(nn.Conv1d(ch_in, channels, kernel_size=kernel, padding=pad))
            blocks.append(nn.ReLU(inplace=True))
            if dropout > 0:
                blocks.append(nn.Dropout(dropout))
            ch_in = channels
        self.net = nn.Sequential(*blocks)

    def forward(self, wrench_hist: torch.Tensor) -> torch.Tensor:
        # wrench_hist: (B,H,6) -> (B,6,H)
        x = wrench_hist.transpose(1, 2).contiguous()
        y = self.net(x)  # (B,C,H)
        return y.transpose(1, 2).contiguous()  # (B,H,C)


class FastResidualPolicy(nn.Module):
    """
    Forward:
      base_hist:   (B,H,D_base)
      wrench_hist: (B,H,6)
      paep_hist:   (B,H,D_paep=1+K)
    Return:
      delta_pred:  (B,9)
      aux: dict with gate probs if enabled
    """
    def __init__(self, d_base: int, d_paep: int):
        super().__init__()
        self.d_base = int(d_base)
        self.d_paep = int(d_paep)

        if FORCE_ENCODER == "tcn":
            self.force_enc = ForceTCN(
                in_ch=6,
                channels=TCN_CHANNELS,
                layers=TCN_LAYERS,
                kernel=TCN_KERNEL,
                dropout=TCN_DROPOUT,
            )
            d_force = TCN_CHANNELS
        elif FORCE_ENCODER == "raw":
            self.force_enc = None
            d_force = 6
        else:
            raise ValueError(f"Unknown FORCE_ENCODER={FORCE_ENCODER}")

        d_in = self.d_base + self.d_paep + d_force

        self.gru = nn.GRU(
            input_size=d_in,
            hidden_size=GRU_HIDDEN,
            num_layers=GRU_LAYERS,
            batch_first=True,
            dropout=0.0 if GRU_LAYERS == 1 else DROPOUT
        )

        self.mlp = nn.Sequential(
            nn.Linear(GRU_HIDDEN, MLP_HIDDEN),
            nn.ReLU(inplace=True),
            nn.Dropout(DROPOUT),
        )

        # single head
        self.head = nn.Linear(MLP_HIDDEN, OUT_DIM)

        # multi-subspace reserve
        self.num_subspace = int(NUM_SUBSPACE)
        if self.num_subspace > 1:
            self.gate = nn.Linear(MLP_HIDDEN, self.num_subspace)
            self.heads = nn.ModuleList([nn.Linear(MLP_HIDDEN, OUT_DIM) for _ in range(self.num_subspace)])
        else:
            self.gate = None
            self.heads = None

    def forward(self, base_hist: torch.Tensor, wrench_hist: torch.Tensor, paep_hist: torch.Tensor):
        # encode force
        if self.force_enc is None:
            f = wrench_hist
        else:
            f = self.force_enc(wrench_hist)

        x = torch.cat([base_hist, f, paep_hist], dim=-1)  # (B,H,D)

        h_seq, h_last = self.gru(x)   # h_last: (L,B,Hid)
        h = h_last[-1]                # (B,Hid)
        z = self.mlp(h)

        aux = {}

        if self.num_subspace <= 1:
            y = self.head(z)
            return y, aux

        # multi-head gating
        logits = self.gate(z) / float(GATE_TEMPERATURE)
        gate = F.softmax(logits, dim=-1)  # (B,M)
        aux["gate"] = gate

        ys = []
        for i in range(self.num_subspace):
            ys.append(self.heads[i](z))  # (B,9)
        Y = torch.stack(ys, dim=1)       # (B,M,9)

        y = torch.sum(gate.unsqueeze(-1) * Y, dim=1)  # (B,9)
        return y, aux


def loss_fast(delta_pred: torch.Tensor,
              delta_gt: torch.Tensor,
              sample_weight: torch.Tensor,
              w_dp: float = 1.0,
              w_rot: float = 0.0,
              base_weight: float = 0.1,
              active_boost: float = 1.0):
    """
    Weighted L1 loss:
      sample_weight in [0,1] from PAEP soft active prob.
      final weight = base_weight + active_boost * sample_weight
    """
    assert delta_pred.shape == delta_gt.shape and delta_pred.shape[-1] == 9
    w = (base_weight + active_boost * sample_weight).view(-1, 1)  # (B,1)

    dp = torch.abs(delta_pred[:, :3] - delta_gt[:, :3]).mean(dim=1, keepdim=True)   # (B,1)
    rot = torch.abs(delta_pred[:, 3:9] - delta_gt[:, 3:9]).mean(dim=1, keepdim=True)

    l_dp = (w * dp).mean()
    l_rot = (w * rot).mean()

    loss = w_dp * l_dp + w_rot * l_rot

    log = {
        "l_dp": float(l_dp.detach().cpu()),
        "l_rot": float(l_rot.detach().cpu()),
        "w_mean": float(w.mean().detach().cpu()),
        "w_max": float(w.max().detach().cpu()),
    }
    return loss, log
