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


import torch
import torch.nn.functional as F

def loss_fast(pred9, target9, w_soft=None, hard_mask=None, fz_err=None, lambda_rule: float = 0.5):
    """
    pred9:   (B,9)
    target9: (B,9)
    fz_err:  (B,) raw error = fz - fz_target
            >0 => force too small => should PRESS_DOWN => dz_ee should be positive
            <0 => force too large => should LIFT_UP   => dz_ee should be negative
    """
    B = pred9.shape[0]
    if w_soft is None:
        w_soft = pred9.new_ones((B,))
    else:
        w_soft = w_soft.reshape(-1).to(pred9.device)

    if hard_mask is None:
        hard_mask = pred9.new_ones((B,))
    else:
        hard_mask = hard_mask.reshape(-1).to(pred9.device)

    # ---- main regression loss ----
    l1 = (pred9 - target9).abs().mean(dim=1)              # (B,)
    loss_reg = (l1 * w_soft * hard_mask).sum() / (hard_mask.sum() + 1e-6)

    # ---- ✅ iron-law directional loss on dz (EE frame) ----
    loss_rule = pred9.new_tensor(0.0)
    if (fz_err is not None) and (lambda_rule is not None) and (float(lambda_rule) > 0):
        e = fz_err.reshape(-1).to(pred9.device)           # (B,)
        s = torch.sign(e)                                 # desired sign of dz
        valid = (hard_mask > 0.5) & (s.abs() > 0)         # ignore near-zero deadband

        dz = pred9[:, 2]                                  # dz in EE frame
        # want dz * s >= 0; penalize opposite direction
        # hinge: relu(-(dz*s))
        viol = F.relu(-(dz * s))
        if valid.any():
            loss_rule = viol[valid].mean()

    loss = loss_reg + float(lambda_rule) * loss_rule
    stats = {
        "loss_reg": float(loss_reg.detach().cpu()),
        "loss_rule": float(loss_rule.detach().cpu()),
    }
    return loss, stats

