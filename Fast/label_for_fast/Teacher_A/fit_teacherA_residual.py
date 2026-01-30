#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import json
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

from reactive_diffusion_policy.common.replay_buffer import ReplayBuffer


ID9 = np.zeros((9,), dtype=np.float32)
ID9[3:9] = np.array([1,0,0,0,1,0], dtype=np.float32)


def one_hot_phase(p, num=3):
    out = np.zeros((num,), dtype=np.float32)
    if 0 <= int(p) < num:
        out[int(p)] = 1.0
    return out


def build_episode_splits(episode_ends, val_ratio=0.2, seed=0):
    # episode_ends: (E,) cumulative ends
    E = len(episode_ends)
    rng = np.random.RandomState(seed)
    ep_ids = np.arange(E)
    rng.shuffle(ep_ids)
    n_val = max(1, int(round(E * val_ratio)))
    val_eps = set(ep_ids[:n_val].tolist())
    train_eps = set(ep_ids[n_val:].tolist())
    return train_eps, val_eps


def episode_id_of_t(episode_ends, t):
    # searchsorted right
    return int(np.searchsorted(episode_ends, t, side="right"))


class ResidualDataset(Dataset):
    def __init__(self, X, Y):
        self.X = torch.from_numpy(X).float()
        self.Y = torch.from_numpy(Y).float()
    def __len__(self):
        return self.X.shape[0]
    def __getitem__(self, idx):
        return self.X[idx], self.Y[idx]


class MLP(nn.Module):
    def __init__(self, in_dim, out_dim=9, hidden=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden),
            nn.ReLU(inplace=True),
            nn.Linear(hidden, out_dim),
        )
    def forward(self, x):
        return self.net(x)


def mae(a, b):
    return float(np.mean(np.abs(a - b)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zarr_path", type=str, required=True)
    ap.add_argument("--ra_path", type=str, required=True, help="teacherA_fast_label_9d.npy")
    ap.add_argument("--slow_exec_path", type=str, required=True, help="teacherA_slow_exec_9d.npy")
    ap.add_argument("--phase_key", type=str, default="paep_phase")
    ap.add_argument("--contact_key", type=str, default="paep_contact")
    ap.add_argument("--wrench_key", type=str, default="left_robot_tcp_wrench")

    ap.add_argument("--model", type=str, choices=["linear", "mlp"], default="linear")
    ap.add_argument("--hidden", type=int, default=64)
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--batch", type=int, default=2048)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--val_ratio", type=float, default=0.2)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--device", type=str, default="cuda:0" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--save_json", type=str, default="", help="optional: save summary json")
    args = ap.parse_args()

    rA = np.load(args.ra_path).astype(np.float32)          # (T,9)
    slow_exec = np.load(args.slow_exec_path).astype(np.float32)  # (T,9)
    assert rA.shape == slow_exec.shape and rA.shape[1] == 9
    T = rA.shape[0]

    rb = ReplayBuffer.copy_from_path(args.zarr_path, keys=[args.phase_key, args.contact_key, args.wrench_key])
    phase = np.asarray(rb[args.phase_key][:], dtype=np.int64).reshape(-1)
    contact = np.asarray(rb[args.contact_key][:], dtype=np.float32).reshape(-1)
    wrench = np.asarray(rb[args.wrench_key][:, :6], dtype=np.float32)

    assert len(phase) == T and len(contact) == T and wrench.shape[0] == T
    episode_ends = np.asarray(rb.episode_ends[:], dtype=np.int64)

    # ---- build features X, targets Y
    # X = [slow_exec(9), wrench(6), phase_onehot(3), contact(1)] -> 19 dims
    X = np.zeros((T, 19), dtype=np.float32)
    Y = rA.copy()

    for t in range(T):
        X[t, 0:9] = slow_exec[t]
        X[t, 9:15] = wrench[t]
        X[t, 15:18] = one_hot_phase(phase[t], 3)
        X[t, 18] = contact[t]

    # ---- episode split
    train_eps, val_eps = build_episode_splits(episode_ends, val_ratio=args.val_ratio, seed=args.seed)
    train_mask = np.zeros((T,), dtype=bool)
    val_mask = np.zeros((T,), dtype=bool)
    for t in range(T):
        eid = episode_id_of_t(episode_ends, t)
        if eid in val_eps:
            val_mask[t] = True
        else:
            train_mask[t] = True

    X_tr, Y_tr = X[train_mask], Y[train_mask]
    X_va, Y_va = X[val_mask], Y[val_mask]

    # ---- baseline: identity residual
    baseline_pred = np.tile(ID9[None, :], (Y_va.shape[0], 1))
    baseline_mae = mae(baseline_pred, Y_va)
    baseline_dp_mae = mae(baseline_pred[:, :3], Y_va[:, :3])
    baseline_rot6_mae = mae(baseline_pred[:, 3:9], Y_va[:, 3:9])

    print("[BASELINE] Identity residual on VAL:")
    print(f"  MAE(all9)={baseline_mae:.6g} | MAE(dp3)={baseline_dp_mae:.6g} | MAE(rot6)={baseline_rot6_mae:.6g}")

    # ---- standardize X using train stats
    mu = X_tr.mean(axis=0, keepdims=True)
    sd = X_tr.std(axis=0, keepdims=True)
    sd = np.maximum(sd, 1e-6)
    X_trn = (X_tr - mu) / sd
    X_van = (X_va - mu) / sd

    ds_tr = ResidualDataset(X_trn, Y_tr)
    ds_va = ResidualDataset(X_van, Y_va)
    dl_tr = DataLoader(ds_tr, batch_size=args.batch, shuffle=True, drop_last=False)
    dl_va = DataLoader(ds_va, batch_size=args.batch, shuffle=False, drop_last=False)

    in_dim = X_trn.shape[1]
    if args.model == "linear":
        net = nn.Linear(in_dim, 9)
    else:
        net = MLP(in_dim, 9, hidden=args.hidden)

    device = torch.device(args.device)
    net.to(device)

    opt = torch.optim.Adam(net.parameters(), lr=args.lr)
    loss_fn = nn.L1Loss()  # MAE

    best_val = 1e9
    best_state = None

    for ep in range(1, args.epochs + 1):
        net.train()
        tr_loss = 0.0
        n_tr = 0
        for xb, yb in dl_tr:
            xb = xb.to(device)
            yb = yb.to(device)
            pred = net(xb)
            loss = loss_fn(pred, yb)
            opt.zero_grad()
            loss.backward()
            opt.step()
            tr_loss += float(loss.item()) * xb.size(0)
            n_tr += xb.size(0)
        tr_loss /= max(n_tr, 1)

        net.eval()
        va_loss = 0.0
        n_va = 0
        preds = []
        ys = []
        with torch.no_grad():
            for xb, yb in dl_va:
                xb = xb.to(device)
                yb = yb.to(device)
                pred = net(xb)
                loss = loss_fn(pred, yb)
                va_loss += float(loss.item()) * xb.size(0)
                n_va += xb.size(0)
                preds.append(pred.detach().cpu().numpy())
                ys.append(yb.detach().cpu().numpy())
        va_loss /= max(n_va, 1)
        pred_va = np.concatenate(preds, axis=0)
        y_va = np.concatenate(ys, axis=0)

        # component metrics
        va_mae_all = mae(pred_va, y_va)
        va_mae_dp = mae(pred_va[:, :3], y_va[:, :3])
        va_mae_rot6 = mae(pred_va[:, 3:9], y_va[:, 3:9])

        if va_mae_all < best_val:
            best_val = va_mae_all
            best_state = {k: v.detach().cpu().clone() for k, v in net.state_dict().items()}

        print(f"[E{ep:02d}] train_L1={tr_loss:.6g} | val_L1={va_loss:.6g} | val_MAE(all)={va_mae_all:.6g} dp={va_mae_dp:.6g} rot6={va_mae_rot6:.6g}")

    # ---- report improvement vs baseline
    improve = (baseline_mae - best_val) / max(baseline_mae, 1e-12)
    print("\n[RESULT] Best val MAE(all9) =", best_val)
    print(f"[RESULT] Improvement vs identity baseline: {(100.0*improve):.2f}%")

    summary = {
        "model": args.model,
        "in_dim": int(in_dim),
        "epochs": int(args.epochs),
        "baseline_val_mae_all9": float(baseline_mae),
        "best_val_mae_all9": float(best_val),
        "improvement_ratio": float(improve),
        "seed": int(args.seed),
        "val_ratio": float(args.val_ratio),
        "feature_def": "X=[slow_exec(9), wrench(6), phase_onehot(3), contact(1)]",
    }
    if args.save_json:
        with open(args.save_json, "w") as f:
            json.dump(summary, f, indent=2)
        print("[SAVE] summary ->", args.save_json)


if __name__ == "__main__":
    main()
