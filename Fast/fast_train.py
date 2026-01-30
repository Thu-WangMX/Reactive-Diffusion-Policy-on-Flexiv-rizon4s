#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
fast_train.py (v2.1) - wandb online by default (best-only artifact)

Trains FastResidualPolicy on FastResidualDataset.
Logs:
- train/loss, train/l_dp, train/l_rot
- val mae (all / active-only): dp3, rot6, all9
- data stats: active_ratio, wrench_clip_ratio
- optional gate entropy if NUM_SUBSPACE>1

Key behavior:
- ✅ No argparse
- ✅ wandb mode defaults to "online"
- ✅ Upload ONLY best checkpoint as a W&B Artifact (no wandb.save -> no symlink warnings)
- ✅ Still saves local ckpt_best.pt; optional local periodic ckpts can be enabled
"""

import os
import json
import time
import numpy as np
import torch
from torch.utils.data import DataLoader, WeightedRandomSampler

import wandb  # default online

from fast_dataset import FastResidualDataset, HIST, PHASE_NAMES, ACTIVE_PHASE_NAMES
from fast_model import FastResidualPolicy, loss_fast, NUM_SUBSPACE


# =========================
# Config (edit here)
# =========================
OUT_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/train_fast_wiping_v2_run1"
os.makedirs(OUT_DIR, exist_ok=True)

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
SEED = 0

EPOCHS = 50
BATCH = 2048
LR = 1e-3
WEIGHT_DECAY = 0.0

# loss weights
W_DP = 1.0
W_ROT = 0.0     # wiping 先关 rot，接口保留

# sample weighting (soft)
BASE_WEIGHT = 0.1
ACTIVE_BOOST = 1.0

# sampler: upweight by soft active_prob
SAMPLER_ALPHA = 5.0     # weights = 1 + alpha * active_prob

NUM_WORKERS_TRAIN = 4
NUM_WORKERS_VAL = 2

# local checkpointing (no wandb warnings)
SAVE_LOCAL_EVERY = 0     # 0 disables; set e.g. 10 to save ckpt_epXXX.pt locally

# ---- wandb (online default) ----
WANDB_ENABLED = True
WANDB_PROJECT = "fast_policy"
WANDB_NAME = None
WANDB_TAGS = ["fast", "residual", "paep-conditioned"]
WANDB_MODE = "online"     # ✅ default online
WANDB_ENTITY = None       # use your default


def set_seed(seed: int):
    import random
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


def grad_global_norm(parameters, norm_type: float = 2.0) -> float:
    params = [p for p in parameters if (p is not None and p.grad is not None)]
    if len(params) == 0:
        return 0.0
    device = params[0].grad.device
    total = torch.zeros((), device=device)
    for p in params:
        total += p.grad.data.norm(norm_type) ** norm_type
    total = total ** (1.0 / norm_type)
    return float(total.detach().cpu())


@torch.no_grad()
def eval_epoch(model, loader, device):
    model.eval()

    sum_abs_all9 = 0.0
    sum_abs_dp3 = 0.0
    sum_abs_rot6 = 0.0
    n = 0

    sum_abs_all9_a = 0.0
    sum_abs_dp3_a = 0.0
    sum_abs_rot6_a = 0.0
    n_a = 0

    gate_entropy_sum = 0.0
    gate_n = 0

    for base_h, wrench_h, paep_h, y, w_soft, m_hard, _t in loader:
        base_h = base_h.to(device)
        wrench_h = wrench_h.to(device)
        paep_h = paep_h.to(device)
        y = y.to(device)
        m_hard = m_hard.to(device)

        pred, aux = model(base_h, wrench_h, paep_h)
        abs9 = (pred - y).abs()                    # (B,9)
        abs_dp = abs9[:, :3].mean(dim=1)          # (B,)
        abs_rot = abs9[:, 3:9].mean(dim=1)        # (B,)

        bs = int(y.size(0))
        sum_abs_all9 += float(abs9.mean().detach().cpu()) * bs
        sum_abs_dp3 += float(abs_dp.mean().detach().cpu()) * bs
        sum_abs_rot6 += float(abs_rot.mean().detach().cpu()) * bs
        n += bs

        active_mask = (m_hard > 0.5)
        if active_mask.any():
            a9 = abs9[active_mask]
            adp = abs_dp[active_mask]
            arot = abs_rot[active_mask]
            bs2 = int(a9.size(0))
            sum_abs_all9_a += float(a9.mean().detach().cpu()) * bs2
            sum_abs_dp3_a += float(adp.mean().detach().cpu()) * bs2
            sum_abs_rot6_a += float(arot.mean().detach().cpu()) * bs2
            n_a += bs2

        if "gate" in aux:
            g = aux["gate"]  # (B,M)
            ent = (-g * (g.clamp_min(1e-9)).log()).sum(dim=1).mean()
            gate_entropy_sum += float(ent.detach().cpu()) * bs
            gate_n += bs

    out = {
        "mae_all9": sum_abs_all9 / max(1, n),
        "mae_dp3": sum_abs_dp3 / max(1, n),
        "mae_rot6": sum_abs_rot6 / max(1, n),

        "mae_all9_active": sum_abs_all9_a / max(1, n_a),
        "mae_dp3_active": sum_abs_dp3_a / max(1, n_a),
        "mae_rot6_active": sum_abs_rot6_a / max(1, n_a),

        "n": int(n),
        "n_active": int(n_a),
    }
    if gate_n > 0:
        out["gate_entropy"] = gate_entropy_sum / gate_n
    return out


def main():
    set_seed(SEED)

    ds_tr = FastResidualDataset(split="train")
    ds_va = FastResidualDataset(split="val")

    d_base = ds_tr.base_dim
    d_paep = ds_tr.paep_dim

    active_ratio_tr = float(np.mean(ds_tr.hard_active[ds_tr.indices] > 0.5))
    active_ratio_va = float(np.mean(ds_va.hard_active[ds_va.indices] > 0.5))

    # sampler weights (soft)
    w_soft = ds_tr.active_prob[ds_tr.indices].astype(np.float32)
    samp_w = 1.0 + SAMPLER_ALPHA * w_soft
    sampler = WeightedRandomSampler(
        weights=torch.from_numpy(samp_w),
        num_samples=len(ds_tr),
        replacement=True
    )

    dl_tr = DataLoader(
        ds_tr, batch_size=BATCH, sampler=sampler,
        num_workers=NUM_WORKERS_TRAIN, pin_memory=True, drop_last=True
    )
    dl_va = DataLoader(
        ds_va, batch_size=BATCH, shuffle=False,
        num_workers=NUM_WORKERS_VAL, pin_memory=True, drop_last=False
    )

    model = FastResidualPolicy(d_base=d_base, d_paep=d_paep).to(DEVICE)
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=WEIGHT_DECAY)

    meta = {
        "phase_names": PHASE_NAMES,
        "active_phase_names": ACTIVE_PHASE_NAMES,
        "HIST": HIST,
        "d_base": d_base,
        "d_paep": d_paep,
        "epochs": EPOCHS,
        "batch": BATCH,
        "lr": LR,
        "weight_decay": WEIGHT_DECAY,
        "W_DP": W_DP,
        "W_ROT": W_ROT,
        "BASE_WEIGHT": BASE_WEIGHT,
        "ACTIVE_BOOST": ACTIVE_BOOST,
        "SAMPLER_ALPHA": SAMPLER_ALPHA,
        "active_ratio_train": active_ratio_tr,
        "active_ratio_val": active_ratio_va,
        "wrench_norm": {"mean": ds_tr.wrench_mean.tolist(), "std": ds_tr.wrench_std.tolist()},
        "wrench_clip_ratio_global": float(ds_tr.wrench_clip_ratio),
        "num_subspace": int(NUM_SUBSPACE),
        "out_dir": OUT_DIR,
        "device": DEVICE,
    }
    with open(os.path.join(OUT_DIR, "train_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    # ---- wandb init (online default) ----
    run = None
    if WANDB_ENABLED:
        run_name = WANDB_NAME if WANDB_NAME is not None else os.path.basename(OUT_DIR.rstrip("/"))
        run = wandb.init(
            project=WANDB_PROJECT,
            name=run_name,
            entity=WANDB_ENTITY,
            mode=WANDB_MODE,   # ✅ online
            dir=OUT_DIR,
            config=meta,
            tags=WANDB_TAGS,
        )

    best = 1e9
    best_ep = -1
    global_step = 0

    for ep in range(1, EPOCHS + 1):
        t0 = time.time()
        model.train()

        sum_loss = 0.0
        sum_ldp = 0.0
        sum_lrot = 0.0
        sum_wmean = 0.0
        nb = 0
        gn_last = 0.0

        for base_h, wrench_h, paep_h, y, w_soft_b, m_hard, _t in dl_tr:
            base_h = base_h.to(DEVICE, non_blocking=True)
            wrench_h = wrench_h.to(DEVICE, non_blocking=True)
            paep_h = paep_h.to(DEVICE, non_blocking=True)
            y = y.to(DEVICE, non_blocking=True)
            w_soft_b = w_soft_b.to(DEVICE, non_blocking=True)

            pred, aux = model(base_h, wrench_h, paep_h)
            loss, ld = loss_fast(
                delta_pred=pred,
                delta_gt=y,
                sample_weight=w_soft_b,
                w_dp=W_DP,
                w_rot=W_ROT,
                base_weight=BASE_WEIGHT,
                active_boost=ACTIVE_BOOST,
            )

            opt.zero_grad(set_to_none=True)
            loss.backward()
            gn_last = grad_global_norm(model.parameters())
            opt.step()

            sum_loss += float(loss.detach().cpu())
            sum_ldp += float(ld["l_dp"])
            sum_lrot += float(ld["l_rot"])
            sum_wmean += float(ld["w_mean"])
            nb += 1
            global_step += 1

        train_loss = sum_loss / max(1, nb)
        train_ldp = sum_ldp / max(1, nb)
        train_lrot = sum_lrot / max(1, nb)
        w_mean = sum_wmean / max(1, nb)

        val = eval_epoch(model, dl_va, DEVICE)
        dt = time.time() - t0
        lr_now = float(opt.param_groups[0]["lr"])

        print(f"[EP {ep:03d}] loss={train_loss:.6f} (dp={train_ldp:.6f}, rot={train_lrot:.6f}) "
              f"| val_dp={val['mae_dp3']:.6f} active_dp={val['mae_dp3_active']:.6f} "
              f"| gn={gn_last:.3f} lr={lr_now:.2e} dt={dt:.1f}s")

        # ---- wandb epoch log ----
        if run is not None:
            log = {
                "epoch": ep,
                "time/epoch_sec": dt,
                "train/loss": train_loss,
                "train/l_dp": train_ldp,
                "train/l_rot": train_lrot,
                "train/grad_norm": gn_last,
                "train/lr": lr_now,
                "train/weight_mean": w_mean,

                "data/active_ratio_train": meta["active_ratio_train"],
                "data/active_ratio_val": meta["active_ratio_val"],
                "data/wrench_clip_ratio_global": meta["wrench_clip_ratio_global"],

                "val/mae_all9": val["mae_all9"],
                "val/mae_dp3": val["mae_dp3"],
                "val/mae_rot6": val["mae_rot6"],
                "val_active/mae_all9": val["mae_all9_active"],
                "val_active/mae_dp3": val["mae_dp3_active"],
                "val_active/mae_rot6": val["mae_rot6_active"],
                "val/n": val["n"],
                "val/n_active": val["n_active"],

                "best/active_dp3": best,
                "best/epoch": best_ep,
            }
            if "gate_entropy" in val:
                log["val/gate_entropy"] = val["gate_entropy"]
            wandb.log(log, step=global_step)

        # ---- best ckpt (use active dp3) ----
        if val["mae_dp3_active"] < best:
            best = float(val["mae_dp3_active"])
            best_ep = int(ep)

            ckpt_path = os.path.join(OUT_DIR, "ckpt_best.pt")
            torch.save(
                {"model": model.state_dict(), "meta": meta, "best": best, "best_ep": best_ep},
                ckpt_path
            )

            if run is not None:
                wandb.log(
                    {"event/new_best": 1, "best/active_dp3": best, "best/epoch": best_ep},
                    step=global_step
                )
                try:
                    art = wandb.Artifact(
                        name=f"{run.name}-ckpt_best",
                        type="model",
                        metadata={"best_active_dp3": float(best), "best_epoch": int(best_ep)},
                    )
                    art.add_file(ckpt_path)
                    run.log_artifact(art)
                except Exception as e:
                    print("[WARN] wandb artifact upload failed:", repr(e))

        # ---- optional local periodic ckpt (no wandb) ----
        if SAVE_LOCAL_EVERY and (ep % int(SAVE_LOCAL_EVERY) == 0):
            ckpt_path = os.path.join(OUT_DIR, f"ckpt_ep{ep:03d}.pt")
            torch.save({"model": model.state_dict(), "meta": meta, "ep": int(ep)}, ckpt_path)

    print("[OK] done. best active_dp3 =", best, "at epoch", best_ep)
    if run is not None:
        run.finish()


if __name__ == "__main__":
    main()
