# train_paep.py
import os
import time
from typing import Dict

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader
from tqdm import tqdm

from paep_dataset import PAEPFutureDataset, EVENT_NAMES, NUM_EVENTS
from paep_model import PAEPFutureNet


# =========================
# 配置区：你只改这里
# =========================
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr"
SPLIT_JSON = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/paep_split_40_5_5.json"
SAVE_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/paep_future_ckpt/paep_runs_0124"

FORCE_HIST = 48
DELTA = 6
FUTURE_K = 12  # 24Hz * 0.5s window

# Force encoder switch: "gru" or "tcn"
FORCE_ENCODER = "tcn"
FORCE_K = 25  # GRU conv kernel

# TCN params (only used if FORCE_ENCODER="tcn")
TCN_CHANNELS = 256
TCN_KERNEL = 5
TCN_BLOCKS = 4
TCN_DROPOUT = 0.0
TCN_POOL = "mean"  # "mean" or "last"

IMG_PRETRAINED = True
IMG_SIZE = 224

BATCH_SIZE = 128
SAMPLES_PER_EPOCH = 20000
VAL_SAMPLES = 4000
EPOCHS = 20
LR = 3e-4

DEVICE = "cuda"
AMP = True

# transition oversampling (phase transition)
TRANSITION_SAMPLING = True
TRANSITION_PROB = 0.3
TRANSITION_WINDOW = 12

# wiping：phase 不是重点，保留通用性但降低权重
PHASE_LOSS_W = 0.1

FREEZE_VISION_EPOCHS = 10
EARLY_STOP_PATIENCE = 5
LOG_EVERY = 50

# wandb
USE_WANDB = True
WANDB_PROJECT = "PAEP"
WANDB_NAME = "paep_future_tcn_0124"
# =========================


def compute_class_weights(class_counts: Dict[str, int]) -> torch.Tensor:
    counts = np.array([class_counts[n] for n in EVENT_NAMES], dtype=np.float64)
    counts = np.maximum(counts, 1.0)
    inv = 1.0 / counts
    w = inv / inv.sum() * len(inv)
    return torch.tensor(w, dtype=torch.float32)


@torch.no_grad()
def eval_n(model, loader, device, norm_torch, img_size: int, phase_w: torch.Tensor):
    model.eval()
    total_n = 0
    loss_p_sum = 0.0
    loss_c_sum = 0.0
    phase_correct = 0
    contact_correct = 0

    for batch in loader:
        ext = batch["ext_img"].to(device, non_blocking=True)
        wrist = batch["wrist_img"].to(device, non_blocking=True)
        wrench = batch["wrench_hist"].to(device, non_blocking=True)
        pose = batch["pose"].to(device, non_blocking=True)

        y_phase = batch["y_phase"].to(device, non_blocking=True)        # (B,)
        y_contact = batch["y_contact"].to(device, non_blocking=True)    # (B,1)

        logits_p, logit_c = model.forward_logits(
            ext, wrist, wrench, pose, norm=norm_torch, img_size=img_size
        )

        loss_p = F.cross_entropy(logits_p, y_phase, weight=phase_w)
        loss_c = F.binary_cross_entropy_with_logits(logit_c, y_contact)

        pred_p = logits_p.argmax(dim=-1)
        phase_correct += int((pred_p == y_phase).sum().item())

        pred_c = (torch.sigmoid(logit_c) >= 0.5).to(torch.float32)
        contact_correct += int((pred_c == y_contact).sum().item())

        B = int(y_phase.shape[0])
        total_n += B
        loss_p_sum += float(loss_p.item()) * B
        loss_c_sum += float(loss_c.item()) * B

    return {
        "val/loss_phase": loss_p_sum / max(total_n, 1),
        "val/loss_contact": loss_c_sum / max(total_n, 1),
        "val/acc_phase": phase_correct / max(total_n, 1),
        "val/acc_contact": contact_correct / max(total_n, 1),
        "val/n": total_n,
    }


def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    device = torch.device(DEVICE)

    # datasets
    train_ds = PAEPFutureDataset(
        ZARR_PATH, SPLIT_JSON, "train",
        force_hist=FORCE_HIST,
        delta=DELTA,
        future_k=FUTURE_K,
        seed=0,
        transition_sampling=TRANSITION_SAMPLING,
        transition_prob=TRANSITION_PROB,
        transition_window=TRANSITION_WINDOW,
        compute_norm=True,
        norm_samples=20000,
    )
    val_ds = PAEPFutureDataset(
        ZARR_PATH, SPLIT_JSON, "val",
        force_hist=FORCE_HIST,
        delta=DELTA,
        future_k=FUTURE_K,
        seed=0,
        transition_sampling=False,
        compute_norm=False,
        provided_norm=train_ds.norm,
    )

    print("Train group:", train_ds.group_name)
    print("Train phase counts:", train_ds.class_counts)
    print("Train contact counts:", train_ds.contact_counts)
    print("Val phase counts:", val_ds.class_counts)
    print("Val contact counts:", val_ds.contact_counts)
    print(f"force_hist={FORCE_HIST}, delta={DELTA}, future_k={FUTURE_K}, force_encoder={FORCE_ENCODER}")

    if TRANSITION_SAMPLING:
        print(f"transition_sampling=ON prob={TRANSITION_PROB} window={TRANSITION_WINDOW}")

    # loaders (dataset is effectively infinite; steps_per_epoch controls)
    train_loader = DataLoader(
        train_ds,
        batch_size=BATCH_SIZE,
        shuffle=False,
        num_workers=0,
        pin_memory=False,
        drop_last=True,
    )

    class _TakeN(torch.utils.data.Dataset):
        def __init__(self, base, n): self.base, self.n = base, n
        def __len__(self): return self.n
        def __getitem__(self, i): return self.base[i]

    val_loader = DataLoader(
        _TakeN(val_ds, VAL_SAMPLES),
        batch_size=min(BATCH_SIZE, 512),
        shuffle=False,
        num_workers=0,
        pin_memory=False,
    )

    # model
    model = PAEPFutureNet(
        num_events=NUM_EVENTS,
        img_pretrained=IMG_PRETRAINED,
        img_feat_dim=256,
        force_encoder=FORCE_ENCODER,
        force_k=FORCE_K,
        force_feat_dim=256,
        tcn_channels=TCN_CHANNELS,
        tcn_kernel=TCN_KERNEL,
        tcn_blocks=TCN_BLOCKS,
        tcn_dropout=TCN_DROPOUT,
        tcn_pool=TCN_POOL,
    ).to(device)

    # norm tensors
    norm_torch = train_ds.norm.to_torch(device=device)

    # class weights for phase
    phase_w = compute_class_weights(train_ds.class_counts).to(device)

    # wandb
    run = None
    if USE_WANDB:
        import wandb
        run = wandb.init(project=WANDB_PROJECT, name=WANDB_NAME)
        run.config.update({
            "ZARR_PATH": ZARR_PATH, "SPLIT_JSON": SPLIT_JSON, "SAVE_DIR": SAVE_DIR,
            "FORCE_HIST": FORCE_HIST, "DELTA": DELTA, "FUTURE_K": FUTURE_K,
            "FORCE_ENCODER": FORCE_ENCODER, "FORCE_K": FORCE_K,
            "TCN_CHANNELS": TCN_CHANNELS, "TCN_KERNEL": TCN_KERNEL, "TCN_BLOCKS": TCN_BLOCKS,
            "TCN_DROPOUT": TCN_DROPOUT, "TCN_POOL": TCN_POOL,
            "BATCH_SIZE": BATCH_SIZE, "SAMPLES_PER_EPOCH": SAMPLES_PER_EPOCH, "VAL_SAMPLES": VAL_SAMPLES,
            "EPOCHS": EPOCHS, "LR": LR, "AMP": AMP, "IMG_PRETRAINED": IMG_PRETRAINED, "IMG_SIZE": IMG_SIZE,
            "TRANSITION_SAMPLING": TRANSITION_SAMPLING, "TRANSITION_PROB": TRANSITION_PROB, "TRANSITION_WINDOW": TRANSITION_WINDOW,
            "PHASE_LOSS_W": PHASE_LOSS_W,
        })

    def make_optimizer(vision_lr_mult=0.1):
        vision_params = []
        other_params = []
        for n, p in model.named_parameters():
            if not p.requires_grad:
                continue
            if n.startswith("ext_enc.backbone") or n.startswith("wrist_enc.backbone"):
                vision_params.append(p)
            else:
                other_params.append(p)
        groups = [{"params": other_params, "lr": LR}]
        if len(vision_params) > 0:
            groups.append({"params": vision_params, "lr": LR * vision_lr_mult})
        return torch.optim.AdamW(groups, weight_decay=1e-4)

    if FREEZE_VISION_EPOCHS > 0:
        model.set_vision_trainable(False)
    optimizer = make_optimizer(vision_lr_mult=0.1)

    scaler = torch.amp.GradScaler("cuda", enabled=AMP)
    autocast = torch.amp.autocast

    steps_per_epoch = int(np.ceil(SAMPLES_PER_EPOCH / BATCH_SIZE))

    best_score = -1.0
    best_epoch = -1
    patience_left = EARLY_STOP_PATIENCE
    global_step = 0

    for epoch in range(EPOCHS):
        if FREEZE_VISION_EPOCHS > 0 and epoch == FREEZE_VISION_EPOCHS:
            model.set_vision_trainable(True)
            optimizer = make_optimizer(vision_lr_mult=0.05)
            print(f"[info] Unfroze vision at epoch {epoch}, recreated optimizer.")

        model.train()
        t0 = time.time()

        n_total = 0
        loss_p_sum = 0.0
        loss_c_sum = 0.0
        phase_correct = 0
        contact_correct = 0

        pbar = tqdm(enumerate(train_loader), total=steps_per_epoch, desc=f"Epoch {epoch:03d}/{EPOCHS-1:03d}")
        for it, batch in pbar:
            if it >= steps_per_epoch:
                break

            ext = batch["ext_img"].to(device, non_blocking=True)
            wrist = batch["wrist_img"].to(device, non_blocking=True)
            wrench = batch["wrench_hist"].to(device, non_blocking=True)
            pose = batch["pose"].to(device, non_blocking=True)

            y_phase = batch["y_phase"].to(device, non_blocking=True)       # (B,)
            y_contact = batch["y_contact"].to(device, non_blocking=True)   # (B,1)

            optimizer.zero_grad(set_to_none=True)

            with autocast(device_type="cuda", enabled=AMP):
                logits_p, logit_c = model.forward_logits(
                    ext, wrist, wrench, pose, norm=norm_torch, img_size=IMG_SIZE
                )
                loss_p = F.cross_entropy(logits_p, y_phase, weight=phase_w)
                loss_c = F.binary_cross_entropy_with_logits(logit_c, y_contact)
                loss = loss_c + float(PHASE_LOSS_W) * loss_p

            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()

            B = int(y_phase.shape[0])
            n_total += B
            loss_p_sum += float(loss_p.item()) * B
            loss_c_sum += float(loss_c.item()) * B

            pred_p = logits_p.argmax(dim=-1)
            phase_correct += int((pred_p == y_phase).sum().item())

            pred_c = (torch.sigmoid(logit_c) >= 0.5).to(torch.float32)
            contact_correct += int((pred_c == y_contact).sum().item())

            if (global_step % LOG_EVERY) == 0:
                avg_lp = loss_p_sum / max(n_total, 1)
                avg_lc = loss_c_sum / max(n_total, 1)
                acc_p = phase_correct / max(n_total, 1)
                acc_c = contact_correct / max(n_total, 1)

                mem_gb = torch.cuda.memory_allocated(device) / (1024**3) if device.type == "cuda" else 0.0
                peak_gb = torch.cuda.max_memory_allocated(device) / (1024**3) if device.type == "cuda" else 0.0

                pbar.set_postfix(lp=f"{avg_lp:.3f}", lc=f"{avg_lc:.3f}", ap=f"{acc_p:.3f}", ac=f"{acc_c:.3f}",
                                 mem=f"{mem_gb:.1f}G", peak=f"{peak_gb:.1f}G")

                if run is not None:
                    run.log({
                        "train/loss_phase": avg_lp,
                        "train/loss_contact": avg_lc,
                        "train/acc_phase": acc_p,
                        "train/acc_contact": acc_c,
                        "train/lr": optimizer.param_groups[0]["lr"],
                        "gpu/mem_alloc_gb": mem_gb,
                        "gpu/mem_peak_gb": peak_gb,
                        "iter": global_step,
                        "epoch": epoch,
                    }, step=global_step)

            global_step += 1

        train_lp = loss_p_sum / max(n_total, 1)
        train_lc = loss_c_sum / max(n_total, 1)
        train_ap = phase_correct / max(n_total, 1)
        train_ac = contact_correct / max(n_total, 1)

        val = eval_n(model, val_loader, device, norm_torch, img_size=IMG_SIZE, phase_w=phase_w)
        epoch_time = time.time() - t0

        print(
            f"[epoch {epoch:03d}] "
            f"train_lp={train_lp:.4f} train_lc={train_lc:.4f} train_ap={train_ap:.4f} train_ac={train_ac:.4f} | "
            f"val_lp={val['val/loss_phase']:.4f} val_lc={val['val/loss_contact']:.4f} "
            f"val_ap={val['val/acc_phase']:.4f} val_ac={val['val/acc_contact']:.4f} "
            f"time={epoch_time:.1f}s"
        )

        if run is not None:
            run.log({
                "epoch_train/loss_phase": train_lp,
                "epoch_train/loss_contact": train_lc,
                "epoch_train/acc_phase": train_ap,
                "epoch_train/acc_contact": train_ac,
                "epoch_val/loss_phase": val["val/loss_phase"],
                "epoch_val/loss_contact": val["val/loss_contact"],
                "epoch_val/acc_phase": val["val/acc_phase"],
                "epoch_val/acc_contact": val["val/acc_contact"],
                "epoch_time_sec": epoch_time,
                "epoch": epoch,
            }, step=global_step)

        # best score: contact 为主，phase 轻权重
        score = float(val["val/acc_contact"]) + float(PHASE_LOSS_W) * float(val["val/acc_phase"])
        improved = score > best_score + 1e-6
        if improved:
            best_score = score
            best_epoch = epoch
            patience_left = EARLY_STOP_PATIENCE
        else:
            patience_left -= 1

        ckpt = {
            "model": model.state_dict(),
            "cfg": {
                "force_hist": FORCE_HIST,
                "delta": DELTA,
                "future_k": FUTURE_K,
                "force_encoder": FORCE_ENCODER,
                "force_k": FORCE_K,
                "tcn": {
                    "channels": TCN_CHANNELS,
                    "kernel": TCN_KERNEL,
                    "blocks": TCN_BLOCKS,
                    "dropout": TCN_DROPOUT,
                    "pool": TCN_POOL,
                },
                "img_size": IMG_SIZE,
                "phase_loss_w": PHASE_LOSS_W,
                "phase_names": EVENT_NAMES,
            },
            "norm": {
                "wrench_mean": train_ds.norm.wrench_mean,
                "wrench_std": train_ds.norm.wrench_std,
                "pose_mean": train_ds.norm.pose_mean,
                "pose_std": train_ds.norm.pose_std,
            },
            "val": {
                "acc_phase": val["val/acc_phase"],
                "acc_contact": val["val/acc_contact"],
                "score": score,
            },
        }

        torch.save(ckpt, os.path.join(SAVE_DIR, f"paep_future_ep{epoch:03d}.pt"))
        if improved:
            torch.save(ckpt, os.path.join(SAVE_DIR, "best.pt"))

        if patience_left <= 0:
            print(f"[early-stop] No improvement for {EARLY_STOP_PATIENCE} epochs. Stop at epoch {epoch}.")
            break

    if run is not None:
        run.summary["best_score"] = best_score
        run.summary["best_epoch"] = best_epoch
        run.finish()

    print("Done. best_score:", best_score, "best_epoch:", best_epoch)


if __name__ == "__main__":
    main()
