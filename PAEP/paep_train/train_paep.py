# PAEP/paep_train/train_paep.py
import argparse
import os
import time
from typing import Dict

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader
from tqdm import tqdm

from paep_dataset import PAEPFutureDataset, EVENT_NAMES, NUM_EVENTS, NormStats
from paep_model import PAEPFutureNet


def compute_class_weights(class_counts: Dict[str, int]) -> torch.Tensor:
    # inverse frequency with smoothing
    counts = np.array([class_counts[n] for n in EVENT_NAMES], dtype=np.float64)
    counts = np.maximum(counts, 1.0)
    inv = 1.0 / counts
    w = inv / inv.sum() * len(inv)
    return torch.tensor(w, dtype=torch.float32)


def confusion_matrix_np(y_true: np.ndarray, y_pred: np.ndarray, C: int) -> np.ndarray:
    cm = np.zeros((C, C), dtype=np.int64)
    for t, p in zip(y_true, y_pred):
        cm[int(t), int(p)] += 1
    return cm


def per_class_prf(cm: np.ndarray):
    out = {}
    for c in range(cm.shape[0]):
        tp = cm[c, c]
        fp = cm[:, c].sum() - tp
        fn = cm[c, :].sum() - tp
        prec = float(tp / (tp + fp + 1e-9))
        rec = float(tp / (tp + fn + 1e-9))
        f1 = float((2 * prec * rec) / (prec + rec + 1e-9))
        out[c] = {"precision": prec, "recall": rec, "f1": f1, "support": float(cm[c, :].sum())}
    macro_f1 = float(np.mean([out[c]["f1"] for c in out.keys()]))
    return out, macro_f1


@torch.no_grad()
def eval_n(model, loader, device, norm_torch, img_size: int):
    model.eval()
    total_loss, total_n = 0.0, 0
    ys, ps = [], []
    for batch in loader:
        ext = batch["ext_img"].to(device, non_blocking=True)
        wrist = batch["wrist_img"].to(device, non_blocking=True)
        wrench = batch["wrench_hist"].to(device, non_blocking=True)
        pose = batch["pose"].to(device, non_blocking=True)
        y = batch["y"].to(device, non_blocking=True)

        logits = model(ext, wrist, wrench, pose, norm=norm_torch, img_size=img_size)
        loss = F.cross_entropy(logits, y)
        pred = logits.argmax(dim=-1)

        total_loss += float(loss.item()) * y.shape[0]
        total_n += int(y.shape[0])

        ys.append(y.cpu().numpy())
        ps.append(pred.cpu().numpy())

    y_true = np.concatenate(ys, axis=0)
    y_pred = np.concatenate(ps, axis=0)
    acc = float((y_true == y_pred).mean())
    cm = confusion_matrix_np(y_true, y_pred, NUM_EVENTS)
    prf, mf1 = per_class_prf(cm)
    return total_loss / max(total_n, 1), acc, mf1, cm, prf


def main():
    ap = argparse.ArgumentParser()

    ap.add_argument("--zarr_path", required=True)
    ap.add_argument("--split_json", required=True)
    ap.add_argument("--save_dir", required=True)

    ap.add_argument("--force_hist", type=int, default=48)
    ap.add_argument("--delta", type=int, default=6)
    ap.add_argument("--force_k", type=int, default=25)

    ap.add_argument("--batch_size", type=int, default=128)
    ap.add_argument("--samples_per_epoch", type=int, default=20000)
    ap.add_argument("--val_samples", type=int, default=4000)
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--lr", type=float, default=3e-4)

    ap.add_argument("--num_workers", type=int, default=0)
    ap.add_argument("--pin_memory", action="store_true")
    ap.add_argument("--device", default="cuda")

    ap.add_argument("--amp", action="store_true")
    ap.add_argument("--img_pretrained", action="store_true")
    ap.add_argument("--img_size", type=int, default=224)

    # improvements
    ap.add_argument("--transition_sampling", action="store_true")
    ap.add_argument("--transition_prob", type=float, default=0.7)
    ap.add_argument("--transition_window", type=int, default=12)

    ap.add_argument("--freeze_vision_epochs", type=int, default=10)  # key
    ap.add_argument("--early_stop_patience", type=int, default=5)

    ap.add_argument("--log_every", type=int, default=50)

    ap.add_argument("--use_wandb", action="store_true")
    ap.add_argument("--wandb_project", default="PAEP")
    ap.add_argument("--wandb_name", default=None)

    args = ap.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)
    device = torch.device(args.device)

    # datasets
    train_ds = PAEPFutureDataset(
        args.zarr_path,
        args.split_json,
        "train",
        force_hist=args.force_hist,
        delta=args.delta,
        seed=0,
        transition_sampling=args.transition_sampling,
        transition_prob=args.transition_prob,
        transition_window=args.transition_window,
        compute_norm=True,
        norm_samples=20000,
    )
    val_ds = PAEPFutureDataset(
        args.zarr_path,
        args.split_json,
        "val",
        force_hist=args.force_hist,
        delta=args.delta,
        seed=0,
        transition_sampling=False,
        compute_norm=False,
        provided_norm=train_ds.norm,
    )

    print("Train group:", train_ds.group_name)
    print("Train class counts:", train_ds.class_counts)
    print("Val class counts:", val_ds.class_counts)
    print(f"force_hist={args.force_hist}, delta={args.delta} (predict t+Δ), force_k={args.force_k}")
    if args.transition_sampling:
        print(f"transition_sampling=ON prob={args.transition_prob} window={args.transition_window}")

    # dataloaders: we control steps/epoch by breaking after N steps
    train_loader = DataLoader(
        train_ds,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=args.pin_memory,
        drop_last=True,
    )

    # val fixed-N wrapper
    class _TakeN(torch.utils.data.Dataset):
        def __init__(self, base, n): self.base, self.n = base, n
        def __len__(self): return self.n
        def __getitem__(self, i): return self.base[i]

    val_take = _TakeN(val_ds, args.val_samples)
    val_loader = DataLoader(
        val_take,
        batch_size=min(args.batch_size, 512),
        shuffle=False,
        num_workers=0,
        pin_memory=False,
    )

    # model
    model = PAEPFutureNet(
        num_events=NUM_EVENTS,
        img_pretrained=args.img_pretrained,
        force_k=args.force_k,
    ).to(device)

    # norm tensors
    norm_torch = train_ds.norm.to_torch(device=device)

    # weights
    w = compute_class_weights(train_ds.class_counts).to(device)

    # wandb
    run = None
    if args.use_wandb:
        import wandb
        run = wandb.init(project=args.wandb_project, name=args.wandb_name)
        run.config.update(vars(args))

    def make_optimizer(vision_lr_mult=0.1):
        # separate lr for vision backbone (optional smaller)
        vision_params = []
        other_params = []
        for n, p in model.named_parameters():
            if not p.requires_grad:
                continue
            if n.startswith("ext_enc.backbone") or n.startswith("wrist_enc.backbone"):
                vision_params.append(p)
            else:
                other_params.append(p)
        groups = [{"params": other_params, "lr": args.lr}]
        if len(vision_params) > 0:
            groups.append({"params": vision_params, "lr": args.lr * vision_lr_mult})
        return torch.optim.AdamW(groups, weight_decay=1e-4)

    # start with vision frozen (saves generalization)
    if args.freeze_vision_epochs > 0:
        model.set_vision_trainable(False)
    optimizer = make_optimizer(vision_lr_mult=0.1)

    # AMP
    scaler = torch.amp.GradScaler("cuda", enabled=args.amp)
    autocast = torch.amp.autocast

    steps_per_epoch = int(np.ceil(args.samples_per_epoch / args.batch_size))

    best_val_acc = -1.0
    best_epoch = -1
    best_val_mf1 = -1.0
    patience_left = args.early_stop_patience

    global_step = 0

    for epoch in range(args.epochs):
        # unfreeze vision after N epochs (optional)
        if args.freeze_vision_epochs > 0 and epoch == args.freeze_vision_epochs:
            model.set_vision_trainable(True)
            optimizer = make_optimizer(vision_lr_mult=0.05)  # smaller lr for vision
            print(f"[info] Unfroze vision at epoch {epoch}, recreated optimizer.")

        model.train()
        t0 = time.time()
        loss_sum, correct, total = 0.0, 0, 0

        pbar = tqdm(enumerate(train_loader), total=steps_per_epoch, desc=f"Epoch {epoch:03d}/{args.epochs-1:03d}")
        for it, batch in pbar:
            if it >= steps_per_epoch:
                break

            ext = batch["ext_img"].to(device, non_blocking=True)
            wrist = batch["wrist_img"].to(device, non_blocking=True)
            wrench = batch["wrench_hist"].to(device, non_blocking=True)
            pose = batch["pose"].to(device, non_blocking=True)
            y = batch["y"].to(device, non_blocking=True)

            optimizer.zero_grad(set_to_none=True)

            with autocast(device_type="cuda", enabled=args.amp):
                logits = model(ext, wrist, wrench, pose, norm=norm_torch, img_size=args.img_size)
                loss = F.cross_entropy(logits, y, weight=w)

            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()

            pred = logits.argmax(dim=-1)
            correct += int((pred == y).sum().item())
            total += int(y.shape[0])
            loss_sum += float(loss.item()) * y.shape[0]

            acc = correct / max(total, 1)
            avg_loss = loss_sum / max(total, 1)

            if (global_step % args.log_every) == 0:
                # GPU mem
                mem_gb = torch.cuda.memory_allocated(device) / (1024**3)
                peak_gb = torch.cuda.max_memory_allocated(device) / (1024**3)
                pbar.set_postfix(loss=f"{avg_loss:.4f}", acc=f"{acc:.3f}", mem=f"{mem_gb:.1f}G", peak=f"{peak_gb:.1f}G")
                if run is not None:
                    run.log({
                        "train/loss": avg_loss,
                        "train/acc": acc,
                        "train/lr": optimizer.param_groups[0]["lr"],
                        "gpu/mem_alloc_gb": mem_gb,
                        "gpu/mem_peak_gb": peak_gb,
                        "iter": global_step,
                        "epoch": epoch,
                    }, step=global_step)

            global_step += 1

        train_loss = loss_sum / max(total, 1)
        train_acc = correct / max(total, 1)

        # val
        val_loss, val_acc, val_mf1, cm, prf = eval_n(model, val_loader, device, norm_torch, img_size=args.img_size)

        epoch_time = time.time() - t0
        print(f"[epoch {epoch:03d}] train_loss={train_loss:.6f} train_acc={train_acc:.4f}  "
              f"val_loss={val_loss:.6f} val_acc={val_acc:.4f} val_macroF1={val_mf1:.4f}  time={epoch_time:.1f}s")

        if run is not None:
            # log epoch metrics
            log = {
                "epoch_train/loss": train_loss,
                "epoch_train/acc": train_acc,
                "epoch_val/loss": val_loss,
                "epoch_val/acc": val_acc,
                "epoch_val/macro_f1": val_mf1,
                "epoch_time_sec": epoch_time,
                "epoch": epoch,
            }
            run.log(log, step=global_step)

            # per-class table
            import wandb
            table = wandb.Table(columns=["class", "precision", "recall", "f1", "support"])
            for i, name in enumerate(EVENT_NAMES):
                d = prf[i]
                table.add_data(name, d["precision"], d["recall"], d["f1"], d["support"])
            run.log({"epoch_val/per_class_prf": table}, step=global_step)

            # confusion matrix as table
            cm_table = wandb.Table(
                columns=["true\\pred"] + EVENT_NAMES,
                data=[[EVENT_NAMES[i]] + [int(cm[i, j]) for j in range(NUM_EVENTS)] for i in range(NUM_EVENTS)],
            )
            run.log({"epoch_val/confusion_matrix": cm_table}, step=global_step)

        # save ckpt each epoch + best by val_macroF1 (or val_acc)
        ckpt = {
            "model": model.state_dict(),
            "force_hist": args.force_hist,
            "delta": args.delta,
            "force_k": args.force_k,
            "img_size": args.img_size,
            "norm": {
                "wrench_mean": train_ds.norm.wrench_mean,
                "wrench_std": train_ds.norm.wrench_std,
                "pose_mean": train_ds.norm.pose_mean,
                "pose_std": train_ds.norm.pose_std,
            },
            "eval_samples": max(args.val_samples, 10000),
        }
        torch.save(ckpt, os.path.join(args.save_dir, f"paep_future_ep{epoch:03d}.pt"))

        improved = False
        # Prefer macro-F1 as "best" (more fair for rare events)
        if val_mf1 > best_val_mf1 + 1e-6:
            best_val_mf1 = val_mf1
            improved = True
        if val_acc > best_val_acc + 1e-6:
            best_val_acc = val_acc
            best_epoch = epoch

        if improved:
            torch.save(ckpt, os.path.join(args.save_dir, "best.pt"))
            patience_left = args.early_stop_patience
        else:
            patience_left -= 1

        if patience_left <= 0:
            print(f"[early-stop] No improvement for {args.early_stop_patience} epochs. Stop at epoch {epoch}.")
            break

    if run is not None:
        run.summary["best_val_acc"] = best_val_acc
        run.summary["best_val_macro_f1"] = best_val_mf1
        run.summary["best_epoch_acc"] = best_epoch
        run.finish()

    print("Done. best_val_acc:", best_val_acc, "best_val_macro_f1:", best_val_mf1, "best_epoch_acc:", best_epoch)


if __name__ == "__main__":
    main()
