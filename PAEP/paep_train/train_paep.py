# train_paep.py
import argparse
import os
import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import DataLoader

from paep_dataset import PAEPZarrDataset, EVENT_NAMES
from paep_model import PAEPNet

try:
    import wandb
except Exception:
    wandb = None


def class_weights_from_counts(counts: dict):
    # weight = total / (K * count), normalize mean=1
    total = sum(counts.values())
    K = len(EVENT_NAMES)
    w = []
    for name in EVENT_NAMES:
        c = max(1, counts.get(name, 1))
        w.append(total / (K * c))
    w = torch.tensor(w, dtype=torch.float32)
    w = w / w.mean()
    return w


@torch.no_grad()
def eval_one_epoch(model, dl, device):
    """
    Offline eval on sampled segments (val split dataloader).
    Returns (loss, acc).
    """
    model.eval()
    total = 0
    correct = 0
    loss_sum = 0.0
    ce = nn.CrossEntropyLoss()

    for batch in dl:
        ext = batch["external_img"].to(device, non_blocking=True)
        wrist = batch["wrist_img"].to(device, non_blocking=True)
        wrench = batch["wrench"].to(device, non_blocking=True)
        pose = batch["tcp_pose"].to(device, non_blocking=True)
        y = batch["event"].to(device, non_blocking=True)

        logits = model(ext, wrist, wrench, pose)   # [B,S,C]
        B, S, C = logits.shape
        loss = ce(logits.reshape(B*S, C), y.reshape(B*S))

        pred = logits.argmax(dim=-1)
        correct += int((pred == y).sum().item())
        total += int(y.numel())
        loss_sum += float(loss.item()) * (B*S)

    return loss_sum / max(1, total), correct / max(1, total)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zarr_path", type=str, required=True)
    ap.add_argument("--split_json", type=str, required=True)
    ap.add_argument("--save_dir", type=str, default="./paep_runs")

    ap.add_argument("--batch_size", type=int, default=8)
    ap.add_argument("--segment_len", type=int, default=96)
    ap.add_argument("--samples_per_epoch", type=int, default=20000)
    ap.add_argument("--val_samples", type=int, default=4000)
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--num_workers", type=int, default=4)
    ap.add_argument("--device", type=str, default="cuda")

    ap.add_argument("--img_pretrained", action="store_true")
    ap.add_argument("--force_k", type=int, default=25)
    ap.add_argument("--seed", type=int, default=0)

    # ---- wandb ----
    ap.add_argument("--use_wandb", action="store_true")
    ap.add_argument("--wandb_project", type=str, default="PAEP")
    ap.add_argument("--wandb_name", type=str, default=None)
    ap.add_argument("--wandb_mode", type=str, default="online", choices=["online", "offline", "disabled"])

    # ---- optional: run test once at end ----
    ap.add_argument("--run_test", action="store_true")
    ap.add_argument("--test_chunk", type=int, default=256)
    ap.add_argument("--test_max_frames", type=int, default=None)

    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    os.makedirs(args.save_dir, exist_ok=True)

    # ---------------- datasets ----------------
    train_ds = PAEPZarrDataset(
        args.zarr_path,
        segment_len=args.segment_len,
        samples_per_epoch=args.samples_per_epoch,
        split_json=args.split_json,
        split="train",
        norm_stats=None,          # compute from train
        norm_from_split="train",
        norm_max_samples=None,
        seed=args.seed,
    )
    val_ds = PAEPZarrDataset(
        args.zarr_path,
        segment_len=args.segment_len,
        samples_per_epoch=args.val_samples,
        split_json=args.split_json,
        split="val",
        norm_stats=train_ds.norm,  # reuse train norm
        seed=args.seed + 1,
    )

    print("Train group:", train_ds.g_name)
    print("Train class counts:", train_ds.class_counts)
    print("Val class counts:", val_ds.class_counts)

    train_dl = DataLoader(
        train_ds, batch_size=args.batch_size, shuffle=True,
        num_workers=args.num_workers, pin_memory=True, drop_last=True
    )
    val_dl = DataLoader(
        val_ds, batch_size=args.batch_size, shuffle=False,
        num_workers=args.num_workers, pin_memory=True, drop_last=False
    )

    # ---------------- model ----------------
    model = PAEPNet(
        num_events=len(EVENT_NAMES),
        img_pretrained=args.img_pretrained,
        img_feat_dim=256,
        force_k=args.force_k,
        force_hidden=256,
        proprio_dim=128,
        fusion_hidden=256,
        use_fusion_gru=True,
    ).to(args.device)

    # loss + opt
    w = class_weights_from_counts(train_ds.class_counts).to(args.device)
    ce = nn.CrossEntropyLoss(weight=w)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr)

    # ---------------- wandb ----------------
    run = None
    if args.use_wandb:
        if wandb is None:
            raise ImportError("wandb is not installed. Please `pip install wandb`.")
        run = wandb.init(
            project=args.wandb_project,
            name=args.wandb_name,
            mode=args.wandb_mode,
            config={
                "zarr_path": args.zarr_path,
                "split_json": args.split_json,
                "batch_size": args.batch_size,
                "segment_len": args.segment_len,
                "samples_per_epoch": args.samples_per_epoch,
                "val_samples": args.val_samples,
                "epochs": args.epochs,
                "lr": args.lr,
                "force_k": args.force_k,
                "img_pretrained": args.img_pretrained,
                "seed": args.seed,
                "train_class_counts": train_ds.class_counts,
                "val_class_counts": val_ds.class_counts,
            },
        )

    best_val = -1.0
    best_epoch = -1

    # ---------------- train loop ----------------
    for ep in range(args.epochs):
        model.train()
        loss_sum = 0.0
        n_frames = 0
        correct = 0
        total = 0

        for batch in train_dl:
            ext = batch["external_img"].to(args.device, non_blocking=True)
            wrist = batch["wrist_img"].to(args.device, non_blocking=True)
            wrench = batch["wrench"].to(args.device, non_blocking=True)
            pose = batch["tcp_pose"].to(args.device, non_blocking=True)
            y = batch["event"].to(args.device, non_blocking=True)

            logits = model(ext, wrist, wrench, pose)  # [B,S,C]
            B, S, C = logits.shape
            loss = ce(logits.reshape(B*S, C), y.reshape(B*S))

            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()

            # train stats
            loss_sum += float(loss.item()) * (B*S)
            n_frames += (B*S)
            pred = logits.argmax(dim=-1)
            correct += int((pred == y).sum().item())
            total += int(y.numel())

        train_loss = loss_sum / max(1, n_frames)
        train_acc = correct / max(1, total)

        val_loss, val_acc = eval_one_epoch(model, val_dl, args.device)

        print(f"[epoch {ep:03d}] train_loss={train_loss:.6f} train_acc={train_acc:.4f}  "
              f"val_loss={val_loss:.6f} val_acc={val_acc:.4f}")

        # save checkpoint
        ckpt = {
            "model": model.state_dict(),
            "event_names": EVENT_NAMES,
            "force_k": args.force_k,
            "segment_len": args.segment_len,
            "train_counts": train_ds.class_counts,
            "split_json": args.split_json,
            "norm": {
                "wrench_mean": train_ds.norm.wrench_mean,
                "wrench_std": train_ds.norm.wrench_std,
                "pose_mean": train_ds.norm.pose_mean,
                "pose_std": train_ds.norm.pose_std,
            },
            "epoch": ep,
            "train_loss": train_loss,
            "train_acc": train_acc,
            "val_loss": val_loss,
            "val_acc": val_acc,
        }
        torch.save(ckpt, os.path.join(args.save_dir, f"paep_ep{ep:03d}.pt"))

        # best
        if val_acc > best_val:
            best_val = val_acc
            best_epoch = ep
            torch.save(ckpt, os.path.join(args.save_dir, "paep_best.pt"))

        # wandb log
        if run is not None:
            wandb.log(
                {
                    "epoch": ep,
                    "train_loss": train_loss,
                    "train_acc": train_acc,
                    "val_loss": val_loss,
                    "val_acc": val_acc,
                    "lr": opt.param_groups[0]["lr"],
                    "best_val_acc_so_far": best_val,
                    "best_epoch_so_far": best_epoch,
                },
                step=ep,
            )

    print("Done. Best val acc:", best_val, "best epoch:", best_epoch)

    if run is not None:
        run.summary["best_val_acc"] = best_val
        run.summary["best_epoch"] = best_epoch

    # ---------------- optional: run test once and log ----------------
    if args.run_test:
        # run eval_paep.py logic in-process to avoid extra command
        from eval_paep import evaluate_split  # uses same file below
        test_res = evaluate_split(
            zarr_path=args.zarr_path,
            ckpt_path=os.path.join(args.save_dir, "paep_best.pt"),
            split_json=args.split_json,
            split="test",
            device=args.device,
            chunk=args.test_chunk,
            max_frames=args.test_max_frames,
            return_cm=True,
        )
        print("[TEST] acc:", test_res["acc"], "frames:", test_res["frames"])
        if run is not None:
            wandb.log(
                {
                    "test_acc": test_res["acc"],
                    "test_frames": test_res["frames"],
                },
                step=args.epochs,
            )
            # 混淆矩阵可以作为 artifact/表格上传（这里给简单 table）
            cm = test_res["cm"]
            table = wandb.Table(columns=["gt\\pred"] + EVENT_NAMES)
            for i, name in enumerate(EVENT_NAMES):
                table.add_data(name, *[int(cm[i, j]) for j in range(len(EVENT_NAMES))])
            wandb.log({"test_confusion_matrix": table}, step=args.epochs)

    if run is not None:
        run.finish()


if __name__ == "__main__":
    main()
