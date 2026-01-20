# PAEP/paep_train/eval_paep.py
import argparse
import json
from typing import Dict, Tuple

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

from paep_dataset import PAEPFutureDataset, EVENT_NAMES, NUM_EVENTS, NormStats
from paep_model import PAEPFutureNet


def confusion_matrix(y_true: np.ndarray, y_pred: np.ndarray, num_classes: int) -> np.ndarray:
    cm = np.zeros((num_classes, num_classes), dtype=np.int64)
    for t, p in zip(y_true, y_pred):
        cm[int(t), int(p)] += 1
    return cm


def per_class_prf(cm: np.ndarray) -> Dict[str, Dict[str, float]]:
    # cm: [C,C], row=true, col=pred
    C = cm.shape[0]
    out = {}
    for c in range(C):
        tp = cm[c, c]
        fp = cm[:, c].sum() - tp
        fn = cm[c, :].sum() - tp
        prec = float(tp / (tp + fp + 1e-9))
        rec = float(tp / (tp + fn + 1e-9))
        f1 = float((2 * prec * rec) / (prec + rec + 1e-9))
        out[c] = {"precision": prec, "recall": rec, "f1": f1, "support": float(cm[c, :].sum())}
    return out


def macro_f1(prf: Dict[int, Dict[str, float]]) -> float:
    f1s = [prf[c]["f1"] for c in prf.keys()]
    return float(np.mean(f1s)) if len(f1s) else 0.0


@torch.no_grad()
def eval_split(model, loader, device, norm_torch, img_size: int) -> Tuple[float, float, np.ndarray, Dict[int, Dict[str, float]]]:
    model.eval()
    total_loss = 0.0
    total_n = 0
    ys = []
    ps = []
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

    y_true = np.concatenate(ys, axis=0) if ys else np.zeros((0,), np.int64)
    y_pred = np.concatenate(ps, axis=0) if ps else np.zeros((0,), np.int64)

    cm = confusion_matrix(y_true, y_pred, NUM_EVENTS)
    prf = per_class_prf(cm)
    acc = float((y_true == y_pred).mean()) if len(y_true) else 0.0
    loss_avg = total_loss / max(total_n, 1)
    return loss_avg, acc, cm, prf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zarr_path", required=True)
    ap.add_argument("--split_json", required=True)
    ap.add_argument("--ckpt", required=True)
    ap.add_argument("--split", default="test", choices=["train", "val", "test"])
    ap.add_argument("--batch_size", type=int, default=256)
    ap.add_argument("--num_workers", type=int, default=0)
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--img_size", type=int, default=224)
    ap.add_argument("--use_wandb", action="store_true")
    ap.add_argument("--wandb_project", default="PAEP")
    ap.add_argument("--wandb_name", default=None)
    ap.add_argument("--out_json", default=None)
    args = ap.parse_args()

    # load ckpt (explicit weights_only=False to avoid future confusion)
    ckpt = torch.load(args.ckpt, map_location="cpu", weights_only=False)

    model = PAEPFutureNet(
        num_events=NUM_EVENTS,
        img_pretrained=False,  # weights are in ckpt anyway
        force_k=int(ckpt.get("force_k", 25)),
    )

    model.load_state_dict(ckpt["model"], strict=True)

    norm = ckpt.get("norm", None)
    if norm is None:
        # fallback: compute from train
        ds_tmp = PAEPFutureDataset(args.zarr_path, args.split_json, "train", force_hist=ckpt["force_hist"], delta=ckpt["delta"])
        norm = ds_tmp.norm.__dict__
    norm_stats = NormStats(
        wrench_mean=np.array(norm["wrench_mean"], dtype=np.float32),
        wrench_std=np.array(norm["wrench_std"], dtype=np.float32),
        pose_mean=np.array(norm["pose_mean"], dtype=np.float32),
        pose_std=np.array(norm["pose_std"], dtype=np.float32),
    )

    device = torch.device(args.device)
    model.to(device)
    norm_torch = norm_stats.to_torch(device=device)

    # dataset uses random sampling; for eval we want deterministic-ish:
    ds = PAEPFutureDataset(
        args.zarr_path,
        args.split_json,
        args.split,
        force_hist=int(ckpt["force_hist"]),
        delta=int(ckpt["delta"]),
        seed=0,
        transition_sampling=False,
        compute_norm=False,
        provided_norm=norm_stats,
    )

    # We'll evaluate on a fixed number of samples if present in ckpt else 10000
    eval_samples = int(ckpt.get("eval_samples", 10000))
    # Wrap dataset by taking first eval_samples __getitem__ calls via a sampler-like trick:
    class _TakeN(torch.utils.data.Dataset):
        def __init__(self, base, n):
            self.base = base
            self.n = n
        def __len__(self): return self.n
        def __getitem__(self, i): return self.base[i]

    ds_take = _TakeN(ds, eval_samples)

    loader = DataLoader(
        ds_take,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        pin_memory=False,
    )

    loss, acc, cm, prf = eval_split(model, loader, device, norm_torch, img_size=args.img_size)
    mf1 = macro_f1(prf)

    print(f"[{args.split}] loss={loss:.6f} acc={acc:.4f} macroF1={mf1:.4f}")
    print("\nConfusion Matrix (row=true, col=pred):")
    header = "true\\pred " + " ".join([f"{n:>9s}" for n in EVENT_NAMES])
    print(header)
    for i, name in enumerate(EVENT_NAMES):
        row = " ".join([f"{cm[i, j]:9d}" for j in range(NUM_EVENTS)])
        print(f"{name:>9s} {row}")

    print("\nPer-class PRF:")
    for i, name in enumerate(EVENT_NAMES):
        d = prf[i]
        print(f"{name:>9s}: P={d['precision']:.3f} R={d['recall']:.3f} F1={d['f1']:.3f} (support={int(d['support'])})")

    out = {
        "split": args.split,
        "loss": loss,
        "acc": acc,
        "macro_f1": mf1,
        "confusion_matrix": cm.tolist(),
        "per_class": {EVENT_NAMES[i]: prf[i] for i in range(NUM_EVENTS)},
        "ckpt": args.ckpt,
    }

    if args.out_json is not None:
        with open(args.out_json, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=2)

    if args.use_wandb:
        import wandb
        run = wandb.init(project=args.wandb_project, name=args.wandb_name, reinit=True)
        # log scalar + table
        run.log({"eval/loss": loss, "eval/acc": acc, "eval/macro_f1": mf1})
        table = wandb.Table(columns=["class", "precision", "recall", "f1", "support"])
        for i, name in enumerate(EVENT_NAMES):
            d = prf[i]
            table.add_data(name, d["precision"], d["recall"], d["f1"], d["support"])
        run.log({"eval/per_class_prf": table})

        # log confusion matrix
        cm_table = wandb.Table(
            columns=["true\\pred"] + EVENT_NAMES,
            data=[[EVENT_NAMES[i]] + [int(cm[i, j]) for j in range(NUM_EVENTS)] for i in range(NUM_EVENTS)],
        )
        run.log({"eval/confusion_matrix": cm_table})
        run.finish()


if __name__ == "__main__":
    main()
