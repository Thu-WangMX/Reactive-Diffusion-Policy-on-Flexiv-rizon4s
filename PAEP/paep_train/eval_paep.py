# PAEP/paep_train/eval_paep.py
# No argparse version. Supports BOTH:
#  - New PAEP: phase(3) + contact(1) heads with forward_logits()
#  - Old PAEP: single event head with forward() -> logits

import json
from typing import Dict, Tuple, Optional

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

from paep_dataset import PAEPFutureDataset, PHASE_NAMES, CONTACT_NAMES, NormStats
from paep_model import PAEPFutureNet


# =========================
# User config (edit here)
# =========================
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr"
SPLIT_JSON = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/paep_split_40_5_5.json"
CKPT_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/paep_future_ckpt/paep_runs_0124/best.pt"

SPLIT = "test"          # "train" | "val" | "test"
DEVICE = "cuda"
IMG_SIZE = 224
BATCH_SIZE = 256
NUM_WORKERS = 0
AMP = False

# For deterministic-ish evaluation on random-sampling dataset:
EVAL_SAMPLES = 10000

# Optional: write results
OUT_JSON: Optional[str] = None
# OUT_JSON = "/tmp/paep_eval.json"


def confusion_matrix(y_true: np.ndarray, y_pred: np.ndarray, num_classes: int) -> np.ndarray:
    cm = np.zeros((num_classes, num_classes), dtype=np.int64)
    for t, p in zip(y_true, y_pred):
        cm[int(t), int(p)] += 1
    return cm


def prf_from_cm(cm: np.ndarray) -> Dict[int, Dict[str, float]]:
    # cm: [C,C], row=true, col=pred
    C = cm.shape[0]
    out: Dict[int, Dict[str, float]] = {}
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
    f1s = [prf[k]["f1"] for k in prf.keys()]
    return float(np.mean(f1s)) if len(f1s) else 0.0


class _TakeN(torch.utils.data.Dataset):
    def __init__(self, base, n: int):
        self.base = base
        self.n = int(n)

    def __len__(self):
        return self.n

    def __getitem__(self, i):
        return self.base[i]


@torch.no_grad()
def eval_new_two_head(
    model: torch.nn.Module,
    loader: DataLoader,
    device: torch.device,
    norm_torch: Dict[str, torch.Tensor],
    img_size: int,
) -> Dict:
    """
    New PAEP: model.forward_logits(...) -> (logits_phase [B,3], logit_contact [B,1])
    Dataset sample keys: y_phase (long), y_contact (float/bool/long)
    """
    model.eval()

    total_lp = 0.0
    total_lc = 0.0
    total_n = 0

    ys_p, ps_p = [], []
    ys_c, ps_c = [], []

    for batch in loader:
        ext = batch["ext_img"].to(device, non_blocking=True)
        wrist = batch["wrist_img"].to(device, non_blocking=True)
        wrench = batch["wrench_hist"].to(device, non_blocking=True)
        pose = batch["pose"].to(device, non_blocking=True)

        y_phase = batch["y_phase"].to(device, non_blocking=True).long()
        y_contact = batch["y_contact"].to(device, non_blocking=True).float().view(-1, 1)

        with torch.autocast(device_type="cuda", dtype=torch.float16, enabled=AMP):
            if hasattr(model, "forward_logits"):
                logits_phase, logit_contact = model.forward_logits(
                    ext, wrist, wrench, pose, norm=norm_torch, img_size=img_size
                )
            else:
                # fallback: assume forward returns dict probs or logits; try best effort
                out = model(ext, wrist, wrench, pose, norm=norm_torch, img_size=img_size)
                if isinstance(out, dict) and ("logits_phase" in out or "logit_contact" in out):
                    logits_phase = out["logits_phase"]
                    logit_contact = out["logit_contact"]
                else:
                    raise RuntimeError("Model does not support forward_logits() for two-head eval.")

            lp = F.cross_entropy(logits_phase, y_phase)
            lc = F.binary_cross_entropy_with_logits(logit_contact, y_contact)

        pred_phase = logits_phase.argmax(dim=-1)
        pred_contact = (torch.sigmoid(logit_contact) >= 0.5).long().view(-1)

        total_lp += float(lp.item()) * y_phase.shape[0]
        total_lc += float(lc.item()) * y_phase.shape[0]
        total_n += int(y_phase.shape[0])

        ys_p.append(y_phase.detach().cpu().numpy())
        ps_p.append(pred_phase.detach().cpu().numpy())

        ys_c.append(y_contact.detach().cpu().numpy().reshape(-1))
        ps_c.append(pred_contact.detach().cpu().numpy().reshape(-1))

    y_phase_true = np.concatenate(ys_p, axis=0) if ys_p else np.zeros((0,), np.int64)
    y_phase_pred = np.concatenate(ps_p, axis=0) if ps_p else np.zeros((0,), np.int64)

    y_contact_true = np.concatenate(ys_c, axis=0) if ys_c else np.zeros((0,), np.int64)
    y_contact_pred = np.concatenate(ps_c, axis=0) if ps_c else np.zeros((0,), np.int64)

    # phase metrics
    C_phase = len(PHASE_NAMES)
    cm_phase = confusion_matrix(y_phase_true, y_phase_pred, C_phase)
    prf_phase = prf_from_cm(cm_phase)
    acc_phase = float((y_phase_true == y_phase_pred).mean()) if len(y_phase_true) else 0.0
    mf1_phase = macro_f1(prf_phase)

    # contact metrics (2-class)
    C_contact = 2
    cm_contact = confusion_matrix(y_contact_true, y_contact_pred, C_contact)
    prf_contact = prf_from_cm(cm_contact)
    acc_contact = float((y_contact_true == y_contact_pred).mean()) if len(y_contact_true) else 0.0
    mf1_contact = macro_f1(prf_contact)

    out = {
        "mode": "two_head",
        "split": SPLIT,
        "loss_phase": total_lp / max(total_n, 1),
        "loss_contact": total_lc / max(total_n, 1),
        "acc_phase": acc_phase,
        "acc_contact": acc_contact,
        "macro_f1_phase": mf1_phase,
        "macro_f1_contact": mf1_contact,
        "cm_phase": cm_phase.tolist(),
        "cm_contact": cm_contact.tolist(),
        "per_class_phase": {PHASE_NAMES[i]: prf_phase[i] for i in range(C_phase)},
        "per_class_contact": {CONTACT_NAMES[i]: prf_contact[i] for i in range(C_contact)},
        "ckpt": CKPT_PATH,
    }
    return out


@torch.no_grad()
def eval_old_single_head(
    model: torch.nn.Module,
    loader: DataLoader,
    device: torch.device,
    norm_torch: Dict[str, torch.Tensor],
    img_size: int,
) -> Dict:
    """
    Old PAEP: model(...) -> logits [B,C]
    Dataset sample keys: y (long)
    """
    # In old dataset, EVENT_NAMES == PHASE_NAMES (3)
    event_names = PHASE_NAMES
    C = len(event_names)

    model.eval()
    total_loss = 0.0
    total_n = 0
    ys, ps = [], []

    for batch in loader:
        ext = batch["ext_img"].to(device, non_blocking=True)
        wrist = batch["wrist_img"].to(device, non_blocking=True)
        wrench = batch["wrench_hist"].to(device, non_blocking=True)
        pose = batch["pose"].to(device, non_blocking=True)
        y = batch["y"].to(device, non_blocking=True).long()

        with torch.autocast(device_type="cuda", dtype=torch.float16, enabled=AMP):
            logits = model(ext, wrist, wrench, pose, norm=norm_torch, img_size=img_size)
            loss = F.cross_entropy(logits, y)

        pred = logits.argmax(dim=-1)
        total_loss += float(loss.item()) * y.shape[0]
        total_n += int(y.shape[0])

        ys.append(y.detach().cpu().numpy())
        ps.append(pred.detach().cpu().numpy())

    y_true = np.concatenate(ys, axis=0) if ys else np.zeros((0,), np.int64)
    y_pred = np.concatenate(ps, axis=0) if ps else np.zeros((0,), np.int64)

    cm = confusion_matrix(y_true, y_pred, C)
    prf = prf_from_cm(cm)
    acc = float((y_true == y_pred).mean()) if len(y_true) else 0.0

    out = {
        "mode": "single_head",
        "split": SPLIT,
        "loss": total_loss / max(total_n, 1),
        "acc": acc,
        "macro_f1": macro_f1(prf),
        "cm": cm.tolist(),
        "per_class": {event_names[i]: prf[i] for i in range(C)},
        "ckpt": CKPT_PATH,
    }
    return out


def build_model_from_ckpt(ckpt: dict) -> PAEPFutureNet:
    """
    Best-effort model construction.
    - If your local paep_model.py is the new two-head version, it should accept cfg keys.
    - If it's the old single-head version, it'll still load if shapes match.
    """
    # If your new ckpt contains cfg, you can read cfg here to init model exactly.
    cfg = ckpt.get("cfg", {})

    # Try: new signature (may exist in your workspace)
    try:
        model = PAEPFutureNet(
            img_pretrained=False,
            force_hist=int(cfg.get("force_hist", ckpt.get("force_hist", 48))),
            delta=int(cfg.get("delta", ckpt.get("delta", 6))),
            future_k=int(cfg.get("future_k", ckpt.get("future_k", 12))),
            force_encoder=str(cfg.get("force_encoder", "tcn")),
            img_size=int(cfg.get("img_size", IMG_SIZE)),
            phase_names=list(cfg.get("phase_names", PHASE_NAMES)),
        )
        return model
    except TypeError:
        # Fallback: old signature (your uploaded eval_paep.py assumes this style)
        force_k = int(ckpt.get("force_k", cfg.get("force_k", 25)))
        model = PAEPFutureNet(
            img_pretrained=False,
            force_k=force_k,
        )
        return model


def main():
    device = torch.device(DEVICE)
    ckpt = torch.load(CKPT_PATH, map_location="cpu", weights_only=False)

    # norm
    norm_dict = ckpt.get("norm", None)
    if norm_dict is None:
        # compute from train if missing
        ds_tmp = PAEPFutureDataset(
            ZARR_PATH, SPLIT_JSON, "train",
            force_hist=int(ckpt.get("force_hist", 48)),
            delta=int(ckpt.get("delta", 6)),
            seed=0,
            transition_sampling=False,
            compute_norm=True,
        )
        norm_stats = ds_tmp.norm
    else:
        norm_stats = NormStats.from_dict(norm_dict) if hasattr(NormStats, "from_dict") else NormStats(**norm_dict)

    norm_torch = norm_stats.to_torch(device=device)

    # model
    model = build_model_from_ckpt(ckpt)
    model.load_state_dict(ckpt["model"], strict=True)
    model.to(device)
    model.eval()

    # dataset
    # Important: we do NOT want transition sampling during eval.
    # compute_norm=False, provided_norm uses ckpt norm
    ds = PAEPFutureDataset(
        ZARR_PATH,
        SPLIT_JSON,
        SPLIT,
        force_hist=int(ckpt.get("cfg", {}).get("force_hist", ckpt.get("force_hist", 48))),
        delta=int(ckpt.get("cfg", {}).get("delta", ckpt.get("delta", 6))),
        seed=0,
        transition_sampling=False,
        compute_norm=False,
        provided_norm=norm_stats,
    )
    ds_take = _TakeN(ds, EVAL_SAMPLES)

    loader = DataLoader(
        ds_take,
        batch_size=BATCH_SIZE,
        shuffle=False,
        num_workers=NUM_WORKERS,
        pin_memory=False,
    )

    # Detect whether dataset is new (y_phase/y_contact) or old (y)
    sample0 = ds_take[0]
    if ("y_phase" in sample0) and ("y_contact" in sample0):
        out = eval_new_two_head(model, loader, device, norm_torch, IMG_SIZE)
        print(f"[{SPLIT}] two-head  lp={out['loss_phase']:.6f} lc={out['loss_contact']:.6f} "
              f"ap={out['acc_phase']:.4f} ac={out['acc_contact']:.4f} "
              f"mf1p={out['macro_f1_phase']:.4f} mf1c={out['macro_f1_contact']:.4f}")
        print("\nPhase Confusion Matrix (row=true, col=pred):")
        for i, name in enumerate(PHASE_NAMES):
            row = " ".join([f"{int(x):7d}" for x in out["cm_phase"][i]])
            print(f"{name:>9s} {row}")
        print("\nContact Confusion Matrix (row=true, col=pred):")
        for i, name in enumerate(CONTACT_NAMES):
            row = " ".join([f"{int(x):7d}" for x in out["cm_contact"][i]])
            print(f"{name:>9s} {row}")
    else:
        out = eval_old_single_head(model, loader, device, norm_torch, IMG_SIZE)
        print(f"[{SPLIT}] single-head loss={out['loss']:.6f} acc={out['acc']:.4f} macroF1={out['macro_f1']:.4f}")
        print("\nConfusion Matrix (row=true, col=pred):")
        for i, name in enumerate(PHASE_NAMES):
            row = " ".join([f"{int(x):7d}" for x in out["cm"][i]])
            print(f"{name:>9s} {row}")

    if OUT_JSON is not None:
        with open(OUT_JSON, "w", encoding="utf-8") as f:
            json.dump(out, f, indent=2)
        print(f"\n[INFO] Wrote: {OUT_JSON}")


if __name__ == "__main__":
    main()
