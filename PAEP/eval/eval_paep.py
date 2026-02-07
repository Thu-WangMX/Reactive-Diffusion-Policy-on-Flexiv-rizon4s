"""
Offline evaluation for PAEP best.pt (no argparse).
- Loads ckpt best.pt
- Loads zarr + split json
- Evaluates N samples from split (dataset is random-sampling style)
- Prints loss/acc/macroF1 + confusion matrices

Run:
  python /home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/train/eval_best_pt_noargs.py
"""

import os
import sys
import json
from typing import Dict, List

import numpy as np
import torch
import torch.nn.functional as F
from torch.utils.data import DataLoader

# ------------------------------------------------------------
# 0) Hard-coded paths (EDIT THESE ONLY IF NEEDED)
# ------------------------------------------------------------
CKPT_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/ckpt/paep_plug_in_charger_0205/best.pt"
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_charger/plug_in_charger_stream_downsample1_zarr/replay_buffer.zarr"
SPLIT_JSON = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/config/paep_split_plugin_charger.json"

SPLIT = "test"          # "train" / "val" / "test"
EVAL_SAMPLES = 20000     # how many random samples to evaluate
BATCH_SIZE = 256
NUM_WORKERS = 4
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# If ckpt doesn't store phase_names (should, but just in case)
FALLBACK_PHASE_NAMES = ["approach", "search", "recovery", "insert"]

# ------------------------------------------------------------
# 1) Import your training code (expects this script under PAEP/train/)
# ------------------------------------------------------------
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
if THIS_DIR not in sys.path:
    sys.path.append(THIS_DIR)

from PAEP.train.paep_dataset import PAEPFutureDataset, NormStats  # noqa: E402
from PAEP.train.paep_model import PAEPFutureNet  # noqa: E402


# ------------------------------------------------------------
# 2) Metrics helpers
# ------------------------------------------------------------
def confusion_matrix(y_true: np.ndarray, y_pred: np.ndarray, num_classes: int) -> np.ndarray:
    cm = np.zeros((num_classes, num_classes), dtype=np.int64)
    for t, p in zip(y_true, y_pred):
        cm[int(t), int(p)] += 1
    return cm


def prf_from_cm(cm: np.ndarray) -> Dict[int, Dict[str, float]]:
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

def compute_class_weights(class_counts: Dict[str, int], phase_names: List[str]) -> torch.Tensor:
    counts = np.array([class_counts.get(n, 0) for n in phase_names], dtype=np.float64)
    counts = np.maximum(counts, 1.0)
    inv = 1.0 / counts
    w = inv / inv.sum() * len(inv)
    return torch.tensor(w, dtype=torch.float32)



class _TakeN(torch.utils.data.Dataset):
    """Wrap random-sampling dataset (len=1e9) into a finite-length dataset for evaluation."""
    def __init__(self, base, n: int):
        self.base = base
        self.n = int(n)

    def __len__(self):
        return self.n

    def __getitem__(self, idx):
        return self.base[idx]


# ------------------------------------------------------------
# 3) Build model from ckpt cfg (or fallback)
# ------------------------------------------------------------
def build_model_from_ckpt_cfg(cfg: dict, phase_names):
    # ckpt里存的是 cfg["tcn"] = {channels,kernel,blocks,dropout,pool}
    tcn = cfg.get("tcn", {})
    model = PAEPFutureNet(
        num_events=int(cfg.get("num_events", len(phase_names))),
        img_pretrained=bool(cfg.get("img_pretrained", True)),  # 这个字段你ckpt里可能没存；没有也无所谓
        img_feat_dim=256,
        force_encoder=str(cfg.get("force_encoder", "tcn")),
        force_k=int(cfg.get("force_k", 25)),
        force_feat_dim=256,
        tcn_channels=int(tcn.get("channels", 256)),
        tcn_kernel=int(tcn.get("kernel", 5)),
        tcn_blocks=int(tcn.get("blocks", 4)),
        tcn_dropout=float(tcn.get("dropout", 0.0)),
        tcn_pool=str(tcn.get("pool", "mean")),
    )
    return model



@torch.no_grad()
def evaluate(
    model: torch.nn.Module,
    loader: DataLoader,
    device: torch.device,
    norm_torch: Dict[str, torch.Tensor],
    img_size: int,
    phase_names: List[str],
    phase_w_torch: torch.Tensor,
) -> dict:

    model.eval()

    total_lp, total_lc, total_n = 0.0, 0.0, 0

    ys_p, ps_p = [], []
    ys_c, ps_c = [], []

    for batch in loader:
        ext = batch["ext_img"].to(device, non_blocking=True)

        # 3-view preferred
        left = batch.get("left_wrist_img", None)
        right = batch.get("right_wrist_img", None)
        if left is None:
            # backward compat
            left = batch["wrist_img"]
        left = left.to(device, non_blocking=True)
        if right is None:
            right = left
        else:
            right = right.to(device, non_blocking=True)

        wrench = batch["wrench_hist"].to(device, non_blocking=True)
        pose = batch["pose"].to(device, non_blocking=True)

        y_phase = batch["y_phase"].to(device, non_blocking=True).long()
        y_contact = batch["y_contact"].to(device, non_blocking=True).float().view(-1, 1)

        logits_phase, logit_contact = model.forward_logits(
            ext, left, right, wrench, pose, norm=norm_torch, img_size=img_size
        )

        lp = F.cross_entropy(logits_phase, y_phase, weight=phase_w_torch)
        lc = F.binary_cross_entropy_with_logits(logit_contact, y_contact)

        pred_phase = logits_phase.argmax(dim=-1)
        pred_contact = (torch.sigmoid(logit_contact) >= 0.5).long().view(-1)

        bs = int(y_phase.shape[0])
        total_lp += float(lp.item()) * bs
        total_lc += float(lc.item()) * bs
        total_n += bs

        ys_p.append(y_phase.detach().cpu().numpy())
        ps_p.append(pred_phase.detach().cpu().numpy())

        ys_c.append(y_contact.detach().cpu().numpy().reshape(-1).astype(np.int64))
        ps_c.append(pred_contact.detach().cpu().numpy().reshape(-1).astype(np.int64))

    y_phase_true = np.concatenate(ys_p, axis=0) if ys_p else np.zeros((0,), np.int64)
    y_phase_pred = np.concatenate(ps_p, axis=0) if ps_p else np.zeros((0,), np.int64)

    y_contact_true = np.concatenate(ys_c, axis=0) if ys_c else np.zeros((0,), np.int64)
    y_contact_pred = np.concatenate(ps_c, axis=0) if ps_c else np.zeros((0,), np.int64)

    C_phase = len(phase_names)
    cm_phase = confusion_matrix(y_phase_true, y_phase_pred, C_phase)
    prf_phase = prf_from_cm(cm_phase)
    acc_phase = float((y_phase_true == y_phase_pred).mean()) if len(y_phase_true) else 0.0
    mf1_phase = macro_f1(prf_phase)

    cm_contact = confusion_matrix(y_contact_true, y_contact_pred, 2)
    prf_contact = prf_from_cm(cm_contact)
    acc_contact = float((y_contact_true == y_contact_pred).mean()) if len(y_contact_true) else 0.0
    mf1_contact = macro_f1(prf_contact)

    return {
        "n": int(total_n),
        "loss_phase": total_lp / max(total_n, 1),
        "loss_contact": total_lc / max(total_n, 1),
        "acc_phase": acc_phase,
        "acc_contact": acc_contact,
        "macro_f1_phase": mf1_phase,
        "macro_f1_contact": mf1_contact,
        "cm_phase": cm_phase,
        "cm_contact": cm_contact,
        "prf_phase": prf_phase,
        "prf_contact": prf_contact,
    }


def main():
    print("[PAEP EVAL] no-arg eval")
    print("CKPT:", CKPT_PATH)
    print("ZARR:", ZARR_PATH)
    print("SPLIT_JSON:", SPLIT_JSON)
    print("SPLIT:", SPLIT)
    print("DEVICE:", DEVICE)

    # basic path sanity
    for p in [CKPT_PATH, ZARR_PATH, SPLIT_JSON]:
        if not os.path.exists(p):
            raise FileNotFoundError(f"Path not found: {p}")

    device = torch.device(DEVICE)

    ckpt = torch.load(CKPT_PATH, map_location="cpu", weights_only=False)
    cfg = ckpt.get("cfg", {})

    phase_names = cfg.get("phase_names", None)
    if phase_names is None:
        phase_names = FALLBACK_PHASE_NAMES
    phase_names = list(phase_names)

    if "norm" not in ckpt:
        raise KeyError("ckpt missing 'norm' (expected ckpt['norm']).")
    norm_obj = NormStats.from_dict(ckpt["norm"]) if hasattr(NormStats, "from_dict") else NormStats(**ckpt["norm"])
    norm_torch = norm_obj.to_torch(device=device)

    model = build_model_from_ckpt_cfg(cfg, phase_names=phase_names)
    model.load_state_dict(ckpt["model"], strict=True)
    model.to(device).eval()

    # dataset params should match training
    force_hist = int(cfg.get("force_hist", 36))
    delta = int(cfg.get("delta", 3))
    future_k = int(cfg.get("future_k", 8))
    img_size = int(cfg.get("img_size", 224))

    # IMPORTANT: eval uses uniform sampling (disable transition oversampling)
    ds = PAEPFutureDataset(
        ZARR_PATH, SPLIT_JSON, SPLIT,
        force_hist=force_hist, delta=delta, future_k=future_k,
        seed=0,
        transition_sampling=False,
        compute_norm=False,
        provided_norm=norm_obj,
        phase_names=phase_names,
        use_img_aug=False,   # <<< 强制 eval 禁用 aug
    )

    # -------------------------------
    # phase class weights (match training definition)
    # -------------------------------
    ds_take = _TakeN(ds, EVAL_SAMPLES)

    # NOTE: Prefer TRAIN-time final weights saved in ckpt to keep eval loss comparable.
    _used_final_phase_w = ("phase_w" in cfg)

    if _used_final_phase_w:
        phase_w_torch = torch.tensor(cfg["phase_w"], dtype=torch.float32, device=device)
    else:
        # fallback：use train split counts if available; otherwise use current split counts
        if "train_class_counts" in cfg:
            phase_w_torch = compute_class_weights(cfg["train_class_counts"], phase_names).to(device)
        else:
            phase_w_torch = compute_class_weights(ds.class_counts, phase_names).to(device)

        # optional multiplier ONLY in fallback path (avoid double-multiply)
        mult = cfg.get("PHASE_WEIGHT_MULT", None) or cfg.get("phase_weight_mult", None)
        if mult is not None:
            if isinstance(mult, dict) and len(mult) > 0:
                for i, name in enumerate(phase_names):
                    if name in mult:
                        phase_w_torch[i] = phase_w_torch[i] * float(mult[name])
                phase_w_torch = phase_w_torch / (phase_w_torch.mean().clamp_min(1e-12))
            else:
                phase_w_torch = phase_w_torch * float(mult)

    # (optional) debug print
    print("[EVAL] phase_w (used_final=%s): %s" % (_used_final_phase_w, phase_w_torch.detach().cpu().numpy()))



    loader = DataLoader(
        ds_take,
        batch_size=BATCH_SIZE,
        shuffle=False,
        num_workers=NUM_WORKERS,
        pin_memory=True,
    )

    out = evaluate(
        model, loader, device, norm_torch,
        img_size=img_size, phase_names=phase_names,
        phase_w_torch=phase_w_torch
    )


    print("\n==================== RESULT ====================")
    print(f"n_samples = {out['n']}")
    print(f"loss_phase   = {out['loss_phase']:.6f}")
    print(f"loss_contact = {out['loss_contact']:.6f}")
    print(f"acc_phase    = {out['acc_phase']:.4f}")
    print(f"acc_contact  = {out['acc_contact']:.4f}")
    print(f"macro_f1_phase   = {out['macro_f1_phase']:.4f}")
    print(f"macro_f1_contact = {out['macro_f1_contact']:.4f}")

    # Phase confusion matrix
    print("\n[Phase Confusion Matrix] row=true col=pred")
    cm_p = out["cm_phase"]
    header = " " * 12 + " ".join([f"{n[:8]:>8s}" for n in phase_names])
    print(header)
    for i, name in enumerate(phase_names):
        row = " ".join([f"{int(x):8d}" for x in cm_p[i]])
        print(f"{name[:10]:>10s}  {row}")

    # Per-class PRF
    print("\n[Phase Per-class PRF]")
    prf_p = out["prf_phase"]
    for i, name in enumerate(phase_names):
        d = prf_p[i]
        print(f"{name:>10s}  P={d['precision']:.3f} R={d['recall']:.3f} F1={d['f1']:.3f}  sup={int(d['support'])}")

    # Contact confusion matrix
    print("\n[Contact Confusion Matrix] row=true col=pred (0=no_contact, 1=contact)")
    cm_c = out["cm_contact"]
    print("            pred0    pred1")
    print(f"true0     {int(cm_c[0,0]):8d} {int(cm_c[0,1]):8d}")
    print(f"true1     {int(cm_c[1,0]):8d} {int(cm_c[1,1]):8d}")

    # Contact PRF
    print("\n[Contact PRF]")
    prf_c = out["prf_contact"]
    print(f"no_contact  P={prf_c[0]['precision']:.3f} R={prf_c[0]['recall']:.3f} F1={prf_c[0]['f1']:.3f}  sup={int(prf_c[0]['support'])}")
    print(f"contact     P={prf_c[1]['precision']:.3f} R={prf_c[1]['recall']:.3f} F1={prf_c[1]['f1']:.3f}  sup={int(prf_c[1]['support'])}")

    # Optional: save a json report next to ckpt
    report_path = os.path.join(os.path.dirname(CKPT_PATH), "eval_report_noargs.json")
    report = {
        "ckpt": CKPT_PATH,
        "zarr": ZARR_PATH,
        "split_json": SPLIT_JSON,
        "split": SPLIT,
        "eval_samples": EVAL_SAMPLES,
        "phase_names": phase_names,
        "metrics": {
            "loss_phase": out["loss_phase"],
            "loss_contact": out["loss_contact"],
            "acc_phase": out["acc_phase"],
            "acc_contact": out["acc_contact"],
            "macro_f1_phase": out["macro_f1_phase"],
            "macro_f1_contact": out["macro_f1_contact"],
        },
        "cm_phase": out["cm_phase"].tolist(),
        "cm_contact": out["cm_contact"].tolist(),
        "per_class_phase": {phase_names[i]: out["prf_phase"][i] for i in range(len(phase_names))},
        "per_class_contact": {"no_contact": out["prf_contact"][0], "contact": out["prf_contact"][1]},
    }
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    print(f"\n[OK] wrote report: {report_path}")


if __name__ == "__main__":
    main()
