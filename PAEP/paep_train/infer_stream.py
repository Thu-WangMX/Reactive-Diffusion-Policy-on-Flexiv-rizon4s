# infer_stream.py
# No argparse version. Supports BOTH:
#  - New PAEP: phase(3) + contact(1) heads with forward() -> {"p_phase","p_contact"} or forward_logits()
#  - Old PAEP: single head logits -> softmax

import numpy as np
import torch

from paep_dataset import imagenet_normalize, NormStats, PHASE_NAMES, CONTACT_NAMES
from paep_model import PAEPFutureNet


# =========================
# User config (edit here)
# =========================
CKPT_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/paep_future_ckpt/paep_runs_0124/best.pt"
DEVICE = "cuda"
AMP = False
RUN_ONCE = False   # True: run one step then exit


def preprocess_img_uint8(img_hwc_uint8: np.ndarray) -> torch.Tensor:
    """
    img: (H,W,3) uint8
    return: (3,H,W) float32, ImageNet normalized (consistent with paep_gated policy)
    """
    x = torch.from_numpy(img_hwc_uint8).permute(2, 0, 1).contiguous().float() / 255.0
    return imagenet_normalize(x)


def build_model_from_ckpt(ckpt: dict) -> PAEPFutureNet:
    cfg = ckpt.get("cfg", {})
    try:
        model = PAEPFutureNet(
            img_pretrained=False,
            force_hist=int(cfg.get("force_hist", ckpt.get("force_hist", 48))),
            delta=int(cfg.get("delta", ckpt.get("delta", 6))),
            future_k=int(cfg.get("future_k", ckpt.get("future_k", 12))),
            force_encoder=str(cfg.get("force_encoder", "tcn")),
            img_size=int(cfg.get("img_size", 224)),
            phase_names=list(cfg.get("phase_names", PHASE_NAMES)),
        )
        return model
    except TypeError:
        force_k = int(ckpt.get("force_k", cfg.get("force_k", 25)))
        model = PAEPFutureNet(
            img_pretrained=False,
            force_k=force_k,
        )
        return model


@torch.no_grad()
def run_once(
    model: torch.nn.Module,
    device: str,
    amp: bool,
    norm: NormStats,
    ext_img: np.ndarray,
    wrist_img: np.ndarray,
    wrench_hist: np.ndarray,   # (L,6) float32
    pose: np.ndarray,          # (P,) float32
    img_size: int = 224,
):
    # normalize force/pose (z-score)
    wrench_hist_n = (wrench_hist - norm.wrench_mean[None, :]) / norm.wrench_std[None, :]
    pose_n = (pose - norm.pose_mean) / norm.pose_std

    # to torch
    ext_t = preprocess_img_uint8(ext_img).unsqueeze(0).to(device)
    wrist_t = preprocess_img_uint8(wrist_img).unsqueeze(0).to(device)

    wrench_t = torch.from_numpy(wrench_hist_n).unsqueeze(0).to(device=device, dtype=torch.float32)
    pose_t = torch.from_numpy(pose_n).unsqueeze(0).to(device=device, dtype=torch.float32)

    with torch.autocast(device_type="cuda", dtype=torch.float16, enabled=amp):
        # Prefer new API: forward() -> probs dict
        out = None
        try:
            out = model(ext_t, wrist_t, wrench_t, pose_t, img_size=img_size)
        except TypeError:
            # old signature without img_size/norm
            out = model(ext_t, wrist_t, wrench_t, pose_t)

        # New two-head: return dict probs or use forward_logits
        if isinstance(out, dict) and ("p_phase" in out) and ("p_contact" in out):
            p_phase = out["p_phase"][0].float().cpu().numpy()     # (3,)
            p_contact = float(out["p_contact"][0].float().cpu().numpy())  # scalar
            pred_phase = int(np.argmax(p_phase))
            pred_contact = int(p_contact >= 0.5)
            return {
                "mode": "two_head_prob",
                "pred_phase": pred_phase,
                "p_phase": p_phase,
                "pred_contact": pred_contact,
                "p_contact": p_contact,
            }

        # If model provides forward_logits (two-head)
        if hasattr(model, "forward_logits"):
            try:
                logits_phase, logit_contact = model.forward_logits(ext_t, wrist_t, wrench_t, pose_t, img_size=img_size)
            except TypeError:
                logits_phase, logit_contact = model.forward_logits(ext_t, wrist_t, wrench_t, pose_t)
            p_phase = torch.softmax(logits_phase, dim=-1)[0].float().cpu().numpy()
            p_contact = float(torch.sigmoid(logit_contact)[0].float().cpu().numpy())
            pred_phase = int(np.argmax(p_phase))
            pred_contact = int(p_contact >= 0.5)
            return {
                "mode": "two_head_logits",
                "pred_phase": pred_phase,
                "p_phase": p_phase,
                "pred_contact": pred_contact,
                "p_contact": p_contact,
            }

        # Old single head: out is logits [1,C]
        logits = out
        prob = torch.softmax(logits, dim=-1)[0].float().cpu().numpy()
        pred = int(prob.argmax())
        return {
            "mode": "single_head",
            "pred_event": pred,
            "prob": prob,
        }


def main():
    ckpt = torch.load(CKPT_PATH, map_location="cpu", weights_only=False)

    # norm
    norm = NormStats.from_dict(ckpt["norm"]) if hasattr(NormStats, "from_dict") else NormStats(**ckpt["norm"])

    model = build_model_from_ckpt(ckpt)
    model.load_state_dict(ckpt["model"], strict=True)
    model.to(DEVICE)
    model.eval()

    cfg = ckpt.get("cfg", {})
    L = int(cfg.get("force_hist", ckpt.get("force_hist", 48)))
    img_size = int(cfg.get("img_size", 224))

    print(f"Loaded ckpt: {CKPT_PATH}")
    print(f"force_hist={L}, img_size={img_size}")
    print("Replace the dummy inputs with your real sensor stream.\n")

    while True:
        # ---------------------------
        # TODO: replace with real sensor inputs
        ext_img = np.zeros((240, 320, 3), dtype=np.uint8)
        wrist_img = np.zeros((240, 320, 3), dtype=np.uint8)
        wrench_hist = np.zeros((L, 6), dtype=np.float32)
        pose = np.zeros((len(norm.pose_mean),), dtype=np.float32)
        # ---------------------------

        out = run_once(model, DEVICE, AMP, norm, ext_img, wrist_img, wrench_hist, pose, img_size=img_size)

        if out["mode"].startswith("two_head"):
            print(
                f"phase={PHASE_NAMES[out['pred_phase']]} p_phase={np.round(out['p_phase'], 3)} | "
                f"contact={CONTACT_NAMES[out['pred_contact']]} p_contact={out['p_contact']:.3f}"
            )
        else:
            print(f"event={PHASE_NAMES[out['pred_event']]} prob={np.round(out['prob'], 3)}")

        if RUN_ONCE:
            break


if __name__ == "__main__":
    main()
