# infer_stream.py
import argparse
import numpy as np
import torch

from paep_dataset import imagenet_normalize, NormStats, EVENT_NAMES
from paep_model import PAEPFutureNet


def preprocess_img_uint8(img_hwc_uint8: np.ndarray) -> torch.Tensor:
    """
    img: (H,W,3) uint8
    return: (3,H,W) float32 ImageNet normalized
    """
    x = torch.from_numpy(img_hwc_uint8).permute(2, 0, 1).contiguous().float() / 255.0
    return imagenet_normalize(x)


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
):
    # normalize force/pose
    wrench_hist_n = (wrench_hist - norm.wrench_mean[None, :]) / norm.wrench_std[None, :]
    pose_n = (pose - norm.pose_mean) / norm.pose_std

    # to torch
    ext_t = preprocess_img_uint8(ext_img).unsqueeze(0).to(device)
    wrist_t = preprocess_img_uint8(wrist_img).unsqueeze(0).to(device)

    wrench_t = torch.from_numpy(wrench_hist_n).unsqueeze(0).to(device=device, dtype=torch.float32)
    pose_t = torch.from_numpy(pose_n).unsqueeze(0).to(device=device, dtype=torch.float32)

    # forward
    with torch.autocast(device_type="cuda", dtype=torch.float16, enabled=amp):
        logits = model(ext_t, wrist_t, wrench_t, pose_t)           # [1,C]
        prob = torch.softmax(logits, dim=-1)[0].float().cpu().numpy()  # [C]

    pred = int(prob.argmax())
    return pred, prob


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", type=str, required=True)
    ap.add_argument("--device", type=str, default="cuda")
    ap.add_argument("--amp", action="store_true")
    ap.add_argument("--once", action="store_true", help="run single step then exit (for quick test)")
    args = ap.parse_args()

    ckpt = torch.load(args.ckpt, map_location="cpu")
    norm = NormStats.from_dict(ckpt["norm"])

    model = PAEPFutureNet(
        num_events=len(EVENT_NAMES),
        img_pretrained=False,
        img_dim=256,
        force_kernel=int(ckpt["force_k"]),
        force_gru_dim=256,
        pose_dim=len(norm.pose_mean),
        pose_mlp_dim=128,
        hidden=256,
        dropout=0.0,
        freeze_vision=False,
    )
    model.load_state_dict(ckpt["model"], strict=True)
    model.to(args.device)
    model.eval()

    L = int(ckpt["force_hist"])
    delta = int(ckpt["delta"])

    print(f"Loaded checkpoint: {args.ckpt}")
    print(f"force_hist={L}, delta={delta}  (predict event at t+Δ)")
    print("This is an API template. Replace the dummy sensor inputs with your real stream.\n")

    # ----- dummy loop (replace with real sensors) -----
    while True:
        # Replace below with your actual streaming inputs:
        ext_img = np.zeros((240, 320, 3), dtype=np.uint8)
        wrist_img = np.zeros((240, 320, 3), dtype=np.uint8)

        # IMPORTANT: wrench_hist should be resampled to 24Hz history if your sensor is not 24Hz
        wrench_hist = np.zeros((L, 6), dtype=np.float32)
        pose = np.zeros((len(norm.pose_mean),), dtype=np.float32)

        pred, prob = run_once(model, args.device, args.amp, norm, ext_img, wrist_img, wrench_hist, pose)
        print("pred_event(t+Δ) =", EVENT_NAMES[pred], "prob=", np.round(prob, 3))

        if args.once:
            break
    # -----------------------------------------------


if __name__ == "__main__":
    main()
