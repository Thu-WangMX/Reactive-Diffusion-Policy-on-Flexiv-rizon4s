# eval_paep.py
import argparse
import numpy as np
import torch
import zarr

from paep_dataset import EVENT_NAMES, pick_data_group, episode_ranges_from_ends, gather_ranges, load_split_ids
from paep_model import PAEPNet


def confusion_matrix_add(cm, pred, gt):
    for p, y in zip(pred, gt):
        cm[int(y), int(p)] += 1


@torch.no_grad()
def evaluate_split(
    zarr_path: str,
    ckpt_path: str,
    split_json: str,
    split: str,
    device: str = "cuda",
    chunk: int = 256,
    max_frames: int | None = None,
    return_cm: bool = False,
):
    """
    Evaluate on full frames within the selected split episodes (no sampling).
    Returns dict with acc/frames (and cm if return_cm).
    """
    ckpt = torch.load(ckpt_path, map_location="cpu")
    model = PAEPNet(num_events=len(EVENT_NAMES), force_k=int(ckpt.get("force_k", 25)))
    model.load_state_dict(ckpt["model"], strict=True)
    model.to(device).eval()

    norm = ckpt["norm"]
    wrench_mean = norm["wrench_mean"]
    wrench_std = norm["wrench_std"]
    pose_mean = norm["pose_mean"]
    pose_std = norm["pose_std"]

    root = zarr.open_group(zarr_path, mode="r")
    g, g_name = pick_data_group(root)

    episode_ends = np.asarray(root["meta/episode_ends"][:], dtype=np.int64)
    starts, ends = episode_ranges_from_ends(episode_ends)

    ep_ids = load_split_ids(split_json, split)
    ranges = gather_ranges(starts, ends, ep_ids)

    ext_arr = g["external_img"]
    wrist_arr = g["left_wrist_img"]
    wrench_all = np.asarray(g["left_robot_tcp_wrench"][:], dtype=np.float32)
    pose_all = np.asarray(g["left_robot_tcp_pose"][:], dtype=np.float32)
    y_all = np.asarray(g["paep_event"][:], dtype=np.int64)

    wrench_all = (wrench_all - wrench_mean) / (wrench_std + 1e-6)
    pose_all = (pose_all - pose_mean) / (pose_std + 1e-6)

    K = len(EVENT_NAMES)
    cm = np.zeros((K, K), dtype=np.int64)
    correct = 0
    total = 0

    for eid, s, e in ranges:
        if max_frames is not None and total >= max_frames:
            break
        a = s
        while a < e:
            b = min(e, a + chunk)
            if max_frames is not None:
                b = min(b, a + (max_frames - total))
            if b <= a:
                break

            ext = np.asarray(ext_arr[a:b], dtype=np.uint8)
            wrist = np.asarray(wrist_arr[a:b], dtype=np.uint8)
            wrench = wrench_all[a:b]
            pose = pose_all[a:b]
            gt = y_all[a:b]

            ext_t = torch.from_numpy(ext).float().div(255.0).permute(0, 3, 1, 2).unsqueeze(0).to(device)
            wrist_t = torch.from_numpy(wrist).float().div(255.0).permute(0, 3, 1, 2).unsqueeze(0).to(device)
            wrench_t = torch.from_numpy(wrench).unsqueeze(0).to(device)
            pose_t = torch.from_numpy(pose).unsqueeze(0).to(device)

            logits = model(ext_t, wrist_t, wrench_t, pose_t)  # [1,S,K]
            pred = logits.argmax(dim=-1).squeeze(0).cpu().numpy()

            confusion_matrix_add(cm, pred, gt)
            correct += int((pred == gt).sum())
            total += int(len(gt))
            a = b

    acc = correct / max(1, total)
    out = {"acc": acc, "frames": total, "cm": cm if return_cm else None}
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zarr_path", type=str, required=True)
    ap.add_argument("--ckpt", type=str, required=True)
    ap.add_argument("--split_json", type=str, required=True)
    ap.add_argument("--split", type=str, default="test", choices=["train", "val", "test"])
    ap.add_argument("--device", type=str, default="cuda")
    ap.add_argument("--chunk", type=int, default=256)
    ap.add_argument("--max_frames", type=int, default=None)
    args = ap.parse_args()

    res = evaluate_split(
        zarr_path=args.zarr_path,
        ckpt_path=args.ckpt,
        split_json=args.split_json,
        split=args.split,
        device=args.device,
        chunk=args.chunk,
        max_frames=args.max_frames,
        return_cm=True,
    )

    print(f"split={args.split}  frames={res['frames']}  acc={res['acc']:.4f}")

    cm = res["cm"]
    print("\nConfusion Matrix (rows=GT, cols=Pred):")
    header = "          " + " ".join([f"{n[:7]:>7}" for n in EVENT_NAMES])
    print(header)
    for i, name in enumerate(EVENT_NAMES):
        row = " ".join([f"{int(cm[i, j]):7d}" for j in range(len(EVENT_NAMES))])
        print(f"{name[:8]:>8} {row}")


if __name__ == "__main__":
    main()
