# infer_stream.py
import argparse
import numpy as np
import torch
import torch.nn.functional as F
import zarr

from paep_dataset import EVENT_NAMES, pick_data_group, episode_ranges_from_ends, gather_ranges, load_split_ids
from paep_model import PAEPNet, PAEPStreamer

def img1_to_tensor(x_u8):
    # [H,W,3] -> [1,1,3,H,W]
    x = torch.from_numpy(x_u8).float().div(255.0).permute(2,0,1).unsqueeze(0).unsqueeze(0)
    return x

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zarr_path", type=str, required=True)
    ap.add_argument("--ckpt", type=str, required=True)
    ap.add_argument("--split_json", type=str, required=True)
    ap.add_argument("--split", type=str, default="test", choices=["train", "val", "test"])
    ap.add_argument("--episode_id", type=int, default=None, help="override: run this episode only")
    ap.add_argument("--device", type=str, default="cuda")
    ap.add_argument("--smooth_alpha", type=float, default=0.6)
    ap.add_argument("--max_steps", type=int, default=None, help="cap steps for this episode")
    args = ap.parse_args()

    ckpt = torch.load(args.ckpt, map_location="cpu")
    net = PAEPNet(num_events=len(EVENT_NAMES), force_k=int(ckpt.get("force_k", 25)))
    net.load_state_dict(ckpt["model"], strict=True)
    net.to(args.device).eval()

    streamer = PAEPStreamer(net).to(args.device)
    streamer.reset()

    norm = ckpt["norm"]
    wrench_mean = norm["wrench_mean"]
    wrench_std = norm["wrench_std"]
    pose_mean = norm["pose_mean"]
    pose_std = norm["pose_std"]

    root = zarr.open_group(args.zarr_path, mode="r")
    g, g_name = pick_data_group(root)
    print("Using group:", g_name)

    episode_ends = np.asarray(root["meta/episode_ends"][:], dtype=np.int64)
    starts, ends = episode_ranges_from_ends(episode_ends)

    if args.episode_id is not None:
        ep_ids = [int(args.episode_id)]
    else:
        ep_ids = load_split_ids(args.split_json, args.split)

    ranges = gather_ranges(starts, ends, ep_ids)
    if len(ranges) == 0:
        raise RuntimeError("No episode ranges found")

    # pick first episode in list
    eid, s, e = ranges[0]
    print(f"Running episode {eid} range [{s},{e}) split={args.split}")

    ext_arr = g["external_img"]
    wrist_arr = g["left_wrist_img"]
    wrench_all = np.asarray(g["left_robot_tcp_wrench"][s:e], dtype=np.float32)
    pose_all = np.asarray(g["left_robot_tcp_pose"][s:e], dtype=np.float32)

    # normalize
    wrench_all = (wrench_all - wrench_mean) / (wrench_std + 1e-6)
    pose_all = (pose_all - pose_mean) / (pose_std + 1e-6)

    prob_ema = None
    T = e - s
    steps = T if args.max_steps is None else min(T, args.max_steps)

    for i in range(steps):
        ext = np.asarray(ext_arr[s+i], dtype=np.uint8)
        wrist = np.asarray(wrist_arr[s+i], dtype=np.uint8)

        ext_t = img1_to_tensor(ext).to(args.device)
        wrist_t = img1_to_tensor(wrist).to(args.device)

        wrench_t = torch.from_numpy(wrench_all[i]).view(1,1,6).to(args.device)
        pose_t = torch.from_numpy(pose_all[i]).view(1,1,9).to(args.device)

        with torch.no_grad():
            logits = streamer.step(ext_t, wrist_t, wrench_t, pose_t)  # [1,1,C]
            prob = F.softmax(logits, dim=-1).squeeze(0).squeeze(0)    # [C]

        if prob_ema is None:
            prob_ema = prob
        else:
            prob_ema = args.smooth_alpha * prob_ema + (1 - args.smooth_alpha) * prob

        eid_pred = int(torch.argmax(prob_ema).item())
        print(f"frame={s+i:06d}  pred={EVENT_NAMES[eid_pred]:>9}  p={prob_ema[eid_pred].item():.3f}")

if __name__ == "__main__":
    main()
