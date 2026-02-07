#把某个 zarr 里的所有时间步都跑一遍 PAEP，输出 paep_prob_phase.npy 和 paep_prob_contact.npy，方便给 Fast 做离线 teacher/特征。

import os, json
import numpy as np
import torch
import zarr

from paep_model import PAEPFutureNet  # 用你训练同一份 model 文件

# -------------------------
# Config
# -------------------------
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr"
PAEP_CKPT = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/paep_future_ckpt/paep_runs_0124/best.pt"

OUT_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/label_for_fast/offline_wiping_board_paep_prob_out"
os.makedirs(OUT_DIR, exist_ok=True)

BATCH = 256
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# keys (must match train dataset)
KEY_EXT = "external_img"
KEY_WRIST = "left_wrist_img"
KEY_WRENCH = "left_robot_tcp_wrench"
KEY_POSE = "left_robot_tcp_pose"

# -------------------------
# Zarr helpers (match paep_dataset.py logic)
# -------------------------
def _open_group(zarr_path: str):
    root = zarr.open(zarr_path, mode="r")
    if isinstance(root, zarr.hierarchy.Group):
        if KEY_EXT in root.array_keys():
            return root, "root"
        if "data" in root.group_keys():
            g = root["data"]
            if KEY_EXT in g.array_keys():
                return g, "data"
    raise KeyError("Cannot find arrays. Expect external_img at root or under group 'data'.")

def _episode_slices(episode_ends: np.ndarray):
    ends = episode_ends.astype(np.int64).tolist()
    s = 0
    out = []
    for e in ends:
        out.append((s, e))
        s = e
    return out

def build_hist_by_episode(wrench: np.ndarray, episode_ends: np.ndarray, L: int):
    """
    Construct (T,L,6) wrench_hist without crossing episode boundary.
    For each episode, do right-aligned window, left pad with repeat-first within that episode.
    """
    T, D = wrench.shape
    out = np.zeros((T, L, D), dtype=np.float32)
    slices = _episode_slices(episode_ends)

    for (s, e) in slices:
        if e <= s:
            continue
        # episode segment
        w_ep = wrench[s:e]  # (Te,6)
        Te = w_ep.shape[0]
        # first sample for padding
        w0 = w_ep[0:1]
        for i in range(Te):
            t_global = s + i
            lo = max(0, i - (L - 1))
            chunk = w_ep[lo:i+1]
            if chunk.shape[0] < L:
                pad = np.repeat(w0, repeats=L - chunk.shape[0], axis=0)
                chunk = np.concatenate([pad, chunk], axis=0)
            out[t_global] = chunk
    return out

def to_torch_img(x_u8: np.ndarray, device: str):
    # (B,H,W,3) uint8 -> (B,3,H,W) float[0,1]
    x = torch.from_numpy(x_u8).to(device)
    if x.dtype != torch.uint8:
        x = x.to(torch.uint8)
    x = x.float() / 255.0
    x = x.permute(0, 3, 1, 2).contiguous()
    return x

# -------------------------
# Main
# -------------------------
def main():
    # load ckpt
    ckpt = torch.load(PAEP_CKPT, map_location="cpu")
    cfg = ckpt.get("cfg", {})
    norm = ckpt.get("norm", None)

    # infer params (match train_paep.py)
    force_hist = int(cfg.get("force_hist", 48))
    img_size = int(cfg.get("img_size", 224))
    force_encoder = cfg.get("force_encoder", "tcn")
    force_k = int(cfg.get("force_k", 25))
    tcn = cfg.get("tcn", {}) if isinstance(cfg.get("tcn", {}), dict) else {}

    # open zarr group (root/data) + episode ends from meta
    g, gname = _open_group(ZARR_PATH)
    root = zarr.open(ZARR_PATH, mode="r")
    if "meta" in root.group_keys() and "episode_ends" in root["meta"].array_keys():
        episode_ends = np.array(root["meta"]["episode_ends"][:], dtype=np.int64)
    elif "episode_ends" in g.array_keys():
        episode_ends = np.array(g["episode_ends"][:], dtype=np.int64)
    else:
        raise KeyError("Missing meta/episode_ends (needed to avoid crossing episodes)")

    # read arrays
    ext = g[KEY_EXT]
    wrist = g[KEY_WRIST]
    wrench = np.asarray(g[KEY_WRENCH][:, :6], dtype=np.float32)
    pose = np.asarray(g[KEY_POSE][:], dtype=np.float32)

    T = int(wrench.shape[0])
    assert ext.shape[0] == T and wrist.shape[0] == T and pose.shape[0] == T, "T mismatch among keys"
    assert pose.ndim == 2 and pose.shape[1] == 9, f"Expected pose (T,9) as in training, got {pose.shape}"

    # wrench_hist (episode-safe)
    wrench_hist = build_hist_by_episode(wrench, episode_ends, L=force_hist)  # (T,L,6)

    # build model strictly from cfg
    model = PAEPFutureNet(
        num_events=3,
        img_pretrained=True,          # 不重要（会被 load_state_dict 覆盖），但保持与训练语义一致
        img_feat_dim=256,
        force_encoder=force_encoder,
        force_k=force_k,
        force_feat_dim=256,
        tcn_channels=int(tcn.get("channels", 256)),
        tcn_kernel=int(tcn.get("kernel", 5)),
        tcn_blocks=int(tcn.get("blocks", 4)),
        tcn_dropout=float(tcn.get("dropout", 0.0)),
        tcn_pool=str(tcn.get("pool", "mean")),
    )
    model.load_state_dict(ckpt["model"], strict=True)
    model.eval().to(DEVICE)

    # norm tensors (same as training forward_logits normalization)
    if norm is not None:
        norm_t = {
            "wrench_mean": torch.as_tensor(norm["wrench_mean"]).float().to(DEVICE),
            "wrench_std":  torch.as_tensor(norm["wrench_std"]).float().to(DEVICE),
            "pose_mean":   torch.as_tensor(norm["pose_mean"]).float().to(DEVICE),
            "pose_std":    torch.as_tensor(norm["pose_std"]).float().to(DEVICE),
        }
    else:
        norm_t = None

    out_phase = np.zeros((T, 3), dtype=np.float32)
    out_contact = np.zeros((T, 1), dtype=np.float32)

    with torch.no_grad():
        for s in range(0, T, BATCH):
            e = min(T, s + BATCH)

            ext_b = np.asarray(ext[s:e], dtype=np.uint8)
            wrist_b = np.asarray(wrist[s:e], dtype=np.uint8)
            ext_t = to_torch_img(ext_b, DEVICE)
            wrist_t = to_torch_img(wrist_b, DEVICE)

            w_b = torch.from_numpy(wrench_hist[s:e]).to(DEVICE).float()
            p_b = torch.from_numpy(pose[s:e]).to(DEVICE).float()

            out = model(ext_t, wrist_t, w_b, p_b, norm=norm_t, img_size=img_size)
            out_phase[s:e] = out["p_phase"].detach().cpu().numpy().astype(np.float32)
            out_contact[s:e] = out["p_contact"].detach().cpu().numpy().astype(np.float32)

    np.save(os.path.join(OUT_DIR, "paep_prob_phase.npy"), out_phase)
    np.save(os.path.join(OUT_DIR, "paep_prob_contact.npy"), out_contact)

    meta = {
        "zarr_path": ZARR_PATH,
        "zarr_group": gname,
        "paep_ckpt": PAEP_CKPT,
        "cfg_used": {"force_hist": force_hist, "img_size": img_size, "force_encoder": force_encoder, "tcn": tcn},
        "has_norm_in_ckpt": (norm is not None),
        "stats": {
            "p_contact_min": float(out_contact.min()),
            "p_contact_max": float(out_contact.max()),
            "p_contact_mean": float(out_contact.mean()),
            "p_phase_mean": [float(out_phase[:, i].mean()) for i in range(3)],
        },
    }
    with open(os.path.join(OUT_DIR, "paep_prob_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    print("[OK] saved to:", OUT_DIR)

if __name__ == "__main__":
    main()
