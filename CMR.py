import os
import json
import argparse
from dataclasses import dataclass
from typing import List, Tuple, Optional

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
from tqdm import tqdm
import zarr


# -------------------------
# Utils
# -------------------------
def set_seed(seed: int = 0):
    import random
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


@dataclass
class NormStats:
    mean: np.ndarray
    std: np.ndarray


def robust_norm(x: np.ndarray) -> np.ndarray:
    # x: [N,D]
    return np.sqrt(np.sum(np.square(x), axis=-1))


def auto_threshold_by_quantile(x: np.ndarray, q: float) -> float:
    q = float(q)
    q = min(max(q, 0.5), 0.99)
    return float(np.quantile(x, q))

def resolve_array(root, key: str):
    # 1) 直接命中
    if key in root:
        return root[key]

    # 2) 常见前缀尝试
    prefixes = ["data", "obs", "observations", "replay_buffer", "dataset"]
    for p in prefixes:
        path = f"{p}/{key}"
        if path in root:
            return root[path]

    # 3) 递归搜索：找末尾名字匹配的 array
    hits = []
    def visitor(name, obj):
        # obj 可能是 zarr.Array 或 Group
        if hasattr(obj, "shape"):  # 粗略判断是 array
            if name.split("/")[-1] == key.split("/")[-1]:
                hits.append(name)

    root.visititems(visitor)
    if len(hits) > 0:
        # 如果找到多个，优先选“完全包含 key 名”的那个
        hits.sort(key=lambda n: (0 if n.endswith(key) else 1, len(n)))
        print(f"[resolve_array] '{key}' not found, use '{hits[0]}'")
        return root[hits[0]]

    raise KeyError(f"Cannot find array '{key}'. Available tree:\n{root.tree()}")

# -------------------------
# 1) Model: Causal Conv + GRU + Event Head
# -------------------------
class CausalConv1d(nn.Module):
    """Causal Conv1d: y[t] depends only on x[:t]. Input: [B,C,T]"""
    def __init__(self, in_ch, out_ch, kernel_size=3, dilation=1, bias=True):
        super().__init__()
        self.kernel_size = kernel_size
        self.dilation = dilation
        self.conv = nn.Conv1d(in_ch, out_ch, kernel_size=kernel_size, dilation=dilation, bias=bias)

    def forward(self, x):
        pad_left = (self.kernel_size - 1) * self.dilation
        x = F.pad(x, (pad_left, 0))
        return self.conv(x)


class ResCausalConvBlock(nn.Module):
    def __init__(self, in_ch, out_ch, kernel_size=5, dilation=1, dropout=0.1):
        super().__init__()
        self.conv = CausalConv1d(in_ch, out_ch, kernel_size=kernel_size, dilation=dilation, bias=False)

        g = min(8, out_ch)
        while out_ch % g != 0 and g > 1:
            g -= 1
        self.norm = nn.GroupNorm(g, out_ch)
        self.act = nn.ReLU(inplace=True)
        self.drop = nn.Dropout(dropout)

        self.skip = None
        if in_ch != out_ch:
            self.skip = nn.Conv1d(in_ch, out_ch, kernel_size=1, bias=False)

    def forward(self, x):
        y = self.conv(x)
        y = self.norm(y)
        y = self.act(y)
        y = self.drop(y)
        s = x if self.skip is None else self.skip(x)
        return y + s


class ForceEncoderConvGRU(nn.Module):
    """
    force_seq: [B,T,6] -> tokens: [B,T,token_dim]
    """
    def __init__(
        self,
        input_dim=6,
        token_dim=256,
        conv_channels=128,
        kernel_size=5,
        dilations=(1, 2, 4, 8),
        gru_layers=1,
        dropout=0.1,
    ):
        super().__init__()
        self.in_proj = nn.Conv1d(input_dim, conv_channels, kernel_size=1, bias=False)

        blocks = []
        for d in dilations:
            blocks.append(
                ResCausalConvBlock(conv_channels, conv_channels, kernel_size=kernel_size, dilation=d, dropout=dropout)
            )
        self.conv_stack = nn.Sequential(*blocks)

        self.gru = nn.GRU(
            input_size=conv_channels,
            hidden_size=token_dim,
            num_layers=gru_layers,
            batch_first=True,
            dropout=0.0 if gru_layers == 1 else dropout,
            bidirectional=False,
        )
        self.out_norm = nn.LayerNorm(token_dim)

    def forward(self, force_seq):
        # force_seq: [B,T,6]
        x = force_seq.transpose(1, 2)     # [B,6,T]
        x = self.in_proj(x)               # [B,C,T]
        x = self.conv_stack(x)            # [B,C,T]
        x = x.transpose(1, 2)             # [B,T,C]
        out, _ = self.gru(x)              # [B,T,token_dim]
        out = self.out_norm(out)
        return out


def masked_mean(tokens, mask):
    """
    tokens: [B,T,D]
    mask: [B,T] bool, True=pad
    """
    valid = (~mask).float()
    denom = valid.sum(dim=1, keepdim=True).clamp_min(1.0)
    return (tokens * valid.unsqueeze(-1)).sum(dim=1) / denom


class EventHead(nn.Module):
    """
    Binary event head:
    - per_timestep=False: logits [B,1]  (window-level)
    - per_timestep=True:  logits [B,T,1]
    """
    def __init__(self, token_dim=256, per_timestep=False, dropout=0.1):
        super().__init__()
        self.per_timestep = per_timestep
        hidden = max(128, token_dim // 2)
        self.net = nn.Sequential(
            nn.LayerNorm(token_dim),
            nn.Linear(token_dim, hidden),
            nn.ReLU(inplace=True),
            nn.Dropout(dropout),
            nn.Linear(hidden, 1),
        )

    def forward(self, tokens, padding_mask=None):
        if self.per_timestep:
            return self.net(tokens)  # [B,T,1]
        if padding_mask is None:
            pooled = tokens.mean(dim=1)
        else:
            pooled = masked_mean(tokens, padding_mask)
        return self.net(pooled)      # [B,1]


class ForceEventModel(nn.Module):
    def __init__(
        self,
        input_dim=6,
        token_dim=256,
        conv_channels=128,
        kernel_size=5,
        dilations=(1,2,4,8),
        gru_layers=1,
        dropout=0.1,
        per_timestep=False,
    ):
        super().__init__()
        self.encoder = ForceEncoderConvGRU(
            input_dim=input_dim,
            token_dim=token_dim,
            conv_channels=conv_channels,
            kernel_size=kernel_size,
            dilations=dilations,
            gru_layers=gru_layers,
            dropout=dropout,
        )
        self.head = EventHead(token_dim=token_dim, per_timestep=per_timestep, dropout=dropout)

    def forward(self, force_seq, padding_mask=None):
        tokens = self.encoder(force_seq)
        logits = self.head(tokens, padding_mask=padding_mask)
        return logits, tokens


# -------------------------
# 2) episode_ends -> episode slices -> window sampling
# -------------------------
def episode_ends_to_slices(episode_ends: np.ndarray, total_len: int) -> List[Tuple[int,int]]:
    """
    支持两种常见格式：
    1) episode_ends 是 “exclusive end”（最后一个等于 total_len）
       例如 [120, 250, 400] 表示 [0,120), [120,250), [250,400)
    2) episode_ends 是 “inclusive end”（最后一个等于 total_len-1）
       例如 [119, 249, 399] 表示 [0,120), [120,250), [250,400)
    """
    episode_ends = np.asarray(episode_ends).astype(np.int64).copy()
    if episode_ends.ndim != 1 or len(episode_ends) == 0:
        raise ValueError("episode_ends must be a non-empty 1D array")

    last = int(episode_ends[-1])
    if last == total_len - 1:
        # inclusive -> convert to exclusive
        episode_ends = episode_ends + 1
    elif last != total_len:
        # 既不是 N，也不是 N-1：尝试兜底修正
        # 如果它本来就是 exclusive 但没到 N，说明 zarr 里可能只存了部分；这里强行补到 total_len
        if last < total_len:
            episode_ends = np.concatenate([episode_ends, np.array([total_len], dtype=np.int64)], axis=0)
        else:
            raise ValueError(f"episode_ends[-1]={last} is > total_len={total_len}, please check data")

    # build slices
    starts = np.concatenate([np.array([0], dtype=np.int64), episode_ends[:-1]], axis=0)
    ends = episode_ends
    slices = []
    for s, e in zip(starts.tolist(), ends.tolist()):
        if e > s:
            slices.append((int(s), int(e)))
    return slices


def build_window_starts(episode_slices: List[Tuple[int,int]], seq_len: int, stride: int) -> np.ndarray:
    starts = []
    for (s, e) in episode_slices:
        last_start = e - seq_len
        if last_start < s:
            continue
        for t0 in range(s, last_start + 1, stride):
            starts.append(t0)
    return np.asarray(starts, dtype=np.int64)


# -------------------------
# 3) Dataset: use episode_ends + pseudo label(contact)
# -------------------------
class ZarrEventDataset(Dataset):
    """
    Window-level binary contact label (pseudo):
      label=1 if window contains any contact frame (or last-frame contact if you choose)
    """
    def __init__(
        self,
        zarr_path: str,
        seq_len: int,
        stride: int,
        signal_key: str = "left_robot_tcp_wrench",  # [N,6]
        use_tau_ext: bool = False,
        tau_ext_key: str = "left_robot_tau_ext",    # [N,7]
        contact_quantile: float = 0.90,
        contact_threshold: Optional[float] = None,
        window_label_mode: str = "any",  # "any" or "last"
        norm_stats: Optional[NormStats] = None,
    ):
        super().__init__()
        self.zarr_path = zarr_path
        self.seq_len = seq_len
        self.stride = stride
        self.signal_key = signal_key
        self.use_tau_ext = use_tau_ext
        self.tau_ext_key = tau_ext_key
        self.contact_quantile = float(contact_quantile)
        self.window_label_mode = window_label_mode

        root = zarr.open(zarr_path, mode="r")
        self.root = root

        if "meta/episode_ends" not in root:
            raise KeyError("Cannot find 'meta/episode_ends' in zarr. Please check zarr structure.")
        self.episode_ends = np.array(root["meta/episode_ends"], dtype=np.int64)

        #self.signal = root[signal_key]
        self.signal = resolve_array(root, signal_key)
        total_len = self.signal.shape[0]

        # episode slices from episode_ends
        self.episode_slices = episode_ends_to_slices(self.episode_ends, total_len)

        # window starts
        self.starts = build_window_starts(self.episode_slices, seq_len=seq_len, stride=stride)
        if len(self.starts) == 0:
            raise RuntimeError(f"No valid windows found. seq_len={seq_len}, check episode_ends and stride.")

        # pseudo labels on all frames
        sig_np = np.asarray(self.signal[:], dtype=np.float32)  # [N,6]
        force_norm = robust_norm(sig_np)

        if contact_threshold is None:
            contact_threshold = auto_threshold_by_quantile(force_norm, self.contact_quantile)
        self.contact_threshold = float(contact_threshold)

        if use_tau_ext and (tau_ext_key in root):
            tau_np = np.asarray(root[tau_ext_key][:], dtype=np.float32)
            tau_norm = robust_norm(tau_np)
            self.tau_threshold = float(np.quantile(tau_norm, self.contact_quantile))
            contact_frame = (force_norm > self.contact_threshold) | (tau_norm > self.tau_threshold)
        else:
            self.tau_threshold = None
            contact_frame = (force_norm > self.contact_threshold)

        # window labels
        labels = np.zeros((len(self.starts),), dtype=np.float32)
        T = seq_len
        for i, t0 in enumerate(self.starts):
            t1 = t0 + T
            if window_label_mode == "any":
                labels[i] = 1.0 if np.any(contact_frame[t0:t1]) else 0.0
            elif window_label_mode == "last":
                labels[i] = 1.0 if bool(contact_frame[t1 - 1]) else 0.0
            else:
                raise ValueError("window_label_mode must be 'any' or 'last'")
        self.labels = labels

        # norm
        if norm_stats is None:
            m = sig_np.mean(axis=0)
            s = sig_np.std(axis=0) + 1e-6
            self.norm_stats = NormStats(mean=m, std=s)
        else:
            self.norm_stats = norm_stats

    def subset(self, indices: np.ndarray):
        indices = np.asarray(indices, dtype=np.int64)
        self.starts = self.starts[indices]
        self.labels = self.labels[indices]
        return self

    def __len__(self):
        return len(self.starts)

    def __getitem__(self, idx):
        t0 = int(self.starts[idx])
        t1 = t0 + self.seq_len
        force = np.asarray(self.signal[t0:t1], dtype=np.float32)  # [T,6]
        force = (force - self.norm_stats.mean) / self.norm_stats.std

        y = np.asarray([self.labels[idx]], dtype=np.float32)      # [1]
        padding_mask = np.zeros((self.seq_len,), dtype=np.bool_)  # fixed window, no padding

        return {
            "force": torch.from_numpy(force),             # [T,6]
            "label": torch.from_numpy(y),                 # [1]
            "padding_mask": torch.from_numpy(padding_mask)  # [T]
        }


# -------------------------
# 4) Train / Eval
# -------------------------
@torch.no_grad()
def evaluate(model, loader, device, threshold=0.5):
    model.eval()
    total = 0
    loss_sum = 0.0
    tp = fp = tn = fn = 0

    bce_sum = nn.BCEWithLogitsLoss(reduction="sum")

    for batch in loader:
        force = batch["force"].to(device)          # [B,T,6]
        label = batch["label"].to(device)          # [B,1]
        mask = batch["padding_mask"].to(device)    # [B,T]

        logits, _ = model(force, padding_mask=mask)
        loss = bce_sum(logits, label)

        prob = torch.sigmoid(logits)
        pred = (prob >= threshold).float()

        total += label.numel()
        loss_sum += float(loss.item())

        tp += int(((pred == 1) & (label == 1)).sum().item())
        tn += int(((pred == 0) & (label == 0)).sum().item())
        fp += int(((pred == 1) & (label == 0)).sum().item())
        fn += int(((pred == 0) & (label == 1)).sum().item())

    eps = 1e-9
    acc = (tp + tn) / (tp + tn + fp + fn + eps)
    prec = tp / (tp + fp + eps)
    rec = tp / (tp + fn + eps)
    f1 = 2 * prec * rec / (prec + rec + eps)
    avg_loss = loss_sum / max(total, 1)

    return {"loss": avg_loss, "acc": acc, "precision": prec, "recall": rec, "f1": f1,
            "tp": tp, "fp": fp, "tn": tn, "fn": fn}


def train(args):
    set_seed(args.seed)
    device = torch.device("cuda" if torch.cuda.is_available() and not args.cpu else "cpu")

    # build full dataset (for threshold + starts + labels)
    full = ZarrEventDataset(
        zarr_path=args.zarr_path,
        seq_len=args.seq_len,
        stride=args.stride,
        signal_key=args.signal_key,
        use_tau_ext=args.use_tau_ext,
        tau_ext_key=args.tau_ext_key,
        contact_quantile=args.contact_quantile,
        contact_threshold=args.contact_threshold,
        window_label_mode=args.window_label_mode,
        norm_stats=None,  # temporary
    )

    n = len(full)
    idx = np.arange(n)
    np.random.shuffle(idx)
    n_val = max(1, int(n * args.val_ratio))
    val_idx = idx[:n_val]
    train_idx = idx[n_val:]

    # recompute norm stats using TRAIN windows only (more严格)
    root = zarr.open(args.zarr_path, mode="r")
    signal = np.asarray(root[args.signal_key][:], dtype=np.float32)  # [N,6]

    sample_windows = min(len(train_idx), args.norm_max_windows)
    chosen = train_idx[:sample_windows]
    frames = []
    for i in chosen:
        t0 = int(full.starts[int(i)])
        t1 = t0 + args.seq_len
        frames.append(signal[t0:t1])
    frames = np.concatenate(frames, axis=0)
    mean = frames.mean(axis=0)
    std = frames.std(axis=0) + 1e-6
    norm_stats = NormStats(mean=mean, std=std)

    # rebuild datasets with fixed threshold + train norm stats
    train_ds = ZarrEventDataset(
        zarr_path=args.zarr_path,
        seq_len=args.seq_len,
        stride=args.stride,
        signal_key=args.signal_key,
        use_tau_ext=args.use_tau_ext,
        tau_ext_key=args.tau_ext_key,
        contact_quantile=args.contact_quantile,
        contact_threshold=full.contact_threshold,
        window_label_mode=args.window_label_mode,
        norm_stats=norm_stats,
    ).subset(train_idx)

    val_ds = ZarrEventDataset(
        zarr_path=args.zarr_path,
        seq_len=args.seq_len,
        stride=args.stride,
        signal_key=args.signal_key,
        use_tau_ext=args.use_tau_ext,
        tau_ext_key=args.tau_ext_key,
        contact_quantile=args.contact_quantile,
        contact_threshold=full.contact_threshold,
        window_label_mode=args.window_label_mode,
        norm_stats=norm_stats,
    ).subset(val_idx)

    # imbalance handling
    pos = float(train_ds.labels.sum())
    neg = float(len(train_ds.labels) - pos)
    if pos < 1:
        print("[WARN] No positive samples. Lower contact_quantile or set contact_threshold manually.")
        pos_weight = torch.tensor(1.0, device=device)
    else:
        pos_weight = torch.tensor(neg / max(pos, 1.0), device=device)

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True,
                              num_workers=args.num_workers, pin_memory=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size, shuffle=False,
                            num_workers=args.num_workers, pin_memory=True)

    model = ForceEventModel(
        input_dim=6,
        token_dim=args.token_dim,
        conv_channels=args.conv_channels,
        kernel_size=args.kernel_size,
        dilations=tuple(args.dilations),
        gru_layers=args.gru_layers,
        dropout=args.dropout,
        per_timestep=False,
    ).to(device)

    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.wd)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=max(args.epochs, 1))
    loss_fn = nn.BCEWithLogitsLoss(pos_weight=pos_weight)

    os.makedirs(args.out_dir, exist_ok=True)
    best_path = os.path.join(args.out_dir, "best_event_head.pt")
    best_f1 = -1.0

    meta = {
        "zarr_path": args.zarr_path,
        "signal_key": args.signal_key,
        "seq_len": args.seq_len,
        "stride": args.stride,
        "window_label_mode": args.window_label_mode,
        "contact_threshold": float(full.contact_threshold),
        "tau_threshold": float(full.tau_threshold) if full.tau_threshold is not None else None,
        "norm_mean": norm_stats.mean.tolist(),
        "norm_std": norm_stats.std.tolist(),
        "model_cfg": {
            "token_dim": args.token_dim,
            "conv_channels": args.conv_channels,
            "kernel_size": args.kernel_size,
            "dilations": list(args.dilations),
            "gru_layers": args.gru_layers,
            "dropout": args.dropout,
        },
        "pos": pos,
        "neg": neg,
        "pos_weight": float(pos_weight.detach().cpu().item()),
        "label_mode": "contact_pseudo",
    }
    with open(os.path.join(args.out_dir, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    print(f"[INFO] device={device}")
    print(f"[INFO] train windows={len(train_ds)}, val windows={len(val_ds)}")
    print(f"[INFO] contact_threshold(force_norm)={full.contact_threshold:.4f}")
    if full.tau_threshold is not None:
        print(f"[INFO] tau_threshold(tau_norm)={full.tau_threshold:.4f}")
    print(f"[INFO] pos={pos:.0f}, neg={neg:.0f}, pos_weight={pos_weight.item():.3f}")

    for epoch in range(1, args.epochs + 1):
        model.train()
        running = 0.0
        count = 0

        pbar = tqdm(train_loader, desc=f"Epoch {epoch}/{args.epochs}", ncols=100)
        for batch in pbar:
            force = batch["force"].to(device)       # [B,T,6]
            label = batch["label"].to(device)       # [B,1]
            mask = batch["padding_mask"].to(device) # [B,T]

            optimizer.zero_grad(set_to_none=True)
            logits, _ = model(force, padding_mask=mask)
            loss = loss_fn(logits, label)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), args.grad_clip)
            optimizer.step()

            running += float(loss.item()) * label.size(0)
            count += label.size(0)
            pbar.set_postfix(loss=running / max(count, 1))

        scheduler.step()

        metrics = evaluate(model, val_loader, device, threshold=args.eval_threshold)
        print(f"[VAL] epoch={epoch} loss={metrics['loss']:.4f} "
              f"acc={metrics['acc']:.3f} prec={metrics['precision']:.3f} rec={metrics['recall']:.3f} f1={metrics['f1']:.3f} "
              f"(tp={metrics['tp']} fp={metrics['fp']} tn={metrics['tn']} fn={metrics['fn']})")

        if metrics["f1"] > best_f1:
            best_f1 = metrics["f1"]
            torch.save({
                "model": model.state_dict(),
                "meta": meta,
                "epoch": epoch,
                "val_metrics": metrics,
            }, best_path)
            print(f"[SAVE] best -> {best_path} (f1={best_f1:.3f})")

    print("[DONE] Training finished.")
    print(f"[BEST] {best_path} f1={best_f1:.3f}")


def parse_args():
    ap = argparse.ArgumentParser("Train event head (contact) using meta/episode_ends")
    ap.add_argument("--zarr_path", type=str, required=True)
    ap.add_argument("--out_dir", type=str, default="outputs/event_head_contact")

    ap.add_argument("--signal_key", type=str, default="left_robot_tcp_wrench")
    ap.add_argument("--use_tau_ext", action="store_true")
    ap.add_argument("--tau_ext_key", type=str, default="left_robot_tau_ext")

    ap.add_argument("--seq_len", type=int, default=100)
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--window_label_mode", type=str, default="any", choices=["any", "last"])

    ap.add_argument("--contact_quantile", type=float, default=0.90)
    ap.add_argument("--contact_threshold", type=float, default=None)

    ap.add_argument("--token_dim", type=int, default=256)
    ap.add_argument("--conv_channels", type=int, default=128)
    ap.add_argument("--kernel_size", type=int, default=5)
    ap.add_argument("--dilations", type=int, nargs="+", default=[1,2,4,8])
    ap.add_argument("--gru_layers", type=int, default=1)
    ap.add_argument("--dropout", type=float, default=0.1)

    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--batch_size", type=int, default=128)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--wd", type=float, default=1e-4)
    ap.add_argument("--grad_clip", type=float, default=1.0)
    ap.add_argument("--val_ratio", type=float, default=0.1)
    ap.add_argument("--eval_threshold", type=float, default=0.5)

    ap.add_argument("--num_workers", type=int, default=4)
    ap.add_argument("--norm_max_windows", type=int, default=2000)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--cpu", action="store_true")
    return ap.parse_args()


if __name__ == "__main__":
    args = parse_args()
    train(args)
