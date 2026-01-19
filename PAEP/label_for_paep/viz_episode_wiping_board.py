#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
from typing import Dict, Tuple, List

import numpy as np
import zarr
import cv2


# --------- Fixed keys (match your zarr) ----------
WRENCH_KEY = "data/left_robot_tcp_wrench"
WRIST_IMG_KEY = "data/left_wrist_img"
EXT_IMG_KEY = "data/external_img"
LABEL_KEY = "data/paep_event"
EP_ENDS_KEY = "meta/episode_ends"

FZ_INDEX = 2          # wrench[:,2] -> usually Fz
USE_ABS_FZ = True

EVENT_NAMES = ["idle", "approach", "under", "effective", "over", "retreat"]
# BGR colors for OpenCV
EVENT_COLORS = {
    "idle": (160, 160, 160),
    "approach": (255, 180, 0),
    "under": (0, 220, 255),
    "effective": (0, 200, 0),
    "over": (0, 0, 255),
    "retreat": (180, 0, 180),
}

EVENT_COLORS_RGB = {
    "idle":      (160, 160, 160),
    "approach":  (255, 165, 0),   # orange in RGB
    "under":     (0, 255, 255),   # cyan in RGB
    "effective": (0, 200, 0),
    "over":      (255, 0, 0),     # red in RGB
    "retreat":   (255, 0, 255),
}

# 自动转成 OpenCV 的 BGR
EVENT_COLORS = {k: (v[2], v[1], v[0]) for k, v in EVENT_COLORS_RGB.items()}


def get_episode_range(episode_ends: np.ndarray, episode_id: int) -> Tuple[int, int]:
    if episode_id < 0 or episode_id >= len(episode_ends):
        raise ValueError(f"episode_id out of range: {episode_id} (0..{len(episode_ends)-1})")
    start = int(0 if episode_id == 0 else episode_ends[episode_id - 1])
    end = int(episode_ends[episode_id])
    return start, end


def ensure_dir(p: str):
    d = os.path.dirname(p)
    if d:
        os.makedirs(d, exist_ok=True)


def draw_plot_panel(
    abs_fz_ep: np.ndarray,
    labels_ep: np.ndarray,
    idx: int,
    width: int,
    height: int,
    window: int = 240,
) -> np.ndarray:
    """
    Bottom panel:
      - colored event strip for last window frames
      - abs(Fz) curve (scaled)
      - vertical cursor at current idx
      - text overlay: current Fz + event name
    """
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    panel[:] = (20, 20, 20)

    T = len(abs_fz_ep)
    idx = int(np.clip(idx, 0, T - 1))

    w = min(window, T)
    left = max(0, idx - w + 1)
    right = idx + 1  # exclusive
    xs = np.arange(left, right)
    ys = abs_fz_ep[left:right]
    lbs = labels_ep[left:right]

    # scaling
    y_max = float(max(np.max(abs_fz_ep), 60.0))  # keep stable scale
    plot_top = 30
    plot_bottom = height - 35
    plot_left = 40
    plot_right = width - 20

    # axes
    cv2.rectangle(panel, (plot_left, plot_top), (plot_right, plot_bottom), (80, 80, 80), 1)
    cv2.putText(panel, f"|Fz| (N), max~{y_max:.1f}", (plot_left, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

    # event color strip
    strip_h = 12
    strip_y0 = plot_top + 4
    strip_y1 = strip_y0 + strip_h
    x_span = plot_right - plot_left

    if len(xs) > 1:
        for i in range(len(xs)):
            # map sample i to x pixel
            x0 = int(plot_left + (i / max(1, len(xs)-1)) * x_span)
            x1 = int(plot_left + ((i+1) / max(1, len(xs))) * x_span)
            ev_id = int(lbs[i])
            ev_name = EVENT_NAMES[ev_id] if 0 <= ev_id < len(EVENT_NAMES) else "unknown"
            c = EVENT_COLORS.get(ev_name, (255, 255, 255))
            cv2.rectangle(panel, (x0, strip_y0), (max(x0+1, x1), strip_y1), c, -1)

    # plot Fz curve
    if len(xs) >= 2:
        pts = []
        for i, f in enumerate(ys):
            x = int(plot_left + (i / max(1, len(xs)-1)) * x_span)
            # normalize y
            yn = float(f) / y_max
            y = int(plot_bottom - yn * (plot_bottom - (strip_y1 + 6)))
            pts.append((x, y))
        cv2.polylines(panel, [np.array(pts, dtype=np.int32)], False, (230, 230, 230), 2, cv2.LINE_AA)

    # cursor (current idx at rightmost of window)
    cursor_x = plot_right
    cv2.line(panel, (cursor_x, plot_top), (cursor_x, plot_bottom), (0, 255, 255), 2)

    # current values text
    cur_f = float(abs_fz_ep[idx])
    cur_ev_id = int(labels_ep[idx])
    cur_ev_name = EVENT_NAMES[cur_ev_id] if 0 <= cur_ev_id < len(EVENT_NAMES) else "unknown"
    cur_color = EVENT_COLORS.get(cur_ev_name, (255, 255, 255))

    cv2.putText(panel, f"t={idx:04d}/{T-1}  |Fz|={cur_f:6.2f} N  event={cur_ev_name}",
                (plot_left, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, cur_color, 2, cv2.LINE_AA)

    return panel


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zarr_path", required=True)
    ap.add_argument("--episode_id", type=int, default=0)
    ap.add_argument("--out_mp4", type=str, default="episode_viz.mp4")
    ap.add_argument("--fps", type=float, default=24.0, help="video fps (use 24 for your data)")
    ap.add_argument("--max_frames", type=int, default=-1, help="limit frames for quick test, -1 means all")
    ap.add_argument("--resize_w", type=int, default=320, help="per-image width after resize")
    ap.add_argument("--resize_h", type=int, default=240, help="per-image height after resize")
    ap.add_argument("--plot_h", type=int, default=180, help="height of plot panel")
    ap.add_argument("--chunk", type=int, default=64, help="zarr read chunk size")
    args = ap.parse_args()

    root = zarr.open(args.zarr_path, mode="r")

    # keys check
    for k in [WRENCH_KEY, WRIST_IMG_KEY, EXT_IMG_KEY, LABEL_KEY, EP_ENDS_KEY]:
        if k not in root:
            raise KeyError(f"Missing key: {k}")

    episode_ends = np.asarray(root[EP_ENDS_KEY][:], dtype=np.int64)
    s, e = get_episode_range(episode_ends, args.episode_id)
    T = e - s
    if args.max_frames > 0:
        T = min(T, args.max_frames)
        e = s + T

    # Load small arrays for episode
    wrench_ep = np.asarray(root[WRENCH_KEY][s:e], dtype=np.float32)      # [T,6]
    labels_ep = np.asarray(root[LABEL_KEY][s:e], dtype=np.int8)          # [T]
    fz = wrench_ep[:, FZ_INDEX]
    abs_fz_ep = np.abs(fz) if USE_ABS_FZ else fz

    # Video layout
    img_w, img_h = args.resize_w, args.resize_h
    top_w = img_w * 2
    top_h = img_h
    plot_h = args.plot_h
    out_w = top_w
    out_h = top_h + plot_h

    ensure_dir(args.out_mp4)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    vw = cv2.VideoWriter(args.out_mp4, fourcc, args.fps, (out_w, out_h))
    if not vw.isOpened():
        raise RuntimeError("Failed to open VideoWriter. Try different codec or path.")

    wrist_arr = root[WRIST_IMG_KEY]
    ext_arr = root[EXT_IMG_KEY]

    print(f"[INFO] episode={args.episode_id}, frames={T}, range=[{s},{e})")
    print(f"[INFO] writing -> {args.out_mp4}  size={out_w}x{out_h} fps={args.fps}")

    # stream images in chunks to avoid huge memory
    chunk = max(1, int(args.chunk))
    idx_local = 0
    while idx_local < T:
        j0 = idx_local
        j1 = min(T, j0 + chunk)
        g0 = s + j0
        g1 = s + j1

        wrist_chunk = np.asarray(wrist_arr[g0:g1], dtype=np.uint8)  # [B,H,W,3] RGB
        ext_chunk = np.asarray(ext_arr[g0:g1], dtype=np.uint8)

        for k in range(j1 - j0):
            i = j0 + k  # local index in episode
            wrist = wrist_chunk[k]
            ext = ext_chunk[k]

            # convert RGB->BGR for OpenCV and resize

            wrist_bgr = wrist.copy()  # treat stored image as BGR already
            ext_bgr = ext.copy()
            wrist_bgr = cv2.resize(wrist_bgr, (img_w, img_h), interpolation=cv2.INTER_AREA)
            ext_bgr = cv2.resize(ext_bgr, (img_w, img_h), interpolation=cv2.INTER_AREA)

            top = np.concatenate([wrist_bgr, ext_bgr], axis=1)

            # plot panel
            plot = draw_plot_panel(abs_fz_ep, labels_ep, idx=i, width=out_w, height=plot_h, window=240)

            # add titles on top images
            cv2.putText(top, "left_wrist_img", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(top, "external_img", (img_w + 10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            frame = np.concatenate([top, plot], axis=0)
            vw.write(frame)

        idx_local = j1

    vw.release()
    print("[DONE] Saved:", args.out_mp4)


if __name__ == "__main__":
    main()

# python /home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/viz_episode_wiping.py \
#   --zarr_path /home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr \
#   --episode_id 0 \
#   --out_mp4 /home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/outputs/viz/ep0_wiping_paep.mp4 \
#   --fps 24
