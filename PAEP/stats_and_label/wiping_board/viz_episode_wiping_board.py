#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Visualize wiping-board episode with NEW PAEP labels (NO argparse).

Reads from Zarr:
  - data/left_wrist_img
  - data/external_img
  - data/left_robot_tcp_wrench (Fz -> |Fz|)
  - data/paep_contact: 0=free, 1=contact
  - data/paep_phase:   0=approach, 1=progress, 2=done
  - meta/episode_ends

Outputs an MP4 that shows:
  - top row: wrist / external images
  - bottom panel: phase strip + contact strip + |Fz| curve + current label text
"""

import os
from typing import Tuple

import numpy as np
import zarr
import cv2


# =========================
# User config (edit here)
# =========================
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr"
EPISODE_ID = 48
OUT_MP4 = f"/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/label_viz/wiping_board/ep{EPISODE_ID}_wiping_paep.mp4"
VIDEO_FPS = 24.0
MAX_FRAMES = -1          # -1 = all frames in episode
CHUNK = 64               # zarr read chunk size
RESIZE_W = 320
RESIZE_H = 240
PLOT_H = 190
PLOT_WINDOW = 240        # how many frames to show in the bottom panel

# If images look RGB (wrong colors), set to True
IMAGES_ARE_RGB = False

# Keys
WRENCH_KEY = "data/left_robot_tcp_wrench"
WRIST_IMG_KEY = "data/left_wrist_img"
EXT_IMG_KEY = "data/external_img"
CONTACT_KEY = "data/paep_contact"
PHASE_KEY = "data/paep_phase"
EP_ENDS_KEY = "meta/episode_ends"

FZ_INDEX = 2
USE_ABS_FZ = True

CONTACT_NAMES = ["free", "contact"]
PHASE_NAMES = ["approach", "progress", "done"]

# Colors (RGB -> will convert to OpenCV BGR)
CONTACT_COLORS_RGB = {
    "free":    (160, 160, 160),
    "contact": (0, 200, 0),
}
PHASE_COLORS_RGB = {
    "approach": (255, 165, 0),   # orange
    "progress": (0, 200, 0),     # green
    "done":     (160, 0, 200),   # purple
}
CONTACT_COLORS = {k: (v[2], v[1], v[0]) for k, v in CONTACT_COLORS_RGB.items()}
PHASE_COLORS = {k: (v[2], v[1], v[0]) for k, v in PHASE_COLORS_RGB.items()}


def ensure_dir(path: str):
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)


def get_episode_range(episode_ends: np.ndarray, episode_id: int) -> Tuple[int, int]:
    if episode_id < 0 or episode_id >= len(episode_ends):
        raise ValueError(f"episode_id out of range: {episode_id} (0..{len(episode_ends)-1})")
    start = int(0 if episode_id == 0 else episode_ends[episode_id - 1])
    end = int(episode_ends[episode_id])
    return start, end


def _draw_strip(panel, x0, y0, x1, y1, ids, names, colors):
    """Draw a colored strip for ids over current window samples."""
    n = len(ids)
    if n <= 0:
        return
    span = x1 - x0
    for i in range(n):
        px0 = int(x0 + (i / max(1, n - 1)) * span)
        px1 = int(x0 + ((i + 1) / max(1, n)) * span)
        _id = int(ids[i])
        name = names[_id] if 0 <= _id < len(names) else "unknown"
        c = colors.get(name, (255, 255, 255))
        cv2.rectangle(panel, (px0, y0), (max(px0 + 1, px1), y1), c, -1)


def draw_plot_panel(
    abs_fz_ep: np.ndarray,
    contact_ep: np.ndarray,
    phase_ep: np.ndarray,
    idx: int,
    width: int,
    height: int,
    window: int,
) -> np.ndarray:
    """
    Bottom panel:
      - phase strip
      - contact strip
      - |Fz| curve (last `window` frames)
      - vertical cursor at current idx
      - text: current |Fz| + phase/contact
    """
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    panel[:] = (20, 20, 20)

    T = len(abs_fz_ep)
    idx = int(np.clip(idx, 0, T - 1))

    w = min(window, T)
    left = max(0, idx - w + 1)
    right = idx + 1  # exclusive

    ys = abs_fz_ep[left:right]
    phase_win = phase_ep[left:right]
    contact_win = contact_ep[left:right]

    # scaling
    y_max = float(max(np.max(abs_fz_ep), 60.0))  # stabilize scale
    plot_top = 34
    plot_bottom = height - 38
    plot_left = 40
    plot_right = width - 20

    # frame box
    cv2.rectangle(panel, (plot_left, plot_top), (plot_right, plot_bottom), (80, 80, 80), 1)
    cv2.putText(panel, f"|Fz| (N), max~{y_max:.1f}", (plot_left, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)

    # strips
    strip_h = 10
    gap = 3
    phase_y0 = plot_top + 4
    phase_y1 = phase_y0 + strip_h
    contact_y0 = phase_y1 + gap
    contact_y1 = contact_y0 + strip_h

    _draw_strip(panel, plot_left, phase_y0, plot_right, phase_y1, phase_win, PHASE_NAMES, PHASE_COLORS)
    _draw_strip(panel, plot_left, contact_y0, plot_right, contact_y1, contact_win, CONTACT_NAMES, CONTACT_COLORS)

    cv2.putText(panel, "phase", (5, phase_y1), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(panel, "contact", (5, contact_y1), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1, cv2.LINE_AA)

    # curve below strips
    curve_top = contact_y1 + 8
    if len(ys) >= 2:
        pts = []
        x_span = plot_right - plot_left
        for i, f in enumerate(ys):
            x = int(plot_left + (i / max(1, len(ys) - 1)) * x_span)
            yn = float(f) / y_max
            y = int(plot_bottom - yn * (plot_bottom - curve_top))
            pts.append((x, y))
        cv2.polylines(panel, [np.array(pts, dtype=np.int32)], False, (230, 230, 230), 2, cv2.LINE_AA)

    # cursor (current idx is at the right edge of window)
    cv2.line(panel, (plot_right, plot_top), (plot_right, plot_bottom), (0, 255, 255), 2)

    # text
    cur_f = float(abs_fz_ep[idx])
    cur_phase_id = int(phase_ep[idx])
    cur_contact_id = int(contact_ep[idx])

    cur_phase = PHASE_NAMES[cur_phase_id] if 0 <= cur_phase_id < len(PHASE_NAMES) else "unknown"
    cur_contact = CONTACT_NAMES[cur_contact_id] if 0 <= cur_contact_id < len(CONTACT_NAMES) else "unknown"

    cur_color = PHASE_COLORS.get(cur_phase, (255, 255, 255))
    cv2.putText(panel,
                f"t={idx:04d}/{T-1}  |Fz|={cur_f:6.2f}N  phase={cur_phase}  contact={cur_contact}",
                (plot_left, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, cur_color, 2, cv2.LINE_AA)
    return panel


def maybe_rgb_to_bgr(img: np.ndarray) -> np.ndarray:
    if not IMAGES_ARE_RGB:
        return img
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def main():
    root = zarr.open(ZARR_PATH, mode="r")

    # keys check
    for k in [WRENCH_KEY, WRIST_IMG_KEY, EXT_IMG_KEY, CONTACT_KEY, PHASE_KEY, EP_ENDS_KEY]:
        if k not in root:
            raise KeyError(f"Missing key: {k}")

    episode_ends = np.asarray(root[EP_ENDS_KEY][:], dtype=np.int64)
    s, e = get_episode_range(episode_ends, EPISODE_ID)
    T = e - s
    if MAX_FRAMES > 0:
        T = min(T, MAX_FRAMES)
        e = s + T

    # load small arrays for episode
    wrench_ep = np.asarray(root[WRENCH_KEY][s:e], dtype=np.float32)    # [T,6]
    contact_ep = np.asarray(root[CONTACT_KEY][s:e], dtype=np.int8)     # [T]
    phase_ep = np.asarray(root[PHASE_KEY][s:e], dtype=np.int8)         # [T]

    fz = wrench_ep[:, FZ_INDEX]
    abs_fz_ep = np.abs(fz) if USE_ABS_FZ else fz

    # video layout
    img_w, img_h = RESIZE_W, RESIZE_H
    top_w = img_w * 2
    top_h = img_h
    plot_h = PLOT_H
    out_w = top_w
    out_h = top_h + plot_h

    ensure_dir(OUT_MP4)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    vw = cv2.VideoWriter(OUT_MP4, fourcc, VIDEO_FPS, (out_w, out_h))
    if not vw.isOpened():
        raise RuntimeError("Failed to open VideoWriter. Try different output path/codec.")

    wrist_arr = root[WRIST_IMG_KEY]
    ext_arr = root[EXT_IMG_KEY]

    print(f"[INFO] episode={EPISODE_ID}, frames={T}, range=[{s},{e})")
    print(f"[INFO] writing -> {OUT_MP4}  size={out_w}x{out_h} fps={VIDEO_FPS}")

    chunk = max(1, int(CHUNK))
    idx_local = 0
    while idx_local < T:
        j0 = idx_local
        j1 = min(T, j0 + chunk)
        g0 = s + j0
        g1 = s + j1

        wrist_chunk = np.asarray(wrist_arr[g0:g1], dtype=np.uint8)
        ext_chunk = np.asarray(ext_arr[g0:g1], dtype=np.uint8)

        for k in range(j1 - j0):
            i = j0 + k  # local index in episode
            wrist = maybe_rgb_to_bgr(wrist_chunk[k])
            ext = maybe_rgb_to_bgr(ext_chunk[k])

            wrist = cv2.resize(wrist, (img_w, img_h), interpolation=cv2.INTER_AREA)
            ext = cv2.resize(ext, (img_w, img_h), interpolation=cv2.INTER_AREA)

            top = np.concatenate([wrist, ext], axis=1)

            # titles
            cv2.putText(top, "left_wrist_img", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(top, "external_img", (img_w + 10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)

            # plot
            plot = draw_plot_panel(abs_fz_ep, contact_ep, phase_ep, idx=i, width=out_w, height=plot_h, window=PLOT_WINDOW)

            frame = np.concatenate([top, plot], axis=0)
            vw.write(frame)

        idx_local = j1

    vw.release()
    print("[DONE] Saved:", OUT_MP4)


if __name__ == "__main__":
    main()
