#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Visualize plug-in-charger episodes with PAEP labels (NO argparse, supports episode id range).

Reads from Zarr:
  - data/left_wrist_img
  - data/right_wrist_img
  - data/external_img
  - data/left_robot_tcp_wrench
  - data/paep_contact: 0=free, 1=contact
  - data/paep_phase:   0=approach, 1=search, 2=recovery, 3=insert
  - meta/episode_ends

Outputs:
  - Either one mp4 per episode (default), or a single concatenated mp4.
"""

import os
from typing import Tuple, List

import numpy as np
import zarr
import cv2


# =========================
# 配置区：你只改这里
# =========================
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_usb/plug_in_usb_stream_downsample1_zarr/replay_buffer.zarr"

# 选择 episode 范围（包含两端）
EPISODE_ID_START = 0
EPISODE_ID_END = 10

# 输出模式
#   "per_episode": 每个 episode 单独一个 mp4（推荐）
#   "concat":      把范围内所有 episode 串到一个 mp4
MODE = "per_episode"

OUT_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/label_viz/plug_in_usb"

# concat 模式的输出文件名
OUT_MP4_CONCAT = f"{OUT_DIR}/ep{EPISODE_ID_START:03d}_to_ep{EPISODE_ID_END:03d}_plug_paep.mp4"

VIDEO_FPS = 24.0
MAX_FRAMES_PER_EPISODE = -1   # -1 = 全部帧；>0 = 每个 episode 最多可视化多少帧（截断）
CHUNK = 64                    # zarr read chunk size

RESIZE_W = 320
RESIZE_H = 240

PLOT_H = 210
PLOT_WINDOW = 240             # bottom panel shows last N frames

# 如果图像颜色不对（RGB 被当 BGR），设 True
IMAGES_ARE_RGB = False

# Keys (你的数据在 /data 下)
WRENCH_KEY = "data/left_robot_tcp_wrench"
LEFT_WRIST_IMG_KEY = "data/left_wrist_img"
RIGHT_WRIST_IMG_KEY = "data/right_wrist_img"
EXT_IMG_KEY = "data/external_img"
CONTACT_KEY = "data/paep_contact"
PHASE_KEY = "data/paep_phase"
EP_ENDS_KEY = "meta/episode_ends"

FZ_INDEX = 2
USE_ABS_FZ = True

CONTACT_NAMES = ["free", "contact"]
PHASE_NAMES = ["approach", "search", "recovery", "insert"]

# Colors (RGB -> convert to OpenCV BGR)
CONTACT_COLORS_RGB = {
    "free":    (160, 160, 160),
    "contact": (0, 200, 0),
}
PHASE_COLORS_RGB = {
    "approach":  (255, 165, 0),   # orange
    "search":    (0, 200, 0),     # green
    "recovery":  (0, 180, 255),   # cyan-ish
    "insert":    (180, 0, 255),   # purple-ish
}
CONTACT_COLORS = {k: (v[2], v[1], v[0]) for k, v in CONTACT_COLORS_RGB.items()}
PHASE_COLORS = {k: (v[2], v[1], v[0]) for k, v in PHASE_COLORS_RGB.items()}

# 曲线颜色（BGR）
CURVE_FZ = (230, 230, 230)       # |Fz|
CURVE_FN = (120, 220, 255)       # ||F||
# =========================


def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)


def get_episode_range(episode_ends: np.ndarray, episode_id: int) -> Tuple[int, int]:
    if episode_id < 0 or episode_id >= len(episode_ends):
        raise ValueError(f"episode_id out of range: {episode_id} (0..{len(episode_ends)-1})")
    start = int(0 if episode_id == 0 else episode_ends[episode_id - 1])
    end = int(episode_ends[episode_id])
    return start, end


def _draw_strip(panel, x0, y0, x1, y1, ids, names, colors):
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


def maybe_rgb_to_bgr(img: np.ndarray) -> np.ndarray:
    if not IMAGES_ARE_RGB:
        return img
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


def _polyline(panel, ys, x0, x1, y_top, y_bottom, y_max, color, thickness=2):
    if len(ys) < 2:
        return
    pts = []
    x_span = x1 - x0
    y_span = y_bottom - y_top
    for i, v in enumerate(ys):
        x = int(x0 + (i / max(1, len(ys) - 1)) * x_span)
        yn = float(v) / max(1e-6, y_max)
        y = int(y_bottom - yn * y_span)
        pts.append((x, y))
    cv2.polylines(panel, [np.array(pts, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)


def draw_plot_panel(
    abs_fz_ep: np.ndarray,
    f_norm_ep: np.ndarray,
    contact_ep: np.ndarray,
    phase_ep: np.ndarray,
    idx: int,
    width: int,
    height: int,
    window: int,
    episode_id: int,
) -> np.ndarray:
    panel = np.zeros((height, width, 3), dtype=np.uint8)
    panel[:] = (20, 20, 20)

    T = len(abs_fz_ep)
    idx = int(np.clip(idx, 0, T - 1))

    w = min(window, T)
    left = max(0, idx - w + 1)
    right = idx + 1

    abs_fz_win = abs_fz_ep[left:right]
    f_norm_win = f_norm_ep[left:right]
    phase_win = phase_ep[left:right]
    contact_win = contact_ep[left:right]

    y_max = float(max(np.max(f_norm_ep), np.max(abs_fz_ep), 80.0))

    plot_top = 38
    plot_bottom = height - 42
    plot_left = 40
    plot_right = width - 20

    cv2.rectangle(panel, (plot_left, plot_top), (plot_right, plot_bottom), (80, 80, 80), 1)
    cv2.putText(panel, f"Episode {episode_id} | Force (N) scale~{y_max:.1f}", (plot_left, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.58, (220, 220, 220), 1, cv2.LINE_AA)

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

    curve_top = contact_y1 + 10
    _polyline(panel, abs_fz_win, plot_left, plot_right, curve_top, plot_bottom, y_max, CURVE_FZ, thickness=2)
    _polyline(panel, f_norm_win, plot_left, plot_right, curve_top, plot_bottom, y_max, CURVE_FN, thickness=2)

    # cursor at right edge
    cv2.line(panel, (plot_right, plot_top), (plot_right, plot_bottom), (0, 255, 255), 2)

    cv2.putText(panel, "|Fz|", (plot_right - 120, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, CURVE_FZ, 1, cv2.LINE_AA)
    cv2.putText(panel, "||F||", (plot_right - 60, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, CURVE_FN, 1, cv2.LINE_AA)

    cur_fz = float(abs_fz_ep[idx])
    cur_fn = float(f_norm_ep[idx])
    cur_phase_id = int(phase_ep[idx])
    cur_contact_id = int(contact_ep[idx])
    cur_phase = PHASE_NAMES[cur_phase_id] if 0 <= cur_phase_id < len(PHASE_NAMES) else "unknown"
    cur_contact = CONTACT_NAMES[cur_contact_id] if 0 <= cur_contact_id < len(CONTACT_NAMES) else "unknown"
    cur_color = PHASE_COLORS.get(cur_phase, (255, 255, 255))

    cv2.putText(panel,
                f"t={idx:04d}/{T-1}  |Fz|={cur_fz:6.2f}N  ||F||={cur_fn:6.2f}N  phase={cur_phase}  contact={cur_contact}",
                (plot_left, height - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, cur_color, 2, cv2.LINE_AA)
    return panel


def open_writer(path: str, w: int, h: int, fps: float) -> cv2.VideoWriter:
    ensure_dir(os.path.dirname(path))
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    vw = cv2.VideoWriter(path, fourcc, fps, (w, h))
    if not vw.isOpened():
        raise RuntimeError(f"Failed to open VideoWriter: {path}")
    return vw


def visualize_one_episode(root, episode_ends: np.ndarray, episode_id: int, writer: cv2.VideoWriter = None) -> str:
    # compute range
    s, e = get_episode_range(episode_ends, episode_id)
    T_full = e - s
    if MAX_FRAMES_PER_EPISODE > 0:
        e = min(e, s + MAX_FRAMES_PER_EPISODE)
    T = e - s

    # load arrays for episode (small, ok)
    wrench_ep = np.asarray(root[WRENCH_KEY][s:e], dtype=np.float32)
    contact_ep = np.asarray(root[CONTACT_KEY][s:e], dtype=np.int8)
    phase_ep = np.asarray(root[PHASE_KEY][s:e], dtype=np.int8)

    Fx, Fy, Fz = wrench_ep[:, 0], wrench_ep[:, 1], wrench_ep[:, 2]
    f_norm_ep = np.sqrt(Fx * Fx + Fy * Fy + Fz * Fz)
    abs_fz_ep = np.abs(Fz) if USE_ABS_FZ else Fz

    # layout
    img_w, img_h = RESIZE_W, RESIZE_H
    out_w = img_w * 3
    out_h = img_h + PLOT_H

    # writer
    close_after = False
    if writer is None:
        out_path = os.path.join(OUT_DIR, f"ep{episode_id:03d}_plug_paep.mp4")
        writer = open_writer(out_path, out_w, out_h, VIDEO_FPS)
        close_after = True
    else:
        out_path = OUT_MP4_CONCAT  # for logging only

    left_arr = root[LEFT_WRIST_IMG_KEY]
    right_arr = root[RIGHT_WRIST_IMG_KEY]
    ext_arr = root[EXT_IMG_KEY]

    print(f"[INFO] episode={episode_id}, frames={T}/{T_full}, range=[{s},{e}) -> {out_path}")

    chunk = max(1, int(CHUNK))
    idx_local = 0
    while idx_local < T:
        j0 = idx_local
        j1 = min(T, j0 + chunk)
        g0 = s + j0
        g1 = s + j1

        left_chunk = np.asarray(left_arr[g0:g1], dtype=np.uint8)
        right_chunk = np.asarray(right_arr[g0:g1], dtype=np.uint8)
        ext_chunk = np.asarray(ext_arr[g0:g1], dtype=np.uint8)

        for k in range(j1 - j0):
            i = j0 + k

            left = maybe_rgb_to_bgr(left_chunk[k])
            right = maybe_rgb_to_bgr(right_chunk[k])
            ext = maybe_rgb_to_bgr(ext_chunk[k])

            left = cv2.resize(left, (img_w, img_h), interpolation=cv2.INTER_AREA)
            right = cv2.resize(right, (img_w, img_h), interpolation=cv2.INTER_AREA)
            ext = cv2.resize(ext, (img_w, img_h), interpolation=cv2.INTER_AREA)

            top = np.concatenate([left, right, ext], axis=1)

            cv2.putText(top, f"left_wrist_img (ep{episode_id:03d})", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(top, "right_wrist_img", (img_w + 10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(top, "external_img", (img_w * 2 + 10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)

            plot = draw_plot_panel(
                abs_fz_ep, f_norm_ep, contact_ep, phase_ep,
                idx=i, width=out_w, height=PLOT_H, window=PLOT_WINDOW, episode_id=episode_id
            )

            frame = np.concatenate([top, plot], axis=0)
            writer.write(frame)

        idx_local = j1

    if close_after:
        writer.release()
        print("[DONE] Saved:", os.path.join(OUT_DIR, f"ep{episode_id:03d}_plug_paep.mp4"))

    return out_path


def main():
    ensure_dir(OUT_DIR)

    root = zarr.open(ZARR_PATH, mode="r")
    for k in [WRENCH_KEY, LEFT_WRIST_IMG_KEY, RIGHT_WRIST_IMG_KEY, EXT_IMG_KEY, CONTACT_KEY, PHASE_KEY, EP_ENDS_KEY]:
        if k not in root:
            raise KeyError(f"Missing key: {k}")

    episode_ends = np.asarray(root[EP_ENDS_KEY][:], dtype=np.int64)
    n_eps = len(episode_ends)

    s_id = int(max(0, EPISODE_ID_START))
    e_id = int(min(n_eps - 1, EPISODE_ID_END))
    if s_id > e_id:
        raise ValueError(f"Invalid range: start={EPISODE_ID_START}, end={EPISODE_ID_END}, n_eps={n_eps}")

    ep_ids = list(range(s_id, e_id + 1))

    if MODE == "concat":
        # open one writer and stream all episodes
        img_w, img_h = RESIZE_W, RESIZE_H
        out_w = img_w * 3
        out_h = img_h + PLOT_H
        vw = open_writer(OUT_MP4_CONCAT, out_w, out_h, VIDEO_FPS)

        print(f"[INFO] MODE=concat, episodes={ep_ids} -> {OUT_MP4_CONCAT}")
        for eid in ep_ids:
            visualize_one_episode(root, episode_ends, eid, writer=vw)
        vw.release()
        print("[DONE] Saved:", OUT_MP4_CONCAT)

    else:
        print(f"[INFO] MODE=per_episode, episodes={ep_ids} -> {OUT_DIR}")
        for eid in ep_ids:
            visualize_one_episode(root, episode_ends, eid, writer=None)


if __name__ == "__main__":
    main()
