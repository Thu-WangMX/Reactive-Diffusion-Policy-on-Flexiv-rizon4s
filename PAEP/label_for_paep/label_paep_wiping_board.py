#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PAEP auto labeling for wiping-board (v3).

Writes per-frame event labels into Zarr:
  data/paep_event: int8 array shape (N,)

Event id mapping:
  0 idle
  1 approach
  2 under
  3 effective
  4 over
  5 retreat

Key improvements:
- After approach_start, NEVER produce idle again.
- retreat is triggered ONLY AFTER contact end, with BOTH:
    (a) low-force confirmation (absFz below contact_exit for some frames)
    (b) clear continuous rising of TCP z (with margin after end)
"""

import argparse
import json
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import zarr


# -----------------------------
# Fixed configuration (NO CLI)
# -----------------------------
WRENCH_KEY = "data/left_robot_tcp_wrench"
TCP_POSE_KEY = "data/left_robot_tcp_pose"   # needed for retreat detection (z)
EPISODE_ENDS_KEY = "meta/episode_ends"
OUT_LABEL_KEY = "data/paep_event"

FZ_INDEX = 2
USE_ABS_FZ = True

FPS_FIXED = 24.0  # don't estimate from timestamp

EVENT_NAMES = ["idle", "approach", "under", "effective", "over", "retreat"]
E = {n: i for i, n in enumerate(EVENT_NAMES)}

PRE_WIPE_LABEL = "idle"

# Timing
APPROACH_PRE_SEC = 1.0     # label 1s before onset as approach
CONTACT_MIN_SEC = 0.20     # onset/end require >=0.2s stable frames

# Thresholds
CONTACT_ENTER = 8.0
CONTACT_EXIT = 4.0

EFF_LOW_ENTER = 15.0
EFF_HIGH_ENTER = 40.0
EFF_LOW_EXIT = 13.0
EFF_HIGH_EXIT = 42.0

OVER_ENTER = 45.0
OVER_EXIT = 42.0

# -----------------------------
# Retreat constraints (NEW)
# -----------------------------
# Only search retreat after (end + margin)
RETREAT_MARGIN_SEC = 0.35          # wait ~0.35s after end before checking retreat
RETREAT_WINDOW_SEC = 0.30          # window to measure rising z
RETREAT_MIN_RISE_M = 0.010         # total rise >= 1 cm
RETREAT_MIN_POS_FRAC = 0.70        # fraction of dz>0 in window

# Low-force confirmation (prevents "retreat" inside contact)
RETREAT_REQUIRE_LOW_FORCE = True
RETREAT_LOW_FORCE_TH = CONTACT_EXIT  # use same as contact_exit (6N)
RETREAT_LOW_FORCE_SEC = 0.25         # require ~0.25s consecutive low force


@dataclass
class Thr:
    contact_enter: float = CONTACT_ENTER
    contact_exit: float = CONTACT_EXIT
    eff_low_enter: float = EFF_LOW_ENTER
    eff_high_enter: float = EFF_HIGH_ENTER
    eff_low_exit: float = EFF_LOW_EXIT
    eff_high_exit: float = EFF_HIGH_EXIT
    over_enter: float = OVER_ENTER
    over_exit: float = OVER_EXIT


def _find_contact_onset(x: np.ndarray, th_enter: float, min_frames: int) -> Optional[int]:
    """First index where x > th_enter for >= min_frames consecutive frames."""
    if len(x) < min_frames:
        return None
    above = x > th_enter
    cnt = 0
    for i, a in enumerate(above):
        cnt = cnt + 1 if a else 0
        if cnt >= min_frames:
            return i - min_frames + 1
    return None


def _find_contact_end(x: np.ndarray, th_exit: float, min_frames: int) -> Optional[int]:
    """Return last index of the last run where x > th_exit for >= min_frames."""
    if len(x) < min_frames:
        return None
    above = x > th_exit
    runs: List[Tuple[int, int]] = []
    cnt = 0
    start = None
    for i, a in enumerate(above):
        if a:
            if cnt == 0:
                start = i
            cnt += 1
        else:
            if cnt >= min_frames:
                runs.append((start, i - 1))
            cnt = 0
            start = None
    if cnt >= min_frames:
        runs.append((start, len(above) - 1))
    if not runs:
        return None
    return runs[-1][1]


def _scan_quality(x: np.ndarray, thr: Thr) -> np.ndarray:
    """
    Assign under/effective/over over a segment with hysteresis.
    Output is event ids (int8).
    """
    out = np.empty((len(x),), dtype=np.int8)

    def init_state(f: float) -> int:
        if f >= thr.over_enter:
            return E["over"]
        if thr.eff_low_enter <= f <= thr.eff_high_enter:
            return E["effective"]
        return E["under"]

    state = init_state(float(x[0]))
    out[0] = state

    for i in range(1, len(x)):
        f = float(x[i])

        if state == E["over"]:
            if f < thr.over_exit:
                if thr.eff_low_exit <= f <= thr.eff_high_exit:
                    state = E["effective"]
                else:
                    state = E["under"]

        elif state == E["effective"]:
            if f >= thr.over_enter:
                state = E["over"]
            elif not (thr.eff_low_exit <= f <= thr.eff_high_exit):
                state = E["under"]

        else:  # under
            if f >= thr.over_enter:
                state = E["over"]
            elif thr.eff_low_enter <= f <= thr.eff_high_enter:
                state = E["effective"]
            else:
                state = E["under"]

        out[i] = state

    return out


def _detect_retreat_start(
    abs_fz: np.ndarray,
    tcp_z: np.ndarray,
    end_idx: int,
    fps: float,
) -> Optional[int]:
    """
    Detect retreat start ONLY after contact end, using:
      - margin after end
      - low-force confirmation
      - clear rising z in a short window
    """
    T = len(tcp_z)
    if end_idx >= T - 2:
        return None

    margin = int(round(RETREAT_MARGIN_SEC * fps))
    W = max(2, int(round(RETREAT_WINDOW_SEC * fps)))
    lowK = int(round(RETREAT_LOW_FORCE_SEC * fps)) if RETREAT_REQUIRE_LOW_FORCE else 0

    # Start search strictly after end + margin
    check_start = min(T - 1, end_idx + 1 + margin)
    if check_start >= T - W - 1:
        return None

    for t in range(check_start, T - W - 1):
        # Low-force gating: require absFz continuously low
        if RETREAT_REQUIRE_LOW_FORCE and lowK > 0:
            if t + lowK >= T:
                break
            if not np.all(abs_fz[t:t + lowK] < RETREAT_LOW_FORCE_TH):
                continue

        z0 = float(tcp_z[t])
        z1 = float(tcp_z[t + W])
        rise = z1 - z0
        if rise < RETREAT_MIN_RISE_M:
            continue

        dz = np.diff(tcp_z[t:t + W + 1].astype(np.float32))
        pos_frac = float(np.mean(dz > 0.0))
        if pos_frac >= RETREAT_MIN_POS_FRAC:
            return t

    return None


def label_episode(abs_fz: np.ndarray, tcp_z: np.ndarray, fps: float, thr: Thr) -> Tuple[np.ndarray, Dict]:
    T = len(abs_fz)
    labels = np.full((T,), E[PRE_WIPE_LABEL], dtype=np.int8)

    min_frames = max(1, int(round(CONTACT_MIN_SEC * fps)))
    onset = _find_contact_onset(abs_fz, thr.contact_enter, min_frames=min_frames)
    end = _find_contact_end(abs_fz, thr.contact_exit, min_frames=min_frames)

    # If no stable contact, keep all PRE_WIPE_LABEL
    if onset is None or end is None or end <= onset:
        stats = {"onset": None, "end": None, "note": "no_stable_contact"}
        return labels, stats

    # Approach start (backfill)
    pre_frames = int(round(APPROACH_PRE_SEC * fps))
    approach_start = max(0, onset - pre_frames)

    # After approach_start -> wiping stage, no more idle
    labels[approach_start:] = E["under"]
    labels[approach_start:onset] = E["approach"]

    # Contact quality segment
    quality = _scan_quality(abs_fz[onset:end + 1], thr)
    labels[onset:end + 1] = quality

    # Retreat detection (strictly AFTER end, with low-force + rising z)
    retreat_start = _detect_retreat_start(abs_fz=abs_fz, tcp_z=tcp_z, end_idx=end, fps=fps)
    if retreat_start is not None and retreat_start < T:
        labels[retreat_start:] = E["retreat"]

    stats = {
        "onset": int(onset),
        "end": int(end),
        "approach_start": int(approach_start),
        "contact_min_frames": int(min_frames),
        "retreat_start": (int(retreat_start) if retreat_start is not None else None),
    }
    return labels, stats


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zarr_path", required=True)
    args = ap.parse_args()

    root = zarr.open(args.zarr_path, mode="a")

    # sanity checks
    for k in [WRENCH_KEY, TCP_POSE_KEY, EPISODE_ENDS_KEY]:
        if k not in root:
            raise KeyError(f"Missing key: {k}. Try root.tree() to inspect.")

    wrench = np.asarray(root[WRENCH_KEY][:], dtype=np.float32)
    tcp_pose = np.asarray(root[TCP_POSE_KEY][:], dtype=np.float32)
    episode_ends = np.asarray(root[EPISODE_ENDS_KEY][:], dtype=np.int64)

    if wrench.ndim != 2 or wrench.shape[1] < 6:
        raise ValueError(f"{WRENCH_KEY} expected shape [N,6], got {wrench.shape}")
    if tcp_pose.ndim != 2 or tcp_pose.shape[1] < 3:
        raise ValueError(f"{TCP_POSE_KEY} expected shape [N,>=3], got {tcp_pose.shape}")

    N = wrench.shape[0]
    if tcp_pose.shape[0] != N:
        raise ValueError(f"Length mismatch: wrench N={N}, tcp_pose N={tcp_pose.shape[0]}")
    if episode_ends.ndim != 1 or len(episode_ends) == 0 or int(episode_ends[-1]) != N:
        raise ValueError(
            f"{EPISODE_ENDS_KEY} must end with N={N}, got last={episode_ends[-1] if len(episode_ends) else None}"
        )

    # Extract abs(Fz) and tcp z
    fz = wrench[:, FZ_INDEX].astype(np.float32)
    abs_fz = np.abs(fz) if USE_ABS_FZ else fz
    tcp_z = tcp_pose[:, 2].astype(np.float32)

    labels_all = np.full((N,), E[PRE_WIPE_LABEL], dtype=np.int8)

    ep_starts = np.concatenate([[0], episode_ends[:-1]])
    thr = Thr()
    stats_list: List[Dict] = []

    for ei, (s, e) in enumerate(zip(ep_starts, episode_ends)):
        lab, st = label_episode(
            abs_fz=abs_fz[s:e],
            tcp_z=tcp_z[s:e],
            fps=FPS_FIXED,
            thr=thr
        )
        labels_all[s:e] = lab
        st.update({"episode_id": int(ei), "start": int(s), "end_frame": int(e)})
        stats_list.append(st)

    # write to zarr
    data_grp = root.require_group("data")
    out_name = OUT_LABEL_KEY.split("/", 1)[1] if OUT_LABEL_KEY.startswith("data/") else OUT_LABEL_KEY
    if out_name in data_grp:
        del data_grp[out_name]
    data_grp.create_dataset(
        out_name,
        data=labels_all,
        dtype=np.int8,
        chunks=(min(8192, N),),
        overwrite=True,
    )

    # meta info
    meta_grp = root.require_group("meta")
    paep_grp = meta_grp.require_group("paep")
    paep_grp.attrs["event_names"] = EVENT_NAMES
    paep_grp.attrs["event2id"] = E
    paep_grp.attrs["fps_fixed"] = float(FPS_FIXED)
    paep_grp.attrs["fz_index"] = int(FZ_INDEX)
    paep_grp.attrs["use_abs_fz"] = bool(USE_ABS_FZ)
    paep_grp.attrs["pre_wipe_label"] = PRE_WIPE_LABEL

    paep_grp.attrs["timing_sec"] = {
        "approach_pre_sec": float(APPROACH_PRE_SEC),
        "contact_min_sec": float(CONTACT_MIN_SEC),
        "retreat_margin_sec": float(RETREAT_MARGIN_SEC),
        "retreat_window_sec": float(RETREAT_WINDOW_SEC),
        "retreat_low_force_sec": float(RETREAT_LOW_FORCE_SEC),
        "retreat_min_rise_m": float(RETREAT_MIN_RISE_M),
        "retreat_min_pos_frac": float(RETREAT_MIN_POS_FRAC),
    }
    paep_grp.attrs["thresholds"] = {
        "contact_enter": float(thr.contact_enter),
        "contact_exit": float(thr.contact_exit),
        "eff_low_enter": float(thr.eff_low_enter),
        "eff_high_enter": float(thr.eff_high_enter),
        "eff_low_exit": float(thr.eff_low_exit),
        "eff_high_exit": float(thr.eff_high_exit),
        "over_enter": float(thr.over_enter),
        "over_exit": float(thr.over_exit),
        "retreat_low_force_th": float(RETREAT_LOW_FORCE_TH),
    }

    counts = {EVENT_NAMES[i]: int(np.sum(labels_all == i)) for i in range(len(EVENT_NAMES))}
    paep_grp.attrs["counts_v3"] = counts
    paep_grp.attrs["episode_stats_json_v3"] = json.dumps(stats_list)

    print("[INFO] Wrote labels:", OUT_LABEL_KEY)
    print("[INFO] Event counts:", counts)
    if stats_list:
        print("[INFO] Episode[0] stats:", stats_list[0])
    print("[INFO] Using keys:", WRENCH_KEY, TCP_POSE_KEY)
    print("[INFO] FZ_INDEX=", FZ_INDEX, "abs=", USE_ABS_FZ, "fps=", FPS_FIXED)
    print("[INFO] Thresholds:", paep_grp.attrs["thresholds"])
    print("[INFO] Retreat constraints:", paep_grp.attrs["timing_sec"])


if __name__ == "__main__":
    main()



# python /home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/label_paep_wiping_board.py   --zarr_path /home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr