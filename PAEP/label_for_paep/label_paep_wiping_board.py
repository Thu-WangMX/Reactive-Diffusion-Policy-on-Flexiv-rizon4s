#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PAEP auto labeling for wiping-board (v4, factorized labels, NO argparse).

Writes per-frame labels into Zarr:
  data/paep_contact: int8 array shape (N,)
      0 = free
      1 = contact

  data/paep_phase: int8 array shape (N,)
      0 = approach
      1 = progress
      2 = done

Definition (per episode):
- contact is detected by abs(Fz) hysteresis + min duration:
    onset = first index where absFz > CONTACT_ENTER for >= CONTACT_MIN_SEC
    end   = last index of the last run where absFz > CONTACT_EXIT for >= CONTACT_MIN_SEC

- phase:
    approach_start = max(0, onset - APPROACH_PRE_SEC*fps)
    approach: [0, approach_start)
    progress: [approach_start, end]
    done: (end, T)

If no stable contact in an episode:
    contact = free for all frames
    phase   = approach for all frames
"""

import json
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import zarr

# =========================
# User config (edit here)
# =========================
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr"

WRENCH_KEY = "data/left_robot_tcp_wrench"
EPISODE_ENDS_KEY = "meta/episode_ends"

OUT_CONTACT_KEY = "data/paep_contact"
OUT_PHASE_KEY   = "data/paep_phase"

FZ_INDEX = 2
USE_ABS_FZ = True

FPS_FIXED = 24.0  # keep fixed; do not estimate from timestamp

# Timing
APPROACH_PRE_SEC = 1.0     # backfill approach 1s before contact onset
CONTACT_MIN_SEC  = 0.20    # onset/end need >= 0.2s stable frames

# Thresholds (recommended starting point from your |Fz| stats)
# Meaning:
#  - CONTACT_ENTER: require stronger force to ENTER contact (avoid false positives)
#  - CONTACT_EXIT : allow lower force to remain in contact, and define the final contact end
CONTACT_ENTER = 12.0
CONTACT_EXIT  = 3.0


CONTACT_NAMES = ["free", "contact"]
C = {n: i for i, n in enumerate(CONTACT_NAMES)}

PHASE_NAMES = ["approach", "progress", "done"]
P = {n: i for i, n in enumerate(PHASE_NAMES)}


@dataclass
class Thr:
    contact_enter: float = CONTACT_ENTER
    contact_exit: float = CONTACT_EXIT


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


def label_episode(abs_fz: np.ndarray, fps: float, thr: Thr) -> Tuple[np.ndarray, np.ndarray, Dict]:
    """
    Returns:
      contact: (T,) int8 in {0,1}
      phase:   (T,) int8 in {0,1,2}
      stats: dict
    """
    T = len(abs_fz)
    contact = np.full((T,), C["free"], dtype=np.int8)
    phase   = np.full((T,), P["approach"], dtype=np.int8)

    min_frames = max(1, int(round(CONTACT_MIN_SEC * fps)))
    onset = _find_contact_onset(abs_fz, thr.contact_enter, min_frames=min_frames)
    end   = _find_contact_end(abs_fz, thr.contact_exit,  min_frames=min_frames)

    # If no stable contact, keep all free + approach
    if onset is None or end is None or end <= onset:
        stats = {"onset": None, "end": None, "note": "no_stable_contact"}
        return contact, phase, stats

    # Approach start (backfill)
    pre_frames = int(round(APPROACH_PRE_SEC * fps))
    approach_start = max(0, onset - pre_frames)

    # contact label
    contact[onset:end + 1] = C["contact"]

    # phase label
    phase[:approach_start] = P["approach"]
    phase[approach_start:end + 1] = P["progress"]
    if end + 1 < T:
        phase[end + 1:] = P["done"]

    stats = {
        "onset": int(onset),
        "end": int(end),
        "approach_start": int(approach_start),
        "contact_min_frames": int(min_frames),
    }
    return contact, phase, stats


def _write_1d_dataset(data_grp, name: str, arr: np.ndarray):
    if name in data_grp:
        del data_grp[name]
    data_grp.create_dataset(
        name,
        data=arr,
        dtype=np.int8,
        chunks=(min(8192, len(arr)),),
        overwrite=True,
    )


def main():
    root = zarr.open(ZARR_PATH, mode="a")

    # sanity checks
    for k in [WRENCH_KEY, EPISODE_ENDS_KEY]:
        if k not in root:
            raise KeyError(f"Missing key: {k}. Try root.tree() to inspect.")

    wrench = np.asarray(root[WRENCH_KEY][:], dtype=np.float32)
    episode_ends = np.asarray(root[EPISODE_ENDS_KEY][:], dtype=np.int64)

    if wrench.ndim != 2 or wrench.shape[1] < 6:
        raise ValueError(f"{WRENCH_KEY} expected shape [N,6], got {wrench.shape}")
    if episode_ends.ndim != 1 or len(episode_ends) == 0:
        raise ValueError(f"{EPISODE_ENDS_KEY} expected shape [E], got {episode_ends.shape}")

    N = wrench.shape[0]
    if int(episode_ends[-1]) != N:
        raise ValueError(f"{EPISODE_ENDS_KEY} must end with N={N}, got last={episode_ends[-1]}")

    # Extract abs(Fz)
    fz = wrench[:, FZ_INDEX].astype(np.float32)
    abs_fz = np.abs(fz) if USE_ABS_FZ else fz

    contact_all = np.full((N,), C["free"], dtype=np.int8)
    phase_all   = np.full((N,), P["approach"], dtype=np.int8)

    ep_starts = np.concatenate([[0], episode_ends[:-1]])
    thr = Thr()
    stats_list: List[Dict] = []

    for ei, (s, e) in enumerate(zip(ep_starts, episode_ends)):
        c_ep, p_ep, st = label_episode(
            abs_fz=abs_fz[int(s):int(e)],
            fps=FPS_FIXED,
            thr=thr
        )
        contact_all[int(s):int(e)] = c_ep
        phase_all[int(s):int(e)] = p_ep
        st.update({"episode_id": int(ei), "start": int(s), "end_frame": int(e)})
        stats_list.append(st)

    # write to zarr (under data group)
    data_grp = root.require_group("data")
    contact_name = OUT_CONTACT_KEY.split("/", 1)[1] if OUT_CONTACT_KEY.startswith("data/") else OUT_CONTACT_KEY
    phase_name   = OUT_PHASE_KEY.split("/", 1)[1] if OUT_PHASE_KEY.startswith("data/") else OUT_PHASE_KEY
    _write_1d_dataset(data_grp, contact_name, contact_all)
    _write_1d_dataset(data_grp, phase_name, phase_all)

    # meta info
    meta_grp = root.require_group("meta")
    paep_grp = meta_grp.require_group("paep")
    paep_grp.attrs["contact_names_v4"] = CONTACT_NAMES
    paep_grp.attrs["contact2id_v4"] = C
    paep_grp.attrs["phase_names_v4"] = PHASE_NAMES
    paep_grp.attrs["phase2id_v4"] = P

    paep_grp.attrs["fps_fixed_v4"] = float(FPS_FIXED)
    paep_grp.attrs["fz_index_v4"] = int(FZ_INDEX)
    paep_grp.attrs["use_abs_fz_v4"] = bool(USE_ABS_FZ)

    paep_grp.attrs["timing_sec_v4"] = {
        "approach_pre_sec": float(APPROACH_PRE_SEC),
        "contact_min_sec": float(CONTACT_MIN_SEC),
    }
    paep_grp.attrs["thresholds_v4"] = {
        "contact_enter": float(thr.contact_enter),
        "contact_exit": float(thr.contact_exit),
    }

    counts_contact = {CONTACT_NAMES[i]: int(np.sum(contact_all == i)) for i in range(len(CONTACT_NAMES))}
    counts_phase   = {PHASE_NAMES[i]: int(np.sum(phase_all == i)) for i in range(len(PHASE_NAMES))}
    paep_grp.attrs["counts_contact_v4"] = counts_contact
    paep_grp.attrs["counts_phase_v4"] = counts_phase
    paep_grp.attrs["episode_stats_json_v4"] = json.dumps(stats_list)

    print("[INFO] Wrote labels:", OUT_CONTACT_KEY, OUT_PHASE_KEY)
    print("[INFO] Contact counts:", counts_contact)
    print("[INFO] Phase counts:", counts_phase)
    if stats_list:
        print("[INFO] Episode[0] stats:", stats_list[0])
    print("[INFO] FZ_INDEX=", FZ_INDEX, "abs=", USE_ABS_FZ, "fps=", FPS_FIXED)
    print("[INFO] Thresholds:", paep_grp.attrs["thresholds_v4"])


if __name__ == "__main__":
    main()
