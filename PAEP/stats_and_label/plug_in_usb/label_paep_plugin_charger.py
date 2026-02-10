#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import zarr

# =========================
# Config
# =========================
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_charger/plug_in_charger_stream_downsample1_zarr/replay_buffer.zarr"

WRENCH_KEY = "data/left_robot_tcp_wrench"
POSE_KEY   = "data/left_robot_tcp_pose"
EPISODE_ENDS_KEY = "meta/episode_ends"

OUT_CONTACT_KEY = "data/paep_contact"
OUT_PHASE_KEY   = "data/paep_phase"

Z_INDEX_IN_POSE = 2
USE_ABS_FZ = True
FPS_FIXED = 24.0

APPROACH_PRE_SEC = 0.15
MIN_STABLE_SEC   = 0.20
RECOVERY_MIN_SEC = 0.08
INSERT_MIN_SEC   = 0.12

INTERACT_ENTER_F = 4.0

DZ_UP   = 0.0005
DF_DROP = -2.5

# ---- search trigger via Z stability ----
ENABLE_Z_STABLE_SEARCH = True
Z_STABLE_WINDOW_SEC = 0.50
Z_RANGE_TH = 0.0015
Z_MEAN_DZ_TH = 0.00020

# ✅ 改动核心：不要用 z_min，改成用 episode 低分位数作为“表面附近”参考
Z_SURFACE_Q = 0.25          # 用 z 的 15% 分位（可改 0.10~0.25）
Z_NEAR_SURFACE_EPS = 0.020  # 12mm：比之前略放宽，让少数 search 不漏判

# ---- hard rule (force>2N cannot be approach, ONLY near-surface) ----
FORCE_NOT_APPROACH_F = 2.0
FORCE_NOT_APPROACH_SEC = 0.12

# ---- insert detection ----
INSERT_PLATEAU_F = 15.0
INSERT_PLATEAU_SEC = 0.25
INSERT_BACKFILL_SEC = 1.2
INSERT_Z_TREND_SEC = 0.6
INSERT_DZ_TREND = -0.002

INSERT_KIN_ENABLE = True
INSERT_KIN_WINDOW_SEC = 0.40
INSERT_KIN_Z_DROP = 0.004
INSERT_KIN_MEAN_DZ_TH = -0.00025

INSERT_ENTER_F   = 15.0
INSERT_ENTER_DF  = 2.0
INSERT_CONFIRM_F = 22.0
INSERT_CONFIRM_FRAMES_SEC = 0.30
INSERT_CONFIRM_MEAN_SEC = 0.6
INSERT_CONFIRM_MEAN_F   = 18.0

MIN_SEARCH_BEFORE_INSERT_SEC = 0.5
# =========================

CONTACT_NAMES = ["free", "contact"]
C = {n: i for i, n in enumerate(CONTACT_NAMES)}

PHASE_NAMES = ["approach", "search", "recovery", "insert"]
P = {n: i for i, n in enumerate(PHASE_NAMES)}


@dataclass
class Thr:
    interact_enter_f: float = INTERACT_ENTER_F
    dz_up: float = DZ_UP
    df_drop: float = DF_DROP

    insert_plateau_f: float = INSERT_PLATEAU_F
    insert_plateau_sec: float = INSERT_PLATEAU_SEC
    insert_backfill_sec: float = INSERT_BACKFILL_SEC
    insert_z_trend_sec: float = INSERT_Z_TREND_SEC
    insert_dz_trend: float = INSERT_DZ_TREND

    insert_enter_f: float = INSERT_ENTER_F
    insert_enter_df: float = INSERT_ENTER_DF
    insert_confirm_f: float = INSERT_CONFIRM_F
    insert_confirm_frames_sec: float = INSERT_CONFIRM_FRAMES_SEC
    insert_confirm_mean_sec: float = INSERT_CONFIRM_MEAN_SEC
    insert_confirm_mean_f: float = INSERT_CONFIRM_MEAN_F

    min_search_before_insert_sec: float = MIN_SEARCH_BEFORE_INSERT_SEC


def _runs_true(mask: np.ndarray, min_frames: int) -> List[Tuple[int, int]]:
    runs: List[Tuple[int, int]] = []
    cnt = 0
    start = None
    for i, a in enumerate(mask):
        if a:
            if cnt == 0:
                start = i
            cnt += 1
        else:
            if cnt >= min_frames and start is not None:
                runs.append((start, i - 1))
            cnt = 0
            start = None
    if cnt >= min_frames and start is not None:
        runs.append((start, len(mask) - 1))
    return runs


def _first_onset(signal: np.ndarray, th_enter: float, min_frames: int) -> Optional[int]:
    if len(signal) < min_frames:
        return None
    runs = _runs_true(signal > th_enter, min_frames=min_frames)
    if not runs:
        return None
    return int(runs[0][0])


def _frame_diff(x: np.ndarray) -> np.ndarray:
    if len(x) == 0:
        return x
    dx = np.diff(x).astype(np.float32)
    return np.concatenate([dx, [np.nan]]).astype(np.float32)


def _z_stable_search_start(z: np.ndarray, dz: np.ndarray, near_surface: np.ndarray, fps: float) -> Optional[int]:
    if not ENABLE_Z_STABLE_SEARCH:
        return None
    T = len(z)
    w = max(2, int(round(Z_STABLE_WINDOW_SEC * fps)))
    if T < w:
        return None
    for t in range(0, T - w + 1):
        ns = near_surface[t:t + w]
        if np.sum(ns) < (w // 2):
            continue
        zz = z[t:t + w]
        if (np.max(zz) - np.min(zz)) > Z_RANGE_TH:
            continue
        dd = dz[t:t + w - 1]
        m = float(np.nanmean(np.abs(dd)))
        if m > Z_MEAN_DZ_TH:
            continue
        return t
    return None


def _insert_kinematic_start(z: np.ndarray, dz: np.ndarray, start_from: int, fps: float) -> Optional[int]:
    if not INSERT_KIN_ENABLE:
        return None
    T = len(z)
    w = max(2, int(round(INSERT_KIN_WINDOW_SEC * fps)))
    if T < start_from + w:
        return None
    for t in range(start_from, T - w + 1):
        z0 = float(z[t])
        z1 = float(z[t + w - 1])
        net = z1 - z0
        mean_dz = float(np.nanmean(dz[t:t + w - 1]))
        if (net <= -INSERT_KIN_Z_DROP) and (mean_dz <= INSERT_KIN_MEAN_DZ_TH):
            return t
    return None


def label_episode(wrench_ep: np.ndarray, pose_ep: np.ndarray, fps: float, thr: Thr) -> Tuple[np.ndarray, np.ndarray, Dict]:
    T = int(wrench_ep.shape[0])
    phase = np.full((T,), P["approach"], dtype=np.int8)

    Fx, Fy, Fz = wrench_ep[:, 0], wrench_ep[:, 1], wrench_ep[:, 2]
    abs_fz = np.abs(Fz) if USE_ABS_FZ else Fz
    F_norm = np.sqrt(Fx * Fx + Fy * Fy + Fz * Fz)

    z = pose_ep[:, Z_INDEX_IN_POSE]
    dz = _frame_diff(z)
    dF = _frame_diff(F_norm)

    # ✅ surface reference: low quantile (avoid z_min being insert depth)
    z_surface = float(np.quantile(z, Z_SURFACE_Q)) if T > 0 else 0.0
    near_surface = z <= (z_surface + Z_NEAR_SURFACE_EPS)

    # A) coarse force onset baseline
    min_frames = max(1, int(round(MIN_STABLE_SEC * fps)))
    onset = _first_onset(F_norm, thr.interact_enter_f, min_frames=min_frames)
    if onset is None:
        onset = T

    pre_frames = int(round(APPROACH_PRE_SEC * fps))
    approach_end = max(0, int(onset) - pre_frames)
    approach_end = min(approach_end, T)

    phase[:approach_end] = P["approach"]
    phase[approach_end:] = P["search"]

    # B) recovery raw
    # rec_min_frames = max(1, int(round(RECOVERY_MIN_SEC * fps)))
    # rec_mask = (dz > thr.dz_up) | (dF < thr.df_drop)
    # rec_runs = _runs_true(rec_mask, min_frames=rec_min_frames)
    # for rs, re in rec_runs:
    #     phase[rs:re + 1] = P["recovery"]
    rec_min_frames = max(1, int(round(RECOVERY_MIN_SEC * fps)))
    rec_mask = (dz > thr.dz_up) | (dF < thr.df_drop)
    # 先不写 phase，等 insert 判完再覆盖（避免 insert 中瞬间误判为 recovery）


    # C) insert raw
    insert_start_raw = None
    plateau_frames = max(1, int(round(thr.insert_plateau_sec * fps)))
    plateau_mask = (F_norm > thr.insert_plateau_f) & (phase != P["approach"])
    plateau_runs = _runs_true(plateau_mask, min_frames=plateau_frames)

    min_search_frames = max(0, int(round(thr.min_search_before_insert_sec * fps)))

    if plateau_runs:
        t_plateau = int(plateau_runs[0][0])
        backfill_frames = int(round(thr.insert_backfill_sec * fps))
        t0 = max(0, t_plateau - backfill_frames)
        t0 = max(t0, approach_end + min_search_frames)

        trend_frames = max(1, int(round(thr.insert_z_trend_sec * fps)))
        t_trend_end = min(T - 1, t_plateau)
        t_trend_start = max(0, t_trend_end - trend_frames)
        dz_net = float(z[t_trend_end] - z[t_trend_start])
        if dz_net < thr.insert_dz_trend:
            insert_start_raw = t0

    kin_start = _insert_kinematic_start(z, dz, start_from=approach_end + min_search_frames, fps=fps)
    if kin_start is not None:
        insert_start_raw = kin_start if insert_start_raw is None else min(insert_start_raw, kin_start)

    if insert_start_raw is not None:
        phase[insert_start_raw:] = P["insert"]

    # --- apply recovery AFTER insert is assigned, and forbid recovery in insert ---
    # 只允许 search 中出现 recovery（推荐：能清掉“插入瞬间被误判 recovery”的那 500 帧）
    rec_mask2 = rec_mask & (phase == P["search"])
    rec_runs2 = _runs_true(rec_mask2, min_frames=rec_min_frames)
    for rs, re in rec_runs2:
        phase[rs:re + 1] = P["recovery"]


    # D) HARD constraints
    z_stable_start = _z_stable_search_start(z, dz, near_surface, fps) if ENABLE_Z_STABLE_SEARCH else None
    first_search_idx = approach_end
    if z_stable_start is not None:
        first_search_idx = min(first_search_idx, int(z_stable_start))

    # force>2N rule ONLY near-surface + debounce
    force_mask = (F_norm > FORCE_NOT_APPROACH_F) & near_surface
    force_runs = _runs_true(force_mask, min_frames=max(1, int(round(FORCE_NOT_APPROACH_SEC * fps))))
    force_first = int(force_runs[0][0]) if force_runs else None
    if force_first is not None:
        first_search_idx = min(first_search_idx, force_first)

    # extra trigger: sustained near-surface => search (avoid whole-episode approach)
    ns_min_frames = int(round(0.5 * fps))  # 0.5s
    ns_runs = _runs_true(near_surface, min_frames=ns_min_frames)
    if ns_runs:
        first_search_idx = min(first_search_idx, ns_runs[0][0])


    first_search_idx = int(np.clip(first_search_idx, 0, T))

    # before search => approach only
    phase[:first_search_idx] = P["approach"]
    # after search => no approach
    if first_search_idx < T:
        phase[first_search_idx:][phase[first_search_idx:] == P["approach"]] = P["search"]

    # strict: any frame force>2N & near-surface cannot be approach
    phase[(phase == P["approach"]) & force_mask] = P["search"]

    # enforce monotonic: after first non-approach, no approach
    non_approach = np.where(phase != P["approach"])[0]
    t_first_non_approach = int(non_approach[0]) if len(non_approach) > 0 else None
    if t_first_non_approach is not None:
        phase[t_first_non_approach:][phase[t_first_non_approach:] == P["approach"]] = P["search"]
        first_search_idx = min(first_search_idx, t_first_non_approach)

    contact = np.where(phase == P["approach"], C["free"], C["contact"]).astype(np.int8)

    st = {
        "z_surface_q": float(Z_SURFACE_Q),
        "z_surface": float(z_surface),
        "near_surface_eps": float(Z_NEAR_SURFACE_EPS),
        "z_stable_search_start": (int(z_stable_start) if z_stable_start is not None else None),
        "force_first_gt2N_near_surface": force_first,
        "first_search_idx": int(first_search_idx),
        "insert_start_raw": (int(insert_start_raw) if insert_start_raw is not None else None),
        "kin_insert_start": (int(kin_start) if kin_start is not None else None),
    }
    return contact, phase, st


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

    for k in [WRENCH_KEY, POSE_KEY, EPISODE_ENDS_KEY]:
        if k not in root:
            raise KeyError(f"Missing key: {k}. Try root.tree() to inspect.")

    wrench = np.asarray(root[WRENCH_KEY][:], dtype=np.float32)
    pose = np.asarray(root[POSE_KEY][:], dtype=np.float32)
    episode_ends = np.asarray(root[EPISODE_ENDS_KEY][:], dtype=np.int64)

    N = int(wrench.shape[0])
    if pose.shape[0] != N:
        raise ValueError(f"pose length must match wrench: pose={pose.shape[0]} wrench={N}")
    if int(episode_ends[-1]) != N:
        raise ValueError(f"{EPISODE_ENDS_KEY} must end with N={N}, got last={episode_ends[-1]}")

    contact_all = np.full((N,), C["free"], dtype=np.int8)
    phase_all   = np.full((N,), P["approach"], dtype=np.int8)

    ep_starts = np.concatenate([[0], episode_ends[:-1]])
    thr = Thr()
    stats_list: List[Dict] = []

    for ei, (s, e) in enumerate(zip(ep_starts, episode_ends)):
        s_i, e_i = int(s), int(e)
        c_ep, p_ep, st = label_episode(wrench[s_i:e_i], pose[s_i:e_i], FPS_FIXED, thr)
        contact_all[s_i:e_i] = c_ep
        phase_all[s_i:e_i] = p_ep
        st.update({"episode_id": int(ei), "start": s_i, "end_frame": e_i})
        stats_list.append(st)

    data_grp = root.require_group("data")
    _write_1d_dataset(data_grp, "paep_contact", contact_all)
    _write_1d_dataset(data_grp, "paep_phase", phase_all)

    meta_grp = root.require_group("meta")
    paep_grp = meta_grp.require_group("paep")

    paep_grp.attrs["task_name_v1"] = "plug_in_charger"
    paep_grp.attrs["contact_names_v1"] = CONTACT_NAMES
    paep_grp.attrs["contact2id_v1"] = C
    paep_grp.attrs["phase_names_v1"] = PHASE_NAMES
    paep_grp.attrs["phase2id_v1"] = P

    counts_contact = {CONTACT_NAMES[i]: int(np.sum(contact_all == i)) for i in range(len(CONTACT_NAMES))}
    counts_phase   = {PHASE_NAMES[i]: int(np.sum(phase_all == i)) for i in range(len(PHASE_NAMES))}
    paep_grp.attrs["counts_contact_v1"] = counts_contact
    paep_grp.attrs["counts_phase_v1"] = counts_phase
    paep_grp.attrs["episode_stats_json_v1"] = json.dumps(stats_list)

    print("[INFO] Wrote labels: data/paep_contact data/paep_phase")
    print("[INFO] Contact counts:", counts_contact)
    print("[INFO] Phase counts:", counts_phase)

    if stats_list:
        idxs = [0, len(stats_list)//2, len(stats_list)-1]
        idxs = sorted(list(set([i for i in idxs if 0 <= i < len(stats_list)])))
        for i in idxs:
            print(f"[INFO] Episode[{i}] stats:", stats_list[i])


if __name__ == "__main__":
    main()
