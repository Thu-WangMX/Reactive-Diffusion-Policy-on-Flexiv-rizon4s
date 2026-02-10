#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PAEP labeling for plug_in_usb (FULL REPLACEMENT)

Semantics:
- phase monotonic: approach -> search (may include recovery) -> insert (final)
- recovery ONLY before insert; after insert do NOT label recovery
- after recovery MUST return to search
- insert start MUST be AFTER last recovery end
- contact is independent of phase (physical gate): hysteresis + debounce (+ optional near-surface)

Insert detection (4 triggers):
1) FORCE PLATEAU (strong): F_norm > INSERT_PLATEAU_F for INSERT_PLATEAU_SEC (+ z-trend) => insert
2) KINEMATIC (medium): z-drop pattern + weak force/near-surface evidence => insert
3) CONTACT-RUN (weak): stable contact run + z downward trend => insert
4) DEPTH-RUN (key for low-force insert): z stays BELOW surface by depth_eps for min_sec => insert

Outputs:
- /data/paep_contact : int8 {0=free, 1=contact}
- /data/paep_phase   : int8 {0=approach, 1=search, 2=recovery, 3=insert}
"""

import json
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import zarr


# =========================
# Paths / Keys
# =========================
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_usb/plug_in_usb_stream_downsample1_zarr/replay_buffer.zarr"

WRENCH_KEY = "data/left_robot_tcp_wrench"
POSE_KEY = "data/left_robot_tcp_pose"
EPISODE_ENDS_KEY = "meta/episode_ends"


# =========================
# Assumptions
# =========================
FPS_FIXED = 24.0
Z_INDEX_IN_POSE = 2
USE_ABS_FZ = True


# =========================
# Phase config (USB)
# =========================
APPROACH_PRE_SEC = 0.15
MIN_STABLE_SEC = 0.20
INTERACT_ENTER_F = 4.0

FORCE_NOT_APPROACH_F = 2.0
FORCE_NOT_APPROACH_SEC = 0.12

Z_SURFACE_Q = 0.25
Z_NEAR_SURFACE_EPS = 0.020  # 20mm

ENABLE_Z_STABLE_SEARCH = True
Z_STABLE_WINDOW_SEC = 0.50
Z_RANGE_TH = 0.0015
Z_MEAN_ABS_DZ_TH = 0.00020


# =========================
# Recovery config (USB)
# =========================
RECOVERY_MIN_SEC = 0.08
RECOVERY_WINDOW_SEC = 0.30
RECOVERY_Z_RISE = 0.004  # 4mm net rise within window

DZ_UP = 0.0005
DF_DROP = -2.5
ENABLE_DF_DROP_RECOVERY = False


# =========================
# Insert config (final stage)
# =========================
MIN_SEARCH_BEFORE_INSERT_SEC = 0.5

# 1) Force plateau insert
INSERT_PLATEAU_F = 12.0
INSERT_PLATEAU_SEC = 0.22
INSERT_BACKFILL_SEC = 1.2
INSERT_Z_TREND_SEC = 0.6
INSERT_DZ_TREND = -0.002  # net z change over trend window must be <= this (descending)

# 2) Kinematic insert
INSERT_KIN_ENABLE = True
INSERT_KIN_WINDOW_SEC = 0.40
INSERT_KIN_Z_DROP = 0.0035
INSERT_KIN_MEAN_DZ_TH = -0.00020

# Kinematic gate: weak evidence (avoid no-contact insert)
INSERT_KIN_GATE_WINDOW_SEC = 0.25
INSERT_KIN_GATE_MIN_SEC = 0.08
INSERT_KIN_GATE_F = 1.2  # 🔧 lowered from 2.0 to catch low-force inserts
INSERT_KIN_GATE_REQUIRE_NEAR_SURFACE = True

# 3) Contact-run + z-drop insert (covers low-force insert)
INSERT_CONTACT_ENABLE = True
INSERT_CONTACT_MIN_SEC = 0.10
INSERT_CONTACT_WINDOW_SEC = 0.35
INSERT_CONTACT_Z_DROP = 0.0015  # 🔧 lowered from 2.0mm -> 1.5mm

# 4) Depth-run insert (most robust for low-force)
INSERT_DEPTH_ENABLE = True
INSERT_DEPTH_BELOW_SURFACE = 0.0020  # 2mm below surface
INSERT_DEPTH_MIN_SEC = 0.12          # require sustained below-surface


# =========================
# Contact config (independent of phase)
# =========================
CONTACT_REQUIRE_NEAR_SURFACE = True
CONTACT_ENTER_F = 2.5
CONTACT_EXIT_F = 2.0
CONTACT_MIN_SEC = 0.10


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

    min_search_before_insert_sec: float = MIN_SEARCH_BEFORE_INSERT_SEC

    contact_enter_f: float = CONTACT_ENTER_F
    contact_exit_f: float = CONTACT_EXIT_F
    contact_min_sec: float = CONTACT_MIN_SEC


# =========================
# Helpers
# =========================
def _frame_diff(x: np.ndarray) -> np.ndarray:
    if len(x) == 0:
        return x
    dx = np.diff(x).astype(np.float32)
    return np.concatenate([dx, [np.nan]]).astype(np.float32)


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
        if m > Z_MEAN_ABS_DZ_TH:
            continue
        return t
    return None


def _recovery_candidate_mask(z: np.ndarray, dz: np.ndarray, dF: np.ndarray, start_from: int, fps: float) -> np.ndarray:
    T = len(z)
    w = max(2, int(round(RECOVERY_WINDOW_SEC * fps)))
    mask_win = np.zeros((T,), dtype=bool)
    if T >= w:
        for t in range(start_from, T - w + 1):
            net = float(z[t + w - 1] - z[t])
            if net >= RECOVERY_Z_RISE:
                mask_win[t:t + w] = True

    mask_frame = (dz > DZ_UP)
    if ENABLE_DF_DROP_RECOVERY:
        mask_frame = mask_frame | (dF < DF_DROP)

    mask = (mask_win | mask_frame)
    mask[:start_from] = False
    return mask


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


def _insert_force_start(F_norm: np.ndarray, z: np.ndarray, start_from: int, fps: float, thr: Thr) -> Optional[int]:
    T = len(F_norm)
    plateau_frames = max(1, int(round(thr.insert_plateau_sec * fps)))
    plateau_mask = np.zeros((T,), dtype=bool)
    plateau_mask[start_from:] = (F_norm[start_from:] > thr.insert_plateau_f)

    runs = _runs_true(plateau_mask, min_frames=plateau_frames)
    if not runs:
        return None

    t_plateau = int(runs[0][0])

    trend_frames = max(2, int(round(thr.insert_z_trend_sec * fps)))
    t_trend_end = min(T - 1, t_plateau)
    t_trend_start = max(start_from, t_trend_end - trend_frames)
    dz_net = float(z[t_trend_end] - z[t_trend_start])
    if dz_net >= thr.insert_dz_trend:
        return None

    backfill = int(round(thr.insert_backfill_sec * fps))
    t0 = max(start_from, t_plateau - backfill)
    return int(t0)


def _contact_from_force(F_norm: np.ndarray, near_surface: np.ndarray, fps: float, thr: Thr) -> np.ndarray:
    T = len(F_norm)
    enter_th = float(thr.contact_enter_f)
    exit_th = float(thr.contact_exit_f)
    min_frames = max(1, int(round(thr.contact_min_sec * fps)))

    can_enter = np.ones((T,), dtype=bool)
    if CONTACT_REQUIRE_NEAR_SURFACE:
        can_enter &= near_surface

    raw = np.zeros((T,), dtype=bool)
    on = False
    for t in range(T):
        f = float(F_norm[t])
        if not on:
            if can_enter[t] and (f > enter_th):
                on = True
        else:
            if f < exit_th:
                on = False
        raw[t] = on

    contact01 = np.zeros((T,), dtype=np.int8)
    for s, e in _runs_true(raw, min_frames=min_frames):
        contact01[s:e + 1] = 1
    return contact01


def _kin_insert_has_contact_evidence(t0: int, F_norm: np.ndarray, near_surface: np.ndarray, fps: float) -> bool:
    if t0 is None:
        return False
    T = len(F_norm)
    w = max(1, int(round(INSERT_KIN_GATE_WINDOW_SEC * fps)))
    t1 = min(T, int(t0) + w)

    cond = (F_norm[int(t0):t1] > INSERT_KIN_GATE_F)
    if INSERT_KIN_GATE_REQUIRE_NEAR_SURFACE:
        cond = cond & (near_surface[int(t0):t1])

    need = max(1, int(round(INSERT_KIN_GATE_MIN_SEC * fps)))
    return len(_runs_true(cond, min_frames=need)) > 0


def _contact_run_insert_start(contact01: np.ndarray, z: np.ndarray, start_from: int, fps: float) -> Optional[int]:
    if not INSERT_CONTACT_ENABLE:
        return None
    T = len(z)
    min_c_frames = max(1, int(round(INSERT_CONTACT_MIN_SEC * fps)))
    mask = (contact01 > 0).astype(bool)
    mask[:start_from] = False

    runs = _runs_true(mask, min_frames=min_c_frames)
    if not runs:
        return None

    rs, _re = runs[0]
    w = max(2, int(round(INSERT_CONTACT_WINDOW_SEC * fps)))
    t0 = int(rs)
    t1 = min(T - 1, t0 + w - 1)
    dz_net = float(z[t1] - z[t0])
    if dz_net <= -INSERT_CONTACT_Z_DROP:
        return int(rs)
    return None


def _depth_run_insert_start(z: np.ndarray, z_surface: float, start_from: int, fps: float) -> Optional[int]:
    """
    Depth-run insert: after start_from, if z stays below (z_surface - depth_eps) for >= min_sec -> insert
    This is robust for low-force insert (your current failure case).
    """
    if not INSERT_DEPTH_ENABLE:
        return None
    T = len(z)
    th = float(z_surface - INSERT_DEPTH_BELOW_SURFACE)
    min_frames = max(1, int(round(INSERT_DEPTH_MIN_SEC * fps)))

    mask = (z <= th)
    mask[:start_from] = False

    runs = _runs_true(mask, min_frames=min_frames)
    if not runs:
        return None
    return int(runs[0][0])


# =========================
# Labeling core
# =========================
def label_episode(wrench_ep: np.ndarray, pose_ep: np.ndarray, fps: float, thr: Thr) -> Tuple[np.ndarray, np.ndarray, Dict]:
    T = int(wrench_ep.shape[0])
    phase = np.full((T,), P["approach"], dtype=np.int8)

    Fx, Fy, Fz = wrench_ep[:, 0], wrench_ep[:, 1], wrench_ep[:, 2]
    _ = np.abs(Fz) if USE_ABS_FZ else Fz
    F_norm = np.sqrt(Fx * Fx + Fy * Fy + Fz * Fz)

    z = pose_ep[:, Z_INDEX_IN_POSE]
    dz = _frame_diff(z)
    dF = _frame_diff(F_norm)

    z_surface = float(np.quantile(z, Z_SURFACE_Q)) if T > 0 else 0.0
    near_surface = z <= (z_surface + Z_NEAR_SURFACE_EPS)

    contact01 = _contact_from_force(F_norm, near_surface, fps, thr)

    # ---- approach -> search ----
    min_frames = max(1, int(round(MIN_STABLE_SEC * fps)))
    onset = _first_onset(F_norm, thr.interact_enter_f, min_frames=min_frames)
    if onset is None:
        onset = T

    pre_frames = int(round(APPROACH_PRE_SEC * fps))
    approach_end = int(np.clip(int(onset) - pre_frames, 0, T))
    phase[:approach_end] = P["approach"]
    phase[approach_end:] = P["search"]

    first_search_idx = approach_end

    z_stable_start = _z_stable_search_start(z, dz, near_surface, fps) if ENABLE_Z_STABLE_SEARCH else None
    if z_stable_start is not None:
        first_search_idx = min(first_search_idx, int(z_stable_start))

    force_mask = (F_norm > FORCE_NOT_APPROACH_F) & near_surface
    force_runs = _runs_true(force_mask, min_frames=max(1, int(round(FORCE_NOT_APPROACH_SEC * fps))))
    force_first = int(force_runs[0][0]) if force_runs else None
    if force_first is not None:
        first_search_idx = min(first_search_idx, force_first)

    ns_min_frames = int(round(0.5 * fps))
    ns_runs = _runs_true(near_surface, min_frames=ns_min_frames)
    if ns_runs:
        first_search_idx = min(first_search_idx, int(ns_runs[0][0]))

    first_search_idx = int(np.clip(first_search_idx, 0, T))
    phase[:first_search_idx] = P["approach"]
    if first_search_idx < T:
        phase[first_search_idx:][phase[first_search_idx:] == P["approach"]] = P["search"]
    phase[(phase == P["approach"]) & force_mask] = P["search"]

    non_approach = np.where(phase != P["approach"])[0]
    t_first_non_approach = int(non_approach[0]) if len(non_approach) > 0 else None
    if t_first_non_approach is not None:
        phase[t_first_non_approach:][phase[t_first_non_approach:] == P["approach"]] = P["search"]
        first_search_idx = min(first_search_idx, t_first_non_approach)

    # ---- recovery (before insert) ----
    rec_min_frames = max(1, int(round(RECOVERY_MIN_SEC * fps)))
    rec_cand = _recovery_candidate_mask(z, dz, dF, start_from=first_search_idx, fps=fps)
    rec_runs = _runs_true(rec_cand, min_frames=rec_min_frames)

    for rs, re in rec_runs:
        rs2 = max(rs, first_search_idx)
        if rs2 <= re:
            phase[rs2:re + 1] = P["recovery"]

    last_recovery_end = int(max(e for _, e in rec_runs)) if rec_runs else None

    # ---- insert (final) after last recovery ----
    min_search_frames = max(0, int(round(thr.min_search_before_insert_sec * fps)))
    insert_search_start = first_search_idx + min_search_frames
    if last_recovery_end is not None:
        insert_search_start = max(insert_search_start, last_recovery_end + 1)
    insert_search_start = int(np.clip(insert_search_start, 0, T))

    insert_force = _insert_force_start(F_norm, z, start_from=insert_search_start, fps=fps, thr=thr)
    insert_kin = _insert_kinematic_start(z, dz, start_from=insert_search_start, fps=fps) if INSERT_KIN_ENABLE else None
    insert_contact = _contact_run_insert_start(contact01, z, start_from=insert_search_start, fps=fps)
    insert_depth = _depth_run_insert_start(z, z_surface, start_from=insert_search_start, fps=fps)

    cands: List[int] = []
    if insert_force is not None:
        cands.append(int(insert_force))
    if insert_kin is not None:
        if _kin_insert_has_contact_evidence(int(insert_kin), F_norm, near_surface, fps):
            cands.append(int(insert_kin))
    if insert_contact is not None:
        cands.append(int(insert_contact))
    if insert_depth is not None:
        cands.append(int(insert_depth))

    insert_start_raw: Optional[int] = int(min(cands)) if cands else None

    if insert_start_raw is not None and last_recovery_end is not None:
        if insert_start_raw <= last_recovery_end:
            insert_start_raw = last_recovery_end + 1
            if insert_start_raw >= T:
                insert_start_raw = None

    if insert_start_raw is not None:
        phase[insert_start_raw:] = P["insert"]

    contact = np.where(contact01 > 0, C["contact"], C["free"]).astype(np.int8)

    st = {
        "z_surface_q": float(Z_SURFACE_Q),
        "z_surface": float(z_surface),
        "near_surface_eps": float(Z_NEAR_SURFACE_EPS),

        "z_stable_search_start": (int(z_stable_start) if z_stable_start is not None else None),
        "force_first_gt2N_near_surface": force_first,
        "first_search_idx": int(first_search_idx),

        "recovery_window_sec": float(RECOVERY_WINDOW_SEC),
        "recovery_z_rise": float(RECOVERY_Z_RISE),
        "last_recovery_end": (int(last_recovery_end) if last_recovery_end is not None else None),

        "insert_search_start": int(insert_search_start),
        "insert_start_raw": (int(insert_start_raw) if insert_start_raw is not None else None),
        "insert_force_start": (int(insert_force) if insert_force is not None else None),
        "insert_kin_start": (int(insert_kin) if insert_kin is not None else None),
        "insert_contact_start": (int(insert_contact) if insert_contact is not None else None),
        "insert_depth_start": (int(insert_depth) if insert_depth is not None else None),

        "kin_gate_window_sec": float(INSERT_KIN_GATE_WINDOW_SEC),
        "kin_gate_min_sec": float(INSERT_KIN_GATE_MIN_SEC),
        "kin_gate_f": float(INSERT_KIN_GATE_F),
        "kin_gate_require_near_surface": bool(INSERT_KIN_GATE_REQUIRE_NEAR_SURFACE),

        "contact_insert_enable": bool(INSERT_CONTACT_ENABLE),
        "contact_insert_min_sec": float(INSERT_CONTACT_MIN_SEC),
        "contact_insert_window_sec": float(INSERT_CONTACT_WINDOW_SEC),
        "contact_insert_z_drop": float(INSERT_CONTACT_Z_DROP),

        "depth_insert_enable": bool(INSERT_DEPTH_ENABLE),
        "depth_below_surface": float(INSERT_DEPTH_BELOW_SURFACE),
        "depth_min_sec": float(INSERT_DEPTH_MIN_SEC),

        "contact_enter_f": float(thr.contact_enter_f),
        "contact_exit_f": float(thr.contact_exit_f),
        "contact_min_sec": float(thr.contact_min_sec),
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
            raise KeyError(f"Missing key: {k}. Please inspect zarr tree.")

    wrench = np.asarray(root[WRENCH_KEY][:], dtype=np.float32)
    pose = np.asarray(root[POSE_KEY][:], dtype=np.float32)
    episode_ends = np.asarray(root[EPISODE_ENDS_KEY][:], dtype=np.int64)

    N = int(wrench.shape[0])
    if pose.shape[0] != N:
        raise ValueError(f"pose length must match wrench: pose={pose.shape[0]} wrench={N}")
    if int(episode_ends[-1]) != N:
        raise ValueError(f"{EPISODE_ENDS_KEY} must end with N={N}, got last={episode_ends[-1]}")

    contact_all = np.full((N,), C["free"], dtype=np.int8)
    phase_all = np.full((N,), P["approach"], dtype=np.int8)

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

    paep_grp.attrs["task_name_v1"] = "plug_in_usb"
    paep_grp.attrs["contact_names_v1"] = CONTACT_NAMES
    paep_grp.attrs["contact2id_v1"] = C
    paep_grp.attrs["phase_names_v1"] = PHASE_NAMES
    paep_grp.attrs["phase2id_v1"] = P

    counts_contact = {CONTACT_NAMES[i]: int(np.sum(contact_all == i)) for i in range(len(CONTACT_NAMES))}
    counts_phase = {PHASE_NAMES[i]: int(np.sum(phase_all == i)) for i in range(len(PHASE_NAMES))}
    paep_grp.attrs["counts_contact_v1"] = counts_contact
    paep_grp.attrs["counts_phase_v1"] = counts_phase
    paep_grp.attrs["episode_stats_json_v1"] = json.dumps(stats_list)

    print("[INFO] Wrote labels: data/paep_contact data/paep_phase")
    print("[INFO] Contact counts:", counts_contact)
    print("[INFO] Phase counts:", counts_phase)

    if stats_list:
        idxs = [0, len(stats_list) // 2, len(stats_list) - 1]
        idxs = sorted(set([i for i in idxs if 0 <= i < len(stats_list)]))
        for i in idxs:
            print(f"[INFO] Episode[{i}] stats:", stats_list[i])


if __name__ == "__main__":
    main()
