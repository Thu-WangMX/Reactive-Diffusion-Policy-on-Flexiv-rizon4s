#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import json
import numpy as np


def _as_list_of_dict(obj_arr):
    """npz里 dtype=object 的 dict 列表 -> python list[dict]"""
    if obj_arr is None:
        return []
    if isinstance(obj_arr, list):
        return obj_arr
    if isinstance(obj_arr, np.ndarray):
        if obj_arr.shape == ():
            x = obj_arr.item()
            if x is None:
                return []
            return x if isinstance(x, list) else [x]
        out = []
        for x in obj_arr.tolist():
            if x is None:
                continue
            out.append(x)
        return out
    return [obj_arr]


def _preview_head_mid_tail(lst, name, n=5, keys=None):
    """打印 head/mid/tail，每条记录做key筛选，防止太长"""
    if not lst:
        print(f"{name}: empty")
        return
    L = len(lst)
    mid = L // 2
    print("-" * 100)
    print(f"{name} preview: head {n}, mid {n}, tail {n} (total={L})")

    def _fmt(r):
        if keys is None:
            return r
        d = {}
        for k in keys:
            if k in r:
                d[k] = r[k]
        return d

    def _block(title, idx0, items):
        print(f"  [{title}]")
        for i, r in items:
            print(f"    idx={i:>5d}  {_fmt(r)}")

    head = list(enumerate(lst[:n]))
    mid_start = max(0, mid - n // 2)
    mid_block = list(enumerate(lst[mid_start: mid_start + n], start=mid_start))
    tail = list(enumerate(lst[-n:], start=L - n))

    _block("head", 0, head)
    _block("mid", mid_start, mid_block)
    _block("tail", L - n, tail)


def _safe_float(x, default=np.nan):
    try:
        return float(x)
    except Exception:
        return default


def main():
    # ======= 你要读的文件路径 =======
    npz_path = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video/fz_logs_wmx_paep_real_plugin_charger_image_dp_absolute_24fps/20260209_181245/episode_000_debug_all.npz"
    # =================================

    if not os.path.exists(npz_path):
        print(f"[ERR] file not found: {npz_path}")
        sys.exit(1)

    data = np.load(npz_path, allow_pickle=True)

    control_fps = int(np.array(data.get("control_fps", [-1])).reshape(-1)[0])
    inference_fps = int(np.array(data.get("inference_fps", [-1])).reshape(-1)[0])
    episode_idx = int(np.array(data.get("episode_idx", [-1])).reshape(-1)[0])

    fz = np.array(data.get("fz", []), dtype=np.float32)
    fz_t = np.array(data.get("fz_wall_time", []), dtype=np.float64)
    fz_step = np.array(data.get("fz_control_step", []), dtype=np.int32)

    paep_ctrl = _as_list_of_dict(data.get("paep_ctrl", None))
    fusion_infer = _as_list_of_dict(data.get("fusion_infer", None))

    print("=" * 100)
    print(f"Loaded: {npz_path}")
    print(f"episode_idx={episode_idx}  control_fps={control_fps}  inference_fps={inference_fps}")
    print("-" * 100)
    print(f"[FZ] len={len(fz)}  step_range=({fz_step.min() if len(fz_step) else 'NA'}..{fz_step.max() if len(fz_step) else 'NA'})")
    if len(fz):
        print(f"     mean={np.nanmean(fz):+.3f}  min={np.nanmin(fz):+.3f}  max={np.nanmax(fz):+.3f}")
    print(f"[PAEP_CTRL] records={len(paep_ctrl)}")
    print(f"[FUSION_INFER] records={len(fusion_infer)}")
    print("=" * 100)

    # ---- quick sanity: expected ratio ----
    if len(fz) and len(fusion_infer):
        ratio = len(fz) / max(1, len(fusion_infer))
        print(f"[SANITY] fz/fusion_infer ≈ {ratio:.3f} (expected ≈ control_fps/inference_fps = {control_fps/max(1,inference_fps):.3f})")
        print("-" * 100)

    # ---- preview keys (avoid huge dict dump) ----
    paep_keys = ["control_step", "phase_id", "p_contact", "p_phase_top_idx", "p_phase_top_val", "p_phase_entropy"]
    fusion_keys = ["infer_step", "control_step", "fusion/alpha", "fusion/g_contact_mean",
                  "fusion/inj_norm_mean", "fusion/scale_mean", "fusion/effective_ratio_mean",
                  "pol/p_contact_last", "paep_cache/p_contact", "paep_cache/p_phase_top_idx", "paep_cache/p_phase_top_val"]

    _preview_head_mid_tail(paep_ctrl, "paep_ctrl", n=8, keys=paep_keys)
    _preview_head_mid_tail(fusion_infer, "fusion_infer", n=5, keys=fusion_keys)

    # ---- build arrays for stats ----
    pc = np.array([_safe_float(r.get("p_contact", np.nan)) for r in paep_ctrl], dtype=np.float64)
    phase_id = np.array([_safe_float(r.get("phase_id", np.nan)) for r in paep_ctrl], dtype=np.float64)
    ent = np.array([_safe_float(r.get("p_phase_entropy", np.nan)) for r in paep_ctrl], dtype=np.float64)
    ctrl_step = np.array([int(r.get("control_step", -1)) for r in paep_ctrl], dtype=np.int32)

    # ---- stats dump ----
    print("-" * 100)
    print("[PAEP_CTRL STATS]")
    if pc.size:
        print(f"  p_contact: min={np.nanmin(pc):.3e}  max={np.nanmax(pc):.3e}  mean={np.nanmean(pc):.3e}  median={np.nanmedian(pc):.3e}")
        for thr in [0.5, 0.1, 1e-2, 1e-3, 1e-4, 1e-6]:
            cnt = int(np.sum(pc > thr))
            print(f"  count(p_contact>{thr:g}) = {cnt} / {pc.size}")
    if ent.size:
        print(f"  phase_entropy: min={np.nanmin(ent):.3e}  max={np.nanmax(ent):.3e}  mean={np.nanmean(ent):.3e}  median={np.nanmedian(ent):.3e}")
        for thr in [1e-1, 1e-2, 1e-3, 1e-4, 1e-6]:
            cnt = int(np.sum(ent < thr))
            print(f"  count(entropy<{thr:g}) = {cnt} / {ent.size}")

    # phase_id histogram (coarse)
    if phase_id.size:
        valid = phase_id[np.isfinite(phase_id)]
        if valid.size:
            uniq, cnts = np.unique(valid.astype(int), return_counts=True)
            print("  phase_id counts:")
            for u, c in zip(uniq.tolist(), cnts.tolist()):
                print(f"    phase_id={u}: {c}")
    print("-" * 100)

    # ---- suspicious events: strong contact in Fz but p_contact tiny ----
    # We'll align by control_step index within this episode (not wall-time).
    # fz_step is already aligned to control_step (runner design).
    print("[SUSPICIOUS CHECK] strong |Fz| but PAEP p_contact ~ 0 ?")
    if len(fz) and len(paep_ctrl):
        # build map: control_step -> p_contact
        pc_map = {int(r.get("control_step", -1)): _safe_float(r.get("p_contact", np.nan)) for r in paep_ctrl}
        # thresholds: pick a few force levels (tune if needed)
        fz_thr = 10.0  # N
        bad = []
        for s, f in zip(fz_step.tolist(), fz.tolist()):
            if np.isfinite(f) and abs(f) >= fz_thr:
                p = pc_map.get(int(s), np.nan)
                if np.isfinite(p) and p < 1e-3:
                    bad.append((int(s), float(f), float(p)))
        print(f"  force_threshold={fz_thr}N, found {len(bad)} steps where |Fz|>=thr but p_contact<1e-3")
        for row in bad[:20]:
            print(f"    control_step={row[0]:>4d}  Fz={row[1]:>+7.3f}  p_contact={row[2]:.3e}")
        if len(bad) > 20:
            print(f"    ... ({len(bad)-20} more)")
    else:
        print("  (skip: missing fz or paep_ctrl)")
    print("-" * 100)

    # ---- fusion gate check ----
    print("[FUSION GATE CHECK] inj_norm / scale / g_contact ranges")
    if fusion_infer:
        inj = np.array([_safe_float(r.get("fusion/inj_norm_mean", np.nan)) for r in fusion_infer], dtype=np.float64)
        scale = np.array([_safe_float(r.get("fusion/scale_mean", np.nan)) for r in fusion_infer], dtype=np.float64)
        gc = np.array([_safe_float(r.get("fusion/g_contact_mean", np.nan)) for r in fusion_infer], dtype=np.float64)
        print(f"  inj_norm_mean: min={np.nanmin(inj):.3e} max={np.nanmax(inj):.3e} mean={np.nanmean(inj):.3e}")
        print(f"  scale_mean:    min={np.nanmin(scale):.3e} max={np.nanmax(scale):.3e} mean={np.nanmean(scale):.3e}")
        print(f"  g_contact:     min={np.nanmin(gc):.3e} max={np.nanmax(gc):.3e} mean={np.nanmean(gc):.3e}")
    else:
        print("  (skip: empty fusion_infer)")
    print("-" * 100)

    # ---- export csv/json for offline digging ----
    out_dir = os.path.dirname(npz_path)
    try:
        import pandas as pd
        df_paep = pd.DataFrame(paep_ctrl)
        df_fusion = pd.DataFrame(fusion_infer)

        paep_csv = os.path.join(out_dir, "paep_ctrl_full.csv")
        fusion_csv = os.path.join(out_dir, "fusion_infer_full.csv")
        df_paep.to_csv(paep_csv, index=False)
        df_fusion.to_csv(fusion_csv, index=False)
        print(f"[EXPORT] saved: {paep_csv}")
        print(f"[EXPORT] saved: {fusion_csv}")

        # save a small json summary
        summary = {
            "npz_path": npz_path,
            "episode_idx": episode_idx,
            "control_fps": control_fps,
            "inference_fps": inference_fps,
            "len_fz": int(len(fz)),
            "len_paep_ctrl": int(len(paep_ctrl)),
            "len_fusion_infer": int(len(fusion_infer)),
            "p_contact_min": float(np.nanmin(pc)) if pc.size else None,
            "p_contact_max": float(np.nanmax(pc)) if pc.size else None,
            "p_contact_mean": float(np.nanmean(pc)) if pc.size else None,
            "entropy_min": float(np.nanmin(ent)) if ent.size else None,
            "entropy_max": float(np.nanmax(ent)) if ent.size else None,
            "entropy_mean": float(np.nanmean(ent)) if ent.size else None,
        }
        summary_path = os.path.join(out_dir, "npz_summary.json")
        with open(summary_path, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)
        print(f"[EXPORT] saved: {summary_path}")

    except ImportError:
        print("[WARN] pandas not installed; skip CSV export.")

    print("=" * 100)
    print("DONE. 😈")
    print("=" * 100)


if __name__ == "__main__":
    main()
