#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


BASE_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video/fz_logs_wmx_paep_real_plugin_usb_image_dp_absolute_24fps/20260212_182659"
NPZ_NAME = "episode_000_debug_all.npz"

# 你的 phase 定义：0/1/2/3
PHASE_NAMES = ["approach", "search", "recovery", "insert"]
NUM_PHASE = 4


def _as_list_of_dict(obj_arr):
    """npz里 dtype=object 的 dict list -> python list[dict]"""
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


def _to_1d_float_array(x):
    """把各种形态的 phase prob (list/np/float16/...) 统一成 1D float32 array"""
    if x is None:
        return None
    try:
        arr = np.asarray(x)
    except Exception:
        return None
    if arr.size == 0:
        return None
    arr = arr.reshape(-1).astype(np.float32, copy=False)
    return arr


def main():
    npz_path = os.path.join(BASE_DIR, NPZ_NAME)
    assert os.path.exists(npz_path), f"File not found: {npz_path}"

    data = np.load(npz_path, allow_pickle=True)

    # ---- Fz (control_fps) ----
    fz = np.array(data.get("fz", []), dtype=np.float32)
    fz_step = np.array(data.get("fz_control_step", []), dtype=np.int32)
    assert len(fz) == len(fz_step), "fz and fz_control_step length mismatch"

    df_fz = pd.DataFrame({
        "control_step": fz_step.astype(int),
        "Fz": fz.astype(float),
    }).sort_values("control_step").reset_index(drop=True)
    df_fz["absFz"] = df_fz["Fz"].abs()

    # ---- PAEP ctrl (control_fps) ----
    paep_ctrl = _as_list_of_dict(data.get("paep_ctrl", None))
    df_paep = pd.DataFrame(paep_ctrl)

    # guard basic cols
    for col in ["control_step", "phase_id", "p_contact"]:
        if col not in df_paep.columns:
            df_paep[col] = np.nan

    df_paep["control_step"] = pd.to_numeric(df_paep["control_step"], errors="coerce")
    df_paep["phase_id"] = pd.to_numeric(df_paep["phase_id"], errors="coerce")
    df_paep["p_contact"] = pd.to_numeric(df_paep["p_contact"], errors="coerce")
    df_paep = df_paep.dropna(subset=["control_step"]).copy()
    df_paep["control_step"] = df_paep["control_step"].astype(int)

    # ---- NEW: decode continuous phase probs ----
    # prefer p_phase_full_f16; fallback to p_phase / p_phase_full
    prob_key = None
    for k in ["p_phase_full_f16", "p_phase_full", "p_phase"]:
        if k in df_paep.columns:
            prob_key = k
            break

    # add p_phase{i} columns
    for i in range(NUM_PHASE):
        df_paep[f"p_phase{i}"] = np.nan

    if prob_key is not None:
        probs = []
        for x in df_paep[prob_key].tolist():
            arr = _to_1d_float_array(x)
            if arr is None:
                probs.append([np.nan] * NUM_PHASE)
                continue
            # 允许长度>=NUM_PHASE，只取前 NUM_PHASE
            if arr.size < NUM_PHASE:
                pad = np.full((NUM_PHASE,), np.nan, dtype=np.float32)
                pad[:arr.size] = arr
                arr = pad
            else:
                arr = arr[:NUM_PHASE]
            probs.append(arr.tolist())

        probs = np.asarray(probs, dtype=np.float32)  # (N, NUM_PHASE)
        for i in range(NUM_PHASE):
            df_paep[f"p_phase{i}"] = probs[:, i]

        # 也顺手算个 argmax_phase（如果你没存 phase_id，也能画）
        df_paep["phase_argmax"] = np.nan
        with np.errstate(invalid="ignore"):
            m = np.isfinite(probs).all(axis=1)
        df_paep.loc[m, "phase_argmax"] = np.argmax(probs[m], axis=1).astype(np.float32)

    df_paep = df_paep.sort_values("control_step").reset_index(drop=True)

    # ---- fusion infer (inference_fps) ----
    fusion_infer = _as_list_of_dict(data.get("fusion_infer", None))
    df_fusion = pd.DataFrame(fusion_infer)

    for col in ["control_step", "fusion/inj_norm_mean", "fusion/scale_mean", "fusion/g_contact_mean", "fusion/effective_ratio_mean"]:
        if col not in df_fusion.columns:
            df_fusion[col] = np.nan

    df_fusion["control_step"] = pd.to_numeric(df_fusion["control_step"], errors="coerce")
    df_fusion = df_fusion.dropna(subset=["control_step"]).copy()
    df_fusion["control_step"] = df_fusion["control_step"].astype(int)

    df_fusion["inj_norm_mean"] = pd.to_numeric(df_fusion["fusion/inj_norm_mean"], errors="coerce")
    df_fusion["scale_mean"] = pd.to_numeric(df_fusion["fusion/scale_mean"], errors="coerce")
    df_fusion["g_contact_mean"] = pd.to_numeric(df_fusion["fusion/g_contact_mean"], errors="coerce")
    df_fusion["eff_ratio_mean"] = pd.to_numeric(df_fusion["fusion/effective_ratio_mean"], errors="coerce")
    df_fusion = df_fusion[["control_step", "inj_norm_mean", "scale_mean", "g_contact_mean", "eff_ratio_mean"]].sort_values("control_step")

    # ---- merge on control_step (dense timeline = control_fps) ----
    df = df_fz.merge(df_paep, on="control_step", how="left")
    df = df.merge(df_fusion, on="control_step", how="left")
    df["inj_norm_ffill"] = df["inj_norm_mean"].ffill()
    df["scale_ffill"] = df["scale_mean"].ffill()

    # ---- plotting ----
    fig, axes = plt.subplots(3, 1, figsize=(16, 11), sharex=True)
    x = df["control_step"].to_numpy()

    # (1) Fz
    ax0 = axes[0]
    ax0.plot(x, df["Fz"].to_numpy(), linewidth=1.2, label="Fz (N)")
    ax0.plot(x, df["absFz"].to_numpy(), linewidth=1.0, linestyle="--", label="|Fz| (N)")
    ax0.set_ylabel("Force (N)")
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc="upper right")

    # (2) injection strength (fusion)
    ax1 = axes[1]
    ax1.plot(x, df["inj_norm_ffill"].to_numpy(), linewidth=1.2, label="inj_norm_mean (ffill)")
    ax1.plot(x, df["scale_ffill"].to_numpy(), linewidth=1.0, linestyle="--", label="scale_mean (ffill)")
    m = np.isfinite(df["inj_norm_mean"].to_numpy())
    ax1.scatter(x[m], df.loc[m, "inj_norm_mean"].to_numpy(), s=14, label="inj_norm_mean (infer pts)")
    ax1.set_ylabel("Injection strength")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")

    # (3) phase probabilities + phase_id + p_contact
    ax2 = axes[2]

    # 画连续概率（如果存在）
    has_probs = all([f"p_phase{i}" in df.columns for i in range(NUM_PHASE)])
    if has_probs and np.isfinite(df[[f"p_phase{i}" for i in range(NUM_PHASE)]].to_numpy()).any():
        for i in range(NUM_PHASE):
            ax2.plot(
                x,
                df[f"p_phase{i}"].to_numpy(),
                linewidth=1.3,
                alpha=0.9,
                label=f"p_phase{i} ({PHASE_NAMES[i]})"
            )
        ax2.set_ylim(-0.05, 1.05)
        ax2.set_ylabel("p_phase (prob)")

        # 同时画一个 argmax 的 phase（淡一些，便于对齐你原来的 phase_id 观感）
        phase_line = None
        if "phase_id" in df.columns and np.isfinite(df["phase_id"].to_numpy()).any():
            phase_line = df["phase_id"].to_numpy()
        elif "phase_argmax" in df.columns and np.isfinite(df["phase_argmax"].to_numpy()).any():
            phase_line = df["phase_argmax"].to_numpy()

        if phase_line is not None:
            ax2.plot(x, phase_line / (NUM_PHASE - 1), linewidth=1.0, alpha=0.35, label="phase_id/argmax (scaled)")
    else:
        # fallback：只有 phase_id
        ax2.step(x, df["phase_id"].to_numpy(), where="post", linewidth=1.5, label="phase_id")
        ax2.set_ylabel("phase_id")

    ax2.grid(True, alpha=0.3)

    # overlay p_contact on a second y-axis
    ax2b = ax2.twinx()
    ax2b.plot(x, df["p_contact"].to_numpy(), linewidth=1.2, alpha=0.8, label="p_contact")
    ax2b.set_ylim(-0.05, 1.05)
    ax2b.set_ylabel("p_contact")

    # merge legends
    h1, l1 = ax2.get_legend_handles_labels()
    h2, l2 = ax2b.get_legend_handles_labels()
    ax2.legend(h1 + h2, l1 + l2, loc="upper right", ncol=2)

    ax2.set_xlabel("control_step")

    # title + save
    ep = int(np.array(data.get("episode_idx", [0])).reshape(-1)[0])
    cfps = int(np.array(data.get("control_fps", [-1])).reshape(-1)[0])
    ifps = int(np.array(data.get("inference_fps", [-1])).reshape(-1)[0])
    fig.suptitle(
        f"Episode {ep:03d} | control_fps={cfps} inference_fps={ifps} | Phase Prob / Injection / Fz",
        y=0.995
    )

    fig.tight_layout()
    out_path = os.path.join(BASE_DIR, "viz_phaseProb_inj_fz_episode000.png")
    fig.savefig(out_path, dpi=220)
    print(f"[OK] saved figure to: {out_path}")

    # also save merged csv for debugging
    csv_path = os.path.join(BASE_DIR, "merged_phaseProb_inj_fz_episode000.csv")
    df.to_csv(csv_path, index=False)
    print(f"[OK] saved merged csv to: {csv_path}")


if __name__ == "__main__":
    main()
