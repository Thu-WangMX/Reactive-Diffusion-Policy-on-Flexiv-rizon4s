#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


BASE_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/video/fz_logs_wmx_paep_real_plugin_charger_image_dp_absolute_24fps/20260209_181245"
NPZ_NAME = "episode_000_debug_all.npz"


def _as_list_of_dict(obj_arr):
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


def main():
    npz_path = os.path.join(BASE_DIR, NPZ_NAME)
    assert os.path.exists(npz_path), f"File not found: {npz_path}"
    data = np.load(npz_path, allow_pickle=True)

    ep = int(np.array(data.get("episode_idx", [0])).reshape(-1)[0])
    cfps = int(np.array(data.get("control_fps", [-1])).reshape(-1)[0])
    ifps = int(np.array(data.get("inference_fps", [-1])).reshape(-1)[0])

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
    for col in ["control_step", "phase_id", "p_contact", "p_phase_entropy"]:
        if col not in df_paep.columns:
            df_paep[col] = np.nan
    df_paep["control_step"] = df_paep["control_step"].astype(int)
    df_paep["phase_id"] = pd.to_numeric(df_paep["phase_id"], errors="coerce")
    df_paep["p_contact"] = pd.to_numeric(df_paep["p_contact"], errors="coerce")
    df_paep["p_phase_entropy"] = pd.to_numeric(df_paep["p_phase_entropy"], errors="coerce")
    df_paep = df_paep[["control_step", "phase_id", "p_contact", "p_phase_entropy"]].sort_values("control_step")

    # ---- fusion infer (inference_fps) ----
    fusion_infer = _as_list_of_dict(data.get("fusion_infer", None))
    df_fusion = pd.DataFrame(fusion_infer)
    for col in ["control_step", "fusion/inj_norm_mean", "fusion/scale_mean", "fusion/g_contact_mean", "fusion/effective_ratio_mean", "fusion/alpha"]:
        if col not in df_fusion.columns:
            df_fusion[col] = np.nan
    df_fusion["control_step"] = df_fusion["control_step"].astype(int)
    df_fusion["inj_norm_mean"] = pd.to_numeric(df_fusion["fusion/inj_norm_mean"], errors="coerce")
    df_fusion["scale_mean"] = pd.to_numeric(df_fusion["fusion/scale_mean"], errors="coerce")
    df_fusion["g_contact_mean"] = pd.to_numeric(df_fusion["fusion/g_contact_mean"], errors="coerce")
    df_fusion["eff_ratio_mean"] = pd.to_numeric(df_fusion["fusion/effective_ratio_mean"], errors="coerce")
    df_fusion["alpha"] = pd.to_numeric(df_fusion["fusion/alpha"], errors="coerce")
    df_fusion = df_fusion[["control_step", "inj_norm_mean", "scale_mean", "g_contact_mean", "eff_ratio_mean", "alpha"]].sort_values("control_step")

    # ---- merge on control_step (dense timeline) ----
    df = df_fz.merge(df_paep, on="control_step", how="left")
    df = df.merge(df_fusion, on="control_step", how="left")

    # continuous line version for injection strength (ffill)
    df["inj_norm_ffill"] = df["inj_norm_mean"].ffill()
    df["scale_ffill"] = df["scale_mean"].ffill()

    # =========================
    # FIG 1: overview (3 rows)
    # =========================
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    x = df["control_step"].to_numpy()

    # (1) Fz
    ax0 = axes[0]
    ax0.plot(x, df["Fz"].to_numpy(), linewidth=1.2, label="Fz (N)")
    ax0.plot(x, df["absFz"].to_numpy(), linewidth=1.0, linestyle="--", label="|Fz| (N)")
    ax0.set_ylabel("Force (N)")
    ax0.grid(True, alpha=0.3)
    ax0.legend(loc="upper right")

    # (2) injection strength
    ax1 = axes[1]
    ax1.plot(x, df["inj_norm_ffill"].to_numpy(), linewidth=1.2, label="inj_norm_mean (ffill)")
    ax1.plot(x, df["scale_ffill"].to_numpy(), linewidth=1.0, linestyle="--", label="scale_mean (ffill)")
    m = np.isfinite(df["inj_norm_mean"].to_numpy())
    ax1.scatter(x[m], df.loc[m, "inj_norm_mean"].to_numpy(), s=12, label="inj_norm_mean (infer pts)")
    ax1.set_ylabel("Injection strength")
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")

    # (3) phase + p_contact
    ax2 = axes[2]
    ax2.step(x, df["phase_id"].to_numpy(), where="post", linewidth=1.5, label="phase_id")
    ax2.set_ylabel("phase_id")
    ax2.set_xlabel("control_step")
    ax2.grid(True, alpha=0.3)

    ax2b = ax2.twinx()
    ax2b.plot(x, df["p_contact"].to_numpy(), linewidth=1.0, alpha=0.85, label="p_contact")
    ax2b.set_ylabel("p_contact")

    h1, l1 = ax2.get_legend_handles_labels()
    h2, l2 = ax2b.get_legend_handles_labels()
    ax2.legend(h1 + h2, l1 + l2, loc="upper right")

    fig.suptitle(f"Episode {ep:03d} | control_fps={cfps} inference_fps={ifps} | Overview: Phase / Injection / Fz", y=0.995)
    fig.tight_layout()

    out1 = os.path.join(BASE_DIR, "viz_overview_episode000.png")
    fig.savefig(out1, dpi=200)
    print(f"[OK] saved: {out1}")

    # ===================================
    # FIG 2: scatter |Fz| vs p_contact
    # ===================================
    fig2 = plt.figure(figsize=(10, 6))
    ax = plt.gca()
    # drop NaNs
    dd = df[["absFz", "p_contact", "phase_id"]].dropna()
    ax.scatter(dd["absFz"].to_numpy(), dd["p_contact"].to_numpy(), s=8)
    ax.set_xlabel("|Fz| (N)")
    ax.set_ylabel("p_contact")
    ax.grid(True, alpha=0.3)
    ax.set_title(f"Episode {ep:03d} | Scatter: p_contact vs |Fz|")

    out2 = os.path.join(BASE_DIR, "viz_scatter_pcontact_absFz_episode000.png")
    fig2.tight_layout()
    fig2.savefig(out2, dpi=200)
    print(f"[OK] saved: {out2}")

    # ===================================
    # FIG 3: boxplot |Fz| grouped by phase
    # ===================================
    fig3 = plt.figure(figsize=(10, 6))
    ax = plt.gca()
    dd2 = df[["absFz", "phase_id"]].dropna()
    # coerce phase_id to int bins
    dd2["phase_id_int"] = dd2["phase_id"].astype(int)

    phases = sorted(dd2["phase_id_int"].unique().tolist())
    groups = [dd2.loc[dd2["phase_id_int"] == p, "absFz"].to_numpy() for p in phases]

    ax.boxplot(groups, labels=[str(p) for p in phases], showfliers=False)
    ax.set_xlabel("phase_id")
    ax.set_ylabel("|Fz| (N)")
    ax.grid(True, alpha=0.3)
    ax.set_title(f"Episode {ep:03d} | |Fz| distribution by phase_id")

    out3 = os.path.join(BASE_DIR, "viz_box_absFz_by_phase_episode000.png")
    fig3.tight_layout()
    fig3.savefig(out3, dpi=200)
    print(f"[OK] saved: {out3}")

    # ---- dump merged csv (optional) ----
    merged_csv = os.path.join(BASE_DIR, "merged_phase_inj_fz_episode000.csv")
    df.to_csv(merged_csv, index=False)
    print(f"[OK] saved merged csv: {merged_csv}")


if __name__ == "__main__":
    main()
