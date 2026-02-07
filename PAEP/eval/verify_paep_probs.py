#读取 .npy 概率 + 从 zarr 读 hard label（paep_phase/paep_contact），算 acc、AUC（粗略）、画直方图/时间曲线/episode 窗口对齐图。

import os, json
import numpy as np
import matplotlib.pyplot as plt
import zarr

ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr"
PROB_DIR  = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/label_for_fast/offline_wiping_board_paep_prob_out"
OUT_DIR   = PROB_DIR
os.makedirs(OUT_DIR, exist_ok=True)

PHASE_KEY = "paep_phase"     # hard id 0/1/2
CONTACT_KEY = "paep_contact" # hard 0/1

PHASE_PROB = os.path.join(PROB_DIR, "paep_prob_phase.npy")
CONTACT_PROB = os.path.join(PROB_DIR, "paep_prob_contact.npy")

def open_group(zarr_path: str):
    root = zarr.open(zarr_path, mode="r")
    if "external_img" in root.array_keys():
        g = root
    elif "data" in root.group_keys() and "external_img" in root["data"].array_keys():
        g = root["data"]
    else:
        g = root
    # episode_ends in meta
    if "meta" in root.group_keys() and "episode_ends" in root["meta"].array_keys():
        episode_ends = np.array(root["meta"]["episode_ends"][:], dtype=np.int64)
    else:
        episode_ends = None
    return g, episode_ends

def auc_approx(y_true01, y_score):
    """Fast AUC via rank statistics (no sklearn). y_true in {0,1}."""
    y_true01 = y_true01.astype(np.int64).reshape(-1)
    y_score = y_score.astype(np.float64).reshape(-1)
    n1 = int(y_true01.sum())
    n0 = int(y_true01.shape[0] - n1)
    if n0 == 0 or n1 == 0:
        return float("nan")
    order = np.argsort(y_score)
    ranks = np.empty_like(order)
    ranks[order] = np.arange(1, len(y_score)+1)
    sum_ranks_pos = ranks[y_true01 == 1].sum()
    auc = (sum_ranks_pos - n1*(n1+1)/2) / (n0*n1)
    return float(auc)

def main():
    p_phase = np.load(PHASE_PROB).astype(np.float32)         # (T,3)
    p_contact = np.load(CONTACT_PROB).astype(np.float32)     # (T,1) or (T,)
    if p_contact.ndim == 1:
        p_contact = p_contact[:, None]

    T = p_phase.shape[0]
    assert p_phase.shape == (T,3), f"phase prob shape wrong: {p_phase.shape}"
    assert p_contact.shape[0] == T, f"contact prob T mismatch: {p_contact.shape}"

    # numeric sanity
    phase_sum = p_phase.sum(axis=1)
    nan_cnt = int(np.isnan(p_phase).sum() + np.isnan(p_contact).sum())
    inf_cnt = int(np.isinf(p_phase).sum() + np.isinf(p_contact).sum())

    # open zarr & hard labels
    g, episode_ends = open_group(ZARR_PATH)
    phase_hard = np.array(g[PHASE_KEY][:], dtype=np.int64).reshape(-1)
    contact_hard = np.array(g[CONTACT_KEY][:], dtype=np.int64).reshape(-1)
    assert phase_hard.shape[0] == T and contact_hard.shape[0] == T

    phase_pred = np.argmax(p_phase, axis=1).astype(np.int64)
    acc_phase = float((phase_pred == phase_hard).mean())

    contact_score = p_contact[:,0]
    contact_pred = (contact_score >= 0.5).astype(np.int64)
    acc_contact = float((contact_pred == contact_hard).mean())
    auc_contact = auc_approx(contact_hard, contact_score)

    stats = {
        "T": int(T),
        "nan_cnt": nan_cnt,
        "inf_cnt": inf_cnt,
        "phase_prob_sum_mean": float(phase_sum.mean()),
        "phase_prob_sum_std": float(phase_sum.std()),
        "phase_prob_min": float(p_phase.min()),
        "phase_prob_max": float(p_phase.max()),
        "contact_prob_min": float(contact_score.min()),
        "contact_prob_max": float(contact_score.max()),
        "phase_acc_argmax_vs_hard": acc_phase,
        "contact_acc_thr0.5_vs_hard": acc_contact,
        "contact_auc_vs_hard": auc_contact,
    }

    # ---- plots ----
    # 1) global histogram
    plt.figure()
    plt.hist(contact_score, bins=50)
    plt.title("p_contact histogram")
    plt.xlabel("p_contact"); plt.ylabel("count")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "verify_contact_hist.png"), dpi=200)
    plt.close()

    # 2) phase mean curve (optional)
    plt.figure()
    plt.plot(p_phase[:,0], label="p_phase[0]")
    plt.plot(p_phase[:,1], label="p_phase[1]")
    plt.plot(p_phase[:,2], label="p_phase[2]")
    plt.legend()
    plt.title("p_phase over time (full)")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "verify_phase_full.png"), dpi=200)
    plt.close()

    # 3) overlay a few episode windows
    if episode_ends is not None and len(episode_ends) >= 3:
        # pick 3 episodes: early/mid/late
        ends = episode_ends.tolist()
        starts = [0] + ends[:-1]
        idx = [0, len(ends)//2, len(ends)-1]
        for j, ei in enumerate(idx):
            s = starts[ei]; e = ends[ei]
            # take first 500 frames or full
            w = min(500, e - s)
            tt = np.arange(w)
            plt.figure(figsize=(10,4))
            plt.plot(tt, contact_hard[s:s+w], label="contact_hard", linewidth=1)
            plt.plot(tt, contact_score[s:s+w], label="p_contact", linewidth=1)
            plt.plot(tt, phase_hard[s:s+w]/2.0, label="phase_hard/2", linewidth=1)  # scaled
            plt.plot(tt, phase_pred[s:s+w]/2.0, label="phase_argmax/2", linewidth=1)
            plt.title(f"Episode {ei} window [s={s}, e={e}] first {w} frames")
            plt.legend()
            plt.tight_layout()
            plt.savefig(os.path.join(OUT_DIR, f"verify_ep{ei}_window.png"), dpi=200)
            plt.close()

        # 4) boundary check around episode cuts
        for cut_i, cut in enumerate(ends[:5]):  # first 5 cuts
            s = max(0, cut-120); e = min(T, cut+120)
            tt = np.arange(s, e) - cut
            plt.figure(figsize=(10,4))
            plt.plot(tt, contact_score[s:e], label="p_contact")
            plt.plot(tt, contact_hard[s:e], label="contact_hard")
            plt.axvline(0, linestyle="--")
            plt.title(f"Boundary check around cut#{cut_i} (t=0 at episode end)")
            plt.legend()
            plt.tight_layout()
            plt.savefig(os.path.join(OUT_DIR, f"verify_cut{cut_i}.png"), dpi=200)
            plt.close()

    with open(os.path.join(OUT_DIR, "verify_paep_prob_report.json"), "w") as f:
        json.dump(stats, f, indent=2)

    print("[OK] wrote report to:", OUT_DIR)
    print(json.dumps(stats, indent=2))

if __name__ == "__main__":
    main()
