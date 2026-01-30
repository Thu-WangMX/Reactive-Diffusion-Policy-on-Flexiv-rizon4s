#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
offline_label_teacherB.py

Wiping 任务 TeacherB 打标（仅法向/下压纠偏，phase-conditioned）。
依赖：
- zarr: left_robot_tcp_wrench, paep_phase, paep_contact
- TeacherA: teacherA_slow_exec_9d.npy (T,9) 作为 base_abs

输出：
- teacherB_virtual_target_9d.npy  (T,9)  # 虚拟目标绝对位姿
- teacherB_fast_label_9d.npy      (T,9)  # 相对 residual：base^-1 * virtual
- teacherB_label_meta.json
"""

import os
import json
import numpy as np

from reactive_diffusion_policy.common.replay_buffer import ReplayBuffer
from reactive_diffusion_policy.common.action_utils import absolute_actions_to_relative_actions
from reactive_diffusion_policy.common.space_utils import ortho6d_to_rotation_matrix


# =========================
# 0) 只需要改这些路径/参数
# =========================
ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr"

TEACHER_A_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/label_for_fast/offline_wiping_board_teacherA_out"
SLOW_EXEC_PATH = os.path.join(TEACHER_A_DIR, "teacherA_slow_exec_9d.npy")

OUT_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/label_for_fast/offline_wiping_board_teacherB_out"

# --- force control params ---
FZ_INDEX = 2          # left_robot_tcp_wrench[:,2]
FZ_TARGET = -20.0     # N (你给定)
K_STIFF = 8000.0      # N/m  (你要的“刚度”)  10N误差动1cm
DZ_CLIP = 0.003       # 每个step生成的最大delta z (m)
CONTACT_TH = 0.5      # paep_contact >= 0.5，TeacherB 才输出非零 residual

# --- phase map ---
PHASE_PROGRESS = 1    # 0/1/2 = approach/progress/done


# =========================
# utils: rot6d -> R, identity rel 9d
# =========================
def _normalize(v: np.ndarray, eps: float = 1e-8) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.zeros_like(v)
    return v / n


# def rot6d_to_R(d6):
#     a1 = d6[:3]; a2 = d6[3:]
#     b1 = _normalize(a1)
#     b2 = a2 - np.dot(b1, a2) * b1
#     b2 = _normalize(b2)

#     # ✅ 在这里加：退化保护（必须在 b3、R 之前）
#     if np.linalg.norm(b1) < 1e-6 or np.linalg.norm(b2) < 1e-6:
#         return np.eye(3, dtype=np.float32)

#     b3 = _normalize(np.cross(b1, b2))   # ✅ 这里把 b3 也 normalize
#     R = np.stack([b1, b2, b3], axis=1).astype(np.float32)
#     return R



# def identity_rel_9d() -> np.ndarray:
#     """
#     9D: [p(3), rot6d(6)]
#     identity rot6d (I): b1=[1,0,0], b2=[0,1,0]
#     """
#     out = np.zeros((9,), dtype=np.float32)
#     out[3:6] = np.array([1, 0, 0], dtype=np.float32)
#     out[6:9] = np.array([0, 1, 0], dtype=np.float32)
#     return out


# =========================
# main
# =========================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    slow_exec = np.load(SLOW_EXEC_PATH).astype(np.float32)  # (T,9)
    T = int(slow_exec.shape[0])

    # load minimal keys from zarr
    keys = ["left_robot_tcp_wrench", "paep_phase", "paep_contact"]
    rb = ReplayBuffer.copy_from_path(ZARR_PATH, keys=keys)

    wrench = np.asarray(rb["left_robot_tcp_wrench"][:, :6], dtype=np.float32)  # (T,6)
    phase = np.asarray(rb["paep_phase"][:], dtype=np.int64).reshape(-1)        # (T,)
    contact = np.asarray(rb["paep_contact"][:], dtype=np.float32).reshape(-1) # (T,)
    
    print("[DBG] phase uniq:", np.unique(phase)[:10])
    print("[DBG] contact uniq:", np.unique(contact)[:10])



    assert wrench.shape[0] == T and phase.shape[0] == T and contact.shape[0] == T, \
        f"Length mismatch: slow_exec={T}, wrench={wrench.shape[0]}, phase={phase.shape[0]}, contact={contact.shape[0]}"

    # outputs
    virtual_abs = slow_exec.copy()                 # (T,9)
    label_rel = np.zeros((T, 9), dtype=np.float32)

    c = 1.0 / float(K_STIFF)                       # compliance (m/N)
    

    dz_list = []
    active_count = 0

    for t in range(T):
        base = slow_exec[t]  # (9,) absolute executed pose in base frame

        # gate by phase/contact
        if not (phase[t] == PHASE_PROGRESS and contact[t] >= CONTACT_TH):
            rel0 = absolute_actions_to_relative_actions(
                actions=base[None, :],          # target == base
                base_absolute_action=base        # base
            ).reshape(-1).astype(np.float32)
            label_rel[t] = rel0
            continue

        Fz = float(wrench[t, FZ_INDEX])           # 你说的法向力（负值接触更大）
        dz_tool = (Fz - float(FZ_TARGET)) * c     # tool-frame down is positive
        dz_tool = float(np.clip(dz_tool, -DZ_CLIP, DZ_CLIP))

        # project tool-z to base: dp_base = R[:,2] * dz_tool
        
        R = ortho6d_to_rotation_matrix(base[None, 3:9])[0]  # (3,3)
        tool_z_in_base = R[:, 2]
        
        if (t % 2000) == 0:
            print("[DBG] t", t, "Fz", Fz, "dz_tool", dz_tool, "tool_z_in_base", tool_z_in_base)

        dp_base = tool_z_in_base * dz_tool

        vab = base.copy()
        vab[:3] = vab[:3] + dp_base.astype(np.float32)  # rotation unchanged
        virtual_abs[t] = vab

        # label: relative(virtual_abs, base=slow_exec)
        rel = absolute_actions_to_relative_actions(
            actions=vab[None, :],             # (1,9)
            base_absolute_action=base         # (9,)
        )
        label_rel[t] = rel.reshape(-1).astype(np.float32)

        dz_list.append(dz_tool)
        active_count += 1

    # save
    out_virtual = os.path.join(OUT_DIR, "teacherB_virtual_target_9d.npy")
    out_label = os.path.join(OUT_DIR, "teacherB_fast_label_9d.npy")
    np.save(out_virtual, virtual_abs)
    np.save(out_label, label_rel)

    dz_arr = np.asarray(dz_list, dtype=np.float32) if len(dz_list) else np.zeros((0,), dtype=np.float32)
    meta = {
        "zarr_path": ZARR_PATH,
        "teacherA_slow_exec_path": SLOW_EXEC_PATH,
        "T": int(T),
        "keys_used": keys,
        "phase_progress": int(PHASE_PROGRESS),
        "contact_th": float(CONTACT_TH),
        "fz_index": int(FZ_INDEX),
        "fz_target": float(FZ_TARGET),
        "k_stiff": float(K_STIFF),
        "compliance_m_per_N": float(c),
        "dz_clip_m": float(DZ_CLIP),
        "active_frames": int(active_count),
        "dz_tool_mean_m": float(dz_arr.mean()) if dz_arr.size else 0.0,
        "dz_tool_abs_max_m": float(np.abs(dz_arr).max()) if dz_arr.size else 0.0,
        "out_virtual": out_virtual,
        "out_label": out_label,
    }
    out_meta = os.path.join(OUT_DIR, "teacherB_label_meta.json")
    with open(out_meta, "w") as f:
        json.dump(meta, f, indent=2)

    print(f"[SAVE] virtual_abs -> {out_virtual}")
    print(f"[SAVE] label_rel   -> {out_label}")
    print(f"[SAVE] meta        -> {out_meta}")


if __name__ == "__main__":
    main()
