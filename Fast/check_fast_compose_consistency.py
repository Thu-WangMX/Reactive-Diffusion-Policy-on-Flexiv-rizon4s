#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import numpy as np

def rot_angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    """Return angle between two rotation matrices in degrees."""
    R = Ra.T @ Rb
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(tr)))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base_abs_npy", type=str, required=True, help="teacherA_slow_exec_9d.npy or any base abs (T,9)")
    ap.add_argument("--target_abs_npy", type=str, required=True, help="teacherB_virtual_target_9d.npy or expert abs (T,9)")
    ap.add_argument("--num_check", type=int, default=200)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    base_abs = np.load(args.base_abs_npy)   # (T,9)
    target_abs = np.load(args.target_abs_npy)  # (T,9)
    assert base_abs.ndim == 2 and base_abs.shape[1] >= 9
    assert target_abs.ndim == 2 and target_abs.shape[1] >= 9
    T = min(len(base_abs), len(target_abs))
    base_abs = base_abs[:T, :9].astype(np.float64)
    target_abs = target_abs[:T, :9].astype(np.float64)

    # -------- import your utils --------
    try:
        # 你如果在 Reactive-Diffusion-Policy-on-Flexiv-rizon4s 里，常见是这个
        from reactive_diffusion_policy.common.action_utils import (
            pose_3d_9d_to_homo_matrix_batch,
            homo_matrix_to_pose_9d_batch,
            absolute_actions_to_relative_actions,
        )
    except Exception:
        # 备选：你自己放 action_utils.py 的路径（按需改）
        from common.action_utils import (
            pose_3d_9d_to_homo_matrix_batch,
            homo_matrix_to_pose_9d_batch,
            absolute_actions_to_relative_actions,
        )

    rng = np.random.default_rng(args.seed)
    idx = rng.choice(T, size=min(args.num_check, T), replace=False)

    pos_err_exact = []
    rot_err_exact = []
    pos_err_approx_ee = []
    pos_err_wrong_base = []

    for t in idx:
        base = base_abs[t:t+1]      # (1,9)
        target = target_abs[t:t+1]  # (1,9)

        # rel = inv(T_base) @ T_target
        rel = absolute_actions_to_relative_actions(target.copy(), base_absolute_action=base[0]).astype(np.float64)  # (1,9)

        T_base = pose_3d_9d_to_homo_matrix_batch(base)      # (1,4,4)
        T_rel  = pose_3d_9d_to_homo_matrix_batch(rel)       # (1,4,4)
        T_tar  = pose_3d_9d_to_homo_matrix_batch(target)    # (1,4,4)

        # (A) exact compose: T_base @ T_rel
        T_comp = T_base @ T_rel
        comp_pose = homo_matrix_to_pose_9d_batch(T_comp)[0]   # (9,)
        tar_pose  = target[0]

        # exact errors
        pos_err_exact.append(np.linalg.norm(comp_pose[:3] - tar_pose[:3]))
        rot_err_exact.append(rot_angle_deg(T_comp[0,:3,:3], T_tar[0,:3,:3]))

        # (B) deploy-like approx for translation: p + R_base @ dp_ee
        Rb = T_base[0,:3,:3]
        dp_ee = rel[0,:3]
        p_approx = base[0,:3] + (Rb @ dp_ee)
        pos_err_approx_ee.append(np.linalg.norm(p_approx - tar_pose[:3]))

        # (C) wrong hypothesis: dp is already in base/world frame
        p_wrong = base[0,:3] + dp_ee
        pos_err_wrong_base.append(np.linalg.norm(p_wrong - tar_pose[:3]))

    def stat(x):
        x = np.array(x, dtype=np.float64)
        return dict(mean=float(x.mean()), p50=float(np.percentile(x, 50)), p90=float(np.percentile(x, 90)), max=float(x.max()))

    print("\n=== CHECK RESULTS (sampled) ===")
    print(f"T_total={T}, checked={len(idx)}")
    print("\n[1] Exact matrix compose: T_base @ T_rel vs target")
    print("  pos_err(m):", stat(pos_err_exact))
    print("  rot_err(deg):", stat(rot_err_exact))

    print("\n[2] Deploy approx translation: p_base + R_base @ dp_ee  (rotation ignored)")
    print("  pos_err(m):", stat(pos_err_approx_ee))

    print("\n[3] Wrong add (dp treated as base/world): p_base + dp")
    print("  pos_err(m):", stat(pos_err_wrong_base))

    # decision hint
    if np.mean(pos_err_approx_ee) < np.mean(pos_err_wrong_base):
        print("\n✅ dp is consistent with EE-frame (R_base @ dp) composition.")
    else:
        print("\n⚠️ dp behaves more like base/world-frame diff. Re-check your compose logic or pose convention.")

if __name__ == "__main__":
    main()
