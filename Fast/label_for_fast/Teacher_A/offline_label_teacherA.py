#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""offline_label_teacherA.py

离线模拟实机 ours_real_runner 的 TCP 执行语义（不含 gripper），用于给 Fast 打 Teacher-A 标签。

对齐实机的关键点：
- control_fps / inference_fps -> steps_per_inference（只有在这些时刻做推理）
- tcp_action_update_interval（只有在这些时刻才把预测序列 push 进 EnsembleBuffer）
- latency_step（push 前丢掉前 latency_step 帧：tcp_seq = action_all[latency_step:, :9]）
- 每个 control step 都 get_action()；若 None 则沿用上一帧 last_tcp_step_action（与实机一致）

输出：
- teacherA_slow_exec_9d.npy  (T,9)
- teacherA_fast_label_9d.npy (T,9) = relative(expert_abs, base_abs=slow_exec)

直接运行：
  python Fast/label_for_fast/offline_label_teacherA.py

只需按需改顶部常量（ckpt、输出目录）。
"""

import os
import json
import time
import pathlib
import pickle
from typing import Dict, Optional, Tuple

import numpy as np
import torch
import dill
import hydra
from hydra import initialize_config_dir, compose
from omegaconf import OmegaConf

from reactive_diffusion_policy.workspace.base_workspace import BaseWorkspace
from reactive_diffusion_policy.common.pytorch_util import dict_apply
from reactive_diffusion_policy.common.replay_buffer import ReplayBuffer
from reactive_diffusion_policy.common.ensemble import EnsembleBuffer
from reactive_diffusion_policy.common.action_utils import absolute_actions_to_relative_actions

# =========================
# 0) 只需要改这些
# =========================
CONFIG_NAME = "train_paep_diffusion_unet_real_image_workspace"
TASK_NAME = "wmx_paep_real_wiping_board_image_dp_absolute_24fps"

DIFF_CKPT_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/ckpt/OURS_v4_0128/epoch0400-train_loss0.001.ckpt"
PAEP_CKPT_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/PAEP/paep_future_ckpt/paep_runs_0124/best.pt"  # 可设为 None
USE_EMA = True
NUM_INFERENCE_STEPS = 8
DEVICE = "cuda:0" if torch.cuda.is_available() else "cpu"

OUT_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/Fast/label_for_fast/offline_wiping_board_teacherA_out"

# =========================
# 1) Hydra / ckpt load
# =========================
OmegaConf.register_new_resolver("eval", eval, replace=True)


def _set_num_inference_steps(p, n):
    # try common wrappers: .policy / .model / .averaged_model / .module
    for cand in [p, getattr(p,"policy",None), getattr(p,"model",None),
                 getattr(p,"averaged_model",None), getattr(p,"module",None)]:
        if cand is not None and hasattr(cand, "num_inference_steps"):
            cand.num_inference_steps = int(n)

def _get_nsteps(p):
    for cand in [p, getattr(p,"policy",None), getattr(p,"model",None),
                 getattr(p,"averaged_model",None), getattr(p,"module",None)]:
        if cand is not None and hasattr(cand, "num_inference_steps"):
            return cand.num_inference_steps
    return None

def _find_repo_root(start: pathlib.Path) -> pathlib.Path:
    
    for p in [start] + list(start.parents):
        if (p / "reactive_diffusion_policy").is_dir():
            return p
    raise RuntimeError(f"Cannot find repo root from: {start}")


def _load_torch(ckpt_path: str):
    with open(ckpt_path, "rb") as f:
        try:
            return torch.load(f, pickle_module=dill, map_location="cpu")
        except Exception:
            f.seek(0)
            return torch.load(f, map_location="cpu")


def _pick_policy_from_workspace(cfg, workspace: BaseWorkspace):
    policy = getattr(workspace, "model", None)
    if policy is None:
        raise RuntimeError("workspace.model not found")

    use_ema = False
    try:
        use_ema = bool(getattr(cfg, "training", {}).get("use_ema", False))
    except Exception:
        pass

    if USE_EMA and use_ema and hasattr(workspace, "ema_model") and (workspace.ema_model is not None):
        ema_policy = workspace.ema_model
        # 同步 normalizer（与 eval 脚本一致）
        if hasattr(workspace, "model") and hasattr(workspace.model, "normalizer"):
            try:
                ema_policy.set_normalizer(workspace.model.normalizer)
            except Exception:
                ema_policy.normalizer = workspace.model.normalizer
        return ema_policy
    return policy


def _get_pred_action_tensor(pred):
    if isinstance(pred, dict):
        if "action" in pred:
            return pred["action"]
        if "actions" in pred:
            return pred["actions"]
        for v in pred.values():
            if torch.is_tensor(v):
                return v
        raise RuntimeError(f"predict_action dict has no tensor. keys={list(pred.keys())}")
    if torch.is_tensor(pred):
        return pred
    raise RuntimeError(f"predict_action returned unsupported type: {type(pred)}")


def build_cfg() -> "OmegaConf":
    repo_root = _find_repo_root(pathlib.Path(__file__).resolve())
    config_dir = repo_root / "reactive_diffusion_policy" / "config"

    overrides = [
        f"task={TASK_NAME}",
        f"+diff_ckpt_path={DIFF_CKPT_PATH}",
        f"+num_inference_steps={NUM_INFERENCE_STEPS}",
    ]
    if PAEP_CKPT_PATH is not None:
        overrides.append(f"+paep_ckpt_path={PAEP_CKPT_PATH}")

    with initialize_config_dir(version_base=None, config_dir=str(config_dir)):
        cfg = compose(config_name=CONFIG_NAME, overrides=overrides)
    return cfg


# =========================
# 2) replay -> obs 构建（对齐 env.get_obs + wrench_hist）
# =========================

def _episode_bounds(episode_ends: np.ndarray, t: int) -> Tuple[int, int, int]:
    eid = int(np.searchsorted(episode_ends, t, side="right"))
    ep_start = 0 if eid == 0 else int(episode_ends[eid - 1])
    ep_end = int(episode_ends[eid])
    return eid, ep_start, ep_end


def _sparse_indices(t: int, ep_start: int, To: int, ratio: int) -> np.ndarray:
    # indices: [t-(To-1)*ratio, ..., t] step=ratio，越界则 clamp 到 ep_start
    idxs = np.array([t - (To - 1 - k) * ratio for k in range(To)], dtype=np.int64)
    idxs = np.clip(idxs, ep_start, t)
    return idxs


def _fetch_wrench_window(
    wrench_arr: np.ndarray,
    t: int,
    ep_start: int,
    L: int,
    pad_mode: str = "repeat_first",
) -> np.ndarray:
    end = int(np.clip(t, ep_start, t))
    start = max(ep_start, end - (L - 1))
    seg = np.asarray(wrench_arr[start : end + 1, :6], dtype=np.float32)
    k = int(seg.shape[0])
    out = np.zeros((L, 6), dtype=np.float32)
    out[-k:] = seg
    if k < L:
        pad_n = L - k
        if pad_mode == "repeat_first":
            out[:pad_n] = seg[:1] if k > 0 else 0.0
        elif pad_mode == "zeros":
            pass
        else:
            raise ValueError(f"Unknown wrench_hist_pad_mode={pad_mode}")
    return out


def build_obs_from_replay(
    rb: ReplayBuffer,
    t: int,
    episode_ends: np.ndarray,
    n_obs_steps: int,
    obs_downsample_ratio: int,
    force_hist: int,
    wrench_hist_pad_mode: str,
    device: str,
) -> Dict[str, torch.Tensor]:
    _, ep_start, _ = _episode_bounds(episode_ends, t)
    To = int(n_obs_steps)
    ratio = int(obs_downsample_ratio)

    idxs = _sparse_indices(t, ep_start, To=To, ratio=ratio)

    obs: Dict[str, np.ndarray] = {}

    # RGB: (To,H,W,C)->(To,C,H,W), float[0,1]
    for k in ["left_wrist_img", "external_img"]:
        arr = rb[k]
        x = np.asarray(arr[idxs], dtype=np.uint8)
        x = np.moveaxis(x, -1, 1).astype(np.float32) / 255.0
        obs[k] = x

    # low-dim
    obs["left_robot_tcp_pose"] = np.asarray(rb["left_robot_tcp_pose"][idxs, :9], dtype=np.float32)
    obs["left_robot_gripper_width"] = np.asarray(rb["left_robot_gripper_width"][idxs, :1], dtype=np.float32)
    obs["left_robot_tcp_wrench"] = np.asarray(rb["left_robot_tcp_wrench"][idxs, :6], dtype=np.float32)

    # wrench_hist: (To, L, 6)
    wh = np.stack(
        [
            _fetch_wrench_window(rb["left_robot_tcp_wrench"], int(tt), ep_start, L=force_hist, pad_mode=wrench_hist_pad_mode)
            for tt in idxs
        ],
        axis=0,
    ).astype(np.float32)
    obs["wrench_hist"] = wh

    # to torch + add batch dim
    obs_t = dict_apply(obs, lambda x: torch.from_numpy(x).unsqueeze(0).to(device=device))
    return {"obs": obs_t} if False else obs_t  # policy.predict_action 接受 dict(obs) 形式


# =========================
# 3) 主流程：实机等价 slow_exec & residual label
# =========================
@torch.no_grad()
def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    cfg = build_cfg()

    # ---- load policy ----
    payload = _load_torch(cfg.diff_ckpt_path)
    ws_cls = hydra.utils.get_class(cfg._target_)
    workspace: BaseWorkspace = ws_cls(cfg)
    workspace.load_payload(payload, exclude_keys=None, include_keys=None)
    policy = _pick_policy_from_workspace(cfg, workspace)
    policy.eval().to(DEVICE)
    print("[DBG] policy type:", type(policy))
    for attr in ["num_inference_steps", "policy", "model", "averaged_model", "module"]:
        if hasattr(policy, attr):
            print("[DBG] has", attr, "->", type(getattr(policy, attr)))

    print("[DBG] effective num_inference_steps (before set) =", _get_nsteps(policy))
    _set_num_inference_steps(policy, NUM_INFERENCE_STEPS)
    print("[DBG] effective num_inference_steps (after set)  =", _get_nsteps(policy))



    # ---- load replay buffer (avoid loading huge unused keys) ----
    dataset_path = str(cfg.task.dataset_path)
    zarr_path = os.path.join(dataset_path, "replay_buffer.zarr")
    keys = [
        "left_wrist_img",
        "external_img",
        "left_robot_tcp_pose",
        "left_robot_gripper_width",
        "left_robot_tcp_wrench",
        "action",
    ]
    rb = ReplayBuffer.copy_from_path(zarr_path, keys=keys)
    episode_ends = np.asarray(rb.episode_ends[:], dtype=np.int64)

    # ---- real-runner params ----
    envr = cfg.task.env_runner
    control_fps = int(envr.control_fps)
    inference_fps = int(envr.inference_fps)
    if control_fps % inference_fps != 0:
        raise ValueError(f"control_fps must be divisible by inference_fps, got {control_fps}/{inference_fps}")
    steps_per_inference = control_fps // inference_fps

    tcp_action_update_interval = int(envr.tcp_action_update_interval)
    latency_step = int(envr.latency_step)
    n_obs_steps = int(envr.n_obs_steps)
    obs_downsample_ratio = int(envr.obs_temporal_downsample_ratio)

    force_hist = int(getattr(cfg.policy, "force_hist", 48))
    try:
        wrench_hist_pad_mode = str(cfg.task.dataset.wrench_hist_pad_mode)
    except Exception:
        wrench_hist_pad_mode = "repeat_first"

    tcp_buf_params = dict(envr.tcp_ensemble_buffer_params)
    tcp_buf = EnsembleBuffer(**tcp_buf_params)

    # ---- outputs ----
    T_total = int(rb["action"].shape[0])
    expert_tcp = np.asarray(rb["action"][:, :9], dtype=np.float32)
    slow_exec = np.zeros((T_total, 9), dtype=np.float32)
    fast_label = np.zeros((T_total, 9), dtype=np.float32)

    last_tcp_step_action: Optional[np.ndarray] = None

    mse_acc = 0.0
    mae_acc = 0.0

    t_start = time.time()

    # ---- timing stats ----
    obs_time_sum = 0.0
    infer_time_sum = 0.0
    infer_calls = 0
    from collections import deque
    obs_ms_win = deque(maxlen=20)
    inf_ms_win = deque(maxlen=20)



    for t in range(T_total):
        # ---- inference tick (simulate real) ----
        if (t % steps_per_inference) == 0:
            _t0 = time.perf_counter()
            obs = build_obs_from_replay(
                rb=rb,
                t=t,
                episode_ends=episode_ends,
                n_obs_steps=n_obs_steps,
                obs_downsample_ratio=obs_downsample_ratio,
                force_hist=force_hist,
                wrench_hist_pad_mode=wrench_hist_pad_mode,
                device=DEVICE,
            )


            # ---- build obs timing (CPU)
            obs_dt = time.perf_counter() - _t0
            obs_time_sum += obs_dt
            obs_ms_win.append(1000.0 * obs_dt)

            # ---- inference timing (GPU accurate)
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            _t1 = time.perf_counter()
            pred = policy.predict_action(obs)
            if torch.cuda.is_available():
                torch.cuda.synchronize()
            inf_dt = time.perf_counter() - _t1

            infer_time_sum += inf_dt
            inf_ms_win.append(1000.0 * inf_dt)
            infer_calls += 1



            pred = _get_pred_action_tensor(pred)  # (1,Ta,D)
            action_all = pred.squeeze(0).detach().to("cpu").numpy()  # (Ta,D)

            # ---- only push on update interval ----
            if (t % tcp_action_update_interval) == 0:
                Ta = int(action_all.shape[0])
                tcp_lat = int(np.clip(latency_step, 0, max(Ta - 1, 0)))
                if action_all.shape[-1] != 10:
                    raise RuntimeError(f"Expected action dim 10, got {action_all.shape[-1]}")
                tcp_seq = action_all[tcp_lat:, :9]
                if tcp_seq.shape[0] > 0:
                    tcp_buf.add_action(tcp_seq, t)

        # ---- control tick (simulate real) ----
        tcp_raw = tcp_buf.get_action()
        tcp_step = tcp_raw if tcp_raw is not None else last_tcp_step_action
        if tcp_step is None:
            # 实机早期会等待 buffer，这里为了产出完整长度，兜底用 expert
            tcp_step = expert_tcp[t]
        last_tcp_step_action = tcp_step

        tcp_step = np.asarray(tcp_step, dtype=np.float32).reshape(-1)
        if tcp_step.shape[0] != 9:
            tcp_step = tcp_step[:9]
            if tcp_step.shape[0] < 9:
                tcp_step = np.pad(tcp_step, (0, 9 - tcp_step.shape[0]))

        slow_exec[t] = tcp_step

        # Teacher A: T_delta = T_base^{-1} * T_demo
        delta = absolute_actions_to_relative_actions(
            actions=expert_tcp[t:t+1],   # (1,9) OK
            base_absolute_action=tcp_step # ✅ (9,) 1D
        )
        fast_label[t] = delta.reshape(-1)


        d = tcp_step - expert_tcp[t]
        mse_acc += float(np.mean(d * d))
        mae_acc += float(np.mean(np.abs(d)))

        if (t % 500) == 0:
            dt = time.time() - t_start
            hz = (t + 1) / max(dt, 1e-9)


            avg_obs_ms = 1000.0 * obs_time_sum / max(infer_calls, 1)
            avg_inf_ms = 1000.0 * infer_time_sum / max(infer_calls, 1)
            win_obs_ms = float(np.mean(obs_ms_win)) if len(obs_ms_win) > 0 else float("nan")
            win_inf_ms = float(np.mean(inf_ms_win)) if len(inf_ms_win) > 0 else float("nan")


            print(
                f"[{t:6d}/{T_total}] mse={mse_acc/(t+1):.6f} mae={mae_acc/(t+1):.6f} "
                f"| sim_hz~{hz:.2f} (spi={steps_per_inference} upd={tcp_action_update_interval} lat={latency_step}) "
                f"| avg_obs={avg_obs_ms:.1f}ms avg_infer={avg_inf_ms:.1f}ms "
                f"| win_obs={win_obs_ms:.1f}ms win_infer={win_inf_ms:.1f}ms"

            )


    mse = mse_acc / max(T_total, 1)
    mae = mae_acc / max(T_total, 1)
    print(f"\n[FINAL] T={T_total} | MSE={mse:.6f} | MAE={mae:.6f} (slow_exec vs expert tcp 9d)")

    out_slow = os.path.join(OUT_DIR, "teacherA_slow_exec_9d.npy")
    out_fast = os.path.join(OUT_DIR, "teacherA_fast_label_9d.npy")
    np.save(out_slow, slow_exec)
    np.save(out_fast, fast_label)

    meta = {
        "task": TASK_NAME,
        "config_name": CONFIG_NAME,
        "diff_ckpt_path": DIFF_CKPT_PATH,
        "use_ema": USE_EMA,
        "num_inference_steps": NUM_INFERENCE_STEPS,
        "dataset_path": dataset_path,
        "T": T_total,
        "control_fps": control_fps,
        "inference_fps": inference_fps,
        "steps_per_inference": steps_per_inference,
        "tcp_action_update_interval": tcp_action_update_interval,
        "latency_step": latency_step,
        "n_obs_steps": n_obs_steps,
        "obs_temporal_downsample_ratio": obs_downsample_ratio,
        "force_hist": force_hist,
        "wrench_hist_pad_mode": wrench_hist_pad_mode,
        "mse": float(mse),
        "mae": float(mae),
        "out_slow": out_slow,
        "out_fast": out_fast,
    }
    out_meta = os.path.join(OUT_DIR, "teacherA_label_meta.json")
    with open(out_meta, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"[SAVE] slow_exec -> {out_slow}")
    print(f"[SAVE] fast_label -> {out_fast}")
    print(f"[SAVE] meta -> {out_meta}")


if __name__ == "__main__":
    main()
