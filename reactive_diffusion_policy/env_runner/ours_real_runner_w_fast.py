# ====== ours_real_runner.py (FULL REPLACEMENT) ======
import threading
import time
import os
import os.path as osp
import numpy as np
import torch
import tqdm
from loguru import logger
from typing import Dict, Tuple, Union, Optional
import rclpy
import transforms3d as t3d
import py_cli_interaction
from rclpy.executors import MultiThreadedExecutor
from omegaconf import DictConfig, ListConfig

import sys
from dataclasses import dataclass


from reactive_diffusion_policy.policy.diffusion_unet_image_policy import DiffusionUnetImagePolicy
from reactive_diffusion_policy.common.pytorch_util import dict_apply
from reactive_diffusion_policy.common.precise_sleep import precise_sleep
from reactive_diffusion_policy.env.real_bimanual.ours_real_env import RealRobotEnvironment
from reactive_diffusion_policy.real_world.real_inference_util import get_real_obs_dict
from reactive_diffusion_policy.real_world.real_world_transforms import RealWorldTransforms
from reactive_diffusion_policy.common.space_utils import ortho6d_to_rotation_matrix
from reactive_diffusion_policy.common.ensemble import EnsembleBuffer
from reactive_diffusion_policy.debug_snippets.runner_fusion_debug import should_log, log_fusion_debug, extract_fusion_debug
from reactive_diffusion_policy.common.action_utils import (
    interpolate_actions_with_ratio,
    relative_actions_to_absolute_actions,
    absolute_actions_to_relative_actions,
    get_inter_gripper_actions,
)

import requests
import psutil
from copy import deepcopy
from datetime import datetime
import csv
import json
import logging
import cv2
from collections import deque

# ---- add Fast runtime to import path (repo_root/Fast) ----
_THIS_DIR = osp.dirname(osp.abspath(__file__))  # .../reactive_diffusion_policy/env_runner
_REPO_ROOT = osp.abspath(osp.join(_THIS_DIR, "..", ".."))  # repo root
_FAST_DIR = osp.join(_REPO_ROOT, "Fast")
if _FAST_DIR not in sys.path:
    sys.path.insert(0, _FAST_DIR)

from Fast.fast_deploy_runtime import FastDeployer, FastGateConfig


__all__ = ["RealRunner"]



def _now_timestamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _sanitize(s: str):
    return "".join(c if (c.isalnum() or c in "-_.") else "_" for c in str(s))


def _make_fz_run_dir(output_dir: str, task_name: str, policy_name: str, run_ts: str):
    # e.g. output_dir/fz_logs_<task_name>/<policy_name>/<timestamp>/
    run_dir = osp.join(
        output_dir,
        f"fz_logs_{_sanitize(task_name)}",
        run_ts,
    )
    os.makedirs(run_dir, exist_ok=True)
    return run_dir




# --- thread env settings ---
os.environ["OPENBLAS_NUM_THREADS"] = "12"
os.environ["MKL_NUM_THREADS"] = "12"
os.environ["NUMEXPR_NUM_THREADS"] = "12"
os.environ["OMP_NUM_THREADS"] = "12"

total_cores = psutil.cpu_count()
num_cores_to_bind = 10
cores_to_bind = set(range(min(num_cores_to_bind, total_cores)))

try:
    cv2.setNumThreads(12)
except Exception as e:
    logger.warning(f"[CPU] cv2.setNumThreads failed: {repr(e)}")

try:
    os.sched_setaffinity(0, cores_to_bind)
except Exception as e:
    logger.warning(f"[CPU] sched_setaffinity failed: {repr(e)}")



class RealRunner:
    """
    Real runner with:
      - thread-safe ensemble buffers
      - latency clamp + empty guard
      - PAEP event logging to file
      - optional EMA action smoothing
      - Fz logging (csv+npy+meta)
    """

    def __init__(
        self,
        output_dir: str,
        transform_params: DictConfig,
        env_params: DictConfig,
        shape_meta: DictConfig,
        tcp_ensemble_buffer_params: DictConfig,
        gripper_ensemble_buffer_params: DictConfig,
        latent_tcp_ensemble_buffer_params: DictConfig = None,
        latent_gripper_ensemble_buffer_params: DictConfig = None,
        use_latent_action_with_rnn_decoder: bool = False,
        use_relative_action: bool = False,
        use_relative_tcp_obs_for_relative_action: bool = True,
        action_interpolation_ratio: int = 1,
        eval_episodes=10,
        max_duration_time: float = 30,
        tcp_action_update_interval: int = 6,
        gripper_action_update_interval: int = 10,
        tcp_pos_clip_range: ListConfig = ListConfig([[0.6, -0.4, 0.03], [1.0, 0.45, 0.4]]),
        tcp_rot_clip_range: ListConfig = ListConfig([[-np.pi, 0.0, np.pi], [-np.pi, 0.0, np.pi]]),
        tqdm_interval_sec=5.0,
        control_fps: float = 12,
        inference_fps: float = 6,
        latency_step: int = 0,
        gripper_latency_step: Optional[int] = None,
        n_obs_steps: int = 2,
        obs_temporal_downsample_ratio: int = 2,
        dataset_obs_temporal_downsample_ratio: int = 1,
        downsample_extended_obs: bool = True,
        enable_video_recording: bool = False,
        vcamera_server_ip: Optional[Union[str, ListConfig]] = None,
        vcamera_server_port: Optional[Union[int, ListConfig]] = None,
        # ---- debug ----
        debug_sine: bool = False,
        debug_sine_amp: float = 0.02,
        debug_sine_freq: float = 0.2,
        debug_sine_axis: str = "x",
        debug_sine_affect_right: bool = False,
        debug_freeze_gripper: bool = False,
        debug_gripper_value: float = 0.0,
        task_name=None,
        # ---- smoothing ----
        action_smoothing_alpha: float = 0.0,   # 0 => off, e.g. 0.15 for on
        smooth_xyz_only: bool = True,
        # ---- dense wrench buffer (24Hz) ----
        enable_dense_wrench_hist: bool = True,
        force_hist: int = 48,

        # ---- fast residual (optional) ----
        enable_fast: bool = False,
        fast_ckpt_path: str = "",
        fast_meta_path: str = "",
        fast_device: str = "cuda",
        fast_hist_len: Optional[int] = None,
        fast_wrench_clip_norm: float = 6.0,
        fast_max_delta_xyz: float = 0.003,
        fast_apply_rot: bool = False,
        fast_gate_mode: str = "soft",          # "soft" or "hard"
        fast_gate_thr: float = 0.5,
        fast_gate_use_contact: bool = True,
        fast_active_phase_ids: Optional[ListConfig] = None,  # e.g. [1] for wiping progress
        fast_debug_log_every: int = 0,

    ):
        self.task_name = task_name
        self.transforms = RealWorldTransforms(option=transform_params)
        self.shape_meta = dict(shape_meta)
        self.eval_episodes = eval_episodes

        self._phase_lock = threading.Lock()
        self._latest_phase_id = None

        # ---- PAEP polling thread (control_fps) ----
        self._paep_stop = threading.Event()
        self._paep_thread = None
        self._paep_cache_lock = threading.Lock()

        # cache (latest step, for fast & diffusion tiling)
        self._fast_paep_p_contact = None  # scalar float
        self._fast_paep_p_phase = None    # np.ndarray (E,)
        
        self._policy_infer_lock = threading.Lock()



        # --- keys ---
        self.rgb_keys = []
        self.lowdim_keys = []
        obs_shape_meta = shape_meta["obs"]
        for key, attr in obs_shape_meta.items():
            typ = attr.get("type", "low_dim")
            if typ == "rgb":
                self.rgb_keys.append(key)
            elif typ == "low_dim":
                self.lowdim_keys.append(key)

        self.extended_rgb_keys = []
        self.extended_lowdim_keys = []
        extended_obs_shape_meta = shape_meta.get("extended_obs", dict())
        for key, attr in extended_obs_shape_meta.items():
            typ = attr.get("type", "low_dim")
            if typ == "rgb":
                self.extended_rgb_keys.append(key)
            elif typ == "low_dim":
                self.extended_lowdim_keys.append(key)

        self.output_dir = output_dir
        self.policy_name = task_name if task_name is not None else "ours"


        self._fz_run_ts = None
        self._fz_run_dir = None


        # ---- debug buffer (no realtime IO, dump once per episode) ----
        self._dbg_buf = []   # list[dict], cleared every episode

        def _dbg(tag, **kw):
            rec = {
                "t": time.time(),
                "tag": str(tag),
                **kw,
            }
            self._dbg_buf.append(rec)

        self._dbg = _dbg


        # --- rclpy/env ---
        rclpy.init(args=None)
        self.env = RealRobotEnvironment(transforms=self.transforms, **env_params)
        time.sleep(2)

        self.max_duration_time = max_duration_time
        self.tcp_action_update_interval = int(tcp_action_update_interval)
        self.gripper_action_update_interval = int(gripper_action_update_interval)
        self.tcp_pos_clip_range = tcp_pos_clip_range
        self.tcp_rot_clip_range = tcp_rot_clip_range
        self.tqdm_interval_sec = tqdm_interval_sec

        # --- fps (IMPORTANT: use int-safe math) ---
        cf, inf = int(control_fps), int(inference_fps)
        if cf <= 0 or inf <= 0:
            raise ValueError(f"control_fps/inference_fps must be positive, got {cf}/{inf}")
        if cf % inf != 0:
            raise ValueError(f"control_fps must be divisible by inference_fps, got {cf}/{inf}")
        self.control_fps = cf
        self.inference_fps = inf
        self.control_interval_time = 1.0 / float(cf)
        self.inference_interval_time = 1.0 / float(inf)
        self.steps_per_inference = cf // inf

        self.latency_step = int(latency_step)
        self.gripper_latency_step = int(gripper_latency_step) if gripper_latency_step is not None else int(latency_step)

        self.n_obs_steps = n_obs_steps
        self.obs_temporal_downsample_ratio = obs_temporal_downsample_ratio
        self.dataset_obs_temporal_downsample_ratio = dataset_obs_temporal_downsample_ratio
        self.downsample_extended_obs = downsample_extended_obs

        # --- buffer init ---
        self.use_latent_action_with_rnn_decoder = use_latent_action_with_rnn_decoder
        if self.use_latent_action_with_rnn_decoder:
            assert latent_tcp_ensemble_buffer_params.ensemble_mode == "new"
            assert latent_gripper_ensemble_buffer_params.ensemble_mode == "new"
            self.tcp_ensemble_buffer = EnsembleBuffer(**latent_tcp_ensemble_buffer_params)
            self.gripper_ensemble_buffer = EnsembleBuffer(**latent_gripper_ensemble_buffer_params)
        else:
            self.tcp_ensemble_buffer = EnsembleBuffer(**tcp_ensemble_buffer_params)
            self.gripper_ensemble_buffer = EnsembleBuffer(**gripper_ensemble_buffer_params)

        self.use_relative_action = use_relative_action
        self.use_relative_tcp_obs_for_relative_action = use_relative_tcp_obs_for_relative_action

        self.action_interpolation_ratio = int(action_interpolation_ratio)
        if self.action_interpolation_ratio < 1:
            self.action_interpolation_ratio = 1

        # --- video recording ---
        self.enable_video_recording = enable_video_recording
        if enable_video_recording:
            assert (isinstance(vcamera_server_ip, str) and isinstance(vcamera_server_port, int)) or (
                isinstance(vcamera_server_ip, ListConfig) and isinstance(vcamera_server_port, ListConfig)
            ), "vcamera_server_ip and vcamera_server_port should be a string or ListConfig."
        if isinstance(vcamera_server_ip, str):
            self.vcamera_server_ip_list = [vcamera_server_ip]
            self.vcamera_server_port_list = [vcamera_server_port]
        elif isinstance(vcamera_server_ip, ListConfig):
            self.vcamera_server_ip_list = list(vcamera_server_ip)
            self.vcamera_server_port_list = list(vcamera_server_port)
        else:
            self.vcamera_server_ip_list = []
            self.vcamera_server_port_list = []
        self.video_dir = osp.join(output_dir, "videos")

        # --- debug ---
        self.debug_sine = bool(debug_sine)
        self.debug_sine_amp = float(debug_sine_amp)
        self.debug_sine_freq = float(debug_sine_freq)
        self.debug_sine_axis = str(debug_sine_axis).lower()
        self.debug_sine_affect_right = bool(debug_sine_affect_right)
        self.debug_freeze_gripper = bool(debug_freeze_gripper)
        self.debug_gripper_value = float(debug_gripper_value)

        # --- smoothing ---
        self.action_smoothing_alpha = float(action_smoothing_alpha)
        self.smooth_xyz_only = bool(smooth_xyz_only)
        self._last_step_action = None

        # --- last-action fallback (per step) ---
        self._last_gripper_step_action = None
        self._last_tcp_step_action = None

        # --- thread control ---
        self.stop_event = threading.Event()
        self._none_streak = 0
        self.action_step_count = 0
        self.session = requests.Session()

        # --- locks ---
        self.tcp_buf_lock = threading.Lock()
        self.grip_buf_lock = threading.Lock()

        # --- dense wrench buffer (24Hz) ---
        self.enable_dense_wrench_hist = bool(enable_dense_wrench_hist)
        self.debug_zero_wrench_hist = False
        self.force_hist = int(force_hist)
        self._wrench_dense_buf = deque(maxlen=self.force_hist + 64)  # store (6,)
        self._wrench_stop = threading.Event()
        self._wrench_thread = None
        # --- dense Fz log (control_fps) ---
        self._fz_dense = []          # list[float]
        self._fz_dense_t = []        # list[float] wall_time
        self._fz_dense_step = []     # list[int]   control-step index within episode
        self._fz_dense_ctr = 0


        # ---- fast runtime ----
        self.enable_fast = bool(enable_fast)
        self.fast_debug_log_every = int(fast_debug_log_every)
        self._fast = None
        self._fast_last_dbg = None


        if self.enable_fast:
            if (not fast_ckpt_path) or (not fast_meta_path):
                raise ValueError("[FAST] enable_fast=True but fast_ckpt_path/fast_meta_path is empty")

            phase_ids = None
            if fast_active_phase_ids is not None:
                # ListConfig -> python list
                phase_ids = [int(x) for x in list(fast_active_phase_ids)]

            gate = FastGateConfig(
                mode=str(fast_gate_mode),
                thr=float(fast_gate_thr),
                use_contact=bool(fast_gate_use_contact),
                active_phase_ids=phase_ids,
            )

            self._fast = FastDeployer(
                ckpt_path=str(fast_ckpt_path),
                meta_path=str(fast_meta_path),
                device=str(fast_device),
                hist_len=None if fast_hist_len is None else int(fast_hist_len),
                wrench_clip_norm=float(fast_wrench_clip_norm),
                max_delta_xyz=float(fast_max_delta_xyz),
                gate=gate,
                apply_rot=bool(fast_apply_rot),
                verbose=True,
            )
            logger.info("[FAST] enabled ✅")

    def _warn_throttle(self, key: str, msg: str, every: int = 60):
        # print at most once per `every` steps (per key)
        if not hasattr(self, "_warn_ctr"):
            self._warn_ctr = {}
        c = int(self._warn_ctr.get(key, 0)) + 1
        self._warn_ctr[key] = c
        if (c % every) == 0:
            logger.warning(msg)
        # always store in dbg buffer
        try:
            self._dbg("warn", key=str(key), msg=str(msg), ctr=int(c))
        except Exception:
            pass



    def _setup_paep_logger(self, output_dir: str):
        os.makedirs(output_dir, exist_ok=True)
        run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        log_path = os.path.join(output_dir, f"paep_events_{run_id}.log")

        log = logging.getLogger(f"paep_event_logger_{run_id}")
        log.setLevel(logging.INFO)
        log.propagate = False

        fh = logging.FileHandler(log_path, mode="w")
        fh.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
        log.addHandler(fh)

        logger.info(f"[PAEP] logging to: {log_path}")
        return log

    @staticmethod
    def spin_executor(executor):
        executor.spin()

    def pre_process_obs(self, obs_dict: Dict) -> Tuple[Dict, Dict]:
        obs_dict = deepcopy(obs_dict)

        for key in self.lowdim_keys:
            if "wrt" not in key:
                obs_dict[key] = obs_dict[key][:, : self.shape_meta["obs"][key]["shape"][0]]

        obs_dict.update(get_inter_gripper_actions(obs_dict, self.lowdim_keys, self.transforms))
        for key in self.lowdim_keys:
            obs_dict[key] = obs_dict[key][:, : self.shape_meta["obs"][key]["shape"][0]]

        absolute_obs_dict = {}
        for key in self.lowdim_keys:
            absolute_obs_dict[key] = obs_dict[key].copy()

        if self.use_relative_action and self.use_relative_tcp_obs_for_relative_action:
            for key in self.lowdim_keys:
                if "robot_tcp_pose" in key and "wrt" not in key:
                    base_absolute_action = obs_dict[key][-1].copy()
                    obs_dict[key] = absolute_actions_to_relative_actions(
                        obs_dict[key], base_absolute_action=base_absolute_action
                    )
        return obs_dict, absolute_obs_dict

    def pre_process_extended_obs(self, extended_obs_dict: Dict) -> Tuple[Dict, Dict]:
        extended_obs_dict = deepcopy(extended_obs_dict)

        absolute_extended_obs_dict = {}
        for key in self.extended_lowdim_keys:
            extended_obs_dict[key] = extended_obs_dict[key][:, : self.shape_meta["extended_obs"][key]["shape"][0]]
            absolute_extended_obs_dict[key] = extended_obs_dict[key].copy()

        if self.use_relative_action and self.use_relative_tcp_obs_for_relative_action:
            for key in self.extended_lowdim_keys:
                if "robot_tcp_pose" in key and "wrt" not in key:
                    base_absolute_action = extended_obs_dict[key][-1].copy()
                    extended_obs_dict[key] = absolute_actions_to_relative_actions(
                        extended_obs_dict[key], base_absolute_action=base_absolute_action
                    )

        return extended_obs_dict, absolute_extended_obs_dict

    def post_process_action(self, action: np.ndarray) -> Tuple[np.ndarray, bool]:
        assert len(action.shape) == 2

        if not self.env.data_processing_manager.use_6d_rotation:
            raise NotImplementedError("Only 6D rotation supported in this runner.")

        if action.shape[-1] in (4, 8):
            left_trans_batch = action[:, :3]
            left_euler_batch = np.zeros_like(left_trans_batch)
            left_action_6d = np.concatenate([left_trans_batch, left_euler_batch], axis=1)
            if action.shape[-1] == 8:
                right_trans_batch = action[:, 3:6]
                right_euler_batch = np.zeros_like(right_trans_batch)
                right_action_6d = np.concatenate([right_trans_batch, right_euler_batch], axis=1)
            else:
                right_action_6d = None
        elif action.shape[-1] in (10, 20):
            left_rot_mat_batch = ortho6d_to_rotation_matrix(action[:, 3:9])
            left_euler_batch = np.array([t3d.euler.mat2euler(rot_mat) for rot_mat in left_rot_mat_batch])
            left_trans_batch = action[:, :3]
            left_action_6d = np.concatenate([left_trans_batch, left_euler_batch], axis=1)
            if action.shape[-1] == 20:
                right_rot_mat_batch = ortho6d_to_rotation_matrix(action[:, 12:18])
                right_euler_batch = np.array([t3d.euler.mat2euler(rot_mat) for rot_mat in right_rot_mat_batch])
                right_trans_batch = action[:, 9:12]
                right_action_6d = np.concatenate([right_trans_batch, right_euler_batch], axis=1)
            else:
                right_action_6d = None
        else:
            raise NotImplementedError(f"Unsupported action dim: {action.shape[-1]}")

        left_action_6d[:, :3] = np.clip(
            left_action_6d[:, :3],
            np.array(self.tcp_pos_clip_range[0]),
            np.array(self.tcp_pos_clip_range[1]),
        )
        left_action_6d[:, 3:] = np.clip(
            left_action_6d[:, 3:],
            np.array(self.tcp_rot_clip_range[0]),
            np.array(self.tcp_rot_clip_range[1]),
        )

        if right_action_6d is not None:
            right_action_6d[:, :3] = np.clip(
                right_action_6d[:, :3],
                np.array(self.tcp_pos_clip_range[2]),
                np.array(self.tcp_pos_clip_range[3]),
            )
            right_action_6d[:, 3:] = np.clip(
                right_action_6d[:, 3:],
                np.array(self.tcp_rot_clip_range[2]),
                np.array(self.tcp_rot_clip_range[3]),
            )

        if action.shape[-1] == 4:
            left_action = np.concatenate([left_action_6d, action[:, 3][:, None], np.zeros((action.shape[0], 1))], axis=1)
            right_action = None
        elif action.shape[-1] == 8:
            left_action = np.concatenate([left_action_6d, action[:, 6][:, None], np.zeros((action.shape[0], 1))], axis=1)
            right_action = np.concatenate([right_action_6d, action[:, 7][:, None], np.zeros((action.shape[0], 1))], axis=1)
        elif action.shape[-1] == 10:
            left_action = np.concatenate([left_action_6d, action[:, 9][:, None], np.zeros((action.shape[0], 1))], axis=1)
            right_action = None
        elif action.shape[-1] == 20:
            left_action = np.concatenate([left_action_6d, action[:, 18][:, None], np.zeros((action.shape[0], 1))], axis=1)
            right_action = np.concatenate([right_action_6d, action[:, 19][:, None], np.zeros((action.shape[0], 1))], axis=1)
        else:
            raise NotImplementedError

        if right_action is None:
            right_action = left_action.copy()
            is_bimanual = False
        else:
            is_bimanual = True

        action_all = np.concatenate([left_action, right_action], axis=-1)
        return action_all, is_bimanual

    def action_command_thread(self, policy: Union[DiffusionUnetImagePolicy], stop_event):
        while not stop_event.is_set():
            start_time = time.time()

            with self.tcp_buf_lock:
                tcp_raw = self.tcp_ensemble_buffer.get_action()
            with self.grip_buf_lock:
                grip_raw = self.gripper_ensemble_buffer.get_action()

            tcp_step_action = tcp_raw if tcp_raw is not None else self._last_tcp_step_action
            gripper_step_action = grip_raw if grip_raw is not None else self._last_gripper_step_action

            if tcp_step_action is None or gripper_step_action is None:
                self._none_streak += 1
                if self._none_streak % 50 == 0:
                    logger.warning(
                        f"[ACTION_THREAD] get_action() None streak={self._none_streak} "
                        f"tcp_raw_none={tcp_raw is None} grip_raw_none={grip_raw is None} "
                        f"action_step_count={getattr(self,'action_step_count',-1)}"
                    )
                cur_time = time.time()
                precise_sleep(max(0.0, self.control_interval_time - (cur_time - start_time)))
                self.action_step_count += 1
                continue

            self._none_streak = 0
            self._last_tcp_step_action = tcp_step_action
            self._last_gripper_step_action = gripper_step_action

            # if self.use_latent_action_with_rnn_decoder:
            #     tcp_extended_obs_step = int(tcp_step_action[-1])
            #     gripper_extended_obs_step = int(gripper_step_action[-1])
            #     tcp_step_action = tcp_step_action[:-1]
            #     gripper_step_action = gripper_step_action[:-1]

            #     longer_extended_obs_step = max(tcp_extended_obs_step, gripper_extended_obs_step)
            #     obs_temporal_downsample_ratio = self.obs_temporal_downsample_ratio if self.downsample_extended_obs else 1
            #     extended_obs = self.env.get_obs(longer_extended_obs_step, temporal_downsample_ratio=obs_temporal_downsample_ratio)

            #     if self.use_relative_action:
            #         action_dim = self.shape_meta["obs"]["left_robot_tcp_pose"]["shape"][0]
            #         if "right_robot_tcp_pose" in self.shape_meta["obs"]:
            #             action_dim += self.shape_meta["obs"]["right_robot_tcp_pose"]["shape"][0]
            #         tcp_base_absolute_action = tcp_step_action[-action_dim:]
            #         gripper_base_absolute_action = gripper_step_action[-action_dim:]
            #         tcp_step_action = tcp_step_action[:-action_dim]
            #         gripper_step_action = gripper_step_action[:-action_dim]

            #     np_extended_obs_dict = dict(extended_obs)
            #     np_extended_obs_dict = get_real_obs_dict(env_obs=np_extended_obs_dict, shape_meta=self.shape_meta, is_extended_obs=True, bgr_to_rgb=True)
            #     np_extended_obs_dict, _ = self.pre_process_extended_obs(np_extended_obs_dict)
            #     extended_obs_dict = dict_apply(np_extended_obs_dict, lambda x: torch.from_numpy(x).unsqueeze(0))

            #     tcp_step_latent_action = torch.from_numpy(tcp_step_action.astype(np.float32)).unsqueeze(0)
            #     gripper_step_latent_action = torch.from_numpy(gripper_step_action.astype(np.float32)).unsqueeze(0)

            #     dor = self.dataset_obs_temporal_downsample_ratio
            #     tcp_step_action = policy.predict_from_latent_action(tcp_step_latent_action, extended_obs_dict, tcp_extended_obs_step, dor)["action"][0].detach().cpu().numpy()
            #     gripper_step_action = policy.predict_from_latent_action(gripper_step_latent_action, extended_obs_dict, gripper_extended_obs_step, dor)["action"][0].detach().cpu().numpy()

            #     if self.use_relative_action:
            #         tcp_step_action = relative_actions_to_absolute_actions(tcp_step_action, tcp_base_absolute_action)
            #         gripper_step_action = relative_actions_to_absolute_actions(gripper_step_action, gripper_base_absolute_action)

            #     if tcp_step_action.shape[-1] == 4:
            #         tcp_len = 3
            #     elif tcp_step_action.shape[-1] == 8:
            #         tcp_len = 6
            #     elif tcp_step_action.shape[-1] == 10:
            #         tcp_len = 9
            #     elif tcp_step_action.shape[-1] == 20:
            #         tcp_len = 18
            #     else:
            #         raise NotImplementedError

            #     tcp_step_action = tcp_step_action[-1][:tcp_len]
            #     gripper_step_action = gripper_step_action[-1][tcp_len:]

            # ---- FAST residual injection (abs9d in, abs9d out) ----
            if self.enable_fast and (self._fast is not None):
                try:
                    # your tcp_step_action for left arm is usually (9,) or (3,) etc.
                    if (tcp_step_action is not None) and (tcp_step_action.shape[0] >= 9):
                        # latest wrench (6,)
                        if (self.enable_dense_wrench_hist) and (len(self._wrench_dense_buf) > 0):
                            wrench6 = np.asarray(self._wrench_dense_buf[-1], dtype=np.float32).reshape(-1)[:6]
                        else:
                            wrench6 = np.zeros((6,), dtype=np.float32)

                        base_abs9d = tcp_step_action[:9].astype(np.float32)

                        # ✅ copy PAEP cache under lock (avoid tearing / race)
                        with self._paep_cache_lock:
                            pc = self._fast_paep_p_contact
                            pp = self._fast_paep_p_phase

                        # guard: PAEP cache not ready -> skip fast this step
                        if (pc is None) or (pp is None):
                            raise RuntimeError(f"[FAST] skip: paep cache not ready (pc={pc}, pp_is_none={pp is None})")

                        cmd_abs9d, dbg = self._fast.step(
                            base_abs9d=base_abs9d,
                            wrench6=wrench6,
                            p_contact=float(pc),
                            p_phase=np.asarray(pp, dtype=np.float32).reshape(-1),
                        )


                        tcp_step_action = tcp_step_action.copy()
                        tcp_step_action[:9] = cmd_abs9d
                        self._fast_last_dbg = dbg
                        

                        # if self.fast_debug_log_every > 0 and (self.action_step_count % self.fast_debug_log_every) == 0:
                        #     try:
                        #         w_raw = np.asarray(dbg.get("wrench_raw", wrench6), dtype=np.float32).reshape(-1)[:6]
                        #         fz = float(w_raw[2])

                        #         dz_base = float(np.asarray(dbg.get("delta_p_base", [np.nan, np.nan, np.nan]), dtype=np.float32).reshape(-1)[2])
                        #         dz_ee   = float(np.asarray(dbg.get("delta_p_ee",   [np.nan, np.nan, np.nan]), dtype=np.float32).reshape(-1)[2])

                        #         alpha    = dbg.get("alpha", None)
                        #         pc_dbg   = dbg.get("pc", None)
                        #         phase_id = dbg.get("phase_id", None)

                        #         fz_target   = float(dbg.get("fz_target", -20.0))
                        #         fz_deadband = float(dbg.get("fz_deadband", 0.5))
                        #         expected    = dbg.get("fast_rule_expected", "NA")
                        #         ok          = bool(dbg.get("fast_rule_ok", True))

                        #         logger.info(
                        #             f"[FAST] step={self.action_step_count} fz={fz:+.2f} tgt={fz_target:+.1f} db={fz_deadband:.1f} "
                        #             f"dz_base_z={dz_base:+.6f} dz_ee_z={dz_ee:+.6f} "
                        #             f"alpha={alpha} pc={pc_dbg} phase={phase_id} expected={expected} ok={ok}"
                        #         )
                        #     except Exception as _e:
                        #         logger.warning(f"[FAST_DBG] failed: {repr(_e)}")



                except Exception as e:
                    logger.warning(f"[FAST] injection failed: {repr(e)}")

            combined_action = np.concatenate([tcp_step_action, gripper_step_action], axis=-1)
            step_action, is_bimanual = self.post_process_action(combined_action[np.newaxis, :])

            step_action = step_action.squeeze(0)

            # ---- EMA smoothing (optional) ----
            a = self.action_smoothing_alpha
            if a > 0.0:
                if self._last_step_action is None:
                    self._last_step_action = step_action.copy()
                else:
                    sm = step_action.copy()

                    # 平滑 left/right 的 tcp 6d（0:6 和 8:14），不动 gripper 宽度等剩余维度
                    sm[0:6]   = (1 - a) * self._last_step_action[0:6]   + a * step_action[0:6]
                    sm[8:14]  = (1 - a) * self._last_step_action[8:14]  + a * step_action[8:14]

                    step_action = sm
                    self._last_step_action = step_action.copy()


        
            with self._phase_lock:
                phase_id = self._latest_phase_id

            # ---- log Fz + PAEP strictly at control_fps (aligned to execute_action) ----
            self._log_ctrl_fz_and_paep(control_step=int(self.action_step_count))


            self.env.execute_action(
                step_action,
                use_relative_action=False,
                is_bimanual=is_bimanual,
                phase_id=phase_id,
            )


            cur_time = time.time()
            precise_sleep(max(0.0, self.control_interval_time - (cur_time - start_time)))
            self.action_step_count += 1

    def start_record_video(self, video_path):
        for vcamera_server_ip, vcamera_server_port in zip(self.vcamera_server_ip_list, self.vcamera_server_port_list):
            response = self.session.post(f"http://{vcamera_server_ip}:{vcamera_server_port}/start_recording/{video_path}")
            if response.status_code == 200:
                logger.info(f"Start recording video to {video_path}")
            else:
                logger.error(f"Failed to start recording video to {video_path}")

    def stop_record_video(self):
        for vcamera_server_ip, vcamera_server_port in zip(self.vcamera_server_ip_list, self.vcamera_server_port_list):
            response = self.session.post(f"http://{vcamera_server_ip}:{vcamera_server_port}/stop_recording")
            if response.status_code == 200:
                logger.info("Stop recording video")
            else:
                logger.error("Failed to stop recording video")

    # ---------------------------------------------------------------------
    # Dense wrench polling (24Hz) -> build dataset-aligned wrench_hist
    # ---------------------------------------------------------------------

    def _infer_paep_only(self, policy, obs_dict):
        """
        Try best-effort to run PAEP without running diffusion.
        Returns: (phase_id, p_contact, p_phase_vec, phase_conf, g_contact)
        """
        # 1) preferred: explicit API
        if hasattr(policy, "predict_paep"):
            out = policy.predict_paep(obs_dict)
            # expect torch tensors
            phase_id = int(out.get("paep_phase_id").detach().cpu().numpy().reshape(-1)[0])
            p_contact = float(out.get("paep_p_contact").detach().cpu().numpy().reshape(-1)[0])
            p_phase = out.get("paep_p_phase").detach().cpu().numpy().reshape(-1)
            phase_conf = float(out.get("paep_phase_conf", torch.tensor(float("nan"))).detach().cpu().numpy().reshape(-1)[0])
            g_contact = float(out.get("paep_g_contact", torch.tensor(float("nan"))).detach().cpu().numpy().reshape(-1)[0])
            return phase_id, p_contact, p_phase, phase_conf, g_contact

        # 2) fallback: some policies expose a paep module
        for attr in ["paep", "paep_net", "paep_model", "_paep", "paep_future_net"]:
            if hasattr(policy, attr):
                m = getattr(policy, attr)
                if callable(m):
                    out = m(obs_dict)
                else:
                    # module with forward
                    out = m(obs_dict)

                # tolerate dict outputs
                if isinstance(out, dict):
                    if "paep_phase_id" in out and "paep_p_contact" in out and "paep_p_phase" in out:
                        phase_id = int(out["paep_phase_id"].detach().cpu().numpy().reshape(-1)[0])
                        p_contact = float(out["paep_p_contact"].detach().cpu().numpy().reshape(-1)[0])
                        p_phase = out["paep_p_phase"].detach().cpu().numpy().reshape(-1)
                        phase_conf = float(out.get("paep_phase_conf", torch.tensor(float("nan"))).detach().cpu().numpy().reshape(-1)[0])
                        g_contact = float(out.get("paep_g_contact", torch.tensor(float("nan"))).detach().cpu().numpy().reshape(-1)[0])
                        return phase_id, p_contact, p_phase, phase_conf, g_contact
        if not hasattr(self, "_warn_paep_api_once"):
            self._warn_paep_api_once = True
            logger.warning("[PAEP_CTRL] policy has no predict_paep() and no paep module hook; polling will not update cache.")


        # 3) last resort: cannot infer PAEP only
        return None, None, None, None, None


    def _paep_poll_thread(self, policy, device):
        """
        Run PAEP at control_fps, update shared cache for BOTH diffusion and fast.
        """
        dt = self.control_interval_time
        self._paep_stop.clear()

        step_idx = 0
        while not self._paep_stop.is_set():
            t0 = time.time()
            try:
                obs = self.env.get_obs(obs_steps=self.n_obs_steps, temporal_downsample_ratio=self.obs_temporal_downsample_ratio)
                if len(obs) > 0:
                    np_obs_dict = dict(obs)
                    np_obs_dict = get_real_obs_dict(env_obs=np_obs_dict, shape_meta=self.shape_meta, bgr_to_rgb=True)
                    np_obs_dict, _ = self.pre_process_obs(np_obs_dict)
                    obs_dict = dict_apply(np_obs_dict, lambda x: torch.from_numpy(x).unsqueeze(0).to(device=device))

                    # keep wrench_hist consistent (same as diffusion loop)
                    if self.enable_dense_wrench_hist:
                        try:
                            To = int(obs_dict["left_robot_tcp_wrench"].shape[1]) if "left_robot_tcp_wrench" in obs_dict else int(self.n_obs_steps)
                            wh = self._build_wrench_hist_from_dense(To=To)
                            if wh is not None:
                                wh_t = torch.from_numpy(wh).unsqueeze(0).to(device=device)
                                obs_dict["wrench_hist"] = wh_t
                        except Exception:
                            pass

                    with torch.no_grad():
                        with self._policy_infer_lock:
                            phase_id, pc, p_phase, conf, g = self._infer_paep_only(policy, obs_dict)


                    if pc is not None and p_phase is not None:
                        with self._paep_cache_lock:
                            self._fast_paep_p_contact = float(pc)
                            self._fast_paep_p_phase = np.asarray(p_phase, dtype=np.float32).reshape(-1)
                        with self._phase_lock:
                            self._latest_phase_id = phase_id


            except Exception as e:
                logger.warning(f"[PAEP_CTRL] poll failed: {repr(e)}")

            step_idx += 1
            precise_sleep(max(0.0, dt - (time.time() - t0)))


    def _start_paep_polling(self, policy, device):
        if self._paep_thread is not None:
            return
        self._paep_stop.clear()
        self._paep_thread = threading.Thread(target=self._paep_poll_thread, args=(policy, device), daemon=True)
        self._paep_thread.start()


    def _stop_paep_polling(self):
        if self._paep_thread is None:
            return
        self._paep_stop.set()
        try:
            self._paep_thread.join(timeout=1.0)
        except Exception:
            pass
        self._paep_thread = None

    def _wrench_poll_thread(self):
        """Poll env.latest_left_robot_tcp_wrench at control_fps and push into deque."""
        dt = self.control_interval_time  # 1/control_fps
        self._wrench_stop.clear()
        last = None
        while not self._wrench_stop.is_set():
            t0 = time.time()
            # fast-path: read cached wrench updated in RealRobotEnvironment.callback()
            try:
                with self.env.mutex:
                    w = getattr(self.env, "latest_left_robot_tcp_wrench", None)
            except Exception:
                w = None

            if w is None:
                w = last
            else:
                last = w

            if w is not None:
                ww = np.asarray(w, dtype=np.float32).reshape(-1)[:6]
                self._wrench_dense_buf.append(ww)


            precise_sleep(max(0.0, dt - (time.time() - t0)))

    def _start_dense_wrench_polling(self):
        if not self.enable_dense_wrench_hist:
            return
        self._wrench_dense_buf.clear()
        self._wrench_stop.clear()
        self._wrench_thread = threading.Thread(target=self._wrench_poll_thread, daemon=True)
        self._wrench_thread.start()

    def _stop_dense_wrench_polling(self):
        if not self.enable_dense_wrench_hist:
            return
        if self._wrench_thread is None:
            return
        self._wrench_stop.set()
        try:
            self._wrench_thread.join(timeout=1.0)
        except Exception:
            pass
        self._wrench_thread = None

    def _build_wrench_hist_from_dense(self, To: int):
        """
        Always build (To, L, 6). If dense buffer is short, pad on the left with the earliest sample.
        Per-frame end alignment:
        end_offset = (To-1-i) * obs_ratio
        """
        L = int(self.force_hist)
        obs_ratio = int(self.obs_temporal_downsample_ratio)

        buf = list(self._wrench_dense_buf)
        if len(buf) == 0:
            return None  # 真没拿到任何 wrench，就只能缺一次（通常不会发生）

        need = L + (To - 1) * obs_ratio + 1
        if len(buf) < need:
            pad = [buf[0]] * (need - len(buf))
            buf = pad + buf

        N = len(buf)
        out = np.zeros((To, L, 6), dtype=np.float32)
        for i in range(To):
            end_offset = (To - 1 - i) * obs_ratio
            end = (N - 1) - end_offset
            start = end - (L - 1)
            out[i] = np.stack(buf[start:end + 1], axis=0)
        return out
    
    def _inject_paep_cache_for_diffusion(self, obs_dict: Dict, device):
        """
        Inject cached PAEP (control-fps) into obs_dict for diffusion (inference-fps).
        Adds:
        - obs_dict["paep_p_phase"]   shape (1, To, E)
        - obs_dict["paep_p_contact"] shape (1, To, 1)

        NOTE:
        - PAEP outputs are probabilities => MUST be floating dtype.
        - NEVER match external_img dtype (often uint8), otherwise probs get cast to uint8 and destroyed.
        """
        try:
            # infer To from stable keys
            if "external_img" in obs_dict:
                To = int(obs_dict["external_img"].shape[1])
            elif "left_robot_tcp_wrench" in obs_dict:
                To = int(obs_dict["left_robot_tcp_wrench"].shape[1])
            else:
                _k0 = next(iter(obs_dict.keys()))
                To = int(obs_dict[_k0].shape[1])

            with self._paep_cache_lock:
                pc_raw = self._fast_paep_p_contact
                pp = None if (self._fast_paep_p_phase is None) else np.asarray(self._fast_paep_p_phase, dtype=np.float32).reshape(-1)

            if (pc_raw is None) or (pp is None):
                raise RuntimeError("paep cache not ready")

            pc = float(pc_raw)
            if not np.isfinite(pc):
                raise RuntimeError(f"paep pc not finite: {pc}")

            E = int(pp.shape[0])

            paep_phase_seq = np.broadcast_to(pp[None, None, :], (1, To, E)).astype(np.float32)  # (1,To,E)
            paep_contact_seq = np.full((1, To, 1), pc, dtype=np.float32)                         # (1,To,1)

            # ---- choose FLOAT dtype for PAEP tensors ----
            # prefer matching other float modalities (e.g., wrench) to avoid dtype mismatch inside model
            out_dtype = None
            if "left_robot_tcp_wrench" in obs_dict and torch.is_tensor(obs_dict["left_robot_tcp_wrench"]):
                out_dtype = obs_dict["left_robot_tcp_wrench"].dtype

            # enforce floating
            if (out_dtype is None) or (not torch.is_floating_point(torch.empty((), dtype=out_dtype))):
                out_dtype = torch.float32

            obs_dict["paep_p_phase"] = torch.from_numpy(paep_phase_seq).to(device=device, dtype=out_dtype)
            obs_dict["paep_p_contact"] = torch.from_numpy(paep_contact_seq).to(device=device, dtype=out_dtype)

        except Exception as e:
            logger.warning(f"[PAEP_INJECT] skipped: {repr(e)}")


    def _inject_wrench_hist_for_diffusion(self, obs_dict: Dict, device):
        """Inject wrench_hist (B,To,L,6) built from dense buffer into obs_dict."""
        if not self.enable_dense_wrench_hist:
            return
        try:
            To = int(obs_dict["left_robot_tcp_wrench"].shape[1]) if "left_robot_tcp_wrench" in obs_dict else int(self.n_obs_steps)
            wh = self._build_wrench_hist_from_dense(To=To)
            if wh is None:
                return
            wh_t = torch.from_numpy(wh).unsqueeze(0).to(device=device)
            if getattr(self, "debug_zero_wrench_hist", False):
                obs_dict["wrench_hist"] = torch.zeros_like(wh_t)
            else:
                obs_dict["wrench_hist"] = wh_t
        except Exception as e:
            logger.warning(f"[WRENCH_HIST] build/inject failed: {e}")

    def _log_ctrl_fz_and_paep(self, control_step: int):
        """Log Fz + PAEP at *control_fps*, aligned to action_command_thread."""
        # --- wrench / Fz ---
        fz = float("nan")
        try:
            if self.enable_dense_wrench_hist and (len(self._wrench_dense_buf) > 0):
                w = np.asarray(self._wrench_dense_buf[-1], dtype=np.float32).reshape(-1)[:6]
                fz = float(w[2])
        except Exception:
            pass

        self._fz_dense.append(np.float32(fz))
        self._fz_dense_t.append(np.float64(time.time()))
        self._fz_dense_step.append(np.int32(control_step))

        # --- PAEP snapshot (compact) ---
        try:
            with self._paep_cache_lock:
                pc = self._fast_paep_p_contact
                pp = self._fast_paep_p_phase

            with self._phase_lock:
                phase_id = self._latest_phase_id

            rec = {
                "control_step": int(control_step),
                "phase_id": None if phase_id is None else int(phase_id),
                "p_contact": None if pc is None else float(pc),
            }

            if pp is not None:
                pp = np.asarray(pp, dtype=np.float32).reshape(-1)
                if pp.size > 0:
                    k = 2 if pp.size >= 2 else 1
                    idx = np.argpartition(pp, -k)[-k:]
                    idx = idx[np.argsort(pp[idx])[::-1]]
                    rec["p_phase_top_idx"] = [int(i) for i in idx.tolist()]
                    rec["p_phase_top_val"] = [float(pp[i]) for i in idx.tolist()]
                    p = np.clip(pp, 1e-8, 1.0)
                    rec["p_phase_entropy"] = float(-(p * np.log(p)).sum())

            self._dbg("paep_ctrl", **rec)
        except Exception:
            pass




    def run(self, policy: Union[DiffusionUnetImagePolicy]):
        logger.info(f"[DBG] runner file = {os.path.abspath(__file__)}")
        if self.use_latent_action_with_rnn_decoder:
            assert policy.at.use_rnn_decoder, "Policy should use rnn decoder for latent action."
        else:
            assert not hasattr(policy, "at") or not policy.at.use_rnn_decoder, "Policy should not use rnn decoder for action."

        device = policy.device

        executor = MultiThreadedExecutor()
        executor.add_node(self.env)

        try:
            spin_thread = threading.Thread(target=self.spin_executor, args=(executor,), daemon=True)
            spin_thread.start()

            if self._fz_run_ts is None:
                self._fz_run_ts = _now_timestamp()
            self._fz_run_dir = _make_fz_run_dir(self.output_dir, self.task_name, self.policy_name, self._fz_run_ts)

            logger.info(f"[FZ] logging to: {self._fz_run_dir}")

            time.sleep(2)

            for episode_idx in tqdm.tqdm(
                range(0, self.eval_episodes),
                desc=f"Eval for {self.task_name}",
                leave=False,
                mininterval=self.tqdm_interval_sec,
            ):
                logger.info(f"Start evaluation episode {episode_idx}")
                reset_flag = py_cli_interaction.parse_cli_bool("Has the environment reset finished?", default_value=True)
                if not reset_flag:
                    logger.warning("Skip this episode.")
                    continue

                logger.info("Start episode rollout.")
                

                self.env.reset()
                time.sleep(1)

                policy.reset()

                with self.tcp_buf_lock:
                    self.tcp_ensemble_buffer.clear()
                with self.grip_buf_lock:
                    self.gripper_ensemble_buffer.clear()

                # IMPORTANT: reset last actions per episode
                self._last_step_action = None
                self._last_tcp_step_action = None
                self._last_gripper_step_action = None

                if self.enable_fast and (self._fast is not None):
                    self._fast.reset()

                with self._paep_cache_lock:
                    self._fast_paep_p_contact = None
                    self._fast_paep_p_phase = None
                with self._phase_lock:
                    self._latest_phase_id = None



                if self.enable_video_recording:
                    video_path = os.path.join(self.video_dir, f"episode_{episode_idx}.mp4")
                    self.start_record_video(video_path)

                self.stop_event.clear()
                self._none_streak = 0
                self.action_step_count = 0

                # ---- start 24Hz wrench polling thread (for dense wrench_hist) ----
                self._start_dense_wrench_polling()

                # ---- start PAEP control-fps polling thread ----
                # device is already defined in your rollout (same as diffusion uses)
                self._start_paep_polling(policy, device)
                
                self._fz_dense.clear()
                self._fz_dense_t.clear()
                self._fz_dense_step.clear()
                self._fz_dense_ctr = 0
                self._dbg_buf = []

                # ---- wait PAEP cache ready (avoid first few steps having no PAEP for diffusion/fast) ----
                t_wait0 = time.time()
                while True:
                    with self._paep_cache_lock:
                        ok = (self._fast_paep_p_contact is not None) and (self._fast_paep_p_phase is not None)
                    if ok:
                        break
                    if time.time() - t_wait0 > 2.0:
                        logger.warning("[PAEP] cache still not ready after 2s; continue anyway")
                        break
                    time.sleep(0.01)


                time.sleep(0.5)
                action_thread = threading.Thread(target=self.action_command_thread, args=(policy, self.stop_event), daemon=True)
                action_thread.start()

                step_count = 0
                start_timestamp = time.time()
                infer_step = 0

                try:
                    while True:
                        if not action_thread.is_alive():
                            raise RuntimeError("action_command_thread died. Check logs above for the real exception.")

                        start_time = time.time()

                        obs = self.env.get_obs(obs_steps=self.n_obs_steps, temporal_downsample_ratio=self.obs_temporal_downsample_ratio)
                        if len(obs) == 0:
                            logger.warning("No observation received! Skip this step.")
                            cur_time = time.time()
                            precise_sleep(max(0.0, self.inference_interval_time - (cur_time - start_time)))
                            step_count += self.steps_per_inference
                            infer_step += 1
                            continue

                        np_obs_dict = dict(obs)

                        np_obs_dict = get_real_obs_dict(env_obs=np_obs_dict, shape_meta=self.shape_meta, bgr_to_rgb=True)

                        # if step_count == 0:
                        #     for k in ["external_img", "left_wrist_img"]:
                        #         if k in np_obs_dict:
                        #             x = np.asarray(np_obs_dict[k])
                        #             logger.info(
                        #                 f"[POLICY_IMG] {k}: dtype={x.dtype} shape={x.shape} "
                        #                 f"min={x.min():.4f} max={x.max():.4f} mean={x.mean():.4f}"
                        #             )

                        np_obs_dict, np_absolute_obs_dict = self.pre_process_obs(np_obs_dict)

                        obs_dict = dict_apply(np_obs_dict, lambda x: torch.from_numpy(x).unsqueeze(0).to(device=device))
                        
                        self._inject_wrench_hist_for_diffusion(obs_dict, device=device)

                        self._inject_paep_cache_for_diffusion(obs_dict, device=device)

                        with torch.no_grad():
                            with self._policy_infer_lock:
                                if self.use_latent_action_with_rnn_decoder:
                                    action_dict = policy.predict_action(
                                        obs_dict,
                                        dataset_obs_temporal_downsample_ratio=self.dataset_obs_temporal_downsample_ratio,
                                        return_latent_action=True,
                                    )
                                else:
                                    action_dict = policy.predict_action(obs_dict)


                        np_action_dict = dict_apply(action_dict, lambda x: x.detach().to("cpu").numpy())

                        # ---- record fusion/attn/policy-stash at inference_fps (+ PAEP snapshot used by diffusion) ----
                        try:
                            d = extract_fusion_debug(policy)  # JSON-serializable

                            # snapshot PAEP cache at this inference moment
                            with self._paep_cache_lock:
                                pc_snap = self._fast_paep_p_contact
                                pp_snap = self._fast_paep_p_phase

                            if pc_snap is not None:
                                d["paep_cache/p_contact"] = float(pc_snap)

                            # keep PAEP snapshot compact (avoid storing full vector at inference_fps)
                            if pp_snap is not None:
                                try:
                                    pp = np.asarray(pp_snap, dtype=np.float32).reshape(-1)
                                    if pp.size > 0:
                                        k = 2 if pp.size >= 2 else 1
                                        idx = np.argpartition(pp, -k)[-k:]
                                        idx = idx[np.argsort(pp[idx])[::-1]]
                                        d["paep_cache/p_phase_top_idx"] = [int(i) for i in idx.tolist()]
                                        d["paep_cache/p_phase_top_val"] = [float(pp[i]) for i in idx.tolist()]
                                except Exception:
                                    pass


                            self._dbg(
                                "fusion_infer",
                                infer_step=int(infer_step),
                                control_step=int(step_count),
                                **d
                            )
                        except Exception:
                            pass


                    
                        # --- PAEP phase logging (policy v4) ---
                        # IMPORTANT:
                        #   PAEP cache for fast/diffusion is updated ONLY by the control-fps PAEP polling thread.
                        #   Do NOT write self._fast_paep_p_contact / self._fast_paep_p_phase here (avoid double-writer / redundancy).
                        if "paep_phase_id" in np_action_dict:
                            phase_id = int(np.array(np_action_dict["paep_phase_id"]).reshape(-1)[0])

                            # optional: still parse these for logging (no cache write)
                            conf = float("nan")
                            if "paep_phase_conf" in np_action_dict:
                                conf = float(np.array(np_action_dict["paep_phase_conf"]).reshape(-1)[0])

                            g = float("nan")
                            if "paep_g_contact" in np_action_dict:
                                g = float(np.array(np_action_dict["paep_g_contact"]).reshape(-1)[0])

                            # (optional) if you want, keep a small debug stash
                            self._last_paep_debug = {"phase_id": phase_id, "phase_conf": conf, "g_contact": g}


                            # # ---- log (keep 10-step frequency) ----
                            # if (infer_step % 10) == 0 and (p is not None):
                            #     fz_str = "nan" if fz is None else f"{fz:.3f}"
                            #     msg = (
                            #         f"infer_step={infer_step} step={step_count} fz={fz_str} "
                            #         f"p_phase={p.tolist()} phase_id={phase_id} conf={conf:.3f} "
                            #         f"p_contact={pc:.3f} g={g:.3f}"
                            #     )
                            #     logger.info("[PAEP] " + msg)
                            #     self.paep_logger.info(msg)
                            #     for h in self.paep_logger.handlers:
                            #         try:
                            #             h.flush()
                            #         except Exception:
                            #             pass

                            


                        action_all = np_action_dict["action"].squeeze(0)

                        # --- relative handling ---
                        if self.use_latent_action_with_rnn_decoder:
                            if self.use_relative_action:
                                base_absolute_action = np.concatenate(
                                    [
                                        np_absolute_obs_dict["left_robot_tcp_pose"][-1] if "left_robot_tcp_pose" in np_absolute_obs_dict else np.array([]),
                                        np_absolute_obs_dict["right_robot_tcp_pose"][-1] if "right_robot_tcp_pose" in np_absolute_obs_dict else np.array([]),
                                    ],
                                    axis=-1,
                                )
                                action_all = np.concatenate([action_all, base_absolute_action[np.newaxis, :].repeat(action_all.shape[0], axis=0)], axis=-1)

                            action_all = np.concatenate(
                                [
                                    action_all,
                                    np.arange(
                                        self.n_obs_steps * self.dataset_obs_temporal_downsample_ratio,
                                        action_all.shape[0] + self.n_obs_steps * self.dataset_obs_temporal_downsample_ratio,
                                    )[:, np.newaxis],
                                ],
                                axis=-1,
                            )
                        else:
                            if self.use_relative_action:
                                base_absolute_action = np.concatenate(
                                    [
                                        np_absolute_obs_dict["left_robot_tcp_pose"][-1] if "left_robot_tcp_pose" in np_absolute_obs_dict else np.array([]),
                                        np_absolute_obs_dict["right_robot_tcp_pose"][-1] if "right_robot_tcp_pose" in np_absolute_obs_dict else np.array([]),
                                    ],
                                    axis=-1,
                                )
                                action_all = relative_actions_to_absolute_actions(action_all, base_absolute_action)

                        # --- optional debug sine & freeze gripper (non-latent only) ---
                        if self.debug_sine and (not self.use_latent_action_with_rnn_decoder):
                            axis_map = {"x": 0, "y": 1, "z": 2}
                            ax = axis_map.get(self.debug_sine_axis, 0)

                            Tseq = action_all.shape[0]
                            t_base = time.time() - start_timestamp
                            t_seq = t_base + (np.arange(Tseq, dtype=np.float32) / float(self.control_fps))
                            dx = self.debug_sine_amp * np.sin(2.0 * np.pi * self.debug_sine_freq * t_seq)

                            def _apply_abs_pos(arr: np.ndarray, pos_start: int, base_pos: np.ndarray):
                                if arr.shape[-1] < pos_start + 3:
                                    return
                                arr[:, pos_start:pos_start + 3] = base_pos[None, :3]
                                arr[:, pos_start + ax] = base_pos[ax] + dx

                            left_base = (
                                np_absolute_obs_dict["left_robot_tcp_pose"][-1][:3]
                                if "left_robot_tcp_pose" in np_absolute_obs_dict
                                else action_all[0, :3]
                            )
                            D = action_all.shape[-1]
                            if D in (4, 10):
                                _apply_abs_pos(action_all, 0, np.asarray(left_base))
                            elif D in (8, 20):
                                _apply_abs_pos(action_all, 0, np.asarray(left_base))

                            if self.debug_freeze_gripper:
                                if D == 4:
                                    action_all[:, 3] = self.debug_gripper_value
                                elif D == 8:
                                    action_all[:, 6] = self.debug_gripper_value
                                    action_all[:, 7] = self.debug_gripper_value
                                elif D == 10:
                                    action_all[:, 9] = self.debug_gripper_value
                                elif D == 20:
                                    action_all[:, 18] = self.debug_gripper_value
                                    action_all[:, 19] = self.debug_gripper_value

                        # --- interpolation ---
                        if self.action_interpolation_ratio > 1:
                            if self.use_latent_action_with_rnn_decoder:
                                action_all = action_all.repeat(self.action_interpolation_ratio, axis=0)
                            else:
                                action_all = interpolate_actions_with_ratio(action_all, self.action_interpolation_ratio)

                        # --- feed tcp buffer (latency clamp + empty guard) ---
                        if step_count % self.tcp_action_update_interval == 0:
                            T = int(action_all.shape[0])
                            tcp_lat = int(self.latency_step)
                            tcp_lat = min(max(tcp_lat, 0), max(T - 1, 0))

                            if self.use_latent_action_with_rnn_decoder:
                                tcp_action = action_all[tcp_lat:, ...]
                            else:
                                if action_all.shape[-1] == 4:
                                    tcp_action = action_all[tcp_lat:, :3]
                                elif action_all.shape[-1] == 8:
                                    tcp_action = action_all[tcp_lat:, :6]
                                elif action_all.shape[-1] == 10:
                                    tcp_action = action_all[tcp_lat:, :9]
                                elif action_all.shape[-1] == 20:
                                    tcp_action = action_all[tcp_lat:, :18]
                                else:
                                    raise NotImplementedError

                            if tcp_action.shape[0] == 0:
                                logger.warning(f"[LATENCY] tcp_action empty: step={step_count} T={T} tcp_lat={tcp_lat}")
                            else:
                                with self.tcp_buf_lock:
                                    self.tcp_ensemble_buffer.add_action(tcp_action, step_count)

                        # --- feed gripper buffer (latency clamp + empty guard) ---
                        if step_count % self.gripper_action_update_interval == 0:
                            T = int(action_all.shape[0])
                            grip_lat = int(self.gripper_latency_step)
                            grip_lat = min(max(grip_lat, 0), max(T - 1, 0))

                            if self.use_latent_action_with_rnn_decoder:
                                gripper_action = action_all[grip_lat:, ...]
                            else:
                                if action_all.shape[-1] == 4:
                                    gripper_action = action_all[grip_lat:, 3:]
                                elif action_all.shape[-1] == 8:
                                    gripper_action = action_all[grip_lat:, 6:]
                                elif action_all.shape[-1] == 10:
                                    gripper_action = action_all[grip_lat:, 9:]
                                elif action_all.shape[-1] == 20:
                                    gripper_action = action_all[grip_lat:, 18:]
                                else:
                                    raise NotImplementedError

                            if gripper_action.shape[0] == 0:
                                logger.warning(f"[LATENCY] gripper_action empty: step={step_count} T={T} grip_lat={grip_lat}")
                            else:
                                with self.grip_buf_lock:
                                    self.gripper_ensemble_buffer.add_action(gripper_action, step_count)

                        infer_step += 1
                        step_count += self.steps_per_inference

                        cur_time = time.time()
                        precise_sleep(max(0.0, self.inference_interval_time - (cur_time - start_time)))

                        if cur_time - start_timestamp >= self.max_duration_time:
                            logger.info(f"Episode {episode_idx} reaches max duration time {self.max_duration_time} seconds.")
                            break

                except KeyboardInterrupt:
                    logger.warning("KeyboardInterrupt! Terminate the episode now!")
                finally:
                    # 1) stop threads FIRST to avoid concurrent writes during dumping
                    try:
                        self.stop_event.set()   # ✅ stop action_command_thread
                    except Exception:
                        pass

                    try:
                        self._stop_paep_polling()          # ✅ stop PAEP control-fps thread
                    except Exception:
                        pass

                    try:
                        self._stop_dense_wrench_polling()  # ✅ stop wrench polling thread
                    except Exception:
                        pass

                    # join action thread (now it should exit quickly)
                    try:
                        action_thread.join(timeout=2.0)
                        if action_thread.is_alive():
                            logger.warning("[ACTION_THREAD] still alive after join timeout; continue teardown.")
                    except Exception:
                        pass

                    # 2) now dump ONE file per episode: Fz(control_fps) + PAEP(control_fps) + fusion(inference_fps)
                    debug_npz_path = osp.join(self._fz_run_dir, f"episode_{episode_idx:03d}_debug_all.npz")

                    # make local snapshot to avoid any residual races
                    try:
                        fz_arr = np.asarray(self._fz_dense, dtype=np.float32)
                        fz_t_arr = np.asarray(self._fz_dense_t, dtype=np.float64)
                        fz_step_arr = np.asarray(self._fz_dense_step, dtype=np.int32)
                        dbg_snapshot = list(getattr(self, "_dbg_buf", []))
                    except Exception:
                        fz_arr = np.asarray([], dtype=np.float32)
                        fz_t_arr = np.asarray([], dtype=np.float64)
                        fz_step_arr = np.asarray([], dtype=np.int32)
                        dbg_snapshot = []

                    paep_ctrl = [r for r in dbg_snapshot if r.get("tag") == "paep_ctrl"]
                    fusion_infer = [r for r in dbg_snapshot if r.get("tag") == "fusion_infer"]

                    np.savez_compressed(
                        debug_npz_path,
                        # ---- Fz at control_fps ----
                        fz=fz_arr,
                        fz_wall_time=fz_t_arr,
                        fz_control_step=fz_step_arr,

                        # ---- PAEP at control_fps (dict list) ----
                        paep_ctrl=np.asarray(paep_ctrl, dtype=object),

                        # ---- fusion at inference_fps (dict list) ----
                        fusion_infer=np.asarray(fusion_infer, dtype=object),

                        # ---- meta ----
                        control_fps=np.asarray([self.control_fps], dtype=np.int32),
                        inference_fps=np.asarray([self.inference_fps], dtype=np.int32),
                        episode_idx=np.asarray([episode_idx], dtype=np.int32),
                    )

                    logger.info(f"[DBG] saved: {debug_npz_path} (fz+paep_ctrl+fusion_infer)")

                    # ---- ask user to label episode success/failure (optional) ----
                    try:
                        episode_success = py_cli_interaction.parse_cli_bool(
                            "Was this episode successful? (y/n)",
                            default_value=True
                        )
                    except Exception:
                        episode_success = None

                    if self.enable_video_recording:
                        self.stop_record_video()
                    self.env.save_exp(episode_idx)


        finally:
            try:
                executor.shutdown()
            except Exception:
                pass
            try:
                self.env.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass

