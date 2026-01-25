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

from reactive_diffusion_policy.policy.diffusion_unet_image_policy import DiffusionUnetImagePolicy
from reactive_diffusion_policy.common.pytorch_util import dict_apply
from reactive_diffusion_policy.common.precise_sleep import precise_sleep
from reactive_diffusion_policy.env.real_bimanual.real_env import RealRobotEnvironment
from reactive_diffusion_policy.real_world.real_inference_util import get_real_obs_dict
from reactive_diffusion_policy.real_world.real_world_transforms import RealWorldTransforms
from reactive_diffusion_policy.common.space_utils import ortho6d_to_rotation_matrix
from reactive_diffusion_policy.common.ensemble import EnsembleBuffer
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

__all__ = ["RealRunner"]

PAEP_EVENT_NAMES = ["idle", "approach", "under", "effective", "over", "retreat"]


def _now_timestamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _sanitize(s: str):
    return "".join(c if (c.isalnum() or c in "-_.") else "_" for c in str(s))


def _make_fz_run_dir(output_dir: str, policy_name: str, run_ts: str):
    run_dir = osp.join(output_dir, "fz_logs", _sanitize(policy_name), run_ts)
    os.makedirs(run_dir, exist_ok=True)
    return run_dir


def _open_fz_csv(run_dir: str, episode_idx: int):
    csv_path = osp.join(run_dir, f"episode_{episode_idx:03d}_left_tcp_fz.csv")
    f = open(csv_path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["wall_time", "step_count", "fz"])
    return csv_path, f, w


def _extract_fz_from_obs(np_obs_dict):
    candidate_keys = [
        "left_tcp_wrench", "tcp_wrench", "wrench",
        "left_wrench", "robot0_tcp_wrench", "left_robot_tcp_wrench",
    ]
    key = next((k for k in candidate_keys if k in np_obs_dict), None)
    if key is None:
        return None
    w = np.asarray(np_obs_dict[key])
    while w.ndim > 2:
        w = w[0]
    if w.ndim == 2:
        w = w[-1]
    if w.shape[-1] < 3:
        return None
    return float(w[2])


# --- thread env settings ---
os.environ["OPENBLAS_NUM_THREADS"] = "12"
os.environ["MKL_NUM_THREADS"] = "12"
os.environ["NUMEXPR_NUM_THREADS"] = "12"
os.environ["OMP_NUM_THREADS"] = "12"
cv2.setNumThreads(12)

total_cores = psutil.cpu_count()
num_cores_to_bind = 10
cores_to_bind = set(range(min(num_cores_to_bind, total_cores)))
os.sched_setaffinity(0, cores_to_bind)


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
    ):
        self.task_name = task_name
        self.transforms = RealWorldTransforms(option=transform_params)
        self.shape_meta = dict(shape_meta)
        self.eval_episodes = eval_episodes

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
        self.paep_logger = self._setup_paep_logger(self.output_dir)

        self._fz_run_ts = None
        self._fz_run_dir = None

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

            if self.use_latent_action_with_rnn_decoder:
                tcp_extended_obs_step = int(tcp_step_action[-1])
                gripper_extended_obs_step = int(gripper_step_action[-1])
                tcp_step_action = tcp_step_action[:-1]
                gripper_step_action = gripper_step_action[:-1]

                longer_extended_obs_step = max(tcp_extended_obs_step, gripper_extended_obs_step)
                obs_temporal_downsample_ratio = self.obs_temporal_downsample_ratio if self.downsample_extended_obs else 1
                extended_obs = self.env.get_obs(longer_extended_obs_step, temporal_downsample_ratio=obs_temporal_downsample_ratio)

                if self.use_relative_action:
                    action_dim = self.shape_meta["obs"]["left_robot_tcp_pose"]["shape"][0]
                    if "right_robot_tcp_pose" in self.shape_meta["obs"]:
                        action_dim += self.shape_meta["obs"]["right_robot_tcp_pose"]["shape"][0]
                    tcp_base_absolute_action = tcp_step_action[-action_dim:]
                    gripper_base_absolute_action = gripper_step_action[-action_dim:]
                    tcp_step_action = tcp_step_action[:-action_dim]
                    gripper_step_action = gripper_step_action[:-action_dim]

                np_extended_obs_dict = dict(extended_obs)
                np_extended_obs_dict = get_real_obs_dict(env_obs=np_extended_obs_dict, shape_meta=self.shape_meta, is_extended_obs=True)
                np_extended_obs_dict, _ = self.pre_process_extended_obs(np_extended_obs_dict)
                extended_obs_dict = dict_apply(np_extended_obs_dict, lambda x: torch.from_numpy(x).unsqueeze(0))

                tcp_step_latent_action = torch.from_numpy(tcp_step_action.astype(np.float32)).unsqueeze(0)
                gripper_step_latent_action = torch.from_numpy(gripper_step_action.astype(np.float32)).unsqueeze(0)

                dor = self.dataset_obs_temporal_downsample_ratio
                tcp_step_action = policy.predict_from_latent_action(tcp_step_latent_action, extended_obs_dict, tcp_extended_obs_step, dor)["action"][0].detach().cpu().numpy()
                gripper_step_action = policy.predict_from_latent_action(gripper_step_latent_action, extended_obs_dict, gripper_extended_obs_step, dor)["action"][0].detach().cpu().numpy()

                if self.use_relative_action:
                    tcp_step_action = relative_actions_to_absolute_actions(tcp_step_action, tcp_base_absolute_action)
                    gripper_step_action = relative_actions_to_absolute_actions(gripper_step_action, gripper_base_absolute_action)

                if tcp_step_action.shape[-1] == 4:
                    tcp_len = 3
                elif tcp_step_action.shape[-1] == 8:
                    tcp_len = 6
                elif tcp_step_action.shape[-1] == 10:
                    tcp_len = 9
                elif tcp_step_action.shape[-1] == 20:
                    tcp_len = 18
                else:
                    raise NotImplementedError

                tcp_step_action = tcp_step_action[-1][:tcp_len]
                gripper_step_action = gripper_step_action[-1][tcp_len:]

            combined_action = np.concatenate([tcp_step_action, gripper_step_action], axis=-1)
            step_action, is_bimanual = self.post_process_action(combined_action[np.newaxis, :])
            step_action = step_action.squeeze(0)

            # ---- EMA smoothing (optional) ----
            a = self.action_smoothing_alpha
            if a > 0.0:
                if self._last_step_action is None:
                    self._last_step_action = step_action.copy()
                else:
                    if self.smooth_xyz_only:
                        sm = step_action.copy()
                        # left xyz: 0:3 , right xyz: 8:11 (because left 8 dims +2? here action_all is 16 dims)
                        # safer: smooth first 3 only; keep others
                        sm[:3] = (1 - a) * self._last_step_action[:3] + a * step_action[:3]
                        step_action = sm
                    else:
                        step_action = (1 - a) * self._last_step_action + a * step_action
                    self._last_step_action = step_action.copy()

            self.env.execute_action(step_action, use_relative_action=False, is_bimanual=is_bimanual)

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
            self._fz_run_dir = _make_fz_run_dir(self.output_dir, self.policy_name, self._fz_run_ts)
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
                fz_log = []
                fz_step_log = []
                csv_path, csv_f, csv_w = _open_fz_csv(self._fz_run_dir, episode_idx)
                meta_path = osp.join(self._fz_run_dir, f"episode_{episode_idx:03d}_left_tcp_fz_meta.json")

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

                if self.enable_video_recording:
                    video_path = os.path.join(self.video_dir, f"episode_{episode_idx}.mp4")
                    self.start_record_video(video_path)

                self.stop_event.clear()
                self._none_streak = 0
                self.action_step_count = 0

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

                        if step_count == 0:
                            for k in ["external_img", "left_wrist_img"]:
                                if k in np_obs_dict:
                                    x = np.asarray(np_obs_dict[k])
                                    logger.info(
                                        f"[RAW_IMG] {k}: dtype={x.dtype} shape={x.shape} "
                                        f"min={x.min()} max={x.max()} mean={x.mean():.4f}"
                                    )


                        fz = _extract_fz_from_obs(np_obs_dict)
                        if fz is not None:
                            fz_log.append(fz)
                            fz_step_log.append(step_count)
                            csv_w.writerow([time.time(), step_count, fz])

                        np_obs_dict = get_real_obs_dict(env_obs=np_obs_dict, shape_meta=self.shape_meta)
                        
                        if step_count == 0:
                            for k in ["external_img", "left_wrist_img"]:
                                if k in np_obs_dict:
                                    x = np.asarray(np_obs_dict[k])
                                    logger.info(
                                        f"[POLICY_IMG] {k}: dtype={x.dtype} shape={x.shape} "
                                        f"min={x.min():.4f} max={x.max():.4f} mean={x.mean():.4f}"
                                    )


                        
                        np_obs_dict, np_absolute_obs_dict = self.pre_process_obs(np_obs_dict)

                        obs_dict = dict_apply(np_obs_dict, lambda x: torch.from_numpy(x).unsqueeze(0).to(device=device))

                        with torch.no_grad():
                            if self.use_latent_action_with_rnn_decoder:
                                action_dict = policy.predict_action(
                                    obs_dict,
                                    dataset_obs_temporal_downsample_ratio=self.dataset_obs_temporal_downsample_ratio,
                                    return_latent_action=True,
                                )
                            else:
                                action_dict = policy.predict_action(obs_dict)

                        np_action_dict = dict_apply(action_dict, lambda x: x.detach().to("cpu").numpy())

                        # ---- DBG: how much force residual is injected into vision ----
                        if (infer_step % 10) == 0:
                            try:
                                # try multiple possible paths to find fusion module
                                fusion = None
                                for cand in ["fusion", "model.fusion", "net.fusion", "policy.fusion"]:
                                    obj = policy
                                    ok = True
                                    for part in cand.split("."):
                                        if not hasattr(obj, part):
                                            ok = False
                                            break
                                        obj = getattr(obj, part)
                                    if ok:
                                        fusion = obj
                                        break

                                if fusion is None:
                                    # only print once in a while to avoid spamming
                                    logger.info("[DBG][FUSION] fusion module not found on policy (tried fusion/model.fusion/net.fusion/policy.fusion)")
                                else:
                                    dd = getattr(fusion, "_last_fusion_debug", None)
                                    if dd is None:
                                        logger.info("[DBG][FUSION] fusion._last_fusion_debug is None (fusion found, but debug not produced)")
                                    else:
                                        # dd entries might be torch tensors; make safe float conversion
                                        def _to_float(x):
                                            try:
                                                if torch.is_tensor(x):
                                                    return float(x.detach().cpu().item())
                                                return float(x)
                                            except Exception:
                                                return float("nan")

                                        inj_mean = _to_float(dd.get("inj_mean", float("nan")))
                                        delta_norm_mean = _to_float(dd.get("delta_norm_mean", float("nan")))
                                        v_norm_mean = _to_float(dd.get("v_norm_mean", float("nan")))
                                        inj_ratio_mean = _to_float(dd.get("inj_ratio_mean", float("nan")))
                                        amp_mean = _to_float(dd.get("amp_mean", float("nan")))

                                        dbg_msg = (
                                            f"infer_step={infer_step} step={step_count} "
                                            f"inj_mean={inj_mean:.3f} "
                                            f"delta_norm_mean={delta_norm_mean:.3f} "
                                            f"v_norm_mean={v_norm_mean:.3f} "
                                            f"inj_ratio_mean={inj_ratio_mean:.3f} "
                                            f"amp_mean={amp_mean:.3f}"
                                        )

                                        # terminal (loguru)
                                        logger.info("[DBG][FUSION] " + dbg_msg)
                                        # file (python logging -> same paep_events_*.log)
                                        self.paep_logger.info("[DBG][FUSION] " + dbg_msg)
                                        for h in self.paep_logger.handlers:
                                            try:
                                                h.flush()
                                            except Exception:
                                                pass

                            except Exception as e:
                                logger.warning(f"[DBG][FUSION] failed to read fusion debug: {e}")
                                try:
                                    self.paep_logger.info(f"[DBG][FUSION] failed to read fusion debug: {e}")
                                    for h in self.paep_logger.handlers:
                                        try:
                                            h.flush()
                                        except Exception:
                                            pass
                                except Exception:
                                    pass


                        # --- PAEP phase logging (policy v2) ---
                        if "paep_phase_id" in np_action_dict:
                            phase_id = int(np.array(np_action_dict["paep_phase_id"]).reshape(-1)[0])

                            conf = float("nan")
                            if "paep_phase_conf" in np_action_dict:
                                conf = float(np.array(np_action_dict["paep_phase_conf"]).reshape(-1)[0])

                            g = float("nan")
                            if "paep_g_contact" in np_action_dict:
                                g = float(np.array(np_action_dict["paep_g_contact"]).reshape(-1)[0])

                            pc = float("nan")
                            if "paep_p_contact" in np_action_dict:
                                pc = float(np.array(np_action_dict["paep_p_contact"]).reshape(-1)[0])

                            if (infer_step % 10) == 0 and "paep_p_phase" in np_action_dict:
                                p = np_action_dict["paep_p_phase"].reshape(-1, 3)[0]
                                fz_str = "nan" if fz is None else f"{fz:.3f}"
                                msg = (
                                    f"infer_step={infer_step} step={step_count} fz={fz_str} "
                                    f"p_phase={p.tolist()} phase_id={phase_id} conf={conf:.3f} "
                                    f"p_contact={pc:.3f} g={g:.3f}"
                                )
                                logger.info("[PAEP] " + msg)
                                self.paep_logger.info(msg)
                                for h in self.paep_logger.handlers:
                                    try:
                                        h.flush()
                                    except Exception:
                                        pass




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
                    self.stop_event.set()

                    try:
                        csv_f.flush()
                        csv_f.close()
                    except Exception:
                        pass

                    npy_path = osp.join(self._fz_run_dir, f"episode_{episode_idx:03d}_left_tcp_fz.npy")
                    np.save(npy_path, np.asarray(fz_log, dtype=np.float32))

                    with open(meta_path, "w") as f:
                        json.dump({
                            "policy": self.policy_name,
                            "run_timestamp": self._fz_run_ts,
                            "episode_idx": episode_idx,
                            "csv_path": csv_path,
                            "npy_path": npy_path,
                            "step": fz_step_log,
                            "n": len(fz_log),
                            "source_key": "left_robot_tcp_wrench (or compatible) index 2 => Fz",
                        }, f, indent=2)
                    logger.info(f"[FZ] saved: {csv_path} (+ npy/meta)")

                    action_thread.join()

                    if self.enable_video_recording:
                        self.stop_record_video()
                    self.env.save_exp(episode_idx)

            spin_thread.join()
        finally:
            self.env.destroy_node()
# ====== END ======
