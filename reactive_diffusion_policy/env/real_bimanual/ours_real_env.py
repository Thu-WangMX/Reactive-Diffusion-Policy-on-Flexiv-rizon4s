import pickle
import os
import os.path as osp
import threading

import cv2
import time
import torch
import numpy as np
import requests
from omegaconf import DictConfig
from copy import deepcopy
from typing import Union, List, Dict, Optional
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from collections import deque

from loguru import logger
from reactive_diffusion_policy.real_world.real_world_transforms import RealWorldTransforms
from reactive_diffusion_policy.real_world.device_mapping.device_mapping_utils import get_topic_and_type
from reactive_diffusion_policy.real_world.device_mapping.device_mapping_server import DeviceToTopic
from reactive_diffusion_policy.real_world.ros_data_converter import ROS2DataConverter
from reactive_diffusion_policy.common.data_models import SensorMessage, SensorMessageList, BimanualRobotStates
from reactive_diffusion_policy.common.time_utils import convert_ros_time_to_float
from reactive_diffusion_policy.common.ring_buffer import RingBuffer
from reactive_diffusion_policy.real_world.post_process_utils import DataPostProcessingManager
from reactive_diffusion_policy.common.space_utils import (
    pose_6d_to_pose_7d,
    pose_6d_to_4x4matrix,
    matrix4x4_to_pose_6d
)

import pyinstrument


def stack_last_n_obs(all_obs, n_steps: int) -> Union[np.ndarray, torch.Tensor]:
    assert (len(all_obs) > 0)
    all_obs = list(all_obs)
    if isinstance(all_obs[0], np.ndarray):
        result = np.zeros((n_steps,) + all_obs[-1].shape, dtype=all_obs[-1].dtype)
        start_idx = -min(n_steps, len(all_obs))
        result[start_idx:] = np.array(all_obs[start_idx:])
        if n_steps > len(all_obs):
            # pad
            result[:start_idx] = result[start_idx]
    elif isinstance(all_obs[0], torch.Tensor):
        result = torch.zeros((n_steps,) + all_obs[-1].shape, dtype=all_obs[-1].dtype)
        start_idx = -min(n_steps, len(all_obs))
        result[start_idx:] = torch.stack(all_obs[start_idx:])
        if n_steps > len(all_obs):
            # pad
            result[:start_idx] = result[start_idx]
    else:
        raise RuntimeError(f'Unsupported obs type {type(all_obs[0])}')
    return result


class RealRobotEnvironment(Node):
    start_gripper_interval_control: bool = False
    gripper_interval_count: int = 0
    last_gripper_width_target: List[float] = [0.1, 0.1]

    def __init__(self,
                 robot_server_ip: str,
                 robot_server_port: int,
                 transforms: RealWorldTransforms,
                 device_mapping_server_ip: str,
                 device_mapping_server_port: int,
                 data_processing_params: DictConfig,
                 max_fps: int = 30,
                 # gripper control parameters
                 use_force_control_for_gripper: bool = True,
                 max_gripper_width: float = 0.05,
                 min_gripper_width: float = 0.,
                 grasp_force: float = 5.0,
                 enable_gripper_interval_control: bool = False,
                 gripper_control_time_interval: float = 60,
                 gripper_control_width_precision: float = 0.02,
                 gripper_width_threshold: float = 0.04,
                 enable_gripper_width_clipping: bool = True,
                 enable_exp_recording: bool = False,
                 output_dir: Optional[str] = None,
                 vcamera_server_ip: Optional[str] = None,
                 vcamera_server_port: Optional[int] = None,
                 time_check: bool = False,
                 debug: bool = False):
        super().__init__('real_env')
        self.robot_server_ip = robot_server_ip
        self.robot_server_port = robot_server_port
        self.transforms = transforms
        self.max_fps = max_fps

        # gripper control parameters
        self.use_force_control_for_gripper = use_force_control_for_gripper
        self.max_gripper_width = max_gripper_width
        self.min_gripper_width = min_gripper_width
        self.grasp_force = grasp_force
        self.enable_gripper_interval_control = enable_gripper_interval_control
        self.gripper_control_time_interval = gripper_control_time_interval
        self.gripper_control_width_precision = gripper_control_width_precision
        self.gripper_width_threshold = gripper_width_threshold
        self.enable_gripper_width_clipping = enable_gripper_width_clipping

        self.data_processing_manager = DataPostProcessingManager(transforms, **data_processing_params)
        self.debug = debug
        self.subscribers = []
        self.obs_buffer = RingBuffer(size=1024, fps=max_fps)

        self.mutex = threading.Lock()

        # ---- fast-path cache (avoid calling get_obs in high-rate threads) ----
        # updated every callback; runner can read this to build dense wrench history at 24Hz
        self.latest_left_robot_tcp_wrench = None  # np.ndarray (6,), float32

        # print raw_obs_dict keys only once (optional but very useful to avoid silent key mismatch)
        self._printed_raw_obs_keys = False

        self.enable_exp_recording = enable_exp_recording
        if self.enable_exp_recording:
            assert output_dir is not None, "output_dir must be provided for experiment recording"
            assert vcamera_server_ip is not None and vcamera_server_port is not None, \
                "vcamera_server_ip and vcamera_server_port must be provided for experiment recording"
        self.exp_dir = osp.join(output_dir, 'exp_data') if output_dir is not None else None
        self.vcamera_server_ip = vcamera_server_ip
        self.vcamera_server_port = vcamera_server_port
        self.predicted_full_tcp_action_buffer = RingBuffer(size=1024, fps=max_fps)
        self.predicted_full_gripper_action_buffer = RingBuffer(size=1024, fps=max_fps)
        self.predicted_partial_tcp_action_buffer = RingBuffer(size=1024, fps=max_fps)
        self.predicted_partial_gripper_action_buffer = RingBuffer(size=1024, fps=max_fps)
        self.sensor_msg_list: SensorMessageList = SensorMessageList(sensorMessages=[])

        logger.debug("Initializing RealEnv node...")
        # Get device to topic mapping
        response = requests.get(f"http://{device_mapping_server_ip}:{device_mapping_server_port}/get_mapping")
        self.device_to_topic_mapping = DeviceToTopic.model_validate(response.json())

        subs_name_type = get_topic_and_type(self.device_to_topic_mapping)
        depth_camera_point_cloud_topic_names: List[Optional[str]] = [None, None, None]  # external, left wrist, right wrist
        depth_camera_rgb_topic_names: List[Optional[str]] = [None, None, None]  # external, left wrist, right wrist
        tactile_camera_rgb_topic_names: List[Optional[str]] = [None, None, None, None]  # left gripper1, left gripper2, right gripper1, right gripper2
        tactile_camera_marker_topic_names: List[Optional[str]] = [None, None, None, None]  # left gripper1, left gripper2, right gripper1, right gripper2

        for topic, msg_type in subs_name_type:
            if "depth/points" in topic:
                if "external_camera" in topic:
                    depth_camera_point_cloud_topic_names[0] = topic
                elif "left_wrist_camera" in topic:
                    depth_camera_point_cloud_topic_names[1] = topic
                elif "right_wrist_camera" in topic:
                    depth_camera_point_cloud_topic_names[2] = topic
            elif "color/image_raw" in topic:
                if "gripper_camera" in topic:
                    if "left_gripper_camera_1" in topic:
                        tactile_camera_rgb_topic_names[0] = topic
                    elif "left_gripper_camera_2" in topic:
                        tactile_camera_rgb_topic_names[1] = topic
                    elif "right_gripper_camera_1" in topic:
                        tactile_camera_rgb_topic_names[2] = topic
                    elif "right_gripper_camera_2" in topic:
                        tactile_camera_rgb_topic_names[3] = topic
                else:
                    if "external_camera" in topic:
                        depth_camera_rgb_topic_names[0] = topic
                    elif "left_wrist_camera" in topic:
                        depth_camera_rgb_topic_names[1] = topic
                    elif "right_wrist_camera" in topic:
                        depth_camera_rgb_topic_names[2] = topic
            elif "marker_offset/information" in topic:
                if "left_gripper_camera_1" in topic:
                    tactile_camera_marker_topic_names[0] = topic
                elif "left_gripper_camera_2" in topic:
                    tactile_camera_marker_topic_names[1] = topic
                elif "right_gripper_camera_1" in topic:
                    tactile_camera_marker_topic_names[2] = topic
                elif "right_gripper_camera_2" in topic:
                    tactile_camera_marker_topic_names[3] = topic

        self.time_check = time_check
        self.timestamps = {name: [] for name, _ in get_topic_and_type(self.device_to_topic_mapping)}
        # for calculating FPS
        self.prev_time = time.time()
        self.frame_count = 0

        if self.debug:
            logger.debug(f"Depth camera point cloud topic names: {depth_camera_point_cloud_topic_names}")
            logger.debug(f"Depth camera rgb topic names: {depth_camera_rgb_topic_names}")
            logger.debug(f"Tactile camera rgb topic names: {tactile_camera_rgb_topic_names}")
            logger.debug(f"Tactile camera marker topic names: {tactile_camera_marker_topic_names}")

        self.data_converter = ROS2DataConverter(
            self.transforms,
            depth_camera_point_cloud_topic_names,
            depth_camera_rgb_topic_names,
            tactile_camera_rgb_topic_names,
            tactile_camera_marker_topic_names,
            debug=self.debug
        )

        for name, msg_type in subs_name_type:
            self.subscribers.append(Subscriber(self, msg_type, name))
            logger.debug(f"Subscribed to topic: {name} with type: {msg_type}")

        # ApproximateTimeSynchronizer is used to synchronize multiple topics
        self.ts = ApproximateTimeSynchronizer(self.subscribers, queue_size=40, slop=0.4, allow_headerless=False)
        self.ts.registerCallback(self.callback)

        # Create a session with robot server
        self.session = requests.session()

    def send_command(self, endpoint: str, data: dict = None):
        url = f"http://{self.robot_server_ip}:{self.robot_server_port}{endpoint}"
        if 'get' in endpoint:
            response = self.session.get(url)
        else:
            if 'move' in endpoint:
                # low-level control commands
                try:
                    response = self.session.post(url, json=data, timeout=0.001)
                except requests.exceptions.ReadTimeout:
                    response = None
            else:
                response = self.session.post(url, json=data)
        if response is not None:
            response.raise_for_status()
            return response.json()
        else:
            return dict()

    # @pyinstrument.profile()
    def callback(self, *msgs):
        topic_dict = dict()
        for i, msg in enumerate(msgs):
            topic_name = self.subscribers[i].topic
            topic_dict[topic_name] = msg

        if self.time_check:
            for i, msg in enumerate(msgs):
                topic_name = self.subscribers[i].topic
                self.timestamps[topic_name].append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

        if self.debug:
            latest_timestamp = max([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 for msg in msgs])
            current_timestamp = convert_ros_time_to_float(self.get_clock().now())
            latency = current_timestamp - latest_timestamp
            logger.debug(f"Latency for time synchronizer: {latency:.4f} seconds")

        sensor_msg: SensorMessage = self.data_converter.convert_all_data(topic_dict)
        raw_obs_dict = self.data_processing_manager.convert_sensor_msg_to_obs_dict(sensor_msg)

        # ---- print keys once (helps confirm wrench key name) ----
        if (not self._printed_raw_obs_keys):
            try:
                logger.info(f"[DBG] raw_obs_dict keys: {list(raw_obs_dict.keys())}")
            except Exception:
                pass
            self._printed_raw_obs_keys = True

        # ---- update fast-path cache (low-dim only) ----
        # NOTE: do NOT call get_obs() here; just read wrench from raw_obs_dict
        try:
            lw = raw_obs_dict.get("left_robot_tcp_wrench", None)
            if lw is not None:
                lw = np.asarray(lw).reshape(-1)[:6].astype(np.float32)
            with self.mutex:
                self.latest_left_robot_tcp_wrench = lw
        except Exception:
            pass

        self.obs_buffer.push(raw_obs_dict)

        # record experiment data
        if self.enable_exp_recording:
            recorded_sensor_msg = deepcopy(sensor_msg)

            vcamera_image = self.get_vcamera_image()
            recorded_sensor_msg.vCameraImage = vcamera_image

            predicted_full_tcp_action, exception = self.predicted_full_tcp_action_buffer.peek_last_n(1)
            predicted_full_gripper_action, exception = self.predicted_full_gripper_action_buffer.peek_last_n(1)
            predicted_partial_tcp_action, exception = self.predicted_partial_tcp_action_buffer.peek_last_n(1)
            predicted_partial_gripper_action, exception = self.predicted_partial_gripper_action_buffer.peek_last_n(1)

            recorded_sensor_msg.predictedFullTCPAction = predicted_full_tcp_action[0] if len(predicted_full_tcp_action) == 1 else []
            recorded_sensor_msg.predictedFullGipperAction = predicted_full_gripper_action[0] if len(predicted_full_gripper_action) == 1 else []
            recorded_sensor_msg.predictedPartialTCPAction = predicted_partial_tcp_action[0] if len(predicted_partial_tcp_action) == 1 else []
            recorded_sensor_msg.predictedPartialGipperAction = predicted_partial_gripper_action[0] if len(predicted_partial_gripper_action) == 1 else []

            with self.mutex:
                self.sensor_msg_list.sensorMessages.append(recorded_sensor_msg)

        # calculate fps
        self.frame_count += 1
        current_time = time.time()
        elapsed_time = current_time - self.prev_time
        if elapsed_time >= 1.0:
            frame_rate = self.frame_count / elapsed_time
            self.prev_time = current_time
            self.frame_count = 0
            if self.time_check:
                logger.debug(f"Frame rate: {frame_rate:.2f} FPS")
                self.check_sync()
                self.check_timestamp()

    def check_sync(self):
        all_times = list(self.timestamps.values())
        if not all(all_times):
            return
        for i in range(len(all_times[0])):
            max_diff = 0
            for j in range(len(all_times)):
                for k in range(j + 1, len(all_times)):
                    if i < len(all_times[j]) and i < len(all_times[k]):
                        time_diff = abs(all_times[j][i] - all_times[k][i])
                        max_diff = max(max_diff, time_diff)
            logger.info(f"Frame {i}: Maximum time difference across topics: {max_diff:.6f} seconds")

    def check_timestamp(self):
        all_times = list(self.timestamps.values())
        if not all(all_times):
            return

        time_stamps = []
        for i in range(len(all_times[0])):
            timestamps_for_frame = []
            for j in range(len(all_times)):
                if i < len(all_times[j]):
                    timestamp = all_times[j][i]
                    timestamps_for_frame.append(timestamp)

            if timestamps_for_frame:
                mean_time_stamp = sum(timestamps_for_frame) / len(timestamps_for_frame)
                time_stamps.append(mean_time_stamp)
                logger.info(f"Frame {i}: Mean timestamp: {mean_time_stamp:.6f} seconds")

    def reset(self) -> None:
        self.start_gripper_interval_control = False
        self.obs_buffer.reset()

        # ---- IMPORTANT: clear fast-path cache per episode ----
        with self.mutex:
            self.latest_left_robot_tcp_wrench = None

        if self.enable_exp_recording:
            self.sensor_msg_list.sensorMessages = []
            self.predicted_full_tcp_action_buffer.reset()
            self.predicted_full_gripper_action_buffer.reset()
            self.predicted_partial_tcp_action_buffer.reset()
            self.predicted_partial_gripper_action_buffer.reset()

    # @pyinstrument.profile()
    def get_obs(self, obs_steps: int = 2, temporal_downsample_ratio: int = 2) -> Dict[str, np.ndarray]:
        """
        Get observations with temporal downsampling support.

        Args:
            obs_steps: The number of observations to stack.
            temporal_downsample_ratio: The ratio for temporal downsampling.
                For example, if ratio=2, it will sample every other observation.
        Returns:
            A dictionary containing stacked observations
        """
        last_n_obs_list, _ = self.obs_buffer.peek_last_n(obs_steps * temporal_downsample_ratio)

        result = dict()
        last_n_obs_list = [obs for obs in last_n_obs_list if obs is not None]
        if len(last_n_obs_list) == 0:
            return result

        # ---- IMPORTANT: make ordering robust using timestamp ----
        # We want last_n_obs_list to be "newest -> oldest" before applying [::ratio]
        try:
            ts = []
            for obs in last_n_obs_list:
                if "timestamp" in obs:
                    ts.append(float(np.asarray(obs["timestamp"]).reshape(-1)[0]))
            if len(ts) >= 2:
                # if timestamps increase along list => list is oldest->newest, reverse it
                if ts[0] < ts[-1]:
                    last_n_obs_list = last_n_obs_list[::-1]
        except Exception:
            pass

        # Apply temporal downsampling from newest backward
        downsampled_obs_list = last_n_obs_list[::temporal_downsample_ratio]
        downsampled_obs_list = downsampled_obs_list[:obs_steps]

        # reverse the order to oldest to newest
        downsampled_obs_list = downsampled_obs_list[::-1]

        for key in downsampled_obs_list[0].keys():
            result[key] = stack_last_n_obs([obs[key] for obs in downsampled_obs_list], obs_steps)

        current_timestamp = convert_ros_time_to_float(self.get_clock().now())
        latency = current_timestamp - downsampled_obs_list[-1]['timestamp'][0]
        logger.debug(f"Overall latency for get_obs() : {latency:.4f} seconds")

        return result

    def send_gripper_command_direct(self, left_gripper_width_target: float, right_gripper_width_target: float):
        self.send_command('/move_gripper/left', {
            'width': left_gripper_width_target,
            'velocity': 0.1,
            'force_limit': self.grasp_force
        })
        self.last_gripper_width_target[0] = left_gripper_width_target

        self.send_command('/move_gripper/right', {
            'width': right_gripper_width_target,
            'velocity': 0.1,
            'force_limit': self.grasp_force
        })
        self.last_gripper_width_target[1] = right_gripper_width_target

    def send_gripper_command(self, left_gripper_width_target: float, right_gripper_width_target: float, is_bimanual: bool = False):
        if self.enable_gripper_interval_control and self.start_gripper_interval_control:
            self.gripper_interval_count += 1
            if self.gripper_interval_count % self.gripper_control_time_interval == 0:
                self.gripper_interval_count = 0
            if self.gripper_interval_count != 0:
                return

        if self.enable_gripper_width_clipping:
            if left_gripper_width_target < self.gripper_width_threshold:
                left_gripper_width_target = self.min_gripper_width
                self.start_gripper_interval_control = True
            if is_bimanual:
                if right_gripper_width_target < self.gripper_width_threshold:
                    right_gripper_width_target = self.min_gripper_width
                    self.start_gripper_interval_control = True
        else:
            self.start_gripper_interval_control = True

        robot_states = BimanualRobotStates.model_validate(self.send_command('/get_current_robot_states'))

        grasp_force = self.grasp_force
        gripper_control_width_precision = self.gripper_control_width_precision
        left_current_width = robot_states.leftGripperState[0]
        # print("left_gripper_width_target", left_gripper_width_target)
        # print("gripper_control_width_precision", gripper_control_width_precision)
        # print("enable_gripper_interval_control", self.enable_gripper_interval_control)
        # print("left_current_width", left_current_width)

        if abs(self.last_gripper_width_target[0] - left_gripper_width_target) >= gripper_control_width_precision:
            if self.use_force_control_for_gripper and self.last_gripper_width_target[0] > left_gripper_width_target:
                logger.debug(f"close left gripper moving from {left_current_width} to target: {left_gripper_width_target} "
                             f"with force {grasp_force}")
                self.send_command('/move_gripper/left', {
                    'width': left_gripper_width_target,
                    'velocity': 0.1,
                    'force_limit': grasp_force
                })
            else:
                logger.debug(f"oepn left gripper moving from {left_current_width} to target: {left_gripper_width_target}")
                self.send_command('/move_gripper/left', {
                    'width': left_gripper_width_target,
                    'velocity': 0.1,
                    'force_limit': grasp_force
                })
            self.last_gripper_width_target[0] = left_gripper_width_target

        if is_bimanual:
            right_current_width = robot_states.rightGripperState[0]
            if abs(self.last_gripper_width_target[1] - right_gripper_width_target) >= gripper_control_width_precision:
                if self.use_force_control_for_gripper and self.last_gripper_width_target[1] > right_gripper_width_target:
                    logger.debug(f"right gripper moving from {right_current_width} to target: {right_gripper_width_target} "
                                 f"with force {grasp_force}")
                    self.send_command('/move_gripper_force/right', {
                        'force_limit': grasp_force
                    })
                else:
                    logger.debug(f"right gripper moving from {right_current_width} to target: {right_gripper_width_target}")
                    self.send_command('/move_gripper/right', {
                        'width': right_gripper_width_target,
                        'velocity': 0.1,
                        'force_limit': grasp_force
                    })
                self.last_gripper_width_target[1] = right_gripper_width_target

    def execute_action(self, action: np.ndarray, use_relative_action: bool = False, is_bimanual: bool = False) -> None:
        left_action = action[:8]
        right_action = action[8:]

        if use_relative_action:
            raise NotImplementedError
        else:
            left_gripper_width_target = float(left_action[-2])
            right_gripper_width_target = float(right_action[-2])
        self.send_gripper_command(left_gripper_width_target, right_gripper_width_target, is_bimanual=is_bimanual)

        if use_relative_action:
            raise NotImplementedError
        else:
            left_tcp_target_6d_in_robot = left_action[:6]
            right_tcp_target_6d_in_robot = right_action[:6]
        left_tcp_target_7d_in_robot = pose_6d_to_pose_7d(left_tcp_target_6d_in_robot)
        right_tcp_target_7d_in_robot = pose_6d_to_pose_7d(right_tcp_target_6d_in_robot)

        self.send_command('/move_tcp/left', {'target_tcp': left_tcp_target_7d_in_robot.tolist()})
        if is_bimanual:
            self.send_command('/move_tcp/right', {'target_tcp': right_tcp_target_7d_in_robot.tolist()})

    def get_vcamera_image(self):
        response = self.session.get(f'http://{self.vcamera_server_ip}:{self.vcamera_server_port}/peek_latest_capture')
        if response.status_code == 200 and len(response.content) != 0:
            img = np.frombuffer(response.content, np.uint8)
            img = cv2.imdecode(img, cv2.IMREAD_COLOR)
            return img
        else:
            logger.warning(f"Failed to get vcamera image, status code: {response.status_code}")
            return []

    def get_predicted_action(self, action: np.ndarray, type):
        if self.enable_exp_recording:
            if type == 'full_tcp':
                self.predicted_full_tcp_action_buffer.push(action)
            elif type == "full_gripper":
                self.predicted_full_gripper_action_buffer.push(action)
            elif type == "partial_tcp":
                self.predicted_partial_tcp_action_buffer.push(action)
            elif type == "partial_gripper":
                self.predicted_partial_gripper_action_buffer.push(action)
            else:
                raise ValueError(f"Unknown action type: {type}")

    def save_exp(self, episode_idx):
        if self.enable_exp_recording:
            logger.debug('Trying to save sensor messages...')
            if not osp.exists(self.exp_dir):
                os.makedirs(self.exp_dir)
            record_path = osp.join(self.exp_dir, f'episode_{episode_idx}.pkl')
            if osp.exists(record_path):
                record_path = ".".join(record_path.split('.')[:-1]) + f'{time.strftime("_%Y%m%d_%H%M%S")}.pkl'
                logger.warning(f'Experiment path already exists, save to {record_path}')
            with open(record_path, 'wb') as f:
                with self.mutex:
                    pickle.dump(self.sensor_msg_list, f)
            logger.debug(f'Saved experiment record to {record_path}')
