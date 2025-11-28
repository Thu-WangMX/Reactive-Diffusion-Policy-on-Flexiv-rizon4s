import os
import os.path as osp
import pickle

import cv2
import matplotlib.pyplot as plt
import numpy as np
import py_cli_interaction
import zarr
from hydra import initialize, compose
from loguru import logger
from omegaconf import DictConfig

from reactive_diffusion_policy.real_world.real_world_transforms import RealWorldTransforms
from reactive_diffusion_policy.common.visualization_utils import visualize_rgb_image
from reactive_diffusion_policy.real_world.post_process_utils import DataPostProcessingManager

# ================== 配置 ==================
DEBUG = False
DEBUG_TACTILE_LATENCY = False  # 现在脚本不再保留全局数组，只在本地 episode 里可视化
DRAW_FORCE = False             # 同上

USE_ABSOLUTE_ACTION = True

TAG = 'plug_in'  # 任务名称，对应 data 目录名
ACTION_DIM = 10  # 我们现在只支持 4 或 10，其他 raise
TEMPORAL_DOWNSAMPLE_RATIO = 1  # 每条 episode 内的时间下采样比例

SENSOR_MODE = 'single_arm_two_realsense_two_tactile'
GELSIGHT_PCA_PATH = 'data/PCA_Transform_GelSight'
MCTAC_PCA_PATH = 'data/PCA_Transform_McTAC_v1'
PCA_DIM = 15


def downsample_indices_per_episode(length: int, ratio: int):
    """
    给定一条 episode 的长度 length 和下采样比例 ratio，
    返回保留下来的全局索引（相对当前 episode 起点）。
    保留首尾，中间按 step=ratio 取样。
    """
    if length <= 2 or ratio <= 1:
        return np.arange(length, dtype=np.int64)

    idx = np.arange(length, dtype=np.int64)
    middle = idx[1:-1]
    middle_ds = middle[::ratio]
    keep = np.concatenate([[idx[0]], middle_ds, [idx[-1]]], axis=0)
    return keep


if __name__ == '__main__':
    # ========= 路径设置 =========
    data_dir = f'/home/wmx/myspace/RDP/data/{TAG}'
    save_data_dir = f'/home/wmx/myspace/RDP/data/{TAG}_stream_downsample{TEMPORAL_DOWNSAMPLE_RATIO}{"_debug" if DEBUG else ""}_zarr'
    os.makedirs(save_data_dir, exist_ok=True)
    save_data_path = osp.join(osp.abspath(os.getcwd()), save_data_dir, 'replay_buffer.zarr')

    # 已存在则询问是否覆盖
    if os.path.exists(save_data_path):
        logger.info(f'Data already exists at {save_data_path}')
        if py_cli_interaction.parse_cli_bool('Do you want to overwrite the data?', default_value=True):
            logger.warning(f'Overwriting {save_data_path}')
            os.system(f'rm -rf {save_data_path}')
        else:
            logger.info('Abort.')
            exit(0)

    # ========= 加载 transforms & PCA 配置 =========
    with initialize(config_path='reactive_diffusion_policy/config', version_base="1.3"):
        cfg = compose(config_name="real_world_env")
    transforms = RealWorldTransforms(option=cfg.task.transforms)

    gelsight_pca_transform_matrix_path = os.path.join(GELSIGHT_PCA_PATH, 'pca_transform_matrix.npy')
    gelsight_pca_mean_matrix_path = os.path.join(GELSIGHT_PCA_PATH, 'pca_mean_matrix.npy')
    gelsight_pca_param = DictConfig({
        'n_components': PCA_DIM,
        'normalize': False,
        'mode': 'Eval',
        'store': False,
        'transformation_matrix_path': gelsight_pca_transform_matrix_path,
        'mean_matrix_path': gelsight_pca_mean_matrix_path
    })
    mctac_pca_transform_matrix_path = os.path.join(MCTAC_PCA_PATH, 'pca_transform_matrix.npy')
    mctac_pca_mean_matrix_path = os.path.join(MCTAC_PCA_PATH, 'pca_mean_matrix.npy')
    mctac_gelsight_pca_param = DictConfig({
        'n_components': PCA_DIM,
        'normalize': False,
        'mode': 'Eval',
        'store': False,
        'transformation_matrix_path': mctac_pca_transform_matrix_path,
        'mean_matrix_path': mctac_pca_mean_matrix_path
    })
    pca_param_dict = DictConfig({
        'GelSight': gelsight_pca_param,
        'McTac': mctac_gelsight_pca_param
    })

    data_processing_manager = DataPostProcessingManager(
        transforms=transforms,
        pca_param_dict=pca_param_dict,
        mode=SENSOR_MODE,
        use_6d_rotation=True,
        marker_dimension=2
    )

    # ========= 准备 zarr 根节点（可扩展 dataset） =========
    store = zarr.DirectoryStore(save_data_path)
    root = zarr.group(store=store, overwrite=True)
    zarr_data = root.create_group('data')
    zarr_meta = root.create_group('meta')

    # 可扩展 dataset（第一次 episode 时创建）
    ds_timestamp = None
    ds_ext_img = None
    ds_left_wrist_img = None
    ds_left_tcp_pose = None
    ds_left_tcp_vel = None
    ds_left_tcp_wrench = None
    ds_left_gripper_width = None
    ds_left_q = None
    ds_left_tau = None
    ds_left_tau_ext = None
    ds_target = None
    ds_action = None

    compressor = zarr.Blosc(cname='zstd', clevel=3, shuffle=1)

    # 全局计数：当前已经写入 zarr 的总帧数
    total_len = 0
    episode_ends = []

    # ========= 遍历所有 pkl 文件 =========
    data_files = sorted([f for f in os.listdir(data_dir) if f.endswith('.pkl')])
    logger.info(f'Found {len(data_files)} pkl files in {data_dir}')

    for seq_idx, data_file in enumerate(data_files):
        if DEBUG and seq_idx <= 58:
            # 保持和原逻辑一致，debug 时可跳过前面的
            continue

        data_path = osp.join(data_dir, data_file)
        logger.info(f'[{seq_idx+1}/{len(data_files)}] Loading data from {osp.abspath(data_path)}')

        with open(data_path, 'rb') as f:
            data = pickle.load(f)

        # ---------- 局部 episode list ----------
        ts_list = []
        ext_img_list = []
        left_wrist_img_list = []
        left_tcp_pose_list = []
        left_tcp_vel_list = []
        left_tcp_wrench_list = []
        left_gripper_width_list = []
        left_q_list = []
        left_tau_list = []
        left_tau_ext_list = []

        # ---------- 遍历当前 pkl 的所有 sensor_msg ----------
        for step_idx, sensor_msg in enumerate(data.sensorMessages):
            if DEBUG and step_idx <= 60:
                continue

            logger.info(f'  Processing {step_idx}th sensor message in sequence {seq_idx}')

            obs_dict = data_processing_manager.convert_sensor_msg_to_obs_dict(sensor_msg)

            # 时间戳
            ts_list.append(float(sensor_msg.timestamp))

            # 左臂 TCP 状态
            left_tcp_pose_list.append(np.asarray(obs_dict['left_robot_tcp_pose'], dtype=np.float32))
            left_tcp_vel_list.append(np.asarray(obs_dict['left_robot_tcp_vel'], dtype=np.float32))
            left_tcp_wrench_list.append(np.asarray(obs_dict['left_robot_tcp_wrench'], dtype=np.float32))
            left_gripper_width_list.append(np.asarray(obs_dict['left_robot_gripper_width'], dtype=np.float32))

            # 左臂关节量（来自 sensor_msg）
            if hasattr(sensor_msg, "leftRobotQ"):
                left_q_list.append(np.asarray(sensor_msg.leftRobotQ, dtype=np.float32))
            else:
                left_q_list.append(np.zeros(7, dtype=np.float32))

            if hasattr(sensor_msg, "leftRobotTau"):
                left_tau_list.append(np.asarray(sensor_msg.leftRobotTau, dtype=np.float32))
            else:
                left_tau_list.append(np.zeros(7, dtype=np.float32))

            if hasattr(sensor_msg, "leftRobotTauExt"):
                left_tau_ext_list.append(np.asarray(sensor_msg.leftRobotTauExt, dtype=np.float32))
            else:
                left_tau_ext_list.append(np.zeros(7, dtype=np.float32))

            # 图像
            ext_img_list.append(np.asarray(obs_dict['external_img'], dtype=np.uint8))
            left_wrist_img_list.append(np.asarray(obs_dict['left_wrist_img'], dtype=np.uint8))

            if DEBUG:
                # 可视化用，按需打开
                visualize_rgb_image(sensor_msg.externalCameraRGB, 'External Camera RGB')
                visualize_rgb_image(sensor_msg.leftWristCameraRGB, 'Left Wrist Camera RGB')

            # 及时释放引用
            del sensor_msg, obs_dict

        # 如果这一条 pkl（episode）为空，就跳过
        if len(ts_list) == 0:
            logger.warning(f'  Sequence {seq_idx} has no valid sensor messages, skip.')
            continue

        # ---------- 当前 episode：list -> numpy ----------
        ts_arr = np.asarray(ts_list, dtype=np.float32)                    # (T,)
        left_tcp_pose_arr = np.stack(left_tcp_pose_list, axis=0)          # (T, 9)
        left_tcp_vel_arr = np.stack(left_tcp_vel_list, axis=0)            # (T, 6)
        left_tcp_wrench_arr = np.stack(left_tcp_wrench_list, axis=0)      # (T, 6)
        left_gripper_width_arr = np.stack(left_gripper_width_list, axis=0)  # (T, 1)
        left_q_arr = np.stack(left_q_list, axis=0)                        # (T, 7)
        left_tau_arr = np.stack(left_tau_list, axis=0)                    # (T, 7)
        left_tau_ext_arr = np.stack(left_tau_ext_list, axis=0)            # (T, 7)
        ext_img_arr = np.stack(ext_img_list, axis=0)                      # (T, H, W, 3)
        left_wrist_img_arr = np.stack(left_wrist_img_list, axis=0)        # (T, H, W, 3)

        T = ts_arr.shape[0]
        logger.info(f'  Sequence {seq_idx} has {T} frames before downsampling.')

        # ---------- 当前 episode：构造 state / action ----------
        if ACTION_DIM == 4:
            # (x,y,z, gripper_width)
            state_arr = np.concatenate(
                [left_tcp_pose_arr[:, :3], left_gripper_width_arr],
                axis=-1
            ).astype(np.float32)
        elif ACTION_DIM == 10:
            # (x,y,z, 6D rotation, gripper_width)  -> 9 + 1 = 10
            state_arr = np.concatenate(
                [left_tcp_pose_arr, left_gripper_width_arr],
                axis=-1
            ).astype(np.float32)
        else:
            raise NotImplementedError(f"ACTION_DIM={ACTION_DIM} not supported in streaming script.")

        if USE_ABSOLUTE_ACTION:
            new_action_arr = state_arr[1:, ...].copy()
            action_arr = np.concatenate(
                [new_action_arr, new_action_arr[-1][np.newaxis, :]],
                axis=0
            ).astype(np.float32)
        else:
            raise NotImplementedError("Only USE_ABSOLUTE_ACTION=True is supported.")

        # ---------- 当前 episode：时间下采样（在本地做） ----------
        if TEMPORAL_DOWNSAMPLE_RATIO > 1:
            keep_idx = downsample_indices_per_episode(T, TEMPORAL_DOWNSAMPLE_RATIO)
            logger.info(f'  Downsample sequence {seq_idx}: {T} -> {len(keep_idx)} frames.')

            ts_arr = ts_arr[keep_idx]
            left_tcp_pose_arr = left_tcp_pose_arr[keep_idx]
            left_tcp_vel_arr = left_tcp_vel_arr[keep_idx]
            left_tcp_wrench_arr = left_tcp_wrench_arr[keep_idx]
            left_gripper_width_arr = left_gripper_width_arr[keep_idx]
            left_q_arr = left_q_arr[keep_idx]
            left_tau_arr = left_tau_arr[keep_idx]
            left_tau_ext_arr = left_tau_ext_arr[keep_idx]
            ext_img_arr = ext_img_arr[keep_idx]
            left_wrist_img_arr = left_wrist_img_arr[keep_idx]
            state_arr = state_arr[keep_idx]
            action_arr = action_arr[keep_idx]

        T_ds = ts_arr.shape[0]

        # ---------- DEBUG: 可选画力/轨迹 ----------
        if DEBUG_TACTILE_LATENCY:
            # 举个例：画 z 方向 tcp pose
            left_tcp_z = -left_tcp_pose_arr[:, 2]
            left_tcp_z_norm = (left_tcp_z - left_tcp_z.min()) / (left_tcp_z.max() - left_tcp_z.min() + 1e-8)
            plt.figure()
            plt.plot(left_tcp_z_norm, label='Left TCP Z (norm)')
            plt.legend()
            plt.title(f'Seq {seq_idx} TCP Z (downsampled)')
            plt.show()

        if DRAW_FORCE:
            # 举个例：画前三个分量的 wrench
            F = left_tcp_wrench_arr[:200, :3]
            plt.figure()
            plt.plot(F[:, 0], label='Fx')
            plt.plot(F[:, 1], label='Fy')
            plt.plot(F[:, 2], label='Fz')
            plt.legend()
            plt.title(f'Seq {seq_idx} force (first 200 frames)')
            plt.show()

        # ---------- 第一次 episode：创建可扩展 dataset ----------
        if ds_timestamp is None:
            logger.info('  Create resizable zarr datasets.')

            # 图像 chunk 大小
            H_ext, W_ext = ext_img_arr.shape[1], ext_img_arr.shape[2]
            H_wrist, W_wrist = left_wrist_img_arr.shape[1], left_wrist_img_arr.shape[2]

            ds_timestamp = zarr_data.create_dataset(
                'timestamp',
                shape=(0,),
                maxshape=(None,),
                chunks=(10000,),
                dtype='float32',
                compressor=compressor,
            )
            ds_left_tcp_pose = zarr_data.create_dataset(
                'left_robot_tcp_pose',
                shape=(0, left_tcp_pose_arr.shape[1]),
                maxshape=(None, left_tcp_pose_arr.shape[1]),
                chunks=(10000, left_tcp_pose_arr.shape[1]),
                dtype='float32',
                compressor=compressor,
            )
            ds_left_tcp_vel = zarr_data.create_dataset(
                'left_robot_tcp_vel',
                shape=(0, left_tcp_vel_arr.shape[1]),
                maxshape=(None, left_tcp_vel_arr.shape[1]),
                chunks=(10000, left_tcp_vel_arr.shape[1]),
                dtype='float32',
                compressor=compressor,
            )
            ds_left_tcp_wrench = zarr_data.create_dataset(
                'left_robot_tcp_wrench',
                shape=(0, left_tcp_wrench_arr.shape[1]),
                maxshape=(None, left_tcp_wrench_arr.shape[1]),
                chunks=(10000, left_tcp_wrench_arr.shape[1]),
                dtype='float32',
                compressor=compressor,
            )
            ds_left_gripper_width = zarr_data.create_dataset(
                'left_robot_gripper_width',
                shape=(0, left_gripper_width_arr.shape[1]),
                maxshape=(None, left_gripper_width_arr.shape[1]),
                chunks=(10000, left_gripper_width_arr.shape[1]),
                dtype='float32',
                compressor=compressor,
            )
            ds_left_q = zarr_data.create_dataset(
                'left_robot_q',
                shape=(0, left_q_arr.shape[1]),
                maxshape=(None, left_q_arr.shape[1]),
                chunks=(10000, left_q_arr.shape[1]),
                dtype='float32',
                compressor=compressor,
            )
            ds_left_tau = zarr_data.create_dataset(
                'left_robot_tau',
                shape=(0, left_tau_arr.shape[1]),
                maxshape=(None, left_tau_arr.shape[1]),
                chunks=(10000, left_tau_arr.shape[1]),
                dtype='float32',
                compressor=compressor,
            )
            ds_left_tau_ext = zarr_data.create_dataset(
                'left_robot_tau_ext',
                shape=(0, left_tau_ext_arr.shape[1]),
                maxshape=(None, left_tau_ext_arr.shape[1]),
                chunks=(10000, left_tau_ext_arr.shape[1]),
                dtype='float32',
                compressor=compressor,
            )
            ds_target = zarr_data.create_dataset(
                'target',
                shape=(0, state_arr.shape[1]),
                maxshape=(None, state_arr.shape[1]),
                chunks=(10000, state_arr.shape[1]),
                dtype='float32',
                compressor=compressor,
            )
            ds_action = zarr_data.create_dataset(
                'action',
                shape=(0, action_arr.shape[1]),
                maxshape=(None, action_arr.shape[1]),
                chunks=(10000, action_arr.shape[1]),
                dtype='float32',
                compressor=compressor,
            )
            ds_ext_img = zarr_data.create_dataset(
                'external_img',
                shape=(0, H_ext, W_ext, 3),
                maxshape=(None, H_ext, W_ext, 3),
                chunks=(100, H_ext, W_ext, 3),
                dtype='uint8',
                compressor=compressor,
            )
            ds_left_wrist_img = zarr_data.create_dataset(
                'left_wrist_img',
                shape=(0, H_wrist, W_wrist, 3),
                maxshape=(None, H_wrist, W_wrist, 3),
                chunks=(100, H_wrist, W_wrist, 3),
                dtype='uint8',
                compressor=compressor,
            )

        # # ---------- 追加当前 episode 数据到 zarr ----------
        # new_total_len = total_len + T_ds

        # ds_timestamp.resize(new_total_len, axis=0)
        # ds_timestamp[total_len:new_total_len] = ts_arr

        # ds_left_tcp_pose.resize(new_total_len, axis=0)
        # ds_left_tcp_pose[total_len:new_total_len, :] = left_tcp_pose_arr

        # ds_left_tcp_vel.resize(new_total_len, axis=0)
        # ds_left_tcp_vel[total_len:new_total_len, :] = left_tcp_vel_arr

        # ds_left_tcp_wrench.resize(new_total_len, axis=0)
        # ds_left_tcp_wrench[total_len:new_total_len, :] = left_tcp_wrench_arr

        # ds_left_gripper_width.resize(new_total_len, axis=0)
        # ds_left_gripper_width[total_len:new_total_len, :] = left_gripper_width_arr

        # ds_left_q.resize(new_total_len, axis=0)
        # ds_left_q[total_len:new_total_len, :] = left_q_arr

        # ds_left_tau.resize(new_total_len, axis=0)
        # ds_left_tau[total_len:new_total_len, :] = left_tau_arr

        # ds_left_tau_ext.resize(new_total_len, axis=0)
        # ds_left_tau_ext[total_len:new_total_len, :] = left_tau_ext_arr

        # ds_target.resize(new_total_len, axis=0)
        # ds_target[total_len:new_total_len, :] = state_arr

        # ds_action.resize(new_total_len, axis=0)
        # ds_action[total_len:new_total_len, :] = action_arr

        # ds_ext_img.resize(new_total_len, axis=0)
        # ds_ext_img[total_len:new_total_len, ...] = ext_img_arr

        # ds_left_wrist_img.resize(new_total_len, axis=0)
        # ds_left_wrist_img[total_len:new_total_len, ...] = left_wrist_img_arr
        
                # ---------- 追加当前 episode 数据到 zarr ----------
        new_total_len = total_len + T_ds

        # 1D
        ds_timestamp.resize((new_total_len,))
        ds_timestamp[total_len:new_total_len] = ts_arr

        # 2D
        ds_left_tcp_pose.resize((new_total_len, left_tcp_pose_arr.shape[1]))
        ds_left_tcp_pose[total_len:new_total_len, :] = left_tcp_pose_arr

        ds_left_tcp_vel.resize((new_total_len, left_tcp_vel_arr.shape[1]))
        ds_left_tcp_vel[total_len:new_total_len, :] = left_tcp_vel_arr

        ds_left_tcp_wrench.resize((new_total_len, left_tcp_wrench_arr.shape[1]))
        ds_left_tcp_wrench[total_len:new_total_len, :] = left_tcp_wrench_arr

        ds_left_gripper_width.resize((new_total_len, left_gripper_width_arr.shape[1]))
        ds_left_gripper_width[total_len:new_total_len, :] = left_gripper_width_arr

        ds_left_q.resize((new_total_len, left_q_arr.shape[1]))
        ds_left_q[total_len:new_total_len, :] = left_q_arr

        ds_left_tau.resize((new_total_len, left_tau_arr.shape[1]))
        ds_left_tau[total_len:new_total_len, :] = left_tau_arr

        ds_left_tau_ext.resize((new_total_len, left_tau_ext_arr.shape[1]))
        ds_left_tau_ext[total_len:new_total_len, :] = left_tau_ext_arr

        ds_target.resize((new_total_len, state_arr.shape[1]))
        ds_target[total_len:new_total_len, :] = state_arr

        ds_action.resize((new_total_len, action_arr.shape[1]))
        ds_action[total_len:new_total_len, :] = action_arr

        # 4D：图像
        ds_ext_img.resize((new_total_len,
                           ext_img_arr.shape[1],
                           ext_img_arr.shape[2],
                           ext_img_arr.shape[3]))
        ds_ext_img[total_len:new_total_len, ...] = ext_img_arr

        ds_left_wrist_img.resize((new_total_len,
                                  left_wrist_img_arr.shape[1],
                                  left_wrist_img_arr.shape[2],
                                  left_wrist_img_arr.shape[3]))
        ds_left_wrist_img[total_len:new_total_len, ...] = left_wrist_img_arr


        total_len = new_total_len
        episode_ends.append(total_len)

        logger.info(f'  Sequence {seq_idx} done. Global total_len = {total_len}')

        # 局部数组用完就让它们掉出作用域，减轻内存
        del (
            ts_arr, left_tcp_pose_arr, left_tcp_vel_arr, left_tcp_wrench_arr,
            left_gripper_width_arr, left_q_arr, left_tau_arr, left_tau_ext_arr,
            ext_img_arr, left_wrist_img_arr, state_arr, action_arr
        )

    # ========= 写 episode_ends =========
    episode_ends_arr = np.asarray(episode_ends, dtype=np.int64)
    zarr_meta.create_dataset(
        'episode_ends',
        data=episode_ends_arr,
        shape=episode_ends_arr.shape,
        chunks=(min(10000, len(episode_ends_arr)),),
        dtype='int64',
        compressor=compressor,
        overwrite=True,
    )

    # ========= 打印结构 =========
    logger.info('Zarr data structure:')
    logger.info(zarr_data.tree())
    logger.info(f'Total frames written: {total_len}')
    logger.info(f'Number of episodes: {len(episode_ends)}')
    logger.info(f'Saved to {save_data_path}')
