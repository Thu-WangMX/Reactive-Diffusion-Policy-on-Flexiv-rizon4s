"""
Convert your RDP-style multi-episode zarr dataset to a LeRobot dataset
that can be used by openpi.

Usage:
    python convert_rdp_zarr_to_lerobot.py \
        --zarr_path /path/to/replay_buffer.zarr \
        --repo_name your_hf_username/flexiv_rdp \
        --fps 10 \
        [--push_to_hub]

依赖：
    pip install "lerobot>=0.3.0" zarr tyro
"""

import shutil
from pathlib import Path
from typing import List

import numpy as np
import tyro
import zarr
from lerobot.common.datasets.lerobot_dataset import (
    HF_LEROBOT_HOME,
    LeRobotDataset,
)


def build_state(root, idx: int) -> np.ndarray:
    """
    把各种机器人状态拼成一个 state 向量：
    [gripper_width(1),
     q(7),
     tau(7),
     tau_ext(7),
     tcp_pose(9),
     tcp_vel(6),
     tcp_wrench(6)] = 49 维
    """
    parts = [
        root["left_robot_gripper_width"][idx],  # (1,)
        root["left_robot_q"][idx],              # (7,)
        root["left_robot_tau"][idx],            # (7,)
        root["left_robot_tau_ext"][idx],        # (7,)
        root["left_robot_tcp_pose"][idx],       # (9,)
        root["left_robot_tcp_vel"][idx],        # (6,)
        root["left_robot_tcp_wrench"][idx],     # (6,)
    ]
    state = np.concatenate(parts, axis=-1).astype("float32")
    return state


def get_episode_bounds(episode_ends: np.ndarray, total_steps: int) -> List[tuple]:
    """
    根据 episode_ends (长度 E) 生成每个 episode 的 [start, end) 区间。
    第 0 个 episode：0 ~ episode_ends[0]
    第 i 个 episode：episode_ends[i-1] ~ episode_ends[i]
    """
    assert episode_ends.ndim == 1
    assert episode_ends[-1] == total_steps, (
        f"episode_ends[-1] = {episode_ends[-1]} "
        f"but total_steps = {total_steps}"
    )

    bounds = []
    prev = 0
    for end in episode_ends:
        bounds.append((prev, int(end)))
        prev = int(end)
    return bounds


def main(
    zarr_path: str,
    *,
    repo_name: str = "your_hf_username/flexiv_rdp",
    fps: int = 10,
    push_to_hub: bool = False,
) -> None:
    # 1. 打开 zarr
    root = zarr.open(zarr_path, mode="r")

    num_steps = root["action"].shape[0]
    print(f"Loaded zarr with {num_steps} steps")

    # 读 episode_ends
    if "episode_ends" not in root:
        raise ValueError("zarr 中没有 'episode_ends' 键")
    episode_ends = root["episode_ends"][:].astype(np.int64)
    print(f"Found {len(episode_ends)} episodes")
    episode_bounds = get_episode_bounds(episode_ends, num_steps)

    # 图像大小
    H, W, C = root["external_img"].shape[1:]
    assert root["left_wrist_img"].shape[1:] == (H, W, C)

    # state / action 维度
    state_dim = (
        root["left_robot_gripper_width"].shape[1]
        + root["left_robot_q"].shape[1]
        + root["left_robot_tau"].shape[1]
        + root["left_robot_tau_ext"].shape[1]
        + root["left_robot_tcp_pose"].shape[1]
        + root["left_robot_tcp_vel"].shape[1]
        + root["left_robot_tcp_wrench"].shape[1]
    )
    action_dim = root["action"].shape[1]

    print(f"State dim = {state_dim}, action dim = {action_dim}")

    # 2. 准备输出目录： HF_LEROBOT_HOME/repo_name
    output_path: Path = HF_LEROBOT_HOME / repo_name
    if output_path.exists():
        print(f"Removing existing dataset at {output_path}")
        shutil.rmtree(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # 3. 创建 LeRobotDataset
    dataset = LeRobotDataset.create(
        repo_id=repo_name,
        robot_type="flexiv",  # 你平台的名字，随便起个标识
        fps=fps,
        features={
            "image": {
                "dtype": "image",
                "shape": (H, W, C),
                "names": ["height", "width", "channel"],
            },
            "wrist_image": {
                "dtype": "image",
                "shape": (H, W, C),
                "names": ["height", "width", "channel"],
            },
            "state": {
                "dtype": "float32",
                "shape": (state_dim,),
                "names": ["state"],
            },
            "actions": {
                "dtype": "float32",
                "shape": (action_dim,),
                "names": ["actions"],
            },
            # 如果未来你想要语言任务，可以在这里额外定义 "task" 这种文本字段，
            # 目前 LeRobotDataset 会自动处理字符串，不定义也可以直接 add_frame 里放一个 "task"。
        },
        image_writer_threads=10,
        image_writer_processes=5,
    )

    # 4. 遍历每个 episode
    for epi_idx, (start, end) in enumerate(episode_bounds):
        print(f"Episode {epi_idx}: [{start}, {end}) length = {end - start}")
        for t in range(start, end):
            frame = {
                "image":        root["external_img"][t],
                "wrist_image":  root["left_wrist_img"][t],
                "state":        build_state(root, t),
                "actions":      root["action"][t].astype("float32"),
                # 简单起见先给一个固定任务文本，后面你可以按 task id 改成不同 prompt
                "task": "do the task",
            }
            dataset.add_frame(frame)

        # 一个 episode 结束
        dataset.save_episode()

    dataset.flush()
    print(f"Saved LeRobot dataset to {output_path}")

    # 5. 可选：推到 Hugging Face Hub
    if push_to_hub:
        dataset.push_to_hub(
            tags=["flexiv", "rdp", "zarr"],
            private=False,
            push_videos=True,
            license="apache-2.0",
        )
        print(f"Pushed dataset to Hub as {repo_name}")


if __name__ == "__main__":
    tyro.cli(main)
