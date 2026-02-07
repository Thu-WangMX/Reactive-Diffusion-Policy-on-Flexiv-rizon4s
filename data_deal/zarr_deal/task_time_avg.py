import zarr
import numpy as np
import os

# ================= 配置区域 =================
DATASETS = {
    "Plug-in Charger": "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_charger/plug_in_charger_stream_downsample1_zarr/replay_buffer.zarr",
    "Wiping Board": "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board/wiping_board_stream_downsample1_zarr/replay_buffer.zarr"
}

FPS = 24
# ===========================================

def analyze_zarr_duration(name, path):
    if not os.path.exists(path):
        print(f"【错误】找不到路径: {path}")
        return

    try:
        # 打开 zarr 文件
        root = zarr.open(path, mode='r')
        # 根据你提供的路径，episode_ends 在 meta 下
        episode_ends = root['meta/episode_ends'][:]
        
        # 计算每个 episode 的帧数
        # np.diff 可以快速计算 [ends[0], ends[1]-ends[0], ends[2]-ends[1]...]
        lengths = np.diff(episode_ends, prepend=0)
        
        num_episodes = len(lengths)
        total_frames = episode_ends[-1]
        
        # 计算时间 (秒)
        durations_sec = lengths / FPS
        avg_duration = np.mean(durations_sec)
        max_duration = np.max(durations_sec)
        min_duration = np.min(durations_sec)
        total_time_min = np.sum(durations_sec) / 60
        
        print(f"--- 统计结果: {name} ---")
        print(f"  - 总 Episode 数量: {num_episodes}")
        print(f"  - 总帧数: {total_frames}")
        print(f"  - 总遥操时长: {total_time_min:.2f} 分钟")
        print(f"  - 平均每条时长: {avg_duration:.2f} 秒")
        print(f"  - 最长的一条: {max_duration:.2f} 秒")
        print(f"  - 最短的一条: {min_duration:.2f} 秒")
        print("")

    except Exception as e:
        print(f"【出错】处理 {name} 时发生异常: {e}")

def main():
    print("正在统计数据集遥操时长 (频率: 24Hz)...\n")
    for name, path in DATASETS.items():
        analyze_zarr_duration(name, path)

if __name__ == "__main__":
    main()