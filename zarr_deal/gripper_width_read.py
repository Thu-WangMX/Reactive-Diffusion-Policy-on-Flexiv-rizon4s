import zarr
import numpy as np

zarr_path = "/home/wmx/myspace/RDP/data/plug_in_stream_downsample1_zarr/replay_buffer.zarr/meta"
root = zarr.open(zarr_path, mode="r")
episode_ends = root["episode_ends"][:]       # shape (50,)

zarr_path = "/home/wmx/myspace/RDP/data/plug_in_stream_downsample1_zarr/replay_buffer.zarr/data"
root = zarr.open(zarr_path, mode="r")
gripper_width = root["left_robot_gripper_width"][:]  # shape (28273, 1)

N = 500  # 想看最后几帧就改这里，比如 5 或 10

prev_end = 0
for epi_idx, end in enumerate(episode_ends):
    start = prev_end           # 本 episode 起始 index（包含）
    epi_end = int(end)         # 本 episode 结束 index（不包含）
    
    # 这个 episode 的最后 N 帧区间
    last_start = max(start, epi_end - N)
    last_widths = gripper_width[last_start:epi_end, 0]  # 取出 (<=N,) 并去掉最后一维

    print(f"Episode {epi_idx}: indices [{start}, {epi_end})")
    print(f"  last {len(last_widths)} widths:", last_widths)
    print(f"  min={last_widths.min():.4f}, max={last_widths.max():.4f}, mean={last_widths.mean():.4f}")
    print("-" * 50)

    prev_end = epi_end
