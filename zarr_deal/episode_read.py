import zarr
import numpy as np

zarr_path = "/home/wmx/myspace/RDP/data/plug_in_stream_downsample1_zarr/replay_buffer.zarr"

# 打开 zarr 根
root = zarr.open(zarr_path, mode="r")

# 两种等价写法，任选一个：
# episode_ends = root["meta"]["episode_ends"]
episode_ends = root["meta/episode_ends"]   # 推荐这种

print("episode_ends shape:", episode_ends.shape, "dtype:", episode_ends.dtype)

# 转成 numpy 数组
episode_ends_np = np.array(episode_ends)
print("episode_ends:", episode_ends_np)

# # 如果只想看前几个：
# print("first 20 episode_ends:", episode_ends_np[:50])
