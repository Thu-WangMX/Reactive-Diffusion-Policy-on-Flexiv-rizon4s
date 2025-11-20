import zarr
import numpy as np

# 路径按你的实际情况改
zarr_path = "/home/wmx/myspace/RDP/data/test1_downsample1_zarr/replay_buffer.zarr/data"

# 打开 zarr（只读）
root = zarr.open(zarr_path, mode="r")

# 看看有哪些字段
print("Keys in replay_buffer.zarr:")
print(list(root.array_keys()))
print()

# 只取前 N 帧
N = 3  # 想多看就改大一点
left_wrist_img = root["left_wrist_img"][:N]        # (N, H, W, 3)
left_tcp_pose   = root["left_robot_tcp_wrench"][:N]  # (N, 9)

# 打印基本信息
print(f"left_wrist_img shape: {left_wrist_img.shape}, dtype: {left_wrist_img.dtype}")
print(f"left_robot_tcp_pose shape: {left_tcp_pose.shape}, dtype: {left_tcp_pose.dtype}")
print()

# 为了打印更好看一点，设置 numpy 打印选项
np.set_printoptions(precision=4, suppress=True, linewidth=120)

# 逐帧打印
for i in range(N):
    print(f"===== Frame {i} =====")
    # 打印位姿向量（9维：3个位置 + 6个旋转）
    print("left_robot_tcp_pose:", left_tcp_pose[i])

    # 图像整体打印会非常长，这里给你几种常用方式任选：

    # 1) 打印图像的整体统计信息（推荐）
    img = left_wrist_img[i]
    print("left_wrist_img stats:")
    print("  shape:", img.shape)
    print("  min:", img.min(), "max:", img.max(), "mean:", img.mean(), "std:", img.std())
# import zarr
# import matplotlib.pyplot as plt

# zarr_path = "/home/wmx/myspace/RDP/data/test1_downsample1_zarr/replay_buffer.zarr/data"
# root = zarr.open(zarr_path, mode="r")

# print("Keys:", list(root.array_keys()))

# # 取第 k 帧的图像
# k = 0  # 想看第几帧就改这里
# img = root["left_wrist_img"][k]  # shape: (240, 320, 3), uint8

# print("Frame", k, "img shape:", img.shape, "dtype:", img.dtype)

# plt.figure(figsize=(4, 4))
# plt.imshow(img)       # zarr 里是 HWC、RGB 顺序，直接 imshow 即可
# plt.axis("off")
# plt.title(f"left_wrist_img frame {k}")
# plt.show()