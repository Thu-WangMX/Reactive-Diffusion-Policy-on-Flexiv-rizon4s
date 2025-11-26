import zarr
import numpy as np

# 路径按你的实际情况改
zarr_path = "/home/wmx/myspace/RDP/data/plug_in_downsample1_zarr/replay_buffer.zarr/data"

# 打开 zarr（只读）
root = zarr.open(zarr_path, mode="r")

# 看看有哪些字段
print("Keys in replay_buffer.zarr:")
print(list(root.array_keys()))
print()

# 只取后 N 帧
N = 1  # 想多看就改大一点

# 安全起见，防止 N > 总长度
T = root["timestamp"].shape[0]
N = min(N, T)

# 读取后 N 帧
left_wrist_img        = root["left_wrist_img"][-N:]         # (N, H, W, 3)
external_img          = root["external_img"][-N:]           # (N, H, W, 3)
left_tcp_pose         = root["left_robot_tcp_pose"][-N:]    # (N, 9)
left_tcp_wrench       = root["left_robot_tcp_wrench"][-N:]  # (N, 6)
left_q                = root["left_robot_q"][-N:]           # (N, J)
left_tau              = root["left_robot_tau"][-N:]         # (N, J)
gripper_width         = root["left_robot_gripper_width"][-N:] 
# left_q                = root["left_robot_q"][:N]           # (N, J)
# left_tau              = root["left_robot_tau"][:N]        # (N, J)
# 打印基本信息
print(f"left_wrist_img shape: {left_wrist_img.shape}, dtype: {left_wrist_img.dtype}")
print(f"external_img shape: {external_img.shape}, dtype: {external_img.dtype}")
print(f"left_robot_tcp_pose shape: {left_tcp_pose.shape}, dtype: {left_tcp_pose.dtype}")
print(f"left_robot_tcp_wrench shape: {left_tcp_wrench.shape}, dtype: {left_tcp_wrench.dtype}")
print(f"left_robot_q shape: {left_q.shape}, dtype: {left_q.dtype}")
print(f"left_robot_tau shape: {left_tau.shape}, dtype: {left_tau.dtype}")
print(f"gripper_width  shape: {gripper_width.shape}, dtype: {gripper_width.dtype}")
print()

# 为了打印更好看一点，设置 numpy 打印选项
np.set_printoptions(precision=4, suppress=True, linewidth=120)

# 逐帧打印
for i in range(N):
    print(f"===== Frame {T - N + i} (index {i} in this slice) =====")

    # 打印位姿向量（9维：3个位置 + 6个旋转）
    print("left_robot_tcp_pose:", left_tcp_pose[i])
    print("left_robot_tcp_wrench:", left_tcp_wrench[i])
    print("left_robot_q:", left_q[i])
    print("left_robot_tau:", left_tau[i])
    print("gripper_width:", gripper_width [i])

    # 图像打印统计信息
    wrist = left_wrist_img[i]
    ext   = external_img[i]

    print("left_wrist_img stats:")
    print("  shape:", wrist.shape,
          "min:", wrist.min(), "max:", wrist.max(),
          "mean:", wrist.mean(), "std:", wrist.std())

    print("external_img stats:")
    print("  shape:", ext.shape,
          "min:", ext.min(), "max:", ext.max(),
          "mean:", ext.mean(), "std:", ext.std())

    print()
