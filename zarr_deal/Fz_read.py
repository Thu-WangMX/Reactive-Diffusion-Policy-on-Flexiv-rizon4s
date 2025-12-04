import zarr
import numpy as np
import matplotlib.pyplot as plt

# -------------------------- 1. 读取数据（与修改后代码一致）--------------------------
# 读取 episode_ends（episode 结束索引）
zarr_path_meta = "/home/wmx/myspace/RDP/data/plug_in_stream_downsample1_zarr/replay_buffer.zarr/meta"
root_meta = zarr.open(zarr_path_meta, mode="r")
episode_ends = root_meta["episode_ends"][:]  # shape (50,)，共50个episode

# 读取 TCP 力/扭矩数据，提取 Fz 分量（索引2）
zarr_path_data = "/home/wmx/myspace/RDP/data/plug_in_stream_downsample1_zarr/replay_buffer.zarr/data"
root_data = zarr.open(zarr_path_data, mode="r")
tcp_wrench = root_data["left_robot_tcp_wrench"][:]  # shape (总帧数, 6)：Fx,Fy,Fz,Tx,Ty,Tz
fz_force = tcp_wrench[:, 2]  # 提取所有帧的 Fz 力，shape (总帧数,)

# -------------------------- 2. 查看所有Episode的基础信息（原逻辑保留）--------------------------
N = 500  # 查看每个episode最后N帧的统计信息
prev_end = 0
episode_info = []  # 存储每个episode的关键信息，方便后续选择

print("=" * 60)
print("所有Episode的Fz力统计（最后{}帧）".format(N))
print("=" * 60)

for epi_idx, end in enumerate(episode_ends):
    start = prev_end
    epi_end = int(end)
    episode_length = epi_end - start  # 当前episode的总帧数
    
    # 最后N帧的Fz数据
    last_start = max(start, epi_end - N)
    last_fz = fz_force[last_start:epi_end]
    min_fz = last_fz.min()
    max_fz = last_fz.max()
    mean_fz = last_fz.mean()
    
    # 存储信息（用于后续选择）
    episode_info.append({
        "index": epi_idx,
        "start": start,
        "end": epi_end,
        "length": episode_length,
        "min_fz": min_fz,
        "max_fz": max_fz,
        "mean_fz": mean_fz
    })
    
    # 打印信息
    print(f"Episode {epi_idx}: 索引[{start}, {epi_end}) | 总帧数: {episode_length}")
    print(f"  最后{len(last_fz)}帧Fz: min={min_fz:.4f}N, max={max_fz:.4f}N, mean={mean_fz:.4f}N")
    print("-" * 60)
    
    prev_end = epi_end

# -------------------------- 3. 选择特定Episode并绘制Fz曲线 --------------------------
print("\n" + "=" * 60)
print("绘制特定Episode的Fz力变化曲线")
print("=" * 60)

# 步骤1：让用户选择Episode（支持输入索引或自动推荐）
while True:
    # 显示可选的Episode范围
    max_epi_idx = len(episode_info) - 1
    print(f"可选Episode索引：0 ~ {max_epi_idx}（共{max_epi_idx+1}个）")
    user_input = input(f"请输入要绘制的Episode索引（例如0、{max_epi_idx//2}、{max_epi_idx}）：")
    
    # 验证输入是否有效
    try:
        target_epi_idx = int(user_input)
        if 0 <= target_epi_idx <= max_epi_idx:
            break
        else:
            print(f"输入无效！请输入0到{max_epi_idx}之间的整数。")
    except ValueError:
        print("输入无效！请输入整数索引。")

# 步骤2：提取目标Episode的Fz数据和时间轴
target_info = episode_info[target_epi_idx]
epi_start = target_info["start"]
epi_end = target_info["end"]
epi_fz = fz_force[epi_start:epi_end]  # 目标Episode的所有Fz数据

# 生成时间轴（假设数据采样频率与RDP AT推理频率一致：24 FPS，即每帧间隔≈41.67ms）
fps = 24  # RDP数据集的典型采样频率（可根据实际数据集调整）
time_axis = np.arange(len(epi_fz)) / fps  # 单位：秒

# 步骤3：绘制曲线（美化样式，突出关键信息）
plt.rcParams['font.sans-serif'] = ['Arial']  # 解决中文显示问题（如需中文可改为'SimHei'）
plt.figure(figsize=(12, 6))

# 绘制Fz力曲线
plt.plot(time_axis, -epi_fz, color='#2E86AB', linewidth=1.5, label=f'Episode {target_epi_idx} Fz Force')

# 标注关键统计值（min/max点）
min_idx = np.argmin(epi_fz)
max_idx = np.argmax(epi_fz)
plt.scatter(time_axis[min_idx], epi_fz[min_idx], color='#E63946', s=50, zorder=5)
plt.scatter(time_axis[max_idx], epi_fz[max_idx], color='#2A9D8F', s=50, zorder=5)
plt.annotate(f'Min: {epi_fz[min_idx]:.4f}N', 
             xy=(time_axis[min_idx], epi_fz[min_idx]),
             xytext=(10, -20), textcoords='offset points',
             fontsize=10, color='#E63946')
plt.annotate(f'Max: {epi_fz[max_idx]:.4f}N', 
             xy=(time_axis[max_idx], epi_fz[max_idx]),
             xytext=(10, 10), textcoords='offset points',
             fontsize=10, color='#2A9D8F')

# 添加参考线（0N线，区分拉力/压力）
plt.axhline(y=0, color='#8D99AE', linestyle='--', linewidth=1, label='Zero Force (0N)')
plt.ylim(-20, 100)
# 设置图表标签和标题
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Fz Force (N)', fontsize=12)
plt.title(f'Episode {target_epi_idx} Fz Force Change Over Time\n'
          f'Frame Range: [{epi_start}, {epi_end}) | Total Frames: {len(epi_fz)} | FPS: {fps}',
          fontsize=14, pad=20)

# 网格和图例
plt.grid(True, alpha=0.3, linestyle='-')
plt.legend(loc='best', fontsize=10)

# 调整布局，避免标签被截断
plt.tight_layout()

# 显示图片（如果是远程服务器，可改为保存图片：plt.savefig('episode_fz_curve.png', dpi=300)）
plt.show()