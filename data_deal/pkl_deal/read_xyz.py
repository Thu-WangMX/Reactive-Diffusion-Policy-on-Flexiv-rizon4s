import sys
import os
import pickle
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# ================= 配置区域 =================
# 项目根目录
project_root = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s'
# 数据文件路径
file_path = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/wiping_board/seq_00001.pkl'
# ===========================================

sys.path.append(project_root)

def plot_3d_data():
    if not os.path.exists(file_path):
        print(f"文件不存在: {file_path}")
        return

    print(f"正在读取: {os.path.basename(file_path)}")
    with open(file_path, 'rb') as f:
        data = pickle.load(f)

    msgs = data.sensorMessages
    timestamps = [m.timestamp for m in msgs]
    t = np.array(timestamps) - timestamps[0]

    # 1. 提取 TCP 位置 (X, Y, Z)
    # leftRobotTCP: [x, y, z, rx, ry, rz]
    pos = np.array([m.leftRobotTCP[:3] for m in msgs])
    x, y, z = pos[:, 0], pos[:, 1], pos[:, 2]

    # 2. 提取 Wrench Z (Fz)
    # leftRobotTCPWrench: [fx, fy, fz, tx, ty, tz]
    force_z = np.array([m.leftRobotTCPWrench[2] for m in msgs])

    # ================= 绘图开始 =================
    fig = plt.figure(figsize=(16, 8))

    # --- 子图 1: 3D 空间轨迹 ---
    ax1 = fig.add_subplot(121, projection='3d')
    
    # 使用散点图实现颜色渐变 (代表时间进程)
    # c=t (时间), cmap='viridis' (蓝->绿->黄)
    scatter = ax1.scatter(x, y, z, c=t, cmap='jet', s=10, label='Trajectory')
    
    # 连线 (让路径更清晰)
    ax1.plot(x, y, z, color='gray', alpha=0.3, linewidth=1)

    # 标记起点和终点
    ax1.scatter(x[0], y[0], z[0], color='green', s=100, marker='o', label='Start')
    ax1.scatter(x[-1], y[-1], z[-1], color='red', s=150, marker='*', label='End')

    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Spatial Trajectory (Color=Time)')
    ax1.legend()
    
    # 添加颜色条
    cbar = plt.colorbar(scatter, ax=ax1, shrink=0.5)
    cbar.set_label('Time (s)')

    # --- 子图 2: Wrench Z (受力) ---
    ax2 = fig.add_subplot(122)
    ax2.plot(t, force_z, color='purple', linewidth=1.5)
    ax2.set_title('Force Z-axis (Over Time)')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Force (N)')
    ax2.grid(True)
    
    # 标出最大受力点
    max_idx = np.argmax(np.abs(force_z))
    ax2.scatter(t[max_idx], force_z[max_idx], color='red', zorder=5)
    ax2.text(t[max_idx], force_z[max_idx], f' Peak: {force_z[max_idx]:.2f}N', 
             verticalalignment='bottom')

    # 保存并显示
    output_file = 'check_3d_trajectory.png'
    plt.savefig(output_file)
    print(f"图表已保存为: {output_file}")
    plt.tight_layout()
    
    # 注意: 如果你是远程连接 SSH，可能弹不出窗口，请查看生成的 png 图片
    # 或者把下面这行注释掉
    plt.show()

if __name__ == "__main__":
    plot_3d_data()