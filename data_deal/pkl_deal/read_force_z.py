import sys
import os
import pickle
import matplotlib.pyplot as plt
import numpy as np

# ================= 配置区域 =================
project_root = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s'
file_path = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/wiping_board/seq_00002.pkl'
# ===========================================

sys.path.append(project_root)

def plot_tcp_force_z():
    if not os.path.exists(file_path):
        print(f"文件不存在: {file_path}")
        return

    print("正在加载数据...")
    with open(file_path, 'rb') as f:
        data = pickle.load(f)

    msgs = data.sensorMessages
    timestamps = [m.timestamp for m in msgs]
    t = np.array(timestamps) - timestamps[0]

    # 提取 Wrench 的第3维 (索引为2)，通常是 Fz
    # leftRobotTCPWrench shape (6,) -> [Fx, Fy, Fz, Tx, Ty, Tz]
    forces_z = np.array([m.leftRobotTCPWrench[2] for m in msgs])

    print(f"提取完成，数据点数量: {len(forces_z)}")

    # 绘图
    plt.figure(figsize=(12, 5))
    plt.plot(t, forces_z, color='purple', linewidth=1.2)

    plt.title('Left Robot TCP Force (Z-axis)')
    plt.xlabel('Time (s)')
    plt.ylabel('Force (N)')
    plt.grid(True)
    
    # 可以在图上标出最大最小力
    max_f = np.max(forces_z)
    min_f = np.min(forces_z)
    plt.axhline(y=max_f, color='r', linestyle=':', alpha=0.5, label=f'Max: {max_f:.2f}')
    plt.axhline(y=min_f, color='b', linestyle=':', alpha=0.5, label=f'Min: {min_f:.2f}')
    plt.legend()

    output_file = 'check_force_z.png'
    plt.savefig(output_file)
    print(f"图表已保存为: {output_file}")
    plt.show()

if __name__ == "__main__":
    plot_tcp_force_z()