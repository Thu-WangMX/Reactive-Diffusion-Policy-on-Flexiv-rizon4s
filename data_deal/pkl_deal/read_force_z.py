import pickle
import matplotlib.pyplot as plt
import numpy as np
import os
import sys
from tqdm import tqdm  # 如果没有安装 tqdm，可以注释掉相关代码，或者 pip install tqdm

# ================= 配置区域 (请在此修改) =================

# .pkl 文件所在的文件夹路径
DATA_DIR = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/plug_in_charger'

# 起始 Episode 序号 (整数)
START_EPISODE_ID = 13

# 结束 Episode 序号 (整数，包含此序号)
END_EPISODE_ID = 20

# 要可视化的 Wrench 维度索引 (0=Fx, 1=Fy, 2=Fz, 3=Tx, 4=Ty, 5=Tz)
WRENCH_AXIS_INDEX = 2 
WRENCH_AXIS_NAME = "Force Z"

# 输出图片的保存目录 (默认在 dataset_pkl 旁边的 wrench_plots 文件夹)
OUTPUT_DIR = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/plug_in_charger/wrench_plots'

# ========================================================

# 尝试导入自定义数据模型 (反序列化必须)
try:
    from reactive_diffusion_policy.common.data_models import SensorMessageList, SensorMessage
except ImportError:
    print("【错误】无法导入 SensorMessageList 类。")
    print("请确保你在 'Reactive-Diffusion-Policy-on-Flexiv-rizon4s' 仓库根目录下运行此脚本。")
    sys.exit(1)

def main():
    # 1. 检查数据目录
    if not os.path.exists(DATA_DIR):
        print(f"Error: 数据目录不存在: {DATA_DIR}")
        return

    # 2. 创建输出目录
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"图片将保存至: {OUTPUT_DIR}")
    print(f"准备处理序号范围: {START_EPISODE_ID} -> {END_EPISODE_ID}\n")

    # 3. 循环遍历文件
    # 使用 range 生成序号序列
    for seq_idx in range(START_EPISODE_ID, END_EPISODE_ID + 1):
        
        # 构造文件名: seq_00017.pkl (注意这里假定是5位补零，根据您提供的示例)
        file_name = f"seq_{seq_idx:05d}.pkl"
        file_path = os.path.join(DATA_DIR, file_name)

        if not os.path.exists(file_path):
            print(f"[跳过] 文件不存在: {file_name}")
            continue

        print(f"[处理中] 正在读取: {file_name} ...")

        try:
            with open(file_path, 'rb') as f:
                data = pickle.load(f)
            
            # 检查数据有效性
            if not hasattr(data, 'sensorMessages'):
                print(f"  [警告] {file_name} 格式不正确，跳过。")
                continue
            
            messages = data.sensorMessages
            total_frames = len(messages)
            
            # 提取数据
            wrench_values = []
            for i in range(total_frames):
                msg = messages[i]
                if hasattr(msg, 'leftRobotTCPWrench'):
                    w_vec = msg.leftRobotTCPWrench
                    # 确保索引不越界
                    val = w_vec[WRENCH_AXIS_INDEX] if len(w_vec) > WRENCH_AXIS_INDEX else 0.0
                    wrench_values.append(val)
                else:
                    wrench_values.append(0.0)
            
            # 绘图
            plot_episode(seq_idx, wrench_values, file_name)

        except Exception as e:
            print(f"  [错误] 读取 {file_name} 时发生异常: {e}")

    print("\n所有任务已完成！")

def plot_episode(ep_idx, data_array, original_filename):
    """绘制并保存单个 Episode 的曲线图"""
    data_array = np.array(data_array)
    frames = np.arange(len(data_array))
    
    plt.figure(figsize=(12, 6), dpi=100)
    plt.plot(frames, data_array, label=f'Left Robot {WRENCH_AXIS_NAME}', color='#1f77b4', linewidth=1.2)
    
    # 标注极值点，方便分析
    if len(data_array) > 0:
        max_val = np.max(data_array)
        min_val = np.min(data_array)
        plt.axhline(y=max_val, color='r', linestyle=':', alpha=0.5, label=f'Max: {max_val:.2f}')
        plt.axhline(y=min_val, color='g', linestyle=':', alpha=0.5, label=f'Min: {min_val:.2f}')

    plt.title(f'Episode {ep_idx} - Left TCP Wrench [{WRENCH_AXIS_INDEX}] ({original_filename})', fontsize=14)
    plt.xlabel('Frame Index', fontsize=12)
    plt.ylabel('Force (N) / Torque (Nm)', fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.tight_layout()
    
    # 保存图片
    # 图片命名格式: seq_00017_wrench_2.png
    save_name = f"seq_{ep_idx:05d}_wrench_{WRENCH_AXIS_INDEX}.png"
    save_path = os.path.join(OUTPUT_DIR, save_name)
    
    plt.savefig(save_path)
    plt.close() # 必须关闭，否则循环多次会内存溢出
    print(f"  -> 图片已保存: {save_name}")

if __name__ == "__main__":
    main()