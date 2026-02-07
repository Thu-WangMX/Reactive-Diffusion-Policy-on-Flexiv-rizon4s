import zarr
import matplotlib.pyplot as plt
import numpy as np
import os
from tqdm import tqdm

# ================= 配置区域 =================
# Zarr 文件路径 (请根据实际情况修改)
ZARR_PATH = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_charger_stream_downsample1_zarr/replay_buffer.zarr'

# 输出图片的保存目录
OUTPUT_DIR = os.path.join(os.path.dirname(ZARR_PATH), 'wrench_plots')

# 要可视化的 Wrench 维度索引 (0=Fx, 1=Fy, 2=Fz, 3=Tx, 4=Ty, 5=Tz)
WRENCH_AXIS_INDEX = 2 
WRENCH_AXIS_NAME = "Force Z"

# 绘图样式
FIGURE_SIZE = (12, 6)
DPI = 100

# 【新增】Episode 范围选择
# 设置为 None 表示不限制 (即从头开始 或 直到最后)
# 例如: (0, 5) 表示可视化第 0,1,2,3,4 集
VISUALIZE_START_IDX = 12      # 从第几个 Episode 开始 (包含)
VISUALIZE_END_IDX = None     # 到第几个 Episode 结束 (不包含)，None 表示画到最后
# ===========================================

def main():
    # 1. 检查路径
    if not os.path.exists(ZARR_PATH):
        print(f"Error: Zarr path not found: {ZARR_PATH}")
        return

    # 2. 打开 Zarr 文件
    print(f"Opening Zarr: {ZARR_PATH}")
    try:
        root = zarr.open(ZARR_PATH, mode='r')
        data_group = root['data']
        meta_group = root['meta']
        
        # 获取 Wrench 数据和 Episode 索引
        # shape: (N, 6), usually [Fx, Fy, Fz, Tx, Ty, Tz]
        left_wrench = data_group['left_robot_tcp_wrench'] 
        episode_ends = meta_group['episode_ends'][:]
        
        print(f"Found arrays:")
        print(f"  - Left TCP Wrench: {left_wrench.shape}")
        print(f"  - Episode ends: {episode_ends}")
        
    except KeyError as e:
        print(f"Error: Could not find key {e} in Zarr file. Please check structure.")
        return

    # 3. 准备输出目录
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Saving plots to: {OUTPUT_DIR}")
    
    # 打印范围信息
    start_info = VISUALIZE_START_IDX if VISUALIZE_START_IDX is not None else 0
    end_info = VISUALIZE_END_IDX if VISUALIZE_END_IDX is not None else len(episode_ends)
    print(f"Processing Episode range: [{start_info}, {end_info})")

    # 4. 遍历每个 Episode 并生成曲线图
    start_idx = 0
    for ep_idx, end_idx in enumerate(episode_ends):
        # --- 范围筛选逻辑 ---
        # 1. 检查是否还没到开始的集数
        if VISUALIZE_START_IDX is not None and ep_idx < VISUALIZE_START_IDX:
            # 重要：即使跳过绘图，也要更新 start_idx，否则后续数据读取会错位
            start_idx = end_idx
            continue
        
        # 2. 检查是否超过了结束集数
        if VISUALIZE_END_IDX is not None and ep_idx >= VISUALIZE_END_IDX:
            break
        # --------------------

        # 计算当前 episode 的长度
        ep_len = end_idx - start_idx
        
        # 读取当前 Episode 的 Wrench 数据
        # 注意：这里只读取需要的那个维度 (:, 2)
        # 此时 wrench_segment 是一个一维数组 (T,)
        wrench_segment = left_wrench[start_idx:end_idx, WRENCH_AXIS_INDEX]
        
        # 创建时间轴 (帧数)
        frames = np.arange(ep_len)
        
        # --- 绘图 ---
        plt.figure(figsize=FIGURE_SIZE, dpi=DPI)
        plt.plot(frames, wrench_segment, label=f'Left Robot {WRENCH_AXIS_NAME}', color='blue', linewidth=1.5)
        
        # 添加标题和标签
        plt.title(f'Episode {ep_idx} - Left Robot TCP Wrench [{WRENCH_AXIS_INDEX}] ({WRENCH_AXIS_NAME})', fontsize=14)
        plt.xlabel('Frame', fontsize=12)
        plt.ylabel('Value', fontsize=12)
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.legend()
        
        # 保存图片
        plot_path = os.path.join(OUTPUT_DIR, f'episode_{ep_idx:03d}_wrench.png')
        plt.savefig(plot_path)
        plt.close() # 关闭画布释放内存
        
        print(f"Processed Episode {ep_idx}: {ep_len} frames -> {plot_path}")
        
        # 更新下一段的起始索引
        start_idx = end_idx

    print("All done!")

if __name__ == "__main__":
    main()