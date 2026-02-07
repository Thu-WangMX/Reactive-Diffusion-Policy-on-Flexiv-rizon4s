import zarr
import cv2
import numpy as np
import os   
from tqdm import tqdm

# ================= 配置区域 =================
# Zarr 文件路径
ZARR_PATH = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_charger/plug_in_charger_stream_downsample1_zarr/replay_buffer.zarr'
# 输出视频的保存目录
OUTPUT_DIR = os.path.join(os.path.dirname(ZARR_PATH), 'videos')

# 视频帧率
FPS = 24

# 是否在画面上绘制文字信息 (帧号等)
DRAW_TEXT = True

# 【新增】Episode 范围选择
# 设置为 None 表示不限制 (即从头开始 或 直到最后)
# 例如: (12, 15) 表示可视化第 12, 13, 14 集
VISUALIZE_START_IDX = 12      # 从第几个 Episode 开始 (包含)
VISUALIZE_END_IDX = 13        # 到第几个 Episode 结束 (不包含)，None 表示画到最后
# ===========================================

def main():
    # 1. 检查路径
    if not os.path.exists(ZARR_PATH):
        print(f"Error: Zarr path not found: {ZARR_PATH}")
        return

    # 2. 打开 Zarr 文件
    print(f"Opening Zarr: {ZARR_PATH}")
    root = zarr.open(ZARR_PATH, mode='r')
    
    # 3. 获取数据句柄
    try:
        data_group = root['data']
        meta_group = root['meta']
        
        # 获取图像数组
        img_external = data_group['external_img']
        img_left = data_group['left_wrist_img']
        img_right = data_group['right_wrist_img']
        
        # 获取 Episode 结束索引
        episode_ends = meta_group['episode_ends'][:]
        
        print(f"Found arrays:")
        print(f"  - External: {img_external.shape}")
        print(f"  - Left Wrist: {img_left.shape}")
        print(f"  - Right Wrist: {img_right.shape}")
        print(f"  - Total Episodes: {len(episode_ends)}")
        
    except KeyError as e:
        print(f"Error: Could not find key {e} in Zarr file. Please check structure.")
        return

    # 4. 准备输出目录
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    # 打印范围信息
    start_info = VISUALIZE_START_IDX if VISUALIZE_START_IDX is not None else 0
    end_info = VISUALIZE_END_IDX if VISUALIZE_END_IDX is not None else len(episode_ends)
    print(f"Processing Episode range: [{start_info}, {end_info})")
    print(f"Saving videos to: {OUTPUT_DIR}")

    # 5. 遍历每个 Episode 并生成视频
    start_idx = 0
    for ep_idx, end_idx in enumerate(episode_ends):
        # --- 范围筛选逻辑 (参考 Wrench 脚本) ---
        # 1. 检查是否还没到开始的集数
        if VISUALIZE_START_IDX is not None and ep_idx < VISUALIZE_START_IDX:
            # 重要：即使跳过，也要更新 start_idx 以维持索引同步
            start_idx = end_idx
            continue
        
        # 2. 检查是否超过了结束集数
        if VISUALIZE_END_IDX is not None and ep_idx >= VISUALIZE_END_IDX:
            break
        # ------------------------------------

        # 视频文件名
        video_path = os.path.join(OUTPUT_DIR, f'episode_{ep_idx:03d}.mp4')
        
        # 计算当前 episode 的长度
        ep_len = end_idx - start_idx
        
        # 获取第一帧以确定视频尺寸
        sample_img = img_external[start_idx]
        h, w, c = sample_img.shape
        
        # 拼接后的尺寸：水平三并排
        video_w = w * 3
        video_h = h
        
        # 初始化 VideoWriter
        fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
        out = cv2.VideoWriter(video_path, fourcc, FPS, (video_w, video_h))
        
        # 逐帧处理
        for i in tqdm(range(start_idx, end_idx), desc=f"Ep {ep_idx:03d}"):
            # 读取三张图 (注意：若 Zarr 存的是 RGB，需转换为 BGR 供 OpenCV 使用)
            frame_ext = cv2.cvtColor(img_external[i], cv2.COLOR_RGB2BGR)
            frame_left = cv2.cvtColor(img_left[i], cv2.COLOR_RGB2BGR)
            frame_right = cv2.cvtColor(img_right[i], cv2.COLOR_RGB2BGR)
            
            # 水平拼接
            combined_frame = np.hstack([frame_ext, frame_left, frame_right])
            
            # 绘制文字信息
            if DRAW_TEXT:
                text = f"Ep: {ep_idx} | Frame: {i - start_idx}/{ep_len}"
                cv2.putText(combined_frame, text, (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
                
                # 标注子画面名称
                labels = ["External", "Left Wrist", "Right Wrist"]
                for j, label in enumerate(labels):
                    cv2.putText(combined_frame, label, (w * j + 20, h - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # 写入视频
            out.write(combined_frame)
            
        out.release()
        print(f"Saved: {video_path} ({ep_len} frames)")
        
        # 更新下一段的起始索引
        start_idx = end_idx

    print("All done!")

if __name__ == "__main__":
    main()