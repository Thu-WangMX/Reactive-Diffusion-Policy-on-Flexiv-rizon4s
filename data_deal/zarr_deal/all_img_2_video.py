#将 zarr 格式的多摄像头图像数据转换为视频文件
#每个 episode 生成一个视频，画面包含外部相机、左手、右手图像的拼接
#视频保存在 zarr 文件同级目录下的 videos 文件夹中
import zarr
import cv2
import numpy as np
import os   
from tqdm import tqdm

# ================= 配置区域 =================
# Zarr 文件路径
ZARR_PATH = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_charger_stream_downsample1_zarr/replay_buffer.zarr'

# 输出视频的保存目录 (默认保存在 zarr 同级目录下的 videos 文件夹)
OUTPUT_DIR = os.path.join(os.path.dirname(ZARR_PATH), 'videos')

# 视频帧率
FPS = 24

# 是否在画面上绘制文字信息 (帧号等)
DRAW_TEXT = True
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
        
        # 获取图像数组 (懒加载，不会一次性读入内存)
        img_external = data_group['external_img']
        img_left = data_group['left_wrist_img']
        img_right = data_group['right_wrist_img']
        
        # 获取 Episode 结束索引
        episode_ends = meta_group['episode_ends'][:]
        
        print(f"Found arrays:")
        print(f"  - External: {img_external.shape}")
        print(f"  - Left Wrist: {img_left.shape}")
        print(f"  - Right Wrist: {img_right.shape}")
        print(f"  - Episode ends: {episode_ends}")
        
    except KeyError as e:
        print(f"Error: Could not find key {e} in Zarr file. Please check structure.")
        return

    # 4. 准备输出目录
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Saving videos to: {OUTPUT_DIR}")

    # 5. 遍历每个 Episode 并生成视频
    start_idx = 0
    for ep_idx, end_idx in enumerate(episode_ends):
        # 视频文件名
        video_path = os.path.join(OUTPUT_DIR, f'episode_{ep_idx:03d}.mp4')
        
        # 计算当前 episode 的长度
        ep_len = end_idx - start_idx
        print(f"Processing Episode {ep_idx}: Frames {start_idx} to {end_idx} ({ep_len} frames)...")
        
        # 获取第一帧以确定视频尺寸
        # 注意：Zarr 存储格式通常是 (N, H, W, C)，OpenCV 需要 (W, H)
        # 这里假设所有图像尺寸一致，如果不一致需要 Resize，这里直接取第一帧的尺寸
        sample_img = img_external[start_idx]
        h, w, c = sample_img.shape
        
        # 三个画面水平拼接，宽度 * 3
        # 拼接顺序：External | Left | Right
        video_w = w * 3
        video_h = h
        
        # 初始化 VideoWriter
        # mp4v 是比较通用的编码格式
        fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
        out = cv2.VideoWriter(video_path, fourcc, FPS, (video_w, video_h))
        
        # 逐帧处理
        # 使用 tqdm 显示进度条
        for i in tqdm(range(start_idx, end_idx), desc=f"Ep {ep_idx}"):
            # 读取三张图
            # 注意：Zarr 读取出来的是 RGB，OpenCV 需要 BGR
            # img_external[i] 返回的是 numpy array
            frame_ext = img_external[i]
            frame_left = img_left[i]
            frame_right = img_right[i]
            
            
            # 水平拼接
            combined_frame = np.hstack([frame_ext, frame_left, frame_right])
            
            # 绘制文字信息
            if DRAW_TEXT:
                text = f"Ep: {ep_idx} | Frame: {i - start_idx}/{ep_len} | Total: {i}"
                cv2.putText(combined_frame, text, (20, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
                
                # 标注每个相机的名字
                cv2.putText(combined_frame, "External", (20, video_h - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(combined_frame, "Left Wrist", (w + 20, video_h - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(combined_frame, "Right Wrist", (w * 2 + 20, video_h - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            # 写入视频
            out.write(combined_frame)
            
        out.release()
        print(f"Saved: {video_path}")
        
        # 更新下一段的起始索引
        start_idx = end_idx

    print("All done!")

if __name__ == "__main__":
    main()