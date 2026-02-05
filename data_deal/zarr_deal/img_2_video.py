import zarr
import cv2
import numpy as np
import os
from pathlib import Path

# ================= 配置区域 (CONFIG) =================

ZARR_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr"
EPISODE_IDX = 38
CAMERA_KEY = "external_img"
#CAMERA_KEY = "left_wrist_img"
OUTPUT_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/video"
FPS = 24

# [关键修改] 如果您的视频颜色不对（橙变蓝），请将此设为 False
# False: 假设数据已经是 BGR 格式 (OpenCV采集)，直接写入
# True : 假设数据是 RGB 格式，需要转换
NEED_CONVERT = False 

# ====================================================

def main():
    print(f"Opening Zarr: {ZARR_PATH}")
    
    try:
        root = zarr.open(ZARR_PATH, mode='r')
    except Exception as e:
        print(f"[Error] 无法打开 Zarr 文件: {e}")
        return

    # 1. 寻找 episode_ends
    if 'meta' in root and 'episode_ends' in root['meta']:
        episode_ends = root['meta']['episode_ends'][:]
    elif 'episode_ends' in root:
        episode_ends = root['episode_ends'][:]
    else:
        print("[Error] 找不到 episode_ends")
        return

    # 2. 确定范围
    total_episodes = len(episode_ends)
    if EPISODE_IDX < 0 or EPISODE_IDX >= total_episodes:
        print(f"[Error] Episode {EPISODE_IDX} 无效")
        return

    start_frame = 0 if EPISODE_IDX == 0 else episode_ends[EPISODE_IDX - 1]
    end_frame = episode_ends[EPISODE_IDX]
    duration = end_frame - start_frame

    print("=" * 60)
    print(f"Target Episode : {EPISODE_IDX}")
    print(f"Color Convert  : {'RGB->BGR' if NEED_CONVERT else 'None (Direct Write)'}")
    print(f"Frame Range    : {start_frame} -> {end_frame} (Total {duration})")
    print("=" * 60)

    # 3. 寻找数据源 ('data' group 兼容)
    data_source = None
    if 'data' in root and CAMERA_KEY in root['data']:
        data_source = root['data']
    elif CAMERA_KEY in root:
        data_source = root
    else:
        print(f"[Error] 找不到图像键: {CAMERA_KEY}")
        if 'data' in root: print(f"  Keys in 'data': {list(root['data'].keys())}")
        return

    # 4. 加载与写入
    print("Loading images...")
    images = data_source[CAMERA_KEY][start_frame:end_frame]
    
    if len(images) == 0: return

    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    height, width, _ = images[0].shape
    save_path = os.path.join(OUTPUT_DIR, f"ep{EPISODE_IDX:03d}_{CAMERA_KEY}.mp4")
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(save_path, fourcc, FPS, (width, height))

    print(f"Rendering to: {save_path}")

    # 进度条处理
    try:
        from tqdm import tqdm
        iterator = tqdm(images, unit="frame")
    except ImportError:
        iterator = images

    for img in iterator:
        if NEED_CONVERT:
            # 如果数据是 RGB，必须转 BGR 才能在 OpenCV 中正确显示
            bgr_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            out.write(bgr_img)
        else:
            # 如果数据已经是 BGR，直接写入，不要乱转
            out.write(img)

    out.release()
    print(f"\nDone.")

if __name__ == "__main__":
    main()