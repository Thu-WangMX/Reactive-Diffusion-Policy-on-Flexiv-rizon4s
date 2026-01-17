import sys
import os
import pickle
import cv2
import numpy as np

# ================= 配置区域 =================
project_root = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s'
file_path = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/wiping_board/seq_00002.pkl'
# ===========================================

sys.path.append(project_root)

def save_video_from_frames(frames, filename, fps=30):
    if len(frames) == 0:
        print(f"警告: {filename} 没有帧数据，跳过。")
        return

    # 获取图像尺寸 (Height, Width)
    h, w, _ = frames[0].shape
    print(f"正在写入视频: {filename} | 尺寸: {w}x{h} | 帧数: {len(frames)} | FPS: {fps}")

    # 初始化视频写入器 (mp4v 编码)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(filename, fourcc, fps, (w, h))

    for frame in frames:
        # 【修正点】: 数据源已经是 BGR 格式了，不需要再转换
        # 直接写入原始 frame
        out.write(frame)

    out.release()
    print(f"保存成功: {filename}")

def generate_videos():
    if not os.path.exists(file_path):
        print(f"文件不存在: {file_path}")
        return

    print("正在加载数据...")
    with open(file_path, 'rb') as f:
        data = pickle.load(f)

    msgs = data.sensorMessages
    
    # 计算 FPS
    if len(msgs) > 1:
        duration = msgs[-1].timestamp - msgs[0].timestamp
        save_fps = int(round(len(msgs) / duration))
    else:
        save_fps = 30 

    # 提取 External Camera RGB
    print("提取 External Camera RGB...")
    ext_frames = []
    for m in msgs:
        if hasattr(m, 'externalCameraRGB') and m.externalCameraRGB.size > 0:
            ext_frames.append(m.externalCameraRGB)
    
    save_video_from_frames(ext_frames, 'check_video_external_fixed.mp4', fps=save_fps)

    # 提取 Left Wrist Camera RGB
    print("提取 Left Wrist Camera RGB...")
    wrist_frames = []
    for m in msgs:
        if hasattr(m, 'leftWristCameraRGB') and m.leftWristCameraRGB.size > 0:
            wrist_frames.append(m.leftWristCameraRGB)
            
    save_video_from_frames(wrist_frames, 'check_video_wrist_fixed.mp4', fps=save_fps)

if __name__ == "__main__":
    generate_videos()