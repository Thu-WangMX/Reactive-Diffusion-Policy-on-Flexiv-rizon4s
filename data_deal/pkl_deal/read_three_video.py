import os
import pickle
import numpy as np
import cv2
from tqdm import tqdm

# ============================ 配置区（直接修改这里的路径/参数）============================
PKL_PATH = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/plug_in_usb/seq_00001.pkl"  # 你的pkl文件路径
OUTPUT_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/plug_in_usb/pkl_videos"    # 视频输出目录
FPS = 24                                                                         # 视频帧率（和采集帧率一致即可）
SINGLE_CAM_SIZE = (640, 480)                                                     # 单相机视频尺寸，和pkl中图像一致
# ======================================================================================

def load_pkl_file(pkl_path):
    """加载pkl文件并返回sensorMessages列表，适配SensorMessageList结构"""
    with open(pkl_path, 'rb') as f:
        data = pickle.load(f)
    
    if hasattr(data, 'sensorMessages'):
        sensor_messages = data.sensorMessages
        print(f"✅ 成功加载{pkl_path}，共{len(sensor_messages)}帧相机数据")
        return sensor_messages
    else:
        raise ValueError("❌ pkl文件结构异常，未找到sensorMessages属性！")

def create_video_writer(video_path, width, height, fps=24):
    """创建视频写入器，MP4格式+H264编码，兼容性强"""
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"❌ 无法创建视频文件：{video_path}，请检查目录权限！")
    return writer

def resize_image(img, target_width, target_height):
    """调整图像大小，保持比例+黑边填充，避免拉伸变形"""
    h, w = img.shape[:2]
    scale = min(target_width/w, target_height/h)
    new_w, new_h = int(w*scale), int(h*scale)
    resized_img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    
    # 黑底画布居中放置图像
    canvas = np.zeros((target_height, target_width, 3), dtype=np.uint8)
    x_offset = (target_width - new_w) // 2
    y_offset = (target_height - new_h) // 2
    canvas[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized_img
    return canvas

def pkl_to_video():
    """核心逻辑：提取pkl中三个相机RGB，仅生成三画面拼接视频（修正颜色反转）"""
    # 创建输出目录
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    pkl_name = os.path.splitext(os.path.basename(PKL_PATH))[0]  # 提取pkl文件名（不含后缀）
    
    # 加载pkl数据
    sensor_messages = load_pkl_file(PKL_PATH)
    
    # 仅初始化三画面拼接视频写入器（横向：外部相机 | 左腕相机 | 右腕相机）
    concat_width = SINGLE_CAM_SIZE[0] * 3
    concat_height = SINGLE_CAM_SIZE[1]
    concat_writer = create_video_writer(
        os.path.join(OUTPUT_DIR, f"{pkl_name}_all_cameras.mp4"),
        concat_width, concat_height, FPS
    )
    
    # 逐帧处理相机数据
    for idx, msg in enumerate(tqdm(sensor_messages, desc="正在生成拼接视频", unit="帧")):
        try:
            # 提取RGB图像：移除BGR转换，直接使用原始RGB格式（修正颜色反转问题）
            ext_rgb = msg.externalCameraRGB
            left_rgb = msg.leftWristCameraRGB
            right_rgb = msg.rightWristCameraRGB
        except AttributeError as e:
            print(f"\n⚠️ 跳过第{idx}帧：缺少相机数据 → {e}")
            continue
        except Exception as e:
            print(f"\n⚠️ 跳过第{idx}帧：帧数据异常 → {e}")
            continue
        
        # 调整图像尺寸（统一尺寸，避免拼接错位）
        ext_img = resize_image(ext_rgb, SINGLE_CAM_SIZE[0], SINGLE_CAM_SIZE[1])
        left_img = resize_image(left_rgb, SINGLE_CAM_SIZE[0], SINGLE_CAM_SIZE[1])
        right_img = resize_image(right_rgb, SINGLE_CAM_SIZE[0], SINGLE_CAM_SIZE[1])
        
        # 拼接三画面并写入视频
        concat_frame = np.hstack([ext_img, left_img, right_img])
        concat_writer.write(concat_frame)
    
    # 释放资源（必须执行，否则视频文件损坏）
    concat_writer.release()
    cv2.destroyAllWindows()
    
    # 输出完成提示
    print(f"\n🎉 拼接视频生成完成！文件已保存至：\033[32m{OUTPUT_DIR}\033[0m")
    print(f"📌 生成文件：{pkl_name}_all_cameras.mp4 （外部相机 | 左腕相机 | 右腕相机 三画面拼接）")

if __name__ == "__main__":
    # 直接执行核心函数
    pkl_to_video()