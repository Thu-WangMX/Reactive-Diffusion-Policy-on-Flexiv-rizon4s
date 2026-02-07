import zarr
import numpy as np
import cv2
import os

# 你的 Zarr 数组路径
ZARR_PATH = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/plug_in_charger/plug_in_charger_stream_downsample1_zarr/replay_buffer.zarr/data/external_img'

def check_color_format(zarr_path):
    print(f"正在加载 Zarr 数组: {zarr_path}")
    
    try:
        # 打开 Zarr 数组 (只读模式)
        arr = zarr.open(zarr_path, mode='r')
        print(f"数组形状: {arr.shape}")
        
        # 获取第一帧 (或者中间某一帧，防止第一帧是黑屏)
        # 如果数据量很大，取中间索引
        idx = min(len(arr) // 2, 100) 
        img = arr[idx]
        print(f"读取第 {idx} 帧，形状: {img.shape}")

        # 检查是否需要转换维度 (例如从 Channel-First [3, H, W] 转为 [H, W, 3])
        if img.shape[0] == 3:
            print("检测到 Channel-First (3, H, W)，正在转换为 (H, W, 3)...")
            img = np.transpose(img, (1, 2, 0))
        
        # 确保数据是 uint8 类型
        if img.dtype != np.uint8:
            print(f"注意：数据类型是 {img.dtype}，正在归一化并转换为 uint8 以便显示...")
            img = img.astype(np.float32)
            img = (img - img.min()) / (img.max() - img.min()) * 255.0
            img = img.astype(np.uint8)

        # === 方案 1：假设原始数据存储的是 RGB ===
        # 如果我们用 cv2.imwrite 保存，cv2 默认期望输入是 BGR。
        # 所以如果数据本身是 RGB，我们直接传给 cv2，保存出来的图片颜色会互换（变怪）。
        # 为了避免 cv2 的混淆，我们这里统一用“将数据视为 RGB 转换成 BGR 再保存”的逻辑，
        # 或者更简单：直接保存两个版本的图片供人眼观察。
        
        # 保存为：test_assume_rgb.jpg
        # 含义：如果我们认为数据里的 (R, G, B) 就是正确的顺序，
        # 为了让 cv2 正确保存成 jpg (cv2 写入时是 BGR 顺序)，我们需要把 RGB 转成 BGR 给它。
        # img[..., ::-1] 做的事情是 RGB -> BGR
        cv2.imwrite('test_assume_stored_as_rgb.jpg', img[..., ::-1])
        
        # 保存为：test_assume_bgr.jpg
        # 含义：如果我们认为数据存的就是 BGR，那它已经符合 cv2 的写入标准了，直接保存即可。
        cv2.imwrite('test_assume_stored_as_bgr.jpg', img)

        print("\n=== 检测完成 ===")
        print("已保存两张图片到当前目录：")
        print("1. test_assume_stored_as_rgb.jpg (假设数据是 RGB)")
        print("2. test_assume_stored_as_bgr.jpg (假设数据是 BGR)")
        print("\n请打开这两张图片查看：")
        print("- 如果 'test_assume_stored_as_rgb.jpg' 颜色正常 -> 数据存储格式为 RGB")
        print("- 如果 'test_assume_stored_as_bgr.jpg' 颜色正常 -> 数据存储格式为 BGR")

    except Exception as e:
        print(f"发生错误: {e}")

if __name__ == "__main__":
    if os.path.exists(ZARR_PATH):
        check_color_format(ZARR_PATH)
    else:
        print(f"错误：路径不存在 -> {ZARR_PATH}")