import zarr
import matplotlib.pyplot as plt
import numpy as np  # 新增：用于通道转换

zarr_path = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset/wiping_board_stream_downsample1_zarr/replay_buffer.zarr/data"
root = zarr.open(zarr_path, mode="r")

print("Keys:", list(root.array_keys()))

# 取第 k 帧的图像
k = 0  # 想看第几帧就改这里
img = root["external_img"][k]  # shape: (240, 320, 3), uint8
print("Frame", k, "img shape:", img.shape, "dtype:", img.dtype)

# 关键修复：将BGR转换为RGB（如果是RGBA则用img[..., [2,1,0,3]]）
img_rgb = img[..., [2, 1, 0]]

plt.figure(figsize=(4, 4))
print("原始BGR通道值:", img[0, 0])
print("转换后RGB通道值:", img_rgb[0, 0])
plt.imshow(img_rgb)  # 使用转换后的RGB图像
plt.axis("off")
plt.title(f"left_wrist_img frame {k}")
plt.show()