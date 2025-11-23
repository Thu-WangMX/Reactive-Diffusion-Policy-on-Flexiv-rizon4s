import zarr
import matplotlib.pyplot as plt

zarr_path = "/home/wmx/myspace/RDP/data/test1_downsample1_zarr/replay_buffer.zarr/data"
root = zarr.open(zarr_path, mode="r")

print("Keys:", list(root.array_keys()))

# 取第 k 帧的图像
k = 1333  # 想看第几帧就改这里
img = root["left_wrist_img"][k]  # shape: (240, 320, 3), uint8
#img = root["external_img"][k]
print("Frame", k, "img shape:", img.shape, "dtype:", img.dtype)

plt.figure(figsize=(4, 4))
plt.imshow(img)       # zarr 里是 HWC、RGB 顺序，直接 imshow 即可
plt.axis("off")
plt.title(f"left_wrist_img frame {k}")
plt.show()