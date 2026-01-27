import pickle
import numpy as np
import matplotlib.pyplot as plt

# 1. 定义pkl文件路径
pkl_path = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/wiping_board/seq_00001.pkl"

# 2. 读取pkl文件并解析SensorMessageList结构
try:
    with open(pkl_path, 'rb') as f:
        data = pickle.load(f)
    print("成功读取pkl文件！")
    print("数据类型：", type(data))
    
    # 获取所有帧的SensorMessage列表
    sensor_msgs = data.sensorMessages
    print(f"sensorMessages总帧数：{len(sensor_msgs)}")

except FileNotFoundError:
    print(f"错误：找不到文件 {pkl_path}")
    exit(1)
except Exception as e:
    print(f"读取文件出错：{e}")
    exit(1)

# 3. 提取第一帧的外部相机RGB图像（使用正确的属性名externalCameraRGB）
try:
    k = 0  # 第一帧
    if k >= len(sensor_msgs):
        raise IndexError(f"帧数{k}超出范围，总帧数仅{len(sensor_msgs)}")
    
    # 核心修正：使用正确的属性名externalCameraRGB
    frame_obj = sensor_msgs[k]
    external_img = frame_obj.externalCameraRGB  # 替换为正确的属性名
    
    # 打印图像信息，确认数据格式
    print(f"第{k}帧externalCameraRGB形状：{external_img.shape}，数据类型：{external_img.dtype}")
    
    # 4. BGR转RGB（解决颜色显示异常问题）
    if len(external_img.shape) == 3 and external_img.shape[-1] == 3:
        img_rgb = external_img[..., [2, 1, 0]]  # BGR -> RGB
    else:
        img_rgb = external_img
    
    # 5. 可视化图像
    plt.figure(figsize=(8, 6))
    plt.imshow(img_rgb)
    plt.axis('off')
    plt.title(f"seq_00001.pkl - externalCameraRGB 第{k}帧", fontsize=12)
    plt.tight_layout()
    plt.show()

except AttributeError:
    print(f"错误：第{k}帧SensorMessage对象没有'externalCameraRGB'属性！")
    print("若需读取其他相机图像，可选属性：")
    print("- 左手腕相机：leftWristCameraRGB")
    print("- 左夹爪相机1：leftGripperCameraRGB1")
    print("- 左夹爪相机2：leftGripperCameraRGB2")
    print("- 右手腕相机：rightWristCameraRGB")
except IndexError as e:
    print(f"索引错误：{e}")
except Exception as e:
    print(f"处理图像出错：{e}")
    print(f"错误详情：{type(e)} - {str(e)}")