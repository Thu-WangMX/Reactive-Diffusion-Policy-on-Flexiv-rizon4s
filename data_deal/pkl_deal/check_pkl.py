import os
import pickle
import numpy as np
from tqdm import tqdm  # 进度条，直观显示检查进度

# ============================ 配置区（只需改这里）============================
PKL_DIR = "/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/plug_in_usb"
# 需要校验的核心字段（根据你的业务场景调整）
CHECK_FIELDS = [
    "sensorMessages",       # 必须存在的核心数据列表
    "externalCameraRGB",    # 相机图像字段
    "leftWristCameraRGB",   # 左手腕相机
    "rightWristCameraRGB",  # 右手腕相机
    "left_tcp_pose",        # 左臂TCP位姿
    "left_gripper_width"    # 左臂夹爪宽度
]
# ============================================================================

def check_pkl_file(pkl_path):
    """
    检查单个PKL文件是否有效
    返回：(is_valid: bool, error_msg: str, frame_count: int)
    """
    try:
        # 1. 加载PKL文件
        with open(pkl_path, 'rb') as f:
            data = pickle.load(f)
        
        # 2. 检查核心字段是否存在
        if not hasattr(data, "sensorMessages"):
            return False, "缺失核心字段: sensorMessages", 0
        
        sensor_msgs = data.sensorMessages
        if len(sensor_msgs) == 0:
            return False, "sensorMessages为空（无帧数据）", 0
        
        # 3. 检查第一帧核心字段有效性（抽样验证，也可遍历所有帧）
        first_frame = sensor_msgs[0]
        frame_error = []
        for field in CHECK_FIELDS[1:]:  # 跳过已检查的sensorMessages
            if not hasattr(first_frame, field):
                frame_error.append(f"缺失字段: {field}")
                continue
            
            # 获取字段值并校验
            field_val = getattr(first_frame, field)
            
            # 校验图像类字段（非空、维度合理、非全黑）
            if "CameraRGB" in field:
                if not isinstance(field_val, np.ndarray):
                    frame_error.append(f"{field} 不是numpy数组（类型：{type(field_val)}）")
                elif field_val.size == 0:
                    frame_error.append(f"{field} 数组为空")
                elif field_val.shape not in [(480, 640, 3), (720, 1280, 3)]:  # 常见相机分辨率
                    frame_error.append(f"{field} 维度异常: {field_val.shape}（预期480x640x3等）")
                elif np.all(field_val == 0):
                    frame_error.append(f"{field} 全黑（所有像素值为0）")
            
            # 校验数值类字段（非空、数值合理）
            else:
                if isinstance(field_val, np.ndarray):
                    if field_val.size == 0:
                        frame_error.append(f"{field} 数组为空")
                    elif np.isnan(field_val).any():
                        frame_error.append(f"{field} 包含NaN值")
                    elif np.isinf(field_val).any():
                        frame_error.append(f"{field} 包含无穷大值")
                else:
                    # 非数组类型（如float/int）
                    if field_val is None:
                        frame_error.append(f"{field} 值为None")
        
        # 4. 汇总帧级错误
        if frame_error:
            return False, f"第一帧数据异常: {'; '.join(frame_error)}", len(sensor_msgs)
        
        # 5. 所有检查通过
        return True, "数据有效", len(sensor_msgs)
    
    except Exception as e:
        # 捕获所有异常（文件损坏、解析失败等）
        return False, f"文件加载/解析失败: {str(e)}", 0

def main():
    # 1. 获取目录下所有PKL文件
    pkl_files = [
        os.path.join(PKL_DIR, f) 
        for f in os.listdir(PKL_DIR) 
        if f.endswith(".pkl")
    ]
    if not pkl_files:
        print(f"❌ 未在 {PKL_DIR} 找到任何.pkl文件")
        return
    
    print(f"✅ 找到 {len(pkl_files)} 个PKL文件，开始检查...\n")
    
    # 2. 批量检查
    stats = {
        "total": len(pkl_files),
        "valid": 0,
        "invalid": 0,
        "invalid_files": []  # 存储无效文件详情
    }
    
    for pkl_path in tqdm(pkl_files, desc="检查进度"):
        pkl_name = os.path.basename(pkl_path)
        is_valid, err_msg, frame_cnt = check_pkl_file(pkl_path)
        
        if is_valid:
            stats["valid"] += 1
        else:
            stats["invalid"] += 1
            stats["invalid_files"].append({
                "filename": pkl_name,
                "error": err_msg,
                "frame_count": frame_cnt
            })
    
    # 3. 输出检查报告
    print("\n" + "="*60)
    print("📊 PKL文件有效性检查报告")
    print("="*60)
    print(f"总文件数: {stats['total']}")
    print(f"有效文件数: {stats['valid']}")
    print(f"无效文件数: {stats['invalid']}")
    print(f"有效率: {stats['valid']/stats['total']*100:.2f}%")
    
    if stats["invalid"] > 0:
        print("\n❌ 无效文件详情：")
        for idx, invalid in enumerate(stats["invalid_files"], 1):
            print(f"{idx}. 文件名: {invalid['filename']}")
            print(f"   错误原因: {invalid['error']}")
            print(f"   帧数量: {invalid['frame_count']}\n")
    else:
        print("\n🎉 所有PKL文件均有效！")

if __name__ == "__main__":
    main()