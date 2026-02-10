import pickle
import os
import sys
import numpy as np

# 修改为其中一个“坏”文件的路径
BAD_FILE_PATH = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/plug_in_usb/seq_00021.pkl'

def get_size(obj, seen=None):
    """递归计算对象大小（估算）"""
    size = sys.getsizeof(obj)
    if seen is None:
        seen = set()
    obj_id = id(obj)
    if obj_id in seen:
        return 0
    seen.add(obj_id)
    
    if isinstance(obj, dict):
        size += sum([get_size(v, seen) for v in obj.values()])
        size += sum([get_size(k, seen) for k in obj.keys()])
    elif hasattr(obj, '__dict__'):
        size += get_size(obj.__dict__, seen)
    elif hasattr(obj, '__iter__') and not isinstance(obj, (str, bytes, bytearray)):
        size += sum([get_size(i, seen) for i in obj])
    return size

def inspect_heavy_file():
    if not os.path.exists(BAD_FILE_PATH):
        print(f"File not found: {BAD_FILE_PATH}")
        return

    print(f"Loading {BAD_FILE_PATH} ... (This might take a moment due to 1GB+ size)")
    with open(BAD_FILE_PATH, 'rb') as f:
        data = pickle.load(f)
    
    print("\n" + "="*40)
    print(f"Type of loaded object: {type(data)}")
    print("="*40)

    # 1. 检查 sensorMessages
    if hasattr(data, 'sensorMessages'):
        msg_list = data.sensorMessages
        print(f"\n[sensorMessages] length: {len(msg_list)}")
        print(f"[sensorMessages] type: {type(msg_list)}")
    else:
        print("\n[sensorMessages] ATTRIBUTE MISSING!")

    # 2. 遍历所有属性，寻找占用内存的“大户”
    print("\n" + "="*40)
    print("Inspecting ALL attributes to find the hidden data:")
    print("="*40)
    
    # 获取所有属性（包括 public 和 private）
    attributes = dir(data)
    
    found_heavy_data = False

    for attr_name in attributes:
        # 跳过部分内置方法，但在不知道数据结构时，建议都看看
        if attr_name.startswith('__') and attr_name.endswith('__'):
            continue
            
        try:
            val = getattr(data, attr_name)
            
            # 简单的类型和长度概览
            val_type = type(val).__name__
            info = f"Type: {val_type}"
            
            # 如果是列表、元组、字典、Numpy数组，打印长度/形状
            if hasattr(val, '__len__'):
                info += f", Len: {len(val)}"
            if hasattr(val, 'shape'):
                info += f", Shape: {val.shape}"
            
            # 尝试估算是否是“大”数据
            # 注意：sys.getsizeof 对自定义对象可能不准，但对 numpy 或 list 有参考价值
            is_suspicious = False
            
            # 如果是 list 且长度很大
            if isinstance(val, list) and len(val) > 0:
                is_suspicious = True
            # 如果是 numpy 且很大
            elif hasattr(val, 'nbytes') and val.nbytes > 1024*1024: # > 1MB
                info += f", Size: {val.nbytes / 1024 / 1024:.2f} MB"
                is_suspicious = True
            
            prefix = "[FOUND DATA?] -> " if is_suspicious else "             "
            print(f"{prefix} Attribute: '{attr_name}' | {info}")
            
            if is_suspicious:
                found_heavy_data = True
                
        except Exception as e:
            print(f"             Attribute: '{attr_name}' | Error accessing: {e}")

    if not found_heavy_data:
        print("\n[WARNING] No obvious heavy attributes found via standard inspection.")
        print("The data might be hidden in a private dict or a deeply nested object.")

if __name__ == '__main__':
    inspect_heavy_file()