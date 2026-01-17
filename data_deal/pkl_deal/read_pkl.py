import sys
import os
import pickle
import numpy as np

# 1. 必须先加入项目路径，否则 pickle 无法识别类
sys.path.append('/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s')

# 2. 指定你的文件路径
file_path = '/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/wiping_board/seq_00016.pkl'

def inspect_deep(data, name="Root", indent=0, max_depth=5):
    prefix = "  " * indent
    if indent > max_depth:
        print(f"{prefix}... (达到最大深度，停止展开)")
        return

    # === 处理 Numpy 数组 ===
    if isinstance(data, np.ndarray):
        print(f"{prefix}{name}: [Numpy Array] Shape={data.shape}, Dtype={data.dtype}")
        return

    # === 处理 基础类型 ===
    if isinstance(data, (int, float, str, bool, np.number)):
        # 只有简单的数值才打印值，避免刷屏
        print(f"{prefix}{name}: {type(data).__name__} = {data}")
        return

    # === 处理 字典 ===
    if isinstance(data, dict):
        print(f"{prefix}{name}: [Dict] Keys={len(data)}")
        for key in data.keys():
            inspect_deep(data[key], name=f"['{key}']", indent=indent+1, max_depth=max_depth)
        return

    # === 处理 列表/元组 ===
    if isinstance(data, (list, tuple)):
        print(f"{prefix}{name}: [{type(data).__name__}] Length={len(data)}")
        if len(data) > 0:
            # 只展示第一个元素作为样本
            print(f"{prefix}  (展示第0个元素的结构):")
            inspect_deep(data[0], name=f"{name}[0]", indent=indent+1, max_depth=max_depth)
        return

    # === 处理 自定义对象 (关键部分) ===
    # 打印对象类型
    obj_type = type(data).__name__
    print(f"{prefix}{name}: <Object: {obj_type}>")

    # 1. 尝试看看它是不是像列表一样 (有长度，能索引)
    if hasattr(data, '__len__') and hasattr(data, '__getitem__'):
        try:
            length = len(data)
            print(f"{prefix}  -> 表现为列表 (Iterable), 长度: {length}")
            if length > 0:
                inspect_deep(data[0], name=f"{name}[0]", indent=indent+1, max_depth=max_depth)
        except:
            pass

    # 2. 暴力查看所有成员变量 (__dict__)
    if hasattr(data, '__dict__'):
        print(f"{prefix}  -> 成员属性 (Attributes):")
        for k, v in vars(data).items():
            inspect_deep(v, name=f".{k}", indent=indent+1, max_depth=max_depth)
    
    # 3. 如果是 dataclass 或 slots (没有 __dict__ 的情况)
    elif hasattr(data, '__slots__'):
         print(f"{prefix}  -> Slots 属性:")
         for k in data.__slots__:
             if hasattr(data, k):
                 inspect_deep(getattr(data, k), name=f".{k}", indent=indent+1, max_depth=max_depth)

if __name__ == "__main__":
    if not os.path.exists(file_path):
        print(f"文件不存在: {file_path}")
    else:
        try:
            with open(file_path, 'rb') as f:
                content = pickle.load(f)
            
            print("="*60)
            print(f"深度解析文件: {os.path.basename(file_path)}")
            print("="*60)
            inspect_deep(content)
            
        except Exception as e:
            print(f"发生错误: {e}")