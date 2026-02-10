import os
import os.path as osp
import pickle
from loguru import logger
import numpy as np

# ================== 配置 ==================
# 请确保这里的 TAG 和 data_dir 与原脚本一致
TAG = 'plug_in_usb'
# 修改为你实际的 dataset_pkl 路径
DATA_DIR = f'/home/wmx/Reactive-Diffusion-Policy-on-Flexiv-rizon4s/dataset_pkl/{TAG}'

def inspect_data():
    if not os.path.exists(DATA_DIR):
        logger.error(f"Directory not found: {DATA_DIR}")
        return

    # 1. 检查文件总数
    all_files = os.listdir(DATA_DIR)
    pkl_files = sorted([f for f in all_files if f.endswith('.pkl')])
    
    total_files = len(pkl_files)
    logger.info(f"Scan complete. Found {total_files} .pkl files in directory.")
    
    if total_files == 0:
        logger.error("No .pkl files found!")
        return

    valid_count = 0
    empty_count = 0
    corrupt_count = 0
    short_count = 0
    
    # 定义“过短”的阈值，防止只有1-2帧的数据混入
    MIN_FRAMES = 5 

    logger.info("============== 开始逐个检查 ==============")

    for seq_idx, data_file in enumerate(pkl_files):
        data_path = osp.join(DATA_DIR, data_file)
        
        try:
            with open(data_path, 'rb') as f:
                data = pickle.load(f)
            
            # 检查 sensorMessages 是否存在
            if not hasattr(data, 'sensorMessages'):
                logger.warning(f"[FAIL] {data_file}: No 'sensorMessages' attribute.")
                corrupt_count += 1
                continue

            msg_len = len(data.sensorMessages)

            # 模拟原脚本的筛选逻辑
            if msg_len == 0:
                logger.warning(f"[EMPTY] {data_file}: sensorMessages is empty (0 frames).")
                empty_count += 1
            elif msg_len < MIN_FRAMES:
                logger.warning(f"[SHORT] {data_file}: Only {msg_len} frames (Too short).")
                short_count += 1
                # 注意：原脚本只要 >0 就会录入，但如果数据只有1帧可能在下采样时出问题
                valid_count += 1 
            else:
                # 这是一个健康的文件
                valid_count += 1
                # logger.info(f"[OK]   {data_file}: {msg_len} frames.") # 太多了，不打印正常的

        except Exception as e:
            logger.error(f"[ERROR] {data_file}: Cannot load pickle. Reason: {e}")
            corrupt_count += 1

    logger.info("============== 统计报告 ==============")
    logger.info(f"Total .pkl files found: {total_files}")
    logger.info(f"Valid Episodes (Expected): {valid_count}")
    logger.info(f"Empty Files (0 frames):    {empty_count}")
    logger.info(f"Corrupt/Error Files:       {corrupt_count}")
    logger.info(f"Short Files (<{MIN_FRAMES} frames):    {short_count}")
    
    logger.info("--------------------------------------")
    actual_loss = total_files - valid_count
    if actual_loss > 0:
        logger.critical(f"Found {actual_loss} files that will be skipped by the main script.")
    else:
        logger.success("All files look valid. If you still miss data, check the 'DEBUG' flag logic.")

if __name__ == '__main__':
    inspect_data()