from hydra import initialize, compose
import rclpy
import os.path as osp
import sys
from loguru import logger
from reactive_diffusion_policy.real_world.real_world_transforms import RealWorldTransforms
from reactive_diffusion_policy.real_world.teleoperation.data_recorder import DataRecorder

import os
import psutil
import argparse  # 导入argparse模块

# python record_data.py seq_0002.pkl  # 基础用法，使用默认保存目录
# python record_data.py seq_0003.pkl --save_file_dir my_task  # 自定义保存目录
# python record_data.py seq_0004.pkl --debug  # 启用调试模式

# 限制线程数量
os.environ["OPENBLAS_NUM_THREADS"] = "12"
os.environ["MKL_NUM_THREADS"] = "12"
os.environ["NUMEXPR_NUM_THREADS"] = "12"
os.environ["OMP_NUM_THREADS"] = "12"

import cv2
cv2.setNumThreads(12)

# 设置CPU亲和性
total_cores = psutil.cpu_count()
num_cores_to_bind = 8
cores_to_bind = set(range(min(num_cores_to_bind, total_cores)))
os.sched_setaffinity(0, cores_to_bind)

def main(args=None):
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Data Recorder')
    # 添加save_file_name作为必选位置参数
    parser.add_argument('save_file_name', type=str, help='File name of the save file (e.g., seq_0002.pkl)')
    # 保留其他可选参数
    parser.add_argument('--save_file_dir', type=str, default='test1', help='Directory to save the file')
    parser.add_argument('--save_to_disk', action='store_true', default=True, help='Whether to save data to disk')
    parser.add_argument('--debug', action='store_true', default=False, help='Enable debug messages')
    args = parser.parse_args()

    with initialize(config_path='reactive_diffusion_policy/config', version_base="1.1"):
        cfg = compose(config_name="real_world_env")

    rclpy_args = sys.argv
    rclpy.init(args=rclpy_args)

    # 使用命令行参数构建路径
    base_dir = f'/home/wmx/myspace/RDP/data/{args.save_file_dir}'
    os.makedirs(base_dir, exist_ok=True)
    save_path = osp.join(base_dir, args.save_file_name)

    transforms = RealWorldTransforms(option=cfg.task.transforms)
    node = DataRecorder(
        transforms,
        save_path=save_path,
        debug=args.debug,
        device_mapping_server_ip=cfg.task.device_mapping_server.host_ip,
        device_mapping_server_port=cfg.task.device_mapping_server.port
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if args.save_to_disk:
            node.save()
        else:
            logger.info("Data not saved to disk, quitting program now...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()