# 1. 从您提供的文件中导入FlexivRobot类
import sys
import os

# 获取当前脚本的绝对路径
current_dir = os.path.dirname(os.path.abspath(__file__))
# 获取上一级目录（即项目根目录）的路径
parent_dir = os.path.dirname(current_dir)

# 将项目根目录添加到 sys.path 中
sys.path.append(parent_dir)
from FlexivRobot import FlexivRobot
import numpy as np

# 2. 替换为您自己的机器人序列号 (SN)
#    注意：必须将 remote_control 设置为 True 才能与机器人通信

ROBOT_SN =  'Rizon4s-062958'
GRIPPER_NAME ='Flexiv-GN01'

try:
    # 3. 初始化机器人对象
    print(f"正在连接机器人: {ROBOT_SN}...")
    robot = FlexivRobot(ROBOT_SN, GRIPPER_NAME, frequency = 100.0, remote_control=True, gripper_init=False)
    
    print("✓ 机器人连接成功！")
    robot.switch_PRIMITIVE_Mode()
    print("✓ 机器人初始化完成并进入PRIMITIVE模式。")


    #position = [0.5300 ,-0.0269 , 0.30] #插插头擦白板的初始位置
    position = [0.5800 ,-0.0269 , 0.25] #插usb的初始位置
    euler_degrees = [-179.1952  ,  0.2877   ,-178.6688]

    
    robot.MoveL(position,euler_degrees, speed=0.1, acc=0.1)
    

    print("完成MoveL和Movegripper")
except Exception as e:
    print(f"发生错误: {e}")

finally:
    # 确保在程序结束时停止机器人
    if 'robot' in locals():
        robot.Stop()