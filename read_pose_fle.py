# 1. 从您提供的文件中导入FlexivRobot类
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

    print("\n--------------------------------------------------")

    # --- 方案一：读取位姿，姿态以四元数 [w, x, y, z] 格式返回 ---
    # 这是默认方式 (Euler_flag=False)
    # 返回一个包含7个元素的列表: [x, y, z, w, x, y, z]
    pose_quat = robot.read_pose() 
    print("读取的TCP位姿 (位置+四元数):")
    print(f"  - 位置 [x, y, z]: {np.round(pose_quat[0:3], 4)}")
    print(f"  - 姿态 [w, x, y, z]: {np.round(pose_quat[3:7], 4)}")

    print("\n--------------------------------------------------")

    # --- 方案二：读取位姿，姿态以欧拉角 [rx, ry, rz] 格式返回 (单位：度) ---
    # 将 Euler_flag 设置为 True
    # 返回两个列表: 位置[x,y,z], 欧拉角[rx,ry,rz]
    position, euler_degrees = robot.read_pose(Euler_flag=True)
    print("读取的TCP位姿 (位置+欧拉角):")
    print(f"  - 位置 [x, y, z]: {np.round(position, 4)}")
    print(f"  - 姿态 [rx, ry, rz] (度): {np.round(euler_degrees, 4)}")
    print("--------------------------------------------------")

    current_pos, current_euler_deg = robot.read_pose(Euler_flag=True)
    #start_pose = Pose.from_xyz_rpy(current_pos, current_euler_deg)


except Exception as e:
    print(f"发生错误: {e}")

finally:
    # 确保在程序结束时停止机器人
    if 'robot' in locals():
        robot.Stop()