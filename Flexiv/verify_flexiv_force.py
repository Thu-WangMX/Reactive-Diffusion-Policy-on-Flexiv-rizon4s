import os
import sys
import time
import numpy as np
import flexivrdk
from typing import List
from loguru import logger
from scipy.spatial.transform import Rotation as R

# ==========================================
# 1. FlexivController 类定义 (你之前写好的部分)
# ==========================================
class FlexivController:
    def __init__(self, 
                 robot_sn: str, 
                 gripper_name:str,
                 frequency: float = 100.0, 
                 gripper_init: bool = False,
                 remote_control: bool = True):
        self.DOF=7
        self.robot_sn = robot_sn
        self.gripper_name = gripper_name
        self.frequency = frequency
        
        try:
            self.robot_states = flexivrdk.RobotStates()
            self.gripper_states = flexivrdk.GripperStates()
            self.mode = flexivrdk.Mode
            
            # 连接机器人
            self.robot = flexivrdk.Robot(self.robot_sn)
 
            if remote_control:
                if self.robot.fault():
                    self.clear_fault()
                    if not self.robot.ClearFault():
                        return 1
                
                self.ZeroFTSensor()
                self.robot.Enable()  
                while not self.robot.operational():
                    time.sleep(1)
         
                self.gripper = flexivrdk.Gripper(self.robot) 
                self.tool = flexivrdk.Tool(self.robot)
                self.gripper.Enable(gripper_name)
                
                if gripper_init:
                    self.gripper.Init()
                    time.sleep(10)
                
                # 初始化时先设好力控参数 (虽然 main 里会再设一次，这里保留也没事)
                self.robot.SwitchMode(self.mode.NRT_CARTESIAN_MOTION_FORCE) 
                self.robot.SetForceControlFrame(flexivrdk.CoordType.TCP)
                self.robot.SetForceControlAxis([False, False, True, False, False, False])
                self.robot.SetMaxContactWrench([float("inf")] * 6)
                print("FlexivController 初始化完成: 已设置为笛卡尔力控模式")

        except Exception as e:
            print(f"FlexivController 初始化异常: {str(e)}")
            return None
        
    def ZeroFTSensor(self):
        print(">>> ZeroFTSensor: Switching to PRIMITIVE execution...")
        self.robot.SwitchMode(self.mode.NRT_PRIMITIVE_EXECUTION)
        self.robot.ExecutePrimitive("ZeroFTSensor", dict())
        
        # 等待归零完成
        while not self.robot.primitive_states()["terminated"]:
            time.sleep(0.5)
        logger.info("Sensor zeroing complete")
        
    def clear_fault(self):
        if self.robot.isFault():
            logger.warning("Fault occurred on robot server, trying to clear ...")
            self.robot.ClearFault()
            time.sleep(2)
            if self.robot.isFault():
                return
            
    def switch_PRIMITIVE_Mode(self):
        self.robot.SwitchMode(self.mode.NRT_PRIMITIVE_EXECUTION)

    def Move_gripper(self, width, speed=0.1, force=10.0):
        self.gripper.Move(width, speed, force)
        time.sleep(2)

# ==========================================
# 2. Main 测试函数 (我刚刚修好的部分)
# ==========================================
def main():
    # 配置你的 SN
    ROBOT_SN = 'Rizon4s-062958'      
    GRIPPER_NAME = 'Flexiv-GN01' 
    PRESSING_FORCE = -5.0  # 目标下压力 (TCP Z轴向外为正，下压为负)

    # 1. 连接机器人
    print(f"正在连接机器人: {ROBOT_SN}...")
    # 这里实例化上面定义的 FlexivController
    controller = FlexivController(ROBOT_SN, GRIPPER_NAME, frequency=100.0, remote_control=True, gripper_init=False)
    
    if not hasattr(controller, 'robot'):
        print("机器人连接失败，请检查网络或 SN。")
        return

    print("✓ 机器人连接成功！")
    
    # 切换到 PRIMITIVE 模式
    controller.switch_PRIMITIVE_Mode()
    print("✓ 进入 PRIMITIVE 模式准备归零。")
    print(">>> 机器人已就绪。请手握急停，准备测试！")
    
    # 2. 归零传感器 (再次手动执行一次确保万无一失)
    print(">>> 正在执行传感器归零 (ZeroFTSensor)...")
    controller.ZeroFTSensor()
    time.sleep(2.0) # 关键：给底层一点时间处理偏置
    print(">>> 传感器已归零")

    # 3. 【核心】 开启力控模式“四部曲”
    print(">>> 正在配置力控...")
    try:
        controller.robot.SwitchMode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
    except Exception as e:
        print(f"!!! 切换模式失败: {e}")
        return

    # A. 选坐标系 (TCP = 跟着手走)
    controller.robot.SetForceControlFrame(flexivrdk.CoordType.TCP)
    
    # B. 激活 Z 轴力控 (关键！没有这行就是纯位控)
    # [X, Y, Z, Rx, Ry, Rz]
    controller.robot.SetForceControlAxis([False, False, True, False, False, False])
    
    # C. 关闭接触保护 (关键！否则一碰就停)
    controller.robot.SetMaxContactWrench([float("inf")] * 6)
    
    print(">>> 力控配置完成！")

    # 4. 执行下压
    print(f">>> 开始下压！目标力: {PRESSING_FORCE} N")
    start_time = time.time()
    
    # 你的目标位置 (保持 XY 不动，只在 Z 轴测试力控)
    # 这里用一个固定的姿态，你可以根据实际情况修改
    target_pose = [0.6398, -0.1896, 0.0801, 0.0563, -0.0008, 0.9983, -0.0125]
    
    while time.time() - start_time < 10.0: # 测试 10 秒
        # 目标力向量 (Z轴 -5N)
        target_wrench = [0.0, 0.0, PRESSING_FORCE, 0.0, 0.0, 0.0]
        
        # 发送指令
        # 此时：Z 轴位置会被忽略，机器人会用力去凑 -5N
        controller.robot.SendCartesianMotionForce(target_pose, target_wrench,0.05)
        
        # 打印实时力数据
        real_fz = controller.robot.states().ext_wrench_in_tcp[2]
        print(f"Target: {PRESSING_FORCE} | Real Fz: {real_fz:.2f}")
        
        time.sleep(0.02) 

    print(">>> 测试结束，停止。")
    controller.robot.Stop()

if __name__ == "__main__":
    main()

# #读取的TCP位姿 (位置+四元数):
#   - 位置 [x, y, z]: [0.6508 0.0032 0.0763]
#   - 姿态 [w, x, y, z]: [ 0.0578 -0.0172  0.998   0.0175]