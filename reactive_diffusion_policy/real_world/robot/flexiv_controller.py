import os
current_path = os.path.dirname(os.path.abspath(__file__))
#flexivrdk_root_path = os.path.join(current_path, '../', '../', '../', 'third_party', "flexiv_rdk-main")

#print(flexivrdk_root_path)
import sys
# sys.path.insert(0, flexivrdk_root_path+"/lib_py")
import flexivrdk
# sys.path.insert(0, flexivrdk_root_path+"/example_py")
import numpy as np
import time
from typing import List
from loguru import logger
from scipy.spatial.transform import Rotation as R
import threading

class FlexivController:
    def __init__(self, 
                 robot_sn: str, 
                 gripper_name:str,
                 frequency: float = 100.0, 
                 gripper_init: bool = False,
                 remote_control: bool = True):
        """
        初始化Flexiv机器人对象

        参数:
            robot_sn (str): 机器人序列号，用于建立连接
            gripper_name (str): 夹爪名称
            frequency (float): 控制频率，默认为100.0Hz
            gripper_init (bool): 是否初始化夹爪，默认为False,需要在remote_control为True时才能使用,也可以在示教器上手动初始化
            remote_control (bool): 是否启用远程控制模式，默认为False

        功能:
            - 建立与机器人的连接
            - 如果remote_control为True，清除故障并启用机器人
            - 初始化夹爪和工具
            - 切换TCP至夹爪坐标系
        """
        self.DOF=7
        
        self.robot_sn = robot_sn
        self.gripper_name = gripper_name
        self.frequency = frequency
        print(robot_sn)
        print(gripper_name)
        #print(1111111122)
        try:
            
            self.robot_states = flexivrdk.RobotStates()
            
            self.gripper_states = flexivrdk.GripperStates()
            #self.log = flexivrdk.Log()
            
            
            #self.mode = flexivrdk.Mode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
            self.mode = flexivrdk.Mode
            #print(1111111122333)
            
            self.robot = flexivrdk.Robot(self.robot_sn)
 
            #print(222222)

            if remote_control:
                
                if self.robot.fault():
                    #self.log.info("Fault occurred on the connected robot, trying to clear ...")
                   
                    self.clear_fault()
                    if not self.robot.ClearFault():
                        #self.log.info("Fault cannot be cleared, exiting ...")
                        
                        return 1
                    #self.log.info("Fault on the connected robot is cleared")
                #self.log.info("Enabling robot ...")
                print(444)
                self.ZeroFTSensor()
                self.robot.Enable()  
                seconds_waited = 0
                print(self.robot.operational())
                while not self.robot.operational():
                    time.sleep(1)
                    
                    # seconds_waited += 1
                    # if seconds_waited == 10:
                    #     logger.warn(
                    #         "Still waiting for robot to become operational, please check that the robot 1) "
                    #         "has no fault, 2) is in [Auto (remote)] mode")
         
                self.gripper = flexivrdk.Gripper(self.robot) 
                self._gripper_lock = threading.Lock()

                print(99999999)
            
                self.tool = flexivrdk.Tool(self.robot)
                
                self.gripper.Enable(gripper_name)
                
                
                # self.gripper.Move(0,0.1,80)#新增：初始时闭合夹爪
                # print("夹爪闭合")
       
                
                #初始化，在开机时可以先在示教器上手动初始化
                if gripper_init:
                    self.gripper.Init()
                    time.sleep(10) #等待夹爪初始化完成
                
                # self.logger.info("Enabling gripper")
                # #切换tcp至夹爪坐标系,默认坐标系是与示教器上一致
                #self.tool.Switch(gripper_name)
                
                #self.log.info("Left robot is now operational")
                self.robot.SwitchMode(self.mode.NRT_CARTESIAN_MOTION_FORCE) 
                # self.robot.SetForceControlFrame(flexivrdk.CoordType.TCP)
                # self.robot.SetForceControlAxis([False, False, True, False, False, False])
                # self.robot.SetMaxContactWrench([float("inf")] * 6)
                print("已成功设置为笛卡尔力控模式")

                # ---- force-control phase debounce ----
                self._fc_on = False
                self._p1_streak = 0          # phase=1 连续计数（允许夹杂 phase=2 毛刺）
                self._p2_run = 0             # phase=2 连续帧计数（>=10 -> 强制关力控）

                # ---- tunables ----
                self.FC_ENTER_STREAK = 30    # 连续(容忍phase2毛刺) >=15 才开力控
                self.FC_P2_OFF_CONSEC = 10   # phase=2 连续>=10 帧 -> 关力控
                self.FC_TARGET_FZ = -10.0


        except Exception as e:
            #self.log.error("Error occurred while connecting to robot server: %s" % str(e))
            return None
        
    def ZeroFTSensor(self):
        print(666999)
        self.robot.SwitchMode(self.mode.NRT_PRIMITIVE_EXECUTION)
        print(666888)
        self.robot.ExecutePrimitive("ZeroFTSensor", dict())
        
        while not self.robot.primitive_states()["terminated"]:
            time.sleep(1)
        logger.info("Sensor zeroing complete")
        
    def clear_fault(self):
        # Fault Clearing
        # ==========================================================================================
        # Check if the robot has fault
        if self.robot.isFault():
            logger.warning("Fault occurred on robot server, trying to clear ...")
            #self.log.warn("Fault occurred on robot server, trying to clear ...")
            # Try to clear the fault
            self.robot.ClearFault()
            time.sleep(2)
            # Check again
            if self.robot.isFault():
                #self.log.error("Fault cannot be cleared, exiting ...")
                return
            #self.log.info("Fault on robot server is cleared")
            
    # def get_current_robot_states(self) -> flexivrdk.RobotStates:
    #     # 返回flexivAPI下机械臂当前states
    #     print(777777777777777)
    #     self.robot.getRobotStates(self.robot_states)
    #     print(8888888888888888)
    #     return self.robot_states
    
    def get_current_robot_states(self):
        # 返回新版 RDK 的 robot.states()
        #print(777777777777777)
        states = self.robot.states()   # 新版 API
        #print(8888888888888888)
        return states

    def get_current_gripper_states(self) -> flexivrdk.GripperStates:
        # 返回flexivAPI下机械臂当前gripper states
        #self.gripper.getGripperStates(self.gripper_states)
        return self.gripper.states().is_moving

    def get_current_gripper_force(self) -> float:
        #self.gripper.getGripperStates(self.gripper_states)
        return self.gripper.states().force

    def get_current_gripper_width(self) -> float:
        #self.gripper.getGripperStates(self.gripper_states)
        return self.gripper.states().width

    def get_current_q(self) -> List[float]:
        # 返回flexivAPI下机械臂当前joints值
        self.robot.getRobotStates(self.robot_states)
        return self.robot_states.q
    
    def get_current_tcp(self) -> List[float]:
        # 返回flexivAPI下机械臂当前tcp值
        self.robot.getRobotStates(self.robot_states)
        return self.robot_states.tcpPose

    def move(self, target_q):
        v = [1.5]*self.DOF #速度限制
        a = [0.8]*self.DOF #加速度限制
        self.robot.sendJointPosition(
                target_q,
                [0.0]*self.DOF,
                [0.0]*self.DOF,
                v,
                a)
        
    def quat2eulerZYX(quat, degree=True):
        print("quat",quat)
        eulerZYX = (
            R.from_quat([quat[3], quat[0], quat[1], quat[2]])
            .as_euler("xyz", degrees=degree)
            .tolist()
        )
        return eulerZYX
    
    def fix_quat_continuity(q, q_prev=None, eps=1e-12):
        q = np.asarray(q, dtype=np.float64)
        n = np.linalg.norm(q)
        if n < eps:
            return q_prev if q_prev is not None else np.array([1.0, 0.0, 0.0, 0.0])  # 建议默认 [w,x,y,z]
        q = q / n
        if q_prev is not None and np.dot(q_prev, q) < 0:
            q = -q
        return q
            

    def tcp_move(self, target_tcp, phase_id=None):

        # 四元数顺序：w,x,y,z
        # target_tcp = np.asarray(target_tcp, dtype=np.float64)
        # pos = target_tcp[:3]
        # quat = target_tcp[3:7]
        # quat = self.fix_quat_continuity(quat, getattr(self, "_prev_quat", None))
        # self._prev_quat = quat.copy()
        # target_tcp[3:7] = quat
        # # target_tcp[3:] = [0.01265409 , 0.00165955,  0.99990422, -0.00534991]

            # phase_id 接收但不使用（保持接口兼容）
        _ = phase_id

        # 正常调用：目标扳手为 0（纯运动模式）
        self.robot.SendCartesianMotionForce(
            target_tcp,
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            0.1
        )
        print("机械臂执行一次tcp_move")

    # def tcp_move(self, target_tcp, phase_id=None): #擦白板
    #     # phase_id 统一为 int 或 None
    #     try:
    #         phase = None if phase_id is None else int(phase_id)
    #     except Exception:
    #         phase = None
        

    #     # hard rule: no phase_id -> never enable force control
    #     if phase is None:
            
    #         self.robot.SendCartesianMotionForce(target_tcp, [0.0] * 6, 0.1)

    #         return


    #     # ----------------------------
    #     # phase 去抖 + 开关力控逻辑
    #     # ----------------------------
    #     if not self._fc_on:
    #         # OFF -> ON：需要 phase=1 “稳定出现”累计 >=15
    #         # phase=2 允许夹杂，不打断计数；其它 phase 打断计数
    #         if phase == 1:
    #             self._p1_streak += 1
    #         elif phase == 2:
    #             # 允许夹杂：不增加p1_streak，也不清零
    #             pass
    #         else:
    #             self._p1_streak = 0

    #         # 进入力控前，phase=2 连续计数也没意义，重置
    #         self._p2_run = 0

    #         if self._p1_streak >= self.FC_ENTER_STREAK:
    #             self._fc_on = True
    #             self._p2_run = 0  # 进入力控后重新开始统计 phase2 连续段

    #     else:
    #         # ON 保持：phase=1 继续保持；phase=2 视为毛刺，但连续>=10则退出
    #         if phase == 2:
    #             self._p2_run += 1
    #             if self._p2_run >= self.FC_P2_OFF_CONSEC:
    #                 # phase2 连续太久：关力控
    #                 self._fc_on = False
    #                 self._p1_streak = 0
    #                 self._p2_run = 0
    #         else:
    #             # 只要不是 phase=2，就清掉 phase2 连续计数
    #             self._p2_run = 0

    #             # 如果 phase 不是1（比如0/None/3...），你希望是否立即关？
    #             # 这里我选择：非1非2 也关（更安全）。如果你不想立刻关，把下面两行删掉。
    #             if phase not in (1, 2):
    #                 self._fc_on = False
    #                 self._p1_streak = 0

    #     # ----------------------------
    #     # 执行力控 / 非力控i
    #     # ----------------------------

    #     if self._fc_on:
    #         self.robot.SetForceControlFrame(flexivrdk.CoordType.TCP)
    #         self.robot.SetForceControlAxis([False, False, True, False, False, False])
    #         self.robot.SetMaxContactWrench([float("inf")] * 6)
    #         self.robot.SendCartesianMotionForce(
    #             target_tcp,
    #             [0.0, 0.0, float(self.FC_TARGET_FZ), 0.0, 0.0, 0.0],
    #             0.05
    #         )
    #         print(f"[tcp_move] raw_phase={phase} -> FORCE_CTRL_ON "
    #             f"(p1_streak={self._p1_streak}, p2_run={self._p2_run})")
    #     else:
    #         self.robot.SetForceControlFrame(flexivrdk.CoordType.TCP)
    #         self.robot.SetForceControlAxis([False, False, False, False, False, False])
    #         self.robot.SetMaxContactWrench([float("inf")] * 6)
    #         self.robot.SendCartesianMotionForce(
    #             target_tcp,
    #             [0.0] * 6,
    #             0.08
    #         )
    #         print(f"[tcp_move] raw_phase={phase} -> FORCE_CTRL_OFF "
    #             f"(p1_streak={self._p1_streak}, p2_run={self._p2_run})")


        

    
    def MovePTP(self,position,euler, jntVelScale=20):
        """
        执行点到点运动(PTP)

        参数:
            position (list): 目标位置[x,y,z]，单位为米
            euler (list): 目标姿态[rx,ry,rz]，单位为度
            jntVelScale (int): 关节速度尺度，范围[1-100]，默认为20

        功能:
            - 控制机器人以点到点方式运动到目标位置和姿态
            - 使用当前关节位置作为参考
            - 使用WORLD坐标系和WORLD_ORIGIN参考系
            - 等待运动完成
        """
        self.robot.ExecutePrimitive(
                    "MovePTP",
                    {
                        "target": flexivrdk.Coord(
                            position, euler, ["WORLD", "WORLD_ORIGIN"]
                        ),
                    "jntVelScale": jntVelScale,
                    "refJntPos": flexivrdk.JPos(self.read_joint(True))
                    },
                )
        # Wait for reached target
        while not self.robot.primitive_states()["reachedTarget"]:
            time.sleep(0.01)
        self.logger.info("Executing primitive: MovePTP")
        
    def MoveL(self, position,euler, speed=0.01, acc=0.1):
        """
        执行tcp直线运动

        参数:
            position (list): 目标位置[x,y,z]，单位为米
            euler (list): 目标姿态[rx,ry,rz]，单位为度
            speed (float): 运动速度，范围[0.001, 2.2]m/s，默认为0.1
            acc (float): 运动加速度，范围[0.1, 3.0]m/s²，默认为0.1

        功能:
            - 控制机器人TCP沿直线路径运动到目标位置和姿态
            - 使用WORLD坐标系和WORLD_ORIGIN参考系
            - 等待运动完成
        """
        self.robot.ExecutePrimitive(
                    "MoveL",
                    {
                        "target": flexivrdk.Coord(
                            position, euler, ["WORLD", "WORLD_ORIGIN"]
                        ),
                        "vel": speed,
                        "acc": acc
                    },
                )
        # Wait for reached target
        while not self.robot.primitive_states()["reachedTarget"]:
            time.sleep(0.01)
        self.logger.info("Executing primitive: MoveL")
        
        
    def execute_primitive(self, primitive_command: str):
        self.robot.SwitchMode(self.mode.NRT_PRIMITIVE_EXECUTION)
        self.robot.ExecutePrimitive(primitive_command)
        while self.parse_pt_states(self.robot.primitive_states(), "reachedTarget") != "1":
            time.sleep(0.1)
        self.robot.SwitchMode(self.mode.NRT_CARTESIAN_MOTION_FORCE)

    @staticmethod
    def parse_pt_states(pt_states, parse_target):
        """
        Parse the value of a specified primitive state from the pt_states string list.
        Parameters
        ----------
        pt_states : list of str
            Primitive states string list returned from Robot::getPrimitiveStates().
        parse_target : str
            Name of the primitive state to parse for.
        Returns
        ----------
        str
            Value of the specified primitive state in string format. Empty string is
            returned if parse_target does not exist.
        """
        for state in pt_states:
            words = state.split()
            if words[0] == parse_target:
                return words[-1]
        return ""
    def switch_PRIMITIVE_Mode(self):
        """
        切换机器人到PRIMITIVE执行模式，使用MOVEL等指令，都需先调用这个

        功能:
            - 将机器人模式切换为NRT_PRIMITIVE_EXECUTION
            - 用于执行基本运动指令
        """
        self.robot.SwitchMode(self.mode.NRT_PRIMITIVE_EXECUTION)
    
    def move_gripper_safe(self, width: float, speed: float = 0.1, force: float = 10.0,
                        width_min: float = 0.0, width_max: float = 0.10,
                        speed_min: float = 0.001, speed_max: float = 0.20,
                        force_min: float = 0.0, force_max: float = 80.0):
        """
        线程安全 + 参数裁剪的夹爪控制接口
        - 解决 FastAPI 并发请求导致的 gripper 非线程安全问题
        - 统一 server 侧调用接口：bimanual_flexiv_server 会调用这个名字
        """
        if not hasattr(self, "gripper"):
            raise RuntimeError("Gripper not initialized: self.gripper not found")

        # ---- sanitize / clamp ----
        w = float(width)
        v = float(speed)
        f = float(force)

        if w < width_min: w = width_min
        if w > width_max: w = width_max

        if v < speed_min: v = speed_min
        if v > speed_max: v = speed_max

        # force_limit 一般用 [0, 80] 更合理；负值在很多实现里语义不清
        if f < force_min: f = force_min
        if f > force_max: f = force_max

        # ---- serialize gripper calls ----
        lock = getattr(self, "_gripper_lock", None)
        if lock is None:
            # 兼容老代码：即便你忘加 lock，也不至于直接崩
            self._gripper_lock = threading.Lock()
            lock = self._gripper_lock

        with lock:
            self.gripper.Move(w, v, f)
            return {"ok": True, "width": w, "velocity": v, "force_limit": f}

    def gripper_states_safe(self):
        # 兼容老接口：即使底层偶发读失败，也别让上层接口直接炸
        try:
            return self.gripper_states()
        except Exception as e:
            # 这里按你项目的 logger 来（loguru / logging）
            try:
                from loguru import logger
                logger.warning(f"[GRIPPER] gripper_states failed: {e}")
            except Exception:
                pass
            return None

    def Move_gripper(self, width, speed=0.1, force=10.0):
        """
        控制夹爪运动

        参数:
            width (float): 目标宽度，范围[0,0.1]米
            speed (float): 运动速度，范围[0.001,0.2]m/s，默认为0.1
            force (float): 接触力，范围[-80,80]N，默认为10.0

        功能:
            - 控制夹爪运动到指定宽度
            - 等待2秒以确保夹爪动作完成
        """
        self.gripper.Move(width, speed, force)
        #time.sleep(2)  # 等待夹爪动作完成
if __name__ == '__main__':
    
    ROBOT_SN = 'Rizon4s-062958'      
    GRIPPER_NAME = 'Flexiv-GN01' 
    robot = FlexivController(ROBOT_SN, GRIPPER_NAME, frequency = 100.0, remote_control=True, gripper_init=False)
    # 切换到PRIMITIVE模式以执行MoveL等指令
    robot.switch_PRIMITIVE_Mode()
    print("✓ 机器人初始化完成并进入PRIMITIVE模式。")
    robot.Move_gripper(0,0.1,10)