import time
import threading
# import spdlog
import flexivrdk
import math
# from scipy.spatial.transform import Rotation as R

class FlexivRobot:

    def __init__(self, robot_sn: str, gripper_name:str,frequency: float = 100.0, gripper_init: bool = False,remote_control: bool = False):
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
        self.robot_sn = robot_sn
        self.frequency = frequency
        # self.logger = spdlog.ConsoleLogger("FlexivRobot")
        self.mode = flexivrdk.Mode
        #建立连接
        self.robot = flexivrdk.Robot(self.robot_sn)
        if remote_control:
            # Clear fault on the connected robot if any
            if self.robot .fault():
                # self.logger.warn("Fault occurred on the connected robot, trying to clear ...")
                # Try to clear the fault
                if not self.robot.ClearFault():
                    # self.logger.error("Fault cannot be cleared, exiting ...")
                    return 1
                # self.logger.info("Fault on the connected robot is cleared")
            # self.logger.info("Enabling robot ...")
            self.robot.Enable()
            while not self.robot.operational():
                time.sleep(1)

            self.gripper = flexivrdk.Gripper(self.robot)

            self.tool = flexivrdk.Tool(self.robot)
            self.gripper.Enable(gripper_name)

            #初始化，在开机时可以先在示教器上手动初始化
            if gripper_init:
                self.gripper.Init()
                time.sleep(10) #等待夹爪初始化完成

            # self.logger.info("Enabling gripper")
            # #切换tcp至夹爪坐标系,默认坐标系是与示教器上一致
            self.tool.Switch(gripper_name)

        # self.logger.info("Robot is now operational")

    def Stop(self):
        """
        停止机器人和夹爪

        功能:
            - 停止机器人
            - 停止夹爪
        """
        self.robot.Stop()
        self.gripper.Stop()
        # self.logger.info("Robot is stopped")

    def quat2eulerZYX(quat, degree=False):
        """
        将四元数转换为欧拉角(ZYX顺序)

        参数:
            quat (list): 四元数，格式为[w,x,y,z]
            degree (bool): 是否返回角度值,默认为False(返回弧度)

        返回:
            list: 欧拉角[x,y,z],默认为弧度,若degree=True则为角度

        注意:
            - 使用scipy库进行转换,scipy使用[x,y,z,w]顺序表示四元数
            - 转换结果为xyz外旋顺序的欧拉角
        """
        eulerZYX = (
            R.from_quat([quat[1], quat[2], quat[3], quat[0]])
            .as_euler("xyz", degrees=degree)
            .tolist()
        )

        return eulerZYX

    def quat2matrix(self,quat):
        """
        将四元数转换为3x3旋转矩阵

        参数:
            quat (list or np.ndarray): 四元数，格式为[w,x,y,z]

        返回:
            np.ndarray: 3x3旋转矩阵

        注意:
            - 使用scipy库进行转换，scipy使用[x,y,z,w]顺序表示四元数
        """
        # 同样，需要将 [w, x, y, z] 转换为 SciPy 的 [x, y, z, w] 格式
        r = R.from_quat([quat[1], quat[2], quat[3], quat[0]])

        # 直接调用 .as_matrix() 方法获取旋转矩阵
        rotation_matrix = r.as_matrix()

        return rotation_matrix

    def convert_radians_to_degrees(self,radian_list):
        """
        将弧度列表转换为角度列表

        参数:
            radian_list (list): 包含弧度值的列表

        返回:
            list: 包含角度值的列表
        """
        # 使用列表推导式和 math.degrees() 函数进行转换
        degree_list = [math.degrees(rad) for rad in radian_list]
        return degree_list

    def read_pose(self,Euler_flag=False):
        """
        读取机器人当前TCP位姿

        参数:
            Euler_flag (bool): 是否返回欧拉角表示的姿态，默认为False(返回四元数)

        返回:
            若Euler_flag=False: 返回完整的TCP位姿，包含位置[x,y,z]和四元数[w,x,y,z]
            若Euler_flag=True: 返回位置[x,y,z]和欧拉角[x,y,z](角度制),，可直接用于MOVEL

        """
        if Euler_flag:
            quat = self.robot.states().tcp_pose[3:7] #w,x,y,z
            euler = FlexivRobot.quat2eulerZYX(quat,degree=True) #x,y,z
            position = self.robot.states().tcp_pose[0:3]
            return position, euler#list x,y,z,rx,ry,rz
        else:
            return self.robot.states().tcp_pose

    def read_joint(self,degree_flag=False):
        """
        读取机器人当前关节角度

        参数:
            degree_flag (bool): 是否返回角度制，默认为False(返回弧度制)

        返回:
            list: 7个关节的角度值，默认为弧度制，若degree_flag=True则为角度制，返回值可直接用于MoveJ函数

        """
        if degree_flag:
            radian_list = self.robot.states().q #list 1-7
            degree_list = self.convert_radians_to_degrees(radian_list)
            return degree_list #list 1-7
        else:
            return self.robot.states().q #list 1-7

    def switch_PRIMITIVE_Mode(self):
        """
        切换机器人到PRIMITIVE执行模式，使用MOVEL等指令，都需先调用这个

        功能:
            - 将机器人模式切换为NRT_PRIMITIVE_EXECUTION
            - 用于执行基本运动指令
        """
        self.robot.SwitchMode(self.mode.NRT_PRIMITIVE_EXECUTION)

    def move_tcp_home(self) -> None:
        """
        让机械臂TCP回到初始（Home）位置。

        功能:
            - 发送 'Home' 原语指令，使机械臂TCP运动到默认的 Home 位置。
            - 阻塞等待，直到机器人到达目标位置。
        """

        self.robot.ExecutePrimitive("Home", {})
        while not self.robot.primitive_states()["reachedTarget"]:
            time.sleep(0.01)

    def MoveL(self, position,euler, speed=0.1, acc=0.1):
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
        # self.logger.info("Executing primitive: MoveL")

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
        # self.logger.info("Executing primitive: MovePTP")

    def MoveJ(self, target_joint, jntVelScale=20):
        """
        执行关节空间运动

        参数:
            target_joint (list): 目标关节角度，7个值，单位为度
            jntVelScale (int): 关节速度尺度，范围[1-100]，默认为20

        功能:
            - 控制机器人关节运动到目标关节角度
            - 等待运动完成
        """
        self.robot.ExecutePrimitive(
                    "MoveJ",
                    {
                        "target": flexivrdk.JPos(
                            target_joint
                        ),
                        "jntVelScale": jntVelScale
                    },
                )
        # Wait for reached target
        while not self.robot.primitive_states()["reachedTarget"]:
            time.sleep(0.01)
        self.logger.info("Executing primitive: MoveJ")

    def MoveL_multi_points(self, points_list, speed=0.1, acc=0.1, zoneRadius="Z100"):
        """
        执行多点笛卡尔空间运动

        参数:
            points_list (list): 多个目标位姿列表，最后一个元素为终点，其余为中间点
            jntVelScale (int): 关节速度尺度，范围[1-100]，默认为20

        功能:
            - 控制机器人关节依次经过多个中间点到达终点
            - 等待运动完成

        注意:
            - 如果points_list长度小于等于1，则不执行任何操作
        """
        if len(points_list) > 1:
            middle_points = []
            for point in points_list:
                position = point[:3]
                euler = point[3:]
                middle_points.append(flexivrdk.Coord(position, euler, ["WORLD", "WORLD_ORIGIN"]))
            self.robot.ExecutePrimitive(
                        "MoveL",
                        {
                            "target": flexivrdk.Coord(
                                points_list[-1][:3],
                                points_list[-1][3:],
                                ["WORLD", "WORLD_ORIGIN"]
                            ),
                            "waypoints": middle_points,
                            "speed": speed,
                            "acc": acc
                        },
                    )
            # Wait for reached target
            while not self.robot.primitive_states()["reachedTarget"]:
                time.sleep(0.01)
            self.logger.info("Executing primitive: MoveL")

    def MoveJ_multi_points(self, joints_list, jntVelScale=20):
        """
        执行多点关节空间运动

        参数:
            joints_list (list): 多个目标关节角度列表，最后一个元素为终点，其余为中间点
            jntVelScale (int): 关节速度尺度，范围[1-100]，默认为20

        功能:
            - 控制机器人关节依次经过多个中间点到达终点
            - 等待运动完成

        注意:
            - 如果joints_list长度小于等于1，则不执行任何操作
        """
        if len(joints_list) > 1:
            middle_points = []
            for joints in joints_list[:-1]:
                middle_points.append(flexivrdk.JPos(joints))
            self.robot.ExecutePrimitive(
                        "MoveJ",
                        {
                            "target": flexivrdk.JPos(
                                joints_list[-1]
                            ),
                            "waypoints": middle_points,
                            "jntVelScale": jntVelScale
                        },
                    )
            # Wait for reached target
            while not self.robot.primitive_states()["reachedTarget"]:
                time.sleep(0.01)
            self.logger.info("Executing primitive: MoveJ")

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
        time.sleep(2)  # 等待夹爪动作完成
if __name__ == '__main__':
    
    ROBOT_SN = 'Rizon4s-062958'      
    GRIPPER_NAME = 'Flexiv-GN01' 
    robot = FlexivRobot(ROBOT_SN, GRIPPER_NAME, frequency = 100.0, remote_control=True, gripper_init=False)
    # 切换到PRIMITIVE模式以执行MoveL等指令
    robot.switch_PRIMITIVE_Mode()
    print("✓ 机器人初始化完成并进入PRIMITIVE模式。")
    robot.Move_gripper(0,0.1,10)