from ui_shop import *
import sys
from multiprocessing import Process
import multiprocessing
import cv2
import kinematics
import Trajectory_generation
import uart
from track_fun import kcftracker

# 在示教器中进行的robot进程 包括了关节空间和笛卡尔空间的内容
class real_robot_Thread(Process):
    def __init__(self,target_Ds,com,now_point,conn=None):
        # 当此处的 flag = 0 时，指向 target_Ds 即forward
        # 当此处的 flag = 1 时，指向 target_Ps 即inverse
        super(real_robot_Thread,self).__init__()
        self.target_Ds = target_Ds
        self.com = com
        self.now_point = now_point
        self.conn = conn

    def run(self):
        self.setup_robot()

    def setup_robot(self):
        # 下面要使用 while 是因为此处要一直监听Qt发来的关节角的变化信息
        # 如果此时传入的是角度
        # 则调用正运动学
        # 初始值
        temp = list(self.target_Ds)
        while (1):
            if self.conn != None:
                _ = self.conn.recv()
                theta = [0 for i in range(5)]
                for i in range(5):
                    theta[i] = self.target_Ds[i]
                order, flag = kinematics.get_order(theta, now_point=self.now_point)
                print(order)
                uart.go(order, self.com)
            else:
                diff_array = np.where(np.array(temp)!=np.array(self.target_Ds),1,0)
                if np.sum(diff_array)!=0:
                    for i in range(5):
                        self.now_point[i] = self.target_Ds[i]
                    idx = list(diff_array).index(1)
                    order, _ = kinematics.get_one_order(idx,self.target_Ds[idx])
                    uart.go(order,self.com)
                    temp = list(self.target_Ds)
                    print(order)


# 创建仿真环境（配合示教器） 包括了关节空间和笛卡尔空间的内容
class sim_Thread(Process):
    def __init__(self,target_Ds,flag):
        # 当此处的 flag = 0 时，指向  关节空间
        # 当此处的 flag = 1 时，指向 笛卡尔空间
        super(sim_Thread,self).__init__()
        self.target_Ds = target_Ds
        self.flag = flag

    def run(self):
        self.setup_sim()

    def setup_sim(self):
        import pybullet as p
        import pybullet_data
        cid = p.connect(p.GUI)
        # urdf 的路径
        # 注意更改 urdf_path
        path = "C:/Users/LZC-solo/Desktop/robot/RobotSimu/RobotSimu/urdf/RobotSimu.urdf"
        # 添加资源
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF("plane.urdf")
        robot_id = p.loadURDF(path, basePosition=[0, 0, 0], useFixedBase=True)

        # 打印标题
        p.addUserDebugText(
            text="5 DOF Robot Simulated",
            textPosition=[0, 0, 0.6],
            textColorRGB=[0.8, 0, 0],
            textSize=1.2,
        )
        p.setGravity(0, 0, -10)
        p.setRealTimeSimulation(1)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        DH_init_angle = [0, np.pi/2, 0, np.pi/2, 0]
        target_Ds_sim = [0,0,0,0,0]

        # 12-19
        # lzc 改
        idxs = [i for i in range(5)]

        p.setJointMotorControlArray(robot_id, idxs, controlMode=p.POSITION_CONTROL, targetPositions=[0,0,0,0,0],
                                            targetVelocities=[0 for i in range(5)], forces=[500 for i in range(5)])

        for i in range(5):
            target_Ds_sim[i] = np.array(self.target_Ds[i]) - np.array(DH_init_angle[i])

        temp = target_Ds_sim.copy()
        while True:
            p.stepSimulation()
            for i in range(5):
                target_Ds_sim[i] = np.array(self.target_Ds[i]) - np.array(DH_init_angle[i])
            if self.flag == 0:
                p.setJointMotorControlArray(robot_id, idxs, controlMode=p.POSITION_CONTROL, targetPositions=target_Ds_sim,
                                            targetVelocities=[0 for i in range(5)], forces=[500 for i in range(5)])

            else:
                diff_array = np.where(np.array(temp) != np.array(target_Ds_sim), 1, 0)
                if np.sum(diff_array) != 0:
                    p.setJointMotorControlArray(robot_id, idxs, controlMode=p.POSITION_CONTROL,
                                                targetPositions=target_Ds_sim,targetVelocities=[0 for i in range(5)], forces=[500 for i in range(5)])
                    temp = target_Ds_sim.copy()


        p.disconnect(cid)

# 创建仿真环境（配合轨迹规划）  包括了关节空间和笛卡尔空间的内容
class sim_path_Thread(Process):
    def __init__(self,pase_list,conn):
        super(sim_path_Thread,self).__init__()
        self.pase_list = pase_list
        self.conn = conn

    def run(self):
        self.setup_sim()

    def setup_sim(self):
        import pybullet as p
        import pybullet_data
        cid = p.connect(p.GUI)
        # urdf 的路径
        path = "C:/Users/LZC-solo/Desktop/robot/RobotSimu/RobotSimu/urdf/RobotSimu.urdf"
        # 添加资源
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF("plane.urdf")
        robot_id = p.loadURDF(path, basePosition=[0, 0, 0], useFixedBase=True)

        # 打印标题
        p.addUserDebugText(
            text="5 DOF Robot Simulated",
            textPosition=[0, 0, 0.6],
            textColorRGB=[0.8, 0, 0],
            textSize=1.2,
        )
        p.setGravity(0, 0, -10)
        p.setRealTimeSimulation(1)
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        idxs = [i for i in range(5)]
        p.setJointMotorControlArray(robot_id, idxs, controlMode=p.POSITION_CONTROL, targetPositions=[0,0,0,0,0],
                                    targetVelocities=[0 for i in range(5)], forces=[500 for i in range(5)])

        p.stepSimulation()

        # 初始化start点
        DH_init_angle = [0, np.pi / 2, 0, np.pi / 2, 0]
        target_Ds_sim = [0 for i in range(5)]

        while True:
            p.stepSimulation()
            # 接受数据
            _ = self.conn.recv()
            start_p_num = self.pase_list[0]
            start = self.pase_list[1]
            theta = self.pase_list[2]
            print("1")
            print("get")
            print(start_p_num)
            print(start)
            print(theta)
            point_num = theta.shape[0]
            if start_p_num != 0:
                for i in range(5):
                    target_Ds_sim[i] = start[0][i] - DH_init_angle[i]
                p.setJointMotorControlArray(robot_id, idxs, controlMode=p.POSITION_CONTROL,
                                            targetPositions=target_Ds_sim,
                                            targetVelocities=[0 for i in range(5)], forces=[500 for i in range(5)])
                time.sleep(2)

            for point_i in range(point_num):
                for i in range(5):
                    target_Ds_sim[i] = theta[point_i][i] - DH_init_angle[i]
                p.setJointMotorControlArray(robot_id, idxs, controlMode=p.POSITION_CONTROL,
                                            targetPositions=target_Ds_sim,
                                            targetVelocities=[0 for i in range(5)], forces=[500 for i in range(5)])
                time.sleep(0.08)

        p.disconnect(cid)

class qt_par_Thread(Process):
    def __init__(self,target_Ds,now_point=None,com=None):
        super(qt_par_Thread, self).__init__()
        self.target_Ds = target_Ds
        self.now_point = now_point
        self.com = com

    def run(self):
        self.setup_qt()

    def setup_qt(self):
        app = QApplication(sys.argv)
        MainWindow = par_form(self.target_Ds,self.now_point,self.com)
        MainWindow.show()
        sys.exit(app.exec_())

class qt_Car_Thread(Process):
    def __init__(self,target_Ds,target_Ps,now_point,debug_flag,conn=None):
        super(qt_Car_Thread, self).__init__()
        self.target_Ds = target_Ds
        self.target_Ps = target_Ps
        self.now_point = now_point
        self.debug_flag = debug_flag
        self.conn = conn

    def run(self):
        self.setup_qt()

    def setup_qt(self):
        app = QApplication(sys.argv)
        MainWindow = Car_form(self.target_Ds,self.target_Ps,self.now_point,self.debug_flag,self.conn)
        MainWindow.show()
        sys.exit(app.exec_())

class qt_tra_form(Process):
    def __init__(self, pase_list,conn):
        super(qt_tra_form, self).__init__()
        self.pase_list = pase_list
        self.conn = conn

    def run(self):
        self.setup_qt()

    def setup_qt(self):
        app = QApplication(sys.argv)
        MainWindow = tra_form(pase_list=self.pase_list,conn=self.conn)
        MainWindow.show()
        sys.exit(app.exec_())


