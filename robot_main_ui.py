import sys
import time
import copy

import numpy as np

import kinematics
import uart
from Robot_ui_files.Ui_robot_ui import Ui_Robot_form
from ui_shop import *
from Process_shop import *
from multiprocessing import Process,Pipe,Manager
import multiprocessing
import serial
from track_fun import kcftracker

from cjp.vision_chess import robotplaychess
from cjp.follow import follow

# remark 位置为未写完位置

# 变量类型
# target_Ds 是角度的list 为弧度制
# target_Ps 是位置的list 为[x,y,z,pitch,roll] 为m为单位
# 在下面部分虽然时刻传入 角度和位置信息，但是时刻应该只有一个被调用
# 就是 fkine -> 使用角度
# ikine -> 使用位置


################ new message #######################
# 需要使用com口时，请调用 self.test_com() 函数
# com 会保持在self.com

# 主界面
class Robot_form(QMainWindow,Ui_Robot_form):
    def __init__(self):
        super(Robot_form,self).__init__()
        self.cap = None
        self.robotplaychess = robotplaychess()

        # 休眠计数器
        self.counter_x = 0
        self.counter_y = 0
        # 初始化取棋标志为0
        self.getflag = 0
        self.com = None
        self.frame = None
        # 如果Done 为 0 则表示不需要画矩形框
        # Done为 1 则表示要画矩形框
        self.Done = 0
        self.img_w = 0
        self.img_h = 0
        self.track_flag = 0
        self.tracker = kcftracker.KCFTracker(True, True, True)
        # 以下的 P2P 是百分比的关系
        # start_x start_y end_x end_y
        self.Rect_P2P = [0,0, 0, 0]
        # robost point
        self.rbsp = [0, 0]
        #
        self.rbsp_flag = 0
        self.get_flag = 0
        # 定义摄像头开关状态
        self.cap_flag = 0
        # 定义夹子的状态 1 为 hook || 0 为未 hook
        self.hook_state = 0
        # 定义 online 还是 offline
        # offline = 0
        # online = 1
        self.debug_flag = 0
        # 记录当前角度 (用于机器人)
        self.Ds_now = [0,np.pi/2,0,np.pi/2,0]
        # 当前点
        self.now_point = multiprocessing.Array('d', [0, np.pi/2, 0, np.pi/2, 0])

        self.setupUi(self)
        self.setWindowIcon(QIcon("./img_src/robot_ico.png"))
        self.big_sheet_h = self.img_sheet_big.height()
        self.big_sheet_w = self.img_sheet_big.width()
        self.panter_worker = UpdateData(0.05)
        self.connect_butt()

        self.initUi()

    def initUi(self):
        icon = QPixmap('./img_src/offline.png').scaled(35, 35, QtCore.Qt.KeepAspectRatio)
        self.state_ico.setPixmap(icon)
        self.update()

    def connect_butt(self):
        self.conn_butt.clicked.connect(self.test_com)
        # 视频刷新
        self.panter_worker.update_signal.connect(self.update)
        self.panter_worker.start()
        # 连接槽函数
        self.cap_state_butt.clicked.connect(self.change_cap)
        self.offline_rad.clicked.connect(self.offline)
        self.online_rad.clicked.connect(self.online)
        self.par_butt.clicked.connect(self.par_widget)
        self.car_butt.clicked.connect(self.pos_widget)
        self.guiji_butt.clicked.connect(self.tra_widget)
        self.reset_butt.clicked.connect(self.reset)
        self.hook_butt.clicked.connect(self.hook_2_unhook)
        self.fun_butt.clicked.connect(self.playchess)

    def mousePressEvent(self,event):
        print("press")
        if self.cap_flag == 1:
            if event.button() == Qt.LeftButton:
                self.track_flag = 0
                # 初始化 Rect_P2P
                for i in range(4):
                    self.Rect_P2P[i] = 0
                x = event.x()
                # 25 是主菜单栏的高度
                y = event.y() - 25
                label_x_min = self.img_sheet_big.x()
                label_y_min = self.img_sheet_big.y()
                label_x_max = self.img_sheet_big.x() + self.img_sheet_big.width()
                label_y_max = self.img_sheet_big.y() + self.img_sheet_big.height()
                # 如果在大框内，则开始画矩形，每张图只可以有一个矩形
                if (x-label_x_max)*(x-label_x_min) < 0 and (y-label_y_max)*(y-label_y_min) < 0:
                    delta_x = (x - label_x_min)/self.big_sheet_w
                    delta_y = (y - label_y_min)/self.big_sheet_h
                    self.Done = 1
                    self.Rect_P2P[0] = delta_x
                    self.Rect_P2P[1] = delta_y
                    self.Rect_P2P[2] = delta_x
                    self.Rect_P2P[3] = delta_y
                    self.rbsp_flag = 0
                    self.update()

            elif event.button() == Qt.RightButton:
                print("double")
                x = event.x()
                # 25 是主菜单栏的高度
                y = event.y() - 25
                label_x_min = self.img_sheet_big.x()
                label_y_min = self.img_sheet_big.y()
                label_x_max = self.img_sheet_big.x() + self.img_sheet_big.width()
                label_y_max = self.img_sheet_big.y() + self.img_sheet_big.height()
                # 如果在大框内，则开始画矩形，每张图只可以有一个矩形
                if (x - label_x_max) * (x - label_x_min) < 0 and (y - label_y_max) * (y - label_y_min) < 0:
                    print("in")
                    delta_x = (x - label_x_min) / self.big_sheet_w
                    delta_y = (y - label_y_min) / self.big_sheet_h
                    self.rbsp[0] = delta_x
                    self.rbsp[1] = delta_y
                    # 传入点
                    self.keep = follow(self.frame, self.rbsp[0], self.rbsp[1])
                    self.clicked_frame = copy.deepcopy(self.frame)

                    self.rbsp_flag = 1
                    self.track_flag = 0
                    self.Done = 0
                    self.update()

    def mouseMoveEvent(self,event):
        if self.cap_flag == 1:
            x = event.x()
            y = event.y() - 25

            label_x_min = self.img_sheet_big.x()
            label_y_min = self.img_sheet_big.y()
            label_x_max = self.img_sheet_big.x() + self.img_sheet_big.width()
            label_y_max = self.img_sheet_big.y() + self.img_sheet_big.height()
            # 如果在大框内
            if (x - label_x_max) * (x - label_x_min) < 0 and (y - label_y_max) * (y - label_y_min) < 0:
                delta_x = (x - label_x_min)/self.big_sheet_w
                delta_y = (y - label_y_min)/self.big_sheet_h
                self.Rect_P2P[2] = delta_x
                self.Rect_P2P[3] = delta_y
            self.update()

    def mouseReleaseEvent(self,event):
        if self.cap_flag == 1:
            if self.Done == 1:
                self.track_flag = 1

    def paintEvent(self,e):
        if self.cap_flag == 1:
            _,frame = self.cap.read()
            self.frame = frame
            height, width, depth = frame.shape
            self.img_h = height
            self.img_w = width
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = QImage(img.data, width, height, width * depth, QImage.Format_RGB888)
            self.pix = QPixmap.fromImage(img)
        else:
            self.img_sheet_big.setText("未开启摄像头，请点击右边控件")

        if self.track_flag == 1:
            ix = min(self.Rect_P2P[0], self.Rect_P2P[2]) * self.img_w
            iy = min(self.Rect_P2P[1], self.Rect_P2P[3]) * self.img_h
            w = abs(self.Rect_P2P[0] - self.Rect_P2P[2]) * self.img_w
            h = abs(self.Rect_P2P[1] - self.Rect_P2P[3]) * self.img_h
            print("pre_init")
            self.tracker.init([ix, iy, w, h], frame)
            print("初始化成功")
            self.track_flag = 2
        elif self.track_flag == 2:
            boundingbox = self.tracker.update(frame)
            self.Rect_P2P[0] = boundingbox[0]/self.img_w
            self.Rect_P2P[1] = boundingbox[1]/self.img_h
            self.Rect_P2P[2] = (boundingbox[0] + boundingbox[2])/self.img_w
            self.Rect_P2P[3] = (boundingbox[1] + boundingbox[3])/self.img_h

        if self.rbsp_flag == 1:
            # cjp
            if self.keep.read_init() == None:
                self.keep = follow(self.clicked_frame, self.rbsp[0], self.rbsp[1])
            print(self.rbsp)
            temp = self.keep.keep(self.frame)
            if temp[0] != None and temp[0] != 0:
                self.rbsp = temp
            print(self.rbsp)

            qp = QPainter(self.pix)
            w = self.pix.width()
            h = self.pix.height()

            qp.setPen(QPen(QColor(0, 0, 255), 5))
            circle_x = self.rbsp[0] * w
            circle_y = self.rbsp[1] * h
            r = 5
            qp.drawEllipse(QPointF(circle_x,circle_y),r,r)

        if self.Done == 1:
            qp = QPainter(self.pix)
            w = self.pix.width()
            h = self.pix.height()

            # 当已经开始跟踪
            # 当位于正中，用绿色框
            # 当未开始跟踪，或是开始跟踪，但是未在正中 用红色框
            if self.track_flag == 2:
                # 绿色？
                center_x = (self.Rect_P2P[0] + self.Rect_P2P[2])/2
                center_y = (self.Rect_P2P[1] + self.Rect_P2P[3])/2
                # 当框位于这一范围内的时候，为绿色
                lb = 0.4
                ub = 0.6
                if center_x > lb and center_x < ub and center_y > lb and center_y < ub:
                    qp.setPen(QPen(QColor(0, 255, 0), 5))
                else:
                    if center_x < lb:
                        scale_x = np.floor(abs(center_x - lb)/0.1) + 1
                        self.left_right(-1,scale_x)
                    elif center_x > ub:
                        scale_x = np.floor(abs(center_x - ub)/0.1) + 1
                        self.left_right(1,scale_x)
                    if center_y < lb:
                        scale_y = np.floor(abs(center_y - lb)/0.1) + 1
                        self.up_down(-1,scale_y)
                    elif center_y > ub:
                        scale_y = np.floor(abs(center_y - ub)/0.1) + 1
                        self.up_down(1,scale_y)

                    qp.setPen(QPen(QColor(255, 0, 0), 5))
            else:
                qp.setPen(QPen(QColor(255, 0, 0), 5))
            start_x = self.Rect_P2P[0] * w
            end_x = self.Rect_P2P[2] * w
            start_y = self.Rect_P2P[1] * h
            end_y = self.Rect_P2P[3] * h
            qp.drawRect(int(start_x), int(start_y), int(end_x - start_x), int(end_y - start_y))
        if self.cap_flag == 1:
            self.img_sheet_big.setPixmap(self.pix)

    def test_com(self):
        com_name = self.COM_edit.text()
        if len(com_name) == 0:
            self.com = None
            # 将目标跟踪功能效果关闭
            self.track_flag = 0
            self.Done = 0
            self.warn_w = warning_form("请输入COM口序号",1)
            self.warn_w.show()

        else:
            ser = serial.Serial()
            ser.port = com_name
            ser.baudrate = 115200
            ser.bytesize = 8
            ser.stopbits = 1
            ser.parity = "N"  # 奇偶校验位
            try:
                ser.open()
            except:
                pass
            if (ser.isOpen()):
                ser.close()
                self.online_rad.setChecked(True)
                self.debug_flag = 1
                print("1")
                icon = QPixmap('./img_src/online.png').scaled(35, 35, QtCore.Qt.KeepAspectRatio)
                self.state_ico.setPixmap(icon)
                if self.com == None:
                    self.success_w = success_form("串口打开成功")
                    self.success_w.show()
                self.com = com_name

            else:
                icon = QPixmap('./img_src/offline.png').scaled(35, 35, QtCore.Qt.KeepAspectRatio)
                self.state_ico.setPixmap(icon)
                self.com = None
                # 将目标跟踪功能效果关闭
                self.track_flag = 0
                self.Done = 0
                self.warn_w = warning_form("串口打开失败", 1)
                self.warn_w.show()
        print(com_name)

    def change_cap(self):
        if self.cap_flag == 0:
            # 打开摄像头
            self.cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
            self.cap_flag = 1
            self.cap_state_butt.setText("关闭摄像头")
        else:
            self.track_flag = 0
            self.Done = 0
            # 关闭摄像头
            self.cap_flag = 0
            self.cap.release()
            self.cap_state_butt.setText("开启摄像头")

    def offline(self):
        self.debug_flag = 0
        print("0")

    def online(self):
        self.test_com()
        self.debug_flag = 1
        print("1")

    # 关节示教器
    def par_widget(self):
        # 共享变量  各关节参数
        target_Ds = multiprocessing.Array('d', [0, np.pi/2, 0, np.pi/2, 0])

        process_list = []
        # 添加进程
        if self.debug_flag == 0:
            p = sim_Thread(target_Ds,0)
            p.start()
            process_list.append(p)
            # target_Ps z只有当离线时才要做
            p = qt_par_Thread(target_Ds)
            p.start()
            process_list.append(p)
        else:
            self.test_com()
            if self.com != None:
                for i in range(5):
                    target_Ds[i] = self.now_point[i]
                p = real_robot_Thread(target_Ds,self.com,self.now_point)
                p.start()
                process_list.append(p)
                # target_Ps z只有当离线时才要做
                p = qt_par_Thread(target_Ds,self.now_point,self.com)
                p.start()
                process_list.append(p)

    # 坐标示教器
    def pos_widget(self):
        # 共享变量  各关节参数
        target_Ds = multiprocessing.Array('d', [0, np.pi/2, 0, np.pi/2, 0])
        process_list = []
        # 添加进程

        if self.debug_flag == 0:
            target_Ps = kinematics.Pkinematics(target_Ds)
            p = sim_Thread(target_Ds,1)
            p.start()
            process_list.append(p)
            p = qt_Car_Thread(target_Ds, target_Ps, self.now_point, self.debug_flag)
            p.start()
            process_list.append(p)
        else:
            self.test_com()
            print(self.com)
            if self.com != None:
                conn1, conn2 = Pipe()
                target_Ps = kinematics.Pkinematics(self.now_point)

                p = real_robot_Thread(target_Ds,self.com,self.now_point,conn2)
                p.start()
                process_list.append(p)
                p = qt_Car_Thread(target_Ds,target_Ps,self.now_point,self.debug_flag,conn1)
                p.start()
                process_list.append(p)

    # 轨迹规划面板
    # 当为sim时，需要多开个进程
    # 当为实体机器人时，只需要单进程即可
    def tra_widget(self):
        process_list = []
        if self.debug_flag == 0:
            manager = Manager()
            pase_list = manager.list(range(3))
            conn1,conn2 = Pipe()
            p = sim_path_Thread(pase_list,conn1)
            p.start()
            process_list.append(p)
            p = qt_tra_form(pase_list=pase_list,conn=conn2)
            p.start()
            process_list.append(p)
        else:
            self.test_com()
            if self.com != None:
                self.tra_w = tra_form(com=self.com,now_point=self.now_point)
                self.tra_w.show()

    def left_right(self,flag,scale=1):
        # 如果 flag 为 1 向左转  +
        # 如果 flag 为 -1 向右转  -
        self.test_com()
        if self.com != None:
            self.counter_x = self.counter_x + 1
            if self.counter_x >= 4:
                self.counter_x = 0
                lb = -135/180*np.pi
                ub = 135/180*np.pi
                delta = 2/180*np.pi*scale
                base_theta = self.now_point[0]
                if flag == -1:
                    if base_theta + delta < ub:
                        theta = base_theta + delta
                        self.now_point[0] = theta
                        order,flag = kinematics.get_one_order(0, theta,200)
                        uart.go(order, self.com)
                else:
                    if base_theta - delta > lb:
                        theta = base_theta - delta
                        self.now_point[0] = theta
                        order,flag = kinematics.get_one_order(0, theta,200)
                        uart.go(order, self.com)

    def up_down(self,flag,scale=1):
        # 如果 flag 为 1 低头  +
        # 如果 flag 为 -1 抬头  -
        self.test_com()
        if self.com != None:
            self.counter_y = self.counter_y + 1
            if self.counter_y >= 4:
                self.counter_y = 0
                lb = -135 / 180 * np.pi
                ub = 135 / 180 * np.pi
                delta = 2 / 180 * np.pi *scale
                base_theta = self.now_point[3]
                if flag == -1:
                    if base_theta + delta < ub:
                        theta = base_theta + delta
                        self.now_point[3] = theta
                        order, flag = kinematics.get_one_order(3, theta, 200)
                        uart.go(order, self.com)
                else:
                    if base_theta - delta > lb:
                        theta = base_theta - delta
                        self.now_point[3] = theta
                        order, flag = kinematics.get_one_order(3, theta, 200)
                        uart.go(order, self.com)

    def reset(self):
        self.test_com()
        if self.com != None:
            theta = [0, np.pi/2, 0, np.pi/2, 0]
            order,flag = kinematics.get_order(theta, now_point=self.now_point)
            print(order)
            uart.go(order,self.com)
            print(self.now_point)

    def hook_2_unhook(self):
        self.test_com()
        if self.com != None:
            if self.hook_state == 0:
                self.hook_state = 1
                # #######
                # 2021.12.20
                # 调用hook函数 com 为 self.com
                # order = "#005P1400T1000!" # 大棋子刚好夹紧
                order = "#005P1610T1000!"   # 小棋子刚好夹紧
                uart.go(order, self.com)
                # 将满足
                self.hook_butt.setText("松开")
            else:
                self.hook_state = 0
                # 调用松开函数
                order = "#005P1170T1000!"
                uart.go(order, self.com)
                # #######
                # 2021.12.20

                self.hook_butt.setText("夹取")

    def chesshook(self,flag):  #0夹1松  # 2021.12.21
        self.test_com()
        if self.com != None:
            if flag == 0:
                order = "#005P1430T1000!" # 大棋子刚好夹紧
                # order = "#005P1610T1000!"  # 小棋子刚好夹紧
                uart.go(order, self.com)

            else:

                order = "#005P1200T1000!"
                uart.go(order, self.com)

    def chessinit(self):
        # Ds = [80 / 1000, gety / 1000, -0.1, pitch / 180 * np.pi, roll]
        # theta, _ = kinematics.Ikinematics(Ds, l3=183)
        theta = [80/180*np.pi, 140/180*np.pi, -55/180*np.pi,0,0]
        order, flag = kinematics.get_order(theta, time=2000)
        uart.go(order, self.com)

        # cjp 下棋,点一下执行一次取棋和下棋?
    def playchess(self):
        # 先取棋子,输入图像获得目标棋子世界坐标
        # 初始位置
        roll = np.pi
        temp_round = -1

        while(1):

            time.sleep(3)
            round = self.robotplaychess.round(self.frame)

            if round != temp_round:

                if self.get_flag == 1:
                    getx, gety = self.robotplaychess.getchess(self.frame)
                    print(getx,gety)
                    pitch = self.robotplaychess.getpitch(getx, gety)
                    print('TSET:',getx, gety,pitch)
                    getx = getx + 10
                    if gety > 0:
                        gety = gety + 10
                    else:
                        gety = gety - 20
                    if getx is not None:
                        print('开始取棋')
                        # 对世界坐标进行处理，运动学解算，发送命令(判断工作空间
                        Ds = [getx / 1000, gety / 1000, -0.1, pitch / 180 * np.pi, roll]
                        theta, _ = kinematics.Ikinematics(Ds, l3=183)
                        order, flag = kinematics.get_order(theta,time=3000)
                        if flag:
                            uart.go(order, self.com)
                            time.sleep(6)
                            self.chesshook(0)  # 夹紧 待完成
                            time.sleep(3)
                            self.chessinit()
                            time.sleep(2)
                            # 机械臂回到一个固定位置不会挡到摄像头
                            self.get_flag = 1  # 如果每次进入这个判断都能夹成功， 否则要再加一个条件判断

                        else:
                            print('取棋失败')
                            self.chessinit()
                            self.get_flag = 0

                # 判断取棋标志为1 且机械臂回到初始位置 可以下棋
                if self.get_flag == 1:
                    playx, playy = self.robotplaychess.playc(self.frame)
                    pitch = self.robotplaychess.getpitch(playx, playy)
                    print('TSET:', playx, playy, pitch)
                    playx = playx + 20
                    if playy > 0:
                        playy = playy
                    else:
                        playy = playy - 20
                    if playx is not None:
                        print('开始下棋')
                        Ds = [playx / 1000, playy / 1000, -0.1, pitch / 180 * np.pi, roll]
                        # 对世界坐标进行处理，运动学解算，发送命令(判断工作空间
                        theta, _ = kinematics.Ikinematics(Ds, l3=193)
                        order, flag = kinematics.get_order(theta,time=3000)
                        if flag:
                            uart.go(order, self.com)
                            time.sleep(6)
                            self.chesshook(1)
                            time.sleep(3)
                            self.chessinit()
                            time.sleep(2)
                            # 执行命令 到达目标位置 放下棋子
                            self.get_flag = 0  # 成功下棋， 取棋标志置为0

                    else:
                        print('下棋失败')

                temp_round =  round
            else:
                temp_round = round



def run_main():
    print(cv2.__version__)
    app = QApplication(sys.argv)
    MainWindow = Robot_form()
    MainWindow.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    run_main()


