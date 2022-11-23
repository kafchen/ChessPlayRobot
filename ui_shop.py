from Robot_ui_files.Ui_Robot_sim_par import Ui_par_form
from Robot_ui_files.Ui_Car_form import Ui_Car_form
from Robot_ui_files.Ui_Warning import Ui_warning_form
from Robot_ui_files.Ui_trajectory_ui import Ui_tra_form
from Robot_ui_files.Ui_success import Ui_success_form
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import QtCore,QtWidgets
import numpy as np
import time
import Trajectory_generation
import uart
import kinematics

# 定义线程，用于实数更新 界面 中的数据
class UpdateData(QThread):
    update_signal = pyqtSignal()
    def __init__(self,wait_time):
        super(UpdateData,self).__init__()
        self.wait_time = wait_time

    def run(self):
        while True:
            self.update_signal.emit()
            time.sleep(self.wait_time)

# warning
class warning_form(QWidget,Ui_warning_form):
    comfirm_signal = pyqtSignal(str)
    def __init__(self,tip,butt_num):
        super(warning_form, self).__init__()
        # 定义 warning 显示的警告信息
        self.tip = tip
        # 定义 warning 下方的按钮数目
        self.butt_num = butt_num
        self.setupUi(self)
        self.initUi()
        self.connect_relation()

    def initUi(self):
        icon = QPixmap('./img_src/warning.png').scaled(60, 60, QtCore.Qt.KeepAspectRatio)
        self.waning_ico.setPixmap(icon)
        self.warning_tip.setText(self.tip)
        self.setWindowIcon(QIcon("./img_src/warning_ico.png"))

    def connect_relation(self):
        if self.butt_num == 1:
            self.cancel_butt.hide()
            self.confirm_butt.setFocus()
            self.confirm_butt.setShortcut(Qt.Key_Enter)
            self.confirm_butt.setShortcut(Qt.Key_Return)
            self.confirm_butt.clicked.connect(self.closeUi)
        else:
            self.cancel_butt.setFocus()
            self.cancel_butt.setShortcut(Qt.Key_Enter)
            self.cancel_butt.setShortcut(Qt.Key_Return)
            self.confirm_butt.clicked.connect(self.send_comfirm)
            self.cancel_butt.clicked.connect(self.closeUi)

    def send_comfirm(self):
        self.comfirm_signal.emit("True")
        self.closeUi()

    def closeUi(self):
        self.close()

# success
class success_form(QWidget,Ui_success_form):
    def __init__(self,tip):
        super(success_form,self).__init__()
        self.setupUi(self)
        self.tip = tip
        self.initUi()
        self.connect_relation()

    def initUi(self):
        icon = QPixmap('./img_src/success.png').scaled(60, 60, QtCore.Qt.KeepAspectRatio)
        self.success_ico.setPixmap(icon)
        self.success_tip.setText(self.tip)
        self.setWindowIcon(QIcon("./img_src/success_ico.png"))

    def connect_relation(self):
        self.confirm_butt.setFocus()
        self.confirm_butt.setShortcut(Qt.Key_Enter)
        self.confirm_butt.setShortcut(Qt.Key_Return)
        self.confirm_butt.clicked.connect(self.closeUi)

    def closeUi(self):
        self.close()

# 关节空间示教器界面
class par_form(QWidget,Ui_par_form):
    def __init__(self,target_Ds,now_point=None,com=None):
        super(par_form,self).__init__()
        self.qss_param = """
        QSlider::groove:horizontal {
        height: 12px;
        background: rgb(200, 200, 200)
        }
        QSlider::handle:horizontal {
        width: 10px;
        background: rgb(0, 160, 230);
        margin: -6px 0px -6px 0px;
        border-radius: 9px;
        }"""
        self.setupUi(self)
        self.setStyleSheet(self.qss_param)
        self.setWindowIcon(QIcon("./img_src/robot_ico.png"))
        icon = QPixmap('./img_src/robot_win_ico.png').scaled(60, 60, QtCore.Qt.KeepAspectRatio)
        self.win_ico.setPixmap(icon)
        self.decimal = 10
        self.target_Ds = target_Ds

        self.now_point = now_point
        self.com = com

        self.connect_butt()
        # 将 now_point 初始化为 示教器的游标值
        if self.com != None:
            for i in range(5):
                print(self.now_point[i])
            self.slide_1.setValue(now_point[0]/np.pi*180*self.decimal)
            self.slide_2.setValue(now_point[1]/np.pi*180*self.decimal)
            self.slide_3.setValue(now_point[2]/np.pi*180*self.decimal)
            self.slide_4.setValue(now_point[3]/np.pi*180*self.decimal)
            self.slide_5.setValue(now_point[4]/np.pi*180*self.decimal)
        # 对标签进行更新
        self.worker = UpdateData(0.01)
        self.worker.update_signal.connect(self.fresh_xyz)
        self.worker.start()

    def connect_butt(self):
        # 游标发生变化时的动作
        self.slide_1.valueChanged.connect(self.set_spinbox1)
        self.slide_2.valueChanged.connect(self.set_spinbox2)
        self.slide_3.valueChanged.connect(self.set_spinbox3)
        self.slide_4.valueChanged.connect(self.set_spinbox4)
        self.slide_5.valueChanged.connect(self.set_spinbox5)
        # 后面的赋值框发生变化的情况
        self.slide_v_1.valueChanged.connect(self.set_slide1)
        self.slide_v_2.valueChanged.connect(self.set_slide2)
        self.slide_v_3.valueChanged.connect(self.set_slide3)
        self.slide_v_4.valueChanged.connect(self.set_slide4)
        self.slide_v_5.valueChanged.connect(self.set_slide5)

        self.exit_butt.clicked.connect(self.close)
        self.reset_butt.clicked.connect(self.reset)

    # 刷新关节空间示教器的 xyz pr label
    def fresh_xyz(self):
        fk = kinematics.Pkinematics(self.target_Ds)
        Ps = np.array(fk[:3])*1000
        PR = np.array(fk[3:])/np.pi*180
        self.x_label.setText(str(round(Ps[0], 1)))
        self.y_label.setText(str(round(Ps[1], 1)))
        self.z_label.setText(str(round(Ps[2], 1)))

        self.p_label.setText(str(round(PR[0], 1)))
        self.r_label.setText(str(round(PR[1], 1)))

    # 将游标的值赋值给后面的框
    def set_spinbox1(self,v):
        self.target_Ds[0] = v*np.pi/(self.decimal*180)
        self.slide_v_1.setValue(v/self.decimal)

    def set_spinbox2(self, v):
        self.target_Ds[1] = v * np.pi / (self.decimal * 180)
        self.slide_v_2.setValue(v / self.decimal)

    def set_spinbox3(self,v):
        self.target_Ds[2] = v * np.pi / (self.decimal * 180)
        self.slide_v_3.setValue(v/self.decimal)

    def set_spinbox4(self,v):
        self.target_Ds[3] = v * np.pi / (self.decimal * 180)
        self.slide_v_4.setValue(v/self.decimal)

    def set_spinbox5(self,v):
        self.target_Ds[4] = v * np.pi / (self.decimal * 180)
        self.slide_v_5.setValue(v/self.decimal)

    def set_slide1(self,v):
        self.slide_1.setValue(v*self.decimal)

    def set_slide2(self,v):
        self.slide_2.setValue(v*self.decimal)

    def set_slide3(self,v):
        self.slide_3.setValue(v*self.decimal)

    def set_slide4(self,v):
        self.slide_4.setValue(v*self.decimal)

    def set_slide5(self,v):
        self.slide_5.setValue(v*self.decimal)

    def reset(self):
        # 复位
        if self.com != None:
            theta = [0, np.pi/2, 0, np.pi/2, 0]
            order,flag = kinematics.get_order(theta, now_point=self.now_point)
            print(order)
            uart.go(order,self.com)
        self.slide_1.setValue(0)
        self.slide_2.setValue(90 * self.decimal)
        self.slide_3.setValue(0)
        self.slide_4.setValue(90 * self.decimal)
        self.slide_5.setValue(0)

# 笛卡尔空间示教器
class Car_form(QWidget,Ui_Car_form):
    def __init__(self,target_Ds,target_Ps,now_point,debug_flag,conn=None):
        super(Car_form,self).__init__()
        self.decimal = 1000
        self.setupUi(self)
        self.setWindowIcon(QIcon("./img_src/robot_ico.png"))
        icon = QPixmap('./img_src/robot_win_ico.png').scaled(60, 60, QtCore.Qt.KeepAspectRatio)
        self.win_ico.setPixmap(icon)
        self.target_Ds = target_Ds
        self.target_Ps = target_Ps
        # 当为仿真模式时,可以改变 sim_now_point
        # 当为真实机器人时, 只可以读 now_point
        self.sim_now_point = [0,np.pi/2,0,np.pi/2,0]
        self.now_point = now_point
        # 0 为 sim
        self.debug_flag = debug_flag
        self.conn = conn
        # 用于判断是否存在输入
        self.temp1 = [0 for i in range(5)]
        theta, Flag = kinematics.Ikinematics(self.target_Ps)
        for i in range(5):
            self.temp1[i] = theta[i]

        # 初始化步长
        self.x_step = self.step_x.value()/self.decimal
        self.y_step = self.step_y.value()/self.decimal
        self.z_step = self.step_z.value()/self.decimal
        self.p_step = self.step_p.value()
        self.r_step = self.step_r.value()

        self.connect_butt()
        # 对标签进行更新
        self.worker = UpdateData(0.01)
        self.worker.update_signal.connect(self.fresh_xyz)
        self.worker.start()

    def fresh_xyz(self):
        theta,Flag = kinematics.Ikinematics(self.target_Ps)
        # 工作原理
        # 只有当逆解存在时,才改变 Ds
        if Flag == False:
            self.warn_w = warning_form("超出机械臂工作空间",1)
            self.warn_w.show()
            if self.debug_flag == 0:
                temp = kinematics.Pkinematics(self.sim_now_point)
            else:
                temp = kinematics.Pkinematics(self.now_point)
            for i in range(5):
                self.target_Ps[i] = temp[i]
        else:
            if self.debug_flag == 0:
                for i in range(5):
                    self.sim_now_point[i] = theta[i]
            for i in range(5):
                self.target_Ds[i] = theta[i]
            if theta != self.temp1:
                for i in range(5):
                    self.temp1[i] = theta[i]
                if self.conn != None:
                    self.conn.send("")
        self.X_label.setText(str(round(self.target_Ps[0]*self.decimal,1)))
        self.Y_label.setText(str(round(self.target_Ps[1]*self.decimal,1)))
        self.Z_label.setText(str(round(self.target_Ps[2]*self.decimal,1)))
        self.P_label.setText(str(round(self.target_Ps[3]*180/np.pi,1)))
        self.R_label.setText(str(round(self.target_Ps[4]*180/np.pi,1)))

    def connect_butt(self):
        self.add_x_butt.clicked.connect(self.add_x)
        self.add_y_butt.clicked.connect(self.add_y)
        self.add_z_butt.clicked.connect(self.add_z)
        self.add_p_butt.clicked.connect(self.add_p)
        self.add_r_butt.clicked.connect(self.add_r)
        self.del_x_butt.clicked.connect(self.del_x)
        self.del_y_butt.clicked.connect(self.del_y)
        self.del_z_butt.clicked.connect(self.del_z)
        self.del_p_butt.clicked.connect(self.del_p)
        self.del_r_butt.clicked.connect(self.del_r)

        self.step_x.valueChanged.connect(self.change_x)
        self.step_y.valueChanged.connect(self.change_y)
        self.step_z.valueChanged.connect(self.change_z)
        self.step_p.valueChanged.connect(self.change_p)
        self.step_r.valueChanged.connect(self.change_r)

        self.exit_butt.clicked.connect(self.close)

    def add_x(self):
        self.target_Ps[0] = self.target_Ps[0] + self.x_step

    def add_y(self):
        self.target_Ps[1] = self.target_Ps[1] + self.y_step

    def add_z(self):
        self.target_Ps[2] = self.target_Ps[2] + self.z_step

    def add_p(self):
        self.target_Ps[3] = (self.target_Ps[3]*180/np.pi + self.p_step)/180*np.pi

    def add_r(self):
        self.target_Ps[4] = (self.target_Ps[4]*180/np.pi + self.r_step)/180*np.pi

    def del_x(self):
        self.target_Ps[0] = self.target_Ps[0] - self.x_step

    def del_y(self):
        self.target_Ps[1] = self.target_Ps[1] - self.y_step

    def del_z(self):
        self.target_Ps[2] = self.target_Ps[2] - self.z_step

    def del_p(self):
        self.target_Ps[3] = (self.target_Ps[3]*180/np.pi - self.p_step)/180*np.pi

    def del_r(self):
        self.target_Ps[4] = (self.target_Ps[4]*180/np.pi - self.r_step)/180*np.pi

    def change_x(self,v):
        self.x_step = v/self.decimal

    def change_y(self,v):
        self.y_step = v/self.decimal

    def change_z(self,v):
        self.z_step = v/self.decimal

    def change_p(self,v):
        self.p_step = v

    def change_r(self,v):
        self.r_step = v

# 轨迹规划
class tra_form(QWidget,Ui_tra_form):
    def __init__(self,pase_list=None,conn=None,com= None,now_point=None):
        super(tra_form,self).__init__()
        self.pase_list = pase_list
        self.conn = conn
        self.com = com
        self.now_point = now_point
        self.setupUi(self)
        self.setWindowIcon(QIcon("./img_src/robot_ico.png"))
        self.start_p_num = 0
        self.mid_p_num = 0
        # 定义空间
        # 关节空间为0
        # 笛卡尔空间为1
        self.space = 0

        self.connect_butt()
        self.initUi()

    def initUi(self):
        if self.space == 0:
            title = ["关节一","关节二","关节三","关节四","关节五","耗时"]
        else:
            title = ["X","Y","Z","P","R","耗时"]
        if self.start_p_num == 0:
            self.start_label.hide()
            self.start_widget.hide()
        else:
            self.start_label.show()
            self.start_widget.show()
            self.fresh_table(self.start_widget,1,title,"start")
            title = None
        if self.mid_p_num == 0:
            self.mid_label.hide()
            self.mid_widget.hide()
        else:
            self.mid_label.show()
            self.mid_widget.show()
            self.fresh_table(self.mid_widget, self.mid_p_num, title)
            title = None

        self.fresh_table(self.target_widget,1, title)
        self.update()

    def connect_butt(self):
        self.rad_par.clicked.connect(self.change_par)
        self.rad_car.clicked.connect(self.change_car)
        self.change_start_butt.clicked.connect(self.change_start)
        self.add_mid_butt.clicked.connect(self.add_mid)
        self.del_mid_butt.clicked.connect(self.del_mid)
        self.submit_butt.clicked.connect(self.submit)
        self.exit_butt.clicked.connect(self.close)

    def fresh_table(self, table, row_num, table_title,flag="mid"):
        # 设置表格行数和列数
        table.setRowCount(row_num)
        for r in range(row_num):
            table.setRowHeight(r,30)
        table.setColumnCount(6)
        for col in range(6):
            table.setColumnWidth(col,73)

        # 设置表格的列标签
        if table_title != None:
            table.setHorizontalHeaderLabels(table_title)
            table.horizontalHeader().setVisible(True)
        else:
            table.horizontalHeader().setVisible(False)

        # 将行表头设为不可见
        table.verticalHeader().setVisible(False)
        # 设置表头颜色
        table.horizontalHeader().setStyleSheet("color: rgb(0, 0, 200);")
        # 表格颜色交错显示
        table.setAlternatingRowColors(True)
        # 选中整行
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        # 使得start的最后一个不可以编辑
        if flag=="start":
            item1 = QtWidgets.QTableWidgetItem()
            item1.setFlags(QtCore.Qt.ItemIsSelectable | QtCore.Qt.ItemIsEnabled)
            table.setItem(0, 5, item1)

    def change_par(self):
        self.space = 0
        self.initUi()

    def change_car(self):
        self.space = 1
        self.initUi()

    def change_start(self):
        if self.start_p_num == 0:
            self.start_p_num = 1
            self.change_start_butt.setText("删除初始点")
        else:
            self.start_p_num = 0
            self.change_start_butt.setText("增加初始点")
        self.initUi()

    def add_mid(self):
        self.mid_p_num = self.mid_p_num + 1
        self.initUi()

    def del_mid(self):
        if self.mid_p_num != 0:
            self.mid_p_num = self.mid_p_num - 1
        self.initUi()

    def submit(self):
        # flag 为 1则可用
        # 为 0 则表示输入内容有误
        flag = 1
        # 定义 列表 [[x,y,z,p,r]] 形式用于存放各个点的信息
        start = []
        mid_end = []
        # 消耗时间
        time_list = []
        if self.start_p_num == 1:
            temp = []
            for i in range(5):
                temp.append(self.start_widget.item(0,i))
            if None in temp:
                self.warn_w = warning_form("请补全数据",1)
                self.warn_w.show()
                flag = 0
            else:
                temp = []
                for i in range(5):
                    temp.append(self.start_widget.item(0, i).text())
                temp = list(map(float, temp))
                if self.space == 1:
                    temp[0:3] = list(np.array(temp[0:3]) / 1000)
                    temp[3:5] = list(np.array(temp[3:5]) / 180 * np.pi)
                else:
                    temp = list(np.array(temp) / 180 * np.pi)
            start.append(temp)
        else:
            temp = [0,np.pi/2,0,np.pi/2,0]
            if self.conn == None:
                for i in range(5):
                    temp[i] = self.now_point[i]
            if self.space == 1:
                temp = kinematics.Pkinematics(temp)
                start.append(temp)
            else:
                start.append(temp)

        if self.mid_p_num != 0:
            for row in range(self.mid_p_num):
                temp = []
                for i in range(5):
                    temp.append(self.mid_widget.item(row, i))
                if None in temp:
                    self.warn_w = warning_form("请补全数据",1)
                    self.warn_w.show()
                    flag = 0
                else:
                    temp = []
                    for i in range(5):
                        temp.append(self.mid_widget.item(row, i).text())
                    temp = list(map(float, temp))
                    if self.space == 1:
                        temp[0:3] = list(np.array(temp[0:3])/100)
                        temp[3:5] = list(np.array(temp[3:5])/180*np.pi)
                    else:
                        temp = list(np.array(temp) / 180*np.pi)
                mid_end.append(temp)

        temp = []
        for i in range(5):
            temp.append(self.target_widget.item(0, i))
        if None in temp:
            self.warn_w = warning_form("请补全数据", 1)
            self.warn_w.show()
            flag = 0
        else:
            temp = []
            for i in range(5):
                temp.append(self.target_widget.item(0, i).text())
            temp = list(map(float, temp))
            if self.space == 1:
                temp[0:3] = list(np.array(temp[0:3]) / 100)
                temp[3:5] = list(np.array(temp[3:5]) / 180 * np.pi)
            else:
                temp = list(np.array(temp) / 180 * np.pi)

        mid_end.append(temp)
        print("起始点")
        print(start)
        print("中间点和目标点")
        print(mid_end)
        print("填写正确")
        print(flag)

        # 获得消耗时间表
        time_list = []
        if self.mid_p_num != 0:
            for i in range(self.mid_p_num):
                try:
                    time_list.append(float(self.mid_widget.item(i, 5).text()))
                except:
                    time_list.append(1)
        try:
            time_list.append(float(self.target_widget.item(0, 5).text()))
        except:
            time_list.append(1)

        print("消耗时间表为")
        print(time_list)

        # 可用参数为 self.space 指定规划的空间信息
        # start 中为 [[]] 的形式 内部存储的为以 m/弧度 为单位的参数
        # mid_end  中为 [[],[],[]] 的形式 内部存的值为 m/弧度 为单位的中间点
        if flag == 1:
            if self.space == 0:
                sp_time = 0.1
                ang = mid_end
                ang[0:0] = start
                ang = np.array(ang).transpose()             # 角度
                print(ang)
                d_num = ang.shape[1]                # 点的个数， 就有 d_num - 1 段过程
                vel = np.zeros([5, d_num])      # 点上的速度，全0
                acc = 0.6*np.ones([5, d_num])     # 加速度， 全
                # times = 3 * np.ones(d_num-1)      # 每段的时间，全
                parameters = Trajectory_generation.joint_space(ang, vel, acc, time_list, d_num - 1)
                order, theta = Trajectory_generation.get_jointorder(parameters, time_list, sp_time)
                print(order)
                # 发送数据 (面向仿真)
                if self.conn != None:
                    self.pase_list[0] = self.start_p_num
                    self.pase_list[1] = start
                    self.pase_list[2] = theta
                    self.conn.send("")
                # 实体机器人操作
                else:
                    # 当有初始点时，go start point 后，sleep
                    # 没有初始点 直接 go
                    if self.start_p_num == 1:
                        uart.go(order[0], self.com)
                        time.sleep(2)
                    for one_order in order[1:]:
                        uart.go(one_order, self.com)
                        time.sleep(0.08)
                    for i in range(5):
                        self.now_point[i] = theta[-1][i]
            # 如果为笛卡尔空间
            else:

                print("开始规划......")
                sample = 50
                d_num = len(mid_end)
                print("mid_end")
                print(mid_end)
                print("start[0]")
                print(start[0])

                ka_fir = start[0]
                ka_end = mid_end[-1]
                ka_mid = mid_end[:-1]
                print(ka_fir)
                print(ka_end)
                print(ka_mid)
                for i in range(3):
                    ka_fir[i] = ka_fir[i] * 1000
                    ka_end[i] = ka_end[i] * 100
                    for j in range(d_num - 1):
                        ka_mid[j][i] = ka_mid[j][i]*100
                print("mid_end")
                print(mid_end)
                print("ka_fir")
                print(ka_fir)
                print("ka_end")
                print(ka_end)
                print("ka_mid")
                print(ka_mid)
                times = 2*np.ones(d_num)
                print("times")
                print(times)
                acc_times = 0.7*np.ones(d_num+1)
                print("轨迹参数生成......")
                cart_acc, vel_jk, uniform_times = Trajectory_generation.cartesian_space(ka_fir, ka_mid, ka_end, times, acc_times)
                print("vel_jk")
                print(vel_jk)
                print("轨迹路径生成......")
                x, sp_time = Trajectory_generation.cart_get_trace(ka_fir, vel_jk, cart_acc, uniform_times, times, sample, acc_times)
                print("x")
                print(x)
                orders,theta = Trajectory_generation.get_cartorder(x, sp_time)
                print("orders")
                print(orders)
                # 发送数据 (面向仿真)
                if self.conn != None:
                    self.pase_list[0] = self.start_p_num
                    self.pase_list[1] = start
                    self.pase_list[2] = np.array(theta)
                    print(np.array(theta))
                    self.conn.send("")
                # 实体机器人操作
                else:
                    # 当有初始点时，go start point 后，sleep
                    # 没有初始点 直接 go
                    if self.start_p_num == 1:
                        uart.go(orders[0], self.com)
                        time.sleep(2)
                    for one_order in orders[1:]:
                        uart.go(one_order, self.com)
                        time.sleep(0.08)
                    for i in range(5):
                        self.now_point[i] = theta[-1][i]
                # for order in orders:
                #     uart.go(order, self.com)
                #     time.sleep(0.09)




