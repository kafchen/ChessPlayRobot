import numpy as np
import math
dh_alpha = [0, np.pi / 2, 0, 0, np.pi / 2]
dh_a = [0, 0, 84, 76, 0]
dh_d = [0, 33, 0, -35, 93]
d_tool = 100

l1 = 84
l2 = 76
l3 = 93


def get_matrix_T(alpha, a, d, theta):
    t = np.zeros((4, 4))
    t[0, 0] = np.cos(theta)
    t[0, 1] = -np.sin(theta)
    t[0, 3] = a
    t[1, 0] = np.sin(theta)*np.cos(alpha)
    t[1, 1] = np.cos(theta)*np.cos(alpha)
    t[1, 2] = -np.sin(alpha)
    t[1, 3] = -np.sin(alpha)*d
    t[2, 0] = np.sin(theta)*np.sin(alpha)
    t[2, 1] = np.cos(theta)*np.sin(alpha)
    t[2, 2] = np.cos(alpha)
    t[2, 3] = np.cos(alpha)*d
    t[3, 3] = 1
    return t


def Pkinematics(theta):
    """
    正运动学
    :param theta: 五个角度
    :return: 基坐标系下的点坐标
    """
    # print("输入进正运动学的theta......")
    # print(theta)
    ang=[]
    for i in range(5):
        ang.append(theta[i]/np.pi*180)
    # print(ang)
    t01 = get_matrix_T(dh_alpha[0], dh_a[0], dh_d[0], theta[0])
    t12 = get_matrix_T(dh_alpha[1], dh_a[1], dh_d[1], theta[1])
    t23 = get_matrix_T(dh_alpha[2], dh_a[2], dh_d[2], theta[2])
    t34 = get_matrix_T(dh_alpha[3], dh_a[3], dh_d[3], theta[3])
    t45 = get_matrix_T(dh_alpha[4], dh_a[4], dh_d[4], theta[4])
    t05 = t01 @ t12 @ t23 @ t34 @ t45
    position = [t05[0, 3], t05[1, 3], t05[2, 3]]
    # print("t05 为：")
    # print(t05)
    # print("求正解位置为：" + str(position))

    roll = np.pi + theta[4]
    pitch = -(theta[1] + theta[2] + theta[3] - np.pi)
    # print("求正解pitch为" + str(pitch/np.pi*180))
    result = [t05[0, 3]/1000, t05[1, 3]/1000, t05[2, 3]/1000, pitch, roll]
    return result


def get_order(theta, time=1000, now_point=None):
    # 0,90,0,90,0
    order = "{"
    time = str(time)
    if len(time) == 3:
        T = "0" + time
    else:
        T = time
    idx = 0
    for i in theta:
        if idx == 1:
            P = int((i - 0.5 * np.pi) / (1.5 * np.pi) * 2000 + 1500)
            if P > 2500 or P < 500:
                print("错误!!关节" + str(idx) + "无法抵达" + str(theta))
                return
            P = str(P)
        else:
            if idx == 3:
                P = int((-i+0.5*np.pi) / (1.5 * np.pi)*2000 + 1500)
                if P > 2500 or P < 500:
                    print("错误!!关节" + str(idx) + "无法抵达" + str(theta))
                    return [], False

            else:
                if idx == 2:
                    P = int(-i / (1.5 * np.pi)*2000 + 1500)
                    if P > 2500 or P < 500:
                        print("错误!!关节" + str(idx) + "无法抵达" + str(theta))
                        return [], False
                else:
                    P = int(i / (1.5 * np.pi) * 2000 + 1500)
                    if P > 2500 or P < 500:
                        print("错误!!关节" + str(idx) + "无法抵达" + str(theta))
                        return [], False
        P = str(P)
        if len(P) == 3:
            order = order + "#00" + str(idx) + "P0" + P + "T" + T + "!"
        else:
            order = order + "#00" + str(idx) + "P" + P + "T" + T + "!"
        idx += 1
    order = order + "}"
    if now_point != None:
        for i in range(5):
            now_point[i] = theta[i]

    return order, True


def get_one_order(idx, theta, time=1000):
    order = ""
    time = str(int(float(time)))

    if len(time) == 3:
        T = "0" + time
    else:
        T = time

    if idx == 1:
        P = int((theta - 0.5 * np.pi) / (1.5 * np.pi) * 2000 + 1500)
        if P > 2500 or P < 500:
            print("错误!!关节" + str(idx) + "无法抵达" + str(theta))
            return [], False
        P = str(P)
    else:
        if idx == 3:
            P = int((-theta + 0.5 * np.pi) / (1.5 * np.pi) * 2000 + 1500)
            if P > 2500 or P < 500:
                print("错误!!关节" + str(idx) + "无法抵达" + str(theta))
                return [], False
            P = str(P)
        else:
            if idx == 2:
                P = int(-theta / (1.5 * np.pi) * 2000 + 1500)
                if P > 2500 or P < 500:
                    print("错误!!关节" + str(idx) + "无法抵达" + str(theta))
                    return [], False
            else:
                P = int(theta / (1.5 * np.pi) * 2000 + 1500)
                if P > 2500 or P < 500:
                    print("错误!!关节" + str(idx) + "无法抵达" + str(theta))
                    return [], False
            P = str(P)
    if len(P) == 3:
        order = order + "#00" + str(idx) + "P0" + P + "T" + T + "!"
    else:
        order = order + "#00" + str(idx) + "P" + P + "T" + T + "!"
    return order, True


def Ikinematics(Ds,l3=93, FLAG=0):    # (x, y, z, theta_xy, tool_ang)
    """
    几何法逆运动学
    :param x: 基于0坐标系的x
    :param y: 基于0坐标系的y
    :param z: 基于0坐标系的z
    :param theta_xy: 末端坐标系的xy平面与水平方向上的夹角
    :param tool_ang: 末端绕z轴旋转的度数
    :return: 返回各关节旋转角度
    """
    print("传进来IK的是......")
    print(Ds)
    x = Ds[0] * 1000
    y = Ds[1] * 1000
    z = Ds[2] * 1000
    theta_xy = Ds[3]
    tool_ang = Ds[4]
    temp = np.sqrt(x**2 + y**2 + z**2)
    # print("Ds")
    # print(Ds)
    ## theta1 已经完全正确
    if abs(x) < 0.0001:
        x = 0
    if abs(y) < 0.0001:
        y = 0
    if temp > 253.1:
        print("超出机械臂工作范围")
        return [],False
    theta1_ = math.atan2(y, x)
    theta1 = math.atan2(y - 2*np.cos(theta1_), x + 2*np.sin(theta1_))

    r = np.sqrt(x**2+y**2)
    print("r")
    print(r)

    # theta2
    a = r - l3*np.sin(theta_xy)
    b = z - l3*np.cos(theta_xy)
    c = -(l2**2-a**2-b**2-l1**2)/2/l1
    print("a**2+b**2-c**2")
    temp_ = a**2+b**2-c**2
    print(temp_)
    if temp_ < 0:
        temp = 0
    else:
        temp = np.sqrt(temp_)
    theta2_fir = math.atan2(a, b) + math.atan2(temp, c)
    theta2_sec = math.atan2(a, b) - math.atan2(temp, c)

    print("theta2_fir")
    print((np.pi/2-theta2_fir)*180/np.pi)
    print("theta2_sec")
    print((np.pi/2-theta2_sec)*180/np.pi)

    # 寻找正确的theta2角，选角度小的
    if FLAG==0:
        if theta2_fir > theta2_sec:
            theta2 = theta2_sec
        else:
            theta2 = theta2_fir
    else:
        if theta2_fir < theta2_sec:
            theta2 = theta2_sec
        else:
            theta2 = theta2_fir
    # theta2 = theta2_sec
    # 求解theta3

    c = -(l1**2-a**2-b**2-l2**2)/2/l2
    temp_ = a**2+b**2-c**2
    if temp_ < 0:
        temp = 0
    else:
        temp = np.sqrt(temp_)
    theta23_fir = math.atan2(a, b) + math.atan2(temp, c)
    theta23_sec = math.atan2(a, b) - math.atan2(temp, c)

    # 寻找正确的theta3
    print("theta23_fir")
    print(theta23_fir*180/np.pi)
    print("theta23_sec")
    print(theta23_sec*180/np.pi)
    if FLAG==0:
        if theta23_fir > theta23_sec:
            theta3 = theta23_fir - theta2
        else:
            theta3 = theta23_sec - theta2
    else:
        if theta23_fir < theta23_sec:
            theta3 = theta23_fir - theta2
        else:
            theta3 = theta23_sec - theta2
    # theta3 = theta23_fir - theta2
    # theta4好解  -45 < 90 < 225
    theta4 = theta_xy - theta2 - theta3
    theta5 = tool_ang - np.pi

    # 重新配置
    theta2 = np.pi/2-theta2
    theta3 = -theta3
    theta4 = np.pi/2 - theta4

    result = np.array([theta1, theta2, theta3, theta4, theta5])

    print("求逆解theta为：")
    print(result*180/np.pi)
    print(result)


    return [theta1, theta2, theta3, theta4, theta5],True




# theta = [-20, 46.4, -54.3, 38.2, 0]
###
# theta = [23, 90, -45, 76, 0]   # 这个用来测theta1 done
# theta = [-51, 90, -114.6, 5.1, 0]
# theta = [50.1, 129.6, -114.6, 5.1, 0]
# theta = [80, 140, -55, 0, 0]


# for i in range(5):
#     theta[i] = theta[i]*np.pi/180
# result = Pkinematics(theta)
# print("resul......")
# print(result)
#
# IKtheta = Ikinematics(result)
# Pkinematics(IKtheta)
