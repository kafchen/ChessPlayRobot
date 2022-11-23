import numpy as np
import kinematics
import matplotlib.pyplot as plt


def joint_space(ang, vel, acc, time, period):
    print("生成关节规划下的参数...")
    """
    关节空间轨迹规划
    :param ang: 经过角度 [i, j] ，第i个关节的第j个角度  给定
    :param vel: 速度 []       给定
    :param acc: 加速度 []      给定
    :param time: 时间 []
    :param period: 段
    :return: 六个参数
    """
    # 判断acc与time的取值是否可行

    parameters = np.zeros([5, period, 6])
    for i in range(5):
        for j in range(period):
            parameters[i, j, 0] = ang[i, j]
            parameters[i, j, 1] = vel[i, j]
            parameters[i, j, 2] = acc[i, j]*0.5
            parameters[i, j, 3] = (20*ang[i, j+1]-20*ang[i, j]-(8*vel[i, j+1]+12*vel[i, j])*time[j]-(3*acc[i, j]-acc[i, j+1])*time[j]**2) / (2*time[j]**3)
            parameters[i, j, 4] = (30*ang[i, j]-30*ang[i, j+1]+(14*vel[i, j+1]+16*vel[i, j])*time[j]+(3*acc[i, j]-2*acc[i, j+1])*time[j]**2) / (2*time[j]**4)
            parameters[i, j, 5] = (12*ang[i, j+1]-12*ang[i, j]-(6*vel[i, j+1]+6*vel[i, j])*time[j]-(acc[i, j]-acc[i, j+1])*time[j]**2) / (2*time[j]**5)
    print("生成完毕...")
    print("生成参数为：")
    print(parameters)
    return parameters


def cartesian_space(ka_fir, ka_mid, ka_end, times, acc_times):
    """
    笛卡尔空间下的轨迹规划
    :param ka_fir: 初始位姿，[x, y, z, ang1, ang2]
    :param ka_mid: 中间点， [[x, y, z, ang1, ang2], [...]]                              有n-2个中间点
    :param ka_end: 终止点，[x, y, z, ang1, ang2]
    :param times: 各段时间，[]                                                           假设有n段
    :param acc_times: 各点加速时间，[], 比times的个数多一个                                  有n+1个
    :return: 返回的是各段加速度，各段匀速度，各段匀速时间
    """

    n = len(times)
    print("段数n：" + str(n))

    print("acc_times")
    print(acc_times)

    # 抛物插值
    # 如果只有一段
    if n == 1:
        ka_mid = np.zeros((1, 5))
        n = n + 1
        temp = times[0]/2
        times = [temp,temp]
        acc_times = np.array([0.7,0.7,0.7])
        # acc_times = np.array(acc_times.tolist().append(0.7))
        for i in range(5):
            ka_mid[0][i] = (ka_fir[i] + ka_end[i]) * 0.5

    print("ka_mid")
    print(ka_mid)

    vel_jk = np.zeros((n, 5))           # 各段匀速度
    cart_acc = np.zeros((n + 1, 5))     # 各点加速度
    uniform_times = np.zeros(n)         # 各段匀速时间
        # for i in range(5):
        #     cart_acc[n][i] = (ka_mid[n-3][i] - ka_end[i]) / (times[n-1] - 0.5 * acc_times[n]) / acc_times[n]
        #     cart_acc[n][i] = np.sign(ka_mid[n-3][i] - ka_end[i]) * np.absolute(cart_acc[n][i])
        #     vel_jk[n-1][i] = (ka_end[i] - ka_mid[n-3][i]) / (times[n-1] - 0.5 * acc_times[n])
        # uniform_times[n-1] = times[n-1] - acc_times[n] - 0.5 * acc_times[n-1]

    # 如果有两端
    print("ka_fir")
    print(ka_fir)
    print("ka_mid")
    print(ka_mid)
    print("ka_end")
    print(ka_end)
    print("times")
    print(times)
    print("acc_times")
    print(acc_times)
    print("cart_acc")
    print(cart_acc)
    print("vel_jk")
    print(vel_jk)

    # 计算各段的速度，其中第一段和最后一段要单独算，要算n-2个速度
    for i in range(n-2):
        for j in range(5):
            vel_jk[i+1][j] = (ka_mid[i+1][j] - ka_mid[i][j]) / times[i+1]
        uniform_times[i+1] = times[i+1] - 0.5 * acc_times[i+1] - 0.5 * acc_times[i+2]
    # 计算第一个
    for i in range(5):
        cart_acc[0][i] = (ka_mid[0][i] - ka_fir[i]) / (times[0] - 0.5 * acc_times[0]) / acc_times[0]

        cart_acc[0][i] = np.sign(ka_mid[0][i] - ka_fir[i]) * np.absolute(cart_acc[0][i])
        vel_jk[0][i] = (ka_mid[0][i] - ka_fir[i]) / (times[0] - 0.5 * acc_times[0])
    uniform_times[0] = times[0] - acc_times[0] - 0.5 * acc_times[1]
    # 计算最后一个
    for i in range(5):
        cart_acc[n][i] = (ka_mid[n-3][i] - ka_end[i]) / (times[n-1] - 0.5 * acc_times[n]) / acc_times[n]
        cart_acc[n][i] = np.sign(ka_mid[n-3][i] - ka_end[i]) * np.absolute(cart_acc[n][i])
        vel_jk[n-1][i] = (ka_end[i] - ka_mid[n-3][i]) / (times[n-1] - 0.5 * acc_times[n])
    uniform_times[n-1] = times[n-1] - acc_times[n] - 0.5 * acc_times[n-1]
    # 计算中间点的加速度
    for i in range(n-1):
        for j in range(5):
            cart_acc[i+1][j] = (vel_jk[i+1][j]-vel_jk[i][j])/acc_times[i+1]
    # print("cart_acc")
    # print(cart_acc)
    # print("vel_jk")
    # print(vel_jk)
    # print("uniform_times")
    # print(uniform_times)
    return cart_acc, vel_jk, uniform_times


# def cart_get_trace(ka_fir, vel_jk, cart_acc, uniform_times, times, sample, acc_times):
#     """
#     笛卡尔空间下轨迹规划
#     :param vel_jk: 直线处jk的匀速度
#     :param cart_acc: 笛卡尔规划的过渡段的加速度
#     :param uniform_times: 匀速度段的时间
#     :param times: 每两点之间的时间
#     :param sample: 总采样数
#     :param acc_times: 每个过渡段的加速时间
#     :return:
#     """
#     x = np.zeros([sample, 5])      # x是5个自由度的位置
#     t_all = 0
#     for i in times:
#         t_all = t_all + i
#     sp_time = t_all / sample
#     n = len(times)
#     t_count = np.zeros(2*n+1)
#     for i in range(n):
#         t_count[2*i] = acc_times[i]
#         t_count[2*i+1] = uniform_times[i]
#     t_count[2*n] = acc_times[n]
#
#     t_count = t_count / sp_time
#     t_count = np.array(t_count, dtype=int)  # t_count 是每一段的计算次数  [抛物，直线，抛物，直线...]
#     flag = 0
#
#     d = 1
#     count = 1
#     x_j = ka_fir
#     x[0] = ka_fir
#     for i in t_count:
#         for j in range(i):
#             for k in range(5):
#                 if flag == 0:
#                     # 写抛物线的
#                     if d == 1:
#                         t_now = (j+1) * sp_time
#                         x[count][k] = ka_fir[k]+0.5 * cart_acc[0][k] * t_now**2
#                     else:
#                         t_now = (j+1) * sp_time
#                         # t_inb = t_now - (0.5*acc_times[int((d-1)/2-1)] + uniform_times[int((d-1)/2-1)])
#                         x[count][k] = x_j[k] + vel_jk[int((d-1)/2-1)][k]*t_now+0.5*cart_acc[int((d-1)/2)][k]*t_now**2
#                         # x[count][k] = x_j[k] + uniform_times[int((d-1)/2-1)]*(t_now-t_inb)+0.5*cart_acc[int((d-1)/2)][k]*t_inb**2
#                 if flag == 1:
#                     # 写直线的
#                     t_now = (j+1) * sp_time
#                     x[count][k] = x_j[k]+vel_jk[int(d/2-1)][k]*t_now
#             count = count + 1
#         if flag == 1:
#             flag = flag - 1
#         else:
#             flag = flag + 1
#         d = d + 1
#         for z in range(5):
#             x_j[z] = x[count-1][z]
#         # zf = 1
#         # while zf:
#         #     print(x)
#         #     if all(x[-1]) == 0:
#         #         x=x[:-1]
#         #
#         #     else:
#         #         zf = 0
#     return x,

def cart_get_trace(ka_fir, vel_jk, cart_acc, uniform_times, times, sample, acc_times):
    """
    笛卡尔空间下轨迹规划
    :param vel_jk: 直线处jk的匀速度
    :param cart_acc: 笛卡尔规划的过渡段的加速度
    :param uniform_times: 匀速度段的时间
    :param times: 每两点之间的时间
    :param sample: 总采样数
    :param acc_times: 每个过渡段的加速时间
    :return:
    """
    t_all = 0
    for i in times:
        t_all = t_all + i
    sample = int(t_all / 0.1)
    print('1',sample)
    sample = sample + 1
    print('2', sample)
    x = np.zeros([sample, 5])      # x是5个自由度的位置
    print('x...',)
    print(x)
    sp_time = t_all / sample
    print("sp_time")
    print(sp_time)
    n = len(times)
    t_count = np.zeros(2*n+1)
    for i in range(n):
        t_count[2*i] = acc_times[i]
        t_count[2*i+1] = uniform_times[i]
    t_count[2*n] = acc_times[n]

    t_count = t_count / sp_time
    t_count = np.array(t_count, dtype=int)  # t_count 是每一段的计算次数  [抛物，直线，抛物，直线...]
    flag = 0

    d = 1
    count = 1
    x_j = ka_fir
    x[0] = ka_fir
    for i in t_count:
        for j in range(i):
            for k in range(5):
                if flag == 0:
                    # 写抛物线的
                    if d == 1:
                        t_now = (j+1) * sp_time
                        x[count][k] = ka_fir[k]+0.5 * cart_acc[0][k] * t_now**2
                    else:
                        t_now = (j+1) * sp_time
                        # t_inb = t_now - (0.5*acc_times[int((d-1)/2-1)] + uniform_times[int((d-1)/2-1)])
                        x[count][k] = x_j[k] + vel_jk[int((d-1)/2-1)][k]*t_now+0.5*cart_acc[int((d-1)/2)][k]*t_now**2
                        # x[count][k] = x_j[k] + uniform_times[int((d-1)/2-1)]*(t_now-t_inb)+0.5*cart_acc[int((d-1)/2)][k]*t_inb**2
                if flag == 1:
                    # 写直线的
                    t_now = (j+1) * sp_time
                    x[count][k] = x_j[k]+vel_jk[int(d/2-1)][k]*t_now
            count = count + 1
        if flag == 1:
            flag = flag - 1
        else:
            flag = flag + 1
        d = d + 1
        for z in range(5):
            x_j[z] = x[count-1][z]
    print("x")
    print(x)
    zf = 1
    while zf:
        print('###',x[-1])
        print((x[-1] == 0).all())
        if (x[-1] == 0).all():
            x=x[:-1]

        else:
            zf = 0
    return x, sp_time

def get_jointorder(parameter, times, sp_time=0.1):
    print("获取关节规划下的命令...")
    # parameters: [i,j,6]   第一维度代表5个舵机，第二维度代表总段数，第三个维度代表六个参数
    # 循环  5个舵机， 选择段数，
    t_all = 0
    # order = "{"
    order = []
    for i in times:
        t_all = t_all + i  # 总时常
    times = np.array(times)
    times = times / sp_time
    times = np.array(times, dtype=int)      # times[] 是每段需要计算的次数
    n = int(t_all / sp_time)                # 每一段需要计算角度的次数的总和
    print("n")
    print(n)
    d = 0                                   # 当前是第几段
    x = 0                                   # 每填进一次theta，x就加一

    theta = np.zeros([n, 5])


    print("theta")
    print(theta)
    print("times")
    print(times)

    # 得到theta，各个时间点的关节位置
    for i in times:
        for j in range(i):
            t = j * sp_time     # t是开始到当前计算点的时间
            for k in range(5):
                print("parameter[k, d, 0]")
                print(parameter[k, d, 0])
                theta[x, k] = parameter[k, d, 0] + parameter[k, d, 1]*t+parameter[k, d, 2]*t**2+\
                              parameter[k, d, 3]*t**3+parameter[k, d, 4]*t**4+parameter[k, d, 5]*t**5
            x = x + 1
        d = d + 1
    print("规划的角度为...")
    print(theta)
    order_t = sp_time*1000

    # 当角度是弧度制
    for i in theta:         # i 是一组角
        idx = 0
        new_order = "{"
        for j in i:         # j 是一个角
            temp_order, _ = kinematics.get_one_order(idx, j, order_t)
            new_order = new_order + temp_order
            idx = idx + 1
        new_order = new_order + "}"
        order.append(new_order)
    # for i in theta:         # i 是一组角
    #     idx = 0
    #     for j in i:         # j 是一个角
    #         new_order, _ = kinematics.get_one_order(idx, j, order_t)
    #         order = order + new_order
    #         idx = idx + 1
    # order = order + "}"

    return order, theta


def get_cartorder(position, sp_time=0.1):
    print("获取笛卡尔下规划的命令")
    """
    得到笛卡尔空间下的轨迹移动命令
    :param position: 全部的位置
    :return:
    """
    theta_all =[]
    # order = "{"
    # if sp_time < 1:
    #     order_t = "0"+str(round(sp_time*1000))
    # else:
    #     order_t = str(round(sp_time*1000))
    #
    # for i in position:
    #     idx = 0
    #     theta = kinematics.Ikinematics(i)
    #
    #     for j in theta:
    #         P = int(500 + j / (3 / 2 * np.pi) * 2000)
    #         if P > 2500 or P < 500:
    #             print("错误!!关节"+str(idx)+"无法抵达" + str(j))
    #             return
    #         P = str(P)
    #         if len(P) == 3:
    #             order = order + "#00" + str(idx) + "P0" + P + "T" + order_t + "!"
    #         else:
    #             order = order + "#00" + str(idx) + "P" + P + "T" + order_t + "!"
    #         idx = idx + 1
    # order = order + "}"
    # return order
    order = []

    if sp_time < 1:
        order_t = "0"+str(round(sp_time*1000))
    else:
        order_t = str(round(sp_time*1000))

    for i in position:
        idx = 0
        i[0] = i[0] / 1000
        i[1] = i[1] / 1000
        i[2] = i[2] / 1000
        theta,flag = kinematics.Ikinematics(i)   # theta 是一组关节
        theta_all.append(theta)
        if not flag:
            return []
        new_order = "{"
        for j in theta:  # j 是一个角
            temp_order, flag = kinematics.get_one_order(idx, j, order_t)
            if not flag:
                return []
            new_order = new_order + temp_order
            idx = idx + 1
        new_order = new_order + "}"
        order.append(new_order)
        # for j in theta:             # j 是一个关节组的一个关节
        #     P = int(500 + j / (3 / 2 * np.pi) * 2000)
        #     if P > 2500 or P < 500:
        #         print("错误!!关节"+str(idx)+"无法抵达" + str(j))
        #         return
        #     P = str(P)
        #     if len(P) == 3:
        #         new_order = new_order + "#00" + str(idx) + "P0" + P + "T" + order_t + "!"
        #     else:
        #         new_order = new_order + "#00" + str(idx) + "P" + P + "T" + order_t + "!"
        #     idx = idx + 1
        # new_order = new_order + "}"
        # order.append(new_order)
    return order, theta_all


# 这段是关节空间的用法
########################
# ang = np.array([[np.pi/2, np.pi/3, np.pi/4, np.pi/5, np.pi/2],
#        [np.pi/2, np.pi/3, np.pi/4, np.pi/5, np.pi/3],
#        [np.pi/2, np.pi/3, np.pi/4, np.pi/5, np.pi/3],
#        [np.pi/2, np.pi/3, np.pi/4, np.pi/5, np.pi/3],
#        [np.pi/2, np.pi/3, np.pi/4, np.pi/5, np.pi/3]])
# vel = np.array([[0, 0, 0, 0, 0],
#                 [0, 0, 0, 0, 0],
#                 [0, 0, 0, 0, 0],
#                 [0, 0, 0, 0, 0],
#                 [0, 0, 0, 0, 0]])
# acc = np.array([[1,2,3,2,1],
#                 [1,2,3,2,1],
#                 [1,2,3,2,1],
#                 [1,2,3,2,1],
#                 [1,2,3,2,1]])
# time = [4, 4, 4]
# period = 3
# parameters = joint_space(ang, vel, acc, time, period)
# order, _ = get_jointorder(parameters, time, 0.1)
# print(order)
#############################

# 这段是笛卡尔空间的用法
#############################
# ka_fir = [0, 0, 233, 0, np.pi]
# ka_mid = [[0, 0, 210, 0, np.pi]]
# ka_end = [0, 0, 200, 0, np.pi]
# times = [4, 4]
# acc_times = [1, 1, 2]
# cart_acc, vel_jk, uniform_times = cartesian_space(ka_fir, ka_mid, ka_end, times, acc_times)
# x, sp_time = cart_get_trace(ka_fir, vel_jk, cart_acc, uniform_times, times, 40, acc_times)
# print(x)
# plt.plot(x[:, 2])
# plt.show()
# # ############################
# #
# #
# order = get_cartorder(x, sp_time)
# print(order)

# print("cart_acc")
# print(cart_acc)
# print("vel_jk")
# print(vel_jk)
# print("uniform_times")
# print(uniform_times)