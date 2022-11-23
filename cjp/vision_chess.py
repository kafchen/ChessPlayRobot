# 打开摄像头并灰度化显示
import time

import cv2
import numpy as np


class robotplaychess:
    def __init__(self):
        self.kernel = np.ones((4, 4), np.uint8)
        self.chesspoint = [[[50, 50], [150, 50], [250, 50]], [[60, 160], [160, 160], [260, 160]],
                           [[60, 260], [140, 250], [260, 260]]]
        self.chess = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.world_points = np.array(
            [
                [191.6, 204.1, 0],  # tl
                [44.8, 158.3, 0],  # bl
                [62, 60.5, 0],  # br
                [210.1, 41.5, 0]  # tr
            ], dtype=np.float32
        )  # 机器人坐标，3D标定需要至少6个坐标，2D标定也就是所有点的z坐标相同，需要至少4个坐标

        self.chesslist = []
        # self.image_points = self.pts1

    # cv2.imshow('frame', self.frame)

    def playc(self, frame):
        getuv = self.robotchess(frame)
        # print(self.getuv)
        if (getuv is not None):
            print(getuv[0], getuv[1])
            x, y = self.getworldpoint(getuv[0], getuv[1])
            return x, y
        else:
            return None, None

    def getc(self, frame):
        getuv = self.getchess(frame)
        if getuv is not None:
            print(getuv[0], getuv[1])
            x, y = self.getworldpoint(getuv[0], getuv[1])
            return x, y
        else:
            return None, None

    def undistortcb(self, frame):
        h1, w1 = frame.shape[:2]
        print("ok")
        # 打开标定文件
        cv_file = cv2.FileStorage("camera.yaml", cv2.FILE_STORAGE_READ)
        camera_matrix = cv_file.getNode("camera_matrix").mat()
        dist_matrix = cv_file.getNode("dist_coeff").mat()
        cv_file.release()

        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_matrix, (w1, h1), 0, (w1, h1))
        # print('newcameramtx:',newcameramtx)
        # 纠正畸变
        dst = cv2.undistort(frame, camera_matrix, dist_matrix, None, newcameramtx)
        # mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_matrix, None, newcameramtx, (w1, h1), 5)
        # dst2 = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

        # 裁剪图像，输出纠正畸变以后的图片
        x, y, w1, h1 = roi
        dst = dst[y:y + h1, x:x + w1]

        print("ok2")
        return dst

    def chessborad(self, frame, flag):  # 检测棋盘，传入原始图像frame，flag表示是否是只需要角点还是需要整个棋盘情况
        # 获取一帧
        # print('t1: ',frame.shape)
        # frame0 = self.undistortcb(frame)
        frame0 = frame
        warp = frame0
        green_lower = np.array([57, 30, 30])
        green_upper = np.array([85, 255, 255])
        # 将这帧转换为灰度图
        hsv = cv2.cvtColor(frame0, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, green_lower, green_upper)  # mask 启动
        # mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, self.kernel)
        # mask = cv2.GaussianBlur(mask, (3, 3), 0)



        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_area = 0
        m = 0

        for i, contour in enumerate(contours):
            area = cv2.contourArea(contour)  # 面积
            if area > max_area:
                max_area = area
                m = i

        approxCurve = cv2.approxPolyDP(contours[m], 20, True)
        print(approxCurve.shape)
        if approxCurve.shape[0] == 4 and max_area > 2000:
            print(approxCurve[0][0])
            print(approxCurve[1][0])
            print(approxCurve[2][0])
            print(approxCurve[3][0])

            cv2.line(frame0, approxCurve[0][0], approxCurve[1][0], (0, 0, 255), 2)
            cv2.line(frame0, approxCurve[1][0], approxCurve[2][0], (0, 0, 255), 2)
            cv2.line(frame0, approxCurve[2][0], approxCurve[3][0], (0, 0, 255), 2)
            cv2.line(frame0, approxCurve[3][0], approxCurve[0][0], (0, 0, 255), 2)

            cv2.imshow('frame0', frame0)
            cv2.waitKey(100)

            if approxCurve[0][0][0] + approxCurve[0][0][1] < approxCurve[1][0][0] + approxCurve[1][0][1] < \
                    approxCurve[2][0][0] + approxCurve[2][0][1]:
                self.topleft = approxCurve[0][0]
                self.bottomleft = approxCurve[1][0]
                self.bottomright = approxCurve[2][0]
                self.topright = approxCurve[3][0]
            elif approxCurve[1][0][0] + approxCurve[1][0][1] < approxCurve[2][0][0] + approxCurve[2][0][1] < \
                    approxCurve[3][0][0] + approxCurve[3][0][1]:
                self.topleft = approxCurve[1][0]
                self.bottomleft = approxCurve[2][0]
                self.bottomright = approxCurve[3][0]
                self.topright = approxCurve[0][0]
            elif approxCurve[2][0][0] + approxCurve[2][0][1] < approxCurve[3][0][0] + \
                    approxCurve[3][0][1] < approxCurve[0][0][0] + approxCurve[0][0][1]:
                self.topleft = approxCurve[2][0]
                self.bottomleft = approxCurve[3][0]
                self.bottomright = approxCurve[0][0]
                self.topright = approxCurve[1][0]
            elif approxCurve[3][0][0] + approxCurve[3][0][1] < approxCurve[0][0][0] + \
                    approxCurve[0][0][1] < approxCurve[1][0][0] + approxCurve[1][0][1]:
                self.topleft = approxCurve[1][0]
                self.bottomleft = approxCurve[2][0]
                self.bottomright = approxCurve[3][0]
                self.topright = approxCurve[0][0]

            cv2.line(warp, self.topleft, self.bottomleft, (0, 0, 255), 2)
            cv2.line(warp, self.bottomleft, self.bottomright, (0, 0, 255), 2)
            cv2.line(warp, self.bottomright, self.topright, (0, 0, 255), 2)
            cv2.line(warp, self.topright, self.topleft, (0, 0, 255), 2)
            print(self.topleft)
            print(self.bottomleft)
            print(self.bottomright)
            print(self.topright)

            self.pts1 = np.float32([self.topleft, self.bottomleft, self.bottomright, self.topright])
            self.pts2 = np.float32([[0, 0], [0, 300], [300, 300], [300, 0]])

            if flag == 1:
                M = cv2.getPerspectiveTransform(self.pts1, self.pts2)
                warp = cv2.warpPerspective(frame, M, (300, 300))
                warp = cv2.medianBlur(warp, 3)
                warp = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)
                cv2.line(warp, (0, 100), (300, 100), (0, 0, 255), 2)
                cv2.line(warp, (0, 200), (300, 200), (0, 0, 255), 2)
                cv2.line(warp, (100, 0), (100, 300), (0, 0, 255), 2)
                cv2.line(warp, (200, 0), (200, 300), (0, 0, 255), 2)
                print(warp.shape)
                for i in range(3):
                    for j in range(3):
                        # print(chesspoint[i][j])
                        H0 = warp[self.chesspoint[i][j][1]][self.chesspoint[i][j][0]][0]
                        S0 = warp[self.chesspoint[i][j][1]][self.chesspoint[i][j][0]][1]
                        V0 = warp[self.chesspoint[i][j][1]][self.chesspoint[i][j][0]][2]

                        if 160 <= H0 <= 190 and 80 <= S0 <= 250 and 90 <= V0 <= 255:
                            self.chess[i][j] = 3  # 红
                            cv2.circle(warp, self.chesspoint[i][j], 13, (0, 0, 255), -1)
                        elif 90 <= H0 <= 130 and 80 <= S0 <= 250 and 85 <= V0 <= 255:
                            self.chess[i][j] = 2  # 蓝
                            cv2.circle(warp, self.chesspoint[i][j], 13, (255, 0, 0), -1)
                        else:
                            self.chess[i][j] = 0
                            cv2.circle(warp, self.chesspoint[i][j], 13, (255, 255, 255), -1)
                        # print(i, j, H0, S0, V0,self.chesspoint[i][j],self.chess[i][j])



                # self.chess_p = self.getcp()
                # self.getuv = self.robotchess()
                # cv2.imshow('warp', self.warp)
                # cv2.waitKey(100)
                cv2.imshow('warp', warp)
                cv2.imshow('mask', mask)
                print(self.chess)

            else:
                pass

        else:
            self.topleft = None
            self.bottomleft = None
            self.bottomright = None
            self.topright = None
            self.pts1 = None
            # self.getuv = None

        return approxCurve.shape[0], frame0

    def getcp(self):
        if self.pts1 is not None:
            M1, state = cv2.findHomography(self.pts2, self.pts1)
            pts = np.float32(self.chesspoint).reshape(-1, 1, 2)
            out_cp = cv2.perspectiveTransform(pts, M1)
            chess_cp = np.around(out_cp).astype(int).reshape((3, 3, 2))
            # print(chess_cp)

            return chess_cp
        else:
            return None


    def round(self,frame):
        try:
            summ = 0
            for i in range(3):
                for j in range(3):
                    summ += self.chess[i][j]
            print(summ)
            return summ
        except:
            time.sleep(2)
            _, _ = self.chessborad(frame, 1)
            self.round(frame)





    # 根据棋盘情况获得对应像素坐标
    def robotchess(self, frame):
        _, _ = self.chessborad(frame, 1)  # 检测一次棋盘，获得角点
        self.chess_p = self.getcp()  # 获得原始图像棋盘九个位置的像素坐标
        if self.chess_p is not None:
            summ = self.round(frame)

            if summ == 10 or summ == 15 or summ == 20:
                p, out = self.attack()
                if p == 1:
                    return out
            else:
                p = 0

            if p == 0 and (summ == 10 or summ == 15 or summ == 20):
                out = self.defend()
                if out is not None:
                    return out

            if p == 0 and summ == 0:
                print('0,0: ', self.chesspoint[0][0])
                return self.chess_p[0][0]
                # chess[0][0] = 3
            elif summ == 5 and (
                    self.chess[1][0] == 2 or self.chess[0][1] == 2 or self.chess[2][1] == 2 or self.chess[1][2] == 2):
                print('1,1: ', self.chesspoint[1][1])
                return self.chess_p[1][1]
                # chess[1][1] = 3
            elif summ == 5 and (self.chess[2][0] == 2 or self.chess[0][2] == 2 or self.chess[1][1] == 2):
                print('2,2: ', self.chesspoint[2][2])
                return self.chess_p[2][2]
                # chess[2][2] = 3
            elif summ == 5 and self.chess[2][2] == 2:
                print('2,0: ', self.chesspoint[2][0])
                return self.chess_p[2][0]
                # chess[2][0] = 3
            elif summ == 10 and self.chess[2][2] == 2 and (self.chess[1][0] == 2 or self.chess[1][2] == 2) and \
                    self.chess[0][2] == 0:
                print('0,2: ', self.chesspoint[0][2])
                return self.chess_p[0][2]
            elif summ == 10 and self.chess[2][2] == 2 and (self.chess[0][1] == 2 or self.chess[2][1] == 2) and \
                    self.chess[2][0] == 0:
                print('2,0: ', self.chesspoint[2][0])
                return self.chess_p[2][0]
        else:
            return None

    def attack(self):
        if ((self.chess[1][0] == 3 and self.chess[2][0] == 3) or (self.chess[1][1] == 3 and self.chess[2][2] == 3) or (
                self.chess[0][1] == 3 and self.chess[0][2] == 3)) and self.chess[0][0] == 0:
            print('0,0: ', self.chesspoint[0][0])
            return 1, self.chess_p[0][0]
        elif ((self.chess[0][0] == 3 and self.chess[1][0] == 3) or (
                self.chess[1][1] == 3 and self.chess[0][2] == 3) or (
                      self.chess[2][1] == 3 and self.chess[2][2] == 3)) and self.chess[2][0] == 0:
            print('2,0: ', self.chesspoint[2][0])
            return 1, self.chess_p[2][0]
        elif ((self.chess[0][0] == 3 and self.chess[1][1] == 3) or (
                self.chess[2][0] == 3 and self.chess[2][1] == 3) or (
                      self.chess[0][2] == 3 and self.chess[1][2] == 3)) and self.chess[2][2] == 0:
            print('2,2: ', self.chesspoint[2][2])
            return 1, self.chess_p[2][2]
        elif ((self.chess[0][0] == 3 and self.chess[0][1] == 3) or (
                self.chess[1][1] == 3 and self.chess[2][0] == 3) or (
                      self.chess[1][2] == 3 and self.chess[2][2] == 3)) and self.chess[0][2] == 0:
            print('0,2: ', self.chesspoint[0][2])
            return 1, self.chess_p[0][2]
        elif ((self.chess[0][0] == 3 and self.chess[2][0] == 3) or (
                self.chess[1][1] == 3 and self.chess[1][2] == 3)) and self.chess[1][0] == 0:
            print('1,0: ', self.chesspoint[1][0])
            return 1, self.chess_p[1][0]
        elif ((self.chess[0][0] == 3 and self.chess[0][2] == 3) or (
                self.chess[1][1] == 3 and self.chess[2][1] == 3)) and self.chess[0][1] == 0:
            print('0,1: ', self.chesspoint[0][1])
            return 1, self.chess_p[0][1]
        elif ((self.chess[1][0] == 3 and self.chess[1][1] == 3) or (
                self.chess[0][2] == 3 and self.chess[2][2] == 3)) and self.chess[1][2] == 0:
            print('1,2: ', self.chesspoint[1][2])
            return 1, self.chess_p[1][2]
        elif ((self.chess[0][1] == 3 and self.chess[1][1] == 3) or (
                self.chess[2][0] == 3 and self.chess[2][2] == 3)) and self.chess[2][1] == 0:
            print('2,1: ', self.chesspoint[2][1])
            return 1, self.chess_p[2][1]
        elif (self.chess[1][1] == 0):
            print('1,1: ', self.chesspoint[1][1])
            return 1, self.chess_p[1][1]
        else:
            return 0, 0

    def defend(self):

        if ((self.chess[1][0] == 2 and self.chess[2][0] == 2) or (self.chess[1][1] == 2 and self.chess[2][2] == 2) or (
                self.chess[0][1] == 2 and self.chess[0][2] == 2)) and self.chess[0][0] == 0:
            print('0,0: ', self.chesspoint[0][0])
            return self.chess_p[0][0]
        elif ((self.chess[0][0] == 2 and self.chess[1][0] == 2) or (
                self.chess[1][1] == 2 and self.chess[0][2] == 2) or (
                      self.chess[2][1] == 2 and self.chess[2][2] == 2)) and self.chess[2][0] == 0:
            print('2,0: ', self.chesspoint[2][0])
            return self.chess_p[2][0]
        elif ((self.chess[0][0] == 2 and self.chess[1][1] == 2) or (
                self.chess[2][0] == 2 and self.chess[2][1] == 2) or (
                      self.chess[0][2] == 2 and self.chess[1][2] == 2)) and self.chess[2][2] == 0:
            print('2,2: ', self.chesspoint[2][2])
            return self.chess_p[2][2]
        elif ((self.chess[0][0] == 2 and self.chess[0][1] == 2) or (
                self.chess[1][1] == 2 and self.chess[2][0] == 2) or (
                      self.chess[1][2] == 2 and self.chess[2][2] == 2)) and self.chess[0][2] == 0:
            print('0,2: ', self.chesspoint[0][2])
            return self.chess_p[0][2]
        elif ((self.chess[0][0] == 2 and self.chess[2][0] == 2) or (
                self.chess[1][1] == 2 and self.chess[1][2] == 2)) and self.chess[1][0] == 0:
            print('1,0: ', self.chesspoint[1][0])
            return self.chess_p[1][0]
        elif ((self.chess[0][0] == 2 and self.chess[0][2] == 2) or (
                self.chess[1][1] == 2 and self.chess[2][1] == 2)) and self.chess[0][1] == 0:
            print('0,1: ', self.chesspoint[0][1])
            return self.chess_p[0][1]
        elif ((self.chess[1][0] == 2 and self.chess[1][1] == 2) or (
                self.chess[0][2] == 2 and self.chess[2][2] == 2)) and self.chess[1][2] == 0:
            print('1,2: ', self.chesspoint[1][2])
            return self.chess_p[1][2]
        elif ((self.chess[0][1] == 2 and self.chess[1][1] == 2) or (
                self.chess[2][0] == 2 and self.chess[2][2] == 2)) and self.chess[2][1] == 0:
            print('2,1: ', self.chesspoint[2][1])
            return self.chess_p[2][1]
        elif ((self.chess[0][0] == 2 and self.chess[2][2] == 2) or (self.chess[2][0] == 2 and self.chess[0][2] == 2) or
              (self.chess[2][1] == 2 and self.chess[0][1] == 2) or (
                      self.chess[1][0] == 2 and self.chess[1][2] == 2)) and self.chess[1][1] == 0:
            print('1,1: ', self.chesspoint[1][1])
            return self.chess_p[1][1]
        else:
            return None

    def getworldpoint(self, u, v):
        if self.pts1 is not None:
            H, state = cv2.findHomography(self.pts1, self.world_points)
            pts = np.float32([u, v]).reshape(-1, 1, 2)
            xy = cv2.perspectiveTransform(pts, H)
            x_w = xy[0][0][0]
            y_w = xy[0][0][1] - 100
            print('x:', x_w)
            print('y:', y_w)
            return x_w, y_w
        else:
            return None

    #  取棋子
    def getchess(self, frame):
        corner, frame1 = self.chessborad(frame, 0)  # 检测棋盘获得角点和无畸变图像

        font = cv2.FONT_HERSHEY_SIMPLEX  # 设置字体样式
        kernel = np.ones((5, 5), np.uint8)  # 卷积核
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换为灰色通道
        hsv = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)  # 转换为HSV空间

        lower_red = np.array([160, 150, 90])  # 设定红色的阈值下限
        upper_red = np.array([180, 255, 255])  # 设定红色的阈值上限

        #  消除噪声
        mask = cv2.inRange(hsv, lower_red, upper_red)  # 设定掩膜取值范围
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)  # 形态学开运算
        edges = cv2.Canny(opening, 50, 100)  # 边缘识别

        cv2.imshow('get',mask)
        cv2.waitKey(100)

        circles = cv2.HoughCircles(
            edges, cv2.HOUGH_GRADIENT, 1, 100, param1=100, param2=10, minRadius=5, maxRadius=50)
        if circles is not None:  # 如果识别出圆
            for circle in circles[0]:
                #  获取圆的坐标与半径
                x = int(circle[0])
                y = int(circle[1])
                r = int(circle[2])
                if corner == 4:
                    if not ((self.topleft[0] < x < self.bottomright[0]) and (
                            self.topleft[1] < y < self.bottomright[1])):
                        cv2.circle(frame1, (x, y), r, (0, 0, 255), 3)  # 标记圆
                        cv2.circle(frame1, (x, y), 3, (255, 255, 0), -1)  # 标记圆心
                        text = 'x:  ' + str(x) + ' y:  ' + str(y) + ' r:  ' + str(r)
                        print(text)
                        # cv2.putText(self.frame, text, (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA, 0)  # 显示圆心位置

                        self.chesslist.append([x, y])

                    else:
                        pass
                else:
                    cv2.circle(frame1, (x, y), r, (0, 0, 255), 3)  # 标记圆
                    cv2.circle(frame1, (x, y), 3, (255, 255, 0), -1)  # 标记圆心
                    text = 'x:  ' + str(x) + ' y:  ' + str(y) + ' r:  ' + str(r)
                    print(text)
                    # cv2.putText(self.frame, text, (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA, 0)  # 显示圆心位置
                    self.chesslist.append([x, y])
        else:
            # 如果识别不出，显示圆心不存在
            print('检测不到圆形')
            # cv2.putText(frame1, 'x: None y: None', (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA, 0)
        # cv2.imshow('frame', frame1)
        # cv2.waitKey(100)
        print("可用棋子像素坐标：",self.chesslist[0])

        getx, gety = self.getworldpoint(self.chesslist[0][0], self.chesslist[0][1])

        return getx, gety  # 返回一个可用棋子坐标
        # cv2.imshow('frame', frame)
        # cv2.imshow('mask', mask)
        # cv2.imshow('edges', edges)

    def getpitch(self, x, y):
        # y=kx+b
        print('INPICTH')
        r = (x ** 2 + y ** 2) ** 0.5
        rmax = 218
        rmin = 73
        pitchmax = 168.1 + 5
        pitchmin = 152.9 - 5
        k = (pitchmax - pitchmin) / (rmax - rmin)
        pitch = (r - rmin) * k + pitchmin
        print(pitch)
        return pitch


if __name__ == "__main__":
    capture = cv2.VideoCapture(0)
    # img = cv2.imread('ttttset.jpg')
    rp= robotplaychess()
    while (1):
        ret, frame0 = capture.read()  # 640x480
        dst = rp.undistortcb(frame0)
        _,_ =rp.playc(dst)
        # cv2.imshow('img',dst)
        # x, y = robotplaychess().playc_xy()  # 输入原始图片获得下棋对应的现实xy,当检测不到时xy为None
        # x, y = robotplaychess(frame0).getc_xy()  # 输入原始图片获得取棋子对应的现实xy，当检测不到时xy为None

        if cv2.waitKey(1) == ord('q'):
            break
    capture.release()
    cv2.destroyAllWindows()
