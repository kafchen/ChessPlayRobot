import cv2
import numpy as np

class follow:
    def __init__(self,frame,u,v):
        self.kernel = np.ones((4, 4), np.uint8)
        a = 170
        b = 90
        self.pts2 = np.float32([[a, b], [a, b + 300], [a + 300, b + 300], [a + 300, b]])
        self.flag,self.pts1 = self.findrectangle(frame)
        u = int(u*640)
        v = int(v*480)
        print(self.flag)
        if self.flag == 4:
            print('following')
            self.M1 = cv2.getPerspectiveTransform(self.pts1, self.pts2)
            # M2 = cv2.getPerspectiveTransform(self.pts2, self.pts1)
            pts = np.float32([u, v]).reshape(-1, 1, 2)
            fix_uv = cv2.perspectiveTransform(pts, self.M1)
            fix_uv = np.around(fix_uv).astype(int)
            self.u = fix_uv[0][0][0]
            self.v = fix_uv[0][0][1]
            print('fix_u:', self.u)
            print('fix_v:', self.v)
        else:
            self.u,self.v = None,None

    def findrectangle(self, frame):
        # 获取一帧
        # print('t1: ',frame.shape)

        warp = frame
        green_lower = np.array([57, 0, 0])
        green_upper = np.array([87, 255, 255])
        # 将这帧转换为灰度图
        hsv = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, green_lower, green_upper)  # mask 启动
        mask = cv2.erode(mask, None, iterations=2)
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

        approxCurve = cv2.approxPolyDP(contours[m], 50, True)
        # print(approxCurve.shape)
        if approxCurve.shape[0] == 4:
            # print(approxCurve[0][0])
            # print(approxCurve[1][0])
            # print(approxCurve[2][0])
            # print(approxCurve[3][0])
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

            # cv2.line(warp, self.topleft, self.bottomleft, (0, 0, 255), 2)
            # cv2.line(warp, self.bottomleft, self.bottomright, (0, 0, 255), 2)
            # cv2.line(warp, self.bottomright, self.topright, (0, 0, 255), 2)
            # cv2.line(warp, self.topright, self.topleft, (0, 0, 255), 2)
            # print(self.topleft)
            # print(self.bottomleft)
            # print(self.bottomright)
            # print(self.topright)

            pts1 = np.float32([self.topleft, self.bottomleft, self.bottomright, self.topright])

        else:

            pts1 = None

        return approxCurve.shape[0],pts1

    def keep(self,frame):
        print(self.u,self.v)
        keep_flag,new_pts = self.findrectangle(frame)
        if keep_flag == 4:
            M2 = cv2.getPerspectiveTransform(self.pts2, new_pts)
            pts = np.float32([self.u, self.v]).reshape(-1, 1, 2)
            new_uv = cv2.perspectiveTransform(pts, M2)
            new_uv = np.around(new_uv).astype(int)
            new_u = new_uv[0][0][0]
            new_v = new_uv[0][0][1]
            print('new_u:', new_u)
            print('new_v:', new_v)
            return [new_u/640, new_v/480]
        else:
            new_u, new_v = None, None
            return [None, None]

    def read_init(self):
        return self.u




if __name__ == "__main__":
    capture = cv2.VideoCapture(1)
    im_org = cv2.imread('test8.jpg')          # 点击一次读取一个图片 作为初始化
    cv2.imshow('org', im_org)
    p = follow(im_org, 155, 13)               #p实例化跟踪类,参数是点一下传入的图片和像素坐标
    cv2.circle(im_org, (155, 13), 3, (255, 255, 0), -1)
    while (1):                                 #进入循环 图片更新
        ret, frame0 = capture.read()
        print(frame0)#frame0是实时读取的图片
        cv2.imshow('org', im_org)                        #显示第一次读取的图片 这个是为了我方便对比 你只显示实时的就行
        new_u,new_v= p.keep(frame0)               #p实例化跟踪类,参数是点一下传入的图片和像素坐标 得到实时图片下对应关系的点坐标
        cv2.circle(frame0, (new_u, new_v), 3, (255, 255, 0), -1)            # 根据点坐标画圆
        # 标记圆心
        cv2.imshow('frame', frame0)                                 #显示
        if cv2.waitKey(1) == ord('q'):
            break
    capture.release()
    cv2.destroyAllWindows()
