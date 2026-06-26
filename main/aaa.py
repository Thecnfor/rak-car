# -*- coding: utf-8 -*-
import sys
sys.path.append("/home/jetson/workspace/vehicle_wbt/")


import time
import threading
import os
import numpy as np
from task_func import MyTask
from log_info import logger
from car_wrap import MyCar
from tools import CountRecord
import math
from ernie_bot.base import answer

import cv2
import numpy as np
import numpy as np
import cv2
import numpy as np

def extract_lane_and_centerline(image):
    # 1. 转换为 HSV 颜色空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 2. 黄色 HSV 范围（你可以微调以适配你的摄像头）
    lower_orange = np.array([5, 60, 60])
    upper_orange = np.array([30, 255, 255])

    # 3. 提取黄色部分
    mask = cv2.inRange(hsv, lower_orange, upper_orange)

    # 4. 可选：形态学操作消除噪点
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 5. 使用 Hough 变换检测线条
    lines = cv2.HoughLinesP(mask, rho=1, theta=np.pi / 180, threshold=50,
                            minLineLength=50, maxLineGap=20)

    if lines is None:
        print("未检测到车道线")
        return None, None, image

    height, width = image.shape[:2]
    mid_x = width // 2

    left_lines = []
    right_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x1 < mid_x and x2 < mid_x:
            left_lines.append(((x1, y1), (x2, y2)))
        elif x1 > mid_x and x2 > mid_x:
            right_lines.append(((x1, y1), (x2, y2)))

    def fit_line(points):
        xs, ys = [], []
        for (x1, y1), (x2, y2) in points:
            xs.extend([x1, x2])
            ys.extend([y1, y2])
        if len(xs) < 2:
            return None
        return np.polyfit(ys, xs, deg=1)  # x = a*y + b

    left_fit = fit_line(left_lines)
    right_fit = fit_line(right_lines)

    if left_fit is None or right_fit is None:
        print("无法拟合左右车道线")
        return None, None, image

    # 获取左右车道线底部和顶部的 x 值
    y1 = height
    y2 = int(height * 0.6)

    def get_x(fit, y):
        return int(fit[0] * y + fit[1])

    x1_left = get_x(left_fit, y1)
    x2_left = get_x(left_fit, y2)
    x1_right = get_x(right_fit, y1)
    x2_right = get_x(right_fit, y2)

    # 车道中心线两端点
    center_start = ((x1_left + x1_right) // 2, y1)
    center_end = ((x2_left + x2_right) // 2, y2)

    # ⬇⬇ 构造小车方向向量 v1：向上，即 y 轴负方向
    v1 = np.array([0, 1])
    
    # 车道方向向量：center_end - center_start，并手动翻转 y 轴方向
    v2 = np.array([
        center_end[0] - center_start[0],
        -(center_end[1] - center_start[1])
    ])

    # 可视化（可选）
    out = image.copy()
    cv2.line(out, (x1_left, y1), (x2_left, y2), (255, 0, 0), 2)  # 左线（蓝）
    cv2.line(out, (x1_right, y1), (x2_right, y2), (0, 255, 0), 2)  # 右线（绿）
    cv2.line(out, center_start, center_end, (0, 0, 255), 2)  # 中线（红）

    return v1, v2, out



def compute_angle(v1, v2):
    # 将 v1、v2 转为单位向量（方向不变，长度为1）
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    # 顺时针为正，逆时针为负（图像坐标系 y 向下）
    angle = np.arctan2(v2[1], v2[0]) - np.arctan2(v1[1], v1[0])
    angle = np.degrees(angle)

    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360

    return angle


if __name__ == '__main__':
    my_car = MyCar()
    my_car.STOP_PARAM = False

    while True:
        image = my_car.cap_front.read()
        v1, v2, out_img = extract_lane_and_centerline(image)

        if v1 is None or v2 is None:
            print("检测失败，跳过本帧")
            continue

        angle_deg = compute_angle(np.array([0, 1]), v2)
        angle_rad = math.radians(angle_deg)  # 将角度转为弧度

        print("偏航角(°):", angle_deg, "偏航角(rad):", angle_rad)

        if abs(angle_rad) < 0.08:
            break

        my_car.set_pose_offset([0, 0, angle_rad], 1)

    
        