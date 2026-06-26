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


def extract_lane_and_centerline(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([5, 60, 60])
    upper_orange = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    lines = cv2.HoughLinesP(mask, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=20)

    if lines is None:
        print("未检测到车道线")
        return None, None, image, None

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
        return np.polyfit(ys, xs, deg=1)  # x = ky + b

    left_fit = fit_line(left_lines)
    right_fit = fit_line(right_lines)
    angle_left =-math.pi/2-np.arctan(1 / left_fit[0])
    angle_right =-math.pi/2-np.arctan(1 / right_fit[0])
    angle=(angle_left+angle_right)/2

    if left_fit is None or right_fit is None:
        print("无法拟合左右车道线")
        return None, None, image, None

    # 计算中线拟合（取两条拟合结果中点）


    # 可选可视化
    y1 = height
    y2 = int(height * 0.6)
    def get_x(fit, y): return int(fit[0] * y + fit[1])
    x1_left, x2_left = get_x(left_fit, y1), get_x(left_fit, y2)
    x1_right, x2_right = get_x(right_fit, y1), get_x(right_fit, y2)
    center_start = ((x1_left + x1_right) // 2, y1)
    center_end = ((x2_left + x2_right) // 2, y2)

    out = image.copy()
    cv2.line(out, (x1_left, y1), (x2_left, y2), (255, 0, 0), 2)
    cv2.line(out, (x1_right, y1), (x2_right, y2), (0, 255, 0), 2)
    cv2.line(out, center_start, center_end, (0, 0, 255), 2)

    # 根据中心线斜率计算与竖直线夹角


    return out, angle


if __name__ == '__main__':
    my_car = MyCar()
    my_car.STOP_PARAM = False
    while True:
        image = my_car.cap_front.read()
        out_img, angle = extract_lane_and_centerline(image)
        print("角度:", angle)

        if abs(angle) < 0.01:
            break
        my_car.set_pose_offset([0, 0, -angle], 0.3)


