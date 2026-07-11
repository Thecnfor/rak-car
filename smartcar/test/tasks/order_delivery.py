# -*- coding: utf-8 -*-
"""
任务: order_delivery()

对应 car_task_function.py 中的 order_delivery(order_list)
机械臂动作：
- move_y_position / move_x_position
- set_arm_pose / grasp(True/False)
"""

import time
from car_wrap_2026 import MyCar, kill_other_python


def find_name(name="name"):
    name_list = []
    for i in range(3):
        my_car.move_to_detection_target(delta_y=None)
        time.sleep(1)
        dets = my_car.get_detection_results(sort_pos=(0, 0.5), limit_x=0.3)
        for j, det in enumerate(dets):
            text = my_car.get_det_ocr(det)
            if text == name:
                return i, j
        if i < 2:
            my_car.lane_dis_offset(speed=0.3, dis_hold=0.11)


def order_delivery(order_list=[
    {"name": "李四", "goods": "芹菜", "address": 2},
    {"name": "钱七", "goods": "青椒", "address": 2},
]):
    my_car.lane_dis_offset(speed=0.3, dis_hold=3.25)

    time.sleep(1)
    my_car.arm.move_y_position(0.2)
    my_car.arm.move_x_position(0.3)
    my_car.arm.set_arm_pose(arm="LEFT", hand=-70)
    time.sleep(1)
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    if label is None:
        my_car.lane_dis_offset(speed=0.3, dis_hold=0.12)
    time.sleep(1)
    loc_flag = 1
    loc = my_car.get_odometry(True)

    for i, order in enumerate(order_list):
        my_car.move_to_position(loc)
        if order["address"] > loc_flag:
            my_car.lane_dis_offset(speed=0.3, dis_hold=0.56)
            loc_flag = 2
            loc = my_car.get_odometry(True)
        time.sleep(0.5)

        my_car.arm.move_y_position(0.13)
        my_car.arm.move_x_position(0.3)
        my_car.arm.set_arm_pose(arm="LEFT", hand='UP')

        _x, y = find_name(order["name"])
        my_car.arm.set_arm_pose(arm="RIGHT", hand="DOWN")
        my_car.arm.move_x_position(0.0)
        my_car.arm.grasp(True)
        my_car.arm.move_y_position(0.135 - i * 0.05)
        my_car.arm.move_y_position(0.155 - i * 0.05)
        my_car.arm.move_x_position(0.2)
        my_car.arm.set_arm_pose(arm="LEFT", hand=-70)
        my_car.arm.move_y_position(y * 0.09)
        my_car.arm.move_x_position(0.1)
        my_car.arm.grasp(False)
        time.sleep(1)
        my_car.arm.move_x_position(0.15)
        my_car.arm.set_arm_pose(arm="LEFT", hand=-80)
        time.sleep(0.5)
        my_car.arm.move_x_position(0.2)

    if loc_flag == 1:
        my_car.lane_dis_offset(speed=0.3, dis_hold=1.7)
    else:
        my_car.lane_dis_offset(speed=0.3, dis_hold=1.1)


if __name__ == "__main__":
    from init import init
    init(reset_arm=True)
    order_delivery()