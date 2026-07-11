# -*- coding: utf-8 -*-
"""
任务: sort_and_store()

对应 car_task_function.py 中的 sort_and_store()
机械臂动作：
- move_y_position / move_x_position / set_arm_pose
- grasp(True/False)
"""

import time
from car_wrap_2026 import MyCar, kill_other_python


def sort_and_store():
    ball_list = [0.0, 0.06]

    my_car.arm.move_y_position(0.17)
    my_car.arm.move_x_position(0.30)
    my_car.arm.set_arm_pose(arm="LEFT", hand=-70)
    my_car.arm.move_y_position(0.05)

    my_car.lane_dis_offset(speed=0.3, dis_hold=2.0)
    time.sleep(0.5)
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    time.sleep(0.5)
    flag = 1 if label == "lable_blue" else 0

    for i in range(2):
        for j in range(4):
            my_car.arm.move_y_position(0.15)
            my_car.arm.set_arm_pose(arm=-107, hand=10)
            my_car.arm.move_x_position(ball_list[(i + flag) % 2])
            my_car.arm.grasp(True)
            my_car.arm.move_y_position(0.08)
            my_car.arm.move_y_position(0.15)
            my_car.arm.move_x_position(0.30)
            my_car.arm.set_arm_pose(arm=94, hand="UP")
            my_car.arm.move_y_position(0.2 - i * 0.15)
            time.sleep(0.5)
            my_car.arm.move_x_position(0.2)
            my_car.arm.grasp(False)
            time.sleep(0.5)
            my_car.arm.move_x_position(0.30)
        if i == 1:
            break
        my_car.move_for([-0.155, 0, 0])


if __name__ == "__main__":
    from init import init
    init(reset_arm=True)
    sort_and_store()