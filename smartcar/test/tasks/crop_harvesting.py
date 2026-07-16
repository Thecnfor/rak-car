# -*- coding: utf-8 -*-
"""
任务: crop_harvesting()

对应 car_task_function.py 中的 crop_harvesting()
机械臂动作：
- move_y_position / set_arm_pose
- move_x_position / set_arm_pose(arm=-115, hand=10)
- grasp(True/False) / adjust_arm_position

注（2026-07-16）：reset_x 已删除，x 位置由视觉闭环控制。
"""

import time
from car_wrap_2026 import MyCar, kill_other_python


def crop_harvesting():
    my_car.arm.move_y_position(0.2)
    my_car.arm.set_arm_pose(arm="LEFT", hand="DOWN")

    my_car.set_storage(True)
    my_car.lane_dis_offset(speed=0.3, dis_hold=2.3)
    my_car.arm.move_y_position(0.17)

    for i in range(8):
        my_car.arm.move_x_position(0.0)
        my_car.arm.set_arm_pose(arm="LEFT", hand="DOWN")
        my_car.lane_dis_offset(speed=0.3, dis_hold=0.04)
        time.sleep(0.5)
        cls_id, label = my_car.move_to_detection_target(delta_x=-0.05, time_out=3.0)
        time.sleep(0.5)
        my_car.adjust_arm_position()
        my_car.arm.grasp(True)
        time.sleep(0.3)
        my_car.arm.move_y_position(0.045)
        time.sleep(0.3)
        my_car.arm.move_y_position(0.17)
        time.sleep(0.3)
        my_car.arm.set_arm_pose(arm=-115, hand=10)
        if label == "ball_yellow":
            my_car.arm.move_x_position(0.0)
            my_car.beep()
        elif label == "ball_blue":
            my_car.arm.move_x_position(0.06)
            my_car.beep()
            my_car.beep()
        time.sleep(1)
        my_car.arm.grasp(False)
        time.sleep(1)
    my_car.set_storage(False)


if __name__ == "__main__":
    from init import init
    init(reset_arm=True)
    crop_harvesting()