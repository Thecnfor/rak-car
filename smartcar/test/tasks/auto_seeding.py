# -*- coding: utf-8 -*-
"""
任务: auto_seeding()

对应 car_task_function.py 中的 auto_seeding()
机械臂动作：
- set_arm_pose(0.0, 0.2, "LEFT", "DOWN")
- move_y_position / move_x_position / set_arm_pose("RIGHT")
- grasp(True) / grasp(False)
- adjust_arm_position()
"""

import math
import time
from car_wrap_2026 import MyCar, kill_other_python


def auto_seeding():
    x_length = 0.45
    dis = 0.55
    heading = math.pi / 4
    sin45 = math.sin(heading)
    cylinder_loc = {
        "cylinder_3": [x_length + dis * sin45, dis * sin45, heading],
        "cylinder_2": [x_length + (dis + 0.15) * sin45, (dis + 0.15) * sin45, heading],
        "cylinder_1": [x_length + (dis + 0.3) * sin45, (dis + 0.3) * sin45, heading],
    }
    cylinder_list = ["cylinder_3", "cylinder_2", "cylinder_1"]
    cylinder_set_list = {}

    # 机械臂初始位姿
    my_car.arm.set_arm_pose(0.0, 0.2, "LEFT", "DOWN")
    my_car.lane_dis_offset(speed=0.3, dis_hold=0.85)
    time.sleep(0.5)

    for i in range(3):
        my_car.move_to_position(cylinder_loc[cylinder_list[i]])
        my_car.move_to_detection_target()
        x, y, z = my_car.get_odometry()
        pose = [x, y, z, my_car.arm.x_get_position()]
        cylinder_set_list[cylinder_list[i]] = pose
        my_car.beep()

    for i in range(3):
        my_car.arm.move_y_position(0.2)
        my_car.arm.move_x_position(0.3)
        my_car.arm.set_arm_pose(arm="RIGHT")

        my_car.move_to_position(cylinder_loc[cylinder_list[i]])
        time.sleep(0.5)
        cls_id, label = my_car.move_to_detection_target()
        pose = cylinder_set_list[label]

        my_car.adjust_arm_position()
        my_car.arm.grasp(True)
        my_car.arm.move_y_position(0.01)
        time.sleep(0.5)
        my_car.arm.move_y_position(0.2)

        my_car.arm.move_x_position(pose[3])
        my_car.arm.set_arm_pose(arm="LEFT")
        time.sleep(1)
        my_car.move_to_position(pose[:3])
        my_car.adjust_arm_position()
        my_car.arm.move_y_position(0.04)
        my_car.arm.grasp(False)
        time.sleep(1)

    my_car.arm.move_y_position(0.1)
    my_car.arm.set_arm_pose(hand="UP")
    my_car.arm.move_x_position(0.15)
    my_car.move_to_position(cylinder_loc[cylinder_list[0]])


if __name__ == "__main__":
    from init import init
    init(reset_arm=True)
    auto_seeding()