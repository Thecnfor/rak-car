# -*- coding: utf-8 -*-
"""
任务: target_shooting_detection()

对应 car_task_function.py 中的 target_shooting_detection()
机械臂动作：
- set_arm_pose(x=0.05, y=0.05, arm="LEFT", hand="UP")
"""

import time
from car_wrap_2026 import MyCar, kill_other_python


def target_shooting_detection():
    animal_list = [0, 0, 0, 0]
    my_car.arm.set_arm_pose(x=0.05, y=0.05, arm="LEFT", hand="UP")
    my_car.lane_dis_offset(speed=0.3, dis_hold=1.45)

    _x, _y, _z = my_car.get_odometry(True)
    my_car.get_distance(True)
    my_car.move_for([0, 0, 0 - _z])
    time.sleep(3)

    for i in range(4):
        my_car.lane_dis_offset(speed=0.3, dis_hold=0.15)
        time.sleep(0.5)
        cls_id, label = my_car.move_to_detection_target(delta_y=None)
        if label == "animal":
            res, analysis = my_car.animal_image_analysis()
            if res is not None:
                my_car.beep()
                animal_list[i] = res
    my_car.beep()
    my_car.beep()
    return animal_list


if __name__ == "__main__":
    from init import init
    init(reset_arm=True)
    target_shooting_detection()