# -*- coding: utf-8 -*-
"""
任务: target_shooting()

对应 car_task_function.py 中的 target_shooting(animal_list)
机械臂动作：
- set_arm_pose(arm="LEFT", hand="UP")
- set_arm_pose(x=0.3, y=0.02)
"""

import time
from car_wrap_2026 import MyCar, kill_other_python


def target_shooting(animal_list=[0, 0, 0, 0]):
    step = 0.16
    relative_loc = []
    last_index = -1
    d_x = 0.2

    for idx, value in enumerate(animal_list):
        if value == 0:
            if last_index == -1:
                dist = idx * step
            else:
                dist = (idx - last_index) * step
            relative_loc.append(dist)
            last_index = idx

    my_car.arm.set_arm_pose(arm="LEFT", hand="UP")
    my_car.arm.set_arm_pose(x=0.3, y=0.02)

    my_car.lane_dis_offset(speed=0.3, dis_hold=3.0)
    my_car.move_for([-0.2, 0, 0])
    my_car.move_to_detection_target(delta_x=d_x, delta_y=None, sort_pos=(d_x, 0))

    for dis in relative_loc:
        my_car.lane_dis_offset(speed=0.3, dis_hold=dis)
        cls_id, label = my_car.move_to_detection_target(
            delta_x=d_x, delta_y=None, sort_pos=(d_x, 0)
        )
        time.sleep(5)
        my_car.beep()
        my_car.shooting()
        time.sleep(5)

    my_car.lane_dis_offset(speed=0.3, dis_hold=0.48 - sum(relative_loc))


if __name__ == "__main__":
    from init import init
    init(reset_arm=True)
    target_shooting()