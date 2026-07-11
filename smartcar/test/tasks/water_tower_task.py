# -*- coding: utf-8 -*-
"""
任务: water_tower_task()

对应 car_task_function.py 中的 water_tower_task()
机械臂动作：
- set_arm_pose(x=0.0, y=0.02, arm="RIGHT", hand="UP")
- move_y_position / move_x_position / set_arm_pose
- grasp(True/False)
- adjust_arm_position
"""

import time
from car_wrap_2026 import MyCar, kill_other_python


def water_tower_task():
    water_num = {"water_l1": 1, "water_l2": 2, "water_l3": 3}
    tower_water = []
    water_loction = []
    tower_loction = {}
    my_car.arm.set_arm_pose(x=0.0, y=0.02, arm="RIGHT", hand="UP")

    my_car.lane_dis_offset(speed=0.3, dis_hold=2.0)
    my_car.get_odometry(True)
    time.sleep(1)
    my_car.move_for([0, -0.05, 0])
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    tower_water.append(label)
    my_car.beep()
    tower_loction[label] = my_car.get_odometry(True)
    headinng = tower_loction[label][2]

    my_car.arm.move_y_position(0.2)
    my_car.arm.set_arm_pose(arm="LEFT", hand="DOWN")

    def record_detection_pose():
        time.sleep(1)
        cls_id, label = my_car.move_to_detection_target()
        x, y, z = my_car.get_odometry()
        pose = [x, y, z, my_car.arm.x_get_position()]
        my_car.beep()
        return pose

    water_loction.append(record_detection_pose())
    my_car.adjust_arm_position(0.1)
    water_loction.append(record_detection_pose())
    my_car.lane_dis_offset(speed=0.3, dis_hold=0.32)
    water_loction.append(record_detection_pose())
    my_car.adjust_arm_position(-0.1)
    water_loction.append(record_detection_pose())
    my_car.lane_dis_offset(speed=0.3, dis_hold=0.32)
    x, y, z = my_car.get_odometry()
    my_car.move_for([0, -0.03, headinng - z])
    water_loction.append(record_detection_pose())
    my_car.adjust_arm_position(0.1)
    water_loction.append(record_detection_pose())

    my_car.arm.set_arm_pose(arm="RIGHT", hand="UP")
    my_car.arm.set_arm_pose(x=0.0, y=0.02)

    time.sleep(0.5)
    cls_id, label = my_car.move_to_detection_target(delta_y=None)
    tower_water.append(label)
    tower_loction[label] = my_car.get_odometry(True)

    for i, label in enumerate(reversed(tower_water)):
        water_num_ = water_num[label]
        for j in range(water_num_):
            my_car.arm.move_y_position(0.2)
            my_car.arm.move_x_position(0.0)
            my_car.arm.set_arm_pose(arm="LEFT", hand="DOWN")
            k = -(j + 1) if i == 0 else j
            my_car.move_to_position(water_loction[k][0:3])
            my_car.arm.move_x_position(water_loction[k][3])
            my_car.move_to_detection_target()
            my_car.adjust_arm_position()
            my_car.arm.grasp(True)
            my_car.arm.move_y_position(0.09)
            my_car.arm.move_y_position(0.2)
            my_car.arm.move_x_position(0.01)
            my_car.arm.set_arm_pose(arm="RIGHT", hand="UP")

            my_car.move_to_position(tower_loction[label])
            my_car.arm.move_y_position(0.01 + 0.055 * j)
            my_car.move_to_detection_target(delta_y=None)
            my_car.arm.move_x_position(0.20)
            my_car.arm.grasp(False)
            time.sleep(0.5)
            my_car.arm.move_x_position(0.15)
            time.sleep(0.5)
            my_car.arm.move_x_position(0.01)
            time.sleep(0.5)


if __name__ == "__main__":
    from init import init
    init(reset_arm=True)
    water_tower_task()