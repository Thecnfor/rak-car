#!/usr/bin/python3
# -*- coding: utf-8 -*-
TASK_ACTION_NAMES = [
    "auto_lane_tracing",
    "auto_seeding",
    "target_shooting_detection",
    "water_tower_task",
    "target_shooting",
    "crop_harvesting",
    "sort_and_store",
    "get_order",
    "order_delivery",
]


def get_task_actions(task_module):
    return {name: getattr(task_module, name) for name in TASK_ACTION_NAMES}


CAR_ACTIONS = {
    "beep": lambda car, *args, **kwargs: car.beep(),
    "stop": lambda car, *args, **kwargs: car.stop(),
    "reset_position": lambda car, *args, **kwargs: car.reset_position(),
    "set_storage": lambda car, *args, **kwargs: car.set_storage(*args, **kwargs),
    "shooting": lambda car, *args, **kwargs: car.shooting(),
    "move_for": lambda car, *args, **kwargs: car.move_for(*args, **kwargs),
    "move_time": lambda car, *args, **kwargs: car.move_time(*args, **kwargs),
    "move_distance": lambda car, *args, **kwargs: car.move_distance(*args, **kwargs),
    "move_to_position": lambda car, *args, **kwargs: car.move_to_position(*args, **kwargs),
    "lane_time": lambda car, *args, **kwargs: car.lane_time(*args, **kwargs),
    "lane_dis": lambda car, *args, **kwargs: car.lane_dis(*args, **kwargs),
    "lane_dis_offset": lambda car, *args, **kwargs: car.lane_dis_offset(*args, **kwargs),
    "move_to_detection_target": lambda car, *args, **kwargs: car.move_to_detection_target(*args, **kwargs),
    "adjust_arm_position": lambda car, *args, **kwargs: car.adjust_arm_position(*args, **kwargs),
    "get_detection_results": lambda car, *args, **kwargs: car.get_detection_results(*args, **kwargs),
    "get_odometry": lambda car, *args, **kwargs: car.get_odometry(*args, **kwargs),
    "get_distance": lambda car, *args, **kwargs: car.get_distance(*args, **kwargs),
    "get_ocr": lambda car, *args, **kwargs: car.get_ocr(*args, **kwargs),
}


ARM_ACTIONS = {
    "reset_position": lambda arm_obj, *args, **kwargs: arm_obj.reset_position(),
    "reset_x": lambda arm_obj, *args, **kwargs: arm_obj.reset_x(),
    "set_arm_pose": lambda arm_obj, *args, **kwargs: arm_obj.set_arm_pose(*args, **kwargs),
    "set_hand_angle": lambda arm_obj, *args, **kwargs: arm_obj.set_hand_angle(*args, **kwargs),
    "set_arm_angle": lambda arm_obj, *args, **kwargs: arm_obj.set_arm_angle(*args, **kwargs),
    "move_x_position": lambda arm_obj, *args, **kwargs: arm_obj.move_x_position(*args, **kwargs),
    "move_y_position": lambda arm_obj, *args, **kwargs: arm_obj.move_y_position(*args, **kwargs),
    "grasp": lambda arm_obj, *args, **kwargs: arm_obj.grasp(*args, **kwargs),
    "x_get_position": lambda arm_obj, *args, **kwargs: arm_obj.x_get_position(),
}
