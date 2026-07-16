#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 2026-07-16 重构：删 TASK_ACTION_NAMES 与 get_task_actions。
# 任务逻辑（自动播种/灌溉/收割/订单）由 main/ 业务层用 CAR_ACTIONS/ARM_ACTIONS 编排，
# runtime 只暴露底层 action 接口，不再负责"任务"。

CAR_ACTIONS = {
    "beep": lambda car, *args, **kwargs: car.beep(),
    "stop": lambda car, *args, **kwargs: car.stop(),
    "reset_position": lambda car, *args, **kwargs: car.reset_position(),
    "set_storage": lambda car, *args, **kwargs: car.set_storage(*args, **kwargs),
    "set_storage_angle": lambda car, *args, **kwargs: car.set_storage_angle(*args, **kwargs),
    "shooting": lambda car, *args, **kwargs: car.shooting(),
    "set_shoot_state": lambda car, *args, **kwargs: car.set_shoot_state(*args, **kwargs),
    "move_for": lambda car, *args, **kwargs: car.move_for(*args, **kwargs),
    "move_time": lambda car, *args, **kwargs: car.move_time(*args, **kwargs),
    "move_distance": lambda car, *args, **kwargs: car.move_distance(*args, **kwargs),
    "move_to_position": lambda car, *args, **kwargs: car.move_to_position(*args, **kwargs),
    "set_chassis_velocity": lambda car, *args, **kwargs: car.set_chassis_velocity(*args, **kwargs),
    "lane_time": lambda car, *args, **kwargs: car.lane_time(*args, **kwargs),
    "lane_dis": lambda car, *args, **kwargs: car.lane_dis(*args, **kwargs),
    "lane_dis_offset": lambda car, *args, **kwargs: car.lane_dis_offset(*args, **kwargs),
    "start_lane_feed": lambda car, *args, **kwargs: car.start_lane_feed(*args, **kwargs),
    "stop_lane_feed": lambda car, *args, **kwargs: car.stop_lane_feed(**kwargs) if kwargs else car.stop_lane_feed(),
    "move_to_detection_target": lambda car, *args, **kwargs: car.move_to_detection_target(*args, **kwargs),
    "adjust_arm_position": lambda car, *args, **kwargs: car.adjust_arm_position(*args, **kwargs),
    "get_detection_results": lambda car, *args, **kwargs: car.get_detection_results(*args, **kwargs),
    "get_lane_results": lambda car, *args, **kwargs: car.get_lane_results(),
    "get_odometry": lambda car, *args, **kwargs: car.get_odometry(*args, **kwargs),
    "get_distance": lambda car, *args, **kwargs: car.get_distance(*args, **kwargs),
    "get_ocr": lambda car, *args, **kwargs: car.get_ocr(*args, **kwargs),
    "get_det_ocr": lambda car, *args, **kwargs: car.get_det_ocr(*args, **kwargs),
    "get_key_event": lambda car, *args, **kwargs: car.get_key_event(),
    "get_key_state": lambda car, *args, **kwargs: car.get_key_state(),
    "get_bluetooth_pad": lambda car, *args, **kwargs: car.get_bluetooth_pad(),
    "get_battery_voltage": lambda car, *args, **kwargs: car.get_battery_voltage(),
    "get_ir_distance": lambda car, *args, **kwargs: car.get_ir_distance(*args, **kwargs),
    "get_all_ir_distance": lambda car, *args, **kwargs: car.get_all_ir_distance(),
    "set_light_color": lambda car, *args, **kwargs: car.set_light_color(*args, **kwargs),
    "show_text": lambda car, *args, **kwargs: car.show_text(*args, **kwargs),
    "set_pwm_servo_angle": lambda car, *args, **kwargs: car.set_pwm_servo_angle(*args, **kwargs),
    "set_digital_output": lambda car, *args, **kwargs: car.set_digital_output(*args, **kwargs),
    "get_arm_state": lambda car, *args, **kwargs: car.get_arm_state(),
}


ARM_ACTIONS = {
    "reset_position": lambda arm_obj, *args, **kwargs: arm_obj.reset_position(),
    "reset_y": lambda arm_obj, *args, **kwargs: arm_obj.reset_y(),
    # 2026-07-16 新加：opt-in 撞墙复位 + 复合复位。
    # 不接入 _create_car_locked / ensure_initialized / _auto_init_kwargs，避免 fb24b1a 描述的 pm2 循环。
    "reset_x": lambda arm_obj, *args, **kwargs: arm_obj.reset_x(**kwargs),
    "reset_all": lambda arm_obj, *args, **kwargs: arm_obj.reset_all(**kwargs),
    "set_arm_pose": lambda arm_obj, *args, **kwargs: arm_obj.set_arm_pose(*args, **kwargs),
    "set_hand_angle": lambda arm_obj, *args, **kwargs: arm_obj.set_hand_angle(*args, **kwargs),
    "set_arm_angle": lambda arm_obj, *args, **kwargs: arm_obj.set_arm_angle(*args, **kwargs),
    "move_x_position": lambda arm_obj, *args, **kwargs: arm_obj.move_x_position(*args, **kwargs),
    "move_y_position": lambda arm_obj, *args, **kwargs: arm_obj.move_y_position(*args, **kwargs),
    "goto_position": lambda arm_obj, *args, **kwargs: arm_obj.goto_position(*args, **kwargs),
    "go_for": lambda arm_obj, *args, **kwargs: arm_obj.go_for(*args, **kwargs),
    "x_speed": lambda arm_obj, *args, **kwargs: arm_obj.x_speed(*args, **kwargs),
    "y_speed": lambda arm_obj, *args, **kwargs: arm_obj.y_speed(*args, **kwargs),
    "grasp": lambda arm_obj, *args, **kwargs: arm_obj.grasp(*args, **kwargs),
    "x_get_position": lambda arm_obj, *args, **kwargs: arm_obj.x_get_position(),
    "y_get_position": lambda arm_obj, *args, **kwargs: arm_obj.y_get_position(),
}
