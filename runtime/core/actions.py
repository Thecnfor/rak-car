#!/usr/bin/python3
# -*- coding: utf-8 -*-
# 2026-07-16 重构：删 TASK_ACTION_NAMES 与 get_task_actions。
# 任务逻辑（自动播种/灌溉/收割/订单）由 main/ 业务层用 CAR_ACTIONS/ARM_ACTIONS 编排，
# runtime 只暴露底层 action 接口，不再负责"任务"。
# 2026-08-10: 新增 run_task1 — runtime 层 task1 核心逻辑，进程内直调跳过网络栈。
# 2026-08-10: 新增 run_task2 — runtime 层 task2 (water_tower_task) 核心逻辑，进程内直调。

from runtime.services.task1_runner import run_task1
from runtime.services.task2_runner import run_task2
from runtime.tasks.task4_direct import run as run_task4

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
    # 2026-08-01: 暴露 arm_feed 控制 —— 让视觉伺服等高频 arm 动作有路可走 (绕开 arm_queue 拥堵)。
    # 安全前提:arm_feed 不被 lane_follow 外环依赖 (lane_follow 用 lane_feed),所以暂停安全。
    # 用法:track 测试前 stop_arm_feed,跑完 start_arm_feed(hz=20) 恢复。
    "start_arm_feed": lambda car, *args, **kwargs: car.start_arm_feed(*args, **kwargs),
    "stop_arm_feed": lambda car, *args, **kwargs: car.stop_arm_feed(**kwargs) if kwargs else car.stop_arm_feed(),
    "restart_arm_feed": lambda car, *args, **kwargs: car.restart_arm_feed(*args, **kwargs),
    # 2026-07-31: 左右 IR / 底盘里程计守护线程开关（默认 50Hz auto-start）。
    # 与 start_lane_feed / start_arm_feed / start_task_feed 同构。
    "start_ir_feed": lambda car, *args, **kwargs: car.start_ir_feed(*args, **kwargs),
    "stop_ir_feed": lambda car, *args, **kwargs: car.stop_ir_feed(**kwargs) if kwargs else car.stop_ir_feed(),
    "restart_ir_feed": lambda car, *args, **kwargs: car.restart_ir_feed(*args, **kwargs),
    "start_odom_feed": lambda car, *args, **kwargs: car.start_odom_feed(*args, **kwargs),
    "stop_odom_feed": lambda car, *args, **kwargs: car.stop_odom_feed(**kwargs) if kwargs else car.stop_odom_feed(),
    "restart_odom_feed": lambda car, *args, **kwargs: car.restart_odom_feed(*args, **kwargs),
    "move_to_detection_target": lambda car, *args, **kwargs: car.move_to_detection_target(*args, **kwargs),
    "adjust_arm_position": lambda car, *args, **kwargs: car.adjust_arm_position(*args, **kwargs),
    "get_detection_results": lambda car, *args, **kwargs: car.get_detection_results(*args, **kwargs),
    "get_lane_results": lambda car, *args, **kwargs: car.get_lane_results(),
    "get_odometry": lambda car, *args, **kwargs: car.get_odometry(*args, **kwargs),
    "get_distance": lambda car, *args, **kwargs: car.get_distance(*args, **kwargs),
    "get_ocr": lambda car, *args, **kwargs: car.get_ocr(*args, **kwargs),
    "get_det_ocr": lambda car, *args, **kwargs: car.get_det_ocr(*args, **kwargs),
    "get_bluetooth_pad": lambda car, *args, **kwargs: car.get_bluetooth_pad(),
    "get_battery_voltage": lambda car, *args, **kwargs: car.get_battery_voltage(),
    "get_ir_distance": lambda car, *args, **kwargs: car.get_ir_distance(*args, **kwargs),
    "get_all_ir_distance": lambda car, *args, **kwargs: car.get_all_ir_distance(),
    "set_light_color": lambda car, *args, **kwargs: car.set_light_color(*args, **kwargs),
    "show_text": lambda car, *args, **kwargs: car.show_text(*args, **kwargs),
    "set_pwm_servo_angle": lambda car, *args, **kwargs: car.set_pwm_servo_angle(*args, **kwargs),
    "set_digital_output": lambda car, *args, **kwargs: car.set_digital_output(*args, **kwargs),
    "get_arm_state": lambda car, *args, **kwargs: car.get_arm_state(),
    # 2026-08-09: 进程内 arm 视觉伺服闭环 (task2 抓取下沉, main 只发目标参数,
    # 每帧读 task_feed 缓存 + 直调 arm, 无网络往返). 返回 {ok, reason, trace_hits, settled, end_arm}.
    "run_arm_servo": lambda car, *args, **kwargs: car.run_arm_servo(**kwargs),
    # 2026-08-08: 一鍵啟動 —— 讀 MC602 板上鍵（BoardKey）。純新增，供 run.py --wait-key 輪詢。
    "read_key": lambda car, *args, **kwargs: car.read_key(),
    # 2026-08-10: runtime 层 task1 主入口，进程内直调跳过网络栈。
    # 等价于 main/task/task1_seeding.run()，但所有 arm/car 调用走 SDK 方法。
    "run_task1": lambda car, *args, **kwargs: run_task1(car),
    # 2026-08-10: runtime 层 task2 (water_tower_task) 主入口，进程内直调跳过网络栈。
    # 等价于 main/task/task2_water_tower.run()，但所有 arm/car 走 SDK 直调
    # （car.composite_run / car.run_arm_servo / car.arm.move_y_position /
    # car.arm.grasp 等），不进 HTTP job_queue，不持 car_lock。
    "run_task2": lambda car, *args, **kwargs: run_task2(car),
    # 2026-08-10: task4 也在 runtime 进程内执行，客户端只发 action。
    "run_task4": lambda car, *args, **kwargs: run_task4(car, **kwargs),
}


ARM_ACTIONS = {
    "reset_position": lambda arm_obj, *args, **kwargs: arm_obj.reset_position(),
    "reset_y": lambda arm_obj, *args, **kwargs: arm_obj.reset_y(),
    # 2026-07-16 新加：opt-in 撞墙复位 + 复合复位。
    # 不接入 _create_car_locked / ensure_initialized / _auto_init_kwargs，避免 fb24b1a 描述的 pm2 循环。
    "reset_x": lambda arm_obj, *args, **kwargs: arm_obj.reset_x(**kwargs),
    "reset_all": lambda arm_obj, *args, **kwargs: arm_obj.reset_all(*args, **kwargs),
    # 2026-07-31 PR#13: 复合动作 (arm_base.composite_*)。业务层用 composite_* 替换
    # 原 pick/release/go_home 的三步串行,实现 2-3 路电机真并发。
    # 与 reset_all 同样的设计:JOB 内 ThreadPoolExecutor,JOB 间 arm_queue 仍串行。
    "composite_pick": lambda arm_obj, *args, **kwargs: arm_obj.composite_pick(**kwargs),
    "composite_release": lambda arm_obj, *args, **kwargs: arm_obj.composite_release(**kwargs),
    "composite_go_home": lambda arm_obj, *args, **kwargs: arm_obj.composite_go_home(**kwargs),
    # 2026-07-31: 四电机通用并行驱动器 (任意 1-4 路可省，None 跳过)。
    # reset_position 已经内部实现 arm+hand 并行 + y 串行收尾（init 入口），
    # 但业务层在运行时也想并发多个电机就用这个。
    "composite_run": lambda arm_obj, *args, **kwargs: arm_obj.composite_run(**kwargs),
    "composite_run_reset": lambda arm_obj, *args, **kwargs: arm_obj.composite_run_reset(**kwargs),
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
