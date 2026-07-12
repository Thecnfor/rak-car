#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import os
import sys

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.append(PROJECT_ROOT)

from main.api_client import RuntimeApiClient


def print_result(title, job):
    print(f"\n=== {title} ===")
    print(json.dumps(job, ensure_ascii=False, indent=2))
    return job.get("result")


def main():
    client = RuntimeApiClient()

    print("API_BASE =", client.api_base)
    print("API_PREFIX =", client.api_prefix)
    print("\n等待小车就绪...")
    client.wait_until_ready()

    # 可选：如果你希望每次脚本开始都重新初始化，打开这一行。
    # client.init_runtime(force=True, reset_arm=False, reset_position=True)

    # 可选：起手蜂鸣一次，确认链路在线。
    # print_result("beep", client.call("car", "beep", timeout=20))

    # ==================== 八任务 API 编排模板 ====================
    # 和官方 `car_start_2026.py` 一样，这里保留同样的任务顺序。
    # 你要跑哪个，就把对应行取消注释。

    # print_result(
    #     "巡线测试",
    #     client.execute_task(
    #         "auto_lane_tracing",
    #         timeout=300,
    #         speed=0.3,
    #         dis_hold=99,
    #     ),
    # )

    # print_result("播种任务", client.execute_task("auto_seeding", timeout=600))

    # animal_list = print_result(
    #     "识别虫害",
    #     client.execute_task("target_shooting_detection", timeout=600),
    # )

    # print_result("灌溉任务", client.execute_task("water_tower_task", timeout=900))

    # print_result(
    #     "射击除害",
    #     client.execute_task(
    #         "target_shooting",
    #         timeout=600,
    #         animal_list=animal_list,
    #     ),
    # )

    # print_result("作物收集", client.execute_task("crop_harvesting", timeout=900))

    # print_result("作物储存", client.execute_task("sort_and_store", timeout=900))

    # order_list = print_result(
    #     "订单获取",
    #     client.execute_task("get_order", timeout=900),
    # )

    # print_result(
    #     "订单配送",
    #     client.execute_task(
    #         "order_delivery",
    #         timeout=900,
    #         order_list=order_list,
    #     ),
    # )

    # ==================== 单步能力调试示例 ====================
    # 如果你要脱离任务流，直接写这些就能开始业务开发。

    # print_result("底盘位姿", client.call("car", "get_odometry", timeout=20))
    # print_result("左右 IR", client.call("car", "get_all_ir_distance", timeout=20))
    # print_result("电池电压", client.call("car", "get_battery_voltage", timeout=20))
    # print_result("机械臂状态", client.call("car", "get_arm_state", timeout=20))
    # print_result(
    #     "原始麦轮速度",
    #     client.call(
    #         "car",
    #         "set_chassis_velocity",
    #         timeout=20,
    #         x=0.10,
    #         y=0.00,
    #         z=0.00,
    #         duration=0.20,
    #     ),
    # )
    # print_result("屏幕显示", client.call("car", "show_text", "API READY", timeout=20))
    # print_result(
    #     "灯带测试",
    #     client.call("car", "set_light_color", timeout=20, led_id=0, r=0, g=255, b=0),
    # )

    print("\n模板已准备好。按需取消注释对应任务或动作即可。")


if __name__ == "__main__":
    main()
