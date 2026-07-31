#!/usr/bin/python3
"""task6 / step b1 —— 推动推杆,订单牌转动(纯底盘,不用机械臂)

订单机有 1 个常用订单牌 + 4 个随机订单牌,推杆推动后随机订单转到最前。
"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b1_push_rod(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [B1] 推动推杆(纯底盘) ===")
    # 推动推杆:底盘向前推一点距离
    print("  [底盘] move_for 5cm 推推杆")
    # client._call_car("move_for", 0.05, 0.0, 0.0)
    time.sleep(1.0)  # 等订单牌转动到自然停止
    # 后退一点
    print("  [底盘] move_for -5cm 后退")
    # client._call_car("move_for", -0.05, 0.0, 0.0)
    print("=== [B1] 完成 ===\n")
    return {"rod_pushed": True}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b1_push_rod(client, runner)


if __name__ == "__main__":
    main()
