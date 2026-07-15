#!/usr/bin/python3
"""task5 / step b1 —— 从储存仓拿出下一个果实

储存仓在底盘上,arm 需要先开仓门,再把果实吸出来。
"""
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b1_take_fruit(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [B1] 从储存仓拿出果实 ===")
    # 开仓门
    print("  [底盘] car.set_storage(open)")
    client._call_car("set_storage", False, timeout=10)
    time.sleep(0.5)
    # arm 移到储存仓开口,吸取
    print("  [arm]  set_hand(DOWN) + move_xy(0, 40)")
    runner.set_side("MID", timeout=10)
    runner.set_hand("DOWN", timeout=10)
    runner.move_xy(x_mm=0.0, y_mm=-40.0)
    print("  [arm]  move_y(0) 伸进去 + grasp(True)")
    runner.move_y(y_mm=0.0)
    time.sleep(0.3)
    runner.grasp(True, timeout=10)
    print("  [arm]  move_y(80) 抬起")
    runner.move_y(y_mm=-80.0)
    # 关仓门
    print("  [底盘] car.set_storage(close)")
    client._call_car("set_storage", True, timeout=10)
    print("=== [B1] 完成 ===\n")
    return {"holding": "fruit"}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b1_take_fruit(client, runner)


if __name__ == "__main__":
    main()
