#!/usr/bin/python3
"""task4 / step b3 —— 把果实临时存到储存仓

储存仓是底盘上的储存盒(car.set_storage),不靠机械臂放。
机械臂只需要把果实移到储存仓上方,放下来。
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b3_store_fruit(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [B3] 存入储存仓 ===")
    # 底盘开回储存仓上方
    print("  [底盘] 回到储存仓上方")
    # client._call_car("move_to_position", 0, 0, 0)
    # arm 把果实放到储存仓开口
    print("  [arm]  move_xy(x=0, y=40)  储存仓开口高度")
    runner.move_xy(x_mm=0.0, y_mm=40.0)
    print("  [arm]  grasp(False) 释放")
    runner.grasp(False, timeout=10)
    print("  [底盘] car.set_storage(close) 关仓门")
    client._call_car("set_storage", True, timeout=10)
    print("=== [B3] 完成 ===\n")
    return {"stored": True}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b3_store_fruit(client, runner)


if __name__ == "__main__":
    main()
