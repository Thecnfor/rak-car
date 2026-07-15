#!/usr/bin/python3
"""task1 / step a —— 到达播种区

底盘动作:
  1) 短直行出基地
  2) 巡线到播种区
  3) 用视觉对准种子堆

机械臂动作:
  - set_hand("UP")    手爪抬起(避免撞场地)
  - set_side("MID")   大臂居中

依赖:运行时 runtime 在线、arm_origin.yaml 已标定
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_a_approach(client: ArmClient, runner: ArmRunner) -> dict:
    """阶段A:到达播种区。"""
    print("=== [A] 到达播种区 ===")

    # ---- 底盘 ----
    # A1 出基地
    print("  [底盘] 出基地 5cm")
    # client._call_car("move_for", 0.05, 0.0, 0.0)   # ← 需要时打开

    # A2 巡线到播种区
    print("  [底盘] 巡线 2m 到播种区")
    # client._call_car("lane_dis_offset", 0.4, 2.0)  # ← speed, dis_hold

    # A3 视觉对齐种子堆
    print("  [底盘] 视觉对齐种子堆区域")
    # client._call_car("move_to_detection_target", label="seed_area")

    # ---- 机械臂(安全位) ----
    print("  [arm]   set_hand(UP)  手爪抬起")
    runner.set_hand("UP", timeout=10)
    print("  [arm]   set_side(MID) 大臂居中")
    runner.set_side("MID", timeout=10)

    print("=== [A] 完成 ===\n")
    return {"phase": "A", "arrived": True}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_a_approach(client, runner)


if __name__ == "__main__":
    main()
