#!/usr/bin/python3
"""task5 / step b3 —— 把果实放入对应仓

高位: y 抬到 120mm 再放下
低位: y 在 40mm 处放下

⚠️ "高位存放" 需要 y 轴高量程(>120mm),需确认 arm_origin.yaml 标定够
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b3_place_bin(client: ArmClient, runner: ArmRunner,
                     target_x_mm: float, target_y_mm: float,
                     is_high: bool) -> dict:
    print("=== [B3] 放入仓 ===")
    # 底盘走到仓前方
    print(f"  [底盘] 走到仓 x={target_x_mm:.0f}")
    # client._call_car("move_to_position", target_x_mm / 1000, 0, 0)
    # arm 移到仓开口
    print(f"  [arm]  move_xy(x={target_x_mm:.0f}, y={target_y_mm:.0f})")
    runner.move_xy(x_mm=target_x_mm, y_mm=target_y_mm)
    print("  [arm]  grasp(False) 释放果实")
    runner.grasp(False, timeout=10)
    # 抬起脱离
    print("  [arm]  move_y(80) 抬起")
    runner.move_y(y_mm=80.0)
    if is_high:
        print("  [note] 这是高位仓,需要后续 y 复位以适应下一轮")
    print("=== [B3] 完成 ===\n")
    return {"placed": True, "bin": "high" if is_high else "low"}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b3_place_bin(client, runner, target_x_mm=100.0, target_y_mm=120.0, is_high=True)


if __name__ == "__main__":
    main()
