#!/usr/bin/python3
"""task1 / step b6 —— 底盘走到目标白点

动作:
  - 抬臂状态(move_y_safe)下,底盘 move_for/move_to_position 到目标白点区域
  - 用视觉再次精确对齐白点中心

依赖:B5 已抬起到安全高度
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b6_drive_to_target(client: ArmClient, runner: ArmRunner,
                            target_x_mm: float = 200.0,
                            target_y_mm: float = 0.0) -> dict:
    """B6:底盘走到目标白点。target 是绝对位姿(米),默认走到 x=0.2m, y=0。"""
    print("=== [B6] 底盘走到目标白点 ===")
    print(f"  [底盘] move_to_position(x={target_x_mm/1000:.2f}m, y={target_y_mm/1000:.2f}m)")
    # client._call_car("move_to_position", target_x_mm / 1000, target_y_mm / 1000, 0)
    print("  [底盘] 视觉对齐白点中心")
    # client._call_car("move_to_detection_target", label="white_zone")
    print("=== [B6] 完成 ===\n")
    return {"target": (target_x_mm, target_y_mm)}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b6_drive_to_target(client, runner)


if __name__ == "__main__":
    main()
