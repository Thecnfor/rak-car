#!/usr/bin/python3
"""task1 / step b7 —— 放到白点

动作:
  - move_xy(target_x_mm, 5)   移到白点正上方
  - move_y(0)                 y 触底
  - grasp(False)              关泵,种子落下
  - move_y(safe_y)            抬起脱离

依赖:B6 底盘已对齐白点
⚠️ grasp() 当前 api.py:234 有 bug,需要修
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b7_place(client: ArmClient, runner: ArmRunner,
                 target_x_mm: float, safe_y_mm: float = -80.0) -> dict:
    """B7:放到白点。"""
    print("=== [B7] 放到白点 ===")
    print(f"  [arm] move_xy(x={target_x_mm:.1f}mm, y=5.0mm)  移到白点正上方")
    runner.move_xy(x_mm=target_x_mm, y_mm=-5.0)
    print("  [arm] move_y(0)  下降触底")
    runner.move_y(y_mm=0.0)
    print("  [arm] grasp(False)  关真空泵,种子落下")
    runner.grasp(False, timeout=10)
    print(f"  [arm] move_y({safe_y_mm:.1f}mm)  抬起脱离")
    runner.move_y(y_mm=safe_y_mm)
    print("=== [B7] 完成 ===\n")
    return {"placed_at": target_x_mm}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b7_place(client, runner, target_x_mm=200.0)


if __name__ == "__main__":
    main()
