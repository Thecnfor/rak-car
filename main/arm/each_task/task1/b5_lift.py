#!/usr/bin/python3
"""task1 / step b5 —— 抬起种子

动作:
  - move_y(safe_y_mm)  抬到安全高度(避免搬运时撞场地)

依赖:B4 已吸取住种子
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b5_lift(client: ArmClient, runner: ArmRunner,
                safe_y_mm: float = -80.0) -> dict:
    """B5:抬起种子到安全高度。"""
    print("=== [B5] 抬起种子 ===")
    print(f"  [arm] move_y({safe_y_mm:.1f}mm)  抬到安全高度")
    runner.move_y(y_mm=safe_y_mm)
    print("=== [B5] 完成 ===\n")
    return {"lifted_to_y": safe_y_mm}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b5_lift(client, runner)


if __name__ == "__main__":
    main()
