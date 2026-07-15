#!/usr/bin/python3
"""task5 / step b2 —— 识别果实颜色 + 决定目标仓

赛前指定颜色对应仓:
  红色 -> 高位仓(单色标签)
  绿色 -> 低位仓(双色)

返回 {color, target_bin, target_x_mm, target_y_mm, is_high}
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


# 赛前公布(占位)
COLOR_TO_BIN = {
    "red":   {"bin": "high",  "x_mm": 100.0, "y_mm": 120.0},  # 高位
    "green": {"bin": "low",   "x_mm": 200.0, "y_mm":  40.0},  # 低位
}


def step_b2_detect_color(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [B2] 识别果实颜色 + 选仓 ===")
    # 这里应该是: 视觉识别手里吸着的果实颜色
    # 简化:从储存仓顺序取,假设红绿红绿
    # 实际应该用 client._call_car("get_detection_results", label="held_fruit")
    detected_color = "red"   # 占位
    target = COLOR_TO_BIN[detected_color]
    print(f"  [视觉] 当前果实颜色: {detected_color}")
    print(f"  [决策] 放入 {target['bin']} 仓  x={target['x_mm']}  y={target['y_mm']}")
    print("=== [B2] 完成 ===\n")
    return {"color": detected_color, **target}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b2_detect_color(client, runner)


if __name__ == "__main__":
    main()
