#!/usr/bin/python3
"""task4 / step b1 —— 视觉识别果实(2 种颜色,4cm 球)

返回 fruits: [{color: "red"/"green", x_mm, y_mm, ...}, ...]
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b1_detect_fruit(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [B1] 视觉识别果实 ===")
    # det = client._call_car("get_detection_results", label="fruit")
    # 模拟:4 个果实,2 红 2 绿,位置随机
    fruits = [
        {"color": "red",   "x_mm":  60.0, "y_mm": 5.0},
        {"color": "red",   "x_mm":  90.0, "y_mm": 5.0},
        {"color": "green", "x_mm": 150.0, "y_mm": 5.0},
        {"color": "green", "x_mm": 180.0, "y_mm": 5.0},
    ]
    print(f"  -> 识别到 {len(fruits)} 个果实")
    for f in fruits:
        print(f"     {f['color']:<6} x={f['x_mm']:.0f}mm")
    print("=== [B1] 完成 ===\n")
    return {"fruits": fruits}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b1_detect_fruit(client, runner)


if __name__ == "__main__":
    main()
