#!/usr/bin/python3
"""task2 / step b1 —— 视觉识别水塔和指示牌(水量需求)

指示牌上水滴数量 = 该水塔需要的水块数。
返回:{tower_id: "left"/"right", need_count: N, target_x_mm: ...}
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b1_detect_tower(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [B1] 识别水塔 + 指示牌 ===")
    # det = client._call_car("get_detection_results", label="water_tower")
    # ocr = client._call_car("get_ocr", label="water_indicator")
    # 模拟数据:左塔需要 2 块,右塔需要 1 块
    towers = [
        {"id": "left",  "need": 2, "x_mm":  80.0},
        {"id": "right", "need": 1, "x_mm": 220.0},
    ]
    print(f"  -> 识别到 {len(towers)} 个水塔:")
    for t in towers:
        print(f"     {t['id']}  需要 {t['need']} 块  x={t['x_mm']:.0f}mm")
    print("=== [B1] 完成 ===\n")
    return {"towers": towers}


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b1_detect_tower(client, runner)


if __name__ == "__main__":
    main()
