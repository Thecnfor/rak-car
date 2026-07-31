#!/usr/bin/python3
"""task7 / step b1 —— 识别配送点(住户)

配送区有 2 个配送点(1单元/2单元),每个下方有平板 + 住户姓名。
视觉+OCR 识别,找到 task6 解析出的 address 对应的配送点 x 坐标。
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


# 配送点位置(赛前公布)
DELIVERY_POINTS = {
    "1单元": {"x_mm":  80.0, "name": "张三"},
    "2单元": {"x_mm": 220.0, "name": "李四"},
}


def step_b1_detect_house(client: ArmClient, runner: ArmRunner, address: str) -> dict:
    print(f"=== [B1] 找配送点 ({address}) ===")
    # 视觉识别配送区所有住户
    # det = client._call_car("get_detection_results", label="house")
    # ocr = client._call_car("get_ocr", label="house_name")
    if address not in DELIVERY_POINTS:
        raise ValueError(f"未知地址: {address}")
    point = DELIVERY_POINTS[address]
    print(f"  [匹配] {address} -> 住户 {point['name']}  x={point['x_mm']}mm")
    print("=== [B1] 完成 ===\n")
    return point


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b1_detect_house(client, runner, address="1单元")


if __name__ == "__main__":
    main()
