#!/usr/bin/python3
"""task6 / step b2 —— OCR 识别订单文本 + 文心大模型解析

订单文本是非结构化、口语化、模糊化的指令,需要大模型解析出:
  - 货物名称(goods_name)
  - 标准配送地址(address)

返回 {goods_name, address}
"""
import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


def step_b2_ocr_lm(client: ArmClient, runner: ArmRunner) -> dict:
    print("=== [B2] OCR + 大模型解析 ===")
    # OCR 读订单牌
    # raw_text = client._call_car("get_ocr", label="order_card").get("result")
    # 文心大模型解析(用 task.get_order)
    # parsed = client.execute_task("get_order").get("result")
    # 模拟结果
    raw_text = "请把西红柿送到 1 单元 302"
    parsed = {
        "goods_name": "西红柿",
        "address": "1单元302",
        "raw_text": raw_text,
    }
    print(f"  [OCR] 原始文本: {raw_text}")
    print(f"  [LM]  解析: 货物={parsed['goods_name']}  地址={parsed['address']}")
    print("=== [B2] 完成 ===\n")
    return parsed


def main() -> None:
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_b2_ocr_lm(client, runner)


if __name__ == "__main__":
    main()
