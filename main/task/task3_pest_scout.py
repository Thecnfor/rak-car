#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务三: 害虫侦察 + 害虫射击 (暂未实现).

期望功能:
  1. 巡路过程中识别场地里的害虫位置 (YOLO 推理 / PaddleOCR 标牌识别)
  2. 使用 PoutD (气枪/发射器) 对识别到的害虫进行射击

当前状态: TODO 占位. 等 task3 业务方案确认后, 把具体流程写到
        main.arm.each_task.task3/, 然后本文件改为薄封装调用.

容错语义: orchestrator 捕获 NotImplementedError 后仅 warning + 跳过, 不中断主任务流.
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """任务三主入口 (当前占位, raise NotImplementedError).

    Args:
        client: 可选 RuntimeApiClient (当前未使用, 仅保持接口一致)

    Returns:
        永远不会正常返回, 直接抛出 NotImplementedError.
    """
    raise NotImplementedError(
        "task3_pest_scout (害虫侦察 + 射击) 业务暂未实现, "
        "等业务方案确认后, 将流程写到 main.arm.each_task.task3/ 再改为此文件的薄封装."
    )
