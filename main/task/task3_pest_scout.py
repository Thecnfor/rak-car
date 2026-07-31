#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task3 / task3_pest_scout —— 害虫侦察 + 射击 (TODO).

业务未实现. 期望功能:
  - 识别场地里的害虫 (YOLO 推理 / paddleocr 标牌识别)
  - 用 PoutD (气枪) 射击

实现时机: 等 task3 业务方案确认后,把具体逻辑写到 main.arm.each_task.task3/,
        然后本文件改为包装.

orchestrator 捕获 NotImplementedError 后 warning + 跳过, 不中断流程.
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """task3 入口. 当前 raise NotImplementedError."""
    raise NotImplementedError(
        "task3_pest_scout (害虫侦察 + 射击) 业务未实现, "
        "等业务方案确认后写到 main.arm.each_task.task3/ 再包装."
    )