#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task7 / task7_deliver —— 投放外卖 (TODO).

业务未实现. 期望功能:
  - 把识别出来的蔬菜投放到外卖柜 (底盘外环 + 机械臂抓取)
  - 投放完后回到寻路位

实现时机: 等 task7 业务方案确认后,把具体逻辑写到 main.arm.each_task.task7/,
        然后本文件改为包装.

orchestrator 捕获 NotImplementedError 后 warning + 跳过, 不中断流程.
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """task7 入口. 当前 raise NotImplementedError."""
    raise NotImplementedError(
        "task7_deliver (投放外卖) 业务未实现, "
        "等业务方案确认后写到 main.arm.each_task.task7/ 再包装."
    )