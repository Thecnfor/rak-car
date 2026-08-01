#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务七: 外卖投放 (暂未实现).

期望功能:
  1. 根据任务六读取的订单, 把识别出来的蔬菜投放到对应外卖柜格口
     (底盘外环寻路 + 机械臂抓取投放)
  2. 所有订单投放完毕后, 机械臂和底盘回到寻路待命位

当前状态: TODO 占位. 等 task7 业务方案确认后, 把具体流程写到
        main.arm.each_task.task7/, 然后本文件改为薄封装调用.

容错语义: orchestrator 捕获 NotImplementedError 后仅 warning + 跳过, 不中断主任务流.
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """任务七主入口 (当前占位, raise NotImplementedError).

    Args:
        client: 可选 RuntimeApiClient (当前未使用, 仅保持接口一致)

    Returns:
        永远不会正常返回, 直接抛出 NotImplementedError.
    """
    raise NotImplementedError(
        "task7_deliver (投放外卖) 业务暂未实现, "
        "等业务方案确认后, 将流程写到 main.arm.each_task.task7/ 再改为此文件的薄封装."
    )
