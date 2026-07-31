#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task4 / task4_harvest —— 抓取作物.

业务逻辑: main.arm.each_task.task4.target4.step_target4
(4642 LOC, am 分支移植). 完整业务流程:
  - 摆臂到 target1 (y=-133, arm=+90°, hand=0°)
  - 循环: 底盘前移 + 视觉识别 + 抓球
  - 多轮抓取后回 init 位

本文件只做薄封装: new ArmClient + ArmRunner, 调 step_target4.
如果 client 已传就复用 (orchestrator 场景);否则内部 new RuntimeApiClient.

失败语义: step_target4 抛任何异常都往上抛, orchestrator 捕获.
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient
from main.arm import ArmClient, ArmRunner


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """task4 入口. 包装 each_task.task4.target4.step_target4.

    Args:
        client: 复用的 RuntimeApiClient;None 时内部 new.

    Returns:
        {"ok": bool, "task": "task4_harvest", "detail": step_target4 返回值}
    """
    cli = client or RuntimeApiClient()
    arm = ArmClient(http=cli)
    runner = ArmRunner(arm)

    # lazy import: each_task 包是业务代码,不进 cold path
    from main.arm.each_task.task4.target4 import step_target4

    detail = step_target4(
        arm_client=arm,
        http_client=cli,
        runner=runner,
    )

    ok = bool(detail.get("ok")) if isinstance(detail, dict) else bool(detail)
    return {"ok": ok, "task": "task4_harvest", "detail": detail}