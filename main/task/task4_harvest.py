#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务四: 作物抓取 (收割).

实际业务逻辑委托至: main.arm.each_task.task4.target4.step_target4
(移植自 am 分支; 2026-08 P1 重写为 pick_by_vision 视觉伺服 + composite_run 并行放 bin).

完整业务流程概览:
  1. 摆臂到初始姿态 target1 (Y=-133, 大臂=+90°, 手爪=0°)
  2. 进入循环: 底盘前移 → 视觉识别作物 → 抓取作物 → 放入存储
  3. 多轮抓取完毕后, 机械臂回到 init 等待位

本文件职责: 纯薄封装. 只负责:
  - 新建或复用 RuntimeApiClient (orchestrator 传 client 场景下不复用新连接)
  - 新建 ArmClient + ArmRunner
  - lazy import 并调用 step_target4

失败语义: step_target4 内部抛出的任何异常都直接向上抛出, 由 orchestrator 统一捕获.
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient
from main.arm import ArmClient, ArmRunner


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """任务四主入口: 薄封装 step_target4.

    Args:
        client: 复用 RuntimeApiClient; None 时内部新建连接 (orchestrator 场景走复用).

    Returns:
        Dict: {
            "ok": bool,                    # step_target4 成功与否
            "task": "task4_harvest",      # 固定任务名
            "detail": step_target4 原始返回值  # 业务层详细数据
        }
    """
    cli = client or RuntimeApiClient()
    arm = ArmClient(http=cli)
    runner = ArmRunner(arm)

    # lazy import: each_task 包体积较大, 不进入 cold path
    from main.arm.each_task.task4.target4 import step_target4

    detail = step_target4(
        arm_client=arm,
        http_client=cli,
        runner=runner,
    )

    ok = bool(detail.get("ok")) if isinstance(detail, dict) else bool(detail)
    return {"ok": ok, "task": "task4_harvest", "detail": detail}