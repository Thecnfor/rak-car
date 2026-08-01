#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务五: 作物颜色分拣 (按颜色分高/低仓放置).

实际业务逻辑委托至: main.arm.each_task.task5.the_final.main
(约 4544 行, 移植自 am 分支).

完整业务流程概览:
  1. 视觉识别高位仓颜色 (蓝/黄) → 记为 color_A
  2. 将与 color_A 同色的球放入 HIGH 高仓 (last_X_to_high)
  3. 底盘后撤 165mm
  4. 将剩余反色的球放入 LOW 低仓 (last_X_to_low)

⚠️ 已知约束:
   the_final.main(argv) 内部会自行执行 ArmClient.connect(),
   不接受外部传入的 client. 因此本 wrapper 的 client 参数会被忽略,
   调用方须以无参方式调用 run().
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """任务五主入口: 薄封装 the_final.main.

    Args:
        client: 形式参数, 实际被忽略 (the_final.main 内部自建 ArmClient).

    Returns:
        Dict: {
            "ok": bool,                  # rc == EXIT_OK 即为成功
            "task": "task5_sort",      # 固定任务名
            "rc": int,                 # the_final.main 返回码 (EXIT_OK=0)
            "detail": str              # 详情请查 the_final.main 的日志输出
        }
    """
    # lazy import: each_task 包是业务代码
    from main.arm.each_task.task5 import the_final

    rc = the_final.main(argv=None)  # 使用默认 CLI 参数 (内部执行颜色识别)

    # the_final.main 用常量: EXIT_OK = 0, EXIT_BAD_COLOR = 非 0
    ok = (rc == 0)
    return {"ok": ok, "task": "task5_sort", "rc": rc, "detail": "see the_final.main logs"}
