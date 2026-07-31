#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task5 / task5_sort —— 分拣作物.

业务逻辑: main.arm.each_task.task5.the_final.main
(4544 LOC, am 分支移植). 完整业务流程:
  1. 识别高仓颜色 (blue/yellow) → color A
  2. 同色球进高仓 (last_X_to_high)
  3. 底盘后撤 165mm
  4. 反色球进 LOW 仓 (last_X_to_low)

⚠️ the_final.main(argv) 不接收 client, 内部 client = ArmClient.connect().
   本 wrapper 因此忽略传入 client 参数 (call site 必须 no-arg).
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """task5 入口. 包装 each_task.task5.the_final.main.

    Args:
        client: 传入但被忽略 (the_final 不接收).

    Returns:
        {"ok": bool, "task": "task5_sort", "rc": main 的 return code, "detail": str}
    """
    # lazy import: each_task 包是业务代码
    from main.arm.each_task.task5 import the_final

    rc = the_final.main(argv=None)  # 用默认 CLI args (内部识别色)

    # the_final 用 EXIT_OK/EXIT_BAD_COLOR 常量
    ok = (rc == 0)  # EXIT_OK = 0
    return {"ok": ok, "task": "task5_sort", "rc": rc, "detail": "see the_final.main logs"}