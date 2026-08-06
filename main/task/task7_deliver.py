#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务七: 外卖投放 (按 task6 订单 + target 识别 → 匹配 → 抓取 → 投递).

实际业务逻辑委托至: ``main.arm.each_task.task7.the_final.main``.
``the_final.py`` 是 task7 完整编排器, 实现:

  ┌─── 循环 (max N 次, 默认 2) ───┐
  │  Step 0: pingcang 平仓 (储存仓 → +90°, 默认开, 失败软警告可配置 strict)
  │  Step 1: duiying.run()         target OCR + liebiao 比对, 拿到 matches
  │    ↓                           (matches: [{source, list_no, name, goods, target_id, target_label}])
  │  Step 2: 检查 matches            matches 数: 0 / 1 / 2
  │    ↓ 0 个                       ↓ 1+ 个
  │    底盘前进 60cm                遍历每个 match, 每个独立执行:
  │    再次 duiying (回 Step 1)      ├ runner.suck()  (启动吸气)
  │                                   └ get_position<source>.run()  (5 步纯臂摆位)
  │                                   └ position<target_id>.run()     (底盘+臂+底盘, 含 Step 放气)
  │                                全部执行完 → continue (本轮结束)
  └─────────────────────────────────┘

⚠️ 已知约束:
  - ``the_final.main(argv)`` 内部会自行执行 ``ArmClient.connect()``,
    不接受外部传入的 client. 因此本 wrapper 的 client 参数会被忽略,
    调用方须以无参方式调用 ``run()``.
  - the_final 默认 ``max_iterations=2``, 现场可传 ``argv=["--max-iterations", "N"]`` 调整。
  - pingcang 失败默认软警告不阻塞 (``--strict-pingcang`` 切到 abort 旧行为)。
  - runtime 卡死 (HTTP 504) 时 pingcang 会失败, 软警告让主循环照跑,
    用户现场仍能看到 OCR + dispatch 链路是否正常。

跑法:
    # ✅ 跟其它 task 同款 — orchestrator 调 (包导入, sys.path 自动解决):
    from main.task import TASK_RUNNERS
    TASK_RUNNERS[7](client=None)

    # ✅ 单独跑 (包导入, 推荐):
    cd /home/jetson/workspace/rak-car   # 或本地仓库根目录
    python -m main.task.task7_deliver
    python -m main.task.task7_deliver --max-iterations 1
    python -m main.task.task7_deliver --skip-pingcang
    python -m main.task.task7_deliver --strict-pingcang   # pingcang 失败 abort (旧硬件安全)

    # ⚠️ 直接 python main/task/task7_deliver.py 会 ModuleNotFoundError (sys.path 缺 repo root),
    #    跟 task1/2/4/5/6 同款约定 (必须走 -m 包导入), 不在本文件处理
"""
from __future__ import annotations

import sys
from typing import Any, Dict, List, Optional

from main.api_client import RuntimeApiClient


def run(client: Optional[RuntimeApiClient] = None,
        argv: Optional[List[str]] = None) -> Dict[str, Any]:
    """任务七主入口: 薄封装 ``the_final.main``.

    Args:
        client: 形式参数, 实际被忽略 (``the_final.main`` 内部自建 ``ArmClient``).
                保留它是为了跟其它 ``taskN_xxx.run(client=None)`` 签名一致。
        argv: 透传给 ``the_final.main`` 的 CLI 参数列表 (None = 用默认参数)。
              例如 ``["--max-iterations", "1", "--skip-pingcang"]``。

    Returns:
        Dict: {
            "ok": bool,                  # rc == 0 即为成功
            "task": "task7_deliver",     # 固定任务名
            "rc": int,                   # the_final.main 返回码 (0 = OK)
            "detail": str                # 详情请查 the_final.main 的日志输出
        }
    """
    # lazy import: each_task 包是业务代码
    from main.arm.each_task.task7 import the_final

    rc = the_final.main(argv=argv)      # 内部执行 pingcang + duiying + dispatch

    ok = (rc == 0)
    return {
        "ok": ok,
        "task": "task7_deliver",
        "rc": rc,
        "detail": "see the_final.main logs (pingcang + duiying + matches + position*)",
    }


def main(argv: Optional[List[str]] = None) -> int:
    """CLI 入口: 让本文件可直接 ``python main/task/task7_deliver.py`` 跑。

    透传 argv 给 ``the_final.main`` (它自带 argparse, --help / --max-iterations /
    --forward / --skip-pingcang / --strict-pingcang / --pingcang-* 都已支持)。

    Returns:
        int: 0 = 任务成功, 非 0 = 失败 (the_final.main 返回码)。
    """
    result = run(client=None, argv=argv)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
