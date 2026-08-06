#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务三: 害虫侦察 + 害虫射击 (thin wrapper over task3_pipeline).

业务方案:在 main/task/task3/task3_pipeline.py 实现的 drive + LLM 判别 +
shoot_target 三段式流水线。本文件只是 orchestrator registry 期望的薄封装
(run(client=None) -> Dict),实际跑 subprocess 调 task3_pipeline.main()。

容错语义: 与占位期一致, 但不再抛 NotImplementedError —— 任何子进程失败
仍以 subprocess 非零退出码冒泡, orchestrator 看到的 ok 决定后续动作。

编排模式: 默认带 --no-shoot —— 只做识别 + 存 json, 不 pause 不射击, 返回后
orchestrator 恢复巡线继续走 task2。射击由 task3_shoot waypoint (task_id 8)
读识别 json 负责。手动全流程 (识别→pause→射击) 直接跑 task3_pipeline 本体。
"""
from __future__ import annotations

import subprocess
import sys
from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient


def run(
    client: Optional[RuntimeApiClient] = None,  # noqa: ARG001
    *,
    extra_args: Optional[list[str]] = None,
) -> Dict[str, Any]:
    """薄封装: subprocess 跑 `python -m main.task.task3.task3_pipeline`。

    Args:
        client: 兼容 orchestrator 签名,本任务不直接用(任务内自带 RuntimeApiClient)
        extra_args: 透传给 task3_pipeline 的额外 CLI 参数,例如 ["--no-pause"]

    Returns:
        {"ok": bool, "status": "ok"|"failed", "exit_code": int, "args": [...]}
    """
    cmd = [sys.executable, "-m", "main.task.task3.task3_pipeline", "--no-shoot"]
    if extra_args:
        cmd.extend(extra_args)

    print(f"[task3] launching: {' '.join(cmd)}", flush=True)
    proc = subprocess.run(cmd, check=False)
    return {
        "ok": proc.returncode == 0,
        "status": "ok" if proc.returncode == 0 else "failed",
        "exit_code": proc.returncode,
        "args": cmd,
    }


if __name__ == "__main__":
    raise SystemExit(run().get("exit_code", 1))