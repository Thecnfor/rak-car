#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务三: 害虫侦察 + 害虫射击 (thin wrapper over task3_pipeline).

业务方案:在 main/task/task3/task3_pipeline.py 实现的 drive + LLM 判别 +
shoot_target 三段式流水线。本文件只是 orchestrator registry 期望的薄封装
(run(client=None) -> Dict),实际跑 subprocess 调 task3_pipeline.main()。

容错语义: 与占位期一致, 但不再抛 NotImplementedError —— 任何子进程失败
仍以 subprocess 非零退出码冒泡, orchestrator 看到的 status 决定后续动作。

注意: task3_pipeline 自身会 await 用户按 Enter 切射击姿态, 第一次跑会
停在 "[pause] 请将车辆移动到射击区..."。用 --no-pause 可以跳过人工确认
(实机 8 球压测时再开)。
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
        {"status": "ok"|"failed", "exit_code": int, "args": [...]}
    """
    cmd = [sys.executable, "-m", "main.task.task3.task3_pipeline"]
    if extra_args:
        cmd.extend(extra_args)

    print(f"[task3] launching: {' '.join(cmd)}", flush=True)
    proc = subprocess.run(cmd, check=False)
    return {
        "status": "ok" if proc.returncode == 0 else "failed",
        "exit_code": proc.returncode,
        "args": cmd,
    }


if __name__ == "__main__":
    raise SystemExit(run().get("exit_code", 1))