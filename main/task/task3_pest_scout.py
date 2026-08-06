#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务三: 害虫侦察 + 害虫射击 (thin wrapper over task3_pipeline).

业务方案:在 main/task/task3/task3_pipeline.py 实现的 drive + LLM 判别 +
shoot_target 三段式流水线。本文件只是 orchestrator registry 期望的薄封装
(run(client=None) -> Dict),实际跑 subprocess 调 task3_pipeline.main()。

容错语义: 与占位期一致, 但不再抛 NotImplementedError —— 任何子进程失败
仍以 subprocess 非零退出码冒泡, orchestrator 看到的 ok 决定后续动作。

编排模式 (2026-08-07): 默认带 --no-shoot --defer-judge —— 识别段只存 raw
targets (status=pending) 立即返回, **LLM 判定放到本进程后台线程**, 不阻塞车;
orchestrator 恢复巡线继续走 task2, 后台线程判定完回写 status=done + pest_numbers。
射击由 task3_shoot waypoint (task_id 8) 读识别 json 负责。手动全流程
(识别→pause→射击) 直接跑 task3_pipeline 本体。
"""
from __future__ import annotations

import json
import subprocess
import sys
import threading
from pathlib import Path
from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient

# task3_pipeline --save 默认路径 (main/task/task3/audit/task3_pipeline.json)
DEFAULT_MANIFEST = (
    Path(__file__).resolve().parent / "task3" / "audit" / "task3_pipeline.json"
)

DEFAULT_LLM_TIMEOUT = 15.0


def _judge_background(manifest: Path, llm_timeout: float = DEFAULT_LLM_TIMEOUT) -> None:
    """后台线程: 读 pending json → 逐张 LLM 判定 → 回写 done json。

    在 orchestrator 进程跑 (task3_pest_scout.run 的调用方), 车已恢复巡线不受影响。
    失败只打日志, 不抛 —— 超时未 done 时 task3_shoot 读到空 pest_numbers 跳过射击。
    """
    try:
        payload = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"[task3-judge] 读 pending json 失败: {exc}", file=sys.stderr)
        return
    if payload.get("status") == "done":
        return
    from main.task.task3.llm_ernie import load_token
    from main.task.task3.task3_pipeline import judge_records

    try:
        token = load_token()
    except SystemExit as exc:
        print(f"[task3-judge] 无 ERNIE token, 判定失败: {exc}", file=sys.stderr)
        return
    print(f"[task3-judge] 后台判定 {len(payload.get('targets') or [])} 个目标"
          f"(llm_timeout={llm_timeout:.1f}s, 不阻塞车)", flush=True)
    judged = judge_records(token, payload.get("targets") or [], llm_timeout)
    payload["targets"] = judged
    payload["status"] = "done"
    payload["pest_numbers"] = [t["number"] for t in judged if t.get("result") == 0]
    payload["beneficial_numbers"] = [t["number"] for t in judged if t.get("result") == 1]
    manifest.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[task3-judge] done: pests={payload['pest_numbers'] or 'none'} "
          f"beneficial={payload['beneficial_numbers'] or 'none'}", flush=True)


def run(
    client: Optional[RuntimeApiClient] = None,  # noqa: ARG001
    *,
    extra_args: Optional[list[str]] = None,
) -> Dict[str, Any]:
    """薄封装: subprocess 跑 `python -m main.task.task3.task3_pipeline --defer-judge`。

    Args:
        client: 兼容 orchestrator 签名,本任务不直接用(任务内自带 RuntimeApiClient)
        extra_args: 透传给 task3_pipeline 的额外 CLI 参数,例如 ["--no-pause"]

    Returns:
        {"ok": bool, "status": "ok"|"failed", "exit_code": int, "args": [...]}
    """
    cmd = [sys.executable, "-m", "main.task.task3.task3_pipeline",
           "--no-shoot", "--defer-judge"]
    if extra_args:
        cmd.extend(extra_args)

    print(f"[task3] launching: {' '.join(cmd)}", flush=True)
    proc = subprocess.run(cmd, check=False)
    # 识别段返回即起后台判定线程 —— 车继续巡线走 task2, 判定不阻塞.
    if proc.returncode == 0:
        threading.Thread(target=_judge_background, args=(DEFAULT_MANIFEST,),
                         daemon=True, name="task3-judge").start()
    return {
        "ok": proc.returncode == 0,
        "status": "ok" if proc.returncode == 0 else "failed",
        "exit_code": proc.returncode,
        "args": cmd,
    }


if __name__ == "__main__":
    raise SystemExit(run().get("exit_code", 1))