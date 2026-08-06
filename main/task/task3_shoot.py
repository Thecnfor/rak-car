#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务三射击: 读识别 json, 射确认害虫 (thin wrapper over shoot_target).

task3_pest_scout (识别段) 以 --no-shoot 跑 task3_pipeline, 把识别结果存到
audit/task3_pipeline.json (含 pest_numbers, 1-based 板上编号)。本模块读该
json, 取 pest_numbers 射确认害虫; 无确认害虫直接返回。

容错语义: 读 json 失败 / 子进程非零退出码 → ok=False, orchestrator 记录并继续。
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient

# task3_pipeline --save 默认路径 (main/task/task3/audit/task3_pipeline.json)
DEFAULT_MANIFEST = (
    Path(__file__).resolve().parent / "task3" / "audit" / "task3_pipeline.json"
)


def run(
    client: Optional[RuntimeApiClient] = None,  # noqa: ARG001
    *,
    extra_args: Optional[list[str]] = None,
) -> Dict[str, Any]:
    """薄封装: 读识别 json → subprocess 跑 shoot_target 射确认害虫.

    Args:
        client: 兼容 orchestrator 签名, 本任务不直接用 (shoot_target 自带).
        extra_args: 透传给 shoot_target 的额外 CLI 参数.

    Returns:
        {"ok": bool, "status": "ok"|"failed", "exit_code": int, "pest_numbers": [...]}
    """
    try:
        payload = json.loads(DEFAULT_MANIFEST.read_text(encoding="utf-8"))
        pests = [int(n) for n in payload.get("pest_numbers") or []]
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"[task3_shoot] 读识别 json 失败: {exc}", file=sys.stderr)
        return {"ok": False, "status": "failed", "error": str(exc)}

    if not pests:
        print("[task3_shoot] 无确认害虫, 跳过射击", flush=True)
        return {"ok": True, "status": "ok", "exit_code": 0, "pest_numbers": []}

    cmd = [
        sys.executable, "-m", "main.task.task3.shoot_target",
        "--targets", " ".join(str(n) for n in pests),
        "--identity-file", str(DEFAULT_MANIFEST),
    ]
    if extra_args:
        cmd.extend(extra_args)

    print(f"[task3_shoot] launching: {' '.join(cmd)}", flush=True)
    proc = subprocess.run(cmd, check=False)
    return {
        "ok": proc.returncode == 0,
        "status": "ok" if proc.returncode == 0 else "failed",
        "exit_code": proc.returncode,
        "pest_numbers": pests,
    }


if __name__ == "__main__":
    raise SystemExit(run().get("exit_code", 1))
