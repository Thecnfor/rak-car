#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务三射击: 读识别 json, 射确认害虫 (thin wrapper over shoot_target).

task3_pest_scout (识别段) 以 --no-shoot --defer-judge 跑 task3_pipeline, 先存 raw
targets (status=pending) 立即返回; 后台线程判定完回写 status=done + pest_numbers
(1-based 板上编号)。本模块读该 json, 等 status=done (后台判定还没跑完则轮询等),
取 pest_numbers 射确认害虫; 无确认害虫直接返回。

容错语义: 读 json 失败 / 子进程非零退出码 → ok=False, orchestrator 记录并继续。
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient

# task3_pipeline --save 默认路径 (main/task/task3/audit/task3_pipeline.json)
DEFAULT_MANIFEST = (
    Path(__file__).resolve().parent / "task3" / "audit" / "task3_pipeline.json"
)

# 后台 LLM 判定最大等待: task2 在 task3_pest_scout 和 task3_shoot 之间跑, 正常已 done;
# 兜底等 max_wait_s (4 张 × 15s timeout 的最坏上界).
MAX_JUDGE_WAIT_S = 90.0

# 射击姿态 (y1 y2 x arm_angle hand_angle) —— 与 task3_pipeline.SHOOTING_ARM 一致.
# cam2 随臂动: 编排/单任务模式 task3_shoot 直接拉 shoot_target (它不动臂),
# 臂停在上一任务姿态 → cam2 看不到卡片. 必须在拉 subprocess 前摆好 (2026-08-09).
SHOOTING_ARM = ("-0.100", "-0.150", "-0.200", "90", "-90")


def _set_shooting_pose() -> int:
    """摆射击姿态, 复用 task3_pipeline.run_arm_pose 的 subprocess 调 arm_seq_v9 做法."""
    command = [
        sys.executable, "-m", "main.task.task3.arm_seq_v9",
        "--y1", SHOOTING_ARM[0], "--y2", SHOOTING_ARM[1], "--x", SHOOTING_ARM[2],
        "--arm-angle", SHOOTING_ARM[3], "--hand-angle", SHOOTING_ARM[4],
    ]
    print(f"[task3_shoot] shooting pose: {' '.join(command)}", flush=True)
    return subprocess.run(command, check=False).returncode


def _load_done_manifest(timeout: float = MAX_JUDGE_WAIT_S) -> Dict[str, Any]:
    """读识别 json, 直到 status=done (后台判定完成) 或超时返回当前内容.

    pending 期间逐秒轮询; 超时仍 pending → 返回内容 (pest_numbers 为空,
    调用方走"无确认害虫跳过"路径).
    """
    deadline = time.time() + timeout
    payload: Optional[Dict[str, Any]] = None
    while time.time() < deadline:
        try:
            payload = json.loads(DEFAULT_MANIFEST.read_text(encoding="utf-8"))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            print(f"[task3_shoot] 读识别 json 失败: {exc}", file=sys.stderr)
            payload = None
        if payload is not None and payload.get("status") == "done":
            return payload
        time.sleep(1.0)
    print(f"[task3_shoot] 后台判定 {timeout:.0f}s 超时, 用当前内容继续 "
          f"(status={payload.get('status') if payload else 'unreadable'})",
          file=sys.stderr)
    return payload or {}


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
    payload = _load_done_manifest()
    pests = [int(n) for n in payload.get("pest_numbers") or []]

    if not pests:
        print("[task3_shoot] 无确认害虫, 跳过射击", flush=True)
        return {"ok": True, "status": "ok", "exit_code": 0, "pest_numbers": []}

    # 2026-08-09: 编排/单任务模式补 SHOOTING_ARM 摆臂 —— 手动全流程由 task3_pipeline
    # 在射前摆臂, task3_shoot 直接拉 shoot_target 时臂停在上一任务姿态, cam2 看不到卡片.
    if _set_shooting_pose() != 0:
        print("[task3_shoot] 摆射击姿态失败, 放弃射击", file=sys.stderr)
        return {"ok": False, "status": "failed", "exit_code": 1, "pest_numbers": pests}

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
