#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task1-7 无真机 dry-run：跑 fake runtime，输出 JSON 动作 trace（含关节姿态）。

用法（repo root）::

    python -m main.testing.dry_run --task all --trace-out trace.json
    python -m main.testing.dry_run --task 4 --deadline 40

特性：
- 复用 `TaskHarness`，所有物理动作路由到 FakeRobotSim 运动学仿真；
- 每个任务独立 `reset_fake_runtime()`，结果含 ok/status/reason/elapsed、
  car/arm/realtime 动作计数、末帧关节姿态、四轮是否归零；
- 附带"代表性物理命令包"样本（每个目标第一条 physical 事件的
  target/action/args/kwargs），可直接读出发给 MC602 的命令内容；
- 外部能力（ERNIE/OCR/cam2 帧）按预期 unsupported stub，不触网、不碰硬件。

输出 trace JSON 结构::

    {
      "tasks": {
        "1": {"task":1, "ok":true, "status":"done", "reason":null,
              "elapsed_s":9.6, "actions": {"car":{...},"arm":{...}},
              "posture": {"x_mm":..,"y_mm":..,"arm_angle":..,"hand_angle":..,
                          "grasped":.., "wheels":[..]},
              "sample_packets": [ {"target":"arm","action":"composite_run",
                                   "args":[],"kwargs":{...}} , ...]},
        ...
      }
    }
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from typing import Any, Dict, Optional

from main.task import TASK_RUNNERS
from main.testing.task_harness import TaskHarness

# 需要 stub 的任务及其补丁：key = task_id，value = (target, factory) 列表
#   target 是模块属性路径（harness.patch 的 key），factory 返回替换函数。
TASK_STUBS: Dict[int, list] = {
    3: [("subprocess.run", lambda _called: lambda cmd, check=False, **kw: _FakeProc(_called))],
    6: [("main.task.task6_get_order.order_read_run", lambda _called: (
        lambda: _called.update(n=_called["n"] + 1) or
        {"ok": False, "orders": [], "error": "dry-run: 无 ERNIE/OCR"}))],
    8: [
        ("subprocess.run", lambda _called: lambda cmd, check=False, **kw: _FakeProc(_called)),
        ("main.task.task3_shoot._load_done_manifest",
         lambda _called: lambda: {"pest_numbers": []}),
    ],
}


class _FakeProc:
    def __init__(self, _called):
        _called["launched"] += 1
        self.returncode = 0


def _sample_packets(recorder, per_action: int = 3) -> list:
    """每个动作类型取前 per_action 条 physical 事件（target/action/args/kwargs）。"""
    seen: Counter = Counter()
    packets: list = []
    for ev in recorder.events:
        if ev.phase != "physical":
            continue
        key = (ev.target, ev.action)
        if seen[key] >= per_action:
            continue
        seen[key] += 1
        packets.append({
            "target": ev.target, "action": ev.action,
            "args": [a for a in ev.args] if ev.args else [],
            "kwargs": dict(ev.kwargs) if ev.kwargs else {},
            "job_id": ev.job_id,
        })
    return packets


def run_one(harness: TaskHarness, task_id: int,
            deadline_s: Optional[float]) -> Dict[str, Any]:
    called: Dict[str, int] = {"n": 0, "launched": 0}
    for target, factory in TASK_STUBS.get(task_id, []):
        harness.patch(target, factory(called))
    res = harness.run(task_id, deadline_s=deadline_s)
    posture = dict(res.final_arm)
    posture["wheels"] = list(res.final_wheels)
    detail = res.result.get("detail") if isinstance(res.result, dict) else None
    reason = detail.get("reason") if isinstance(detail, dict) else None
    return {
        "task": task_id,
        "ok": res.ok,
        "done": res.done,
        "status": "timeout" if not res.done else ("ok" if res.ok else "fail"),
        "reason": reason,
        "error": res.error,
        "elapsed_s": round(res.elapsed_s, 2),
        "actions": {k: dict(v) for k, v in res.actions.items()},
        "posture": posture,
        "sample_packets": _sample_packets(harness.service.recorder),
    }


def main(argv: Optional[list] = None) -> int:
    p = argparse.ArgumentParser(
        prog="main.testing.dry_run",
        description="task1-7 无真机 dry-run：fake runtime 动作 trace（含关节姿态）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--task", default="all",
                   help="任务编号 (1-8) 或 all")
    p.add_argument("--trace-out", default=None,
                   help="JSON 输出路径（缺省只打印摘要）")
    p.add_argument("--deadline", type=float, default=None,
                   help="每任务墙钟期限（s），缺省用内置默认")
    args = p.parse_args(argv)

    task_ids = ([int(args.task)] if args.task != "all"
                else [1, 2, 3, 4, 5, 6, 7, 8])

    harness = TaskHarness()
    results: Dict[str, Any] = {}
    try:
        harness.setUp()
        for tid in task_ids:
            if tid not in TASK_RUNNERS:
                print(f"[dry-run] 未知任务 {tid}", file=sys.stderr)
                continue
            print(f"[dry-run] 运行 task{tid} ...", flush=True)
            results[str(tid)] = run_one(harness, tid, args.deadline)
            print(f"  → {results[str(tid)]['status']} "
                  f"({results[str(tid)]['elapsed_s']}s) "
                  f"{dict(results[str(tid)]['actions'].get('car', {}))} "
                  f"{dict(results[str(tid)]['actions'].get('arm', {}))}")
    finally:
        harness.tearDown()

    trace = {
        "transport": "fake",
        "simulation": "FakeRobotSim 运动学仿真 (forward_kinematics)",
        "tasks": results,
        "summary": {k: v["status"] for k, v in results.items()},
    }
    if args.trace_out:
        with open(args.trace_out, "w", encoding="utf-8") as f:
            json.dump(trace, f, ensure_ascii=False, indent=2)
        print(f"[dry-run] trace 已写入 {args.trace_out}")
    else:
        print(json.dumps(trace, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
