#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""手动补判定: 读 pending 识别 json → 逐张 LLM 判定 → 回写 done.

用途 (2026-08-09): 单任务分开测 task3 识别段/射击段.
  `python run.py --task 3` 识别段跑完后, orchestrator 进程退出会杀掉
  后台判定线程 (_judge_background 是 daemon), json 停在 status=pending.
  跑本脚本把判定补完, json 变 done + pest_numbers, 之后才能单独测射击段.

用法:
  python -m main.task.task3.judge_manifest [--json PATH] [--timeout 15]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

DEFAULT_MANIFEST = Path(__file__).resolve().parent / "audit" / "task3_pipeline.json"
DEFAULT_LLM_TIMEOUT = 15.0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="手动补 task3 LLM 判定")
    ap.add_argument("--json", dest="path", default=str(DEFAULT_MANIFEST),
                    help="识别 json 路径 (默认 audit/task3_pipeline.json)")
    ap.add_argument("--timeout", type=float, default=DEFAULT_LLM_TIMEOUT,
                    help="单张 LLM 判定超时秒数 (默认 15)")
    args = ap.parse_args(argv)

    manifest = Path(args.path)
    try:
        payload = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        print(f"[judge] 读识别 json 失败: {exc}", file=sys.stderr)
        return 1

    if payload.get("status") == "done":
        print(f"[judge] {manifest} 已是 done (pests="
              f"{payload.get('pest_numbers') or 'none'}), 无需判定")
        return 0

    from main.task.task3.llm_ernie import load_token
    from main.task.task3.task3_pipeline import judge_records

    token = load_token()
    targets = payload.get("targets") or []
    print(f"[judge] 判定 {len(targets)} 个目标 "
          f"(llm_timeout={args.timeout:.1f}s)...", flush=True)
    payload["targets"] = judge_records(token, targets, args.timeout)
    payload["status"] = "done"
    payload["pest_numbers"] = [t["number"] for t in payload["targets"]
                               if t.get("result") == 0]
    payload["beneficial_numbers"] = [t["number"] for t in payload["targets"]
                                     if t.get("result") == 1]
    manifest.write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                        encoding="utf-8")
    print(f"[judge] done: pests={payload['pest_numbers'] or 'none'} "
          f"beneficial={payload['beneficial_numbers'] or 'none'}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
