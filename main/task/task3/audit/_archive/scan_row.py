#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/tasks/task333/scan_row.py

任务3 「一行 4 张图水平居中扫描」(v2 — 修正右移方向 + 状态机)

== 流程 ==
1. 机械臂全程锁定不动(假设已复位)
2. 底盘面向图片向右平移 → move_for([0, -dy, 0])
3. 每段后读 task_feed,挑最高分 animal
4. 状态机:
   SCANNING  : 当前帧没有已记录的图在居中区,可以继续右移
   RECORDED  : 刚记完一张图,等它从视野消失(防重复记录)
5. 当 |xc| ≤ tol_x 且当前为 SCANNING → 记一次底盘 odom + arm 状态
6. 累计记满 4 张 或 右移达到 max_travel → stop

== 跑法 ==
    python -m main.tasks.task333.scan_row
    python -m main.tasks.task333.scan_row --dy 0.03 --max-travel 4.0
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Optional

from main.api_client import RuntimeApiClient


DEFAULT_DY = 0.04                # 每段右移米数(右移 = -y)
DEFAULT_MAX_TRAVEL = 4.0         # 累计右移上限米
DEFAULT_TARGET_COUNT = 4         # 期望图数
DEFAULT_TOL_X = 0.10             # |xc| ≤ 此值视为水平居中
DEFAULT_UNLOCK_X = 0.45          # |xc| > 此值认为当前图已离开中心,允许记下一张
DEFAULT_MIN_SCORE = 0.50
DEFAULT_AUDIT = "main/tasks/task333/audit/row_scan_v2.json"
DEFAULT_JOB_TIMEOUT = 15.0


def wait_car(client, name, *a, timeout=None, **k):
    job = client.execute_car_action(name, *a, timeout=timeout, sync=False, **k)
    done = client.wait_job(job["id"], timeout=(timeout or 60.0) + 10.0)
    if done.get("status") != "succeeded":
        raise RuntimeError(
            f"car.{name} failed: status={done.get('status')} error={done.get('error')}"
        )
    return done.get("result")


def try_car(client, name, *a, timeout=None, **k):
    try:
        return wait_car(client, name, *a, timeout=timeout, **k)
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] car.{name} failed: {exc}", file=sys.stderr)
        return None


def read_detections(client) -> list[dict]:
    try:
        resp = client.get_task_state()
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] get_task_state: {exc}", file=sys.stderr)
        return []
    ts = (resp or {}).get("task_state") or {}
    return list(ts.get("detections") or [])


def pick_top(detections: list[dict], label: str = "animal", min_score: float = 0.50) -> Optional[dict]:
    best = None
    best_score = -1.0
    for d in detections:
        if d.get("label") != label:
            continue
        sc = float(d.get("score") or 0.0)
        if sc < min_score:
            continue
        if sc > best_score:
            best_score = sc
            best = d
    return best


def read_arm_state(client) -> dict:
    try:
        return (client.get_arm_state() or {}).get("arm_state") or {}
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] get_arm_state: {exc}", file=sys.stderr)
        return {}


def read_odometry(client) -> list:
    try:
        return list((client.get_runtime() or {}).get("runtime", {}).get("odometry") or [0, 0, 0])
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] get_runtime: {exc}", file=sys.stderr)
        return [0.0, 0.0, 0.0]


def run() -> None:
    parser = argparse.ArgumentParser(description="Rightward row scan: 4 images at xc=0")
    parser.add_argument("--dy", type=float, default=DEFAULT_DY, help=f"per-step rightward meters (default {DEFAULT_DY})")
    parser.add_argument("--max-travel", type=float, default=DEFAULT_MAX_TRAVEL, dest="max_travel",
                        help=f"max total rightward meters (default {DEFAULT_MAX_TRAVEL})")
    parser.add_argument("--target-count", type=int, default=DEFAULT_TARGET_COUNT, dest="target_count",
                        help=f"expected image count (default {DEFAULT_TARGET_COUNT})")
    parser.add_argument("--tol-x", type=float, default=DEFAULT_TOL_X, dest="tol_x",
                        help=f"|xc| <= tol means horizontally centered (default {DEFAULT_TOL_X})")
    parser.add_argument("--unlock-x", type=float, default=DEFAULT_UNLOCK_X, dest="unlock_x",
                        help=f"|xc| > unlock means image left center, allow next (default {DEFAULT_UNLOCK_X})")
    parser.add_argument("--min-score", type=float, default=DEFAULT_MIN_SCORE, dest="min_score",
                        help=f"min detection score (default {DEFAULT_MIN_SCORE})")
    parser.add_argument("--save", type=str, default=DEFAULT_AUDIT, help="output json path")
    parser.add_argument("--job-timeout", type=float, default=DEFAULT_JOB_TIMEOUT, dest="job_timeout",
                        help=f"chassis job timeout sec (default {DEFAULT_JOB_TIMEOUT})")
    parser.add_argument("--dry-run", action="store_true", help="read only, no chassis moves")
    args = parser.parse_args()

    if not 0 < args.dy <= 0.2:
        parser.error("--dy must be in (0, 0.2]")
    if args.max_travel <= 0:
        parser.error("--max-travel must be > 0")
    if args.target_count < 1:
        parser.error("--target-count must be >= 1")
    if not 0 < args.tol_x < args.unlock_x < 1:
        parser.error("must have 0 < tol_x < unlock_x < 1")

    client = RuntimeApiClient()
    for _ in range(60):
        h = client.get_health()
        s = h.get("state", {})
        if s.get("initialized") and not s.get("initializing"):
            break
        time.sleep(0.5)

    init_arm = read_arm_state(client)
    init_odom = read_odometry(client)
    print(
        f"[init] arm y_m={init_arm.get('y_m')} x_m={init_arm.get('x_m')} ref={init_arm.get('ref_encoder')}"
    )
    print(
        f"[init] odom x={init_odom[0]:+.3f} y={init_odom[1]:+.3f} theta={init_odom[2]:+.3f}"
    )
    print(
        f"[ready] dy={args.dy}m max_travel={args.max_travel}m target_count={args.target_count} "
        f"tol_x={args.tol_x} unlock_x={args.unlock_x} min_score={args.min_score} dry_run={args.dry_run}"
    )

    found: list[dict[str, Any]] = []
    traveled = 0.0
    seg_idx = 0
    state = "SCANNING"   # SCANNING | RECORDED

    try:
        while len(found) < args.target_count and traveled < args.max_travel - 1e-3:
            seg_idx += 1
            dets = read_detections(client)
            top = pick_top(dets, min_score=args.min_score)
            xc = None
            label = None
            score = None
            yc = None
            bbox = None
            if top is not None:
                bbox = top.get("bbox_norm") or {}
                xc = float(bbox.get("x_center", 0.0))
                yc = float(bbox.get("y_center", 0.0))
                label = top.get("label")
                score = float(top.get("score") or 0.0)

            odom = read_odometry(client)
            xc_str = f"{xc:+.3f}" if xc is not None else "none"
            print(
                f"[seg {seg_idx} state={state}] traveled={traveled:.3f}m  odom_y={odom[1]:+.3f}  "
                f"top={label + ' xc=' + xc_str if top else 'none'}"
            )

            # 状态机:记录逻辑
            if top is not None and abs(xc) <= args.tol_x and state == "SCANNING":
                arm = read_arm_state(client)
                entry = {
                    "image_idx": len(found) + 1,
                    "seg_idx": seg_idx,
                    "timestamp": time.time(),
                    "label": label,
                    "score": score,
                    "xc": round(xc, 4),
                    "yc": round(yc, 4),
                    "bbox_norm": bbox,
                    "odom": [round(v, 4) for v in odom],
                    "arm_state": {
                        "y_m": arm.get("y_m"),
                        "x_m": arm.get("x_m"),
                        "y_mm": arm.get("y_mm"),
                        "x_mm": arm.get("x_mm"),
                        "ref_encoder": arm.get("ref_encoder"),
                        "active": arm.get("active"),
                    },
                    "traveled_m": round(traveled, 4),
                }
                found.append(entry)
                state = "RECORDED"
                print(
                    f"  >>> [RECORD {len(found)}/{args.target_count}] label={label} "
                    f"xc={xc:+.3f} odom_y={odom[1]:+.3f}m traveled={traveled:.3f}m"
                )
            elif top is None or abs(xc) > args.unlock_x:
                # 当前图已离开居中区(或完全离开视野) → 解锁
                if state == "RECORDED":
                    print(f"  [unlock] image left center, re-arming")
                state = "SCANNING"

            # 前进一段 (+x 方向)
            if len(found) >= args.target_count:
                break
            remaining = args.max_travel - traveled
            step = min(args.dy, remaining)
            if step <= 0:
                break
            if not args.dry_run:
                try_car(client, "move_for", [float(step), 0.0, 0.0], timeout=args.job_timeout)
            traveled += step
            time.sleep(0.12)

    except KeyboardInterrupt:
        print("\n[abort] KeyboardInterrupt")

    try_car(client, "stop", timeout=args.job_timeout)

    out_path = Path(args.save)
    if not out_path.is_absolute():
        out_path = Path(__file__).resolve().parent / args.save
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(
        json.dumps(
            {
                "config": {
                    "dy": args.dy,
                    "max_travel": args.max_travel,
                    "target_count": args.target_count,
                    "tol_x": args.tol_x,
                    "unlock_x": args.unlock_x,
                    "min_score": args.min_score,
                    "direction": "forward = car.move_for([+dx, 0, 0])",
                },
                "init_odom": init_odom,
                "init_arm": {
                    "y_m": init_arm.get("y_m"),
                    "x_m": init_arm.get("x_m"),
                    "ref_encoder": init_arm.get("ref_encoder"),
                },
                "found": found,
                "traveled_m": round(traveled, 4),
                "complete": len(found) >= args.target_count,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    print(f"\n========== ROW SCAN RESULT ({len(found)}/{args.target_count}) ==========")
    for e in found:
        print(
            f"  image#{e['image_idx']} label={e['label']} score={e['score']:.3f} "
            f"xc={e['xc']:+.3f} odom_y={e['odom'][1]:+.3f}m  traveled={e['traveled_m']:.3f}m"
        )
    print(f"  traveled_total = {traveled:.3f}m")
    print(f"  complete       = {len(found) >= args.target_count}")
    print(f"  saved to       = {out_path}")
    print("==================================================\n")


if __name__ == "__main__":
    run()