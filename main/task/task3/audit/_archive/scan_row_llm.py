#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/tasks/task333/scan_row_llm.py

任务3 「扫描 + ERNIE 判害虫 + 居中记录」(v1)

== 流程 ==
1. 机械臂锁定不动(假设已复位)
2. 底盘面向图片向前直走 → move_for([+dx, 0, 0])
3. 每段后读 task_feed,挑最高分 animal 作为候选
4. 对候选裁 bbox → base64 → ERNIE 多模态判:
     result=0 (有害) → 若 |xc| ≤ tol_x 且 SCANNING → 记录 odom + arm
     result=1 (有益) → 不记,继续前进
     result=None (失败) → 不记,继续前进
5. 状态机同 scan_row.py:
   SCANNING  : 可继续记录
   RECORDED  : 已记一张,等目标离开中心区再解锁
6. 累计记满 4 张 或 前进达到 max_travel → stop

== ERNIE 来源 ==
优先级 --token > env ERNIE_ACCESS_TOKEN > config_car.yml:ernie_access_token

== 跑法 ==
    python -m main.tasks.task333.scan_row_llm
    python -m main.tasks.task333.scan_row_llm --token <bce-v3/ALTAK-...>
"""
from __future__ import annotations

import argparse
import base64
import json
import sys
import time
from pathlib import Path
from typing import Any, Optional

import requests

from main.api_client import RuntimeApiClient
from main.misc.test_pest_llm_shoot import (
    DEFAULT_CROP_PADDING,
    DEFAULT_MIN_SCORE,
    NO_ANIMAL_KEYWORDS,
    crop_bbox,
)

# ERNIE 多模态:用紧凑英文 prompt 强制 JSON 输出
PEST_PROMPT = """Analyze the image and classify the animal as pest or beneficial for farmland.

Output STRICT JSON only (no markdown, no commentary, no Chinese):
{"result": <0 or 1>, "analysis": "<one short sentence in English>"}

Rules:
- result=0 if the animal is a crop pest (locust, aphid, caterpillar, weevil, beetle, slug, snail, mite, etc.)
- result=1 if the animal is beneficial (bee, ladybug, butterfly pollinator, earthworm, spider that eats pests, etc.)
- analysis: one sentence describing the animal and why it is pest/beneficial
- If no animal visible, output {"result": 1, "analysis": "no animal detected"}
- Output ONLY the JSON object."""

# 切回百度 ERNIE(用户确认使用千帆 access token)
from main.tasks.task333.llm_ernie import (
    call_vision as _call_llm,
    check_health as _check_token_health,
    mask_token as _mask_token,
)
import os as _os


DEFAULT_DY = 0.04                # 每段前进米数
DEFAULT_MAX_TRAVEL = 4.0
DEFAULT_TARGET_COUNT = 4
DEFAULT_TOL_X = 0.10
DEFAULT_UNLOCK_X = 0.45
DEFAULT_LLM_TIMEOUT = 12.0
DEFAULT_JOB_TIMEOUT = 15.0
DEFAULT_AUDIT = "audit/row_scan_llm.json"


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


def pick_top_animal(detections: list[dict], min_score: float) -> Optional[dict]:
    best = None
    best_score = -1.0
    for d in detections:
        if d.get("label") != "animal":
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
        return {}


def read_odometry(client) -> list:
    try:
        return list((client.get_runtime() or {}).get("runtime", {}).get("odometry") or [0, 0, 0])
    except Exception:  # noqa: BLE001
        return [0.0, 0.0, 0.0]


def fetch_frame_bytes(streamer_url: str, timeout: float) -> Optional[bytes]:
    try:
        r = requests.get(f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout)
        r.raise_for_status()
        return r.content
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] fetch frame: {exc}", file=sys.stderr)
        return None


def det_to_list(d: dict) -> list:
    """把 task_feed dict 转成 crop_bbox 期望的 [cls, det, label, score, xc, yc, w, h]。"""
    b = d.get("bbox_norm") or {}
    return [
        d.get("cls_id"),
        d.get("det_id"),
        d.get("label", ""),
        d.get("score", 0.0),
        b.get("x_center", 0.0),
        b.get("y_center", 0.0),
        b.get("width", 0.0),
        b.get("height", 0.0),
    ]


def run() -> None:
    parser = argparse.ArgumentParser(description="Forward scan + ERNIE pest judge + center record")
    parser.add_argument("--token", type=str, default=None, help="ERNIE access token (or set ERNIE_ACCESS_TOKEN)")
    parser.add_argument("--dy", type=float, default=DEFAULT_DY, help=f"per-step forward meters (default {DEFAULT_DY})")
    parser.add_argument("--max-travel", type=float, default=DEFAULT_MAX_TRAVEL, dest="max_travel",
                        help=f"max total forward meters (default {DEFAULT_MAX_TRAVEL})")
    parser.add_argument("--target-count", type=int, default=DEFAULT_TARGET_COUNT, dest="target_count",
                        help=f"expected pest count (default {DEFAULT_TARGET_COUNT})")
    parser.add_argument("--tol-x", type=float, default=DEFAULT_TOL_X, dest="tol_x",
                        help=f"|xc| <= tol means horizontally centered (default {DEFAULT_TOL_X})")
    parser.add_argument("--unlock-x", type=float, default=DEFAULT_UNLOCK_X, dest="unlock_x",
                        help=f"|xc| > unlock means pest left center, allow next (default {DEFAULT_UNLOCK_X})")
    parser.add_argument("--min-score", type=float, default=DEFAULT_MIN_SCORE, dest="min_score",
                        help=f"min visual detection score (default {DEFAULT_MIN_SCORE})")
    parser.add_argument("--crop-padding", type=float, default=DEFAULT_CROP_PADDING, dest="crop_padding",
                        help=f"bbox padding ratio (default {DEFAULT_CROP_PADDING})")
    parser.add_argument("--llm-timeout", type=float, default=DEFAULT_LLM_TIMEOUT, dest="llm_timeout",
                        help=f"ERNIE POST timeout (default {DEFAULT_LLM_TIMEOUT})")
    parser.add_argument("--job-timeout", type=float, default=DEFAULT_JOB_TIMEOUT, dest="job_timeout",
                        help=f"chassis job timeout (default {DEFAULT_JOB_TIMEOUT})")
    parser.add_argument("--save", type=str, default=DEFAULT_AUDIT, help="output json path")
    parser.add_argument("--dry-run", action="store_true", help="read only, no chassis move")
    args = parser.parse_args()

    if not 0 < args.dy <= 0.2:
        parser.error("--dy must be in (0, 0.2]")
    if args.max_travel <= 0:
        parser.error("--max-travel must be > 0")
    if args.target_count < 1:
        parser.error("--target-count must be >= 1")
    if not 0 < args.tol_x < args.unlock_x < 1:
        parser.error("must have 0 < tol_x < unlock_x < 1")

    token = args.token or _os.getenv("ERNIE_ACCESS_TOKEN") or _os.getenv("MINIMAX_API_KEY")
    if not token:
        print("[fatal] no token: pass --token or set ERNIE_ACCESS_TOKEN env", file=sys.stderr)
        sys.exit(2)

    settings_mod = __import__("main.settings", fromlist=["load_settings"])
    settings = settings_mod.load_settings()
    streamer_url = settings.streamer_url

    print(f"API_BASE = {settings.api_base}")
    print(f"STREAMER = {streamer_url}")
    print(f"[ready] token={_mask_token(token)} dy={args.dy}m max_travel={args.max_travel}m "
          f"target_count={args.target_count} tol_x={args.tol_x} unlock_x={args.unlock_x} "
          f"min_score={args.min_score} crop_padding={args.crop_padding} dry_run={args.dry_run}")

    client = RuntimeApiClient()
    for _ in range(60):
        h = client.get_health()
        s = h.get("state", {})
        if s.get("initialized") and not s.get("initializing"):
            break
        time.sleep(0.5)

    # token 启动期 PONG 探活
    _check_token_health(token, timeout=args.llm_timeout)

    init_arm = read_arm_state(client)
    init_odom = read_odometry(client)
    print(f"[init] arm y_m={init_arm.get('y_m')} x_m={init_arm.get('x_m')} ref={init_arm.get('ref_encoder')}")
    print(f"[init] odom x={init_odom[0]:+.3f} y={init_odom[1]:+.3f} theta={init_odom[2]:+.3f}")

    found: list[dict[str, Any]] = []
    skipped_beneficial: list[dict[str, Any]] = []   # LLM 判有益(被跳过)
    skipped_llm_failed: list[dict[str, Any]] = []   # LLM 失败
    traveled = 0.0
    seg_idx = 0
    state = "SCANNING"
    session = requests.Session()

    try:
        while len(found) < args.target_count and traveled < args.max_travel - 1e-3:
            seg_idx += 1
            dets = read_detections(client)
            top = pick_top_animal(dets, args.min_score)
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
                f"[seg {seg_idx} state={state}] traveled={traveled:.3f}m  odom_x={odom[0]:+.3f}  "
                f"top={label + ' xc=' + xc_str + ' sc=' + f'{score:.2f}' if top else 'none'}"
            )

            # 居中判据 + LLM 判害虫
            if top is not None and abs(xc) <= args.tol_x and state == "SCANNING":
                # 裁 bbox
                frame_bytes = fetch_frame_bytes(streamer_url, timeout=5.0)
                if frame_bytes is None:
                    print("  [warn] no frame, skip this candidate")
                else:
                    det_list = det_to_list(top)
                    crop_bytes, rect = crop_bbox(frame_bytes, det_list, args.crop_padding)
                    if not crop_bytes:
                        print("  [warn] empty crop, skip")
                    else:
                        image_url = "data:image/jpeg;base64," + base64.b64encode(crop_bytes).decode("ascii")
                        verdict = _call_llm(token, image_url, PEST_PROMPT, args.llm_timeout)
                        analysis = (verdict.get("analysis") or "").lower()
                        is_no_animal = any(kw.lower() in analysis for kw in NO_ANIMAL_KEYWORDS)
                        res = verdict.get("result")

                        print(
                            f"  [judge] xc={xc:+.3f} score={score:.3f} -> "
                            f"result={res} analysis={verdict.get('analysis', '')[:120]}"
                        )

                        if is_no_animal:
                            print("  [skip] LLM: no animal")
                        elif res is None:
                            skipped_llm_failed.append({
                                "seg_idx": seg_idx, "xc": xc, "score": score,
                                "reason": verdict.get("analysis", ""),
                            })
                            print("  [skip] LLM unavailable")
                        elif res == 1:
                            skipped_beneficial.append({
                                "seg_idx": seg_idx, "xc": xc, "score": score,
                                "reason": verdict.get("analysis", ""),
                            })
                            print("  [skip] LLM: beneficial animal, keep moving")
                        elif res == 0:
                            arm = read_arm_state(client)
                            entry = {
                                "image_idx": len(found) + 1,
                                "seg_idx": seg_idx,
                                "timestamp": time.time(),
                                "label": label,
                                "visual_score": score,
                                "xc": round(xc, 4),
                                "yc": round(yc, 4),
                                "bbox_norm": bbox,
                                "llm_verdict": verdict,
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
                                f"  >>> [RECORD {len(found)}/{args.target_count}] PEST label={label} "
                                f"xc={xc:+.3f} odom_x={odom[0]:+.3f}m traveled={traveled:.3f}m"
                            )
            elif top is None or abs(xc) > args.unlock_x:
                if state == "RECORDED":
                    print("  [unlock] pest left center, re-arming")
                state = "SCANNING"

            if len(found) >= args.target_count:
                break
            remaining = args.max_travel - traveled
            step = min(args.dy, remaining)
            if step <= 0:
                break
            if not args.dry_run:
                try_car(client, "move_for", [float(step), 0.0, 0.0], timeout=args.job_timeout)
            traveled += step
            time.sleep(0.15)

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
                    "crop_padding": args.crop_padding,
                    "direction": "forward = car.move_for([+dx, 0, 0])",
                },
                "init_odom": init_odom,
                "init_arm": {
                    "y_m": init_arm.get("y_m"),
                    "x_m": init_arm.get("x_m"),
                    "ref_encoder": init_arm.get("ref_encoder"),
                },
                "found": found,
                "skipped_beneficial": skipped_beneficial,
                "skipped_llm_failed": skipped_llm_failed,
                "traveled_m": round(traveled, 4),
                "complete": len(found) >= args.target_count,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    print(f"\n========== LLM ROW SCAN RESULT ({len(found)}/{args.target_count}) ==========")
    for e in found:
        v = e.get("llm_verdict") or {}
        print(
            f"  image#{e['image_idx']} label={e['label']} visual_sc={e['visual_score']:.3f} "
            f"xc={e['xc']:+.3f} odom_x={e['odom'][0]:+.3f}m  "
            f"LLM_result={v.get('result')} analysis={v.get('analysis', '')[:80]}"
        )
    print(f"  skipped_beneficial = {len(skipped_beneficial)}")
    print(f"  skipped_llm_failed = {len(skipped_llm_failed)}")
    print(f"  traveled_total     = {traveled:.3f}m")
    print(f"  complete           = {len(found) >= args.target_count}")
    print(f"  saved to           = {out_path}")
    print("==================================================\n")


if __name__ == "__main__":
    run()