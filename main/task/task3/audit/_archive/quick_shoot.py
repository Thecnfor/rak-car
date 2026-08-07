#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/quick_shoot.py - quick pest detection + shoot (0.5s budget)

Strategy:
  1. Poll task_feed ONCE (~30ms)
  2. (optional) LLM judge via ERNIE VL (~200-400ms)
  3. If pest detected, fire one shot

Total budget: 0.5s

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.quick_shoot
    python -m main.tasks.task333.quick_shoot --use-llm
    python -m main.tasks.task333.quick_shoot --min-score 0.7 --budget 0.5
"""
from __future__ import annotations

import argparse
import base64
import os
import sys
import time

import requests

from main.api_client import RuntimeApiClient
from main.misc.test_pest_llm_shoot import crop_bbox

# Optional ERNIE adapter
try:
    from main.tasks.task333.llm_ernie import call_vision, mask_token
    HAS_ERNIE = True
except Exception:
    HAS_ERNIE = False


PEST_PROMPT = """Classify the animal as pest or beneficial for farmland.
Output STRICT JSON only:
{"result": <0 or 1>, "analysis": "<one short sentence>"}
Rules:
- result=0: crop pest (locust, aphid, caterpillar, weevil, beetle, slug, snail, mite)
- result=1: beneficial (bee, ladybug, butterfly pollinator, earthworm)
- If no animal visible: {"result": 1, "analysis": "no animal"}
- Output ONLY the JSON object."""


def car_call(client, name, *args, timeout=15.0, **kw):
    job = client.execute_car_action(name, *args, timeout=timeout, sync=False, **kw)
    done = client.wait_job(job["id"], timeout=timeout + 10)
    if done.get("status") != "succeeded":
        raise RuntimeError(f"car.{name} failed: {done.get('error')}")
    return done.get("result")


def get_top_animal(client, min_score):
    try:
        ts = (client.get_task_state() or {}).get("task_state") or {}
        dets = list(ts.get("detections") or [])
    except Exception:
        return None
    best = None
    best_score = -1.0
    for d in dets:
        if d.get("label") != "animal":
            continue
        sc = float(d.get("score") or 0.0)
        if sc < min_score:
            continue
        if sc > best_score:
            best_score = sc
            best = d
    return best


def det_to_list(d):
    b = d.get("bbox_norm") or {}
    return [
        d.get("cls_id"), d.get("det_id"), d.get("label", ""),
        d.get("score", 0.0),
        b.get("x_center", 0.0), b.get("y_center", 0.0),
        b.get("width", 0.0), b.get("height", 0.0),
    ]


def main():
    ap = argparse.ArgumentParser(description="quick pest detection + shoot (0.5s budget)")
    ap.add_argument("--min-score", type=float, default=0.50, help="visual score threshold")
    ap.add_argument("--budget", type=float, default=0.5, help="total time budget (sec)")
    ap.add_argument("--use-llm", action="store_true",
                    help="enable ERNIE VL judge (slower but precise)")
    ap.add_argument("--token", default=None, help="ERNIE access token (or env ERNIE_ACCESS_TOKEN)")
    ap.add_argument("--crop-padding", type=float, default=0.10)
    ap.add_argument("--llm-timeout", type=float, default=0.35,
                    help="LLM call timeout (must be < budget)")
    ap.add_argument("--streamer", default=None,
                    help="streamer url (default: settings.streamer_url)")
    args = ap.parse_args()

    if args.budget < 0.1:
        ap.error("--budget must be >= 0.1")
    if args.use_llm and not HAS_ERNIE:
        print("[fatal] --use-llm requested but llm_ernie not importable", file=sys.stderr)
        sys.exit(2)
    if args.use_llm and args.llm_timeout >= args.budget:
        ap.error(f"--llm-timeout ({args.llm_timeout}) must be < --budget ({args.budget})")

    client = RuntimeApiClient()
    client.wait_until_ready()

    settings_mod = __import__("main.settings", fromlist=["load_settings"])
    streamer_url = args.streamer or settings_mod.load_settings().streamer_url

    token = args.token or os.getenv("ERNIE_ACCESS_TOKEN") or os.getenv("MINIMAX_API_KEY")

    t_total = time.time()
    print(f"[quick] budget={args.budget}s use_llm={args.use_llm} min_score={args.min_score}")

    # 1) 视觉检测(单次,~30ms)
    top = get_top_animal(client, args.min_score)
    if top is None:
        dt = time.time() - t_total
        print(f"[quick] no detection (took {dt*1000:.0f}ms) - no shot fired")
        return
    score = float(top.get("score") or 0.0)
    b = top.get("bbox_norm") or {}
    xc = float(b.get("x_center", 0.0))
    print(f"[quick] visual: score={score:.3f} xc={xc:+.3f}")

    # 2) (可选)LLM 判
    is_pest = True  # 默认:有动物就当害虫
    analysis = "visual-only (no LLM)"
    if args.use_llm:
        if not token:
            print("[fatal] --use-llm requires --token or ERNIE_ACCESS_TOKEN env", file=sys.stderr)
            sys.exit(2)
        # 抓帧 + 裁 + 调 LLM
        try:
            r = requests.get(f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=0.2)
            frame = r.content
        except Exception as e:
            dt = time.time() - t_total
            print(f"[quick] frame fetch failed: {e} (took {dt*1000:.0f}ms) - skip LLM, treat as pest")
            frame = None

        if frame:
            crop, _ = crop_bbox(frame, det_to_list(top), args.crop_padding)
            if crop:
                url = "data:image/jpeg;base64," + base64.b64encode(crop).decode()
                t0 = time.time()
                verdict = call_vision(token, url, PEST_PROMPT, timeout=args.llm_timeout)
                llm_dt = time.time() - t0
                is_pest = verdict.get("result") == 0
                analysis = verdict.get("analysis", "")
                print(f"[quick] LLM: result={verdict.get('result')} took={llm_dt*1000:.0f}ms analysis={analysis[:100]}")
            else:
                print("[quick] empty crop, treat as pest")
        else:
            is_pest = True

    # 3) 决定射击 + 检查预算
    elapsed = time.time() - t_total
    if elapsed >= args.budget:
        print(f"[quick] budget exceeded ({elapsed*1000:.0f}ms >= {args.budget*1000:.0f}ms) - no shot")
        return

    if not is_pest:
        print(f"[quick] beneficial animal ({analysis[:80]}) - no shot fired")
        return

    # 4) 射击
    try:
        t0 = time.time()
        car_call(client, "shooting", timeout=8)
        shot_dt = time.time() - t0
        total_dt = time.time() - t_total
        print(f"[quick] FIRED shot took={shot_dt*1000:.0f}ms total={total_dt*1000:.0f}ms")
        print(f"[quick] reason: {analysis}")
    except Exception as e:
        print(f"[quick] shot failed: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()