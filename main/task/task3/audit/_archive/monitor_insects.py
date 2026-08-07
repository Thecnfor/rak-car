#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/monitor_insects.py - real-time pest/beneficial monitor

Continuous loop:
  every 0.1s read task_feed
  if animal detected -> grab frame, crop bbox, call ERNIE VL to judge
  print: [tick] ts=... visual=score xc=... llm=result reason=...

Note:
  - visual polling: 10 Hz (every 0.1s)
  - LLM call: only fires when an animal is detected AND its score > previous
    + verdict changes (avoid spamming same answer at 10Hz)
  - LLM call itself takes ~300-400ms, much slower than 0.1s polling budget,
    so detection reads continue independently

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.monitor_insects
    python -m main.tasks.task333.monitor_insects --interval 0.1 --min-score 0.5
    python -m main.tasks.task333.monitor_insects --no-shoot   # just monitor
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

try:
    from main.tasks.task333.llm_ernie import call_vision, mask_token
    HAS_ERNIE = True
except Exception:
    HAS_ERNIE = False


PEST_PROMPT = """Classify the animal as pest or beneficial for farmland.
Output STRICT JSON only:
{"result": <0 or 1>, "analysis": "<one short sentence in English>"}
Rules:
- result=0: crop pest (locust, aphid, caterpillar, weevil, beetle, slug, snail, mite)
- result=1: beneficial (bee, ladybug, butterfly pollinator, earthworm)
- If no animal visible: {"result": 1, "analysis": "no animal"}
- Output ONLY the JSON object."""


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


def fetch_frame(streamer_url, timeout=0.3):
    try:
        r = requests.get(f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout)
        r.raise_for_status()
        return r.content
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser(description="real-time pest/beneficial monitor at 10Hz")
    ap.add_argument("--interval", type=float, default=0.1, help="visual poll interval (sec)")
    ap.add_argument("--min-score", type=float, default=0.50, dest="min_score",
                    help="visual score threshold")
    ap.add_argument("--use-llm", action="store_true", help="call ERNIE VL on detection")
    ap.add_argument("--token", default=None, help="ERNIE access token (or env ERNIE_ACCESS_TOKEN)")
    ap.add_argument("--llm-timeout", type=float, default=0.4, dest="llm_timeout",
                    help="LLM call timeout (sec)")
    ap.add_argument("--crop-padding", type=float, default=0.10, dest="crop_padding")
    ap.add_argument("--cooldown", type=float, default=2.0,
                    help="seconds between LLM calls for the same target position")
    ap.add_argument("--streamer", default=None)
    args = ap.parse_args()

    if args.use_llm and not HAS_ERNIE:
        print("[fatal] --use-llm requires llm_ernie", file=sys.stderr)
        sys.exit(2)
    if args.use_llm:
        token = args.token or os.getenv("ERNIE_ACCESS_TOKEN") or os.getenv("MINIMAX_API_KEY")
        if not token:
            print("[fatal] --use-llm requires --token or ERNIE_ACCESS_TOKEN env", file=sys.stderr)
            sys.exit(2)
    else:
        token = None

    settings_mod = __import__("main.settings", fromlist=["load_settings"])
    streamer_url = args.streamer or settings_mod.load_settings().streamer_url

    client = RuntimeApiClient()
    client.wait_until_ready()

    print(f"[monitor] interval={args.interval}s use_llm={args.use_llm} "
          f"min_score={args.min_score} cooldown={args.cooldown}s", flush=True)
    print(f"[monitor] streamer={streamer_url}", flush=True)
    print(f"[monitor] press Ctrl+C to stop\n", flush=True)

    last_llm_call_ts = 0.0
    last_llm_key = None
    tick = 0

    try:
        while True:
            t0 = time.time()
            tick += 1

            top = get_top_animal(client, args.min_score)
            ts_now = time.time()

            if top is None:
                if tick % 10 == 0:   # 每 1s 打印一次"无目标",避免刷屏
                    print(f"[tick {tick:>5}] ts={ts_now:.3f}  visual=none",
                          flush=True)
            else:
                score = float(top.get("score") or 0.0)
                b = top.get("bbox_norm") or {}
                xc = float(b.get("x_center", 0.0))
                yc = float(b.get("y_center", 0.0))

                llm_line = "llm=skipped"
                # 是否要调 LLM?--use-llm 启用 + 距离上次 LLM 足够久 + 目标位置变了
                should_call_llm = False
                if args.use_llm:
                    pos_key = (round(xc, 2), round(yc, 2), round(score, 2))
                    if last_llm_key != pos_key and (ts_now - last_llm_call_ts) >= args.cooldown:
                        should_call_llm = True

                if should_call_llm:
                    frame = fetch_frame(streamer_url, timeout=0.3)
                    if frame:
                        crop, _ = crop_bbox(frame, det_to_list(top), args.crop_padding)
                        if crop:
                            url = "data:image/jpeg;base64," + base64.b64encode(crop).decode()
                            t_llm = time.time()
                            verdict = call_vision(token, url, PEST_PROMPT, timeout=args.llm_timeout)
                            llm_dt = time.time() - t_llm
                            res = verdict.get("result")
                            analysis = verdict.get("analysis", "")
                            label = "PEST" if res == 0 else ("BENEFICIAL" if res == 1 else "LLM_ERR")
                            llm_line = f"llm={label}({res}) dt={llm_dt*1000:.0f}ms reason={analysis[:80]}"
                            last_llm_call_ts = ts_now
                            last_llm_key = (round(xc, 2), round(yc, 2), round(score, 2))

                print(
                    f"[tick {tick:>5}] ts={ts_now:.3f}  "
                    f"visual=animal sc={score:.3f} xc={xc:+.3f} yc={yc:+.3f}  "
                    f"{llm_line}",
                    flush=True,
                )

            # 等到下个 tick
            dt = time.time() - t0
            sleep_for = args.interval - dt
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        print(f"\n[monitor] stopped after {tick} ticks", flush=True)


if __name__ == "__main__":
    main()