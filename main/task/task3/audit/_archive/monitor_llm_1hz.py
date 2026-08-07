#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/monitor_llm_1hz.py - LLM-based 1Hz pest/beneficial monitor

每 1 秒:
  1. 读 task_feed,挑出视野里的所有 animal 目标
  2. 对每只动物(同位置 5 秒内复用上次判定,避免反复调 LLM):
     - 抓 cam2 帧 -> 裁 bbox -> base64
     - 调 ERNIE VL -> PEST(0) / BENEFICIAL(1) / ERR(None)
  3. 打印:这一秒每只动物的位置 + LLM 判定

按 Ctrl+C 停止。

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.monitor_llm_1hz
    python -m main.tasks.task333.monitor_llm_1hz --interval 1.0 --cooldown 5.0
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
from main.tasks.task333.llm_ernie import call_vision


PEST_PROMPT = """Classify the animal as pest or beneficial for farmland.
Output STRICT JSON only:
{"result": <0 or 1>, "analysis": "<one short sentence in English>"}
Rules:
- result=0: crop pest (locust, aphid, caterpillar, weevil, beetle, slug, snail, mite)
- result=1: beneficial (bee, ladybug, butterfly pollinator, earthworm)
- If no animal visible: {"result": 1, "analysis": "no animal"}
- Output ONLY the JSON object."""


def get_animals(client, min_score):
    try:
        ts = (client.get_task_state() or {}).get("task_state") or {}
        dets = list(ts.get("detections") or [])
    except Exception:
        return []
    out = []
    for d in dets:
        if d.get("label") != "animal":
            continue
        sc = float(d.get("score") or 0.0)
        if sc < min_score:
            continue
        out.append(d)
    return out


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
    ap = argparse.ArgumentParser(description="LLM-based 1Hz pest/beneficial monitor")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="tick interval sec (default 1.0)")
    ap.add_argument("--min-score", type=float, default=0.50, dest="min_score",
                    help="visual score threshold (default 0.50)")
    ap.add_argument("--cooldown", type=float, default=5.0,
                    help="seconds between LLM calls for same position (default 5.0)")
    ap.add_argument("--llm-timeout", type=float, default=0.8, dest="llm_timeout",
                    help="LLM call timeout per detection sec (default 0.8)")
    ap.add_argument("--crop-padding", type=float, default=0.10, dest="crop_padding")
    ap.add_argument("--token", default=None,
                    help="ERNIE access token (or env ERNIE_ACCESS_TOKEN)")
    ap.add_argument("--streamer", default=None)
    args = ap.parse_args()

    token = args.token or os.getenv("ERNIE_ACCESS_TOKEN") or os.getenv("MINIMAX_API_KEY")
    if not token:
        print("[fatal] no token: --token or ERNIE_ACCESS_TOKEN env required", file=sys.stderr)
        sys.exit(2)

    settings_mod = __import__("main.settings", fromlist=["load_settings"])
    streamer_url = args.streamer or settings_mod.load_settings().streamer_url

    client = RuntimeApiClient()
    client.wait_until_ready()

    print(f"[monitor] interval={args.interval}s min_score={args.min_score} "
          f"cooldown={args.cooldown}s", flush=True)
    print(f"[monitor] LLM model: ernie-4.5-turbo-vl", flush=True)
    print(f"[monitor] Ctrl+C to stop\n", flush=True)

    # verdict cache: pos_key -> {label, analysis, ts}
    verdict_cache: dict[tuple, dict] = {}
    tick = 0

    try:
        while True:
            tick += 1
            t0 = time.time()
            ts_now = time.time()

            animals = get_animals(client, args.min_score)

            if not animals:
                if tick % 5 == 0:
                    print(f"[tick {tick:>5}] ts={ts_now:.3f}  "
                          f"no animals in view  |  cache={len(verdict_cache)}",
                          flush=True)
                dt = time.time() - t0
                sleep_for = args.interval - dt
                if sleep_for > 0:
                    time.sleep(sleep_for)
                continue

            frame_results: list[dict] = []
            frame = fetch_frame(streamer_url, timeout=0.3)

            for det in animals:
                b = det.get("bbox_norm") or {}
                xc = float(b.get("x_center", 0.0))
                yc = float(b.get("y_center", 0.0))
                score = float(det.get("score") or 0.0)
                pos_key = (round(xc, 2), round(yc, 2))

                # 缓存命中
                cached = verdict_cache.get(pos_key)
                if cached and (ts_now - cached["ts"]) < args.cooldown:
                    frame_results.append({
                        "xc": xc, "yc": yc, "score": score,
                        "label": cached["label"],
                        "reason": cached["analysis"],
                        "source": "cache",
                    })
                    continue

                if frame is None:
                    frame_results.append({
                        "xc": xc, "yc": yc, "score": score,
                        "label": "?", "reason": "frame fetch failed",
                        "source": "n/a",
                    })
                    continue

                crop, _ = crop_bbox(frame, det_to_list(det), args.crop_padding)
                if not crop:
                    frame_results.append({
                        "xc": xc, "yc": yc, "score": score,
                        "label": "?", "reason": "empty crop",
                        "source": "n/a",
                    })
                    continue

                url = "data:image/jpeg;base64," + base64.b64encode(crop).decode()
                t_llm = time.time()
                verdict = call_vision(token, url, PEST_PROMPT, timeout=args.llm_timeout)
                llm_dt = time.time() - t_llm

                res = verdict.get("result")
                analysis = verdict.get("analysis", "")
                if res == 0:
                    label = "PEST"
                elif res == 1:
                    label = "BENEFICIAL"
                else:
                    label = "LLM_ERR"
                verdict_cache[pos_key] = {
                    "label": label, "analysis": analysis, "ts": ts_now,
                }
                frame_results.append({
                    "xc": xc, "yc": yc, "score": score,
                    "label": label, "reason": analysis,
                    "source": f"llm({llm_dt*1000:.0f}ms)",
                })

            # 打印
            parts = []
            for r in frame_results:
                parts.append(
                    f"({r['xc']:+.2f},{r['yc']:+.2f})={r['label']} "
                    f"sc={r['score']:.2f} [{r['source']}]"
                )
            print(
                f"[tick {tick:>5}] ts={time.time():.3f}  "
                f"animals={len(frame_results)}  " + " | ".join(parts),
                flush=True,
            )

            dt = time.time() - t0
            sleep_for = args.interval - dt
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        print(f"\n[monitor] stopped after {tick} ticks. cache_size={len(verdict_cache)}",
              flush=True)


if __name__ == "__main__":
    main()