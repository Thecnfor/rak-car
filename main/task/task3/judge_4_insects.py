#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/judge_4_insects.py - judge 4 insects shown one by one

User shows 4 insects to camera, ~2 seconds apart.
Script polls cam2 every 2s, when a new animal (different position) appears:
  - calls ERNIE VL to judge pest/beneficial
  - prints result immediately
  - adds to "judged" set so won't be re-judged

Stops after 4 unique insects have been judged (or Ctrl+C).

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.judge_4_insects
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
    except Exception as exc:
        print(f"[warn] get_task_state: {exc}", file=sys.stderr)
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


def fetch_frame(streamer_url, timeout=0.5):
    try:
        r = requests.get(f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout)
        r.raise_for_status()
        return r.content
    except Exception as exc:
        print(f"[warn] fetch frame: {exc}", file=sys.stderr)
        return None


def main():
    ap = argparse.ArgumentParser(description="judge 4 insects shown one by one")
    ap.add_argument("--interval", type=float, default=2.0,
                    help="poll interval sec (default 2.0)")
    ap.add_argument("--target-count", type=int, default=4, dest="target_count",
                    help="judge this many unique insects (default 4)")
    ap.add_argument("--min-score", type=float, default=0.50, dest="min_score")
    ap.add_argument("--llm-timeout", type=float, default=8.0, dest="llm_timeout")
    ap.add_argument("--crop-padding", type=float, default=0.10, dest="crop_padding")
    ap.add_argument("--token", default=None, help="ERNIE access token (or env ERNIE_ACCESS_TOKEN)")
    ap.add_argument("--streamer", default=None)
    args = ap.parse_args()

    token = args.token or os.getenv("ERNIE_ACCESS_TOKEN") or os.getenv("MINIMAX_API_KEY")
    if not token:
        print("[fatal] no token: --token or ERNIE_ACCESS_TOKEN env required",
              file=sys.stderr)
        sys.exit(2)

    settings_mod = __import__("main.settings", fromlist=["load_settings"])
    streamer_url = args.streamer or settings_mod.load_settings().streamer_url

    client = RuntimeApiClient()
    client.wait_until_ready()

    print(f"[judge] interval={args.interval}s target_count={args.target_count} "
          f"min_score={args.min_score} llm_timeout={args.llm_timeout}s",
          flush=True)
    print(f"[judge] ready - show the first insect when ready (Ctrl+C to stop)\n",
          flush=True)

    # 已判定的位置集合 (x_bin, y_bin)
    judged: set[tuple] = set()
    tick = 0
    t_start = time.time()

    try:
        while len(judged) < args.target_count:
            tick += 1
            t0 = time.time()
            ts_now = time.time()

            animals = get_animals(client, args.min_score)

            # 找一只新动物
            new_det = None
            new_pos = None
            for det in animals:
                b = det.get("bbox_norm") or {}
                xc = round(float(b.get("x_center", 0.0)), 2)
                yc = round(float(b.get("y_center", 0.0)), 2)
                key = (xc, yc)
                if key not in judged:
                    new_det = det
                    new_pos = key
                    break

            if new_det is None:
                elapsed = ts_now - t_start
                waited = int(elapsed) if elapsed < args.interval * 3 else int(args.interval)
                print(f"  [tick {tick}] waiting... (show next insect, judged={len(judged)}/{args.target_count})",
                      flush=True)
            else:
                # 立即判定
                b = new_det.get("bbox_norm") or {}
                xc = float(b.get("x_center", 0.0))
                yc = float(b.get("y_center", 0.0))
                score = float(new_det.get("score") or 0.0)
                frame = fetch_frame(streamer_url, timeout=0.5)

                if frame is None:
                    print(f"  [tick {tick}] frame fetch failed, skip (xc={xc:+.2f} yc={yc:+.2f})",
                          flush=True)
                else:
                    crop, _ = crop_bbox(frame, det_to_list(new_det), args.crop_padding)
                    if not crop:
                        print(f"  [tick {tick}] empty crop, skip", flush=True)
                    else:
                        url = "data:image/jpeg;base64," + base64.b64encode(crop).decode()
                        t_llm = time.time()
                        verdict = call_vision(token, url, PEST_PROMPT, timeout=args.llm_timeout)
                        llm_dt = time.time() - t_llm
                        res = verdict.get("result")
                        analysis = verdict.get("analysis", "")
                        if res == 0:
                            label_en = "PEST"
                            label_cn = "害虫"
                        elif res == 1:
                            label_en = "BENEFICIAL"
                            label_cn = "益虫"
                        else:
                            label_en = "LLM_ERR"
                            label_cn = "LLM失败"

                        judged.add(new_pos)
                        idx = len(judged)
                        print(
                            f"\n  === [{idx}/{args.target_count}] NEW INSECT JUDGED ===",
                            flush=True,
                        )
                        print(
                            f"    位置: (xc={xc:+.2f}, yc={yc:+.2f})  "
                            f"视觉分: {score:.2f}",
                            flush=True,
                        )
                        print(
                            f"    判定: {label_cn} ({label_en}, result={res})  "
                            f"LLM耗时 {llm_dt*1000:.0f}ms",
                            flush=True,
                        )
                        print(f"    理由: {analysis}", flush=True)
                        print("", flush=True)

                        if len(judged) < args.target_count:
                            print(
                                f"  >>> 准备下一只昆虫({len(judged)+1}/{args.target_count})... \n",
                                flush=True,
                            )

            # 等到下个 tick
            dt = time.time() - t0
            sleep_for = args.interval - dt
            if sleep_for > 0:
                time.sleep(sleep_for)

        print(f"\n========== DONE: {len(judged)}/{args.target_count} insects judged ==========",
              flush=True)
        for i, key in enumerate(sorted(judged), 1):
            xc, yc = key
            print(f"  第 {i} 只 (xc={xc:+.2f}, yc={yc:+.2f}): 见上方判定",
                  flush=True)
        print("=" * 60, flush=True)

    except KeyboardInterrupt:
        print(f"\n[judge] stopped after {len(judged)}/{args.target_count} insects",
              flush=True)


if __name__ == "__main__":
    main()