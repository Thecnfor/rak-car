#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/monitor_500ms.py - 0.5s continuous LLM insect identifier

每 0.5s:
  1. 读 task_feed -> 找所有 animal 目标
  2. 对每只新目标(同位置 8s 内复用):调 MiniMax M2.7 判 PEST/BENEFICIAL
  3. 打印:位置 + 视觉分 + LLM 判定 + 理由

注意:
  - MiniMax M2.7 当前不支持 image_url 格式输入(实测 LLM 返回 "no image attached")
  - 这意味着 LLM 判定会一直失败,但代码本身完整,等换 API 后立即能用
  - 想看位置 + visual 分(纯视觉)不依赖 LLM,可以观察 cache 行为

按 Ctrl+C 停。

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.monitor_500ms
    python -m main.tasks.task333.monitor_500ms --interval 0.5 --cooldown 8.0
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


PEST_PROMPT = """You are looking at a cropped image of an animal from a farmland scene.
Classify the animal as PEST (0) or BENEFICIAL (1) for crops.

Output STRICT JSON only:
{"result": <0 or 1>, "analysis": "<one short sentence>"}

Rules:
- result=0 (PEST): locust, aphid, caterpillar, weevil, beetle, slug, snail, mite, grasshopper, moth larva, thrips, leafhopper
- result=1 (BENEFICIAL): bee, ladybug, butterfly (pollinator), earthworm, mantis, parasitoid wasp, spider (pest-eating)
- If you cannot identify an animal, output: {"result": 1, "analysis": "no recognizable animal"}
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
    ap = argparse.ArgumentParser(description="0.5s continuous LLM insect monitor")
    ap.add_argument("--interval", type=float, default=0.5,
                    help="poll interval sec (default 0.5)")
    ap.add_argument("--min-score", type=float, default=0.50, dest="min_score")
    ap.add_argument("--cooldown", type=float, default=8.0,
                    help="seconds between LLM calls for same position (default 8.0)")
    ap.add_argument("--llm-timeout", type=float, default=8.0, dest="llm_timeout")
    ap.add_argument("--crop-padding", type=float, default=0.20, dest="crop_padding")
    ap.add_argument("--no-llm", action="store_true",
                    help="skip LLM, just report visual positions")
    ap.add_argument("--token", default=None, help="ERNIE token (or env ERNIE_ACCESS_TOKEN)")
    ap.add_argument("--streamer", default=None)
    args = ap.parse_args()

    if not args.no_llm:
        token = args.token or os.getenv("ERNIE_ACCESS_TOKEN") or os.getenv("MINIMAX_API_KEY")
        if not token:
            print("[fatal] --no-llm OR provide --token / ERNIE_ACCESS_TOKEN env",
                  file=sys.stderr)
            sys.exit(2)
    else:
        token = None

    settings_mod = __import__("main.settings", fromlist=["load_settings"])
    streamer_url = args.streamer or settings_mod.load_settings().streamer_url

    client = RuntimeApiClient()
    client.wait_until_ready()

    print(f"[monitor] interval={args.interval}s llm={not args.no_llm} "
          f"min_score={args.min_score} cooldown={args.cooldown}s", flush=True)
    print(f"[monitor] Ctrl+C to stop\n", flush=True)

    verdict_cache: dict[tuple, dict] = {}
    tick = 0

    try:
        while True:
            tick += 1
            t0 = time.time()
            ts_now = time.time()

            animals = get_animals(client, args.min_score)

            if not animals:
                if tick % 20 == 0:   # 每 10s 打印一次空状态
                    print(f"[tick {tick:>6}] ts={ts_now:.3f}  no animals in view",
                          flush=True)
            else:
                frame = None if args.no_llm else fetch_frame(streamer_url, timeout=0.3)

                parts = []
                for det in animals:
                    b = det.get("bbox_norm") or {}
                    xc = float(b.get("x_center", 0.0))
                    yc = float(b.get("y_center", 0.0))
                    score = float(det.get("score") or 0.0)
                    pos_key = (round(xc, 2), round(yc, 2))

                    # 缓存
                    cached = verdict_cache.get(pos_key)
                    if cached and (ts_now - cached["ts"]) < args.cooldown:
                        parts.append(
                            f"({xc:+.2f},{yc:+.2f})={cached['label']} "
                            f"sc={score:.2f}[cache]"
                        )
                        continue

                    if args.no_llm or frame is None:
                        parts.append(f"({xc:+.2f},{yc:+.2f})=visual-only sc={score:.2f}")
                        continue

                    crop, _ = crop_bbox(frame, det_to_list(det), args.crop_padding)
                    if not crop:
                        parts.append(f"({xc:+.2f},{yc:+.2f})=empty-crop sc={score:.2f}")
                        continue

                    url = "data:image/jpeg;base64," + base64.b64encode(crop).decode()
                    t_llm = time.time()
                    verdict = call_vision(token, url, PEST_PROMPT, timeout=args.llm_timeout)
                    llm_dt = time.time() - t_llm

                    res = verdict.get("result")
                    analysis = verdict.get("analysis", "")
                    if res == 0:
                        label = "害虫"
                    elif res == 1:
                        label = "益虫"
                    else:
                        label = "?"
                    verdict_cache[pos_key] = {
                        "label": label, "analysis": analysis, "ts": ts_now,
                    }
                    parts.append(
                        f"({xc:+.2f},{yc:+.2f})={label} sc={score:.2f}"
                        f"[llm({llm_dt*1000:.0f}ms)]"
                    )

                print(
                    f"[tick {tick:>6}] ts={ts_now:.3f}  animals={len(animals)}  "
                    + " | ".join(parts),
                    flush=True,
                )

            dt = time.time() - t0
            sleep_for = args.interval - dt
            if sleep_for > 0:
                time.sleep(sleep_for)

    except KeyboardInterrupt:
        print(f"\n[monitor] stopped after {tick} ticks", flush=True)


if __name__ == "__main__":
    main()