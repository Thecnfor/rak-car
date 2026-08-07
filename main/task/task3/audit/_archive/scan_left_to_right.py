#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/scan_left_to_right.py - one-shot LLM scan, left-to-right report

流程:
  1. 抓 cam2 帧 + 读 task_feed 拿所有 animal 目标
  2. 按 xc 从小到大排序(从左到右)
  3. 对每只动物裁 bbox + 调 ERNIE VL 判定
  4. 一行一行打印:从左到右第 N 只是 PEST 还是 BENEFICIAL

跑法:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.scan_left_to_right
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
from main.tasks.task333.llm_ernie import call_vision, mask_token


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
    ap = argparse.ArgumentParser(description="one-shot scan, left-to-right LLM report")
    ap.add_argument("--min-score", type=float, default=0.50, dest="min_score")
    ap.add_argument("--llm-timeout", type=float, default=8.0, dest="llm_timeout",
                    help="LLM call timeout per detection (sec)")
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

    print(f"[scan] min_score={args.min_score} llm_timeout={args.llm_timeout}s",
          flush=True)

    # 1) 读所有动物 + 抓帧
    animals = get_animals(client, args.min_score)
    frame = fetch_frame(streamer_url, timeout=0.5)

    if not animals:
        print("\n>>> RESULT: no animals in view\n")
        return
    if frame is None:
        print("\n>>> RESULT: no frame fetched\n")
        return

    # 2) 按 xc 从小到大排序(从左到右)
    animals_sorted = sorted(
        animals,
        key=lambda d: float((d.get("bbox_norm") or {}).get("x_center", 0.0)),
    )

    print(f"[scan] found {len(animals_sorted)} animals, sorting left-to-right by xc\n",
          flush=True)

    # 3) 每只判一次
    results = []
    for i, det in enumerate(animals_sorted, 1):
        b = det.get("bbox_norm") or {}
        xc = float(b.get("x_center", 0.0))
        yc = float(b.get("y_center", 0.0))
        score = float(det.get("score") or 0.0)

        crop, _ = crop_bbox(frame, det_to_list(det), args.crop_padding)
        if not crop:
            results.append({
                "idx": i, "xc": xc, "yc": yc, "score": score,
                "label": "?", "analysis": "empty crop",
            })
            print(f"  [{i}] xc={xc:+.3f} sc={score:.2f}  LLM=SKIP (empty crop)",
                  flush=True)
            continue

        url = "data:image/jpeg;base64," + base64.b64encode(crop).decode()
        t0 = time.time()
        verdict = call_vision(token, url, PEST_PROMPT, timeout=args.llm_timeout)
        llm_dt = time.time() - t0

        res = verdict.get("result")
        analysis = verdict.get("analysis", "")
        if res == 0:
            label = "PEST"
        elif res == 1:
            label = "BENEFICIAL"
        else:
            label = "LLM_ERR"

        results.append({
            "idx": i, "xc": xc, "yc": yc, "score": score,
            "label": label, "analysis": analysis,
            "llm_ms": int(llm_dt * 1000),
        })
        print(
            f"  [{i}] xc={xc:+.3f} sc={score:.2f}  "
            f"LLM={label}({res}) [{llm_dt*1000:.0f}ms]  "
            f"-> {analysis[:120]}",
            flush=True,
        )

    # 4) 总结:从左到右
    print(f"\n========== FROM LEFT TO RIGHT ({len(results)} animals) ==========", flush=True)
    for r in results:
        verdict_cn = {
            "PEST": "害虫",
            "BENEFICIAL": "益虫",
            "LLM_ERR": "LLM失败",
            "?": "未识别",
        }.get(r["label"], r["label"])
        print(
            f"  第 {r['idx']} 只 (xc={r['xc']:+.2f}, yc={r['yc']:+.2f}, "
            f"score={r['score']:.2f}): {verdict_cn} ({r['label']})",
            flush=True,
        )
        if r.get("analysis") and r["label"] != "?":
            print(f"          LLM reason: {r['analysis']}", flush=True)
    print("=" * 60, flush=True)


if __name__ == "__main__":
    main()