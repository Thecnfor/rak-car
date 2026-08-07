#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/forward_1_5m_scan.py

车向前走 1.5m,沿途每看到一只 animal 就调 ERNIE 判害虫/益虫并报告。

流程:
  1. 读 task_feed -> 挑高分 animal 目标
  2. 对每个新位置(未在 cache 中),调百度 ERNIE 判 PEST/BENEFICIAL
  3. 立即打印 + 写入 cache(同一位置 8s 内复用)
  4. 底盘每段向前推进 dy 米(默认 0.10m,小步)
  5. 累计走满 1.5m 或已达边界 -> 停 + 汇总

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    $env:ERNIE_ACCESS_TOKEN = "..."
    python -m main.tasks.task333.forward_1_5m_scan
    python -m main.tasks.task333.forward_1_5m_scan --max-travel 1.5 --dy 0.10
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import sys
import time
from pathlib import Path

import requests

from main.api_client import RuntimeApiClient
from main.misc.test_pest_llm_shoot import crop_bbox
from main.tasks.task333.llm_ernie import call_vision, mask_token


PEST_PROMPT = """你是一个农田动物识别专家。
严格按 JSON 格式输出(不要 Markdown 不要解释):
{"result": 0 或 1, "analysis": "<一句话中文>"}

- result=0: 有害害虫(蝗虫、蚜虫、毛毛虫、象鼻虫、甲虫、蛞蝓、蜗牛、螨、蛾幼虫、蓟马、叶蝉)
- result=1: 有益动物(蜜蜂、瓢虫、蝴蝶(传粉)、蚯蚓、螳螂、寄生蜂、吃害虫的蜘蛛)
- 如果看不清是什么动物: {"result": 1, "analysis": "未识别出动物"}
- 只输出 JSON。"""


def car_call(client, name, *a, timeout=20.0, **k):
    job = client.execute_car_action(name, *a, timeout=timeout, sync=False, **k)
    done = client.wait_job(job["id"], timeout=timeout + 10)
    if done.get("status") != "succeeded":
        raise RuntimeError(f"car.{name} failed: {done.get('error')}")
    return done.get("result")


def safe(fn, *a, **k):
    try:
        return fn(*a, **k)
    except Exception as e:
        print(f"[warn] {fn.__name__}: {e}", file=sys.stderr)
        return None


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


def fetch_frame(streamer_url, timeout=0.5):
    try:
        r = requests.get(f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout)
        r.raise_for_status()
        return r.content
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser(description="forward 1.5m + ERNIE insect scan")
    ap.add_argument("--token", default=None,
                    help="ERNIE token (or env ERNIE_ACCESS_TOKEN)")
    ap.add_argument("--max-travel", type=float, default=1.5, dest="max_travel",
                    help="forward distance (m, default 1.5)")
    ap.add_argument("--dy", type=float, default=0.10,
                    help="step length (m, default 0.10)")
    ap.add_argument("--min-score", type=float, default=0.50, dest="min_score")
    ap.add_argument("--cooldown", type=float, default=8.0,
                    help="seconds to reuse LLM verdict for same position")
    ap.add_argument("--llm-timeout", type=float, default=8.0, dest="llm_timeout")
    ap.add_argument("--crop-padding", type=float, default=0.20, dest="crop_padding")
    ap.add_argument("--no-llm", action="store_true",
                    help="skip LLM, only report visual positions")
    ap.add_argument("--streamer", default=None)
    ap.add_argument("--save", type=str, default="audit/forward_1_5m_scan.json")
    args = ap.parse_args()

    if not args.no_llm:
        token = args.token or os.getenv("ERNIE_ACCESS_TOKEN") or os.getenv("MINIMAX_API_KEY")
        if not token:
            print("[fatal] --no-llm OR --token / ERNIE_ACCESS_TOKEN env",
                  file=sys.stderr)
            sys.exit(2)
    else:
        token = None

    settings_mod = __import__("main.settings", fromlist=["load_settings"])
    settings = settings_mod.load_settings()
    streamer_url = args.streamer or settings.streamer_url

    client = RuntimeApiClient()
    for _ in range(60):
        h = client.get_health()
        s = h.get("state", {})
        if s.get("initialized") and not s.get("initializing"):
            break
        time.sleep(0.5)

    print(f"[ready] token={mask_token(token) if token else 'none'} "
          f"max_travel={args.max_travel}m dy={args.dy}m "
          f"min_score={args.min_score} cooldown={args.cooldown}s "
          f"llm={not args.no_llm}", flush=True)

    # 行程追踪
    traveled = 0.0
    verdict_cache: dict[tuple, dict] = {}
    found: list[dict] = []
    seg_idx = 0

    try:
        while traveled < args.max_travel - 1e-3:
            seg_idx += 1

            # 1) 视觉检测
            animals = get_animals(client, args.min_score)

            # 2) 对每只 animal 判 LLM(用 cache 复用)
            if animals:
                frame = None if args.no_llm else fetch_frame(streamer_url, timeout=0.3)
                for det in animals:
                    b = det.get("bbox_norm") or {}
                    xc = float(b.get("x_center", 0.0))
                    yc = float(b.get("y_center", 0.0))
                    score = float(det.get("score") or 0.0)
                    pos_key = (round(xc, 2), round(yc, 2))
                    ts_now = time.time()

                    cached = verdict_cache.get(pos_key)
                    if cached and (ts_now - cached["ts"]) < args.cooldown:
                        continue   # 已判过,不重复

                    if args.no_llm or frame is None:
                        continue

                    crop, _ = crop_bbox(frame, det_to_list(det), args.crop_padding)
                    if not crop:
                        continue
                    url = "data:image/jpeg;base64," + base64.b64encode(crop).decode()
                    t0 = time.time()
                    verdict = call_vision(token, url, PEST_PROMPT, timeout=args.llm_timeout)
                    llm_dt = time.time() - t0

                    res = verdict.get("result")
                    analysis = verdict.get("analysis", "")
                    if res == 0:
                        label_cn = "害虫"
                        label_en = "PEST"
                    elif res == 1:
                        label_cn = "益虫"
                        label_en = "BENEFICIAL"
                    else:
                        label_cn = "未识别"
                        label_en = "UNKNOWN"
                    verdict_cache[pos_key] = {
                        "label_cn": label_cn, "label_en": label_en,
                        "analysis": analysis, "ts": ts_now,
                    }
                    entry = {
                        "seg_idx": seg_idx,
                        "traveled_m": round(traveled, 3),
                        "xc": round(xc, 3),
                        "yc": round(yc, 3),
                        "score": round(score, 3),
                        "label_cn": label_cn,
                        "label_en": label_en,
                        "analysis": analysis,
                        "llm_ms": int(llm_dt * 1000),
                    }
                    found.append(entry)
                    print(
                        f"\n  >>> [{len(found)}] INSECT at traveled={traveled:.2f}m  "
                        f"(xc={xc:+.2f}, yc={yc:+.2f}, sc={score:.2f})",
                        flush=True,
                    )
                    print(f"      判定: {label_cn} ({label_en})  LLM {llm_dt*1000:.0f}ms",
                          flush=True)
                    print(f"      理由: {analysis}", flush=True)

            # 3) 推进
            remaining = args.max_travel - traveled
            step = min(args.dy, remaining)
            if step <= 0:
                break
            print(
                f"[seg {seg_idx}] traveled={traveled:.2f}m -> move +{step:.2f}m "
                f"(animals={len(animals)})",
                flush=True,
            )
            safe(car_call, client, "move_for", [float(step), 0.0, 0.0],
                 timeout=args.llm_timeout)
            traveled += step
            time.sleep(0.15)

    except KeyboardInterrupt:
        print("\n[abort] KeyboardInterrupt", flush=True)

    safe(car_call, client, "stop", timeout=10)

    # 落盘
    out = Path(args.save)
    if not out.is_absolute():
        out = Path(__file__).resolve().parent / args.save
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        json.dumps(
            {
                "config": vars(args),
                "traveled_m": round(traveled, 3),
                "insects": found,
                "summary": {
                    "total": len(found),
                    "pest": sum(1 for f in found if f["label_en"] == "PEST"),
                    "beneficial": sum(1 for f in found if f["label_en"] == "BENEFICIAL"),
                    "unknown": sum(1 for f in found if f["label_en"] == "UNKNOWN"),
                },
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    print(f"\n========== DONE: traveled={traveled:.2f}m ==========", flush=True)
    print(f"  共发现 {len(found)} 只动物", flush=True)
    for e in found:
        print(
            f"    害虫/益虫/未识别 = {e['label_cn']:<6}  "
            f"位置 traveled={e['traveled_m']:.2f}m xc={e['xc']:+.2f}  "
            f"理由: {e['analysis'][:80]}",
            flush=True,
        )
    print(f"  结果文件: {out}", flush=True)


if __name__ == "__main__":
    main()