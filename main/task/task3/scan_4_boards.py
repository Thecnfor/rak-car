#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/scan_4_boards.py

任务:4 块板子(每块 1 只虫子,板间 ~8cm),通过摆臂(y)+前后(x)
让摄像头扫描并居中对准每块板子,用 ERNIE 识别每只是害虫/益虫,按顺序记录。

== 扫描策略 ==
1. y 从 -0.16 扫到 +0.16(覆盖 ~32cm,大于 4 块板的 24cm),每个 y 点停留一会看检测
2. 检测到 animal -> 调整 y 使 xc≈0(居中)+ 选合适的 x 距离
3. 用 ERNIE 判 PEST/BENEFICIAL,记录到 records
4. 用 距离+时间 去重(同一只不被多次计入)
5. 扫完一轮后若不够 4 只,可再扫一轮(直到 4 只 或 超时)

== 假设 ==
- 摄像头(cam2)装在 arm 上,可由 arm.move_y_position 摆左右
- 摄像头也可由 arm.move_x_position 前后调整距离
- 板子沿一条直线排列(横向),可由 y 摆动覆盖

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    $env:ERNIE_ACCESS_TOKEN = "..."
    python -m main.tasks.task333.scan_4_boards
    python -m main.tasks.task333.scan_4_boards --y-min -0.16 --y-max 0.16 --y-step 0.04
"""
from __future__ import annotations

import argparse
import base64
import json
import math
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


def arm_call(client, name, *a, timeout=20.0, **k):
    job = client.execute_arm_action(name, *a, timeout=timeout, sync=False, **k)
    done = client.wait_job(job["id"], timeout=timeout + 10)
    if done.get("status") != "succeeded":
        raise RuntimeError(f"arm.{name} failed: {done.get('error')}")
    return done.get("result")


def safe_arm(client, name, *a, **k):
    try:
        return arm_call(client, name, *a, **k)
    except Exception as e:
        print(f"[warn] arm.{name}: {e}", file=sys.stderr)
        return None


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


def fetch_frame(streamer_url, timeout=0.5):
    try:
        r = requests.get(f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout)
        r.raise_for_status()
        return r.content
    except Exception:
        return None


def find_matching_record(records, xc, yc, ts_now, merge_dist, merge_window):
    best_idx = -1
    best_dist = float("inf")
    for i, r in enumerate(records):
        if (ts_now - r["ts"]) > merge_window:
            continue
        d = math.hypot(xc - r["xc"], yc - r["yc"])
        if d < merge_dist and d < best_dist:
            best_dist = d
            best_idx = i
    return best_idx


def main():
    ap = argparse.ArgumentParser(
        description="scan 4 boards by sweeping arm y + ERNIE classify")
    ap.add_argument("--token", default=None,
                    help="ERNIE token (or env ERNIE_ACCESS_TOKEN)")
    ap.add_argument("--y-min", type=float, default=-0.16, dest="y_min",
                    help="sweep start y (m, default -0.16)")
    ap.add_argument("--y-max", type=float, default=0.16, dest="y_max",
                    help="sweep end y (m, default 0.16)")
    ap.add_argument("--y-step", type=float, default=0.04, dest="y_step",
                    help="sweep step y (m, default 0.04)")
    ap.add_argument("--x-pos", type=float, default=0.0, dest="x_pos",
                    help="arm x position while scanning (m, default 0.0)")
    ap.add_argument("--target-count", type=int, default=4, dest="target_count",
                    help="scan until this many insects found (default 4)")
    ap.add_argument("--max-rounds", type=int, default=3, dest="max_rounds",
                    help="max sweep rounds (default 3)")
    ap.add_argument("--center-tol", type=float, default=0.08, dest="center_tol",
                    help="after centering, xc must be < this (default 0.08)")
    ap.add_argument("--min-score", type=float, default=0.50, dest="min_score")
    ap.add_argument("--merge-dist", type=float, default=0.18, dest="merge_dist",
                    help="distance threshold for dedup (default 0.18)")
    ap.add_argument("--merge-window", type=float, default=8.0, dest="merge_window")
    ap.add_argument("--llm-timeout", type=float, default=8.0, dest="llm_timeout")
    ap.add_argument("--crop-padding", type=float, default=0.20, dest="crop_padding")
    ap.add_argument("--settle", type=float, default=0.4,
                    help="seconds to wait after move before reading (default 0.4)")
    ap.add_argument("--streamer", default=None)
    ap.add_argument("--save", type=str, default="audit/scan_4_boards.json")
    args = ap.parse_args()

    if args.y_step <= 0:
        ap.error("--y-step must be > 0")
    if args.y_min > args.y_max:
        ap.error("--y-min must be <= --y-max")

    token = args.token or os.getenv("ERNIE_ACCESS_TOKEN") or os.getenv("MINIMAX_API_KEY")
    if not token:
        print("[fatal] no token", file=sys.stderr)
        sys.exit(2)

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

    print(f"[ready] token={mask_token(token)} y=[{args.y_min},{args.y_max}] "
          f"step={args.y_step} x_pos={args.x_pos} target={args.target_count}",
          flush=True)

    # 先把 arm 移到指定 x 距离
    print(f"[setup] move_x_position({args.x_pos}) ...", flush=True)
    safe_arm(client, "move_x_position", args.x_pos, timeout=30)
    time.sleep(0.3)

    records: list[dict] = []
    sweep_round = 0

    try:
        while sweep_round < args.max_rounds and len(records) < args.target_count:
            sweep_round += 1
            print(f"\n========== SWEEP ROUND {sweep_round} ==========", flush=True)

            y = args.y_min
            while y <= args.y_max + 1e-3 and len(records) < args.target_count:
                # 1) 摆臂到 y
                safe_arm(client, "move_y_position", y, timeout=30)
                time.sleep(args.settle)

                # 2) 读检测
                top = get_top_animal(client, args.min_score)
                if top is None:
                    print(f"  [y={y:+.3f}] no detection", flush=True)
                    y += args.y_step
                    continue

                b = top.get("bbox_norm") or {}
                xc = float(b.get("x_center", 0.0))
                yc = float(b.get("y_center", 0.0))
                score = float(top.get("score") or 0.0)
                ts_now = time.time()

                # 3) 居中:调整 y 让 xc 接近 0
                if abs(xc) > args.center_tol:
                    # 根据 xc 偏移估计 y 调整量(粗略 0.02 rad ≈ 1cm)
                    y_adj = y + xc * 0.03
                    print(f"  [y={y:+.3f}] center adjust -> y={y_adj:+.3f} "
                          f"(xc={xc:+.3f})", flush=True)
                    safe_arm(client, "move_y_position", y_adj, timeout=30)
                    time.sleep(args.settle)
                    top = get_top_animal(client, args.min_score)
                    if top is None:
                        y += args.y_step
                        continue
                    b = top.get("bbox_norm") or {}
                    xc = float(b.get("x_center", 0.0))
                    yc = float(b.get("y_center", 0.0))
                    score = float(top.get("score") or 0.0)
                    y = y_adj   # 更新当前 y
                    ts_now = time.time()

                # 4) 去重
                match_idx = find_matching_record(
                    records, xc, yc, ts_now, args.merge_dist, args.merge_window,
                )
                if match_idx >= 0:
                    print(f"  [skip] same insect as #{records[match_idx]['order']}",
                          flush=True)
                    y += args.y_step
                    continue

                # 5) 调 LLM 判定
                if len(records) >= args.target_count:
                    break

                frame = fetch_frame(streamer_url, timeout=0.3)
                if frame is None:
                    print(f"  [y={y:+.3f}] frame fetch failed", flush=True)
                    y += args.y_step
                    continue

                crop, _ = crop_bbox(frame, det_to_list(top), args.crop_padding)
                if not crop:
                    print(f"  [y={y:+.3f}] empty crop", flush=True)
                    y += args.y_step
                    continue

                url = "data:image/jpeg;base64," + base64.b64encode(crop).decode()
                t0 = time.time()
                verdict = call_vision(token, url, PEST_PROMPT, timeout=args.llm_timeout)
                llm_dt = time.time() - t0

                res = verdict.get("result")
                analysis = verdict.get("analysis", "")
                if res == 0:
                    label_cn, label_en = "害虫", "PEST"
                elif res == 1:
                    label_cn, label_en = "益虫", "BENEFICIAL"
                else:
                    label_cn, label_en = "未识别", "UNKNOWN"

                order = len(records) + 1
                records.append({
                    "order": order,
                    "xc": round(xc, 3),
                    "yc": round(yc, 3),
                    "score": round(score, 3),
                    "label_cn": label_cn,
                    "label_en": label_en,
                    "analysis": analysis,
                    "llm_ms": int(llm_dt * 1000),
                    "ts": ts_now,
                    "y_at_found": round(y, 3),
                })

                print(
                    f"\n  >>> [{order}/{args.target_count}] INSECT "
                    f"at y={y:+.3f}m (xc={xc:+.2f}, yc={yc:+.2f})",
                    flush=True,
                )
                print(f"      视觉分: {score:.2f}", flush=True)
                print(f"      判定: {label_cn} ({label_en})  LLM {llm_dt*1000:.0f}ms",
                      flush=True)
                print(f"      理由: {analysis}", flush=True)

                y += args.y_step

            if len(records) >= args.target_count:
                break
            # 否则再扫一轮(从 y_min 开始,反向扫)

    except KeyboardInterrupt:
        print("\n[abort] KeyboardInterrupt", flush=True)

    # === 最终汇总(按发现顺序)===
    print("\n" + "=" * 60, flush=True)
    print(f"========== 最终汇总: 4 块板子识别结果 ==========", flush=True)
    print("=" * 60, flush=True)
    if records:
        for e in records:
            verdict_cn = {"PEST": "害虫", "BENEFICIAL": "益虫", "UNKNOWN": "未识别"}[e["label_en"]]
            print(
                f"  板 {e['order']}: {verdict_cn:<6}  "
                f"(at y={e['y_at_found']:+.3f}m, "
                f"xc={e['xc']:+.2f}, yc={e['yc']:+.2f})",
                flush=True,
            )
    else:
        print("  (本次扫描未识别到任何虫子)", flush=True)
    print("=" * 60, flush=True)

    # 落盘
    out = Path(args.save)
    if not out.is_absolute():
        out = Path(__file__).resolve().parent / args.save
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        json.dumps(
            {
                "config": vars(args),
                "sweep_rounds": sweep_round,
                "records": [{k: v for k, v in e.items() if k != "ts"} for e in records],
                "summary": {
                    "pest": sum(1 for e in records if e["label_en"] == "PEST"),
                    "beneficial": sum(1 for e in records if e["label_en"] == "BENEFICIAL"),
                    "unknown": sum(1 for e in records if e["label_en"] == "UNKNOWN"),
                },
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    print(f"\n[done] saved to {out}", flush=True)


if __name__ == "__main__":
    main()