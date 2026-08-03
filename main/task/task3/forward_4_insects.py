#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/forward_4_insects.py

车向前走 1.5m,沿途最多记录 4 只虫子,每只遇到时立刻报告 PEST/BENEFICIAL,
最后按遇到顺序打印 4 只汇总(害虫/益虫)。

== 关键改进(v2) ==
老的 cache 用 (round(xc,2), round(yc,2)) 当 key,但车移动 0.10m 时,xc 会变化 ~0.05,
导致同一只虫子在视野里反复被判成"新位置",4 只名额被同一只占满。

现在用 **距离阈值 + 时间阈值** 去重:
  - 已记录虫子列表,每只存 (xc, yc, timestamp)
  - 新检测 -> 找最近的一只:距离 < 0.20 且时间差 < 5s -> 同一只(忽略)
  - 否则 -> 新虫子,记录

效果:同一只虫子在视野里 1-2 秒(车移动 ~0.3m)只判一次,
后续车开走或角度漂移都不会重复计入。

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    $env:ERNIE_ACCESS_TOKEN = "..."
    python -m main.tasks.task333.forward_4_insects
    python -m main.tasks.task333.forward_4_insects --merge-dist 0.25
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


# === LLM 视觉相似度比对(防"误识别成同一只")===
# 用同一套 schema (result=0/1, analysis=描述) 让 _call_llm 能解析。
# result=1 = 是已记录的某只,analysis 写明匹配 #N
# result=0 = 是新动物
COMPARE_PROMPT = """Compare the CURRENT IMAGE (provided) with the previously seen animals listed below.

Previously seen animals:
{known_list}

Task: Is the current image the SAME individual animal as any of the above (just seen from a slightly different angle/distance/zoom)?
Or is it a COMPLETELY DIFFERENT animal (different species, or different individual)?

Output STRICT JSON only (no markdown, no Chinese):
{{"result": <0 or 1>, "analysis": "<one short sentence>"}}

Rules:
- result=1 if the current image is the SAME individual as one of the above animals (same species, just angle/distance differs)
  - In analysis, mention which #N it matches, e.g. "matches #2, same bee at different angle"
- result=0 if it is a DIFFERENT animal (different species or different individual)
  - In analysis, briefly describe what it is, e.g. "different insect, looks like a beetle"
- When in doubt (e.g., same species but unsure individual), prefer result=0 to avoid false merges
- Output ONLY the JSON object."""


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


def find_matching_record(records, xc, yc, merge_dist):
    """找最近的已记录虫子,如果距离<merge_dist,返回其 index,否则 -1。

    永久登记:已记录的虫子不再受时间限制,只要距离近就视为同一只。
    """
    best_idx = -1
    best_dist = float("inf")
    for i, r in enumerate(records):
        d = math.hypot(xc - r["xc"], yc - r["yc"])
        if d < merge_dist and d < best_dist:
            best_dist = d
            best_idx = i
    return best_idx


def find_nearby_record(records, xc, yc, near_dist):
    """位置 "模糊区"(near_dist 内)的所有已记录虫子,用于 LLM 比对。"""
    out = []
    for i, r in enumerate(records):
        d = math.hypot(xc - r["xc"], yc - r["yc"])
        if d < near_dist:
            out.append((i, r, d))
    return out


def llm_compare_to_known(token, image_url, records, llm_timeout):
    """调 LLM 比对当前图与已记录的所有昆虫,返回 (match_order, reason)。

    COMPARE prompt 用标准 schema {result: 0/1, analysis: "..."}:
      - result=1 -> 同一只(analysis 里说明匹配 #N)
      - result=0 -> 不同
    """
    import re
    if not records:
        return 0, "no known records"
    lines = []
    for r in records:
        lines.append(
            f"  #{r['order']}: {r['label_cn']} - {r['analysis']} "
            f"(at xc={r['xc']:+.2f}, yc={r['yc']:+.2f})"
        )
    known_list = "\n".join(lines)
    prompt = COMPARE_PROMPT.format(known_list=known_list)

    verdict = call_vision(token, image_url, prompt, timeout=llm_timeout)
    res = verdict.get("result")
    analysis = verdict.get("analysis") or ""
    if res == 1:
        # 从 analysis 里抠出 #N
        m = re.search(r"#\s*(\d+)", analysis)
        match_order = int(m.group(1)) if m else 0
    else:
        match_order = 0
    return match_order, analysis


def main():
    ap = argparse.ArgumentParser(
        description="forward 1.5m + ERNIE scan, max 4 insects (smart dedup)")
    ap.add_argument("--token", default=None,
                    help="ERNIE token (or env ERNIE_ACCESS_TOKEN)")
    ap.add_argument("--max-travel", type=float, default=1.5, dest="max_travel",
                    help="forward distance (m, default 1.5)")
    ap.add_argument("--dy", type=float, default=0.10,
                    help="step length (m, default 0.10)")
    ap.add_argument("--max-insects", type=int, default=4, dest="max_insects",
                    help="record up to this many insects (default 4)")
    ap.add_argument("--min-score", type=float, default=0.50, dest="min_score")
    ap.add_argument("--merge-dist", type=float, default=0.25, dest="merge_dist",
                    help="distance threshold for position-based dedup "
                         "(default 0.25)")
    ap.add_argument("--llm-compare-dist", type=float, default=0.45,
                    dest="llm_compare_dist",
                    help="positions within this distance get LLM visual similarity check "
                         "(default 0.45)")
    ap.add_argument("--llm-timeout", type=float, default=8.0, dest="llm_timeout")
    ap.add_argument("--crop-padding", type=float, default=0.20, dest="crop_padding")
    ap.add_argument("--streamer", default=None)
    ap.add_argument("--save", type=str, default="audit/forward_4_insects_v2.json")
    args = ap.parse_args()

    token = args.token or os.getenv("ERNIE_ACCESS_TOKEN") or os.getenv("MINIMAX_API_KEY")
    if not token:
        print("[fatal] no token: --token or ERNIE_ACCESS_TOKEN env", file=sys.stderr)
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

    print(f"[ready] token={mask_token(token)} max_travel={args.max_travel}m "
          f"dy={args.dy}m max_insects={args.max_insects} "
          f"min_score={args.min_score} merge_dist={args.merge_dist} "
          f"llm_compare_dist={args.llm_compare_dist} (permanent registration + "
          f"LLM visual similarity check)", flush=True)

    traveled = 0.0
    seg_idx = 0
    # 已记录虫子:每只有 xc/yc/ts/label/order
    records: list[dict] = []
    # 已调 LLM 的位置(避免短时间内对同一只反复调;与 records 互补)
    # 这里直接用 records 实现合并,所以不再单独 cache

    try:
        while traveled < args.max_travel - 1e-3:
            seg_idx += 1

            animals = get_animals(client, args.min_score)

            if animals and len(records) < args.max_insects:
                frame = fetch_frame(streamer_url, timeout=0.3)
                for det in animals:
                    if len(records) >= args.max_insects:
                        break

                    b = det.get("bbox_norm") or {}
                    xc = float(b.get("x_center", 0.0))
                    yc = float(b.get("y_center", 0.0))
                    score = float(det.get("score") or 0.0)
                    ts_now = time.time()

                    # 1) 位置近 (< merge_dist) -> 已登记,直接跳
                    match_idx = find_matching_record(
                        records, xc, yc, args.merge_dist,
                    )
                    if match_idx >= 0:
                        print(
                            f"  [skip-registered] board #{records[match_idx]['order']} "
                            f"already recorded (dist<{args.merge_dist}), skip",
                            flush=True,
                        )
                        continue

                    # 2) 位置模糊区 -> 让 LLM 视觉比对
                    if frame is None:
                        frame = fetch_frame(streamer_url, timeout=0.3)
                    if frame is None:
                        continue
                    crop, _ = crop_bbox(frame, det_to_list(det), args.crop_padding)
                    if not crop:
                        continue

                    url = "data:image/jpeg;base64," + base64.b64encode(crop).decode()

                    # 找位置近的已记录虫子(< llm_compare_dist),让 LLM 比对
                    nearby = find_nearby_record(records, xc, yc, args.llm_compare_dist)
                    if nearby:
                        match_order, cmp_reason = llm_compare_to_known(
                            token, url, [r for _, r, _ in nearby], args.llm_timeout,
                        )
                        if match_order and 1 <= match_order <= len(records):
                            # LLM 判定为已登记的某一只
                            print(
                                f"  [skip-llm-same] board #{match_order} "
                                f"matched by LLM (reason: {cmp_reason})",
                                flush=True,
                            )
                            continue
                        # LLM 判定是不同 -> 注册
                    else:
                        cmp_reason = "no nearby registered"

                    # 3) 调 LLM 判害虫/益虫
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

                    order = len(records) + 1
                    records.append({
                        "order": order,
                        "first_seen_at_seg": seg_idx,
                        "first_seen_traveled_m": round(traveled, 3),
                        "xc": round(xc, 3),
                        "yc": round(yc, 3),
                        "score": round(score, 3),
                        "label_cn": label_cn,
                        "label_en": label_en,
                        "analysis": analysis,
                        "llm_ms": int(llm_dt * 1000),
                        "ts": ts_now,
                    })

                    print(
                        f"\n  >>> [{order}/{args.max_insects}] NEW INSECT "
                        f"(segment {seg_idx}, traveled={traveled:.2f}m)",
                        flush=True,
                    )
                    print(f"      位置: (xc={xc:+.2f}, yc={yc:+.2f})  "
                          f"视觉分: {score:.2f}", flush=True)
                    print(f"      判定: {label_cn} ({label_en})  "
                          f"LLM {llm_dt*1000:.0f}ms", flush=True)
                    print(f"      理由: {analysis}", flush=True)
                    print(f"      (compare check: {cmp_reason})", flush=True)

            # 推进
            remaining = args.max_travel - traveled
            step = min(args.dy, remaining)
            if step <= 0:
                break
            print(
                f"[seg {seg_idx}] traveled={traveled:.2f}m -> move +{step:.2f}m "
                f"(animals={len(animals)}, recorded={len(records)}/{args.max_insects})",
                flush=True,
            )
            safe(car_call, client, "move_for", [float(step), 0.0, 0.0],
                 timeout=args.llm_timeout)
            traveled += step
            time.sleep(0.15)

    except KeyboardInterrupt:
        print("\n[abort] KeyboardInterrupt", flush=True)

    safe(car_call, client, "stop", timeout=10)

    # === 最终汇总(按遇到顺序)===
    print("\n" + "=" * 60, flush=True)
    print(f"========== 最终汇总: 按遇到顺序的 4 只虫子 ==========", flush=True)
    print(f"========== total traveled = {traveled:.2f}m ==========", flush=True)
    print("=" * 60, flush=True)
    if records:
        for e in records:
            verdict_cn = {
                "PEST": "害虫",
                "BENEFICIAL": "益虫",
                "UNKNOWN": "未识别",
            }[e["label_en"]]
            print(
                f"  第 {e['order']} 只: {verdict_cn:<6}  "
                f"(首次发现于 traveled={e['first_seen_traveled_m']:.2f}m, "
                f"xc={e['xc']:+.2f}, yc={e['yc']:+.2f})",
                flush=True,
            )
    else:
        print("  (本次扫描未记录到任何虫子)", flush=True)
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
                "traveled_m": round(traveled, 3),
                "insects_by_order": [
                    {k: v for k, v in e.items() if k != "ts"} for e in records
                ],
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