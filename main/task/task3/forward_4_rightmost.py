#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/forward_4_rightmost.py

任务:
  1. 车共前进 1m,每段至少 0.20m(dy >= 0.20)
  2. 每段停下后,从视野里挑**最右**(xc 最大)的虫子
  3. 调 ERNIE 判 PEST/BENEFICIAL + 视觉相似度去重
  4. 累计记 4 只不同虫子,按遇到顺序输出最终汇总

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    $env:ERNIE_ACCESS_TOKEN = "..."
    python -m main.tasks.task333.forward_4_rightmost
    python -m main.tasks.task333.forward_4_rightmost --max-travel 1.0 --dy 0.25
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import re
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


COMPARE_PROMPT = """Compare the CURRENT IMAGE with the previously seen animals:
{known_list}

Is the current image the SAME individual animal as any of the above (just seen at a different angle/distance)?
Or is it a COMPLETELY DIFFERENT animal?

Output STRICT JSON only:
{{"result": <0 or 1>, "analysis": "<one short sentence>"}}

- result=1: same individual. In analysis, mention which #N, e.g. "matches #2, same bee at different angle"
- result=0: different animal. Briefly describe it.
- When in doubt, prefer result=0 to avoid false merges.
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


def pick_rightmost(animals):
    """从 animals 里挑最右(xc 最大)的。"""
    if not animals:
        return None
    best = None
    best_xc = -2.0
    for d in animals:
        xc = float((d.get("bbox_norm") or {}).get("x_center", -2.0))
        if xc > best_xc:
            best_xc = xc
            best = d
    return best


def find_matching_record(records, xc, yc, merge_dist):
    best_idx = -1
    best_dist = float("inf")
    for i, r in enumerate(records):
        d = math.hypot(xc - r["xc"], yc - r["yc"])
        if d < merge_dist and d < best_dist:
            best_dist = d
            best_idx = i
    return best_idx


def find_nearby_record(records, xc, yc, near_dist):
    out = []
    for i, r in enumerate(records):
        d = math.hypot(xc - r["xc"], yc - r["yc"])
        if d < near_dist:
            out.append((i, r, d))
    return out


def llm_compare_to_known(token, image_url, records, llm_timeout):
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
        m = re.search(r"#\s*(\d+)", analysis)
        match_order = int(m.group(1)) if m else 0
    else:
        match_order = 0
    return match_order, analysis


def main():
    ap = argparse.ArgumentParser(
        description="forward 1m (dy>=0.20), pick rightmost, record 4")
    ap.add_argument("--token", default=None,
                    help="ERNIE token (or env ERNIE_ACCESS_TOKEN)")
    ap.add_argument("--max-travel", type=float, default=1.0, dest="max_travel",
                    help="total forward distance (m, default 1.0)")
    ap.add_argument("--dy", type=float, default=0.20,
                    help="per-step distance, must be >= 0.15 (m, default 0.20)")
    ap.add_argument("--max-insects", type=int, default=4, dest="max_insects",
                    help="record up to this many insects (default 4)")
    ap.add_argument("--min-score", type=float, default=0.50, dest="min_score")
    ap.add_argument("--merge-dist", type=float, default=0.25, dest="merge_dist")
    ap.add_argument("--llm-compare-dist", type=float, default=0.45,
                    dest="llm_compare_dist")
    ap.add_argument("--llm-timeout", type=float, default=8.0, dest="llm_timeout")
    ap.add_argument("--crop-padding", type=float, default=0.20, dest="crop_padding")
    ap.add_argument("--streamer", default=None)
    ap.add_argument("--save", type=str, default="audit/forward_4_rightmost.json")
    args = ap.parse_args()

    if args.dy < 0.15:
        ap.error("--dy must be >= 0.15 m (per spec)")
    if args.max_travel <= 0:
        ap.error("--max-travel must be > 0")

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

    n_steps = int(args.max_travel / args.dy)
    print(f"[ready] token={mask_token(token)} max_travel={args.max_travel}m "
          f"dy={args.dy}m (>=0.20) -> {n_steps} steps "
          f"max_insects={args.max_insects} merge_dist={args.merge_dist} "
          f"llm_compare_dist={args.llm_compare_dist} "
          f"(pick rightmost xc)", flush=True)

    traveled = 0.0
    seg_idx = 0
    records: list[dict] = []

    try:
        while traveled < args.max_travel - 1e-3 and len(records) < args.max_insects:
            seg_idx += 1

            # 1) 推进(每段至少 0.20m)
            remaining = args.max_travel - traveled
            step = min(args.dy, remaining)
            if step < 0.15 and remaining > 0:
                # 最后一段不足 0.15,但允许(已经接近目标)
                pass
            print(
                f"[seg {seg_idx}] traveled={traveled:.2f}m -> move +{step:.2f}m",
                flush=True,
            )
            safe(car_call, client, "move_for", [float(step), 0.0, 0.0],
                 timeout=args.job_timeout if hasattr(args, "job_timeout") else 20)
            traveled += step
            time.sleep(0.2)

            if len(records) >= args.max_insects:
                break

            # 2) 读检测 -> 挑**最右**
            animals = get_animals(client, args.min_score)
            rightmost = pick_rightmost(animals)

            if rightmost is None:
                print(f"  no animals in view, continue", flush=True)
                continue

            b = rightmost.get("bbox_norm") or {}
            xc = float(b.get("x_center", 0.0))
            yc = float(b.get("y_center", 0.0))
            score = float(rightmost.get("score") or 0.0)
            ts_now = time.time()

            # 3) 位置 dedup
            match_idx = find_matching_record(records, xc, yc, args.merge_dist)
            if match_idx >= 0:
                print(
                    f"  [skip-registered] board #{records[match_idx]['order']} "
                    f"already recorded (dist<{args.merge_dist})",
                    flush=True,
                )
                continue

            # 4) 抓帧 + LLM
            frame = fetch_frame(streamer_url, timeout=0.3)
            if frame is None:
                continue
            crop, _ = crop_bbox(frame, det_to_list(rightmost), args.crop_padding)
            if not crop:
                continue
            url = "data:image/jpeg;base64," + base64.b64encode(crop).decode()

            # 5) 位置模糊区 -> LLM 视觉相似度比对
            nearby = find_nearby_record(records, xc, yc, args.llm_compare_dist)
            if nearby:
                match_order, cmp_reason = llm_compare_to_known(
                    token, url, [r for _, r, _ in nearby], args.llm_timeout,
                )
                if match_order and 1 <= match_order <= len(records):
                    print(
                        f"  [skip-llm-same] board #{match_order} matched by LLM "
                        f"(reason: {cmp_reason})",
                        flush=True,
                    )
                    continue
            else:
                cmp_reason = "no nearby registered"

            # 6) 调 LLM 判害虫/益虫
            t0 = time.time()
            verdict = call_vision(token, url, PEST_PROMPT, timeout=args.llm_timeout)
            llm_dt = time.time() - t0

            res = verdict.get("result")
            analysis = verdict.get("analysis", "")

            # LLM 判不了(result 不是 0 或 1) -> 跳过不记录
            if res not in (0, 1):
                print(
                    f"  [skip-llm-unknown] LLM could not classify "
                    f"(result={res}, analysis={analysis[:80]})",
                    flush=True,
                )
                continue

            label_cn = "害虫" if res == 0 else "益虫"
            label_en = "PEST" if res == 0 else "BENEFICIAL"

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
                f"\n  >>> [{order}/{args.max_insects}] NEW INSECT (rightmost) "
                f"at traveled={traveled:.2f}m",
                flush=True,
            )
            print(f"      位置: xc={xc:+.2f}, yc={yc:+.2f}  视觉分: {score:.2f}",
                  flush=True)
            print(f"      判定: {label_cn} ({label_en})  LLM {llm_dt*1000:.0f}ms",
                  flush=True)
            print(f"      理由: {analysis}", flush=True)

    except KeyboardInterrupt:
        print("\n[abort] KeyboardInterrupt", flush=True)

    safe(car_call, client, "stop", timeout=10)

    # === 最终汇总(按遇到顺序)===
    print("\n" + "=" * 60, flush=True)
    print(f"========== 最终汇总: 按遇到顺序的 {len(records)} 只虫子 ==========",
          flush=True)
    print(f"========== total traveled = {traveled:.2f}m ==========", flush=True)
    print("=" * 60, flush=True)
    if records:
        for e in records:
            verdict_cn = {"PEST": "害虫", "BENEFICIAL": "益虫", "UNKNOWN": "未识别"}[e["label_en"]]
            print(
                f"  第 {e['order']} 只: {verdict_cn:<6}  "
                f"(首次发现于 traveled={e['first_seen_traveled_m']:.2f}m, "
                f"xc={e['xc']:+.2f})",
                flush=True,
            )
    else:
        print("  (本次扫描未识别到任何虫子)", flush=True)
    print("=" * 60, flush=True)

    out = Path(args.save)
    if not out.is_absolute():
        out = Path(__file__).resolve().parent / args.save
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(
        json.dumps(
            {
                "config": vars(args),
                "traveled_m": round(traveled, 3),
                "records": [{k: v for k, v in e.items() if k != "ts"} for e in records],
                "summary": {
                    "pest": sum(1 for e in records if e["label_en"] == "PEST"),
                    "beneficial": sum(1 for e in records if e["label_en"] == "BENEFICIAL"),
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