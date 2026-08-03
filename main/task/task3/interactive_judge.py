#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/interactive_judge.py - one-by-one LLM judge (manual trigger)

Interactive flow:
  1. Script starts, prints "Show insect #1, press Enter when ready"
  2. You place insect in cam2 view, press Enter
  3. Script captures frame, finds best animal, calls ERNIE VL
  4. Prints verdict: PEST / BENEFICIAL / LLM_ERR + reason
  5. Repeats for #2, #3, #4 (or Ctrl+C to stop)

Each judgment only fires once when you press Enter.
No auto-polling - you control the pace.

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.interactive_judge
    python -m main.tasks.task333.interactive_judge --target-count 4
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


PEST_PROMPT = """You are looking at a small cropped image of an animal from a farmland scene.
Identify the animal and decide if it is a CROP PEST or BENEFICIAL for the farmland.

Output STRICT JSON only (no markdown, no commentary, no Chinese):
{"result": <0 or 1>, "analysis": "<one short sentence in English>"}

Rules:
- result=0 (PEST): locust, aphid, caterpillar, weevil, beetle, slug, snail, mite, grasshopper, locust, moth larva, aphid, thrips, leafhopper
- result=1 (BENEFICIAL): bee, ladybug/ladybird, butterfly (pollinator), earthworm, mantis (praying mantis), parasitoid wasp, spider that eats pests
- If you cannot identify an animal in the image, output: {"result": 1, "analysis": "no recognizable animal in image"}
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


def fetch_frame(streamer_url, timeout=0.5):
    try:
        r = requests.get(f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout)
        r.raise_for_status()
        return r.content
    except Exception:
        return None


def wait_for_enter(prompt, auto_after=None):
    """Wait for Enter; if auto_after is set, auto-press after N seconds."""
    if auto_after is None or auto_after <= 0:
        try:
            input(prompt)
            return True
        except EOFError:
            return False
    print(prompt, end="", flush=True)
    print(f" (auto-press in {auto_after}s)", flush=True)
    # 用 select 非阻塞读 stdin(只在类 Unix 平台/有 tty 时有效)
    try:
        import select
        import sys
        if sys.stdin.isatty():
            ready, _, _ = select.select([sys.stdin], [], [], auto_after)
            if ready:
                sys.stdin.readline()
            else:
                print(f"  [auto] pressing Enter after {auto_after}s timeout", flush=True)
            return True
        else:
            # 没 tty(从其他进程拉起来),直接 sleep + 继续
            time.sleep(auto_after)
            print(f"  [auto] pressing Enter after {auto_after}s timeout (no tty)",
                  flush=True)
            return True
    except (ImportError, OSError):
        time.sleep(auto_after)
        print(f"  [auto] pressing Enter after {auto_after}s timeout", flush=True)
        return True


def main():
    ap = argparse.ArgumentParser(description="interactive one-by-one LLM judge")
    ap.add_argument("--target-count", type=int, default=4, dest="target_count",
                    help="how many insects to judge (default 4)")
    ap.add_argument("--min-score", type=float, default=0.50, dest="min_score")
    ap.add_argument("--llm-timeout", type=float, default=15.0, dest="llm_timeout")
    ap.add_argument("--crop-padding", type=float, default=0.20, dest="crop_padding",
                    help="bbox padding (default 0.20, larger than 0.10 for better LLM)")
    ap.add_argument("--auto-press-after", type=float, default=None, dest="auto_press_after",
                    help="auto-press Enter after N seconds (no manual Enter needed)")
    ap.add_argument("--save-frames", type=str, default=None, dest="save_frames",
                    help="save each captured frame to this dir for debug")
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

    print(f"[judge] interactive mode, target={args.target_count} insects",
          flush=True)
    print(f"[judge] min_score={args.min_score} crop_padding={args.crop_padding} "
          f"llm_timeout={args.llm_timeout}s", flush=True)
    print("", flush=True)

    try:
        for i in range(1, args.target_count + 1):
            print(f"--- [{i}/{args.target_count}] Place insect #{i} in cam2 view ---",
                  flush=True)
            if not wait_for_enter(f"  Press Enter when insect #{i} is ready: ",
                                  auto_after=args.auto_press_after):
                print("\n[judge] input closed, exit", flush=True)
                return

            # 抓当前帧
            frame = fetch_frame(streamer_url, timeout=0.5)
            if frame is None:
                print(f"  [err] frame fetch failed", flush=True)
                continue

            # (可选)保存原帧到 audit 目录
            if args.save_frames:
                from pathlib import Path
                save_dir = Path(args.save_frames)
                save_dir.mkdir(parents=True, exist_ok=True)
                save_path = save_dir / f"insect_{i:02d}.jpg"
                save_path.write_bytes(frame)
                print(f"  [saved] frame -> {save_path}", flush=True)

            top = get_top_animal(client, args.min_score)
            if top is None:
                print(f"  [err] no animal detected (min_score={args.min_score})", flush=True)
                print(f"         try lower min_score or improve lighting/position",
                      flush=True)
                # 允许重试同一只
                i -= 1
                continue

            b = top.get("bbox_norm") or {}
            xc = float(b.get("x_center", 0.0))
            yc = float(b.get("y_center", 0.0))
            score = float(top.get("score") or 0.0)

            crop, _ = crop_bbox(frame, det_to_list(top), args.crop_padding)
            if not crop:
                print(f"  [err] empty crop at (xc={xc:+.2f}, yc={yc:+.2f})", flush=True)
                i -= 1
                continue

            url = "data:image/jpeg;base64," + base64.b64encode(crop).decode()
            t0 = time.time()
            print(f"  [LLM] judging (visual sc={score:.2f}, xc={xc:+.2f}, yc={yc:+.2f})...",
                  flush=True)
            verdict = call_vision(token, url, PEST_PROMPT, timeout=args.llm_timeout)
            dt = time.time() - t0

            res = verdict.get("result")
            analysis = verdict.get("analysis", "")
            if res == 0:
                label_cn = "害虫"
                label_en = "PEST"
            elif res == 1:
                label_cn = "益虫"
                label_en = "BENEFICIAL"
            else:
                label_cn = "LLM失败"
                label_en = "LLM_ERR"

            print("", flush=True)
            print(f"  ===== INSECT #{i} =====", flush=True)
            print(f"    位置:   (xc={xc:+.2f}, yc={yc:+.2f})", flush=True)
            print(f"    视觉分: {score:.2f}", flush=True)
            print(f"    判定:   {label_cn} ({label_en}, result={res})", flush=True)
            print(f"    理由:   {analysis}", flush=True)
            print(f"    耗时:   {dt*1000:.0f}ms", flush=True)
            print("", flush=True)

        print(f"========== DONE: {args.target_count}/{args.target_count} ==========",
              flush=True)

    except KeyboardInterrupt:
        print("\n[judge] stopped by user", flush=True)


if __name__ == "__main__":
    main()