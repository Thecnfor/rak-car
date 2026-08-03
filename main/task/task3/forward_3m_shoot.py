#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/tasks/task333/forward_3m_shoot.py

完整业务:
1. 把机械臂 y 推到 0(复位)
2. 底盘面向图片向前走 3m
3. 每段检测侧摄 cam2 视野里的 animal
4. 当目标水平居中(|xc| <= tol_x)且是高分,裁 bbox 调 ERNIE VL 多模态
5. LLM 判 result=0(有害) -> shooting 一发
6. LLM 判 result=1(有益) / None(失败) -> 跳过继续走
7. 走满 3m 或弹药用完或 4 只害虫全部命中 -> 停

用法:
    $env:ERNIE_ACCESS_TOKEN = "..."
    python -m main.tasks.task333.forward_3m_shoot

可选参数:
    --max-shots 4    --max-travel 3.0    --dy 0.05
    --tol-x 0.10    --min-score 0.50
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
from main.tasks.task333.llm_ernie import call_vision, check_health, mask_token
from main.misc.test_pest_llm_shoot import crop_bbox


# ===== ERNIE 多模态 prompt(英文 + 强制 JSON) =====
PEST_PROMPT = """Analyze the image and classify the animal as pest or beneficial for farmland.
Output STRICT JSON only (no markdown, no commentary, no Chinese):
{"result": <0 or 1>, "analysis": "<one short sentence in English>"}
Rules:
- result=0: crop pest (locust, aphid, caterpillar, weevil, beetle, slug, snail, mite)
- result=1: beneficial (bee, ladybug, butterfly pollinator, earthworm, pest-eating spider)
- analysis: one sentence describing the animal and why it is pest/beneficial
- If no animal visible: {"result": 1, "analysis": "no animal detected"}
- Output ONLY the JSON object."""


# ===== HTTP 工具(异步+轮询,避开网关 504) =====
def car_call(client, name, *args, timeout=20.0, **kw):
    job = client.execute_car_action(name, *args, timeout=timeout, sync=False, **kw)
    done = client.wait_job(job["id"], timeout=timeout + 10)
    if done.get("status") != "succeeded":
        raise RuntimeError(f"car.{name} failed: {done.get('error')}")
    return done.get("result")


def arm_call(client, name, *args, timeout=30.0, **kw):
    job = client.execute_arm_action(name, *args, timeout=timeout, sync=False, **kw)
    done = client.wait_job(job["id"], timeout=timeout + 10)
    if done.get("status") != "succeeded":
        raise RuntimeError(f"arm.{name} failed: {done.get('error')}")
    return done.get("result")


def safe(fn, *a, **kw):
    try:
        return fn(*a, **kw)
    except Exception as e:
        print(f"[warn] {fn.__name__}: {e}", file=sys.stderr)
        return None


def get_detections(client):
    try:
        ts = (client.get_task_state() or {}).get("task_state") or {}
        return list(ts.get("detections") or [])
    except Exception as e:
        print(f"[warn] get_task_state: {e}", file=sys.stderr)
        return []


def get_odom(client):
    try:
        return list((client.get_runtime() or {}).get("runtime", {}).get("odometry") or [0, 0, 0])
    except Exception:
        return [0.0, 0.0, 0.0]


def get_arm(client):
    try:
        return (client.get_arm_state() or {}).get("arm_state") or {}
    except Exception:
        return {}


def fetch_frame(streamer_url, timeout=5.0):
    try:
        r = requests.get(f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout)
        r.raise_for_status()
        return r.content
    except Exception as e:
        print(f"[warn] fetch frame: {e}", file=sys.stderr)
        return None


def det_to_list(d):
    b = d.get("bbox_norm") or {}
    return [
        d.get("cls_id"), d.get("det_id"), d.get("label", ""),
        d.get("score", 0.0),
        b.get("x_center", 0.0), b.get("y_center", 0.0),
        b.get("width", 0.0), b.get("height", 0.0),
    ]


# ===== 复位 arm.y = 0 =====
def reset_arm_y(client, job_timeout=30.0):
    print("[reset] arm.move_y_position(0.0) ...")
    cur = get_arm(client).get("y_m")
    print(f"[reset] arm before y_m={cur}")
    try:
        arm_call(client, "move_y_position", 0.0, timeout=job_timeout)
    except Exception as e:
        print(f"[warn] move_y_position(0.0) failed: {e}", file=sys.stderr)
        return
    time.sleep(0.5)
    after = get_arm(client).get("y_m")
    print(f"[reset] arm after  y_m={after}")


# ===== 主流程 =====
def main():
    ap = argparse.ArgumentParser(description="arm y->0, forward 3m, LLM-judge pests, shoot")
    ap.add_argument("--token", default=None)
    ap.add_argument("--max-shots", type=int, default=4)
    ap.add_argument("--max-travel", type=float, default=3.0)
    ap.add_argument("--dy", type=float, default=0.05)
    ap.add_argument("--tol-x", type=float, default=0.10)
    ap.add_argument("--unlock-x", type=float, default=0.45)
    ap.add_argument("--min-score", type=float, default=0.50)
    ap.add_argument("--crop-padding", type=float, default=0.10)
    ap.add_argument("--llm-timeout", type=float, default=12.0)
    ap.add_argument("--job-timeout", type=float, default=15.0)
    ap.add_argument("--arm-timeout", type=float, default=30.0)
    ap.add_argument("--save", type=str, default="audit/forward_3m_shoot.json")
    ap.add_argument("--skip-reset", action="store_true", help="跳过 arm 复位")
    args = ap.parse_args()

    token = args.token or os.getenv("ERNIE_ACCESS_TOKEN") or os.getenv("MINIMAX_API_KEY")
    if not token:
        print("[fatal] no token: --token or ERNIE_ACCESS_TOKEN env", file=sys.stderr)
        sys.exit(2)

    settings_mod = __import__("main.settings", fromlist=["load_settings"])
    settings = settings_mod.load_settings()
    streamer_url = settings.streamer_url

    print(f"[ready] token={mask_token(token)} max_travel={args.max_travel}m "
          f"max_shots={args.max_shots} dy={args.dy}m tol_x={args.tol_x} "
          f"min_score={args.min_score} skip_reset={args.skip_reset}")
    print(f"[stream] {streamer_url}")

    client = RuntimeApiClient()
    for _ in range(60):
        h = client.get_health()
        s = h.get("state", {})
        if s.get("initialized") and not s.get("initializing"):
            break
        time.sleep(0.5)

    check_health(token, timeout=args.llm_timeout)

    # 1) 复位 arm.y = 0
    if not args.skip_reset:
        reset_arm_y(client, job_timeout=args.arm_timeout)
    else:
        print("[reset] skipped (--skip-reset)")

    init_odom = get_odom(client)
    init_arm = get_arm(client)
    print(f"[init] odom x={init_odom[0]:+.3f} y={init_odom[1]:+.3f} theta={init_odom[2]:+.3f}")
    print(f"[init] arm y={init_arm.get('y_m')} x={init_arm.get('x_m')} ref={init_arm.get('ref_encoder')}")

    found, skip_bene, skip_err = [], [], []
    traveled = 0.0
    state = "SCANNING"

    try:
        while shots_fired := len(found):
            if shots_fired >= args.max_shots:
                print(f"[done] ammo used {shots_fired}/{args.max_shots}")
                break
            if traveled >= args.max_travel - 1e-3:
                print(f"[done] traveled {traveled:.3f}m >= {args.max_travel}")
                break

            dets = get_detections(client)
            top = None
            for d in dets:
                if d.get("label") != "animal":
                    continue
                sc = float(d.get("score") or 0.0)
                if sc < args.min_score:
                    continue
                if top is None or sc > float(top.get("score") or 0.0):
                    top = d

            xc = float((top.get("bbox_norm") or {}).get("x_center", 0.0)) if top else None
            odom = get_odom(client)
            xc_str = f"{xc:+.3f}" if xc is not None else "none"
            sc_str = f"{top.get('score'):.2f}" if top else "none"
            print(f"[t={traveled:.3f}m] xc={xc_str} score={sc_str} state={state}")

            # 居中 + LLM 判
            if top is not None and xc is not None and abs(xc) <= args.tol_x and state == "SCANNING":
                frame = fetch_frame(streamer_url, timeout=5.0)
                if frame:
                    crop, _ = crop_bbox(frame, det_to_list(top), args.crop_padding)
                    if crop:
                        url = "data:image/jpeg;base64," + base64.b64encode(crop).decode()
                        verdict = call_vision(token, url, PEST_PROMPT, timeout=args.llm_timeout)
                        arm = get_arm(client)
                        res = verdict.get("result")
                        analysis = verdict.get("analysis", "")
                        print(f"  [judge] LLM result={res}  analysis={analysis[:120]}")

                        if res == 0:
                            # 害虫 -> 射击
                            t0 = time.time()
                            try:
                                car_call(client, "shooting", timeout=args.job_timeout)
                                shots_fired_now = len(found) + 1
                                print(f"  >>> [PEST #{shots_fired_now}/{args.max_shots}] "
                                      f"SHOT  odom_x={odom[0]:+.3f}m  took={time.time()-t0:.2f}s")
                            except Exception as e:
                                print(f"  [err] shooting failed: {e}", file=sys.stderr)
                            found.append({
                                "pest_idx": len(found) + 1,
                                "timestamp": time.time(),
                                "label": "pest",
                                "verdict": verdict,
                                "odom": odom,
                                "arm_state": {"y_m": arm.get("y_m"), "x_m": arm.get("x_m"),
                                              "ref_encoder": arm.get("ref_encoder")},
                                "traveled_m": traveled,
                            })
                            state = "RECORDED"
                        elif res == 1:
                            skip_bene.append({"xc": xc, "reason": analysis})
                            print(f"  [skip] beneficial, keep moving")
                        else:
                            skip_err.append({"xc": xc, "reason": analysis})
                            print(f"  [skip] LLM error, keep moving")
            elif top is None or (xc is not None and abs(xc) > args.unlock_x):
                if state == "RECORDED":
                    print("  [unlock] re-arming")
                state = "SCANNING"

            # 推进
            remaining = args.max_travel - traveled
            step = min(args.dy, remaining)
            if step <= 0:
                break
            safe(car_call, client, "move_for", [float(step), 0.0, 0.0], timeout=args.job_timeout)
            traveled += step
            time.sleep(0.15)

    except KeyboardInterrupt:
        print("\n[abort]")

    safe(car_call, client, "stop", timeout=args.job_timeout)

    out = Path(args.save)
    if not out.is_absolute():
        out = Path(__file__).resolve().parent / args.save
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "config": vars(args),
        "init_odom": init_odom,
        "init_arm": {"y_m": init_arm.get("y_m"), "x_m": init_arm.get("x_m"),
                     "ref_encoder": init_arm.get("ref_encoder")},
        "found": found,
        "skipped_beneficial": skip_bene,
        "skipped_llm_failed": skip_err,
        "traveled_m": round(traveled, 4),
        "complete_pests": len(found) >= args.max_shots,
        "complete_travel": traveled >= args.max_travel - 1e-3,
    }, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"\n========== RESULT ({len(found)}/{args.max_shots}) ==========")
    for e in found:
        v = e.get("verdict") or {}
        print(f"  PEST #{e['pest_idx']}  odom_x={e['odom'][0]:+.3f}m  "
              f"traveled={e['traveled_m']:.3f}m  "
              f"reason: {v.get('analysis', '')[:100]}")
    print(f"  skipped_beneficial={len(skip_bene)}  skipped_err={len(skip_err)}")
    print(f"  traveled={traveled:.3f}m  saved={out}")
    print("===========================================\n")


if __name__ == "__main__":
    main()