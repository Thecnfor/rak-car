#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/drive_then_analyze.py - clean rewrite v2

任务(2 阶段,简单直接):
  Phase 1 - DRIVE: 直线前进 1m,每段 dy 米抓一帧,YOLO 检测 animal,
            裁 bbox 存盘,按位置去重(同位置只留一次)。
  Phase 2 - ANALYZE: 串行调 ERNIE 给每只捕获判 PEST/BENEFICIAL + 物种。
                  同一物种 + 距离很近时,再调一次 ERNIE 确认"是否同一只",
                  **只有 ERNIE 确认"完全一样的同一只"才合并**。
                  同品种但长相不同的保留为不同个体。
                  → 输出按遇到顺序的 N 只虫子(target_unique 上限)。

Usage:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    $env:ERNIE_ACCESS_TOKEN = "..."
    python -m main.tasks.task333.drive_then_analyze
"""
from __future__ import annotations

import argparse
import base64
import datetime
import json
import math
import os
import sys
import time
from io import BytesIO
from pathlib import Path

import requests
from PIL import Image

from main.api_client import RuntimeApiClient
from main.misc.test_pest_llm_shoot import crop_bbox
from main.tasks.task333.llm_ernie import call_vision, mask_token


# 百度千帆 BCE 兼容 API 需要的常量(用于多图比较时的内联调用)
ERNIE_CHAT_URL = "https://aistudio.baidu.com/llm/lmapi/v3/chat/completions"
ERNIE_VL_MODEL = "ernie-4.5-turbo-vl"


def _sanitize_key(k: str) -> str:
    """剥 key 前后的空白 / \\r / \\n / \\t / \\0,Go net/http 服务端会拒非法的 header 字符。"""
    return (k or "").strip().strip("\r\n\t\0")


def concat_two_crops(jpeg_a: bytes, jpeg_b: bytes, sep_w: int = 20) -> bytes:
    """两张裁剪图左右拼成一张(中间白色分隔条),用于单次 LLM 多图对比。"""
    img_a = Image.open(BytesIO(jpeg_a)).convert("RGB")
    img_b = Image.open(BytesIO(jpeg_b)).convert("RGB")
    h = min(img_a.height, img_b.height)
    img_a = img_a.resize((max(1, int(img_a.width * h / img_a.height)), h))
    img_b = img_b.resize((max(1, int(img_b.width * h / img_b.height)), h))
    sep = Image.new("RGB", (sep_w, h), (255, 255, 255))
    canvas = Image.new("RGB", (img_a.width + sep_w + img_b.width, h), (0, 0, 0))
    canvas.paste(img_a, (0, 0))
    canvas.paste(sep, (img_a.width, 0))
    canvas.paste(img_b, (img_a.width + sep_w, 0))
    buf = BytesIO()
    canvas.save(buf, format="JPEG", quality=85)
    return buf.getvalue()


def ask_llm_same_individual(token: str, jpeg_a: bytes, jpeg_b: bytes,
                             timeout: float = 15.0):
    """问 ERNIE: 这两张图(A 左,B 右)是不是同一只虫子?

    返回:
        True  -> LLM 说"完全一样"
        False -> LLM 说"不同个体"(即使是同品种也不算)
        None  -> 调用失败/解析失败,保守起见按"不同个体"处理(不合并)
    """
    combined_jpg = concat_two_crops(jpeg_a, jpeg_b)
    b64 = base64.b64encode(combined_jpg).decode()
    url = f"data:image/jpeg;base64,{b64}"

    prompt = """这张图被白色竖条分成左右两半:左半是昆虫 A,右半是昆虫 B。
它们是**同一只虫子**(同一只动物,只是不同角度/距离/时间拍到)还是**两只不同的虫子**(即使是同品种也不算)?

严格按 JSON 输出(不要 Markdown 不要解释):
{"result": 0 或 1, "analysis": "<一句话中文>"}

- result=1: 同一只虫子(完全一样)
- result=0: 不同的两只(即使是同品种也不算)
- 只看个体特征(体型、颜色、姿态、斑纹、缺损等),品种相同不算"完全一样"
- 不确定时倾向 result=0(宁可错分,不可错合)
- 只输出 JSON。"""

    headers = {
        "Authorization": f"Bearer {_sanitize_key(token)}",
        "Content-Type": "application/json",
        "x-bce-date": datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ"),
    }
    body = {
        "model": ERNIE_VL_MODEL,
        "messages": [{
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url", "image_url": {"url": url}},
            ],
        }],
    }
    try:
        r = requests.post(ERNIE_CHAT_URL, headers=headers, json=body, timeout=timeout)
        if not r.ok:
            print(f"[warn] ask_llm_same_individual HTTP {r.status_code}: "
                  f"{r.text[:200]}", file=sys.stderr)
            return None
        content = r.json()["choices"][0]["message"]["content"]
        text = content.strip()
        if text.startswith("```"):
            lines = text.splitlines()
            if lines[0].lstrip().startswith("```"):
                lines = lines[1:]
            if lines and lines[-1].lstrip().startswith("```"):
                lines = lines[:-1]
            text = "\n".join(lines).strip()
        data = json.loads(text)
        if data.get("result") == 1:
            return True
        if data.get("result") == 0:
            return False
        return None
    except Exception as e:
        print(f"[warn] ask_llm_same_individual: {e}", file=sys.stderr)
        return None


# === ERNIE 多模态 prompt ===
# 要求:严格 JSON,result=0=害虫 / result=1=益虫 / result=-1=看不清
# analysis 里包含"是什么虫子" + 一句简短说明
PEST_PROMPT = """你是农田动物识别专家。
严格按 JSON 格式输出(不要 Markdown 不要解释):
{"result": 0 或 1, "analysis": "<一句话中文>"}

- result=0: 有害害虫(蝗虫、蚜虫、毛毛虫、象鼻虫、甲虫、蛞蝓、蜗牛、螨、蛾幼虫、蓟马、叶蝉)
- result=1: 有益动物(蜜蜂、瓢虫、蝴蝶、蚯蚓、螳螂、寄生蜂、吃害虫的蜘蛛)
- 如果看不清是什么动物: {"result": -1, "analysis": "<简短说明>"}
- analysis 里请明确写明动物名称(如"蝗虫""瓢虫")
- 只输出 JSON。
"""


def car_call(client, name, *args, timeout=30.0, **kwargs):
    """同步执行底盘动作,失败抛 RuntimeError。"""
    job = client.execute_car_action(name, *args, timeout=timeout, sync=False, **kwargs)
    done = client.wait_job(job["id"], timeout=timeout + 10)
    if done.get("status") != "succeeded":
        raise RuntimeError(f"car.{name} failed: {done.get('error')}")
    return done.get("result")


def get_animals(client, min_score):
    """从 task_feed 读所有 confidence >= min_score 的 animal 检测框。"""
    try:
        ts = (client.get_task_state() or {}).get("task_state") or {}
        return [
            d for d in (ts.get("detections") or [])
            if d.get("label") == "animal"
            and float(d.get("score") or 0.0) >= min_score
        ]
    except Exception:
        return []


def det_to_list(d):
    """det dict -> list 格式(crop_bbox 要求)。"""
    b = d.get("bbox_norm") or {}
    return [
        d.get("cls_id"), d.get("det_id"), d.get("label", ""),
        d.get("score", 0.0),
        b.get("x_center", 0.0), b.get("y_center", 0.0),
        b.get("width", 0.0), b.get("height", 0.0),
    ]


def fetch_frame(streamer_url, timeout=0.5):
    """从 streamer 拉 cam2 一帧 JPEG bytes。失败返回 None。"""
    try:
        r = requests.get(
            f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout
        )
        r.raise_for_status()
        return r.content
    except Exception:
        return None


def read_odom_x(client):
    """读底盘真实 odom X (米)。失败返回 NaN。"""
    try:
        odo = (client.get_runtime() or {}).get("runtime", {}).get("odometry") or [0, 0, 0]
        return float(odo[0])
    except Exception:
        return float("nan")


def crop_to_png_bytes(jpeg_bytes):
    """JPEG bytes -> PNG bytes(LLM 更好识别)。失败 fallback 回 JPEG。"""
    try:
        img = Image.open(BytesIO(jpeg_bytes)).convert("RGB")
        buf = BytesIO()
        img.save(buf, format="PNG")
        return buf.getvalue()
    except Exception:
        return jpeg_bytes


def main():
    ap = argparse.ArgumentParser(
        description="drive 1m straight + record scene + post-drive ERNIE classify")
    ap.add_argument("--token", default=None,
                    help="ERNIE access token (or env ERNIE_ACCESS_TOKEN)")
    ap.add_argument("--max-travel", type=float, default=1.0,
                    help="drive distance (m, default 1.0)")
    ap.add_argument("--dy", type=float, default=0.05,
                    help="每段前进米数 (default 0.05,更细颗粒)")
    ap.add_argument("--max-captures", type=int, default=20,
                    help="Phase 1 最多记多少只(防止过度采集)")
    ap.add_argument("--target-unique", type=int, default=4,
                    help="最终要几只虫子(default 4)")
    ap.add_argument("--min-score", type=float, default=0.50,
                    help="YOLO 置信度阈值")
    ap.add_argument("--merge-dist", type=float, default=0.20,
                    help="位置去重阈值(归一化坐标,default 0.20)")
    ap.add_argument("--crop-padding", type=float, default=0.20,
                    help="bbox 裁剪 padding")
    ap.add_argument("--llm-timeout", type=float, default=15.0,
                    help="单次 ERNIE 调用超时 (s)")
    ap.add_argument("--streamer", default=None,
                    help="streamer URL (default settings.streamer_url)")
    ap.add_argument("--save", default="audit/drive_then_analyze.json",
                    help="结果落盘路径")
    ap.add_argument("--save-crops", default="audit/crops/",
                    help="裁剪图保存目录")
    args = ap.parse_args()

    # ---- token ----
    token = args.token or os.getenv("ERNIE_ACCESS_TOKEN")
    if not token:
        print("[fatal] no token: set ERNIE_ACCESS_TOKEN env or pass --token",
              file=sys.stderr)
        sys.exit(2)

    # ---- streamer URL ----
    import main.settings as settings_mod
    settings = settings_mod.load_settings()
    streamer_url = args.streamer or settings.streamer_url

    # ---- 等 runtime ready ----
    client = RuntimeApiClient()
    for _ in range(60):
        h = client.get_health()
        s = h.get("state", {})
        if s.get("initialized") and not s.get("initializing"):
            break
        time.sleep(0.5)

    print(f"[ready] token={mask_token(token)} max_travel={args.max_travel}m "
          f"dy={args.dy}m target_unique={args.target_unique} "
          f"merge_dist={args.merge_dist}", flush=True)

    # ---- 裁剪目录 ----
    crops_dir = Path(__file__).resolve().parent / args.save_crops
    crops_dir.mkdir(parents=True, exist_ok=True)

    # ============================================================
    #  Phase 1: DRIVE 1m + capture
    # ============================================================
    print(f"\n========== PHASE 1: DRIVE {args.max_travel}m + capture ==========",
          flush=True)

    init_x = read_odom_x(client)
    print(f"[init odom] x={init_x:+.3f}m", flush=True)

    captures = []  # {order, xc, yc, score, crop_bytes, traveled_m, seg_idx}
    seg_idx = 0

    while seg_idx * args.dy < args.max_travel - 1e-3:
        seg_idx += 1
        seg_target_x = init_x + min(seg_idx * args.dy, args.max_travel)

        print(f"[drive seg {seg_idx}] -> move_to_position(x={seg_target_x:+.3f}m)",
              flush=True)
        try:
            car_call(client, "move_to_position",
                     [float(seg_target_x), 0.0, 0.0], timeout=30)
        except Exception as e:
            print(f"[err] move_to_position failed: {e}", file=sys.stderr)
            break
        time.sleep(0.15)

        # 读当前位置(算 traveled)
        post_x = read_odom_x(client)
        if not math.isnan(post_x):
            traveled = abs(post_x - init_x)
        else:
            traveled = seg_idx * args.dy  # fallback
        print(f"           -> x={post_x:+.3f}m, traveled={traveled:.3f}m",
              flush=True)

        # 抓帧 + 检测
        animals = get_animals(client, args.min_score)
        if not animals:
            continue
        frame = fetch_frame(streamer_url, timeout=0.5)
        if frame is None:
            continue

        for det in animals:
            if len(captures) >= args.max_captures:
                break
            b = det.get("bbox_norm") or {}
            xc = float(b.get("x_center", 0.0))
            yc = float(b.get("y_center", 0.0))
            score = float(det.get("score") or 0.0)

            # 位置去重(同位置只留一次)
            too_close = False
            for prev in captures:
                dist = math.hypot(prev["xc"] - xc, prev["yc"] - yc)
                if dist < args.merge_dist:
                    too_close = True
                    break
            if too_close:
                continue

            crop, _ = crop_bbox(frame, det_to_list(det), args.crop_padding)
            if not crop:
                continue

            order = len(captures) + 1
            captures.append({
                "order": order,
                "xc": xc, "yc": yc, "score": score,
                "traveled_m": traveled,
                "seg_idx": seg_idx,
                "crop_bytes": crop,
            })
            crop_path = crops_dir / f"insect_{order:02d}_t{traveled:.2f}m.jpg"
            crop_path.write_bytes(crop)
            print(f"  [capture {order}/{args.max_captures}] "
                  f"at traveled={traveled:.2f}m, xc={xc:+.2f}, yc={yc:+.2f}, "
                  f"score={score:.2f}", flush=True)

    # 停车
    safe(car_call, client, "stop", timeout=10)
    final_x = read_odom_x(client)
    final_traveled = abs(final_x - init_x) if not math.isnan(final_x) else 0.0
    print(f"\n[drive] DONE. traveled={final_traveled:.3f}m, "
          f"captured {len(captures)} insects (after position-dedup)", flush=True)

    # ============================================================
    #  Phase 2: ANALYZE (slow ERNIE classify, 1 只 1 次)
    # ============================================================
    print(f"\n========== PHASE 2: ANALYZE (slow ERNIE classify) ==========",
          flush=True)
    print(f"  共 {len(captures)} 个位置去重后的捕获,逐个调 ERNIE 判 PEST/BENEFICIAL",
          flush=True)

    results = []  # deduped unique list(每个元素可能合并了多次同只再捕获)
    skipped_unidentified = 0
    LLM_DEDUP_DIST = 0.30  # 同物种 + 位置差 < 这个 → 问 ERNIE 是否同一只

    for cap_idx, cap in enumerate(captures):
        # 防御:cap 缺 crop_bytes 直接跳过(理论上 Phase 1 已保证,但保险一下)
        if "crop_bytes" not in cap or not cap["crop_bytes"]:
            skipped_unidentified += 1
            print(f"  [{cap_idx + 1}/{len(captures)}] 第 {cap.get('order', '?')} 只: "
                  f"缺 crop_bytes,跳过", flush=True)
            continue

        png_bytes = crop_to_png_bytes(cap["crop_bytes"])
        url = "data:image/png;base64," + base64.b64encode(png_bytes).decode()
        t0 = time.time()
        verdict = call_vision(token, url, PEST_PROMPT, timeout=args.llm_timeout)
        dt_ms = (time.time() - t0) * 1000

        res = verdict.get("result")
        analysis = (verdict.get("analysis") or "").strip()

        # 用 if/else 而不是 continue,让"识别不到"分支完全独立,
        # 绝对不会进入下面的 dedup 块。
        if res not in (0, 1):
            # 识别不到(result != 0/1,如 -1 / None / 解析失败)→ 当做没有,直接丢掉
            skipped_unidentified += 1
            print(f"  [{cap_idx + 1}/{len(captures)}] 第 {cap['order']} 只: "
                  f"识别不到,跳过  ({dt_ms:.0f}ms)", flush=True)
            print(f"          {analysis[:120] or '(无原因)'}", flush=True)
        else:
            # 成功识别 → 分类 + dedup
            label_cn, label_en = ("害虫", "PEST") if res == 0 else ("益虫", "BENEFICIAL")

            # 保守 dedup:只在 (同物种) AND (位置很近) 时才问 ERNIE"是不是同一只"
            # 不同物种或位置远的 → 不合并(直接当新个体)
            # 同物种 + 位置近:问 LLM;只有 LLM 说"完全一样"才合并;
            # 同物种 + 位置近但 LLM 说"不同个体" → 保留为 2 只
            matched_idx = None
            for j, u in enumerate(results):
                if u["label_cn"] != label_cn:
                    continue
                if abs(u["xc"] - cap["xc"]) >= LLM_DEDUP_DIST:
                    continue
                if abs(u["yc"] - cap["yc"]) >= LLM_DEDUP_DIST:
                    continue
                same = ask_llm_same_individual(
                    token, u["crop_bytes"], cap["crop_bytes"],
                    timeout=args.llm_timeout,
                )
                if same is True:
                    matched_idx = j
                    break
                # same=False 或 None → 不合并,继续找下一个

            if matched_idx is not None:
                # 合并到已确认的 unique
                results[matched_idx]["merged_orders"].append(cap["order"])
                results[matched_idx]["merge_llm_count"] = (
                    results[matched_idx].get("merge_llm_count", 0) + 1
                )
                print(f"  [{cap_idx + 1}/{len(captures)}] 第 {cap['order']} 只: "
                      f"{label_cn}({label_en})  {dt_ms:.0f}ms  "
                      f"[合并 -> #{results[matched_idx]['order']} 同只再捕获]",
                      flush=True)
                print(f"          {analysis[:120]}", flush=True)
            else:
                # 新 unique
                results.append({
                    "order": cap["order"],
                    "xc": cap["xc"], "yc": cap["yc"], "score": cap["score"],
                    "traveled_m": cap["traveled_m"],
                    "label_cn": label_cn, "label_en": label_en,
                    "analysis": analysis,
                    "llm_ms": int(dt_ms),
                    "merged_orders": [cap["order"]],
                    "merge_llm_count": 0,
                })
                print(f"  [{cap_idx + 1}/{len(captures)}] 第 {cap['order']} 只: "
                      f"{label_cn}({label_en})  {dt_ms:.0f}ms  "
                      f"[新个体]",
                      flush=True)
                print(f"          {analysis[:120]}", flush=True)

    # ============================================================
    #  最终输出:按遇到顺序的 N 只(target_unique 上限)
    # ============================================================
    final = results[:args.target_unique]
    print("\n" + "=" * 64, flush=True)
    if skipped_unidentified:
        print(f"  (Phase 2 识别失败 {skipped_unidentified} 只,已丢弃)",
              flush=True)
    total_merge = sum(len(r["merged_orders"]) - 1 for r in final)
    if total_merge:
        print(f"  (LLM dedup 合并了 {total_merge} 次同只再捕获,"
              f"基于 ERNIE '完全一样' 判定)", flush=True)
    print(f"========== 最终 {len(final)} 只虫子(按遇到顺序) ==========", flush=True)
    print("=" * 64, flush=True)
    for i, r in enumerate(final, 1):
        label = f"{r['label_cn']}({r['label_en']})"
        print(f"  第 {i} 只: {label}  "
              f"在 traveled={r['traveled_m']:.2f}m, xc={r['xc']:+.2f}, "
              f"yc={r['yc']:+.2f}", flush=True)
        print(f"          {r['analysis']}", flush=True)
        if len(r["merged_orders"]) > 1:
            print(f"          (合并 #{', #'.join(map(str, r['merged_orders']))} "
                  f"共 {len(r['merged_orders'])} 次同只再捕获)", flush=True)
    print("=" * 64, flush=True)

    # 落盘
    out = Path(args.save)
    if not out.is_absolute():
        out = Path(__file__).resolve().parent / args.save
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({
        "config": vars(args),
        "traveled_m": round(final_traveled, 3),
        "raw_capture_count": len(captures),
        "final_count": len(final),
        "results": [
            {k: v for k, v in r.items() if k != "crop_bytes"}
            for r in final
        ],
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\n[done] saved to {out}", flush=True)
    print(f"[done] crops saved to {crops_dir}", flush=True)


def safe(fn, *args, **kwargs):
    try:
        return fn(*args, **kwargs)
    except Exception as e:
        print(f"[warn] {fn.__name__}: {e}", file=sys.stderr)
        return None


if __name__ == "__main__":
    main()