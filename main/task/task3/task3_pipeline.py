#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Task 3 pipeline: record four boards, judge them after driving, then shoot.

The recognition phase drives slowly until all target crops are recorded,
judging each one inline (with front/back recapture on unknown). The arm is
NOT adjusted before recognition (2026-08-12 user directive). After the car
stops, the operator moves the car to the shooting area; Enter starts the
existing shooting algorithm after the arm is moved to its shooting pose.

Run from the repository root:
    python -m main.task.task3.task3_pipeline --token <ERNIE_TOKEN>
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Optional

import requests
import yaml

from main.api_client import RuntimeApiClient
from main.task.task3.arm_poses import SHOOTING_ARM, arm_at_pose
from main.task.task3.llm_ernie import call_vision, check_health, load_token, mask_token


RECOGNITION_PROMPT = """Analyze the animal in this image as an agricultural expert.
Return STRICT JSON only, with no markdown:
{"name":"animal species in English or Chinese","result":0,"analysis":"short reason"}
result=0 means harmful crop pest; result=1 means beneficial animal.
Classify the pictured animal, including cartoon or printed animal cards.
Examples of pests: locust, aphid, caterpillar, beetle, snail, mite.
Examples of beneficial animals: bee, ladybug, butterfly, earthworm, spider.
If uncertain, still choose the most likely result and explain briefly.
"""


def _load_pest_prompt() -> str:
    """读 llm_config.yml 的 pest_detect.system_prompt (详细版) 替换简单硬编码提示词。

    2026-08-09: 旧简单 prompt 没有"图里不是动物 → result=1"规则, LLM 把 YOLO 误检
    (空图 / 蓝色结构物) 默认判成害虫 → 假 pest (#1/#2 实机案例)。详细版明确
    "如果图片中不是动物或根本无法识别, 返回 result=1"。读不到/解析失败回退
    RECOGNITION_PROMPT 保底。
    """
    try:
        with open(Path(__file__).resolve().parent / "llm_config.yml",
                  encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        prompt = (cfg.get("pest_detect") or {}).get("system_prompt")
        if prompt and str(prompt).strip():
            print(f"[ready] 害虫判定用 llm_config.yml 详细提示词 "
                  f"({len(str(prompt))} 字符)", flush=True)
            return str(prompt)
        print("[warn] llm_config.yml 缺 pest_detect.system_prompt, 回退硬编码提示词",
              file=sys.stderr)
    except Exception as exc:
        print(f"[warn] 读 llm_config.yml 失败, 回退硬编码提示词: {exc}",
              file=sys.stderr)
    return RECOGNITION_PROMPT


PEST_PROMPT = _load_pest_prompt()

DEFAULT_TARGET_COUNT = 4
DEFAULT_CREEP_SPEED = 0.18          # m/s, 识别段 creep 速度 (2026-08-04 现场: 0.05→0.18)
DEFAULT_MIN_SCORE = 0.40
DEFAULT_CENTER_TOL = 0.15
DEFAULT_MIN_GAP = 0.16          # 卡片中心距 16cm → 去重窗口需覆盖一张卡的间距
DEFAULT_CLASSIFY_WORKERS = 2
DEFAULT_TARGET_SPACING_M = 0.16       # 卡片中心距 16cm (2026-08-09: 改回 16; 补录推进 + 超程保护都用它)
DEFAULT_TARGET_SETTLE_S = 0.15
# 前后微调识别 (2026-08-12 用户需求: 识别不出目标时前后微调, 直到识别出来再继续 16cm)
DEFAULT_RECOG_SEARCH_STEP_M = 0.04        # 微调步长 4cm
DEFAULT_RECOG_SEARCH_BACK_MAX_M = 0.12    # 向后微调上限 12cm (over-shot 常见, 先退)
DEFAULT_RECOG_SEARCH_FWD_MAX_M = 0.16     # 向前微调上限 16cm
# LLM 判定 result=None (判不出害虫/益虫) 时前后微调重拍 (2026-08-12 用户需求)
MAX_LLM_RECAPTURE = 2                     # 单张卡判定失败最大重拍次数


def step_for_xc(xc, base=DEFAULT_TARGET_SPACING_M, adjust=0.02, tol=DEFAULT_CENTER_TOL):
    """按上一个目标的横向位置决定下一步长度 (最后一个目标除外).

    xc 为 [-1,1] 中心化坐标 (正=目标偏右, 负=目标偏左):
      正中 (|xc|<=tol) -> base        (16cm)
      偏右 (xc>tol)    -> base+adjust  (18cm, 多走 2cm)
      偏左 (xc<-tol)   -> base-adjust  (14cm, 少走 2cm)
    """
    if xc > tol:
        return base + adjust
    if xc < -tol:
        return base - adjust
    return base


def read_detections(client):
    try:
        state = (client.get_task_state() or {}).get("task_state") or {}
        return list(state.get("detections") or [])
    except Exception as exc:
        print(f"[warn] detection read failed: {exc}", file=sys.stderr)
        return []


def bbox(det):
    return det.get("bbox_norm") or {}


def animal_center(det):
    return float(bbox(det).get("x_center", 0.0))


def det_to_list(det):
    b = bbox(det)
    return [
        det.get("cls_id"), det.get("det_id"), det.get("label", ""),
        det.get("score", 0.0), b.get("x_center", 0.0), b.get("y_center", 0.0),
        b.get("width", 0.0), b.get("height", 0.0),
    ]


def fetch_frame(streamer_url, timeout=5.0) -> Optional[bytes]:
    try:
        response = requests.get(
            f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout
        )
        response.raise_for_status()
        return response.content
    except Exception as exc:
        print(f"[warn] frame fetch failed: {exc}", file=sys.stderr)
        return None


def car_call(client, name, *args, timeout=20.0):
    job = client.execute_car_action(name, *args, timeout=timeout, sync=False)
    done = client.wait_job(job["id"], timeout=timeout + 10.0)
    if done.get("status") != "succeeded":
        raise RuntimeError(f"car.{name} failed: {done.get('error')}")
    return done.get("result")


def safe_car_call(client, name, *args, timeout=20.0):
    try:
        return car_call(client, name, *args, timeout=timeout)
    except Exception as exc:
        print(f"[warn] car.{name}: {exc}", file=sys.stderr)
        return None


def run_arm_pose(args, label, pose, client=None):
    # 2026-08-09: orchestrator 已在途中摆好姿态 → 已在位则跳过, 省任务点串行等待.
    if client is not None and not args.dry_run and arm_at_pose(client, pose):
        print(f"[arm] {label}: 已在目标姿态 {pose}, 跳过摆臂", flush=True)
        return 0
    command = [sys.executable, "-m", "main.task.task3.arm_seq_v9",
               "--y1", pose[0], "--y2", pose[1], "--x", pose[2],
               "--arm-angle", pose[3], "--hand-angle", pose[4]]
    print(f"[arm] {label}: {' '.join(command)}", flush=True)
    if args.dry_run:
        return 0
    return subprocess.run(command, check=False).returncode


def capture_target(streamer_url, det, crop_padding, output_dir, number, frame=None):
    """按 bbox 裁剪最新帧（或传入的 frame）并保存 target_%02d.jpg。"""
    from main.misc.test_pest_llm_shoot import crop_bbox
    if frame is None:
        frame = fetch_frame(streamer_url)
    if not frame:
        return None
    crop, _ = crop_bbox(frame, det_to_list(det), crop_padding)
    if not crop:
        return None
    output_dir.mkdir(parents=True, exist_ok=True)
    image_path = output_dir / f"target_{number:02d}.jpg"
    image_path.write_bytes(crop)
    return image_path


def classify_target(token, image_path, timeout):
    if not image_path:
        return {"name": "unknown", "result": None, "analysis": "target image unavailable"}
    try:
        image_bytes = Path(image_path).read_bytes()
    except OSError:
        return {"name": "unknown", "result": None, "analysis": "target image unavailable"}
    image_url = "data:image/jpeg;base64," + base64.b64encode(image_bytes).decode("ascii")
    verdict = call_vision(token, image_url, PEST_PROMPT, timeout=timeout)
    result = verdict.get("result")
    try:
        result = int(result) if result is not None else None
    except (TypeError, ValueError):
        result = None
    return {
        "name": str(verdict.get("name") or verdict.get("species") or "unknown"),
        "result": result if result in (0, 1) else None,
        "analysis": str(verdict.get("analysis") or ""),
    }


def _classify_records(token, records, timeout, workers):
    if not records:
        return []
    worker_count = max(1, min(int(workers), len(records)))

    def classify_one(index, record):
        # 错开发起时刻，避免并发请求同时打到 ERNIE 触发限流
        time.sleep(index * 0.3)
        return classify_target(token, record["image_path"], timeout)

    with ThreadPoolExecutor(max_workers=worker_count, thread_name_prefix="task3-llm") as pool:
        futures = [
            pool.submit(classify_one, index, record)
            for index, record in enumerate(records)
        ]
        verdicts = [future.result() for future in futures]

    judged = []
    for record, verdict in zip(records, verdicts):
        entry = dict(record)
        entry.pop("detection", None)
        result = verdict["result"]
        entry.update({
            "species": verdict["name"],
            "classification": "pest" if result == 0 else "beneficial" if result == 1 else "unknown",
            "result": result,
            "analysis": verdict["analysis"],
        })
        judged.append(entry)
        print(f"  -> target #{entry['number']}: {entry['species']} / "
              f"{entry['classification']} | {entry['analysis']}", flush=True)
    return judged


def judge_records(token, records, llm_timeout):
    """逐张对已记录目标做 LLM 判定, 返回带 species/result/analysis 的 judged 列表.

    从 recognize_phase 拆出 (2026-08-07): orchestrator 编排时判定放后台线程跑,
    识别段存 pending json 后立即返回, 不阻塞车. 手动全流程仍在本进程同步判定.
    """
    judged = []
    for record in records:
        verdict = classify_target(token, record["image_path"], llm_timeout)
        entry = dict(record)
        entry.pop("detection", None)
        entry.update({
            "species": verdict["name"],
            "classification": "pest" if verdict["result"] == 0 else "beneficial" if verdict["result"] == 1 else "unknown",
            "result": verdict["result"],
            "analysis": verdict["analysis"],
        })
        judged.append(entry)
        print(f"  -> target #{entry['number']}: {entry['species']} / "
              f"{entry['classification']} | {entry['analysis']}", flush=True)
    return judged


def read_traveled(client):
    """读取 odom 累计路径长 (m)，失败返回 None。"""
    try:
        state = (client.get_odom_state() or {}).get("odom_state") or {}
        distance = state.get("distance")
        return float(distance) if distance is not None else None
    except Exception:
        return None


def search_front_back(detect_fn, move_fn, *,
                      step_m=DEFAULT_RECOG_SEARCH_STEP_M,
                      back_max_m=DEFAULT_RECOG_SEARCH_BACK_MAX_M,
                      fwd_max_m=DEFAULT_RECOG_SEARCH_FWD_MAX_M,
                      label="目标"):
    """识别不出目标时的前后微调搜索 (2026-08-12 用户需求).

    顺序: 当前位置 → 后退(≤back_max_m) → 前进(≤back_max_m+fwd_max_m, 穿过原点)。
    每步 move_fn(distance) 后用 detect_fn() 看是否识别到完整目标;
    第一个识别到即返回 (net_offset_m 为相对起点的净位移, 后退负/前进正);
    全程没识别到 → 回到原点, 返回 (None, 0.0) (不改变车位)。

    Args:
        detect_fn: () -> det|None, 需自带「稳定确认」语义 (调用方用 make_latch+wait 包一层).
        move_fn:   (distance_m) -> None, 前后移动 (负=后退), 走 move_for.
    Returns:
        (det|None, net_offset_m)
    """
    offset = 0.0

    def try_detect():
        det = detect_fn()
        if det is not None:
            print(f"  [recog-search] {label} 前后微调 {offset:+.2f}m 后识别到 "
                  f"(xc={animal_center(det):+.3f})", flush=True)
            return det
        return None

    # Phase 1: 优先后退 (over-shot 常见: 目标可能刚被越过)
    while offset - step_m >= -back_max_m - 1e-9:
        move_fn(-step_m)
        offset -= step_m
        det = try_detect()
        if det is not None:
            return det, round(offset, 3)

    # Phase 2: 前进 (从后退终点穿过原点向前, 覆盖「目标偏前」的情况)
    while offset + step_m <= fwd_max_m + 1e-9:
        move_fn(+step_m)
        offset += step_m
        det = try_detect()
        if det is not None:
            return det, round(offset, 3)

    # 兜底: 前后都没找到 → 回到原点
    if abs(offset) >= step_m * 0.5:
        print(f"  [recog-search] {label} 前后 {back_max_m*100:.0f}/{fwd_max_m*100:.0f}"
              f"cm 均未识别到, 回到原点", flush=True)
        move_fn(-offset)
    return None, 0.0


def judge_and_recapture(record, *, classify_fn, search_fn, recapture_fn,
                        max_retries=MAX_LLM_RECAPTURE, label="目标"):
    """内联 LLM 判定当前卡; **result=None (判不出害虫/益虫) 时前后微调重拍** (2026-08-12).

    用户需求: 卡识别到了但 LLM 判不出是什么 → 前后微调重拍, 最多 max_retries 次;
    重拍后仍判不出 → 保留原结果 (result=None, 不射该卡)。

    Args:
        record:     目标记录 dict (image_path/detection/xc/...).
        classify_fn: (record) -> verdict {name, result, analysis}, result∈{0,1,None}.
        search_fn:   () -> (det|None, net_offset_m): 前后微调找目标.
        recapture_fn:(det) -> None: 用新检测重拍并就地更新 record 的 image/detection.
        max_retries: 判定失败重拍上限.
        label:       日志用 (如 "目标 #3").
    Returns:
        verdict: 最终判定 (可能仍为 result=None).
    """
    verdict = classify_fn(record)
    retries = 0
    while verdict.get("result") is None and retries < max_retries:
        retries += 1
        print(f"[warn] {label} LLM 判定 result=None, 前后微调重拍 "
              f"(第 {retries}/{max_retries} 次)", file=sys.stderr, flush=True)
        try:
            det, _net = search_fn()
        except Exception as exc:
            print(f"[warn] {label} 重拍移动失败: {exc}, 保留原图",
                  file=sys.stderr, flush=True)
            break
        if det is None:
            print(f"[warn] {label} 重拍未找到目标, 保留原图",
                  file=sys.stderr, flush=True)
            break
        recapture_fn(det)
        verdict = classify_fn(record)
    record["species"] = str(verdict.get("name") or "unknown")
    record["result"] = verdict.get("result")
    record["analysis"] = str(verdict.get("analysis") or "")
    record["classification"] = ("pest" if verdict.get("result") == 0
                                else "beneficial" if verdict.get("result") == 1
                                else "unknown")
    print(f"  -> {label}: {record['species']} / {record['classification']} "
          f"(recapture={retries})", flush=True)
    return verdict


def recognize_phase_fixed_slots(client, args, token, streamer_url, output_dir,
                                defer_judge: bool = False):
    """首卡定位后按固定卡间距逐个记录，避免同一张卡重复计数。"""
    records = []
    period = max(args.poll_interval, 0.05)
    # 后续卡锁存器要求的最小稳定帧数: 2→4 (2026-08-09 用户要求, 卡要连续 4 帧稳定)
    # 公式含义: settle 秒内能采到的样本数; max(4, ...) 保证至少 4 帧, settle 调大可更多.
    # (行驶中首卡用 required_samples=1 触发停车, 停车后再用本稳定帧数确认 —— 2026-08-09)
    min_samples = max(4, int(args.target_settle_s / period) + 1)
    first_window = max(args.center_tol, 0.30)
    slot_window = max(args.center_tol * 2.0, 0.35)
    start_odom = read_traveled(client)
    nominal_travel = 0.0

    class DetectionReader:
        def __init__(self):
            self._lock = threading.Lock()
            self._stop = threading.Event()
            self._latest = []
            self._cache_updated = None
            self._last_direct = 0.0
            self._thread = threading.Thread(
                target=self._run, name="task3-detection-reader", daemon=True,
            )

        def start(self):
            self._thread.start()

        def stop(self):
            self._stop.set()

        def read(self):
            with self._lock:
                return [dict(det) for det in self._latest]

        def _publish(self, detections):
            with self._lock:
                self._latest = list(detections or [])

        def _run(self):
            direct_client = RuntimeApiClient()
            warned = False
            while not self._stop.is_set():
                try:
                    state = (client.get_task_state() or {}).get("task_state") or {}
                    updated = state.get("updated_at")
                    error = state.get("last_error")
                    if updated != self._cache_updated and not error:
                        self._cache_updated = updated
                        self._publish(state.get("detections"))
                        warned = False
                    # 2026-08-09: task_feed 缓存不新鲜(识别起点/补录时 feed 可能未刷新)
                    # 或报错 → 主动跑一次同步推理, 避免"车已在卡面前却识别不到".
                    now = time.monotonic()
                    try:
                        age = time.time() - float(updated) if updated is not None else float("inf")
                    except (TypeError, ValueError):
                        age = float("inf")
                    stale = (error or self._cache_updated is None or age > 0.6)
                    if stale and now - self._last_direct >= 0.45:
                        self._last_direct = now
                        result = direct_client.request_vision_task(timeout=3.0)
                        if result.get("ok"):
                            self._publish(result.get("detections"))
                            if warned:
                                print("[detect] direct task detection recovered", flush=True)
                                warned = False
                    elif error and not warned:
                        print(f"[detect] task_feed error: {error}; using direct detection",
                              file=sys.stderr, flush=True)
                        warned = True
                except Exception:
                    pass
                self._stop.wait(0.05)

    detector = DetectionReader()
    detector.start()

    def eligible(det):
        return (det.get("label") == "animal"
                and float(det.get("score") or 0.0) >= args.min_score)

    def traveled_now():
        distance = read_traveled(client)
        if distance is None or start_odom is None or distance < start_odom:
            return nominal_travel
        return max(nominal_travel, distance - start_odom)

    def is_complete(det, edge_tol=0.02):
        """目标"完整入画": bbox 未被画面边缘截断 (完整目标, 不需要在视野正中央).

        bbox_norm 是 [-1,1] 中心化坐标 (runtime 实测 norm=(px/dim)*2-1,
        width_norm=(px_w/dim)*2), 画面边缘 = ±1, 不是 [0,1]!
        """
        b = bbox(det)
        try:
            w = float(b.get("width") or 0.0)
            h = float(b.get("height") or 0.0)
            xc = float(b.get("x_center") or 0.0)
            yc = float(b.get("y_center") or 0.0)
        except (TypeError, ValueError):
            return False
        if w <= 0.0 or h <= 0.0:
            return False
        return (xc - w / 2.0 >= -1.0 - edge_tol and xc + w / 2.0 <= 1.0 + edge_tol
                and yc - h / 2.0 >= -1.0 - edge_tol and yc + h / 2.0 <= 1.0 + edge_tol)

    def centered(window):
        """返回最接近画面中央的合格目标; window=None 时只看"完整入画", 不限 xc."""
        detections = [det for det in detector.read() if eligible(det)]
        if window is None:
            candidates = [det for det in detections if is_complete(det)]
        else:
            candidates = [det for det in detections
                          if is_complete(det)
                          and abs(animal_center(det)) <= window]
        return min(candidates, key=lambda det: abs(animal_center(det))) \
            if candidates else None

    def make_latch(window, required_samples=min_samples, sample_period=period):
        state = {"last": 0.0, "streak": 0, "det": None, "ready": False}

        def sample(force=False):
            now = time.monotonic()
            if not force and now - state["last"] < sample_period:
                return state["ready"]
            state["last"] = now
            det = centered(window)
            if det is None:
                return state["ready"]
            previous = state["det"]
            stable = previous is not None and abs(
                animal_center(det) - animal_center(previous)
            ) <= 0.12
            state["streak"] = state["streak"] + 1 if stable else 1
            state["det"] = dict(det)
            state["ready"] = state["streak"] >= required_samples
            if state["ready"]:
                print(f"  [center] xc={animal_center(det):+.3f} "
                      f"streak={state['streak']}/{required_samples}", flush=True)
            return state["ready"]

        return state, sample

    def save_slot(det, number):
        image_path = None
        if not args.dry_run:
            image_path = capture_target(
                streamer_url, det, args.crop_padding, output_dir, number,
            )
            if image_path is None:
                time.sleep(0.10)
                image_path = capture_target(
                    streamer_url, det, args.crop_padding, output_dir, number,
                )
        live = traveled_now()
        records.append({
            "number": number,
            "image_path": str(image_path) if image_path else None,
            "detection": dict(det),
            "xc": animal_center(det),
            "yc": float(bbox(det).get("y_center", 0.0)),
            "score": float(det.get("score") or 0.0),
            "traveled_m": round(live, 4),
            "timestamp": time.time(),
        })
        print(f"  [record] target #{number} xc={animal_center(det):+.3f} "
              f"at {live:.2f}m image={'saved' if image_path else 'missing'}",
              flush=True)

    def judge_and_recapture_slot(record, number):
        """内联 LLM 判定当前 slot; result=None 时前后微调重拍 (2026-08-12 用户需求)."""
        def classify(rec):
            return classify_target(token, rec.get("image_path"), args.llm_timeout)

        def search():
            nonlocal nominal_travel
            det, net = search_front_back(
                detect_fn=lambda: wait_for_target(
                    make_latch(None, required_samples=2), args.slot_wait_s),
                move_fn=move_forward_fallback,
                step_m=args.recog_search_step_m,
                back_max_m=args.recog_search_back_max_m,
                fwd_max_m=args.recog_search_fwd_max_m,
                label=f"slot #{number} LLM 重拍",
            )
            nominal_travel += max(0.0, net)
            return det, net

        def recapture(det):
            image_path = capture_target(
                streamer_url, det, args.crop_padding, output_dir, number,
            )
            if image_path is None:
                time.sleep(0.10)
                image_path = capture_target(
                    streamer_url, det, args.crop_padding, output_dir, number,
                )
            record["image_path"] = (str(image_path) if image_path
                                    else record.get("image_path"))
            record["detection"] = dict(det)
            record["xc"] = animal_center(det)
            record["yc"] = float(bbox(det).get("y_center", 0.0))
            record["score"] = float(det.get("score") or 0.0)

        judge_and_recapture(record, classify_fn=classify, search_fn=search,
                            recapture_fn=recapture,
                            max_retries=args.max_llm_recapture,
                            label=f"目标 #{number}")

    def save_and_judge(det, number):
        """记录一张卡; 非 defer-judge 且有 token 时当场 LLM 判定 + result=None 重拍."""
        save_slot(det, number)
        if not defer_judge and token and not args.dry_run:
            try:
                judge_and_recapture_slot(records[-1], number)
            except Exception as exc:
                # LLM/网络故障不能打断识别 → 留 result=None, 末尾兜底判定.
                print(f"[warn] 目标 #{number} 内联判定异常: {exc}",
                      file=sys.stderr, flush=True)

    def move_forward_fallback(distance):
        print(f"  [drive] move_for fallback {distance:+.2f}m", flush=True)
        job = client.execute_car_action(
            "move_for", [distance, 0.0, 0.0],
            timeout=args.job_timeout, sync=False,
            max_velocities=[args.creep_speed, args.creep_speed, 3.14159 / 3],
        )
        done = client.wait_job(job["id"], timeout=args.job_timeout + 10.0)
        if done.get("status") != "succeeded":
            raise RuntimeError(f"car.move_for failed: {done.get('error')}")

    def move_along_lane(distance, stop_when=None, stopped=None, vx=None):
        """沿车道推进 distance 米; 返回实际里程增量 (m, odom 不可用按请求距离估算)."""
        if args.dry_run:
            return distance
        from main.chassis.config import LANE_FOLLOW
        from main.chassis.controllers import move_along_lane as lane_move
        eff_vx = vx if vx is not None else args.creep_speed
        print(f"  [drive] move_along_lane +{distance:.2f}m @{eff_vx:.2f} m/s",
              flush=True)
        before = read_traveled(client)
        try:
            lane_state = client.get("/v1/vision/lane/state") or {}
            lane_error = lane_state.get("last_error")
        except Exception as exc:
            lane_error = f"lane state unavailable: {exc}"
        if lane_error:
            print(f"  [warn] lane feed unavailable: {lane_error}",
                  file=sys.stderr, flush=True)
            move_forward_fallback(distance)
            after = read_traveled(client)
        else:
            try:
                lane_move(
                    vx=eff_vx,
                    distance_m=distance,
                    profile=LANE_FOLLOW.tuned(watchdog_ms=None),
                    max_seconds=max(
                        5.0,
                        distance / max(eff_vx, 0.05) * 3.0 + 2.0,
                    ),
                    stop_when=stop_when,
                )
            except Exception as exc:
                # 2026-08-09: lane 偶发异常不能把整个识别任务打挂 → 回退 move_for.
                print(f"  [warn] move_along_lane 异常({exc}), 切换 move_for",
                      file=sys.stderr, flush=True)
                move_forward_fallback(distance)
                after = read_traveled(client)
            else:
                after = read_traveled(client)
                moved = (after is not None and before is not None
                         and after >= before + max(0.02, distance * 0.25))
                if not moved and not (stopped and stopped()):
                    print("  [warn] move_along_lane 无里程变化，切换 move_for",
                          file=sys.stderr, flush=True)
                    move_forward_fallback(distance)
                    after = read_traveled(client)
        if before is not None and after is not None:
            return max(0.0, float(after) - float(before))
        return distance

    def wait_for_target(latch, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if latch[1](force=True):
                return latch[0]["det"]
            time.sleep(period)
        return None

    print(f"[recognition] move_along_lane 首卡定位，之后按上张卡横向位置 "
          f"步进 14/16/18cm (偏左/正中/偏右, 最后一个目标除外)，直到记录 "
          f"{args.target_count} 个目标或前进 {args.max_travel:.2f}m")
    try:
        # 2026-08-09 用户: 车完成 task1 到达识别区时已在第一张卡面前, 无需微调.
        # 起点直接识别"完整目标"(整张卡完整入画即可, 不需要在视野正中央):
        #   有 → 直接记录 #1; 没有 → 缓慢前进(4cm/步)直到第一张卡完整入画.
        first_det = None
        best_det = None
        if args.dry_run:
            first_latch = make_latch(None, required_samples=1, sample_period=0.05)
            first_det = wait_for_target(
                first_latch, args.dry_run_steps * period,
            )
            nominal_travel += args.creep_speed * args.dry_run_steps * period
        else:
            search_distance = 0.0
            initial = make_latch(None, required_samples=2)
            first_det = wait_for_target(initial, args.slot_wait_s)
            if first_det is None:
                # **2026-08-12 用户需求**: 首卡识别不出 → 前后微调搜索,
                # 直到识别出来再继续 16cm 移动; 前后都找不到才走向前蠕行兜底.
                print("[recognition] 首卡未识别到, 前后微调搜索", flush=True)
                first_det, search_net = search_front_back(
                    detect_fn=lambda: wait_for_target(
                        make_latch(None, required_samples=2), args.slot_wait_s),
                    move_fn=move_forward_fallback,
                    step_m=args.recog_search_step_m,
                    back_max_m=args.recog_search_back_max_m,
                    fwd_max_m=args.recog_search_fwd_max_m,
                    label="首卡",
                )
                nominal_travel += max(0.0, search_net)
            while first_det is None and search_distance < args.max_travel:
                search_step = min(0.04, args.max_travel - search_distance)
                move_latch = make_latch(None, required_samples=1,
                                        sample_period=0.05)
                move_along_lane(search_step,
                                stop_when=lambda *_: move_latch[1](),
                                stopped=lambda: move_latch[0]["ready"])
                search_distance += search_step
                time.sleep(max(period, 0.10))
                confirm = make_latch(None, required_samples=2)
                first_det = wait_for_target(confirm, args.slot_wait_s)
                if first_det is None and move_latch[0]["ready"]:
                    first_det = move_latch[0]["det"]
                if first_det is None:
                    nearest = centered(None)
                    if nearest is not None:
                        print(f"  [search] nearest xc={animal_center(nearest):+.3f}",
                              flush=True)
                        if best_det is None or abs(animal_center(nearest)) < abs(
                                animal_center(best_det)):
                            best_det = dict(nearest)
                    print(f"  [search] checked {search_distance:.2f}m, "
                          "首卡尚未完整入画", flush=True)
            nominal_travel = max(nominal_travel, search_distance)
        if first_det is None and best_det is not None:
            print(f"[warn] 首卡未完整入画稳定识别, 用最近目标补偿 "
                  f"(xc={animal_center(best_det):+.3f})", file=sys.stderr)
            first_det = best_det
        if first_det is None:
            print("[warn] 未找到首卡, 结束识别", file=sys.stderr)
        else:
            save_and_judge(first_det, 1)
        if records and not args.dry_run:
            # 2026-08-09 用户: 完整识别到第一个目标后立即停车,
            # 再连续前进 3 次 16cm 补录后面 3 个目标.
            safe_car_call(client, "stop", timeout=args.job_timeout)

        while records and len(records) < args.target_count:
            next_number = len(records) + 1
            # 2026-08-09 用户: 按上一个目标的横向位置决定下一步长度 (最后一个目标除外):
            #   正中 -> 16cm; 偏右 -> 18cm (多走 2cm); 偏左 -> 14cm (少走 2cm).
            last_xc = records[-1].get("xc") or 0.0
            step = step_for_xc(last_xc, base=args.target_spacing, tol=args.center_tol)
            side = ("偏右" if last_xc > args.center_tol
                    else "偏左" if last_xc < -args.center_tol else "居中")
            if traveled_now() + step > args.max_travel + 0.02:
                print("[warn] 已达到识别区最大行程，停止补录", file=sys.stderr)
                break
            nominal_travel += step
            if next_number == args.target_count:
                # 2026-08-09: 最后一次 (补录最后一张卡前) 改 move_for 直线
                print(f"[recognition] slot {next_number}: move_for "
                      f"+{step:.2f}m ({side}, 上张卡 xc={last_xc:+.3f}, 最后一次)",
                      flush=True)
                move_forward_fallback(step)
            else:
                print(f"[recognition] slot {next_number}: move_along_lane "
                      f"+{step:.2f}m ({side}, 上张卡 xc={last_xc:+.3f})", flush=True)
                move_along_lane(step)
            latch = make_latch(slot_window)
            det = wait_for_target(latch, args.slot_wait_s)
            if det is None:
                # 2026-08-09 用户: 目标不在视野正中央不能跳过 →
                # 兜底: 只要有一张完整入画的卡就记录(选最接近中央的).
                det = wait_for_target(
                    make_latch(None, required_samples=2), args.slot_wait_s,
                )
            if det is None:
                nearest = centered(None)
                if nearest is not None:
                    print(f"  [slot] nearest xc={animal_center(nearest):+.3f}",
                          flush=True)
                    det = nearest
                if det is None:
                    # **2026-08-12 用户需求**: 识别不出目标 → 前后微调找目标,
                    # 找到再继续 16cm 移动; 前后预算内都找不到才放弃该槽.
                    print(f"[warn] slot {next_number} 未识别到目标, "
                          f"前后微调搜索", file=sys.stderr, flush=True)
                    det, search_net = search_front_back(
                        detect_fn=lambda: wait_for_target(
                            make_latch(None, required_samples=2), args.slot_wait_s),
                        move_fn=move_forward_fallback,
                        step_m=args.recog_search_step_m,
                        back_max_m=args.recog_search_back_max_m,
                        fwd_max_m=args.recog_search_fwd_max_m,
                        label=f"slot #{next_number}",
                    )
                    if det is not None:
                        nominal_travel += max(0.0, search_net)
                        print(f"  [slot] #{next_number} 前后微调 "
                              f"{search_net:+.2f}m 后识别到", flush=True)
                    else:
                        print(f"[warn] slot {next_number} 前后微调后仍未识别到, "
                              f"放弃该槽", file=sys.stderr)
                        break
            save_and_judge(det, next_number)
    finally:
        detector.stop()
        if not args.dry_run:
            safe_car_call(client, "stop", timeout=args.job_timeout)

    print(f"[recognition] driving complete; judging {len(records)} recorded targets")
    if defer_judge:
        # 2026-08-07: LLM 判定放后台, 识别段只存 raw 记录 (status=pending) 立即返回,
        # 不阻塞车. task3_pest_scout 后台线程读到 pending json 后逐张判定并回写 done.
        print(f"[recognition] driving complete; deferring LLM judging "
              f"({len(records)} targets, status=pending)")
        return records, traveled_now()
    # 2026-08-12: 内联判定已当场覆盖大部分 (含 result=None 重拍);
    # 仅兜底判定仍缺 result 的 (重拍预算耗尽 / 内联判定异常) —— 不重判已判定的卡,
    # 避免 LLM 非确定性覆盖内联结果.
    pending = [r for r in records if r.get("result") is None]
    if pending:
        print(f"[recognition] 内联判定缺 {len(pending)} 张, 兜底批量判定", flush=True)
        try:
            judged = _classify_records(
                token, pending, args.llm_timeout, args.classify_workers,
            )
            judged_by_num = {j["number"]: j for j in judged}
            for i, r in enumerate(records):
                if r["number"] in judged_by_num:
                    records[i] = judged_by_num[r["number"]]
        except Exception as exc:
            # 2026-08-12: ERNIE 异常不打断识别 → 保留 result=None, 后台可重试.
            print(f"[warn] 兜底批量判定异常: {exc} (保留未判定, 后台可重试)",
                  file=sys.stderr, flush=True)
    if len(records) != args.target_count:
        print(f"[warn] recorded {len(records)}/{args.target_count} targets",
              file=sys.stderr)
    return records, traveled_now()


def save_result(path_text, args, targets, traveled, *, status: str = "done"):
    path = Path(path_text)
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / path
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "config": vars(args),
        "status": status,
        "targets": targets,
        "pest_numbers": [t["number"] for t in targets if t.get("result") == 0],
        "beneficial_numbers": [t["number"] for t in targets if t.get("result") == 1],
        "traveled_m": round(traveled, 4),
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[recognition] saved: {path} (status={status})")
    return path


def run_shooting(client, args, pest_numbers, identity_file=None):
    if not pest_numbers:
        print("[shooting] no confirmed pests; skip shooting")
        return 0
    target_arg = " ".join(str(n) for n in pest_numbers)
    print(f"[shooting] confirmed pest targets: {target_arg}")
    command = [
        sys.executable, "-m", "main.task.task3.shoot_target",
        "--targets", target_arg,
    ]
    if identity_file:
        command.extend(["--identity-file", str(identity_file)])
    if args.dry_run:
        print("[dry-run] " + " ".join(command))
        return 0
    try:
        return subprocess.run(command, check=False).returncode
    finally:
        if not args.dry_run:
            safe_car_call(client, "stop", timeout=args.job_timeout)


def main():
    parser = argparse.ArgumentParser(description="Task 3 recognition + shooting pipeline")
    parser.add_argument("--token", default=None)
    parser.add_argument("--target-count", type=int, default=DEFAULT_TARGET_COUNT)
    parser.add_argument("--creep-speed", type=float, default=DEFAULT_CREEP_SPEED,
                        help="recognition creep speed in m/s (default 0.05)")
    parser.add_argument("--max-travel", type=float, default=1.5,
                        help="safety cap: stop driving after this many meters even if "
                             "fewer than target-count targets were recorded")
    parser.add_argument("--min-score", type=float, default=DEFAULT_MIN_SCORE)
    parser.add_argument("--center-tol", type=float, default=DEFAULT_CENTER_TOL)
    parser.add_argument("--min-gap", type=float, default=DEFAULT_MIN_GAP,
                        help="frame-to-frame association radius for xc (normalized coords, effective minimum 0.30; yc gate separates side-by-side cards)")
    parser.add_argument("--crop-padding", type=float, default=0.10)
    parser.add_argument("--llm-timeout", type=float, default=15.0)
    parser.add_argument("--job-timeout", type=float, default=20.0)
    parser.add_argument("--poll-interval", type=float, default=0.15)
    parser.add_argument("--target-spacing", "--straight-step", dest="target_spacing",
                        type=float, default=DEFAULT_TARGET_SPACING_M,
                        help="center-to-center distance between target cards (default 0.15m)")
    parser.add_argument("--slot-wait-s", type=float, default=1.0,
                        help="seconds to wait for a centered target after each fixed move")
    parser.add_argument("--target-settle-s", type=float, default=DEFAULT_TARGET_SETTLE_S,
                        help="stable observation time before a card is recorded (seconds)")
    parser.add_argument("--recog-search-step-m", type=float,
                        default=DEFAULT_RECOG_SEARCH_STEP_M,
                        help="前后微调识别步长 m (默认 4cm)")
    parser.add_argument("--recog-search-back-max-m", type=float,
                        default=DEFAULT_RECOG_SEARCH_BACK_MAX_M,
                        help="前后微调向后上限 m (默认 12cm)")
    parser.add_argument("--recog-search-fwd-max-m", type=float,
                        default=DEFAULT_RECOG_SEARCH_FWD_MAX_M,
                        help="前后微调向前上限 m (默认 16cm)")
    parser.add_argument("--max-llm-recapture", type=int, default=MAX_LLM_RECAPTURE,
                        help="单张卡 LLM 判定 result=None 时前后微调重拍最大次数 (默认 2)")
    parser.add_argument("--classify-workers", type=int, default=DEFAULT_CLASSIFY_WORKERS,
                        help="parallel vision requests after scanning")
    parser.add_argument("--save", default="audit/task3_pipeline.json")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--dry-run-steps", type=int, default=10,
                        help="safety limit used only with --dry-run")
    parser.add_argument("--no-pause", action="store_true", help="start shooting immediately after recognition")
    parser.add_argument("--no-shoot", action="store_true",
                        help="识别完成不射击 (编排模式: 射击由 task3_shoot waypoint 负责)")
    parser.add_argument("--defer-judge", action="store_true",
                        help="识别段不 LLM 判定: 存 raw targets (status=pending) 立即返回, "
                             "判定由调用方后台线程 judge_records 回写 done")
    args = parser.parse_args()

    if (args.target_count < 1 or not 0 < args.creep_speed <= 0.2
            or args.dry_run_steps < 1 or args.target_spacing <= 0
            or args.slot_wait_s <= 0 or args.target_settle_s < 0
            or args.classify_workers < 1
            or args.min_gap < 0
            or args.max_llm_recapture < 0):
        parser.error("target-count/creep-speed/dry-run-steps values are invalid")
    # --defer-judge: 识别段完全不碰 LLM (token/health 都省), 判定由后台线程做.
    if args.defer_judge:
        token = ""
        print("[ready] --defer-judge: LLM 判定推迟到后台线程, 识别段不调 token/health")
    else:
        token = load_token(args.token)
        print(f"[ready] token={mask_token(token)}")
        if not args.dry_run:
            try:
                check_health(token, timeout=args.llm_timeout)
            except Exception as exc:
                # 2026-08-12: ERNIE 异常不打断识别 → 卡照录, 内联判定留 result=None,
                # 末尾兜底/后台线程可重试.
                print(f"[warn] ERNIE health 检查失败: {exc} (识别照跑, 判定后置)",
                      file=sys.stderr, flush=True)
    client = RuntimeApiClient()
    if not args.dry_run:
        client.wait_until_ready()
    settings = __import__("main.settings", fromlist=["load_settings"]).load_settings()
    # 2026-08-12 用户指令: task3 识别前**不对机械臂做任何调整** (原 RECOGNITION_ARM 摆臂取消).
    # 需要恢复时改回:  if run_arm_pose(args, "recognition pose", RECOGNITION_ARM, client=client) != 0: return 1
    image_dir = Path(__file__).resolve().parent / "audit" / "task3_pipeline" / "targets"
    targets, traveled = recognize_phase_fixed_slots(
        client, args, token, settings.streamer_url, image_dir,
        defer_judge=args.defer_judge,
    )
    status = "pending" if args.defer_judge else "done"
    result_path = save_result(args.save, args, targets, traveled, status=status)
    pests = [t["number"] for t in targets if t.get("result") == 0]
    beneficial = [t["number"] for t in targets if t.get("result") == 1]
    print(f"[recognition] pests={pests or 'none'} beneficial={beneficial or 'none'}")

    if args.no_shoot:
        print("[recognition] --no-shoot: recognition done, return to orchestrator "
              "(shooting deferred to task3_shoot waypoint)", flush=True)
        return 0
    if not args.no_pause and not args.dry_run:
        input(
            "[pause] Recognition complete. Move the car to the shooting area, "
            "place it correctly, then press Enter to continue: "
        )
    print(f"[shooting] preparing confirmed pest targets: {pests}", flush=True)
    if run_arm_pose(args, "shooting pose", SHOOTING_ARM, client=client) != 0:
        return 1
    return run_shooting(client, args, pests, result_path)


if __name__ == "__main__":
    raise SystemExit(main())
