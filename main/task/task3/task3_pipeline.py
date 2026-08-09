#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Task 3 pipeline: record four boards, judge them after driving, then shoot.

The recognition phase moves the arm to the recognition pose, drives slowly
until all target crops are recorded without waiting for the LLM. After the car
stops, the crops are judged in order. The operator then moves the car to the
shooting area; Enter starts the existing shooting algorithm after the arm is
moved to its shooting pose.

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

from main.api_client import RuntimeApiClient
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

DEFAULT_TARGET_COUNT = 4
DEFAULT_CREEP_SPEED = 0.18          # m/s, 识别段 creep 速度 (2026-08-04 现场: 0.05→0.18)
DEFAULT_MIN_SCORE = 0.40
DEFAULT_CENTER_TOL = 0.15
DEFAULT_MIN_GAP = 0.16          # 卡片中心距 16cm → 去重窗口需覆盖一张卡的间距
DEFAULT_CLASSIFY_WORKERS = 2
DEFAULT_TARGET_SPACING_M = 0.16       # 卡片 8cm + 间隔 8cm = 中心距 16cm
DEFAULT_TARGET_SETTLE_S = 0.15
RECOGNITION_ARM = ("-0.100", "-0.040", "-0.270", "90", "-70")
SHOOTING_ARM = ("-0.100", "-0.150", "-0.200", "90", "-90")


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


def run_arm_pose(args, label, pose):
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
    verdict = call_vision(token, image_url, RECOGNITION_PROMPT, timeout=timeout)
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


def recognize_phase_fixed_slots(client, args, token, streamer_url, output_dir,
                                defer_judge: bool = False):
    """首卡定位后按固定卡间距逐个记录，避免同一张卡重复计数。"""
    records = []
    period = max(args.poll_interval, 0.05)
    min_samples = max(2, int(args.target_settle_s / period) + 1)
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
                    now = time.monotonic()
                    if error and now - self._last_direct >= 0.45:
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

    def centered(window):
        detections = [det for det in detector.read() if eligible(det)]
        candidates = [det for det in detections
                      if abs(animal_center(det)) <= window]
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

    def move_along_lane(distance, stop_when=None, stopped=None):
        if args.dry_run:
            return
        from main.chassis.config import LANE_FOLLOW
        from main.chassis.controllers import move_along_lane as lane_move
        print(f"  [drive] move_along_lane +{distance:.2f}m", flush=True)
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
            return
        lane_move(
            vx=args.creep_speed,
            distance_m=distance,
            profile=LANE_FOLLOW.tuned(watchdog_ms=None),
            max_seconds=max(
                5.0,
                distance / max(args.creep_speed, 0.05) * 3.0 + 2.0,
            ),
            stop_when=stop_when,
        )
        after = read_traveled(client)
        moved = (after is not None and before is not None
                 and after >= before + max(0.02, distance * 0.25))
        if not moved and not (stopped and stopped()):
            print("  [warn] move_along_lane 无里程变化，切换 move_for",
                  file=sys.stderr, flush=True)
            move_forward_fallback(distance)

    def wait_for_target(latch, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if latch[1](force=True):
                return latch[0]["det"]
            time.sleep(period)
        return None

    print(f"[recognition] move_along_lane 首卡定位，之后每次前进 "
          f"{args.target_spacing:.2f}m，直到记录 {args.target_count} 个目标"
          f"或前进 {args.max_travel:.2f}m")
    try:
        # 车辆行驶时首卡可能只在一个检测周期内经过中心，因此首卡不要求
        # 连续两帧；停车后再用普通稳定条件确认并裁剪。
        first_latch = make_latch(first_window, required_samples=1,
                                 sample_period=0.05)
        if args.dry_run:
            first_det = wait_for_target(
                first_latch, args.dry_run_steps * period,
            )
            nominal_travel += args.creep_speed * args.dry_run_steps * period
        else:
            first_det = None
            search_distance = 0.0
            best_det = None
            best_abs_x = None
            best_distance = 0.0
            initial = make_latch(first_window, required_samples=1)
            first_det = wait_for_target(initial, min(args.slot_wait_s, 0.5))
            while first_det is None and search_distance < args.max_travel:
                search_step = min(0.04, args.max_travel - search_distance)
                first_latch = make_latch(first_window, required_samples=1,
                                         sample_period=0.05)
                move_along_lane(search_step,
                                stop_when=lambda *_: first_latch[1](),
                                stopped=lambda: first_latch[0]["ready"])
                search_distance += search_step
                time.sleep(max(period, 0.10))
                first_confirm = make_latch(first_window, required_samples=1)
                first_det = wait_for_target(first_confirm, min(args.slot_wait_s, 0.5))
                if first_det is None and first_latch[0]["ready"]:
                    first_det = first_latch[0]["det"]
                nearest = centered(1.0)
                if nearest is not None:
                    nearest_abs_x = abs(animal_center(nearest))
                    print(f"  [search] nearest xc={animal_center(nearest):+.3f}",
                          flush=True)
                    if best_abs_x is None or nearest_abs_x < best_abs_x:
                        best_det = dict(nearest)
                        best_abs_x = nearest_abs_x
                        best_distance = search_distance
                    elif (best_abs_x < nearest_abs_x
                          and search_distance > best_distance
                          and best_abs_x <= first_window):
                        backoff = min(search_step, search_distance - best_distance)
                        move_forward_fallback(-backoff)
                        search_distance -= backoff
                        first_det = best_det
                if first_det is None:
                    print(f"  [search] checked {search_distance:.2f}m, "
                          "首卡尚未进入中心", flush=True)
            nominal_travel = max(nominal_travel, search_distance)
        if first_det is None:
            if best_det is not None:
                print(f"[warn] 首卡未在中心窗口内稳定识别，"
                      f"用最近接的 best_det (xc={animal_center(best_det):+.3f}) 補救",
                      file=sys.stderr)
                first_det = best_det
                save_slot(first_det, 1)
            else:
                print("[warn] 首卡未在中心窗口內穩定識別", file=sys.stderr)
        else:
            save_slot(first_det, 1)

        while records and len(records) < args.target_count:
            next_number = len(records) + 1
            if traveled_now() + args.target_spacing > args.max_travel + 0.02:
                print("[warn] 已达到识别区最大行程，停止补录", file=sys.stderr)
                break
            nominal_travel += args.target_spacing
            print(f"[recognition] slot {next_number}: move_along_lane "
                  f"+{args.target_spacing:.2f}m", flush=True)
            move_along_lane(args.target_spacing)
            latch = make_latch(slot_window)
            det = wait_for_target(latch, args.slot_wait_s)
            if det is None:
                nearest = centered(1.0)
                if nearest is not None:
                    print(f"  [slot] nearest xc={animal_center(nearest):+.3f}",
                          flush=True)
                    if abs(animal_center(nearest)) <= 0.45:
                        det = nearest
                if det is None:
                    print(f"[warn] slot {next_number} 未找到目标，车辆保持停车",
                          file=sys.stderr)
                    break
            save_slot(det, next_number)
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
    judged = _classify_records(
        token, records, args.llm_timeout, args.classify_workers,
    )
    if len(judged) != args.target_count:
        print(f"[warn] recorded {len(judged)}/{args.target_count} targets",
              file=sys.stderr)
    return judged, traveled_now()


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
                        help="center-to-center distance between target cards (default 0.16m)")
    parser.add_argument("--slot-wait-s", type=float, default=1.0,
                        help="seconds to wait for a centered target after each fixed move")
    parser.add_argument("--target-settle-s", type=float, default=DEFAULT_TARGET_SETTLE_S,
                        help="stable observation time before a card is recorded (seconds)")
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
            or args.min_gap < 0):
        parser.error("target-count/creep-speed/dry-run-steps values are invalid")
    # --defer-judge: 识别段完全不碰 LLM (token/health 都省), 判定由后台线程做.
    if args.defer_judge:
        token = ""
        print("[ready] --defer-judge: LLM 判定推迟到后台线程, 识别段不调 token/health")
    else:
        token = load_token(args.token)
        print(f"[ready] token={mask_token(token)}")
        if not args.dry_run:
            check_health(token, timeout=args.llm_timeout)
    client = RuntimeApiClient()
    if not args.dry_run:
        client.wait_until_ready()
    settings = __import__("main.settings", fromlist=["load_settings"]).load_settings()
    if run_arm_pose(args, "recognition pose", RECOGNITION_ARM) != 0:
        return 1
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
    if run_arm_pose(args, "shooting pose", SHOOTING_ARM) != 0:
        return 1
    return run_shooting(client, args, pests, result_path)


if __name__ == "__main__":
    raise SystemExit(main())
