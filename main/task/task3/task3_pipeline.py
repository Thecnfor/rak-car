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
import time
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
DEFAULT_MIN_SCORE = 0.50
DEFAULT_CENTER_TOL = 0.10
DEFAULT_UNLOCK_TOL = 0.45
RECOGNITION_ARM = ("-0.040", "-0.270", "90", "-70")
SHOOTING_ARM = ("-0.150", "-0.200", "90", "-90")


def read_detections(client):
    try:
        state = (client.get_task_state() or {}).get("task_state") or {}
        return list(state.get("detections") or [])
    except Exception as exc:
        print(f"[warn] detection read failed: {exc}", file=sys.stderr)
        return []


def read_odom_distance(client):
    """读 odom_feed 缓存的累计行驶距离 (m)。odom_feed 未运行/失败返回 None。"""
    try:
        odo = (client.get_odom_state() or {}).get("odom_state") or {}
        distance = odo.get("distance")
        return float(distance) if distance is not None else None
    except Exception as exc:
        print(f"[warn] odom read failed: {exc}", file=sys.stderr)
        return None


def bbox(det):
    return det.get("bbox_norm") or {}


def animal_center(det):
    return float(bbox(det).get("x_center", 0.0))


def pick_center_animal(detections, min_score, center_tol):
    candidates = [
        d for d in detections
        if d.get("label") == "animal"
        and float(d.get("score") or 0.0) >= min_score
        and abs(animal_center(d)) <= center_tol
    ]
    return min(candidates, key=lambda d: abs(animal_center(d))) if candidates else None


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
               "--y1", pose[0], "--x", pose[1],
               "--arm-angle", pose[2], "--hand-angle", pose[3]]
    print(f"[arm] {label}: {' '.join(command)}", flush=True)
    if args.dry_run:
        return 0
    return subprocess.run(command, check=False).returncode


def capture_target(streamer_url, det, crop_padding, output_dir, number):
    from main.misc.test_pest_llm_shoot import crop_bbox
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


def _set_chassis_vel(client, vx):
    """下一帧底盘 realtime 速度 (同 task4 creep 通道)。异常只 warn, 下一帧自愈。"""
    try:
        client.post(
            f"{client.api_prefix}/realtime/chassis-velocity",
            {"vx": float(vx), "vy": 0.0, "wz": 0.0},
            timeout=1.0,
        )
    except Exception as exc:  # noqa: BLE001
        print(f"[warn] chassis-velocity: {exc}", file=sys.stderr)


def recognize_phase(client, args, token, streamer_url, output_dir):
    """creep 识别: 持续低速前移 + 连续读检测, 记满 target_count 立即停车。

    底盘走 /v1/realtime/chassis-velocity (realtime 门, 不进 job_queue),
    每 poll_interval 秒下发一次速度 + 读一帧检测; 开环 速度×时间 记账里程。
    里程停止条件: odom 从识别开始快照起算的增量 (不受触发点/巡航里程影响),
    creep 满 odom_stop_m 即停; 触发点浮动不影响实际扫描长度。
    finally 保证速度清零, 再补一个 car.stop 兜底。
    """
    records = []
    traveled = 0.0
    drive_iters = 0
    locked = False
    missed_frames = 0
    period = max(args.poll_interval, 0.02)
    odom_start = None if args.dry_run else read_odom_distance(client)
    if odom_start is None and not args.dry_run:
        print("[recognition] odom unavailable; fall back to open-loop max_travel",
              file=sys.stderr)
    print(f"[recognition] creep at {args.creep_speed} m/s until "
          f"{args.target_count} targets are recorded; record only")
    try:
        while len(records) < args.target_count:
            if not args.dry_run:
                _set_chassis_vel(client, args.creep_speed)
            time.sleep(period)
            traveled += args.creep_speed * period
            drive_iters += 1

            if odom_start is not None:
                odom_delta = read_odom_distance(client)
                if odom_delta is not None and odom_delta - odom_start >= args.odom_stop_m:
                    print(f"[recognition] odom distance reached "
                          f"{odom_delta - odom_start:.2f}m >= {args.odom_stop_m}m, stopping")
                    break

            animals = read_detections(client)
            target = pick_center_animal(animals, args.min_score, args.center_tol)
            if target is not None and not locked and len(records) < args.target_count:
                locked = True
                missed_frames = 0
                number = len(records) + 1
                if args.dry_run:
                    image_path = None
                else:
                    # 停稳再拍: 0.18 m/s 直拍有运动模糊 + 帧滞后板偏框边
                    _set_chassis_vel(client, 0.0)
                    time.sleep(0.4)
                    fresh = pick_center_animal(
                        read_detections(client), args.min_score, 1.0)
                    image_path = capture_target(
                        streamer_url, fresh or target, args.crop_padding,
                        output_dir, number
                    )
                records.append({
                    "number": number,
                    "image_path": str(image_path) if image_path else None,
                    "detection": dict(target),
                    "xc": animal_center(target),
                    "score": float(target.get("score") or 0.0),
                    "traveled_m": round(traveled, 4),
                    "timestamp": time.time(),
                })
                print(f"  [record] target #{number} xc={animal_center(target):+.3f} "
                      f"at {traveled:.2f}m image={'saved' if image_path else 'missing'}",
                      flush=True)
                if len(records) >= args.target_count:
                    break
            elif locked and (target is None or abs(animal_center(target)) >= args.unlock_tol):
                missed_frames += 1
                if missed_frames >= 3:
                    locked = False
                    missed_frames = 0
            elif target is not None:
                missed_frames = 0

            if traveled >= args.max_travel:
                print(f"[recognition] max travel {args.max_travel}m reached with "
                      f"{len(records)}/{args.target_count} targets, stopping",
                      file=sys.stderr)
                break
            if args.dry_run and drive_iters >= args.dry_run_steps:
                break
    finally:
        if not args.dry_run:
            _set_chassis_vel(client, 0.0)
    if not args.dry_run:
        safe_car_call(client, "stop", timeout=args.job_timeout)

    print(f"[recognition] driving complete; judging {len(records)} recorded targets")
    judged = []
    for record in records:
        verdict = classify_target(token, record["image_path"], args.llm_timeout)
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
    if len(judged) != args.target_count:
        print(f"[warn] recorded {len(judged)}/{args.target_count} targets", file=sys.stderr)
    return judged, traveled


def save_result(path_text, args, targets, traveled):
    path = Path(path_text)
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / path
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "config": vars(args),
        "targets": targets,
        "pest_numbers": [t["number"] for t in targets if t["result"] == 0],
        "beneficial_numbers": [t["number"] for t in targets if t["result"] == 1],
        "traveled_m": round(traveled, 4),
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[recognition] saved: {path}")
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
    parser.add_argument("--max-travel", type=float, default=4.0,
                        help="safety cap: stop driving after this many meters even if "
                             "fewer than target-count targets were recorded")
    parser.add_argument("--odom-stop-m", type=float, default=0.66,
                        help="stop driving once odometer distance delta from recognition start reaches this (m)")
    parser.add_argument("--min-score", type=float, default=DEFAULT_MIN_SCORE)
    parser.add_argument("--center-tol", type=float, default=DEFAULT_CENTER_TOL)
    parser.add_argument("--unlock-tol", type=float, default=DEFAULT_UNLOCK_TOL)
    parser.add_argument("--crop-padding", type=float, default=0.10)
    parser.add_argument("--llm-timeout", type=float, default=15.0)
    parser.add_argument("--job-timeout", type=float, default=20.0)
    parser.add_argument("--poll-interval", type=float, default=0.15)
    parser.add_argument("--save", default="audit/task3_pipeline.json")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--dry-run-steps", type=int, default=10,
                        help="safety limit used only with --dry-run")
    parser.add_argument("--no-pause", action="store_true", help="start shooting immediately after recognition")
    parser.add_argument("--no-shoot", action="store_true",
                        help="recognition only: skip pause & shooting, return after saving json "
                             "(orchestrator 编排模式, 射击由 task3_shoot waypoint 负责)")
    args = parser.parse_args()

    if args.target_count < 1 or not 0 < args.creep_speed <= 0.2 or args.dry_run_steps < 1 \
            or args.odom_stop_m <= 0:
        parser.error("target-count/creep-speed/dry-run-steps/odom-stop-m values are invalid")
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
    targets, traveled = recognize_phase(
        client, args, token, settings.streamer_url, image_dir
    )
    result_path = save_result(args.save, args, targets, traveled)
    pests = [t["number"] for t in targets if t["result"] == 0]
    beneficial = [t["number"] for t in targets if t["result"] == 1]
    print(f"[recognition] pests={pests or 'none'} beneficial={beneficial or 'none'}")

    if args.no_shoot:
        print("[recognition] --no-shoot: recognition done, return to orchestrator "
              "(shooting deferred to task3_shoot waypoint)", flush=True)
        return 0
    if not args.no_pause and not args.dry_run:
        input("[pause] 请将车辆移动到射击区并摆正，准备好后按 Enter 继续射击: ")
    if pests and run_arm_pose(args, "shooting pose", SHOOTING_ARM) != 0:
        return 1
    return run_shooting(client, args, pests, result_path)


if __name__ == "__main__":
    raise SystemExit(main())
