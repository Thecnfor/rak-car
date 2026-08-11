#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task3 识别段独立脚本: 首卡居中 → 动态步进 14/16/18cm → 后台判定害虫/益虫.

识别工作单独抽出 (不包含射击), 供单独测试; 输出 manifest 与编排流程同款
(audit/task3_pipeline.json, status=pending → 后台线程判定回写 done).

用法 (仓库根目录, 车在识别区起点停好):
    python -m main.task.task3.recognize_targets
    python -m main.task.task3.recognize_targets --dry-run        # 纯模拟, 不动硬件
    python -m main.task.task3.recognize_targets --step-m 0.16 --creep-speed 0.15
    python -m main.task.task3.recognize_targets --judge-inline   # 同步判定 (不等后台)

流程:
    1. 识别前不对机械臂做任何调整 (2026-08-12 用户指令)
    2. 匀速缓慢前进 (creep), 第一个目标完整入画 → 停车 → 稳定确认 → 记录 #1
    3. 按 #1 横向位置动态步进 14/16/18cm (里程计闭环 move_for) → 确认 → 记录 #2
    4. 按 #2 横向位置动态步进 → 记录 #3
    5. 按 #3 横向位置动态步进 → 记录 #4
    6. 停车, 保存 audit json (status=pending)
    7. 后台线程逐张 LLM 判定 (害虫/益虫/物种) → 回写 status=done + pest_numbers
    8. 立即返回 (不阻塞后续任务)
"""
from __future__ import annotations

import argparse
import base64
import json
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from main.api_client import RuntimeApiClient
from main.task.task3.arm_poses import arm_at_pose
from main.task.task3.llm_ernie import call_vision, load_token
# 复用 task3_pipeline 的判定/重拍纯逻辑 (同包, 避免重复实现 2026-08-12)
from main.task.task3.task3_pipeline import classify_target, judge_and_recapture

# 默认保存路径与 task3_pipeline 一致 (task3_shoot 读同一文件)
DEFAULT_SAVE = Path(__file__).resolve().parent / "audit" / "task3_pipeline.json"

DEFAULT_TARGET_COUNT = 4
DEFAULT_STEP_M = 0.16          # 卡片中心距 16cm
DEFAULT_CREEP_SPEED = 0.18     # m/s, 匀速缓慢前进
DEFAULT_CENTER_WINDOW = 0.15   # 首卡"视野较为中央"窗口 (归一化 xc)
DEFAULT_SLOT_WINDOW = 0.35     # 后续卡确认窗口 (归一化 xc)
DEFAULT_SEARCH_STEP_M = 0.04   # 首卡搜索小步 (m)
DEFAULT_SETTLE_SAMPLES = 4     # 稳定确认帧数
DEFAULT_SLOT_WAIT_S = 1.0      # 每步停稳后等目标的最长时间
DEFAULT_MAX_TRAVEL_M = 1.5     # 识别区最大行程保护 (m)
DEFAULT_MIN_SCORE = 0.40
DEFAULT_POLL_INTERVAL = 0.15
DEFAULT_JOB_TIMEOUT = 20.0
DEFAULT_LLM_TIMEOUT = 15.0
# 前后微调识别 (2026-08-12 用户需求: 识别不出目标时前后微调, 直到识别出来再继续 16cm)
DEFAULT_RECOG_SEARCH_STEP_M = 0.04        # 微调步长 4cm
DEFAULT_RECOG_SEARCH_BACK_MAX_M = 0.12    # 向后微调上限 12cm (over-shot 常见, 先退)
DEFAULT_RECOG_SEARCH_FWD_MAX_M = 0.16     # 向前微调上限 16cm


def step_for_xc(xc, base=DEFAULT_STEP_M, adjust=0.02, tol=DEFAULT_CENTER_WINDOW):
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


def _load_pest_prompt() -> str:
    """读 llm_config.yml 的 pest_detect.system_prompt (与 task3_pipeline 同源)."""
    try:
        cfg = yaml.safe_load(
            (Path(__file__).resolve().parent / "llm_config.yml").read_text(encoding="utf-8")
        ) or {}
        prompt = (cfg.get("pest_detect") or {}).get("system_prompt")
        if prompt and str(prompt).strip():
            return str(prompt)
    except Exception:
        pass
    return ("Analyze the animal in this image as an agricultural expert.\n"
            "Return STRICT JSON only: {\"name\":\"...\",\"result\":0,\"analysis\":\"...\"}\n"
            "result=0 means harmful pest, result=1 means beneficial.\n")


PEST_PROMPT = _load_pest_prompt()


# ── 基础读取 ────────────────────────────────────────────────

def read_detections(client) -> List[dict]:
    try:
        state = (client.get_task_state() or {}).get("task_state") or {}
        return list(state.get("detections") or [])
    except Exception:
        return []


def bbox(det):
    return det.get("bbox_norm") or {}


def animal_center(det) -> float:
    return float(bbox(det).get("x_center", 0.0))


def read_traveled(client) -> Optional[float]:
    """读 odom 累计路径长 (m), 失败返回 None."""
    try:
        state = (client.get_odom_state() or {}).get("odom_state") or {}
        distance = state.get("distance")
        return float(distance) if distance is not None else None
    except Exception:
        return None


def fetch_frame(streamer_url, timeout=5.0) -> Optional[bytes]:
    try:
        import requests
        response = requests.get(
            f"{streamer_url.rstrip('/')}/frame/cam2.jpg", timeout=timeout
        )
        response.raise_for_status()
        return response.content
    except Exception:
        return None


def det_to_list(det):
    b = bbox(det)
    return [det.get("cls_id"), det.get("det_id"), det.get("label", ""),
            det.get("score", 0.0), b.get("x_center", 0.0), b.get("y_center", 0.0),
            b.get("width", 0.0), b.get("height", 0.0)]


def capture_target(streamer_url, det, crop_padding, output_dir, number, frame=None):
    """按 bbox 裁剪最新帧并保存 target_%02d.jpg, 返回路径或 None."""
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


def safe_stop(client, job_timeout=DEFAULT_JOB_TIMEOUT) -> None:
    """下发 stop (best-effort)."""
    try:
        job = client.execute_car_action("stop", timeout=job_timeout, sync=False)
        jid = job.get("id") if isinstance(job, dict) else None
        if jid:
            client.wait_job(jid, timeout=job_timeout + 10.0)
    except Exception as exc:
        print(f"[warn] stop 失败: {exc}", file=sys.stderr, flush=True)


class DetectionReader:
    """后台读 cam2 目标检测缓存 (fast-path), 失败时直连 request_vision_task 兜底."""

    def __init__(self, client):
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._latest: List[dict] = []
        self._cache_updated = None
        self._last_direct = 0.0
        self._client = client
        self._thread = threading.Thread(
            target=self._run, name="task3-detect-reader", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def read(self) -> List[dict]:
        with self._lock:
            return [dict(d) for d in self._latest]

    def _publish(self, detections):
        with self._lock:
            self._latest = list(detections or [])

    def _run(self):
        direct_client = RuntimeApiClient()
        while not self._stop.is_set():
            try:
                state = (self._client.get_task_state() or {}).get("task_state") or {}
                updated = state.get("updated_at")
                error = state.get("last_error")
                if updated != self._cache_updated and not error:
                    self._cache_updated = updated
                    self._publish(state.get("detections"))
                # 2026-08-09: task_feed 缓存不新鲜(识别起点/补录时 feed 可能未刷新)
                # 或报错 → 主动跑一次同步推理, 避免"车已在卡面前却识别不到".
                try:
                    age = time.time() - float(updated) if updated is not None else float("inf")
                except (TypeError, ValueError):
                    age = float("inf")
                stale = (error or self._cache_updated is None or age > 0.6)
                if stale and time.monotonic() - self._last_direct >= 0.45:
                    self._last_direct = time.monotonic()
                    result = direct_client.request_vision_task(timeout=3.0)
                    if result.get("ok"):
                        self._publish(result.get("detections"))
            except Exception:
                pass
            self._stop.wait(0.05)


# ── 核心识别 ────────────────────────────────────────────────

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


def recognize_targets(client, *,
                      target_count: int = DEFAULT_TARGET_COUNT,
                      step_m: float = DEFAULT_STEP_M,
                      creep_speed: float = DEFAULT_CREEP_SPEED,
                      center_window: float = DEFAULT_CENTER_WINDOW,
                      slot_window: float = DEFAULT_SLOT_WINDOW,
                      search_step_m: float = DEFAULT_SEARCH_STEP_M,
                      settle_samples: int = DEFAULT_SETTLE_SAMPLES,
                      slot_wait_s: float = DEFAULT_SLOT_WAIT_S,
                      max_travel_m: float = DEFAULT_MAX_TRAVEL_M,
                      min_score: float = DEFAULT_MIN_SCORE,
                      poll_interval: float = DEFAULT_POLL_INTERVAL,
                      job_timeout: float = DEFAULT_JOB_TIMEOUT,
                      streamer_url: Optional[str] = None,
                      output_dir: Optional[Path] = None,
                      dry_run: bool = False,
                      recog_search_step_m: float = DEFAULT_RECOG_SEARCH_STEP_M,
                      recog_search_back_max_m: float = DEFAULT_RECOG_SEARCH_BACK_MAX_M,
                      recog_search_fwd_max_m: float = DEFAULT_RECOG_SEARCH_FWD_MAX_M,
                      token: str = "",
                      llm_timeout: float = DEFAULT_LLM_TIMEOUT,
                      max_llm_recapture: int = 0
                      ) -> Tuple[List[dict], float]:
    """识别目标卡: 首卡完整入画停车记录 → 动态步进逐个记录.

    匀速缓慢前进 (creep); 按上一张卡的横向位置决定下一步长度 (最后一个目标除外):
    正中 → 16cm, 偏右 → 18cm, 偏左 → 14cm; 每步走 move_for (里程计闭环)。
    识别不出目标时 (2026-08-12): 前后微调 (先退后进) 直到识别出来, 再继续下一步;
    给了 token 且 max_llm_recapture>0 时, 每张卡当场 LLM 判定, result=None
    (判不出害虫/益虫) 也前后微调重拍。
    返回 (raw_records, traveled_m); raw_records 未做 LLM 判定 (含 image_path/detection)。
    """
    period = max(poll_interval, 0.05)
    records: List[dict] = []
    start_odom = read_traveled(client)
    nominal_travel = 0.0

    def traveled_now() -> float:
        distance = read_traveled(client)
        if distance is None or start_odom is None or distance < start_odom:
            return nominal_travel
        return max(nominal_travel, distance - start_odom)

    def _move_for(distance: float) -> float:
        """执行 move_for (匀速, 里程计闭环), 返回实际里程增量 (不更新 nominal_travel)."""
        if dry_run:
            return distance
        before = read_traveled(client)
        job = client.execute_car_action(
            "move_for", [distance, 0.0, 0.0],
            timeout=job_timeout, sync=False,
            max_velocities=[creep_speed, creep_speed, 3.14159 / 3],
        )
        done = client.wait_job(job["id"], timeout=job_timeout + 10.0)
        if done.get("status") != "succeeded":
            raise RuntimeError(f"car.move_for failed: {done.get('error')}")
        after = read_traveled(client)
        if before is not None and after is not None:
            return max(0.0, float(after) - float(before))
        return distance

    def move_forward(distance: float) -> float:
        """16cm 步进: move_for 匀速直线, 返回实际里程增量."""
        nonlocal nominal_travel
        print(f"  [drive] move_for +{distance:.3f}m @{creep_speed:.2f} m/s", flush=True)
        delta = _move_for(distance)
        nominal_travel += delta
        print(f"  [drive]   actual +{delta:.3f}m", flush=True)
        return delta

    def move_along_lane(distance: float, stop_when=None, stopped=None) -> float:
        """首卡搜索: 沿车道匀速小步, 行驶中可提前停车; lane 不可用回退 move_for."""
        nonlocal nominal_travel
        if dry_run:
            nominal_travel += distance
            return distance
        from main.chassis.config import LANE_FOLLOW
        from main.chassis.controllers import move_along_lane as lane_move
        print(f"  [drive] move_along_lane +{distance:.3f}m @{creep_speed:.2f} m/s", flush=True)
        before = read_traveled(client)
        lane_error = None
        try:
            lane_state = client.get("/v1/vision/lane/state") or {}
            lane_error = lane_state.get("last_error")
        except Exception as exc:
            lane_error = f"lane state unavailable: {exc}"
        if lane_error:
            print(f"  [warn] lane feed unavailable: {lane_error}", file=sys.stderr, flush=True)
            delta = _move_for(distance)
            nominal_travel += delta
            return delta
        try:
            lane_move(
                vx=creep_speed,
                distance_m=distance,
                profile=LANE_FOLLOW.tuned(watchdog_ms=None),
                max_seconds=max(5.0, distance / max(creep_speed, 0.05) * 3.0 + 2.0),
                stop_when=stop_when,
            )
        except Exception as exc:
            # 2026-08-09: lane 偶发异常不能把整个识别任务打挂 → 回退 move_for.
            print(f"  [warn] move_along_lane 异常({exc}), 回退 move_for", file=sys.stderr, flush=True)
            delta = _move_for(distance)
            nominal_travel += delta
            return delta
        after = read_traveled(client)
        moved = (before is not None and after is not None
                 and after >= before + max(0.02, distance * 0.25))
        if not moved and not (stopped and stopped()):
            print("  [warn] move_along_lane 无里程变化, 回退 move_for", file=sys.stderr, flush=True)
            delta = _move_for(distance)
            nominal_travel += delta
            return delta
        delta = (after - before) if (before is not None and after is not None) else distance
        nominal_travel += max(0.0, delta)
        return max(0.0, delta)

    # ── 检测 / 锁存 ──
    detector = DetectionReader(client)
    detector.start()

    def fake_det() -> dict:
        return {"cls_id": 1, "det_id": 1, "label": "animal", "score": 1.0,
                "bbox_norm": {"x_center": 0.0, "y_center": 0.35,
                              "width": 0.1, "height": 0.1}}

    def eligible(det) -> bool:
        return (det.get("label") == "animal"
                and float(det.get("score") or 0.0) >= min_score)

    def is_complete(det, edge_tol=0.02) -> bool:
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

    def centered(window) -> Optional[dict]:
        """返回最接近画面中央的合格目标; window=None 时只看"完整入画", 不限 xc."""
        if dry_run:
            return fake_det()
        detections = [det for det in detector.read() if eligible(det)]
        if window is None:
            candidates = [det for det in detections if is_complete(det)]
        else:
            candidates = [det for det in detections
                          if is_complete(det)
                          and abs(animal_center(det)) <= window]
        return min(candidates, key=lambda d: abs(animal_center(d))) \
            if candidates else None

    def make_latch(window, required_samples=settle_samples, sample_period=period):
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
                animal_center(det) - animal_center(previous)) <= 0.12
            state["streak"] = state["streak"] + 1 if stable else 1
            state["det"] = dict(det)
            state["ready"] = state["streak"] >= required_samples
            if state["ready"]:
                print(f"  [center] xc={animal_center(det):+.3f} "
                      f"streak={state['streak']}/{required_samples}", flush=True)
            return state["ready"]

        return state, sample

    def wait_target(latch, timeout) -> Optional[dict]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if latch[1](force=True):
                return latch[0]["det"]
            time.sleep(period)
        return None

    def save_slot(det, number):
        image_path = None
        if not dry_run and streamer_url and output_dir:
            image_path = capture_target(streamer_url, det, 0.10, output_dir, number)
            if image_path is None:
                time.sleep(0.10)
                image_path = capture_target(streamer_url, det, 0.10, output_dir, number)
        records.append({
            "number": number,
            "image_path": str(image_path) if image_path else None,
            "detection": dict(det),
            "xc": animal_center(det),
            "yc": float(bbox(det).get("y_center", 0.0)),
            "score": float(det.get("score") or 0.0),
            "traveled_m": round(traveled_now(), 4),
            "timestamp": time.time(),
        })
        print(f"  [record] target #{number} xc={animal_center(det):+.3f} "
              f"at {traveled_now():.2f}m image={'saved' if image_path else 'missing'}",
              flush=True)

    def save_and_judge(det, number):
        """记录一张卡; 给了 token 且 max_llm_recapture>0 时当场 LLM 判定 + result=None 重拍."""
        save_slot(det, number)
        if token and max_llm_recapture > 0 and not dry_run:
            try:
                def classify(rec):
                    return classify_target(token, rec.get("image_path"),
                                           llm_timeout)

                def search():
                    nonlocal nominal_travel
                    det2, net = search_front_back(
                        detect_fn=lambda: wait_target(
                            make_latch(None, required_samples=2), slot_wait_s),
                        move_fn=_move_for,
                        step_m=recog_search_step_m,
                        back_max_m=recog_search_back_max_m,
                        fwd_max_m=recog_search_fwd_max_m,
                        label=f"目标 #{number} LLM 重拍",
                    )
                    nominal_travel += max(0.0, net)
                    return det2, net

                def recapture(det2):
                    image_path = None
                    if not dry_run and streamer_url and output_dir:
                        image_path = capture_target(streamer_url, det2, 0.10,
                                                    output_dir, number)
                    rec = records[-1]
                    rec["image_path"] = (str(image_path) if image_path
                                         else rec.get("image_path"))
                    rec["detection"] = dict(det2)
                    rec["xc"] = animal_center(det2)
                    rec["yc"] = float(bbox(det2).get("y_center", 0.0))
                    rec["score"] = float(det2.get("score") or 0.0)

                judge_and_recapture(records[-1], classify_fn=classify,
                                    search_fn=search, recapture_fn=recapture,
                                    max_retries=max_llm_recapture,
                                    label=f"目标 #{number}")
            except Exception as exc:
                # LLM/网络故障不能打断识别 → 留 result=None, 末尾兜底判定.
                print(f"[warn] 目标 #{number} 内联判定异常: {exc}",
                      file=sys.stderr, flush=True)

    try:
        # ── 1) 首卡: 2026-08-09 用户要求 —— 车到达识别区时已在第一张卡面前,
        #    起点直接识别"完整目标"(整张卡完整入画即可, 不需要在视野正中央):
        #    有 → 直接记录 #1; 没有 → 匀速缓慢前进(4cm/步)直到第一张卡完整入画.
        print(f"[recognition] 首卡定位: 起点即识别完整目标(不限居中), "
              f"没有则匀速 {creep_speed:.2f} m/s 缓慢前进 (最多 {max_travel_m:.2f}m)",
              flush=True)
        first_det = None
        best_det = None
        first_det = wait_target(make_latch(None, required_samples=2), slot_wait_s)
        if first_det is None:
            # **2026-08-12 用户需求**: 首卡识别不出 → 前后微调搜索,
            # 直到识别出来再继续 16cm 移动; 前后都找不到才走向前蠕行兜底.
            print("[recognition] 首卡未识别到, 前后微调搜索", flush=True)
            first_det, search_net = search_front_back(
                detect_fn=lambda: wait_target(
                    make_latch(None, required_samples=2), slot_wait_s),
                move_fn=_move_for,
                step_m=recog_search_step_m,
                back_max_m=recog_search_back_max_m,
                fwd_max_m=recog_search_fwd_max_m,
                label="首卡",
            )
            nominal_travel += max(0.0, search_net)
        while first_det is None and traveled_now() < max_travel_m:
            step = min(search_step_m, max_travel_m - traveled_now())
            move_latch = make_latch(None, required_samples=1,
                                    sample_period=0.05)
            move_along_lane(step,
                            stop_when=lambda *_: move_latch[1](),
                            stopped=lambda: move_latch[0]["ready"])
            time.sleep(max(period, 0.10))
            first_det = wait_target(make_latch(None, required_samples=2), slot_wait_s)
            if first_det is None and move_latch[0]["ready"]:
                first_det = move_latch[0]["det"]
            if first_det is None:
                nearest = centered(None)
                if nearest is not None:
                    if best_det is None or abs(animal_center(nearest)) < abs(
                            animal_center(best_det)):
                        best_det = dict(nearest)
                print(f"  [search] checked {traveled_now():.2f}m, 首卡尚未完整入画", flush=True)
        if first_det is None and best_det is not None:
            print(f"[warn] 首卡未完整入画稳定识别, 用最近目标补救 "
                  f"(xc={animal_center(best_det):+.3f})", file=sys.stderr, flush=True)
            first_det = best_det
        if first_det is None:
            print("[warn] 未找到首卡, 结束识别", file=sys.stderr, flush=True)
        else:
            save_and_judge(first_det, 1)
        if records and not dry_run:
            safe_stop(client, job_timeout)
            print("  [drive] 首卡已记录, 立即停车", flush=True)

        # ── 2) 步进: 每步先向前再稳定确认 → 记录;
        #    2026-08-09 用户: 按上一个目标的横向位置决定下一步长度 (最后一个目标除外):
        #    正中 → 16cm; 偏右 → 18cm (多走 2cm); 偏左 → 14cm (少走 2cm).
        while records and len(records) < target_count:
            number = len(records) + 1
            last_xc = records[-1].get("xc") or 0.0
            step = step_for_xc(last_xc, base=step_m, tol=center_window)
            side = ("偏右" if last_xc > center_window
                    else "偏左" if last_xc < -center_window else "居中")
            print(f"  [drive] 上张卡 {side} (xc={last_xc:+.3f}), 本次前进 {step:.2f}m",
                  flush=True)
            if traveled_now() + step > max_travel_m + 0.02:
                print("[warn] 已达识别区最大行程, 停止补录", file=sys.stderr, flush=True)
                break
            move_forward(step)
            det = wait_target(make_latch(slot_window), slot_wait_s)
            if det is None:
                # 2026-08-09 用户: 目标不在视野正中央不能跳过 →
                # 兜底: 只要有一张完整入画的卡就记录(选最接近中央的).
                det = wait_target(
                    make_latch(None, required_samples=2), slot_wait_s,
                )
            if det is None:
                nearest = centered(None)
                if nearest is not None:
                    det = nearest
            if det is None:
                # **2026-08-12 用户需求**: 识别不出目标 → 前后微调找目标,
                # 找到再继续 16cm 移动; 前后预算内都找不到才放弃该槽.
                print(f"[warn] 目标 #{number} 未识别到, 前后微调搜索",
                      file=sys.stderr, flush=True)
                det, search_net = search_front_back(
                    detect_fn=lambda: wait_target(
                        make_latch(None, required_samples=2), slot_wait_s),
                    move_fn=_move_for,
                    step_m=recog_search_step_m,
                    back_max_m=recog_search_back_max_m,
                    fwd_max_m=recog_search_fwd_max_m,
                    label=f"目标 #{number}",
                )
                if det is not None:
                    nominal_travel += max(0.0, search_net)
                    print(f"  [slot] #{number} 前后微调 {search_net:+.2f}m 后识别到",
                          flush=True)
                else:
                    print(f"[warn] 目标 #{number} 前后微调后仍未识别到, 放弃该槽",
                          file=sys.stderr, flush=True)
                    break
            save_and_judge(det, number)
    finally:
        detector.stop()
        if not dry_run:
            safe_stop(client, job_timeout)

    print(f"[recognition] driving complete; recorded {len(records)}/{target_count} targets")
    return records, traveled_now()


# ── 后台 LLM 判定 (识别这是什么害虫/益虫) ─────────────────────

def _judge_one(token, record, llm_timeout) -> dict:
    image_path = record.get("image_path")
    if not image_path or not Path(image_path).exists():
        return {**record, "species": "unknown", "classification": "unknown",
                "result": None, "analysis": "target image unavailable"}
    try:
        image_bytes = Path(image_path).read_bytes()
        image_url = "data:image/jpeg;base64," + base64.b64encode(image_bytes).decode("ascii")
        verdict = call_vision(token, image_url, PEST_PROMPT, timeout=llm_timeout)
        result = verdict.get("result")
        try:
            result = int(result) if result is not None else None
        except (TypeError, ValueError):
            result = None
        entry = dict(record)
        entry.pop("detection", None)
        entry.update({
            "species": str(verdict.get("name") or verdict.get("species") or "unknown"),
            "classification": "pest" if result == 0
            else "beneficial" if result == 1 else "unknown",
            "result": result,
            "analysis": str(verdict.get("analysis") or ""),
        })
        print(f"  -> target #{entry['number']}: {entry['species']} / "
              f"{entry['classification']}", flush=True)
        return entry
    except Exception as exc:
        return {**record, "species": "unknown", "classification": "unknown",
                "result": None, "analysis": f"judge error: {exc}"}


def _judge_all(token, records, llm_timeout) -> List[dict]:
    judged = []
    for index, record in enumerate(records):
        # 2026-08-12: 内联判定(含重拍)已出结果的直接保留, 不重判 (避免 LLM 非确定性覆盖)
        if record.get("result") in (0, 1):
            judged.append(record)
            continue
        time.sleep(index * 0.3)   # 错开发起时刻, 防 ERNIE 限流
        judged.append(_judge_one(token, record, llm_timeout))
    return judged


def judge_targets_background(manifest: Path, llm_timeout: float = DEFAULT_LLM_TIMEOUT) -> None:
    """后台线程: 读 pending json → 逐张 LLM 判定 → 回写 done json (不阻塞车)."""

    def _bg() -> None:
        try:
            payload = json.loads(manifest.read_text(encoding="utf-8"))
        except Exception as exc:
            print(f"[judge] 读 json 失败: {exc}", file=sys.stderr, flush=True)
            return
        if payload.get("status") == "done":
            return
        try:
            token = load_token()
        except Exception as exc:
            print(f"[judge] 无 ERNIE token, 判定失败: {exc}", file=sys.stderr, flush=True)
            return
        targets = payload.get("targets") or []
        print(f"[judge] 后台判定 {len(targets)} 个目标 (不阻塞车)...", flush=True)
        judged = _judge_all(token, targets, llm_timeout)
        payload["targets"] = judged
        payload["status"] = "done"
        payload["pest_numbers"] = [t["number"] for t in judged if t.get("result") == 0]
        payload["beneficial_numbers"] = [t["number"] for t in judged if t.get("result") == 1]
        manifest.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"[judge] done: pests={payload['pest_numbers'] or 'none'} "
              f"beneficial={payload['beneficial_numbers'] or 'none'}", flush=True)

    threading.Thread(target=_bg, daemon=True, name="task3-judge").start()


def save_pending(path_text, records, traveled, config: Dict[str, Any]) -> Path:
    path = Path(path_text)
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / path
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "config": config,
        "status": "pending",
        "targets": records,
        "pest_numbers": [],
        "beneficial_numbers": [],
        "traveled_m": round(traveled, 4),
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"[recognition] saved: {path} (status=pending)")
    return path


def run_arm_pose(args, label, pose, client=None) -> int:
    """摆臂 (arm_seq_v9); 已在目标姿态则跳过."""
    if client is not None and arm_at_pose(client, pose):
        print(f"[arm] {label}: 已在目标姿态, 跳过摆臂", flush=True)
        return 0
    command = [sys.executable, "-m", "main.task.task3.arm_seq_v9",
               "--y1", pose[0], "--y2", pose[1], "--x", pose[2],
               "--arm-angle", pose[3], "--hand-angle", pose[4]]
    print(f"[arm] {label}: {' '.join(command)}", flush=True)
    return subprocess.run(command, check=False).returncode


def main() -> int:
    parser = argparse.ArgumentParser(description="task3 识别段独立脚本 (只识别, 不射击)")
    parser.add_argument("--target-count", type=int, default=DEFAULT_TARGET_COUNT)
    parser.add_argument("--step-m", type=float, default=DEFAULT_STEP_M,
                        help=f"每次向前步进 (m, 默认 {DEFAULT_STEP_M} = 16cm)")
    parser.add_argument("--creep-speed", type=float, default=DEFAULT_CREEP_SPEED,
                        help="匀速前进速度 m/s (默认 0.18)")
    parser.add_argument("--center-window", type=float, default=DEFAULT_CENTER_WINDOW,
                        help="首卡居中窗口 (归一化 xc, 默认 0.15)")
    parser.add_argument("--slot-window", type=float, default=DEFAULT_SLOT_WINDOW,
                        help="后续卡确认窗口 (归一化 xc, 默认 0.35)")
    parser.add_argument("--search-step-m", type=float, default=DEFAULT_SEARCH_STEP_M)
    parser.add_argument("--recog-search-step-m", type=float,
                        default=DEFAULT_RECOG_SEARCH_STEP_M,
                        help="前后微调识别步长 m (默认 4cm)")
    parser.add_argument("--recog-search-back-max-m", type=float,
                        default=DEFAULT_RECOG_SEARCH_BACK_MAX_M,
                        help="前后微调向后上限 m (默认 12cm)")
    parser.add_argument("--recog-search-fwd-max-m", type=float,
                        default=DEFAULT_RECOG_SEARCH_FWD_MAX_M,
                        help="前后微调向前上限 m (默认 16cm)")
    parser.add_argument("--max-llm-recapture", type=int, default=0,
                        help="每张卡 LLM 判定 result=None 时前后微调重拍最大次数 "
                             "(默认 0 = 不内联判定; >0 时逐卡当场判定+重拍)")
    parser.add_argument("--settle-samples", type=int, default=DEFAULT_SETTLE_SAMPLES)
    parser.add_argument("--slot-wait-s", type=float, default=DEFAULT_SLOT_WAIT_S)
    parser.add_argument("--max-travel-m", type=float, default=DEFAULT_MAX_TRAVEL_M)
    parser.add_argument("--min-score", type=float, default=DEFAULT_MIN_SCORE)
    parser.add_argument("--poll-interval", type=float, default=DEFAULT_POLL_INTERVAL)
    parser.add_argument("--job-timeout", type=float, default=DEFAULT_JOB_TIMEOUT)
    parser.add_argument("--llm-timeout", type=float, default=DEFAULT_LLM_TIMEOUT)
    parser.add_argument("--save", default=str(DEFAULT_SAVE))
    parser.add_argument("--no-arm", action="store_true",
                        help="(历史遗留) 2026-08-12 起识别前不再摆臂, 该参数无实际作用")
    parser.add_argument("--judge-inline", action="store_true",
                        help="同步判定并回写 done (默认后台线程判定, 立即返回)")
    parser.add_argument("--token", default=None)
    parser.add_argument("--dry-run", action="store_true", help="模拟运行, 不动作臂/底盘/相机")
    args = parser.parse_args()

    if args.target_count < 1 or args.step_m <= 0 or not 0 < args.creep_speed <= 0.2:
        parser.error("target-count/step-m/creep-speed 参数无效")

    if args.dry_run:
        print("[dry-run] 模拟识别流程 (无硬件动作)")
        client = None
    else:
        client = RuntimeApiClient()
        if not client.wait_until_ready(timeout=10.0):
            print("runtime not ready (pm2 logs rak-car-api)", file=sys.stderr)
            return 2

    # 2026-08-12 用户指令: task3 识别前**不对机械臂做任何调整** (原 RECOGNITION_ARM 摆臂取消).
    # --no-arm 参数保留为历史遗留 (无实际作用), 需要恢复摆臂时改回原 run_arm_pose 调用.

    from main.settings import load_settings
    settings = load_settings()
    image_dir = Path(__file__).resolve().parent / "audit" / "task3_pipeline" / "targets"

    # 2026-08-12: 内联判定/重拍需要 token → 在识别前加载 (识别时逐卡判定用)
    token = ""
    if (args.judge_inline or args.max_llm_recapture > 0) and not args.dry_run:
        try:
            token = load_token(args.token)
        except Exception as exc:
            print(f"[warn] 无 ERNIE token, 内联判定/重拍禁用: {exc}",
                  file=sys.stderr, flush=True)
            token = ""

    records, traveled = recognize_targets(
        client,
        target_count=args.target_count,
        step_m=args.step_m,
        creep_speed=args.creep_speed,
        center_window=args.center_window,
        slot_window=args.slot_window,
        search_step_m=args.search_step_m,
        settle_samples=args.settle_samples,
        slot_wait_s=args.slot_wait_s,
        max_travel_m=args.max_travel_m,
        min_score=args.min_score,
        poll_interval=args.poll_interval,
        job_timeout=args.job_timeout,
        streamer_url=None if args.dry_run else settings.streamer_url,
        output_dir=None if args.dry_run else image_dir,
        dry_run=args.dry_run,
        recog_search_step_m=args.recog_search_step_m,
        recog_search_back_max_m=args.recog_search_back_max_m,
        recog_search_fwd_max_m=args.recog_search_fwd_max_m,
        token=token,
        llm_timeout=args.llm_timeout,
        max_llm_recapture=args.max_llm_recapture,
    )

    result_path = save_pending(args.save, records, traveled, vars(args))

    if args.dry_run:
        print("[dry-run] 结束 (不启动后台判定)")
        return 0

    if args.judge_inline:
        if not token:
            print("[judge] 无 ERNIE token, 跳过判定", file=sys.stderr, flush=True)
            return 0
        judged = _judge_all(token, records, args.llm_timeout)
        payload = json.loads(result_path.read_text(encoding="utf-8"))
        payload["targets"] = judged
        payload["status"] = "done"
        payload["pest_numbers"] = [t["number"] for t in judged if t.get("result") == 0]
        payload["beneficial_numbers"] = [t["number"] for t in judged if t.get("result") == 1]
        result_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2),
                               encoding="utf-8")
        print(f"[judge] inline done: pests={payload['pest_numbers'] or 'none'}")
        return 0

    judge_targets_background(result_path, args.llm_timeout)
    print("[recognition] 识别完成, 后台判定害虫种类中, 脚本立即返回", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
