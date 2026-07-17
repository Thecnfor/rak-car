"""main/chassis/tasks/auto_navigate.py
自动导航任务：外环巡线 + 视觉目标让位 + 安全兜底。

按用户指示（2026-07-16）：所有算法在 main/ 实现，runtime 只暴露
原始 lane_state / arm_state / task_state 数据，不参与决策。

与 examples/03_p2p_with_vision.py 的差异：
- example 是 5 秒超时的演示骨架
- 本任务是生产可用的完整流程：
  * 50Hz Stanley 外环持续巡线
  * 巡线中持续检测 task_feed 缓存的目标
  * 检测到目标 → 切到 track_target（视觉闭环对齐）
  * 对齐完成 → 回到外环继续巡线（exit_after_align=False）
              或退出（exit_after_align=True）
  * 失线 / watchdog / 急停 → 兜底
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import List, Optional

from ..api import ChassisClient
from ..controllers.base import OuterLoop
from ..loops.safety import EmergencyWatchdog, LostLineDetector
from ..state import LaneState
from . import track_target

logger = logging.getLogger(__name__)

# task_feed 检测结果字段顺序（与 smartcar/paddlebaidu 推理输出对齐）
# 格式: [cls_id, obj_id, label, score, x_c, y_c, w, h]
_DET_LABEL_INDEX = 2
_DET_SCORE_INDEX = 3
_DEFAULT_CONFIDENCE = 0.5


@dataclass
class AutoNavConfig:
    """自动导航任务配置。"""

    target_label: Optional[str] = None
    confidence_threshold: float = _DEFAULT_CONFIDENCE
    align_time_out: float = 3.0
    max_seconds: float = 30.0
    hz: float = 50.0
    exit_after_align: bool = False


def _select_target(detections, label: Optional[str], conf_threshold: float):
    """从 task_feed 检测结果中选一个最匹配的目标。返回 (index, det) 或 (None, None)。"""
    if not detections:
        return None, None
    candidates = []
    for idx, det in enumerate(detections):
        if label is not None and det[_DET_LABEL_INDEX] != label:
            continue
        score = det[_DET_SCORE_INDEX]
        if score < conf_threshold:
            continue
        candidates.append((idx, det, score))
    if not candidates:
        return None, None
    # 选置信度最高的
    candidates.sort(key=lambda x: x[2], reverse=True)
    return candidates[0][0], candidates[0][1]


def _safe_sense(api: ChassisClient) -> LaneState:
    """从 lane_feed HTTP 拉一次（异常时返回空 state 走 watchdog）。"""
    try:
        payload = api.get_lane_state()
    except Exception:
        return LaneState()
    return LaneState.from_lane_state_payload(payload or {})


def _fetch_task_detections(api: ChassisClient, timeout: float = 0.05):
    """从 task_feed HTTP 拉一次侧摄目标检测（轻量、不抢锁）。"""
    try:
        return api.http.get(f"{api.http.api_prefix}/vision/task/state", timeout=timeout)
    except Exception:
        return None


def run(
    api: ChassisClient,
    outer: OuterLoop,
    *,
    target_label: Optional[str] = None,
    confidence_threshold: float = _DEFAULT_CONFIDENCE,
    align_time_out: float = 3.0,
    max_seconds: float = 30.0,
    hz: float = 50.0,
    exit_after_align: bool = False,
) -> dict:
    """执行自动导航：外环 + 视觉让位 + 安全。

    流程：
      1. 复用 runtime 默认启的 lane_feed (20Hz) + task_feed (10Hz) 守护线程
      2. 50Hz Stanley 外环跑 max_seconds
      3. 每 tick 拉一次 task_feed 检测，命中目标则切到 track_target 对齐
      4. 对齐完成：exit_after_align=True 退出；否则回外环
      5. watchdog/lost_line 触发急停

    返回 dict 包含：status / aligned_count / steps / final_state
    """
    cfg = AutoNavConfig(
        target_label=target_label,
        confidence_threshold=confidence_threshold,
        align_time_out=align_time_out,
        max_seconds=max_seconds,
        hz=hz,
        exit_after_align=exit_after_align,
    )

    aligned_count = 0
    steps = 0
    final_state = None

    # 2026-07-16: runtime 默认启 lane_feed + task_feed 守护线程
    # （_create_car_locked line 352-369），客户端不需要 start_*/stop_*。
    deadline = time.monotonic() + cfg.max_seconds
    next_tick = time.monotonic()
    watchdog = EmergencyWatchdog(threshold_ms=500.0)
    lost_line = LostLineDetector()
    try:
        while time.monotonic() < deadline:
            steps += 1
            # 1. 拉 lane_state
            state = _safe_sense(api)
            final_state = state
            if watchdog.should_stop(state) or lost_line.should_alert(state):
                logger.warning("auto_navigate: safety triggered, emergency stop")
                api.emergency_stop()
                return {
                    "status": "safety_stop",
                    "steps": steps,
                    "aligned_count": aligned_count,
                    "final_state": state,
                }
            # 2. 拉 task 检测（侧摄）
            det_payload = _fetch_task_detections(api)
            dets = (det_payload or {}).get("detections") or []
            if dets:
                idx, det = _select_target(dets, cfg.target_label, cfg.confidence_threshold)
                if det is not None:
                    # 3. 检测到目标 → 让位 track_target
                    logger.info(
                        "auto_navigate: target hit idx=%d label=%s score=%.2f",
                        idx, det[_DET_LABEL_INDEX], det[_DET_SCORE_INDEX],
                    )
                    api.stop_wheel_speeds()
                    job = track_target.track_target(
                        api, label=cfg.target_label, delta_x=0.0,
                        time_out=cfg.align_time_out,
                    )
                    aligned_count += 1
                    if cfg.exit_after_align:
                        return {
                            "status": "aligned",
                            "steps": steps,
                            "aligned_count": aligned_count,
                            "final_state": state,
                            "align_job": job,
                        }
                    # 不退出则回到外环继续巡线
            # 4. 外环 + 下发
            dt = 1.0 / max(cfg.hz, 1.0)
            speeds = outer.step(state, dt)
            api.set_wheel_speeds(speeds)
            # 5. 调度
            next_tick += dt
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()
    finally:
        api.stop_wheel_speeds()
    return {
        "status": "timeout",
        "steps": steps,
        "aligned_count": aligned_count,
        "final_state": final_state,
    }


__all__ = ["run", "AutoNavConfig"]
