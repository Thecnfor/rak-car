#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/start/orchestrator.py

run.py 的实现后端：
  - 后台 A：50Hz 巡线外环 (受 running Event 控制)
  - 后台 B：20Hz 里程计累计
  - 主线程：顺序遍历 DEFAULT_WAYPOINTS，等待「IR + 里程计」双触发 → 暂停巡线
    → 调 task.run() → 恢复巡线 → 终点处 break。

所有 main.start 之外的脚本都不应该 import 本文件 —— 只服务于 run.py。
"""
from __future__ import annotations

import importlib
import logging
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

# 让 main.start.orchestrator 可被仓库根目录的 run.py 直接 import
_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.api_client import RuntimeApiClient
from main.chassis.api import ChassisClient
from main.chassis.controllers.base import WheelSmoother
from main.chassis.controllers.curvature_adaptive import CurvatureAdaptiveOuterLoop
from main.chassis.loops.safety import EmergencyWatchdog
from main.chassis.tasks.read_dis import read_dis
from main.chassis.tasks.read_ir import read_ir

logger = logging.getLogger("main.start.orchestrator")


@dataclass
class Waypoint:
    """一个任务点位。

    Attributes:
        name:           人类可读名字，会出现在日志。
        task_module:    任务模块路径（"main.tasks.auto_seeding"）；None 表示纯导航段。
        ir_threshold_m: IR 接近阈值（None 表示不参与 IR 判断）。
        ir_side:        IR 哪一侧触发："left" / "right" / "any"（默认 "right"）。
        dis_at_least_m: 累计里程计 ≥ 该值才算「到了这个点」（None 表示不参与）。
        trigger_op:     "AND"（默认，严格防误触）/ "OR"。
        pause_before_s: 触发后、调 task 前的停顿。
        pause_after_s:  任务跑完、恢复巡线前的停顿。
        is_finish:      True = 这是终点（里程计达到即整个流程结束）。
    """
    name: str
    task_module: Optional[str] = None
    ir_threshold_m: Optional[float] = None
    ir_side: str = "right"
    dis_at_least_m: Optional[float] = None
    trigger_op: str = "AND"
    pause_before_s: float = 0.0
    pause_after_s: float = 0.0
    is_finish: bool = False


# 默认 8 任务点位 + 1 终点。换场地改这里。
DEFAULT_WAYPOINTS: List[Waypoint] = [
    Waypoint("seed",        task_module="main.tasks.auto_seeding",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=1.20, trigger_op="AND"),
    Waypoint("scout_pests", task_module="main.tasks.scout_pests",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=3.50, trigger_op="AND"),
    Waypoint("water",       task_module="main.tasks.water_tower_task",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=5.20, trigger_op="AND"),
    Waypoint("shoot_pests", task_module="main.tasks.target_shooting",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=7.00, trigger_op="AND"),
    Waypoint("harvest",     task_module="main.tasks.crop_harvesting",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=9.00, trigger_op="AND"),
    Waypoint("sort",        task_module="main.tasks.sort_and_store",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=11.0, trigger_op="AND"),
    Waypoint("ocr",         task_module="main.tasks.get_order",
             ir_threshold_m=0.50, ir_side="left",
             dis_at_least_m=13.0, trigger_op="AND"),
    Waypoint("deliver",     task_module="main.tasks.order_delivery",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=14.5, trigger_op="AND"),
    # 终点：里程计达到 16.5m → 整个流程结束
    Waypoint("cruise_done", task_module=None, ir_threshold_m=None,
             dis_at_least_m=16.5, is_finish=True),
]


class Orchestrator:
    """巡线导航 + 任务点位调度器。"""

    def __init__(self,
                 waypoints: Optional[List[Waypoint]] = None,
                 lane_hz: float = 50.0,
                 ir_interval_s: float = 0.1):
        self.waypoints = waypoints if waypoints is not None else DEFAULT_WAYPOINTS
        self.lane_hz = lane_hz
        self.ir_interval_s = ir_interval_s

    def run(self) -> None:
        client = RuntimeApiClient()
        if not client.wait_until_ready(timeout=10.0):
            raise RuntimeError("runtime not ready (pm2 logs rak-car-api)")
        api = ChassisClient.connect()
        try:
            api.start_lane_feed(hz=self.lane_hz)
        except Exception as exc:
            logger.warning("start_lane_feed failed: %s", exc)

        # 后台 A：巡线外环（受 running Event 控制）
        running = threading.Event()
        running.set()
        threading.Thread(target=self._lane_loop,
                         args=(api, running),
                         daemon=True, name="lane").start()

        # 后台 B：里程计（全程累计，写共享 buffer）
        dis_buf = [0.0]
        threading.Thread(target=read_dis,
                         kwargs={"api": api, "hz": 20.0,
                                 "on_tick": lambda v: dis_buf.__setitem__(0, v)},
                         daemon=True, name="distance").start()

        completed: List[str] = []
        try:
            for wp in self.waypoints:
                logger.info("=== navigating to %s ===", wp.name)
                self._wait_until_triggered(wp, api, dis_buf)
                if wp.is_finish:
                    logger.info("finish waypoint reached (dis=%.2fm), mission done",
                                dis_buf[0])
                    completed.append(wp.name)
                    break
                self._pause_lane(api, running)
                time.sleep(wp.pause_before_s)
                if wp.task_module:
                    self._run_task(client, wp)
                time.sleep(wp.pause_after_s)
                self._resume_lane(running)
                completed.append(wp.name)
        except KeyboardInterrupt:
            logger.info("interrupted by user")
        finally:
            running.clear()
            try:
                api.stop_wheel_speeds()
            except Exception:
                pass
            try:
                api.stop_lane_feed()
            except Exception:
                pass
            api.close()
            logger.info("mission completed: %s", completed)

    # ── 后台线程 ────────────────────────────────────────────

    def _lane_loop(self, api: ChassisClient, running: threading.Event) -> None:
        """50Hz 巡线：读 lane → 控制律 → smoother → 下发。running.clear() 暂停。"""
        outer = CurvatureAdaptiveOuterLoop()
        smoother = WheelSmoother()
        watchdog = EmergencyWatchdog(threshold_ms=500.0)
        dt = 1.0 / max(self.lane_hz, 1.0)
        while True:
            running.wait()
            smoother.reset([0.0, 0.0, 0.0, 0.0])
            t0 = time.monotonic()
            state = api.read_lane()
            if watchdog.should_stop(state):
                try:
                    api.emergency_stop()
                except Exception:
                    pass
                time.sleep(dt)
                continue
            raw = outer.step(state, dt)
            safe = smoother.step(raw)
            try:
                api.set_wheel_speeds(safe)
            except Exception:
                pass
            sleep_s = dt - (time.monotonic() - t0)
            if sleep_s > 0:
                time.sleep(sleep_s)

    # ── 主线程辅助 ──────────────────────────────────────────

    @staticmethod
    def _pause_lane(api: ChassisClient, running: threading.Event) -> None:
        """暂停外环：clear Event + 主动发零速 + 等一帧让车端消化。"""
        running.clear()
        try:
            api.stop_wheel_speeds()
        except Exception:
            pass
        time.sleep(0.1)

    @staticmethod
    def _resume_lane(running: threading.Event) -> None:
        running.set()

    @staticmethod
    def _wait_until_triggered(wp: Waypoint, api: ChassisClient,
                              dis_buf: list, interval_s: float = 0.1) -> None:
        """轮询 IR + 里程计，直到 wp 的触发条件满足（默认 AND）。

        任一条件字段为 None 时视为「已满足」，避免任务永不触发。
        IR 分左右：wp.ir_side 取 left / right，"any" 表示两侧任一触发即可。
        """
        while True:
            ir: dict = {}
            try:
                ir = read_ir(api, timeout=2.0)
            except Exception:
                pass
            right = ir.get("right") if isinstance(ir, dict) else None
            left = ir.get("left") if isinstance(ir, dict) else None
            dis = dis_buf[0]

            if wp.ir_threshold_m is None:
                ir_ok = True
            elif wp.ir_side == "left":
                ir_ok = left is not None and left < wp.ir_threshold_m
            elif wp.ir_side == "any":
                ir_ok = ((left is not None and left < wp.ir_threshold_m) or
                         (right is not None and right < wp.ir_threshold_m))
            else:  # "right"
                ir_ok = right is not None and right < wp.ir_threshold_m

            dis_ok = (wp.dis_at_least_m is None or dis >= wp.dis_at_least_m)
            hit = (ir_ok and dis_ok) if wp.trigger_op == "AND" else (ir_ok or dis_ok)
            if hit:
                logger.info("triggered: %s (ir_left=%s ir_right=%s dis=%.2f)",
                            wp.name, left, right, dis)
                return
            time.sleep(interval_s)

    @staticmethod
    def _run_task(client: RuntimeApiClient, wp: Waypoint) -> None:
        """按需 import 任务模块，调 run()。失败不致命：记日志、继续下一站。"""
        try:
            mod = importlib.import_module(wp.task_module)
        except ImportError:
            logger.warning("task module %s not implemented, skipping", wp.task_module)
            return
        try:
            result = mod.run(client)
            logger.info("task %s -> %s", wp.name, result)
        except Exception:
            logger.exception("task %s failed", wp.name)


__all__ = ["Waypoint", "Orchestrator", "DEFAULT_WAYPOINTS"]