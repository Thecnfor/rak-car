#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/start/orchestrator.py

run.py 的实现后端（#1 重构）：
  - 后台 A：DoubleLoopRunner 50Hz 巡线外环（pause/resume 控制）
  - 后台 B：20Hz 里程计累计
  - 主线程：顺序遍历 DEFAULT_WAYPOINTS，等待「IR + 里程计」双触发 → 暂停巡线
    → 调 task.run() → 恢复巡线 → 终点处 break。

所有 main.start 之外的脚本都不应该 import 本文件 —— 只服务于 run.py。
"""
from __future__ import annotations

import logging
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

# 让 main.start.orchestrator 可被仓库根目录的 run.py 直接 import
_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.api_client import RuntimeApiClient
from main.chassis import LANE_FOLLOW
from main.chassis.api import ChassisClient
from main.chassis.loops.closed_loop import DoubleLoopRunner
from main.chassis.tasks.read_dis import read_dis
from main.chassis.tasks.read_ir import read_ir

logger = logging.getLogger("main.start.orchestrator")


@dataclass
class Waypoint:
    """一个任务点位.

    Attributes:
        name:           人类可读名字,会出现在日志 (位置参数,保持向后兼容).
        task_id:        任务编号 (1..7), 用于 TASK_RUNNERS[id] 查表. None 表示纯导航/finish.
        task_module:    (兼容旧字段,实际不再用 —— orchestrator 走 TASK_RUNNERS).
        ir_threshold_m: IR 接近阈值 (None 表示不参与 IR 判断).
        ir_side:        IR 哪一侧触发: "left" / "right" / "any" (默认 "right").
        dis_at_least_m: 累计里程计 ≥ 该值才算"到了这个点" (None 表示不参与).
        trigger_op:     "AND" (默认,严格防误触) / "OR".
        pause_before_s: 触发后、调 task 前的停顿.
        pause_after_s:  任务跑完、恢复巡线前的停顿.
        is_finish:      True = 这是终点 (里程计达到即整个流程结束).
    """
    name: str
    task_id: Optional[int] = None
    task_module: Optional[str] = None
    ir_threshold_m: Optional[float] = None
    ir_side: str = "right"
    dis_at_least_m: Optional[float] = None
    trigger_op: str = "AND"
    pause_before_s: float = 0.0
    pause_after_s: float = 0.0
    is_finish: bool = False


# 默认 8 任务点位 + 1 终点. 换场地改 task_config.yml 的 waypoints 段 (业务代码不动).
# 保留 DEFAULT_WAYPOINTS 作为 fallback —— 启动时优先从 yaml 加载.
DEFAULT_WAYPOINTS: List[Waypoint] = [
    Waypoint("task1_seeding",     task_id=1,
             ir_threshold_m=0.6, ir_side="right",
             dis_at_least_m=1.00, trigger_op="AND"),
    Waypoint("task2_water_tower", task_id=2,
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=5.20, trigger_op="AND"),
    Waypoint("task3_pest_scout",  task_id=3,
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=7.00, trigger_op="AND"),
    Waypoint("task4_harvest",     task_id=4,
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=9.00, trigger_op="AND"),
    Waypoint("task5_sort",        task_id=5,
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=11.0, trigger_op="AND"),
    Waypoint("task6_get_order",   task_id=6,
             ir_threshold_m=0.50, ir_side="left",
             dis_at_least_m=13.0, trigger_op="AND"),
    Waypoint("task7_deliver",     task_id=7,
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=14.5, trigger_op="AND"),
    # 终点: 里程计达到 16.5m → 整个流程结束
    Waypoint("cruise_done",       ir_threshold_m=None,
             dis_at_least_m=16.5, is_finish=True),
]


class Orchestrator:
    """巡线导航 + 任务点位调度器。"""

    def __init__(self,
                 waypoints: Optional[List[Waypoint]] = None,
                 lane_hz: float = 50.0,
                 ir_interval_s: float = 0.02,
                 config_path: Optional[str] = None):
        """config_path: 自定义 task_config.yml 路径, None 走默认 (根目录 task_config.yml)."""
        if waypoints is not None:
            self.waypoints = waypoints
        else:
            # 优先从 yaml 加载; 失败 fallback DEFAULT_WAYPOINTS
            self.waypoints = self._load_waypoints_from_yaml(config_path) or DEFAULT_WAYPOINTS
        self.lane_hz = lane_hz
        self.ir_interval_s = ir_interval_s

    @staticmethod
    def _load_waypoints_from_yaml(config_path: Optional[str]) -> Optional[List[Waypoint]]:
        """从 yaml 加载 waypoints, 失败返 None."""
        try:
            from main.task._config import load_waypoints
            wp_dicts = load_waypoints()
        except (FileNotFoundError, KeyError, ValueError) as exc:
            logger.warning("yaml load_waypoints failed, fallback DEFAULT_WAYPOINTS: %s", exc)
            return None
        out = []
        for w in wp_dicts:
            out.append(Waypoint(
                name=w.get("name", ""),
                task_id=w.get("task_id"),
                task_module=w.get("task_module"),  # 保留旧字段, 不参与 _run_task
                ir_threshold_m=w.get("ir_threshold_m"),
                ir_side=w.get("ir_side", "right"),
                dis_at_least_m=w.get("dis_at_least_m"),
                trigger_op=w.get("trigger_op", "AND"),
                is_finish=w.get("is_finish", False),
            ))
        return out

    def run(self) -> None:
        client = RuntimeApiClient()
        if not client.wait_until_ready(timeout=10.0):
            raise RuntimeError("runtime not ready (pm2 logs rak-car-api)")
        api = ChassisClient.connect()
        try:
            api.start_lane_feed(hz=self.lane_hz)
        except Exception as exc:
            logger.warning("start_lane_feed failed: %s", exc)

        # 确保 ir_feed 就绪（#seed-overshoot）：首次 read_ir 走 fast-path，
        # 避免 fallback 到 sync execute(timeout=2s) 导致 0.6m 盲区。
        _ir_deadline = time.time() + 5.0
        while time.time() < _ir_deadline:
            ir_state = api.get_ir_state()
            if ir_state.active:
                logger.info("ir_feed active (age_ms=%s)", ir_state.age_ms)
                break
            time.sleep(0.2)
        else:
            logger.warning("ir_feed not active after 5s, IR reads may use slow fallback")

        # 用 LANE_FOLLOW profile 装配 DoubleLoopRunner（#1）
        # —— 不再自己 new CurvatureAdaptiveOuterLoop + WheelSmoother。
        profile = LANE_FOLLOW
        runner = DoubleLoopRunner(
            api=api,
            outer=profile.build_outer(),
            hz=self.lane_hz,
            watchdog_ms=profile.watchdog_ms,
            lost_line_ms=profile.lost_line_ms,
            smoother=profile.build_smoother(),
        )

        # 后台 A：DoubleLoopRunner 50Hz 巡线（#1：用 runner.pause/resume 控制暂停）
        # max_seconds=inf：常驻，由 runner.stop() 终止
        runner_thread = threading.Thread(
            target=runner.run,
            kwargs={"max_seconds": math.inf},
            daemon=True, name="lane",
        )
        runner_thread.start()

        # 后台 B：里程计（全程累计，写共享 buffer）
        dis_buf = [0.0]
        threading.Thread(target=read_dis,
                         kwargs={"api": api, "hz": 20.0,
                                 "on_tick": lambda v: dis_buf.__setitem__(0, v)},
                         daemon=True, name="distance").start()

        # 后台 C：TUI 状态栏（200ms 刷新）
        tui_running = threading.Event()
        tui_running.set()
        tui_buf: List[Dict[str, Any]] = [{"wp": "", "dis": 0.0,
                                          "ir_left": None, "ir_right": None,
                                          "state": "init"}]
        threading.Thread(target=self._tui_loop,
                         args=(tui_buf, tui_running),
                         daemon=True, name="tui").start()

        completed: List[str] = []
        try:
            for wp in self.waypoints:
                logger.info("=== navigating to %s ===", wp.name)
                self._wait_until_triggered(wp, api, dis_buf, tui_buf,
                                           interval_s=self.ir_interval_s)
                if wp.is_finish:
                    logger.info("finish waypoint reached (dis=%.2fm), mission done",
                                dis_buf[0])
                    tui_buf[0] = {"wp": wp.name, "dis": dis_buf[0],
                                  "ir_left": None, "ir_right": None,
                                  "state": "done"}
                    completed.append(wp.name)
                    break
                self._pause_lane(runner, api)
                time.sleep(wp.pause_before_s)
                if wp.task_module:
                    tui_buf[0] = {"wp": wp.name, "dis": dis_buf[0],
                                  "ir_left": None, "ir_right": None,
                                  "state": "task"}
                    ok = self._run_task(client, wp)
                    if not ok:
                        logger.warning("task %s did not succeed, continuing to next waypoint", wp.name)
                time.sleep(wp.pause_after_s)
                self._resume_lane(runner)
                completed.append(wp.name)
        except KeyboardInterrupt:
            logger.info("interrupted by user")
        finally:
            # 终止 runner（#1）：stop() 唤醒 _pause.wait() 让 run() 看到 _stop，
            # join 等其 finally 块跑完（smoother 归零 + api.close()）。
            # 同时关掉 TUI 后台线程.
            tui_running.clear()
            try:
                api.stop_wheel_speeds()
            except Exception:
                pass
            runner.stop()
            runner_thread.join(timeout=2.0)
            try:
                api.stop_lane_feed()
            except Exception:
                pass
            # 注意：不要再调 api.close() —— runner 的 finally 已经调过了
            logger.info("mission completed: %s", completed)

# ── 后台线程 ────────────────────────────────────────────

    @staticmethod
    def _tui_loop(tui_buf: List[Dict[str, Any]],
                  tui_running: threading.Event) -> None:
        """每 200ms 刷一行 TUI 状态栏到终端。"""
        while tui_running.wait():
            info = tui_buf[0]
            state = info.get("state", "?")
            wp = info.get("wp", "")
            dis = info.get("dis", 0.0)
            il = info.get("ir_left")
            ir = info.get("ir_right")
            il_s = f"{il:.2f}" if il is not None else "---"
            ir_s = f"{ir:.2f}" if ir is not None else "---"
            line = f"\r[{state}] wp={wp:<14s} | dis={dis:6.2f}m | IR L:{il_s} R:{ir_s}   "
            sys.stdout.write(line)
            sys.stdout.flush()
            time.sleep(0.2)
        # 退出时换行
        sys.stdout.write("\n")
        sys.stdout.flush()

    # ── 主线程辅助 ──────────────────────────────────────────

    @staticmethod
    def _pause_lane(runner: DoubleLoopRunner, api: ChassisClient) -> None:
        """暂停外环（#1）：runner.pause() 阻塞下一帧 + 主动发零速兜底。"""
        runner.pause()
        try:
            api.stop_wheel_speeds()
        except Exception:
            pass

    @staticmethod
    def _resume_lane(runner: DoubleLoopRunner) -> None:
        """恢复外环（#1）：resume() 唤醒 + 清 smoother 记忆，从静止起步。"""
        runner.resume()

    @staticmethod
    def _wait_until_triggered(wp: Waypoint, api: ChassisClient,
                              dis_buf: list, tui_buf: List[Dict[str, Any]],
                              interval_s: float = 0.02) -> None:
        """轮询 IR + 里程计，直到 wp 的触发条件满足（默认 AND）。

        任一条件字段为 None 时视为「已满足」，避免任务永不触发。
        IR 分左右：wp.ir_side 取 left / right，"any" 表示两侧任一触发即可。
        每轮更新 tui_buf 供 TUI 线程读取。
        """
        while True:
            ir: dict = {}
            try:
                ir = read_ir(api, timeout=0.5)  # fast-path 不受影响；fallback 盲区从 0.6m→0.15m
            except Exception:
                pass
            right = ir.get("right") if isinstance(ir, dict) else None
            left = ir.get("left") if isinstance(ir, dict) else None
            dis = dis_buf[0]

            # 更新 TUI
            tui_buf[0] = {"wp": wp.name, "dis": dis,
                          "ir_left": left, "ir_right": right,
                          "state": "nav"}

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
    def _run_task(client: RuntimeApiClient, wp: Waypoint) -> bool:
        """按 task_id 查 TASK_RUNNERS 字典, 调 run(). 返回 True 表示成功."""
        if wp.task_id is None:
            # 纯导航段或 finish, 不应到这里
            return True
        try:
            from main.task import TASK_RUNNERS
            runner = TASK_RUNNERS[wp.task_id]
        except (ImportError, KeyError) as exc:
            logger.warning("task_id=%d not registered in TASK_RUNNERS: %s",
                           wp.task_id, exc)
            return False
        try:
            result = runner(client)
        except NotImplementedError as exc:
            # 未实现 task (3/7) 抛 NotImplementedError, warning + 跳过
            logger.warning("task_id=%d not implemented, skipping: %s",
                           wp.task_id, exc)
            return False
        except Exception:
            logger.exception("task %s raised exception", wp.name)
            return False
        if isinstance(result, dict) and not result.get("ok"):
            logger.warning("task %s failed: %s", wp.name,
                           result.get("error", result.get("detail", "?")))
            return False
        logger.info("task %s (id=%d) succeeded -> %s", wp.name, wp.task_id, result)
        return True


__all__ = ["Waypoint", "Orchestrator", "DEFAULT_WAYPOINTS"]