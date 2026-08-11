#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""runtime 侧巡线导航环 —— 进程内闭环，无每帧网络往返。

2026-08-11 把 run.py 的 50Hz lane-follow 外环下沉到 runtime：
客户端只发低频 start/pause/resume/stop，控制环本身在 runtime 进程内
读 streamer 缓存(lane/odom) + 直发轮速，完全免疫客户端网络抖动 / 时钟漂移
（跨机墙钟算 age 的坑在进程内不复存在——同机时钟）。

复用 main.chassis.loops.closed_loop.DoubleLoopRunner —— 控制律 / smoother /
watchdog / 弯道阶梯 100% 复用，零行为漂移。RuntimeLaneIo 只是给它一个
进程内 I/O 适配器（实现它依赖的 6 个 api 方法）。

main.* 全部懒 import（函数内部）：runtime 启动时不应因 main 不可 import 崩溃，
只有真正 start_lane_nav 时才拉 main.chassis（方向与 task2_runner 一致）。
"""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any, Dict, Optional

logger = logging.getLogger("runtime.lane_nav")


class RuntimeLaneIo:
    """DoubleLoopRunner 的进程内 I/O 适配器（替代客户端 ChassisClient）。

    全部委托给 CarRuntimeService 的现成 realtime 方法：
      - read_lane / get_odometry_state：读 streamer 缓存（同机时钟，age 正确）
      - set_wheel_speeds / emergency_stop：经 _realtime_gate 取**当前** car 直发
        （MC602 重启重建 car 后自动指向新实例，自愈）
      - start_lane_subscription：进程内恒新鲜，无需订阅 → True
      - close：只发零速（service 归 service，不关任何东西）
    """

    def __init__(self, svc):
        self._svc = svc

    def start_lane_subscription(self) -> bool:
        return True

    def read_lane(self):
        from main.chassis.state import LaneState
        try:
            payload = self._svc.get_lane_state()
        except Exception:
            return LaneState()
        if not isinstance(payload, dict):
            return LaneState()
        return LaneState.from_lane_state_payload(payload)

    def get_odometry_state(self):
        from main.chassis.state import OdometryState
        try:
            payload = self._svc.get_odom_state()
        except Exception:
            return OdometryState()
        if not isinstance(payload, dict):
            return OdometryState()
        return OdometryState.from_odom_state_payload(payload)

    def set_wheel_speeds(self, speeds) -> dict:
        return self._svc.set_wheel_speeds([float(s) for s in speeds])

    def emergency_stop(self) -> bool:
        return self._svc.emergency_stop()

    def close(self) -> None:
        try:
            self._svc.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])
        except Exception:
            pass


def _build_runner(
    svc,
    *,
    hz: float = 50.0,
    controller_type: str = "straight",
    turn_cfg: Optional[Dict[str, Any]] = None,
    watchdog_ms: Optional[float] = 500.0,
    lost_line_ms: Optional[float] = None,
    crossroad_turn: Optional[int] = None,
    on_tick=None,
):
    """装配 DoubleLoopRunner —— 镜像 orchestrator._init_mission 的构造。

    参数与客户端 orchestrator 传的一致：controller_type 决定外层控制律，
    turn_cfg 从 task_config.yml 的 turn: 段来（缺段回退类默认），
    watchdog/lost_line 来自 LANE_FOLLOW profile。换场地调 yml，代码不动。
    """
    from main.chassis.config import ControllerType, LaneFollowProfile
    from main.chassis.controllers.odom_turn import CurveDetector, StaircaseTurn
    from main.chassis.loops.closed_loop import DoubleLoopRunner

    if isinstance(controller_type, str):
        controller_type = ControllerType(controller_type)
    profile = LaneFollowProfile(controller_type=controller_type, hz=float(hz))
    turn_cfg = turn_cfg or {}
    return DoubleLoopRunner(
        api=RuntimeLaneIo(svc),
        outer=profile.build_outer(),
        hz=float(hz),
        watchdog_ms=watchdog_ms,
        lost_line_ms=lost_line_ms,
        smoother=profile.build_smoother(),
        turn=StaircaseTurn(**turn_cfg.get("staircase", {})),
        detector=CurveDetector(**turn_cfg.get("detector", {})),
        crossroad_turn=crossroad_turn,
        on_tick=on_tick,
    )


def _new_health() -> Dict[str, Any]:
    """导航环心跳：state 端点 / 调试用（iter_count 每帧递增 = 环活着）。"""
    return {
        "alive": False,
        "started_at": time.time(),
        "last_iter_at": 0.0,
        "iter_count": 0,
        "err_count": 0,
        "last_err": None,
    }


class LaneNavController:
    """持有 DoubleLoopRunner + daemon 线程的导航环生命周期。

    全局保证只有一条导航环（_lock + 幂等 start）：
      - start：已跑 → already_running；线程不存在 / 已死（watchdog break）→ 重建
      - pause：原生同步语义（先补发零速再 ack），进任务点前停得干净
      - resume：loop 已死 → 用上次参数自动重建重启（覆盖 watchdog 急停退出）
      - stop：runner.stop()（finally 内零速）→ 置空
    """

    def __init__(self, svc):
        self._svc = svc
        self._lock = threading.Lock()
        self._runner: Optional[Any] = None
        self._thread: Optional[threading.Thread] = None
        self._params: Dict[str, Any] = {}
        self._health = _new_health()

    # ── helpers ────────────────────────────────────────────────
    def _thread_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def _on_tick(self, state, safe) -> None:
        """DoubleLoopRunner 每帧回调：只更新心跳，不日志（50Hz 不刷屏）。"""
        h = self._health
        h["last_iter_at"] = time.time()
        h["iter_count"] += 1

    # ── lifecycle ──────────────────────────────────────────────
    def start(self, **params) -> dict:
        with self._lock:
            if self._thread_alive():
                return {"started": False, "reason": "already_running"}
            self._params = dict(params)
            self._health = _new_health()
            try:
                runner = _build_runner(self._svc, on_tick=self._on_tick, **params)
            except Exception as exc:
                self._health["last_err"] = "build: {}".format(exc)
                self._health["err_count"] += 1
                logger.warning("lane_nav build failed: %s", exc)
                return {"started": False, "reason": "build_failed", "error": str(exc)}
            self._runner = runner
            self._health["alive"] = True

            def _loop() -> None:
                try:
                    runner.run(max_seconds=math.inf)  # 阻塞到 stop / watchdog break
                except Exception as exc:
                    self._health["last_err"] = str(exc)
                    self._health["err_count"] += 1
                    logger.warning("lane_nav loop crashed: %s", exc)
                finally:
                    self._health["alive"] = False

            self._thread = threading.Thread(target=_loop, name="lane-nav", daemon=True)
            self._thread.start()
            return {"started": True, "hz": float(params.get("hz", 50.0))}

    def pause(self, timeout: float = 1.0) -> dict:
        runner = self._runner
        if not self._thread_alive():
            return {"paused": True, "reason": "not_running"}
        if runner is None:
            return {"paused": True, "reason": "no_runner"}
        try:
            ok = runner.pause(timeout=float(timeout))
        except Exception as exc:
            logger.warning("lane_nav pause failed: %s", exc)
            return {"paused": False, "error": str(exc)}
        return {"paused": bool(ok)}

    def resume(self, **overrides) -> dict:
        if self._thread_alive() and self._runner is not None:
            try:
                self._runner.resume()
                return {"resumed": True}
            except Exception as exc:
                logger.warning("lane_nav resume failed: %s", exc)
                return {"resumed": False, "error": str(exc)}
        # loop 已死（watchdog 急停等）→ 用上次参数自动重建重启
        if not self._params:
            return {"resumed": False, "reason": "never_started"}
        params = dict(self._params)
        params.update({k: v for k, v in (overrides or {}).items() if v is not None})
        st = self.start(**params)
        return {"resumed": bool(st.get("started")), "restarted": True}

    def stop(self, force: bool = True) -> dict:
        runner = self._runner
        thread = self._thread
        with self._lock:
            self._runner = None
            self._thread = None
            self._params = {}
        if runner is not None:
            try:
                runner.stop()
            except Exception as exc:
                logger.warning("lane_nav stop failed: %s", exc)
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._health["alive"] = False
        # 兜底零速（runner.finally 已零速，这里再补一次）
        try:
            self._svc.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])
        except Exception:
            pass
        return {"stopped": True}

    def state(self) -> dict:
        runner = self._runner
        try:
            paused = bool(runner is not None and runner.is_paused())
        except Exception:
            paused = False
        return {
            "running": self._thread_alive(),
            "paused": paused,
            "health": dict(self._health),
        }
