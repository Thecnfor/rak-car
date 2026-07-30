#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/start/ir_pause_resume.py

流程：巡线 → 右侧 IR 检测任务点 → 暂停巡线 → 恢复巡线（不执行任何任务）。

架构：
- 巡线线程：持续跑外环控制律（CurvatureAdaptiveOuterLoop + WheelSmoother）
- 主线程：轮询右侧 IR，命中阈值后暂停巡线、等待片刻、再恢复
- 通过 ``threading.Event`` 协调暂停 / 恢复
"""

from __future__ import annotations

import logging
import sys
import threading
import time
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from main.chassis import (
    ChassisClient,
    CurvatureAdaptiveOuterLoop,
    EmergencyWatchdog,
    WheelSmoother,
    read_dis,
    read_ir,
)

logger = logging.getLogger("start.ir_pause_resume")

# ---- TUI 常量 ----
_TUI_LINES = 8  # TUI 显示行数（含距离一行）


def _tui_print(
    status: str,
    ir_threshold_m: float,
    left: float | None,
    right: float | None,
    distance: float = 0.0,
    err_msg: str = "",
    width: int = 40,
) -> None:
    """TUI 刷新显示：状态 + 距离 + 左右 IR 柱状图 + 错误提示。"""
    def _bar(val: float | None) -> tuple[str, str]:
        if val is None:
            return "", "  N/A  "
        n = min(int(val * 10), width)
        bar = "█" * n + " " * (width - n)
        return bar, f"{val:.3f} m"

    bar_l, label_l = _bar(left)
    bar_r, label_r = _bar(right)

    triggered = right is not None and right < ir_threshold_m
    trigger_mark = "  ← 触发!" if triggered else ""

    lines = [
        f"  IR 触发暂停 — 巡线 + IR 检测 + 里程计",
        f"  {'─' * (width + 34)}",
        f"  状态    │ {status}",
        f"  距离    │ {distance:6.3f} m",
        f"  阈值    │ 右侧 IR < {ir_threshold_m:.3f} m 触发",
        f"  Left  IR │ {bar_l} │ {label_l}",
        f"  Right IR │ {bar_r} │ {label_r}{trigger_mark}",
        f"  {'─' * (width + 34)}",
    ]
    if err_msg:
        lines.append(f"  错误: {err_msg[:60]}")

    print("\n".join(f"{line}\033[K" for line in lines))


def _lane_loop(
    api: ChassisClient,
    running: threading.Event,
    hz: float = 50.0,
) -> None:
    """后台巡线线程。

    ``running`` 为 True 时持续跑外环；为 False 时阻塞等待。
    控制链：read_lane → outer.step → smoother.step → set_wheel_speeds。
    兜底：EmergencyWatchdog 检测 lane_state 过期（> 500ms）时急停。
    """
    outer = CurvatureAdaptiveOuterLoop()
    smoother = WheelSmoother()
    watchdog = EmergencyWatchdog(threshold_ms=500.0)
    dt = 1.0 / max(hz, 1.0)

    while True:
        running.wait()

        smoother.reset([0.0, 0.0, 0.0, 0.0])
        t0 = time.monotonic()

        state = api.read_lane()

        if watchdog.should_stop(state):
            logger.warning("lane data stale (age=%.0f ms), emergency stop", state.age_ms or -1)
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

        elapsed = time.monotonic() - t0
        sleep_s = dt - elapsed
        if sleep_s > 0:
            time.sleep(sleep_s)


def run(
    ir_threshold_m: float = 0.45,
    ir_interval_s: float = 0.1,
    lane_hz: float = 50.0,
    pause_duration_s: float = 1.0,
) -> None:
    """主入口：巡线 + IR 检测 → 暂停 → 恢复（不执行任务）。

    Args:
        ir_threshold_m: 右侧 IR 触发阈值（m）。
        ir_interval_s: IR 采样间隔（s），默认 0.1s（10Hz）。
        lane_hz: 巡线外环频率，默认 50Hz。
        pause_duration_s: 触发后暂停时长（s），默认 1.0s。
    """
    # ---- 0. 初始化 ----
    api = ChassisClient.connect()
    api.start_lane_feed(hz=lane_hz)
    logger.info("lane_feed started @ %.0f Hz", lane_hz)

    # ---- 1. 起巡线后台线程 ----
    running = threading.Event()
    running.set()

    lane_thread = threading.Thread(
        target=_lane_loop,
        args=(api, running, lane_hz),
        daemon=True,
        name="lane",
    )
    lane_thread.start()
    logger.info("lane loop started")

    # ---- 1.5. 起里程计后台线程 ----
    _dis_buf: list[float] = [0.0]  # 共享缓冲区，read_dis 回调写入，主线程读取

    def _on_dis(value: float):
        if value == value:  # 排除 NaN
            _dis_buf[0] = value

    dis_thread = threading.Thread(
        target=read_dis,
        kwargs={"api": api, "hz": 20.0, "on_tick": _on_dis},
        daemon=True,
        name="distance",
    )
    dis_thread.start()
    logger.info("distance reader started")

    # ---- 2. TUI 初始化 ----
    for _ in range(_TUI_LINES):
        print()
    print(f"\033[{_TUI_LINES}A", end="", flush=True)

    last_left: float | None = None
    last_right: float | None = None
    err_msg = ""

    try:
        while True:
            t0 = time.monotonic()

            # ---- 3. 读左右 IR ----
            try:
                ir_data = read_ir(api, timeout=2.0)
                left = float(ir_data.get("left", 0.0))
                right = float(ir_data.get("right", 0.0))
                last_left = left
                last_right = right
                err_msg = ""
            except Exception as e:
                left = last_left
                right = last_right
                err_msg = str(e)

            # ---- 4. TUI 刷新 ----
            _tui_print("巡线中", ir_threshold_m, left, right, _dis_buf[0], err_msg)

            # ---- 5. 检测触发 ----
            if right is not None and right < ir_threshold_m:
                logger.info(
                    "IR triggered: right=%.3f m < %.3f m → pausing",
                    right,
                    ir_threshold_m,
                )

                # TUI 刷新为"暂停中"
                _tui_print("暂停中", ir_threshold_m, left, right, _dis_buf[0], err_msg)

                # ---- 6. 暂停巡线 ----
                running.clear()
                time.sleep(5)  # 等巡线线程停到 wait()
                try:
                    api.stop_wheel_speeds()
                except Exception:
                    pass
                logger.info("lane paused (%.1f s)", pause_duration_s)

                # ---- 7. 等待后恢复 ----
                time.sleep(pause_duration_s)

                logger.info("resuming lane")
                running.set()
                err_msg = ""

            # 光标回到 TUI 第一行覆盖刷新
            print(f"\033[{_TUI_LINES}A", end="", flush=True)

            # 保持 IR 采样间隔
            elapsed = time.monotonic() - t0
            sleep_s = ir_interval_s - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    except KeyboardInterrupt:
        logger.info("stopped by user")
    finally:
        running.clear()
        try:
            api.stop_wheel_speeds()
        except Exception:
            pass
        try:
            api.close()
        except Exception:
            pass
        print("\n" * _TUI_LINES + "退出。")
        logger.info("cleanup done")


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(message)s",
    )
    run()
