"""task4 / target4 —— 后台保前移线程 + 底盘速度 helper。

从 target4.py 拆出 (2026-08-10 拆分): 单一职责 = "边前移边扫球" 的并发/状态逻辑。
- ``_CreepThread``       后台线程保底盘前移 + 主线程摆臂; 见球即停 / 走满距离预算即停。
- ``_set_chassis_vel``   下一次 chassis 速度 (realtime 门, 与 track_chassis 同通道)。

2026-08-10 用户拍板简化:
  - 速度 0.12 → 0.05 m/s (慢扫, 减少运动模糊/漏检)。
  - 删除红外离区生命周期 (左 IR >0.7m 判离区 + 再走 0.3m 收工) —— 暂不启用。
  - 删除"里程计卡死超 1s 退回速度×时间"的开环外推 —— 距离只认里程计, 不认时间。
  - 删除单次 creep 30s 墙钟上限 —— 走满距离预算 (max_distance_m) 是收尾依据。
"""
from __future__ import annotations

import sys
import time
from typing import Optional

from . import target2  # noqa: E402
from .constants import (  # noqa: E402
    COLOR_BLUE, COLOR_YELLOW,
    CREEP_POLL_HZ,
    LOG_PREFIX_TARGET4 as LOG_PREFIX,
)


class _CreepThread:
    """后台线程保底盘前移 + 主线程摆臂; 见球即停 / 走满距离预算即停。"""

    def __init__(self, http_client, *, speed_mps: float, max_distance_m: float,
                 poll_hz: float = CREEP_POLL_HZ):
        import threading
        self.http = http_client
        self.speed_mps = speed_mps
        self.max_distance_m = max_distance_m
        self.poll_hz = poll_hz
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="task4-creep")
        self._stop_event = threading.Event()
        self.completion_event = threading.Event()
        self.distance_m = 0.0
        self.elapsed_s = 0.0
        self.balls = None
        self.found_ball = False
        self.distance_exhausted = False
        self._odo_start_x = None

    def start(self) -> None:
        # 记录启动时里程计 x，后续闭环累加。
        try:
            odo = self.http.get_odom_state() or {}
            odo_data = odo.get("odom_state") or {}
            self._odo_start_x = odo_data.get("x")
        except Exception:
            self._odo_start_x = None
        self._thread.start()

    def _loop(self) -> None:
        period = 1.0 / max(self.poll_hz, 1.0)
        t0 = time.monotonic()
        try:
            while not self._stop_event.is_set():
                try:
                    self.http.post(
                        "/v1/realtime/chassis-velocity",
                        {"vx": float(self.speed_mps), "vy": 0.0, "wz": 0.0},
                        timeout=1.0,
                    )
                except Exception:
                    pass
                time.sleep(period)

                # 走多远只认里程计增量; 读不到 / 卡死都不外推 (2026-08-10 删开环回退)。
                if self._odo_start_x is not None:
                    try:
                        odo = self.http.get_odom_state() or {}
                        odo_data = odo.get("odom_state") or {}
                        current_x = odo_data.get("x")
                        if current_x is not None:
                            self.distance_m = max(
                                self.distance_m,
                                max(0.0, current_x - self._odo_start_x),
                            )
                    except Exception:
                        pass  # 读不到就维持上一帧
                self.elapsed_s = time.monotonic() - t0

                # 搜索阶段优先保证"看到球就停"，避免运动模糊/距离变化
                # 让过严的静态框阈值把真球过滤掉。
                try:
                    balls = target2.fetch_balls(
                        self.http, color_filter=None,
                        score_min=0.35,
                        aspect_tol=1.0,
                        area_min=0.03,
                        area_max=0.90,
                        debug=True,
                    )
                    if any(b.get("color") in (COLOR_BLUE, COLOR_YELLOW)
                           for b in balls):
                        self.balls = balls
                        self.found_ball = True
                        try:
                            self.http.post(
                                "/v1/realtime/chassis-velocity",
                                {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                                timeout=0.5,
                            )
                        except Exception:
                            pass
                        break
                except Exception as e:
                    print(f"  [{LOG_PREFIX}] fetch_balls 异常: "
                          f"{type(e).__name__}: {str(e)[:100]}", file=sys.stderr)

                # 距离预算走满 → 停 (预算内没看到球 = 采区扫空, 上层判 zone_cleared)。
                # 放在 fetch 之后: 预算边界与球同帧出现时, 见球优先 (先抓眼前这颗)。
                if self.distance_m >= self.max_distance_m:
                    self.distance_exhausted = True
                    break
        finally:
            try:
                self.http.post(
                    "/v1/realtime/chassis-velocity",
                    {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                    timeout=1.0,
                )
            except Exception:
                pass
            self.completion_event.set()

    def wait_for_ball(self, timeout_s: float) -> dict:
        """阻塞等见球, 见球/走满预算/超时返回。"""
        got = self.completion_event.wait(timeout=timeout_s)
        return {
            "balls": self.balls if got and self.found_ball else None,
            "distance_exhausted": bool(self.distance_exhausted),
            "distance_m": self.distance_m,
            "elapsed_s": self.elapsed_s,
        }

    def stop_and_join(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self._thread.is_alive():
            try:
                self.http.post(
                    "/v1/realtime/chassis-velocity",
                    {"vx": 0.0, "vy": 0.0, "wz": 0.0},
                    timeout=1.0,
                )
            except Exception:
                pass


# ---------- 底盘速度 helper ----------

def _set_chassis_vel(http_client, vx: float, vy: float = 0.0) -> None:
    """下一次 chassis 速度 (realtime 门, 与 track_chassis 同通道)。

    异常只 warn 不抛 —— creep 是搜索阶段, 单次下发失败下一帧自愈。
    """
    try:
        http_client.post(
            "/v1/realtime/chassis-velocity",
            {"vx": float(vx), "vy": float(vy), "wz": 0.0},
            timeout=1.5,
        )
    except Exception as e:
        print(f"  [{LOG_PREFIX}] ⚠️ chassis 速度下发失败 "
              f"({type(e).__name__}: {str(e)[:60]})")
