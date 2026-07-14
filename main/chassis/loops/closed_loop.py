"""main/chassis/loops/closed_loop.py
外环主循环：50Hz 拉一次 lane_state，调一次控制律，下发一次轮速。
任何异常路径都会先 zero out 轮速再返回。
"""
import time
from typing import Callable, List, Optional

from .safety import EmergencyWatchdog, LostLineDetector
from ..api import ChassisClient
from ..state import LaneState
from ..controllers.base import OuterLoop


class DoubleLoopRunner:
    """双环 runner：外环在客户端、内环在车端（这里只负责发事件）。

    用法：
        api = ChassisClient.connect()
        api.start_lane_feed(hz=20.0)
        runner = DoubleLoopRunner(api=api, outer=StanleyOuterLoop())
        runner.run(max_seconds=20.0)
    """

    def __init__(
        self,
        api: ChassisClient,
        outer: OuterLoop,
        hz: float = 50.0,
        watchdog_ms: float = 500.0,
        on_tick: Optional[Callable[[LaneState, List[float]], None]] = None,
    ) -> None:
        self.api = api
        self.outer = outer
        self.hz = float(hz)
        self.dt = 1.0 / max(self.hz, 1.0)
        self.watchdog = EmergencyWatchdog(threshold_ms=watchdog_ms)
        self.lost_line = LostLineDetector()
        self.on_tick = on_tick
        self._stop = False

    def stop(self) -> None:
        self._stop = True

    def _sense(self) -> LaneState:
        try:
            payload = self.api.get_lane_state()
        except Exception:
            return LaneState()
        return LaneState.from_lane_state_payload(payload or {})

    def run(self, max_seconds: float = 30.0) -> None:
        """阻塞：每 ~dt 跑一次外环 + 下发；任何异常路径都会 zero out 退出。"""
        deadline = time.monotonic() + max(0.0, float(max_seconds))
        next_tick = time.monotonic()
        last_wheel = [0.0, 0.0, 0.0, 0.0]
        try:
            while not self._stop:
                now = time.monotonic()
                if now > deadline:
                    break
                state = self._sense()
                if self.watchdog.should_stop(state) or self.lost_line.should_alert(state):
                    self.api.emergency_stop()
                    break
                speeds = self.outer.step(state, self.dt)
                last_wheel = list(speeds)
                self.api.set_wheel_speeds(speeds)
                if self.on_tick is not None:
                    try:
                        self.on_tick(state, speeds)
                    except Exception:
                        pass
                next_tick += self.dt
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    # 调度已经落后了，放弃补偿避免 catching up
                    next_tick = time.monotonic()
        finally:
            try:
                self.api.set_wheel_speeds(last_wheel if False else [0.0, 0.0, 0.0, 0.0])
            except Exception:
                pass
