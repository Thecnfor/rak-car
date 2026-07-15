"""main/chassis/loops/closed_loop.py
外环主循环：50Hz 拉一次 lane_state，调一次控制律，下发一次轮速。
任何异常路径都会先 zero out 轮速再返回。

下发前会经过 ``WheelSmoother`` 做单轮 |v| 饱和 + 单帧 slew rate 限幅，
避免弯道瞬间单轮目标跨度过大把下位机电源拉爆。
"""
import time
from typing import Callable, List, Optional

from .safety import EmergencyWatchdog, LostLineDetector
from ..api import ChassisClient
from ..state import LaneState
from ..controllers.base import OuterLoop, WheelSmoother


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
        smoother: Optional[WheelSmoother] = None,
    ) -> None:
        self.api = api
        self.outer = outer
        self.hz = float(hz)
        self.dt = 1.0 / max(self.hz, 1.0)
        self.watchdog = EmergencyWatchdog(threshold_ms=watchdog_ms)
        self.lost_line = LostLineDetector()
        self.on_tick = on_tick
        # 默认挂一个 smoother；要彻底关掉就显式传一个 max_abs=∞ / max_accel=∞ 的实例
        self.smoother = smoother if smoother is not None else WheelSmoother()
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
        """阻塞：每 ~dt 跑一次外环 + 下发；任何异常路径都会 zero out 退出。

        关键流程：
            raw = outer.step(state, dt)         # 控制律原始输出
            safe = self.smoother.step(raw)      # 单轮饱和 + slew rate 限幅
            api.set_wheel_speeds(safe)          # 下发
        """
        deadline = time.monotonic() + max(0.0, float(max_seconds))
        next_tick = time.monotonic()
        # smoother 用 0 起步（外环起来前车就是停的），避免被首帧目标"撞到"
        self.smoother.reset([0.0, 0.0, 0.0, 0.0])
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
                raw = self.outer.step(state, self.dt)
                safe = self.smoother.step(raw)
                last_wheel = list(safe)
                self.api.set_wheel_speeds(safe)
                if self.on_tick is not None:
                    try:
                        self.on_tick(state, safe)
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
            # 退出前先把 smoother 也清回 0，再发零速，保证 no-op 顺序
            self.smoother.reset([0.0, 0.0, 0.0, 0.0])
            try:
                self.api.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])
            except Exception:
                pass