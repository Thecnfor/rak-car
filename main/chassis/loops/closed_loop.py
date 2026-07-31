"""main/chassis/loops/closed_loop.py
外环主循环：50Hz 拉一次 lane_state，调一次控制律，下发一次轮速。
任何异常路径都会先 zero out 轮速再返回。

下发前会经过 ``WheelSmoother`` 做单轮 |v| 饱和 + 单帧 slew rate 限幅，
避免弯道瞬间单轮目标跨度过大把下位机电源拉爆。
"""
import threading
import time
from typing import Callable, List, Optional

from .safety import EmergencyWatchdog, LostLineDetector
from ..api import ChassisClient
from ..state import LaneState
from ..controllers.base import OuterLoop, WheelSmoother


class DoubleLoopRunner:
    """双环 runner：外环在客户端、内环在车端（这里只负责发事件）。

    用法 1（一次性跑 N 秒）：
        api = ChassisClient.connect()
        runner = DoubleLoopRunner(api=api, outer=StanleyOuterLoop())
        runner.run(max_seconds=20.0)

    用法 2（#1：run 在后台线程，主线程 pause/resume）：
        runner = DoubleLoopRunner(...)
        threading.Thread(target=runner.run, kwargs={"max_seconds": math.inf},
                         daemon=True).start()
        ...
        runner.pause()      # 停外环（断触发保护：smoother 保持最后值）
        do_task()
        runner.resume()     # 续外环（smoother 重置为 0，避免 stale 数据跳变）
        ...
        runner.stop()
    """

    def __init__(
        self,
        api: ChassisClient,
        outer: OuterLoop,
        hz: float = 50.0,
        watchdog_ms: Optional[float] = 500.0,
        lost_line_ms: Optional[float] = 300.0,
        dry_run: bool = False,
        on_tick: Optional[Callable[[LaneState, List[float]], None]] = None,
        smoother: Optional[WheelSmoother] = None,
    ) -> None:
        self.api = api
        self.outer = outer
        self.hz = float(hz)
        self.dt = 1.0 / max(self.hz, 1.0)
        # 两个兜底各自可关：传 None 就不挂
        # - watchdog_ms=None：不因 lane_state 过期急停
        # - lost_line_ms=None：不因误差齐 0 急停（笔直居中的路段本来就会齐 0）
        self.watchdog = None if watchdog_ms is None else EmergencyWatchdog(threshold_ms=watchdog_ms)
        self.lost_line = None if lost_line_ms is None else LostLineDetector(stable_ms=lost_line_ms)
        # dry_run：只跑控制律不下发轮速，用于离线看数
        self.dry_run = bool(dry_run)
        self.on_tick = on_tick
        # 默认挂一个 smoother；要彻底关掉就显式传一个 max_abs=∞ / max_accel=∞ 的实例
        self.smoother = smoother if smoother is not None else WheelSmoother()
        self._stop = False
        # pause/resume 控制（#1）：pause 后外环在 _pause.wait() 处阻塞；
        # smoother 保持最后值，不会"忘记"已下发速度（避免发零后恢复时跳变）。
        self._pause = threading.Event()
        self._pause.set()  # 初始为"运行"状态

    def stop(self) -> None:
        """请求 run() 退出（finally 会兜底 zero out + api.close）。"""
        self._stop = True
        # 唤醒 _pause.wait()，让 run() 立刻看到 _stop
        self._pause.set()

    def pause(self) -> None:
        """暂停外环：当前帧跑完后阻塞在 _pause.wait()。smoother 保留最后值。

        安全：调用方通常紧跟着 ``api.stop_wheel_speeds()`` 主动发零速，
        然后跑任务逻辑（车端内环 PID 接管）。这样 resume() 时车已静止，
        smoother 重置到 0 是连续的。
        """
        self._pause.clear()

    def resume(self) -> None:
        """恢复外环：smoother 重置为 0，下一帧从静止起步（避免 stale 跳变）。"""
        # 先清 smoother 的"上一帧"记忆，再唤醒外环
        self.smoother.reset([0.0, 0.0, 0.0, 0.0])
        self._pause.set()

    def is_paused(self) -> bool:
        return not self._pause.is_set()

    def _sense(self) -> LaneState:
        # ChassisClient.read_lane() 内部已 ws 优先 + 异常兜底返回空 LaneState，
        # 外环不再自己 try/except —— 空 state 的 has_error 为 False，控制律自然输出零速。
        return self.api.read_lane()

    def run(self, max_seconds: float = 30.0) -> None:
        """阻塞：每 ~dt 跑一次外环 + 下发；任何异常路径都会 zero out 退出。

        关键流程：
            raw  = outer.step(state, dt)         # 控制律原始输出
            safe = self.smoother.step(raw)       # 单轮饱和 + slew rate 限幅
            api.set_wheel_speeds(safe)           # dry_run=False 时才下发

        pause/resume（#1）：pause 时循环阻塞在 ``_pause.wait()``，
        唤醒后从阻塞处继续；resume() 同时清 smoother 记忆。
        """
        deadline = time.monotonic() + max(0.0, float(max_seconds))
        next_tick = time.monotonic()
        # smoother 用 0 起步（外环起来前车就是停的），避免被首帧目标"撞到"
        self.smoother.reset([0.0, 0.0, 0.0, 0.0])
        try:
            while not self._stop:
                # pause 点（#1）：pause() 后阻塞；resume() / stop() 唤醒。
                self._pause.wait()
                if self._stop:
                    break
                now = time.monotonic()
                if now > deadline:
                    break
                state = self._sense()
                # 兜底项各自可关：传 None = 不挂这个检查
                if self.watchdog and self.watchdog.should_stop(state):
                    if not self.dry_run:
                        self.api.emergency_stop()
                    break
                if self.lost_line and self.lost_line.should_alert(state):
                    if not self.dry_run:
                        self.api.emergency_stop()
                    break
                raw = self.outer.step(state, self.dt)
                safe = self.smoother.step(raw)
                if not self.dry_run:
                    try:
                        self.api.set_wheel_speeds(safe)
                    except Exception:
                        # 下发掉帧不退出循环：下一帧会再发给当前 smoother 软化后的目标
                        pass
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
            # 退出收尾（#5）：smoother 归零 + api.close()（close 内部自动发零速）。
            # 不要再单独发 [0,0,0,0] —— close 已经做了。
            self.smoother.reset([0.0, 0.0, 0.0, 0.0])
            try:
                self.api.close()
            except Exception:
                pass