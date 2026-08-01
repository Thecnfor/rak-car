"""main/chassis/loops/closed_loop.py
外环主循环：50Hz 拉一次 lane_state，调一次控制律，下发一次轮速。
任何异常路径都会先 zero out 轮速再返回。

下发前会经过 ``WheelSmoother`` 做单轮 |v| 饱和 + 单帧 slew rate 限幅，
避免弯道瞬间单轮目标跨度过大把下位机电源拉爆。
"""
import threading
import time
from typing import Callable, List, Optional, TYPE_CHECKING

from .safety import EmergencyWatchdog, LostLineDetector
from ..api import ChassisClient
from ..state import LaneState
from ..controllers.base import OuterLoop, WheelSmoother

if TYPE_CHECKING:
    from ..controllers.calibration import ErrorCalibrator


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
        runner.pause()      # 停外环（同步等确认：外环补发零速 + 阻塞，smoother 保持最后值）
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
        calibrator: Optional["ErrorCalibrator"] = None,
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
        # 误差标定层（scale/offset/EMA）。None = 不标定，控制律吃原始误差。
        # 传入后 _sense() 在喂给控制律之前先过一遍 calibrate()。
        self.calibrator = calibrator
        self._stop = False
        # pause/resume 控制（#1）：pause 后外环在 _pause.wait() 处阻塞；
        # smoother 保持最后值，不会"忘记"已下发速度（避免发零后恢复时跳变）。
        self._pause = threading.Event()
        self._pause.set()  # 初始为"运行"状态
        # 2026-08-01：pause 同步确认。外环检测到暂停时先补发零速、再 set 这个
        # ack、然后才阻塞；pause() 等到 ack 才返回 —— 保证调用方随后补发的零速
        # 不会被外环在途的非零 wheel_speeds 覆盖（旧实现的"停不下来"竞态）。
        self._paused_ack = threading.Event()

    def stop(self) -> None:
        """请求 run() 退出（finally 会兜底 zero out + api.close）。"""
        self._stop = True
        # 唤醒 _pause.wait()，让 run() 立刻看到 _stop
        self._pause.set()

    def pause(self, timeout: float = 1.0) -> bool:
        """暂停外环：**同步**等到外环线程确认已停后才返回。

        时序：
            1. 清 _paused_ack + 清 _pause
            2. 外环跑完当前帧后在循环顶部发现 _pause 已清：
               先补发零速兜底（覆盖在途非零帧），再 set _paused_ack，然后阻塞在 _pause.wait()
            3. 本方法等到 _paused_ack 后返回 True（超时返回 False）

        为什么同步：旧实现 pause() 只是清事件立即返回，外环把当前帧跑完时可能
        在 ``stop_wheel_speeds()`` **之后**又下发一条非零轮速；而外环随后就阻塞，
        再没有任何零速补发 → MC602 保持最后收到的非零速度，车停不下来。
        同步后，调用方在 pause() 返回后补发的零速一定是"最新一条命令"。

        幂等：已在暂停且已确认过 → 直接返回 True（避免重复等 ack 卡 1s）。
        """
        if not self._pause.is_set() and self._paused_ack.is_set():
            return True
        self._paused_ack.clear()
        self._pause.clear()
        return self._paused_ack.wait(timeout)

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
        state = self.api.read_lane()
        # 标定层：把 lane 模型裸输出标成控制律物理量（默认 no-op）。
        # 放在 _sense 边界而不是控制律内部，控制器零改动。
        if self.calibrator is not None:
            state = self.calibrator.calibrate(state)
        return state

    def run(self, max_seconds: float = 30.0) -> None:
        """阻塞：每 ~dt 跑一次外环 + 下发；任何异常路径都会 zero out 退出。

        关键流程：
            raw  = outer.step(state, dt)         # 控制律原始输出
            safe = self.smoother.step(raw)       # 单轮饱和 + slew rate 限幅
            api.set_wheel_speeds(safe)           # dry_run=False 时才下发

        pause/resume（#1）：pause() 后循环先补发零速 + set _paused_ack，
        再阻塞在 ``_pause.wait()``；resume() 唤醒并同时清 smoother 记忆。
        """
        deadline = time.monotonic() + max(0.0, float(max_seconds))
        next_tick = time.monotonic()
        # smoother 用 0 起步（外环起来前车就是停的），避免被首帧目标"撞到"
        self.smoother.reset([0.0, 0.0, 0.0, 0.0])
        try:
            while not self._stop:
                # pause 点（#1）：pause() 清事件后这里先补发零速 + set ack，再阻塞；
                # resume() / stop() 唤醒。注意必须先于任何非零帧检查 _pause：
                # 否则当前帧在 stop_wheel_speeds() 之后下发的非零会把零速顶掉（2026-08-01）。
                if not self._pause.is_set():
                    # 已暂停：补发零速兜底（覆盖可能刚下发的在途非零帧），
                    # set ack 让 pause() 知道外环已真正停住，然后阻塞等恢复。
                    if not self.dry_run:
                        try:
                            self.api.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])
                        except Exception:
                            pass
                    self._paused_ack.set()
                    self._pause.wait()
                    self._paused_ack.clear()
                    continue
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