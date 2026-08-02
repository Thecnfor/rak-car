"""main/chassis/loops/visual_align.py
视觉微调主循环：**只前进/后退**，靠 ``/v1/realtime/vision/task`` 缓存读侧摄 bbox。

整体与 DoubleLoopRunner 同构（pause/resume/stop/run），但控制律是
``VisualAlignOuterLoop``，物理输出直接下发 ``[vx,vx,vx,vx]``，绕开 IK，
确保**不准左右不准旋转**。

调用链路：
    api.get_vision_task_cache()     -> raw payload (task_feed 10Hz+ 缓存)
    -> AlignState.from_task_payload -> 选目标（label 优先 / 面积兜底） + 算 area_error
    -> VisualAlignOuterLoop.step    -> 4 轮全 vx
    -> WheelSmoother.step           -> 单轮 |v| 饱和 + slew rate 限幅（快档）
    -> api.set_wheel_speeds         -> ws 优先, HTTP 兜底（ChassisClient 已封装）

退出路径：任何异常 / 收敛到达 / watchdog 触发 / 到时 → smoother 归零 + api.close()，
并返回 ``AlignRunResult``（arrived / reason / final_state / elapsed）。
"""
import threading
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, TYPE_CHECKING

from ..api import ChassisClient
from ..state_align import AlignState
from ..controllers.base import OuterLoop, WheelSmoother
from ..controllers.visual_align import VisualAlignOuterLoop


# ============ trace 回调 ============


def align_trace(every_n: int = 1) -> Callable[[AlignState, List[float]], None]:
    """给 ``VisualAlignRunner(on_tick=...)`` 用的回调。

    每 ``every_n`` 帧打一行:目标 label / score / area / ref_area / area_error
    + 4 轮速度。area_error 正数=目标比期望位置更远(车要前进),
    负数=更近(车要后退);0 表示对齐。
    """
    counter = {"n": 0}

    def _on_tick(state: AlignState, wheels: List[float]) -> None:
        counter["n"] += 1
        if every_n > 1 and counter["n"] % every_n != 0:
            return
        v1, v2, v3, v4 = (wheels + [0.0] * 4)[:4]
        label = state.label or "-"
        score = "%.2f" % state.score if state.score is not None else "-"
        area = "%.4f" % state.area if state.area is not None else "-"
        ref = "%.4f" % state.ref_area if state.ref_area is not None else "-"
        err = "%+.4f" % state.area_error if state.area_error is not None else "-"
        print(
            "[align] n=%d label=%s score=%s area=%s ref=%s err=%s "
            "w=[%+.2f,%+.2f,%+.2f,%+.2f]"
            % (counter["n"], label, score, area, ref, err, v1, v2, v3, v4)
        )

    return _on_tick


if TYPE_CHECKING:
    pass


# ============ 收敛检测器 ============


class AlignConvergenceDetector:
    """连续 N 帧 ``|area_error| < tol`` 视为到达。

    设计要点（2026-08-02 用户要求"灵敏+快速，不要收敛超级久"）：
      - **3 帧死区内**就 arrival：20Hz 下 ≈ 0.15s 判定（之前默认要 N 帧 × 误差窗口）
      - **启动保护区** ``settle_skip_frames``：前 N 帧不计数,避免"车还没动就判到达"
        的假阳性（启动时车位置 / 目标 / 抖动都还没稳定）
      - **目标丢失重置**：target_found=False 时清零计数器（车不在画面对,继续累
        计会判错）

    参数:
      tol                  - 死区阈值（绝对值）,默认 0.005,和外环控制律 deadband 同档
      required_frames      - 连续满足帧数,默认 3（用户选"很快"档）
      settle_skip_frames   - 启动保护区帧数,默认 5（确保车先动起来）
    """

    def __init__(
        self,
        tol: float = 0.005,
        required_frames: int = 3,
        settle_skip_frames: int = 5,
    ) -> None:
        self.tol = float(tol)
        self.required_frames = int(required_frames)
        self.settle_skip_frames = int(settle_skip_frames)
        self._consecutive: int = 0
        self._frames_seen: int = 0

    def reset(self) -> None:
        self._consecutive = 0
        self._frames_seen = 0

    def update(self, state: AlignState) -> bool:
        """喂一帧,返回是否到达（arrived=True）。

        启动保护区（前 settle_skip_frames 帧）不算;目标丢失 / 无 error 重置计数。
        """
        self._frames_seen += 1
        if self._frames_seen <= self.settle_skip_frames:
            self._consecutive = 0
            return False
        if not state.has_error:
            self._consecutive = 0
            return False
        if abs(state.area_error) < self.tol:
            self._consecutive += 1
            if self._consecutive >= self.required_frames:
                return True
        else:
            self._consecutive = 0
        return False


# ============ run() 返回值 ============


@dataclass
class AlignRunResult:
    """``VisualAlignRunner.run()`` 返回的结果。

    字段:
      arrived      - 是否到达（连续 N 帧在死区内 → True）
      reason       - 退出原因: "arrived" / "timeout" / "stopped" / "watchdog"
                     / "no_target" / "max_seconds"
      final_state  - 最后一帧的 AlignState(可读 area_error / area / ref_area)
      elapsed_s    - 实际运行时长(秒)
      frames       - 跑过的帧数
    """

    arrived: bool = False
    reason: str = "unknown"
    final_state: Optional[AlignState] = None
    elapsed_s: float = 0.0
    frames: int = 0


# ============ runner ============


class VisualAlignRunner:
    """视觉微调 runner：与 DoubleLoopRunner 同构, 但物理上只动 vx,且可判收敛。

    用法 1（一次性跑 + 自动判定收敛返回结果）:
        from main.chassis import make_align_runner
        runner = make_align_runner(ref_area=0.04, label="hopper")
        result = runner.run(max_seconds=10.0)
        if result.arrived:
            print("已对齐,耗时 %.2fs" % result.elapsed_s)
        else:
            print("没对齐,reason=%s" % result.reason)

    用法 2（run 在后台线程, 主线程 pause/resume/stop）:
        threading.Thread(
            target=runner.run, kwargs={"max_seconds": math.inf}, daemon=True
        ).start()
        ...
        runner.pause()      # 同步等确认
        do_other_thing()
        runner.resume()
        ...
        runner.stop()

    参数:
      api              - ChassisClient 实例（用 .connect()）
      outer            - VisualAlignOuterLoop（**必须是它**,其他控制律会绕过"只前后"约束）
      hz               - 循环频率（默认 20Hz,task_feed 默认 10Hz,20Hz 足够且不浪费算力）
      ref_area         - 期望面积（必传,标度阶段记录后填入;None → 控制律永远零速）
      label            - 优先选这个 label;None 时按面积最大
      watchdog_ms      - task_feed 太久没刷 → 急停 + reason="watchdog";None 不挂
      convergence      - AlignConvergenceDetector（默认带 3 帧死区 + 5 帧启动保护区）;
                         ``arrival_enabled=False``(None) 时不判定收敛,只等超时
      max_seconds      - run() 默认时长(覆盖用 max_seconds=math.inf 永久跑)
      dry_run          - True 时只算控制律不下发轮速
      on_tick          - 每帧回调(state, safe_speeds),用于 trace
      smoother         - WheelSmoother(默认快档);要禁用就显式传一个 max_abs=∞ / max_accel=∞ 的实例
    """

    def __init__(
        self,
        api: ChassisClient,
        outer: OuterLoop,
        hz: float = 20.0,
        *,
        ref_area: Optional[float] = None,
        label: Optional[str] = None,
        watchdog_ms: Optional[float] = 1000.0,
        convergence: Optional[AlignConvergenceDetector] = None,
        arrival_enabled: bool = True,
        max_seconds: float = 30.0,
        dry_run: bool = False,
        on_tick: Optional[Callable[[AlignState, List[float]], None]] = None,
        smoother: Optional[WheelSmoother] = None,
    ) -> None:
        self.api = api
        self.outer = outer
        self.hz = float(hz)
        self.dt = 1.0 / max(self.hz, 1.0)
        self.ref_area = ref_area
        self.label = label
        self.watchdog_ms = watchdog_ms
        self.dry_run = bool(dry_run)
        self.on_tick = on_tick
        # 快档 smoother(2026-08-02):max_abs 0.40 / max_accel 0.15 / max_decel 0.25
        # 比 LANE_FOLLOW (0.55/0.20/0.30) 略激进,但给视觉微调"快进快停"留出余量
        self.smoother = smoother if smoother is not None else WheelSmoother(
            max_abs=0.40, max_accel=0.15, max_decel=0.25,
        )
        # 收敛检测:默认开启,3 帧死区内就 arrival
        self.convergence = convergence if convergence is not None else AlignConvergenceDetector()
        self.arrival_enabled = bool(arrival_enabled) and self.convergence is not None
        self.max_seconds_default = float(max_seconds)
        self._stop = False
        # pause/resume(与 DoubleLoopRunner 同构)
        self._pause = threading.Event()
        self._pause.set()  # 初始为"运行"状态
        self._paused_ack = threading.Event()

    # ---- 控制面:与 DoubleLoopRunner 一致 ----

    def stop(self) -> None:
        """请求 run() 退出（finally 会兜底 zero out + api.close）。"""
        self._stop = True
        self._pause.set()

    def pause(self, timeout: float = 1.0) -> bool:
        """暂停外环:同步等 ack 后返回 True(超时 False)。

        幂等:已在暂停且已确认过 → 直接返回 True。
        """
        if not self._pause.is_set() and self._paused_ack.is_set():
            return True
        self._paused_ack.clear()
        self._pause.clear()
        return self._paused_ack.wait(timeout)

    def resume(self) -> None:
        """恢复外环:smoother 重置为 0,下一帧从静止起步。"""
        self.smoother.reset([0.0, 0.0, 0.0, 0.0])
        self._pause.set()

    def is_paused(self) -> bool:
        return not self._pause.is_set()

    # ---- 感知 ----

    def _sense(self) -> AlignState:
        """读 task_feed 缓存,造 AlignState。任何异常兜底为空 state(无目标)。"""
        try:
            payload = self.api.http.get_vision_task_cache()
        except Exception:
            payload = None
        return AlignState.from_task_payload(
            payload or {},
            ref_area=self.ref_area,
            label=self.label,
        )

    # ---- 主循环 ----

    def run(self, max_seconds: Optional[float] = None) -> AlignRunResult:
        """阻塞:每 ~dt 跑一次外环 + 下发 + 判收敛。

        返回 ``AlignRunResult``:任何异常路径都会 zero out 退出。

        退出原因:
          "arrived"  - 连续 N 帧 area_error 在死区内（仅当 arrival_enabled=True）
          "watchdog" - task_feed 太久没刷
          "timeout"  - 到 max_seconds
          "stopped"  - 外部 stop() 调用
        """
        if max_seconds is None:
            max_seconds = self.max_seconds_default
        deadline = time.monotonic() + max(0.0, float(max_seconds))
        next_tick = time.monotonic()
        started_at = time.monotonic()
        # smoother 用 0 起步(外环起来前车是停的),避免被首帧目标撞到
        self.smoother.reset([0.0, 0.0, 0.0, 0.0])
        if self.convergence is not None:
            self.convergence.reset()
        frames = 0
        last_state: Optional[AlignState] = None
        reason = "timeout"
        arrived = False
        try:
            while not self._stop:
                # pause 点(与 DoubleLoopRunner 同构)
                if not self._pause.is_set():
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
                    reason = "stopped"
                    break
                now = time.monotonic()
                if now > deadline:
                    reason = "timeout"
                    break
                state = self._sense()
                last_state = state
                frames += 1
                # 兜底:task_feed 太久没刷 → 急停(视觉信号没了,继续动=瞎走)
                if self.watchdog_ms is not None and state.age_ms is not None:
                    if state.age_ms > self.watchdog_ms:
                        if not self.dry_run:
                            self.api.emergency_stop()
                        reason = "watchdog"
                        break
                # 收敛判定:在 3 帧死区内 → arrival(2026-08-02 用户要求灵敏快速)
                if self.arrival_enabled and self.convergence is not None:
                    if self.convergence.update(state):
                        arrived = True
                        reason = "arrived"
                        break
                raw = self.outer.step(state, self.dt)
                safe = self.smoother.step(raw)
                if not self.dry_run:
                    try:
                        self.api.set_wheel_speeds(safe)
                    except Exception:
                        # 下发掉帧不退出循环:下一帧会再发给当前 smoother 软化后的目标
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
                    # 调度落后,放弃补偿避免 catching up
                    next_tick = time.monotonic()
        finally:
            # 退出收尾:smoother 归零 + api.close()(close 内部自动发零速)
            self.smoother.reset([0.0, 0.0, 0.0, 0.0])
            try:
                self.api.close()
            except Exception:
                pass
        return AlignRunResult(
            arrived=arrived,
            reason=reason,
            final_state=last_state,
            elapsed_s=time.monotonic() - started_at,
            frames=frames,
        )


# ============ builder 入口(用户主入口) ============


def make_align_runner(
    *,
    ref_area: float,
    label: Optional[str] = None,
    hz: float = 20.0,
    kp: float = 1.5,
    v_max: float = 0.35,
    deadband: float = 0.005,
    smoother: Optional[WheelSmoother] = None,
    watchdog_ms: Optional[float] = 1000.0,
    arrival_enabled: bool = True,
    arrival_tol: float = 0.005,
    arrival_required_frames: int = 3,
    arrival_settle_skip_frames: int = 5,
    on_tick: Optional[Callable[[AlignState, List[float]], None]] = None,
    max_seconds: float = 30.0,
    dry_run: bool = False,
    api: Optional[ChassisClient] = None,
) -> VisualAlignRunner:
    """一键构造一个**快档**视觉微调 runner(主入口)。

    等价于手动写::

        api = ChassisClient.connect()
        outer = VisualAlignOuterLoop(kp=kp, v_max=v_max, deadband=deadband)
        smoother = WheelSmoother(max_abs=0.40, max_accel=0.15, max_decel=0.25)
        conv = AlignConvergenceDetector(
            tol=arrival_tol,
            required_frames=arrival_required_frames,
            settle_skip_frames=arrival_settle_skip_frames,
        )
        runner = VisualAlignRunner(
            api=api, outer=outer, hz=hz, ref_area=ref_area, label=label,
            smoother=smoother,
            watchdog_ms=watchdog_ms,
            convergence=conv,
            arrival_enabled=arrival_enabled,
            max_seconds=max_seconds,
            dry_run=dry_run,
            on_tick=on_tick,
        )

    用法::

        from main.chassis import make_align_runner
        runner = make_align_runner(ref_area=0.04, label="hopper")
        result = runner.run(max_seconds=10.0)
        if result.arrived:
            print("已对齐,耗时 %.2fs,跑了 %d 帧" % (result.elapsed_s, result.frames))

    默认参数为**快档**(2026-08-02)：收敛灵敏 + 响应快。要更平滑就减小 kp / v_max。
    """
    if api is None:
        api = ChassisClient.connect()
    outer = VisualAlignOuterLoop(kp=kp, v_max=v_max, deadband=deadband)
    if smoother is None:
        smoother = WheelSmoother(max_abs=0.40, max_accel=0.15, max_decel=0.25)
    conv = AlignConvergenceDetector(
        tol=arrival_tol,
        required_frames=arrival_required_frames,
        settle_skip_frames=arrival_settle_skip_frames,
    )
    return VisualAlignRunner(
        api=api,
        outer=outer,
        hz=hz,
        ref_area=ref_area,
        label=label,
        watchdog_ms=watchdog_ms,
        smoother=smoother,
        convergence=conv,
        arrival_enabled=arrival_enabled,
        max_seconds=max_seconds,
        dry_run=dry_run,
        on_tick=on_tick,
    )


__all__ = [
    "VisualAlignRunner",
    "AlignConvergenceDetector",
    "AlignRunResult",
    "align_trace",
    "make_align_runner",
]