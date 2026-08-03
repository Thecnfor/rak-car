# main/chassis 子包：底盘组独占目录
# 外部 import 只允许指向 main.*，不接触 runtime / smartcar
from __future__ import annotations

from typing import Callable, Optional

from .api import ChassisClient
from .state import LaneState
from .state_align import AlignState, select_target
from .controllers.base import OuterLoop, WheelSmoother
from .controllers.p_controller import POuterLoop
from .controllers.stanley import StanleyOuterLoop
from .controllers.curvature_adaptive import CurvatureAdaptiveOuterLoop
from .controllers.calibration import ErrorCalibrator
from .controllers.visual_align import VisualAlignOuterLoop
from .loops.closed_loop import DoubleLoopRunner
from .loops.safety import EmergencyWatchdog, LostLineDetector
from .loops.telemetry import lane_trace
from .loops.visual_align import (
    VisualAlignRunner,
    align_trace,
    AlignConvergenceDetector,
    AlignRunResult,
    make_align_runner,
)
from .loops.visual_track import (
    track_chassis,
    TrackChassisResult,
    TrackFrame,
    expand_label_set,
    track_trace,
)
from .tasks.monitor_ir import monitor_ir, IRAlertCallback, IRTickCallback
from .tasks.read_dis import read_dis, DisTickCallback
from .tasks.read_ir import read_ir
from .config import LANE_FOLLOW, LANE_FOLLOW_SLOW, ControllerType, LaneFollowProfile


def subscribe_lane_state(
    *,
    profile: LaneFollowProfile = LANE_FOLLOW,
    hz: Optional[float] = None,
    max_seconds: Optional[float] = None,
    dry_run: bool = False,
    with_trace: bool = True,
    on_tick: Optional[Callable[[LaneState, list[float]], None]] = None,
    calibrator: Optional["ErrorCalibrator"] = None,
) -> None:
    """巡线外环的**一健装配**：profile → outer / smoother → DoubleLoopRunner。

    等价于手动写::

        api = ChassisClient.connect()
        outer = profile.build_outer()
        smoother = profile.build_smoother()
        on_tick = lane_trace(outer) if with_trace else None
        runner = DoubleLoopRunner(api=api, outer=outer, hz=..., smoother=smoother, on_tick=on_tick)
        runner.run(max_seconds=...)

    用法::

        from main.chassis import subscribe_lane_state, LANE_FOLLOW
        subscribe_lane_state(profile=LANE_FOLLOW.tuned(v_max=0.2), max_seconds=10.0)

    参数：
        profile    - 调参 profile，默认 LANE_FOLLOW
        hz         - 循环频率，默认用 profile.hz
        max_seconds - 最大运行时间，默认用 profile.max_seconds
        dry_run    - True 时只跑控制律不下发轮速
        with_trace - True 时每帧打印 lane 误差 + 轮速
        on_tick    - 覆盖 with_trace 的自定义回调
    """
    api = ChassisClient.connect()
    effective_hz = profile.hz if hz is None else hz

    try:
        api.start_lane_feed(hz=effective_hz)
    except Exception:
        pass

    outer = profile.build_outer()
    smoother = profile.build_smoother()

    if on_tick is None and with_trace:
        on_tick = lane_trace(outer)

    runner = DoubleLoopRunner(
        api=api,
        outer=outer,
        hz=effective_hz,
        watchdog_ms=profile.watchdog_ms,
        lost_line_ms=profile.lost_line_ms,
        dry_run=dry_run,
        smoother=smoother,
        on_tick=on_tick,
        calibrator=calibrator,
    )
    try:
        runner.run(max_seconds=profile.max_seconds if max_seconds is None else max_seconds)
    finally:
        try:
            api.stop_lane_feed()
        except Exception:
            pass


def subscribe_visual_align(
    *,
    ref_area: float,
    label: Optional[str] = None,
    hz: float = 20.0,
    kp: float = 0.6,
    v_max: float = 0.20,
    deadband: float = 0.002,
    watchdog_ms: Optional[float] = 1000.0,
    max_seconds: Optional[float] = None,
    dry_run: bool = False,
    with_trace: bool = True,
    on_tick: Optional[Callable[[AlignState, list[float]], None]] = None,
) -> None:
    """视觉微调的**一键装配**：读 cam2 task_feed 缓存,只前进/后退到 ref_area。

    等价于手动写::

        api = ChassisClient.connect()
        outer = VisualAlignOuterLoop(kp=kp, v_max=v_max, deadband=deadband)
        on_tick = align_trace() if with_trace else None
        runner = VisualAlignRunner(
            api=api, outer=outer, hz=hz, ref_area=ref_area, label=label,
            watchdog_ms=watchdog_ms, dry_run=dry_run, on_tick=on_tick,
        )
        runner.run(max_seconds=max_seconds)

    用法::

        from main.chassis import subscribe_visual_align
        subscribe_visual_align(ref_area=0.04, label="hopper", max_seconds=10.0)

    几何含义：
      - 通过 ``/v1/realtime/vision/task`` 读 cam2 task_feed 缓存（不直接触发推理,免 5-15s 阻塞）。
      - 优先按 ``label`` 选目标;不传 label 时按面积最大兜底。
      - 比例控制 ``vx = kp * (ref_area - area)``:area 偏小 → 车前进;area 偏大 → 车后退。
      - 物理输出强制 ``vy=0 / omega=0`` → 4 轮全等 vx,**不准左右不准旋转**。

    参数：
        ref_area   - 期望面积(标度阶段记录后填入)。``None`` → 控制器永远零速,安全默认。
        label      - 优先选这个 label 的目标;``None`` 时取面积最大。
        hz         - 循环频率(默认 20Hz;task_feed 默认 10Hz,20Hz 足够且不浪费算力)。
        kp         - 比例增益(m/s 每单位 area_error)。
        v_max      - 单向速度上限(绝对值),只动前后,默认 0.20 m/s。
        deadband   - area_error 死区,小于此值视为 0,防止抖动。
        watchdog_ms - task_feed 太久没刷 → 急停;``None`` 不挂。
        max_seconds - 最大运行时间;``None`` 默认跑 30 秒后退出。
        dry_run    - True 时只算控制律不下发轮速。
        with_trace - True 时每帧打印目标 label/score/area/err + 4 轮速。
        on_tick    - 覆盖 with_trace 的自定义回调。
    """
    api = ChassisClient.connect()
    outer = VisualAlignOuterLoop(kp=kp, v_max=v_max, deadband=deadband)
    if on_tick is None and with_trace:
        on_tick = align_trace()

    runner = VisualAlignRunner(
        api=api,
        outer=outer,
        hz=hz,
        ref_area=ref_area,
        label=label,
        watchdog_ms=watchdog_ms,
        dry_run=dry_run,
        on_tick=on_tick,
    )
    runner.run(max_seconds=30.0 if max_seconds is None else max_seconds)

__all__ = [
    # --- 一键装配函数 ---
    "subscribe_lane_state",        # 巡线外环一键装配：profile → outer/smoother → DoubleLoopRunner
    "subscribe_visual_align",      # 视觉微调一键装配：只前后微调，把目标面积拉到 ref_area
    "track_chassis",               # 通用底盘视觉追踪：把 target bbox 中心拉到 setpoint_cxcy
    # --- API / 状态视图 ---
    "ChassisClient",               # 底盘专用 HTTP client
    "LaneState",                   # lane 误差缓存视图（/v1/vision/lane/state）
    "AlignState",                  # 视觉微调外环状态：当前帧选中目标 + ref_area 误差
    "select_target",               # 从 task detections 列表里选一个目标
    # --- 外环控制律 ---
    "OuterLoop",                   # 外环控制律接口（ABC）
    "WheelSmoother",               # 4 轮目标线速度软化器：饱和 + slew rate 限幅
    "POuterLoop",                  # 外环 = P：error_y 直给 vy，error_angle 进 omega
    "StanleyOuterLoop",            # Stanley 控制律（参考实现，需按场地再调）
    "CurvatureAdaptiveOuterLoop",  # 曲率自适应控制律（当前默认调参对象）
    "ErrorCalibrator",             # error_y / error_angle 的标定 + 去抖
    "VisualAlignOuterLoop",        # 视觉微调控制律：只前进/后退
    # --- 闭环 runner / 安全 / 遥测 ---
    "DoubleLoopRunner",            # 双环 runner：外环在客户端、内环在车端
    "EmergencyWatchdog",           # lane_state.updated_at 超阈值时报警
    "LostLineDetector",            # 误差值齐 0 持续 N 帧 → 丢线报警
    "lane_trace",                  # on_tick 回调：每 N 帧打印 lane 误差 + 4 轮速
    # --- 视觉微调 runner 族 ---
    "VisualAlignRunner",           # 视觉微调 runner：只动 vx，可判收敛（与 DoubleLoopRunner 同构）
    "AlignConvergenceDetector",    # 连续 N 帧 |area_error| < tol 视为到达
    "AlignRunResult",              # VisualAlignRunner.run() 的返回结果
    "align_trace",                 # on_tick 回调：每 N 帧打印对齐信息
    "make_align_runner",           # 一键构造快档视觉微调 runner（主入口）
    # --- 视觉追踪 runner 族 ---
    "TrackChassisResult",          # track_chassis 的返回结果
    "TrackFrame",                  # 一帧追踪状态（传给 on_tick / 放在结果里）
    "expand_label_set",            # 把目标 label(s) 展开成匹配集合
    "track_trace",                 # on_tick 回调：每 N 帧打印一行追踪信息
    # --- 配置 ---
    "LaneFollowProfile",           # 巡线外环全部可调量（字段语义见 curvature_adaptive.py 等）
    "ControllerType",              # profile 支持的控制律枚举（build_outer 按此分发）
    "LANE_FOLLOW",                 # 默认巡线 profile
    "LANE_FOLLOW_SLOW",            # 慢速巡线 profile（hz=20）
    # --- 红外 / 里程计任务 ---
    "monitor_ir",                  # 持续采样 IR，命中阈值时调 on_alert
    "IRAlertCallback",             # IR 告警回调类型
    "IRTickCallback",              # IR 每帧采样回调类型
    "read_dis",                    # 实时轮询里程计累计距离，每帧调 on_tick
    "DisTickCallback",             # 里程计每帧回调类型
    "read_ir",                     # 读取 IR 距离传感器（用户视角左/右，底层已调换）
]