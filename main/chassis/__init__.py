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
from .controllers.odom_turn import OdomTurnPID, wrap_pi
from .controllers.visual_align import VisualAlignOuterLoop
from .controllers.move_along_lane import move_along_lane
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
from .controllers.straight import StraightOuterLoop
from .heading import HeadingEstimator, HeadingState, TrackMap
from .tasks.monitor_ir import monitor_ir, IRAlertCallback, IRTickCallback
from .tasks.read_dis import read_dis, DisTickCallback
from .tasks.read_heading import read_heading, record_track_profile, HeadingTickCallback
from .tasks.read_ir import read_ir
from .config import LANE_FOLLOW, LANE_FOLLOW_SLOW, ControllerType, LaneFollowProfile


_client: Optional[ChassisClient] = None


def get_odometry() -> tuple[float, float, float]:
    """读底盘里程计 (x, y, theta)，单位 m / m / rad。模块级持有 ChassisClient 实例复用。

    用法::

        from main.chassis import get_odometry
        x, y, theta = get_odometry()
    """
    global _client
    if _client is None:
        _client = ChassisClient.connect()
    return _client.get_odometry()


def subscribe_lane_state(
    *,
    profile: LaneFollowProfile = LANE_FOLLOW,
    outer: Optional[OuterLoop] = None,
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

        from main.chassis import subscribe_lane_state
        from main.chassis.controllers.curvature_adaptive import CurvatureAdaptiveOuterLoop
        subscribe_lane_state(
            outer=CurvatureAdaptiveOuterLoop(v_max=0.2, kp_y=0.5),
            max_seconds=10.0,
        )

    参数：
        profile    - 调参 profile（循环节律 / 下发软化），默认 LANE_FOLLOW
        outer      - 自构造外环，如 ``CurvatureAdaptiveOuterLoop(v_max=..., kp_y=...)``；
                     传了就用它调控制器增益，否则 ``profile.build_outer()``
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

    if outer is None:
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
        hz         - 循环频率(默认 20Hz;task_feed 默认 30Hz,20Hz 足够且不浪费算力)。
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
    # --- 一键入口（写 task 优先用这些，见 main/README.md 速查表） ---
    "get_odometry",                # 读底盘里程计 (x, y, theta)
    "subscribe_lane_state",        # 一键巡线：profile → outer/smoother → DoubleLoopRunner
    "subscribe_visual_align",      # 一键面积视觉对准：只前进/后退到 ref_area
    # --- client / 状态 ---
    "ChassisClient",               # 薄封装 RuntimeApiClient/WS；move_for 是唯一合法平移
    "LaneState",                   # lane_feed 缓存帧
    "AlignState",                  # 视觉微调状态（area/ref_area/error）
    "select_target",               # 按 label/面积选检测目标
    # --- 控制器 ---
    "OuterLoop",                   # 外环 ABC
    "WheelSmoother",               # 轮速软化
    "POuterLoop",
    "StanleyOuterLoop",
    "CurvatureAdaptiveOuterLoop",  # 循迹主力
    "StraightOuterLoop",
    "VisualAlignOuterLoop",        # 面积对准控制器
    "move_along_lane",             # 沿中心车道线只前进/后退
    "ErrorCalibrator",
    "OdomTurnPID",
    "wrap_pi",
    # --- 循环 / 安全 / 遥测 ---
    "DoubleLoopRunner",            # 50Hz 外环主循环（orchestrator 在用）
    "EmergencyWatchdog",
    "LostLineDetector",
    "lane_trace",                  # 循迹每帧 trace 回调工厂
    # --- 视觉追踪（task 在用） ---
    "track_chassis",               # 底盘把目标拉画面中心（task1/2/4）
    "TrackChassisResult",
    "TrackFrame",
    "expand_label_set",
    "track_trace",
    "VisualAlignRunner",
    "make_align_runner",           # 面积对准 runner 工厂（task5）
    "align_trace",
    "AlignConvergenceDetector",
    "AlignRunResult",
    # --- 航向估计 / 赛道剖面 ---
    "read_heading",                # 实时轮询航向传感器，每帧调 on_tick
    "HeadingTickCallback",         # 航向每帧回调类型
    "record_track_profile",        # 根据航向/速度记录赛道剖面（起终点 + 弯道标记）
    "HeadingEstimator",            # 航向估计器：融合原始航向 + 速度门控 + 漂移校正
    "HeadingState",                # 航向估计器实时状态（航向角 / 累计距离 / 赛道标记）
    "TrackMap",                    # 赛道地图：起终点 + 弯道区间 + 方向表
    # --- 低层读取任务 ---
    "monitor_ir",
    "IRAlertCallback",
    "IRTickCallback",
    "read_dis",
    "DisTickCallback",
    "read_ir",
    # --- 调参 profile ---
    "LANE_FOLLOW",
    "LANE_FOLLOW_SLOW",
    "ControllerType",
    "LaneFollowProfile",
]