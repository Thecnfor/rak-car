"""main/chassis/controllers/move_along_lane.py
沿中心车道线方向**只前进/后退**（视觉对齐）的底盘方法。

控制律 = ``StraightOuterLoop(vx_cruise=vx, strafe_v=0.0)``：
  - **vy 通道被 ``strafe_v=0`` 锁死** → 物理上只有 vx 平移（vx>0 前进 / vx<0 后退），
    不会左右横移 / 侧滑。
  - **ω 通道照常**用 error_angle PI + error_y cross-track 让车头始终对齐车道
    中心线方向（车偏右 → 左转拉回，漂移归零 = 真平行）。
  - 其余节律 / 软化 / 兜底走 ``subscribe_lane_state``（LANE_FOLLOW profile）。

实现说明：``move_along_lane`` 是装配方法（不是控制律类），放在 controllers 目录
便于底盘组按"新写一个可调用方法"的路径落地；为避免 controllers ↔ 包 __init__
的循环 import，``subscribe_lane_state`` 用延迟 import（调用时才解析，届时包已
加载完毕——与 ``config/lane_follow.py`` 里 build_outer() 的延迟 import 同款）。
"""
from __future__ import annotations

from typing import Callable, Optional

from ..state import LaneState
from ..config import LANE_FOLLOW, LaneFollowProfile
from .straight import StraightOuterLoop


def move_along_lane(
    *,
    vx: float = 0.20,
    max_seconds: Optional[float] = 5.0,
    profile: LaneFollowProfile = LANE_FOLLOW,
    hz: Optional[float] = None,
    dry_run: bool = False,
    with_trace: bool = False,
    on_tick: Optional[Callable[[LaneState, list[float]], None]] = None,
    calibrator: Optional["ErrorCalibrator"] = None,
    straight: Optional[dict] = None,
) -> None:
    """沿中心车道线方向**只前进/后退**（视觉对齐）的一键方法。

    用法::

        from main.chassis import move_along_lane
        move_along_lane(vx=0.20, max_seconds=5.0)    # 沿车道中心线前进 5 秒
        move_along_lane(vx=-0.15, max_seconds=3.0)   # 沿车道中心线后退 3 秒

    参数：
        vx          - 带符号前向速度 (m/s)：正=前进，负=后退。默认 0.20。
        max_seconds - 运行时长上限。默认 5.0。
        profile     - 节律 / 软化 / 兜底 profile，默认 LANE_FOLLOW。
        hz          - 循环频率，默认用 profile.hz。
        dry_run     - True 时只跑控制律不下发轮速。
        with_trace  - True 时每帧打印 lane 误差 + 轮速。默认 False（任务原语，
                      避免 50Hz 刷屏；排障时再开）。
        on_tick     - 覆盖 with_trace 的自定义回调。
        calibrator  - 误差标定层（lane 模型裸输出 → 物理量）。
        straight    - 透传 ``StraightOuterLoop`` 调参（kp_theta / sign_theta /
                      k_ey_omega / omega_max …），现场方向反了改这里。
                      ``vx_cruise`` 恒用 vx，``strafe_v`` 恒 0，不可覆盖。
    """
    from .. import subscribe_lane_state  # 延迟 import：避免 controllers ↔ __init__ 循环

    straight_kwargs = dict(straight or {})
    straight_kwargs["vx_cruise"] = float(vx)
    straight_kwargs["strafe_v"] = 0.0  # 锁死 vy：物理上只有前进/后退
    outer = StraightOuterLoop(**straight_kwargs)
    return subscribe_lane_state(
        outer=outer,
        max_seconds=max_seconds,
        profile=profile,
        hz=hz,
        dry_run=dry_run,
        with_trace=with_trace,
        on_tick=on_tick,
        calibrator=calibrator,
    )


__all__ = ["move_along_lane"]
