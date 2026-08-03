"""main/chassis/tasks/read_heading.py
实时全局航向估计循环：每帧读 odom + lane，喂 HeadingEstimator，回调 HeadingState。

另外提供赛道标定工具 ``record_track_profile``：沿赛道跑一段，记录
(distance, ψ_lane) 采样点，用来构造 TrackMap。

用法::

    from main.chassis import ChassisClient
    from main.chassis.tasks.read_heading import read_heading
    from main.chassis.heading import TrackMap

    api = ChassisClient.connect()
    track = TrackMap.straight()  # 或标定后替换
    est = HeadingEstimator(track_map=track)

    def on_state(state):
        print(state.heading, state.x, state.y, state.confidence)

    read_heading(api, estimator=est, hz=50.0, max_seconds=30.0, on_tick=on_state)

标定用法（跑完把打印的 samples 填进 TrackMap）::

    samples = record_track_profile(api, max_seconds=60.0)
    print(samples)  # [(distance, psi_lane), ...]
"""
from __future__ import annotations

import time
from typing import Callable, List, Optional, Tuple

from ..api import ChassisClient
from ..heading import HeadingEstimator, HeadingState

# 每帧回调：当前航向估计快照
HeadingTickCallback = Callable[[HeadingState], None]

__all__ = ["read_heading", "record_track_profile", "HeadingTickCallback"]


def read_heading(
    api: ChassisClient,
    estimator: HeadingEstimator,
    *,
    hz: float = 50.0,
    max_seconds: Optional[float] = None,
    on_tick: Optional[HeadingTickCallback] = None,
) -> None:
    """实时轮询 odom + lane，喂 HeadingEstimator，每帧回调 on_tick。

    Args:
        api: ChassisClient（需 ws/http 可用；odom fast-path 优先）。
        estimator: HeadingEstimator 实例（调用方持有，跨 run 保留状态）。
        hz: 采样频率。建议 20-50Hz。
        max_seconds: 最大运行时长，None 表示一直跑（依赖 Ctrl-C 终止）。
        on_tick: 每帧回调，签名 `(state: HeadingState)`。
    """
    if hz <= 0:
        raise ValueError("hz must be > 0")
    dt = 1.0 / hz
    deadline = None if max_seconds is None else time.monotonic() + max_seconds

    try:
        while True:
            t0 = time.monotonic()
            if deadline is not None and t0 >= deadline:
                break

            odom = api.get_odometry_state()
            lane = api.read_lane()

            state = estimator.update(
                theta_odom=odom.theta,
                distance=odom.distance,
                x_odom=odom.x,
                y_odom=odom.y,
                da=lane.error_angle,
                da_fresh=lane.is_fresh,
            )

            if on_tick is not None:
                try:
                    on_tick(state)
                except Exception:
                    pass

            elapsed = time.monotonic() - t0
            sleep_s = dt - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        pass


def record_track_profile(
    api: ChassisClient,
    *,
    hz: float = 20.0,
    max_seconds: float = 60.0,
    interval_m: float = 0.5,
    steady_frames: int = 5,
    da_max: float = 0.35,
) -> List[Tuple[float, float]]:
    """沿赛道跑，记录 (distance, ψ_lane) 采样点用于构造 TrackMap。

    原理：直行稳态时 heading ≈ ψ_lane，而 da = heading - ψ_lane ≈ 0；
    更一般地 ψ_lane(s) = heading_true(s) - da(s)。标定时假设车基本沿车道走，
    取 ψ_lane ≈ -da 的滑动均值（连续 steady_frames 帧 da 稳定才记录）。

    标定流程：
      1. 把车放到起点，手动/低速巡线跑完全程，本函数自动记录
      2. 把返回的 samples 填进 TrackMap(samples)
      3. 弯道上 ψ_lane 变化会被 distance 索引自然捕捉

    Args:
        api: ChassisClient。
        hz: 采样频率（20Hz 够，da 不需要高频）。
        max_seconds: 最长录制时间。
        interval_m: 每隔多少米记一个点。
        steady_frames: da 连续 N 帧变化 < 0.02 rad 才认为稳态。
        da_max: |da| 超过此值视为大偏/弯道急转，跳过该采样点。

    Returns:
        [(distance_m, psi_lane_rad), ...] 按 distance 升序。
    """
    samples: List[Tuple[float, float]] = []
    last_recorded = -10.0  # 保证第一个点一定记录
    recent_da: List[float] = []
    deadline = time.monotonic() + max_seconds
    dt = 1.0 / hz

    try:
        while time.monotonic() < deadline:
            t0 = time.monotonic()

            odom = api.get_odometry_state()
            lane = api.read_lane()

            if (odom.distance is not None and lane.error_angle is not None
                    and lane.is_fresh):
                s = float(odom.distance)
                da = float(lane.error_angle)
                recent_da.append(da)
                if len(recent_da) > steady_frames:
                    recent_da.pop(0)

                if s - last_recorded >= interval_m and abs(da) < da_max:
                    # 稳态判定：最近 N 帧 da 波动小
                    if len(recent_da) >= steady_frames:
                        spread = max(recent_da) - min(recent_da)
                        if spread < 0.04:
                            # ψ_lane ≈ -da 均值（heading≈0 假设，直道标定足够）
                            psi = -sum(recent_da) / len(recent_da)
                            samples.append((round(s, 3), round(psi, 4)))
                            last_recorded = s
                            print(f"  [标定] s={s:6.2f}m  ψ_lane={psi:+.3f} rad "
                                  f"(da={da:+.3f}, 共 {len(samples)} 点)")

            elapsed = time.monotonic() - t0
            if dt - elapsed > 0:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        pass

    print(f"[标定] 完成，共 {len(samples)} 个采样点：")
    print("TrackMap([")
    for s, psi in samples:
        print(f"    ({s}, {psi}),")
    print("])")
    return samples


if __name__ == "__main__":
    def _main() -> None:
        api = ChassisClient.connect()
        print("=== 赛道标定模式 ===")
        print("让车沿赛道正常巡线行驶，本工具自动记录 (distance, ψ_lane)。")
        print("按 Ctrl-C 提前结束。\n")
        try:
            samples = record_track_profile(api, max_seconds=120.0)
        finally:
            api.close()
        if not samples:
            print("没有记录到任何采样点（lane 一直丢线？）")

    _main()
