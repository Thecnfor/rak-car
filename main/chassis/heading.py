"""main/chassis/heading.py
全局航向估计器 — 用 da（视觉角度误差）+ 赛道地图 修正漂移的 odom theta。

物理模型：
  heading_true = da + ψ_lane(s)
  - da: lane 推理输出的 error_angle（车头相对车道切线的夹角），每帧独立、零累积漂移
  - ψ_lane(s): 赛道地图上距离 s 处的车道切线在世界系下的朝向（rad），场地标定面
  - s: 用 distance（可靠标量）索引

互补滤波：
  heading = (1-α) × (heading_prev + Δtheta_odom) + α × (da + ψ_lane(s))
  - 高频信任 odom 增量（捕捉真实转弯，低延迟）
  - 低频信任 da+地图（压掉编码器漂移）
  - lane 丢线时 α→0，纯航位推算 + 置信度衰减

x/y 重积分：
  odom 的 x/y 烂是因为积分时用了坏 theta 做旋转矩阵。
  修正方法：从 odom 增量反解车体坐标位移（不依赖 theta），再用修正后的 theta 重新旋转到世界系。

用法::

    from main.chassis.heading import HeadingEstimator, TrackMap
    from main.chassis.api import ChassisClient

    api = ChassisClient.connect()
    track = TrackMap.from_segments([(0, 10, 0.0), (10, 14, math.pi/2)])  # 直道→右弯
    est = HeadingEstimator(track_map=track, alpha=0.15)

    # 每帧（50Hz）：
    odom = api.get_odometry_state()
    lane = api.read_lane()
    state = est.update(
        theta_odom=odom.theta, distance=odom.distance,
        x_odom=odom.x, y_odom=odom.y,
        da=lane.error_angle, da_fresh=lane.is_fresh,
    )
    print(state.heading, state.x, state.y, state.confidence)
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple


# ============ 赛道地图 ============


class TrackMap:
    """赛道切线朝向 ψ_lane(s)：distance → 世界系航向角（rad）。

    最简单的标定方式：车沿赛道手动跑一圈，每 0.5m 记录一次 da 稳态值，
    则 ψ_lane(s) ≈ -da_steady(s)（直行稳态时 heading≈0 → da ≈ -ψ_lane）。
    或者对已知几何的赛道直接硬编码。

    未标定区域（超出最后一个标定点）→ 外推最后一个值（保守：假设赛道继续直行）。
    """

    def __init__(self, samples: List[Tuple[float, float]]):
        """samples: [(distance_m, lane_heading_rad), ...] 按 distance 升序。"""
        if not samples:
            raise ValueError("TrackMap 至少需要一个标定点")
        self._samples = sorted(samples, key=lambda p: p[0])

    def psi(self, s: float) -> float:
        """查询距离 s 处的车道切线朝向（rad），线性插值 + 边界外推。"""
        pts = self._samples
        if s <= pts[0][0]:
            return pts[0][1]
        if s >= pts[-1][0]:
            return pts[-1][1]
        # 线性插值
        for i in range(len(pts) - 1):
            s0, h0 = pts[i]
            s1, h1 = pts[i + 1]
            if s0 <= s <= s1:
                if s1 == s0:
                    return h0
                t = (s - s0) / (s1 - s0)
                return h0 + t * (h1 - h0)
        return pts[-1][1]  # fallback

    @classmethod
    def from_segments(cls, segments: List[Tuple[float, float, float]]) -> "TrackMap":
        """从分段常数构造：[(s_start, s_end, heading_rad), ...]。

        例：[(0, 10, 0.0), (10, 14, math.pi/2)] 表示 0-10m 朝东，10-14m 朝北。
        段间自动插值过渡。
        """
        samples: List[Tuple[float, float]] = []
        for s_start, _s_end, heading in segments:
            samples.append((s_start, heading))
        # 最后一段的终点也加上
        if segments:
            samples.append((segments[-1][1], segments[-1][2]))
        return cls(samples)

    @classmethod
    def straight(cls, heading: float = 0.0) -> "TrackMap":
        """纯直道（默认朝东 heading=0）。最简标定：全场 ψ_lane 恒定。"""
        return cls([(0.0, heading), (999.0, heading)])


# ============ 估计输出 ============


@dataclass
class HeadingState:
    """HeadingEstimator.update() 的输出快照。

    字段语义：
    - heading: 修正后的全局航向（rad），世界系，0 = 朝东（与 SDK odom 约定一致）
    - x / y: 用修正后 heading 重积分的世界坐标（m）
    - drift_rate: 当前漂移率估计（rad/m），正值 = odom theta 比真实偏大
    - confidence: 0~1，lane 在 = 1.0，丢线后按帧指数衰减
    - anchored: 是否至少被 da 锚定过一次（False 时 heading 纯靠 odom，不可信）
    """

    heading: float = 0.0
    x: float = 0.0
    y: float = 0.0
    drift_rate: float = 0.0
    confidence: float = 0.0
    anchored: bool = False


def _wrap_pi(a: float) -> float:
    """角度归一化到 [-π, π)。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ============ 航向估计器 ============


class HeadingEstimator:
    """互补滤波航向估计器 + x/y 重积分。

    每帧调 ``update()``，喂 odom (theta, x, y, distance) + lane (da, da_fresh)。
    输出 ``HeadingState``。

    设计约束：
    - 纯计算，不做 IO / 线程 / 网络
    - 线程安全由调用方保证（一般在外环线程里调）
    - 所有角度 rad，所有长度 m
    """

    def __init__(
        self,
        track_map: Optional[TrackMap] = None,
        alpha: float = 0.15,
        confidence_decay: float = 0.98,
        drift_ema_beta: float = 0.05,
    ):
        """
        参数：
            track_map        - 赛道切线朝向地图；None → 默认纯直道 ψ_lane=0
            alpha            - 互补滤波系数（0~1）。越大越信 da+地图（漂移压得狠但弯道延迟高）
            confidence_decay - 丢线后每帧置信度衰减系数
            drift_ema_beta   - 漂移率 EMA 平滑系数
        """
        self._track = track_map or TrackMap.straight()
        self._alpha = max(0.0, min(1.0, alpha))
        self._confidence_decay = confidence_decay
        self._drift_ema_beta = drift_ema_beta

        # 状态
        self._heading: float = 0.0
        self._x: float = 0.0
        self._y: float = 0.0
        self._drift_rate: float = 0.0
        self._confidence: float = 0.0
        self._anchored: bool = False

        # 上一帧 odom（用于算增量）
        self._prev_theta_odom: Optional[float] = None
        self._prev_x_odom: Optional[float] = None
        self._prev_y_odom: Optional[float] = None

    # ---- 公开方法 ----

    def reset(self, heading: float = 0.0, x: float = 0.0, y: float = 0.0) -> None:
        """重置估计器（对应 reset_position / 新一局）。"""
        self._heading = heading
        self._x = x
        self._y = y
        self._drift_rate = 0.0
        self._confidence = 0.0
        self._anchored = False
        self._prev_theta_odom = None
        self._prev_x_odom = None
        self._prev_y_odom = None

    def update(
        self,
        theta_odom: Optional[float],
        distance: Optional[float],
        x_odom: Optional[float] = None,
        y_odom: Optional[float] = None,
        da: Optional[float] = None,
        da_fresh: bool = True,
    ) -> HeadingState:
        """喂一帧数据，返回修正后的航向/位置估计。

        参数：
            theta_odom - SDK odom theta（rad），None → 跳过增量更新
            distance   - SDK odom distance（m），None → 用 x_odom/y_odom 估算
            x_odom     - SDK odom x（m），用于重积分
            y_odom     - SDK odom y（m），用于重积分
            da         - lane error_angle（rad），None/不新鲜 → 丢线模式
            da_fresh   - da 是否新鲜（<500ms），False 等同于 None
        """
        # 1) 算 odom 增量
        d_theta = 0.0
        if theta_odom is not None and self._prev_theta_odom is not None:
            d_theta = _wrap_pi(float(theta_odom) - self._prev_theta_odom)
        if theta_odom is not None:
            self._prev_theta_odom = float(theta_odom)

        # 2) 航位推算（高频，信任 odom 增量）
        self._heading += d_theta

        # 3) da 锚定（低频，压漂移）
        da_valid = da is not None and da_fresh
        if da_valid:
            s = float(distance) if distance is not None else math.hypot(self._x, self._y)
            psi_lane = self._track.psi(s)
            heading_from_da = float(da) + psi_lane  # type: ignore[operator]
            # 互补滤波：(1-α) × 航位推算 + α × da 锚定
            innovation = _wrap_pi(heading_from_da - self._heading)
            self._heading += self._alpha * innovation
            self._heading = _wrap_pi(self._heading)
            # 漂移率估计：正向漂移 → innovation 为负（heading 被推高，da 说没偏）
            # 每帧漂移量 ≈ -α × innovation（稳态时修正量 = -漂移量）
            instantaneous_drift = -innovation * self._alpha
            self._drift_rate += self._drift_ema_beta * (instantaneous_drift - self._drift_rate)
            # 置信度恢复
            self._confidence = 1.0
            self._anchored = True
        else:
            # 丢线：纯航位推算 + 置信度衰减
            self._confidence *= self._confidence_decay

        # 4) x/y 重积分
        #    SDK 的 x_odom/y_odom 是世界系坐标（用坏 theta 旋转过），不是车体系增量。
        #    步骤：a) 用 theta_odom 反旋转 → 车体系增量（撤销 SDK 的坏旋转）
        #          b) 用修正后 heading 正旋转 → 世界系增量
        #    近似：SDK 用"本帧更新前的旧 theta"旋转，我们用更新后的 theta_odom 近似，
        #    每帧 dtheta 极小（20Hz, <0.05 rad），误差可忽略。
        if (x_odom is not None and y_odom is not None
                and self._prev_x_odom is not None and self._prev_y_odom is not None
                and theta_odom is not None):
            dx_world = float(x_odom) - self._prev_x_odom
            dy_world = float(y_odom) - self._prev_y_odom
            # a) 反旋转：world → body（用 theta_odom 撤销 SDK 旋转）
            cos_b = math.cos(float(theta_odom))
            sin_b = math.sin(float(theta_odom))
            dx_body = dx_world * cos_b + dy_world * sin_b
            dy_body = -dx_world * sin_b + dy_world * cos_b
            # b) 正旋转：body → world（用修正后 heading）
            cos_h = math.cos(self._heading)
            sin_h = math.sin(self._heading)
            self._x += dx_body * cos_h - dy_body * sin_h
            self._y += dx_body * sin_h + dy_body * cos_h
        if x_odom is not None:
            self._prev_x_odom = float(x_odom)
        if y_odom is not None:
            self._prev_y_odom = float(y_odom)

        return HeadingState(
            heading=self._heading,
            x=self._x,
            y=self._y,
            drift_rate=self._drift_rate,
            confidence=self._confidence,
            anchored=self._anchored,
        )

    @property
    def heading(self) -> float:
        return self._heading

    @property
    def confidence(self) -> float:
        return self._confidence

    @property
    def anchored(self) -> bool:
        return self._anchored


__all__ = ["HeadingEstimator", "HeadingState", "TrackMap"]

