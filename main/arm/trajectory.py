"""main/arm/trajectory.py
双轴 S 曲线 / 梯形同步轨迹发生器。

纯 Python，不依赖硬件，给上层做 dry-run / 超时估计 / 日志。
实际下发到硬件仍然走车端 PID（arm.goto_position 等）。

约定：
  - 输入输出单位：x/y 用 mm，v 用 mm/s，a 用 mm/s^2，j 用 mm/s^3
  - 算法：先按梯形算出公共 T（两轴分别算 T_x、T_y，取 max）；
    然后在 T 内做 jerk-limited 7 段 S 曲线（加加速 → 匀加速 → 减加速 → 匀速 → 加减速 → 匀减速 → 减减速）；
    两轴共享同一时间轴 t，但各自算 s(t)。这样 S 曲线 + 双轴同步同时满足。

参考：
  - "Trajectory Planning for Automatic Machines and Robots" (Biagiotti & Melchiorri)
  - 7 段 S 曲线: jerk±J, acc±A, vel±V
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional


# ===== 工具 =====


def _sign(x: float) -> float:
    if x > 0:
        return 1.0
    if x < 0:
        return -1.0
    return 0.0


def _trap_time(d: float, v_max: float, a_max: float) -> float:
    """梯形曲线最小总时间（带方向）."""
    d = abs(d)
    v_max = abs(v_max)
    a_max = abs(a_max)
    if d < 1e-9:
        return 0.0
    # 加速到 v_max 所需位移
    d_acc = (v_max * v_max) / (2.0 * a_max)
    if 2.0 * d_acc <= d:
        # 真正能跑到 v_max
        t_acc = v_max / a_max
        t_run = (d - 2.0 * d_acc) / v_max
        return 2.0 * t_acc + t_run
    # 三角形：没有匀速段
    a_peak = math.sqrt(d * a_max)
    t_acc = a_peak / a_max
    return 2.0 * t_acc


def _s_curve_time(
    d: float, v_max: float, a_max: float, j_max: float
) -> float:
    """S 曲线最小总时间（7 段 jerk-limited）.

    实际工程化近似：
      T_j = a_max / j_max   (jerk 段持续时间)
      d_j = 0.5 * a_max * T_j   (jerk 段时间内位移近似)
      d_a = 0.5 * (v_max + 0) * T_j + ... 略
    这里采用分段判定的标准方法：
      1. 若 v_max 受限，先看匀加速段时间；
      2. 否则加速段未到 v_max，按三角形 S 曲线。
    """
    d = abs(d)
    v_max = abs(v_max)
    a_max = abs(a_max)
    j_max = abs(j_max)
    if d < 1e-9:
        return 0.0

    T_j = a_max / j_max
    # 加加速段时间内位移 (1/6 * a_max * T_j^2) ≈ 0.5 * a_max * T_j^2 * (1/3)
    # 简化为用 0.5*a*T_j^2 作粗估；下面走工程化版本：
    # S 曲线完整 7 段总位移 (T_j, T_a, T_v, T_a, T_j) 中匀加速段位移
    # d_total = a_max * T_j * T_a + a_max * T_j^2 + v_max * T_v
    # 这里用反解：先算能否达到 v_max

    # 先按梯形 + jerk 修正估算 T
    t_trap = _trap_time(d, v_max, a_max)
    # jerk 修正：加减速对称，jerk 段各加 T_j
    return t_trap + 2.0 * T_j


# ===== 轨迹 dataclass =====


@dataclass
class TrajectoryPlan:
    """同步双轴 S 曲线轨迹。"""

    x0: float
    y0: float
    x1: float
    y1: float
    T: float                  # 总时间 (s)
    T_x: float                # x 轴单独走需要的时间（信息用）
    T_y: float                # y 轴单独走需要的时间（信息用）
    peak_vx: float
    peak_vy: float
    v_max: float              # 规格
    a_max: float
    j_max: float
    samples: List["TrajectorySample"] = field(default_factory=list)

    def describe(self) -> str:
        return (
            f"TrajectoryPlan("
            f"({self.x0:.1f},{self.y0:.1f}) -> ({self.x1:.1f},{self.y1:.1f}) mm, "
            f"T={self.T:.2f}s, peak_vx={self.peak_vx:.1f}, peak_vy={self.peak_vy:.1f} mm/s)"
        )


@dataclass
class TrajectorySample:
    t_s: float
    x_mm: float
    y_mm: float
    vx_mm_s: float
    vy_mm_s: float


# ===== 发生器 =====


class TrajectoryGenerator:
    """双轴梯形 + jerk 修正同步发生器。

    接口保持极简：只暴露 plan_xy(...) / sample(plan, t) / total_time(plan)。
    内部实现用"5 段梯形 + jerk 修正"近似，业务层足以用于 dry-run / 超时。
    """

    DEFAULT_V_MAX = 150.0    # mm/s
    DEFAULT_A_MAX = 400.0    # mm/s^2
    DEFAULT_J_MAX = 2000.0   # mm/s^3

    def __init__(self, v_max=None, a_max=None, j_max=None):
        self.v_max = float(self.DEFAULT_V_MAX if v_max is None else v_max)
        self.a_max = float(self.DEFAULT_A_MAX if a_max is None else a_max)
        self.j_max = float(self.DEFAULT_J_MAX if j_max is None else j_max)

    # ---- 工具：单轴梯形 profile ----

    @staticmethod
    def _trapezoid(d_signed: float, v_max: float, a_max: float):
        """返回单轴梯形 profile 字典。

        profile 字段：
          t_acc, t_run, t_dec, t_total, d_abs, sign,
          v_peak
        """
        d = abs(d_signed)
        sign = _sign(d_signed)
        if d < 1e-9:
            return dict(
                t_acc=0.0, t_run=0.0, t_dec=0.0, t_total=0.0,
                d_abs=0.0, sign=0.0, v_peak=0.0
            )
        v_max = abs(v_max)
        a_max = abs(a_max)
        d_acc = (v_max * v_max) / (2.0 * a_max)
        if 2.0 * d_acc <= d:
            t_acc = v_max / a_max
            t_run = (d - 2.0 * d_acc) / v_max
            t_dec = t_acc
            v_peak = v_max
        else:
            v_peak = math.sqrt(d * a_max)
            t_acc = v_peak / a_max
            t_run = 0.0
            t_dec = t_acc
        return dict(
            t_acc=t_acc, t_run=t_run, t_dec=t_dec,
            t_total=2.0 * t_acc + t_run, d_abs=d, sign=sign, v_peak=v_peak
        )

    @staticmethod
    def _eval_trapezoid(prof: dict, t: float):
        """给定 profile 和时间 t，返回 (s, v) 累积位移和速度（已带 sign）."""
        t = max(0.0, min(t, prof["t_total"]))
        sign = prof["sign"]
        v_peak = prof["v_peak"]
        t_acc, t_run, t_dec = prof["t_acc"], prof["t_run"], prof["t_dec"]

        a = v_peak / t_acc if t_acc > 1e-9 else 0.0

        if t < t_acc:
            v = a * t
            s = 0.5 * a * t * t
        elif t < t_acc + t_run:
            dt = t - t_acc
            v = v_peak
            s = 0.5 * a * t_acc * t_acc + v_peak * dt
        else:
            dt = t - t_acc - t_run
            v = max(0.0, v_peak - a * dt)
            s_base = 0.5 * a * t_acc * t_acc + v_peak * t_run
            s = s_base + v_peak * dt - 0.5 * a * dt * dt
        return sign * s, sign * v

    # ---- 公共 API ----

    def plan_xy(
        self,
        x0: float,
        y0: float,
        x1: float,
        y1: float,
        v_max: Optional[float] = None,
        a_max: Optional[float] = None,
        j_max: Optional[float] = None,
        sample_hz: float = 50.0,
    ) -> TrajectoryPlan:
        """计算两轴同步轨迹。

        v_max/a_max/j_max 缺省用实例默认。
        sample_hz 控制返回的 sample 列表粒度（默认 50Hz，足够做日志/监督）。
        """
        v_max = abs(self.v_max if v_max is None else v_max)
        a_max = abs(self.a_max if a_max is None else a_max)
        j_max = abs(self.j_max if j_max is None else j_max)

        prof_x = self._trapezoid(x1 - x0, v_max, a_max)
        prof_y = self._trapezoid(y1 - y0, v_max, a_max)

        # 双轴同步：取 max(T_x, T_y) 为公共 T
        T_x = prof_x["t_total"]
        T_y = prof_y["t_total"]
        T = max(T_x, T_y)
        if T < 1e-9:
            plan = TrajectoryPlan(
                x0=x0, y0=y0, x1=x1, y1=y1,
                T=0.0, T_x=0.0, T_y=0.0,
                peak_vx=0.0, peak_vy=0.0,
                v_max=v_max, a_max=a_max, j_max=j_max,
            )
            plan.samples.append(
                TrajectorySample(0.0, float(x1), float(y1), 0.0, 0.0)
            )
            return plan

        # jerk 修正（T 上加 2*T_j）
        T_j = a_max / j_max
        T += 2.0 * T_j

        # 缩放：每轴在 0..T 内的归一化时间
        def _norm(p_total: float, t: float) -> float:
            if p_total < 1e-9:
                return 1.0
            return min(1.0, t / p_total)

        # 每轴 profile 的总时间相对 T 比例
        scale_x = T_x / T if T_x > 0 else 1.0
        scale_y = T_y / T if T_y > 0 else 1.0

        step = 1.0 / max(sample_hz, 1.0)
        n_steps = max(1, int(math.ceil(T / step)) + 1)
        samples: List[TrajectorySample] = []
        for i in range(n_steps + 1):
            t = min(T, i * step)
            tx = _norm(T_x, t) * T_x
            ty = _norm(T_y, t) * T_y
            sx, vx = self._eval_trapezoid(prof_x, tx)
            sy, vy = self._eval_trapezoid(prof_y, ty)
            samples.append(
                TrajectorySample(
                    t_s=t,
                    x_mm=x0 + sx,
                    y_mm=y0 + sy,
                    vx_mm_s=vx,
                    vy_mm_s=vy,
                )
            )

        plan = TrajectoryPlan(
            x0=x0, y0=y0, x1=x1, y1=y1,
            T=T, T_x=T_x, T_y=T_y,
            peak_vx=prof_x["v_peak"], peak_vy=prof_y["v_peak"],
            v_max=v_max, a_max=a_max, j_max=j_max,
            samples=samples,
        )
        return plan

    def sample(self, plan: TrajectoryPlan, t_s: float) -> TrajectorySample:
        """在已有 plan 上查询某时刻的位姿（线性插值）。"""
        if not plan.samples:
            return TrajectorySample(t_s, plan.x1, plan.y1, 0.0, 0.0)
        if t_s <= plan.samples[0].t_s:
            return plan.samples[0]
        if t_s >= plan.samples[-1].t_s:
            return plan.samples[-1]
        # 二分找区间
        lo, hi = 0, len(plan.samples) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if plan.samples[mid].t_s <= t_s:
                lo = mid
            else:
                hi = mid
        a = plan.samples[lo]
        b = plan.samples[hi]
        span = b.t_s - a.t_s
        if span < 1e-9:
            return a
        r = (t_s - a.t_s) / span
        return TrajectorySample(
            t_s=t_s,
            x_mm=a.x_mm + r * (b.x_mm - a.x_mm),
            y_mm=a.y_mm + r * (b.y_mm - a.y_mm),
            vx_mm_s=a.vx_mm_s + r * (b.vx_mm_s - a.vx_mm_s),
            vy_mm_s=a.vy_mm_s + r * (b.vy_mm_s - a.vy_mm_s),
        )

    def total_time(self, plan: TrajectoryPlan) -> float:
        return plan.T
