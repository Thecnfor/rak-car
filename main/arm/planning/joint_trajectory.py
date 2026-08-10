"""main/arm/planning/joint_trajectory.py
机械臂 4-DOF 多关键点平滑轨迹（goal → waypoint → … → goal），样条版。

依赖：**scipy.interpolate.PchipInterpolator**（关节空间插值的事实标准，C¹ 连续、
单调无过冲、精确过点）+ numpy（弧长/速度采样）。scipy/numpy 缺失时自动降级为
线性路径（仍按弧长时间参数化，连续不停顿），保证 Jetson 无 scipy 也能跑。

为什么用 PCHIP 而不是 CubicSpline：
  - PCHIP 在数据点之间**单调、无过冲** → 机械臂不会"甩过"关键点（靠限位的点安全）；
  - CubicSpline C² 更光滑但会在点间过冲，机械臂场景风险大。速度连续性由
    弧长时间参数化保证（C¹ 路径 + 弧长上连续速度 → 关节速度连续）。

语义：
  - **所有关键点默认连续平滑经过（不停顿）**，任意数量都行；
  - 仅 `stop=True` 的关键点在该处停车（速度归 0 → 再起步），用于取/放等动作点；
  - 每条关节速度/加速度被保守约束在限位内（弧长速度 = 各关节限速的公共上界）。

离线：FakeRobotSim 可仿真；真机：`dense_waypoints()` 喂 composite_run。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

try:  # pragma: no cover
    import numpy as _np
    _HAS_NUMPY = True
except Exception:  # pragma: no cover
    _np = None
    _HAS_NUMPY = False

try:  # pragma: no cover
    from scipy.interpolate import PchipInterpolator as _PchipInterpolator
    _HAS_SCIPY = True
except Exception:  # pragma: no cover
    _PchipInterpolator = None
    _HAS_SCIPY = False

# ---- 关节限位（同步 runtime/safety.py 硬限） ----
ARM_MIN_DEG, ARM_MAX_DEG = -150.0, 150.0
HAND_MIN_DEG, HAND_MAX_DEG = -90.0, 10.0
Y_MIN_MM, Y_MAX_MM = -200.0, 0.0
X_MIN_MM, X_MAX_MM = -300.0, 300.0

# ---- 默认运动学规格（按轴，mm/s·mm/s² / deg/s·deg/s²） ----
JOINT_VMAX = {"x_mm": 150.0, "y_mm": 90.0, "arm_deg": 90.0, "hand_deg": 90.0}
JOINT_AMAX = {"x_mm": 400.0, "y_mm": 240.0, "arm_deg": 100.0, "hand_deg": 100.0}
# 弧长默认兜底（各关节都不动时） mm/s / mm/s²
DEFAULT_ARC_VMAX = 120.0
DEFAULT_ARC_AMAX = 240.0

# 弧长加速的保守系数（考虑 sdot²·d²q/ds² 曲率项，取半）
_ACCEL_SAFETY = 0.5


def _sign(x: float) -> float:
    return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass(frozen=True)
class JointPose:
    """4-DOF 关节姿势（示教器标定值，单位 mm / deg）。

    stop: True=该关键点停车（取/放等动作点）；False（默认）=连续平滑经过。
    """

    x_mm: float
    y_mm: float
    arm_deg: float
    hand_deg: float
    stop: bool = False

    def __post_init__(self):
        for name, value, lo, hi in (
            ("x_mm", self.x_mm, X_MIN_MM, X_MAX_MM),
            ("y_mm", self.y_mm, Y_MIN_MM, Y_MAX_MM),
            ("arm_deg", self.arm_deg, ARM_MIN_DEG, ARM_MAX_DEG),
            ("hand_deg", self.hand_deg, HAND_MIN_DEG, HAND_MAX_DEG),
        ):
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"JointPose.{name} 必须有限: {value}")
            if not (lo <= value <= hi):
                raise ValueError(f"JointPose.{name}={value} 超出限位 [{lo}, {hi}]")

    def to_dict(self) -> dict:
        return {"x_mm": self.x_mm, "y_mm": self.y_mm,
                "arm_deg": self.arm_deg, "hand_deg": self.hand_deg,
                "stop": self.stop}

    @classmethod
    def from_mapping(cls, m: dict, *, stop: bool = False) -> "JointPose":
        """从姿势 dict（字段可缺省）构造。兼容 arm_deg/hand_deg 与 arm/hand 别名。"""
        return cls(
            x_mm=float(m.get("x_mm", 0.0)),
            y_mm=float(m.get("y_mm", 0.0)),
            arm_deg=float(m.get("arm_deg", m.get("arm_angle_deg", m.get("arm", 0.0)))),
            hand_deg=float(m.get("hand_deg", m.get("hand_angle_deg", m.get("hand", 0.0)))),
            stop=bool(m.get("stop", stop)),
        )


# ---- 单轴梯形 profile（带终端速度，用于弧长 s(t)） ----

def _trapezoid_v(d_signed: float, v_start: float, v_end: float,
                 v_max: float, a_max: float) -> dict:
    """带起止速度的单轴梯形 profile：accel→cruise→decel 到 v_end。"""
    d = abs(d_signed)
    sign = _sign(d_signed)
    v_max = max(abs(v_start), abs(v_end), abs(v_max))
    a = max(abs(a_max), 1e-6)
    v_start = max(0.0, min(abs(v_start), v_max))
    v_end = max(0.0, min(abs(v_end), v_max))
    if d < 1e-9:
        if abs(v_start - v_end) < 1e-9:
            return dict(t_acc=0.0, t_run=0.0, t_dec=0.0, t_total=0.0,
                        v_peak=0.0, v_end=0.0, sign=0.0, d_abs=0.0)
        t_acc = (v_end - v_start) / a
        return dict(t_acc=t_acc, t_run=0.0, t_dec=0.0, t_total=abs(t_acc),
                    v_peak=max(v_start, v_end), v_end=v_end, sign=sign, d_abs=0.0)
    d_acc = (v_max * v_max - v_start * v_start) / (2.0 * a)
    d_dec = (v_max * v_max - v_end * v_end) / (2.0 * a)
    if d_acc + d_dec <= d:
        t_acc = (v_max - v_start) / a
        t_dec = (v_max - v_end) / a
        t_run = (d - d_acc - d_dec) / v_max
        return dict(t_acc=t_acc, t_run=t_run, t_dec=t_dec,
                    t_total=t_acc + t_run + t_dec,
                    v_peak=v_max, v_end=v_end, sign=sign, d_abs=d)
    v_peak = math.sqrt((2.0 * a * d + v_start * v_start + v_end * v_end) / 2.0)
    t_acc = (v_peak - v_start) / a
    t_dec = (v_peak - v_end) / a
    return dict(t_acc=t_acc, t_run=0.0, t_dec=t_dec,
                t_total=t_acc + t_dec,
                v_peak=v_peak, v_end=v_end, sign=sign, d_abs=d)


def _eval_trapezoid_v(prof: dict, t: float) -> Tuple[float, float]:
    t = max(0.0, min(t, prof["t_total"]))
    sign = prof["sign"]
    v_peak = prof["v_peak"]
    v_end = prof["v_end"]
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
        v = max(v_end, v_peak - a * dt)
        s_base = 0.5 * a * t_acc * t_acc + v_peak * t_run
        s = s_base + v_peak * dt - 0.5 * a * dt * dt
    return sign * s, sign * v


# ---- 关节空间路径（弧长参数化） ----

class _JointPath:
    """关节空间几何路径：s(弧长) -> 4 关节值。

    scipy 可用 → PCHIP（C¹、单调无过冲）；否则线性插值（C⁰，仍时间平滑）。
    """

    def __init__(self, poses: List[JointPose], engine: str):
        pts = [[p.x_mm, p.y_mm, p.arm_deg, p.hand_deg] for p in poses]
        self.poses = poses
        if _HAS_NUMPY:
            arr = _np.array(pts, dtype=float)
            d = _np.linalg.norm(_np.diff(arr, axis=0), axis=1)
            self.s = _np.concatenate([[0.0], _np.cumsum(d)])
            self.L = float(self.s[-1])
        else:  # pragma: no cover（无 numpy 的兜底）
            self.s = [0.0]
            for i in range(1, len(pts)):
                self.s.append(self.s[-1] + math.hypot(
                    pts[i][0] - pts[i-1][0], pts[i][1] - pts[i-1][1]))
            self.L = self.s[-1]
        if engine == "scipy" and _HAS_SCIPY and _HAS_NUMPY:
            self._interp = [_PchipInterpolator(self.s, arr[:, j]) for j in range(4)]
            self._deriv = [p.derivative() for p in self._interp]
        else:
            self._interp = None

    def value_at(self, s: float, j: Optional[int] = None):
        """弧长 s 处的关节值（0..L）。返回单个关节（j 给定）或 4 元组。"""
        s = float(_clamp(float(s), 0.0, self.L))
        if self._interp is not None:
            if j is not None:
                return float(self._interp[j](s))
            return [float(p(s)) for p in self._interp]
        # 线性兜底（按弧长分段线性）
        pts = [self._pose_vals(i) for i in range(len(self.poses))]
        for i in range(len(self.s) - 1):
            if self.s[i] <= s <= self.s[i + 1]:
                span = self.s[i + 1] - self.s[i]
                r = (s - self.s[i]) / span if span > 1e-9 else 0.0
                vals = [a + (b - a) * r for a, b in zip(pts[i], pts[i + 1])]
                return vals[j] if j is not None else vals
        return self._pose_vals(-1)[j] if j is not None else self._pose_vals(-1)

    def dq_ds(self, s: float, j: Optional[int] = None):
        """弧长导数 dq/ds。scipy 用解析导数；线性用所在段斜率。"""
        if self._interp is not None:
            if j is not None:
                return float(self._deriv[j](s))
            return [float(p(s)) for p in self._deriv]
        # 线性路径：d q/d s = 所在线段的斜率（分段常值）
        s = float(_clamp(float(s), 0.0, self.L))
        pts = [self._pose_vals(i) for i in range(len(self.poses))]
        for i in range(len(self.s) - 1):
            if self.s[i] <= s <= self.s[i + 1]:
                span = self.s[i + 1] - self.s[i]
                if span > 1e-9:
                    slope = [(pts[i + 1][k] - pts[i][k]) / span for k in range(4)]
                    return slope[j] if j is not None else slope
                break
        zero = [0.0, 0.0, 0.0, 0.0]
        return zero[j] if j is not None else zero

    def _pose_vals(self, i: int) -> List[float]:
        p = self.poses[i]
        return [p.x_mm, p.y_mm, p.arm_deg, p.hand_deg]


@dataclass
class JointLeg:
    """弧长区间 [s_start, s_end] + 该区间梯形 s(t)；区间内部连续不停车。"""

    path: _JointPath
    s_start: float
    s_end: float
    profile: dict          # 弧长梯形 s(t)，起点/终点速度 0
    arc_vmax: float
    arc_amax: float
    kp_start: int          # keypoints 下标（区间的起点关键点）
    kp_end: int            # keypoints 下标（区间的终点关键点）

    @property
    def T(self) -> float:
        return self.profile["t_total"]

    def pose_at(self, t: float) -> JointPose:
        s, _v = _eval_trapezoid_v(self.profile, t)
        q = self.path.value_at(self.s_start + s)
        return JointPose(q[0], q[1], q[2], q[3], stop=False)


@dataclass
class JointTrajectory:
    """goal → waypoint → … → goal 的 4-DOF 平滑轨迹（连续经过，任意数量点）。"""

    keypoints: Tuple[JointPose, ...]
    legs: Tuple[JointLeg, ...]
    sample_hz: float = 50.0

    @property
    def total_time(self) -> float:
        return sum(leg.T for leg in self.legs)

    def leg_at(self, t: float) -> Tuple[int, JointLeg, float]:
        t = max(0.0, min(float(t), self.total_time))
        acc = 0.0
        for index, leg in enumerate(self.legs):
            if t <= acc + leg.T + 1e-9:
                return index, leg, max(0.0, t - acc)
            acc += leg.T
        leg = self.legs[-1]
        return len(self.legs) - 1, leg, leg.T

    def sample(self, t: float) -> JointPose:
        """任意时刻 t 的 4-DOF 位姿（末端停在最后一个关键点）。"""
        if not self.legs:
            return self.keypoints[-1]
        if t <= 0.0:
            return self.keypoints[0]
        if t >= self.total_time:
            return self.keypoints[-1]
        _idx, leg, t_in = self.leg_at(t)
        # 区间边界精确命中关键点（保留 stop 标志，避免浮点误差）
        if t_in <= 1e-9 and leg.kp_start != 0:
            return self.keypoints[leg.kp_start]
        if abs(t_in - leg.T) <= 1e-9:
            return self.keypoints[leg.kp_end]
        return leg.pose_at(t_in)

    def dense_waypoints(self, spacing_mm: float = 5.0,
                        sample_hz: float = 50.0) -> List[JointPose]:
        """重采样成密集姿势序列（composite_run 喂点）；关键点精确保留。"""
        dt = 1.0 / max(sample_hz, 1.0)
        out: List[JointPose] = [self.keypoints[0]]
        last = self.keypoints[0]
        for leg in self.legs:
            end_kp = self.keypoints[leg.kp_end]
            n = max(1, int(math.ceil(leg.T / dt)))
            for k in range(1, n + 1):
                pose = leg.pose_at(leg.T * k / n)
                if abs(pose.x_mm - last.x_mm) >= spacing_mm:
                    out.append(pose)
                    last = pose
            out.append(end_kp)   # 关键点硬保留
            last = end_kp
        dedup = [out[0]]
        for pose in out[1:]:
            if (pose.x_mm, pose.y_mm, pose.arm_deg, pose.hand_deg) != \
               (dedup[-1].x_mm, dedup[-1].y_mm, dedup[-1].arm_deg, dedup[-1].hand_deg):
                dedup.append(pose)
        return dedup

    def describe(self) -> str:
        lines = []
        for i, leg in enumerate(self.legs):
            a = leg.path.value_at(leg.s_start)
            b = leg.path.value_at(leg.s_end)
            lines.append(
                f"  leg{i}: s[{leg.s_start:.0f},{leg.s_end:.0f}] "
                f"({a[0]:.0f},{a[1]:.0f},{a[2]:.0f},{a[3]:.0f}) -> "
                f"({b[0]:.0f},{b[1]:.0f},{b[2]:.0f},{b[3]:.0f}) "
                f"T={leg.T:.2f}s arc_v={leg.arc_vmax:.0f} arc_a={leg.arc_amax:.0f}")
        return ("JointTrajectory(kp=%d legs=%d T=%.2fs engine=%s)\n%s"
                % (len(self.keypoints), len(self.legs), self.total_time,
                   _ENGINE_NAME, "\n".join(lines)))


_ENGINE_NAME = "scipy-pchip" if (_HAS_SCIPY and _HAS_NUMPY) else "linear-fallback"


def _dedup_poses(poses: List[JointPose]) -> List[JointPose]:
    out = []
    for p in poses:
        if not out or (p.x_mm, p.y_mm, p.arm_deg, p.hand_deg) != \
                      (out[-1].x_mm, out[-1].y_mm, out[-1].arm_deg, out[-1].hand_deg):
            out.append(p)
    return out


def _arc_limits(path: _JointPath, joint_vmax: dict, joint_amax: dict,
                max_speed_scale: float) -> Tuple[float, float]:
    """弧长速度/加速上界 = 各关节限速(限加速)/|dq/ds| 的公共下界（保守）。"""
    if path.L <= 1e-9 or not _HAS_NUMPY:
        return DEFAULT_ARC_VMAX * max_speed_scale, DEFAULT_ARC_AMAX * max_speed_scale
    grid = _np.linspace(0.0, path.L, max(8, int(path.L / 2.0) + 1))
    dq = _np.abs(_np.array([path.dq_ds(float(s)) for s in grid]))  # (N, 4)
    axes = ["x_mm", "y_mm", "arm_deg", "hand_deg"]
    v_scale = [joint_vmax[a] / float(dq[:, j].max())
               for j, a in enumerate(axes)
               if float(dq[:, j].max()) > 1e-6]
    a_scale = [joint_amax[a] / float(dq[:, j].max())
               for j, a in enumerate(axes)
               if float(dq[:, j].max()) > 1e-6]
    arc_v = (min(v_scale) if v_scale else DEFAULT_ARC_VMAX) * max_speed_scale
    arc_a = (min(a_scale) if a_scale else DEFAULT_ARC_AMAX) * _ACCEL_SAFETY * max_speed_scale
    return float(max(arc_v, 1e-6)), float(max(arc_a, 1e-6))


def plan_joint_trajectory(keypoints: Sequence[JointPose], *,
                          joint_vmax: Optional[Dict[str, float]] = None,
                          joint_amax: Optional[Dict[str, float]] = None,
                          sample_hz: float = 50.0,
                          max_speed_scale: float = 1.0,
                          engine: str = "auto") -> JointTrajectory:
    """把关键点序列规划成 4-DOF 平滑轨迹（任意数量点，连续不停顿）。

    - 路径：scipy PCHIP 关节空间插值（C¹、精确过点、无过冲）；无 scipy 降级线性。
    - 时间：弧长梯形 s(t)，弧长速度 = 各关节限速的公共下界 → 全程关节速度
      ≤ 限速；弧长加速取半作曲率项保守余量。
    - 语义：`stop=True` 关键点停车（切开成独立 leg，速度归 0 再起步）；
      其余关键点**连续平滑经过，不停顿**。

    max_speed_scale: 全局速度缩放（0~1，现场降速用）。
    engine: "auto"（scipy 可用则用）/ "scipy" / "linear"。
    """
    if len(keypoints) < 2:
        raise ValueError("至少需要 2 个关键点（start + goal）")
    max_speed_scale = _clamp(float(max_speed_scale), 0.0, 1.0)
    jv = dict(JOINT_VMAX)
    ja = dict(JOINT_AMAX)
    if joint_vmax:
        jv.update({k: float(v) for k, v in joint_vmax.items()})
    if joint_amax:
        ja.update({k: float(v) for k, v in joint_amax.items()})

    use_scipy = engine in ("auto", "scipy") and _HAS_SCIPY and _HAS_NUMPY
    global _ENGINE_NAME
    _ENGINE_NAME = "scipy-pchip" if use_scipy else "linear-fallback"

    # 一整条 PCHIP 关节空间路径；stop=True 的关键点在弧长对应位置切开
    # （速度归 0 再起步），其余关键点全部连续平滑经过。
    pts = _dedup_poses(list(keypoints))
    if len(pts) < 2:
        raise ValueError("去重后至少需要 2 个关键点（start + goal）")
    path = _JointPath(pts, "scipy" if use_scipy else "linear")
    arc_v, arc_a = _arc_limits(path, jv, ja, max_speed_scale)

    s_of = path.s
    bounds_idx = [0]
    for i in range(1, len(pts) - 1):
        if pts[i].stop:
            bounds_idx.append(i)
    bounds_idx.append(len(pts) - 1)
    bounds_idx = sorted(set(bounds_idx))

    legs: List[JointLeg] = []
    for a_i, b_i in zip(bounds_idx, bounds_idx[1:]):
        s0, s1 = float(s_of[a_i]), float(s_of[b_i])
        if s1 - s0 < 1e-9:
            continue
        prof = _trapezoid_v(s1 - s0, 0.0, 0.0, arc_v, arc_a)
        legs.append(JointLeg(path=path, s_start=s0, s_end=s1, profile=prof,
                             arc_vmax=arc_v, arc_amax=arc_a,
                             kp_start=a_i, kp_end=b_i))

    return JointTrajectory(tuple(pts), tuple(legs), sample_hz=sample_hz)
