"""main/arm/planning/joint_trajectory.py
机械臂 4-DOF 多关键点平滑轨迹（goal → waypoint → … → goal）。

纯 Python（stdlib math 即可），不依赖硬件、不依赖 numpy/scipy/PyBullet；
离线可跑在 FakeRobotSim 上仿真，真机可把 dense_waypoints 喂给 composite_run。

输入：示教器标定的关节姿势序列 `JointPose(x_mm, y_mm, arm_deg, hand_deg)`。
约束：
  - 每个关键点**精确经过**（硬约束——示教点是有意义的物理位置）；
  - 每个关键点默认**停车**（v=0，与真机 composite_run 逐点到位语义一致）；
  - 中间点若 `stop=False`，则计算"可穿越速度"做角点圆滑（速度连续，
    不停车直接滑过），本模块给出保守上界（相邻段各留一半做加减速）。

算法：逐段做 4 轴**带终端速度的梯形**（accel→cruise→decel 到 v_end），
四轴以 `T = max(各轴时间)` 同步（复用 main/arm/trajectory.py 的 _norm 缩放
约定，两轴同步已实车验证过）；`sample(t)` 沿时间轴采样全部 4 关节值。

参考：Biagiotti & Melchiorri, "Trajectory Planning for Automatic Machines
and Robots"（与 main/arm/trajectory.py 同源）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

# ---- 关节限位（同步 runtime/safety.py 硬限） ----
ARM_MIN_DEG, ARM_MAX_DEG = -150.0, 150.0
HAND_MIN_DEG, HAND_MAX_DEG = -90.0, 10.0
Y_MIN_MM, Y_MAX_MM = -200.0, 0.0
X_MIN_MM, X_MAX_MM = -300.0, 300.0

# ---- 默认运动学规格 (mm / deg / s) ----
DEFAULT_V_MAX = 150.0     # 主滑块 x mm/s
DEFAULT_A_MAX = 400.0     # mm/s^2
DEFAULT_ARM_DEG_S = 90.0  # 大臂角速度 deg/s
DEFAULT_HAND_DEG_S = 90.0  # 手爪角速度 deg/s


def _sign(x: float) -> float:
    return 1.0 if x > 0 else (-1.0 if x < 0 else 0.0)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass(frozen=True)
class JointPose:
    """4-DOF 关节姿势（示教器标定值，单位 mm / deg）。"""

    x_mm: float
    y_mm: float
    arm_deg: float
    hand_deg: float
    stop: bool = True  # True=该关键点停车；False=不停直接滑过（角点圆滑）

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
    def from_mapping(cls, m: dict, *, stop: bool = True) -> "JointPose":
        """从姿势库 dict（字段可缺省，缺省保持当前语义用 0 占位）构造。"""
        return cls(
            x_mm=float(m.get("x_mm", 0.0)),
            y_mm=float(m.get("y_mm", 0.0)),
            arm_deg=float(m.get("arm_deg", m.get("arm_angle_deg", 0.0))),
            hand_deg=float(m.get("hand_deg", m.get("hand_angle_deg", 0.0))),
            stop=bool(m.get("stop", stop)),
        )


# ---- 单轴梯形 profile（带终端速度） ----

def _trapezoid_v(d_signed: float, v_start: float, v_end: float,
                 v_max: float, a_max: float) -> dict:
    """带起止速度的单轴梯形 profile。

    返回 dict: t_acc / t_run / t_dec / t_total / v_peak / v_end / sign / d_abs。
    处理 v_end>0 的角点穿越；距离过短时自动降 v_peak 成三角形。
    """
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
        # 0 距离但起止速度不同 → 原地变速（近似：按加减速时间算）
        t_acc = (v_end - v_start) / a
        return dict(t_acc=t_acc, t_run=0.0, t_dec=0.0, t_total=abs(t_acc),
                    v_peak=max(v_start, v_end), v_end=v_end, sign=sign, d_abs=0.0)
    # 三段：accel(0→v_peak) → cruise → decel(v_peak→v_end)
    d_acc = (v_max * v_max - v_start * v_start) / (2.0 * a)
    d_dec = (v_max * v_max - v_end * v_end) / (2.0 * a)
    if d_acc + d_dec <= d:
        t_acc = (v_max - v_start) / a
        t_dec = (v_max - v_end) / a
        t_run = (d - d_acc - d_dec) / v_max
        return dict(t_acc=t_acc, t_run=t_run, t_dec=t_dec,
                    t_total=t_acc + t_run + t_dec,
                    v_peak=v_max, v_end=v_end, sign=sign, d_abs=d)
    # 到不了 v_max：三角形（无 cruise），反解 v_peak
    #  (v_peak² - v_start²) + (v_peak² - v_end²) = 2 a d
    v_peak = math.sqrt((2.0 * a * d + v_start * v_start + v_end * v_end) / 2.0)
    t_acc = (v_peak - v_start) / a
    t_dec = (v_peak - v_end) / a
    return dict(t_acc=t_acc, t_run=0.0, t_dec=t_dec,
                t_total=t_acc + t_dec,
                v_peak=v_peak, v_end=v_end, sign=sign, d_abs=d)


def _eval_trapezoid_v(prof: dict, t: float) -> Tuple[float, float]:
    """给定 profile 和时间 t（0..t_total），返回 (s, v)。"""
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


# ---- 分段轨迹 ----

@dataclass
class JointSegment:
    """相邻两关键点之间的一整段（4 轴各自 profile + 同步时长）。"""

    start: JointPose
    end: JointPose
    profiles: dict          # {axis: profile}
    T: float                # 本段同步时长 (s)
    pass_speed: float       # 本段到达速度（= 下个关键点穿越速度，mm/s 量纲的保守上界）

    def axis_pose(self, t: float) -> JointPose:
        values = {}
        for attr, prof in self.profiles.items():
            values[attr] = self._axis_value(attr, prof, t)
        return JointPose(
            x_mm=values["x_mm"], y_mm=values["y_mm"],
            arm_deg=values["arm_deg"], hand_deg=values["hand_deg"],
            stop=False,
        )

    def _axis_value(self, attr: str, prof: dict, t: float) -> float:
        start_val = getattr(self.start, attr)
        end_val = getattr(self.end, attr)
        span = end_val - start_val
        if abs(span) < 1e-9:
            return end_val
        s, _v = _eval_trapezoid_v(prof, t)
        return start_val + s


@dataclass
class JointTrajectory:
    """goal → waypoint → … → goal 的 4-DOF 平滑轨迹。"""

    keypoints: Tuple[JointPose, ...]
    segments: Tuple[JointSegment, ...]
    sample_hz: float = 50.0

    @property
    def total_time(self) -> float:
        return sum(seg.T for seg in self.segments)

    def segment_at(self, t: float) -> Tuple[int, JointSegment, float]:
        """定位 (segment_index, segment, 段内时间 t_in)。"""
        t = max(0.0, min(float(t), self.total_time))
        acc = 0.0
        for index, seg in enumerate(self.segments):
            if t <= acc + seg.T + 1e-9:
                return index, seg, max(0.0, t - acc)
            acc += seg.T
        seg = self.segments[-1]
        return len(self.segments) - 1, seg, seg.T

    def sample(self, t: float) -> JointPose:
        """任意时刻 t 的 4-DOF 位姿（末端停在最后一个关键点）。

        段边界精确命中关键点时返回原关键点（保留 stop 标志），
        避免梯形求值的浮点误差污染"关键点精确经过"语义。
        """
        if not self.segments:
            return self.keypoints[-1]
        if t <= 0.0:
            return self.keypoints[0]
        if t >= self.total_time:
            return self.keypoints[-1]
        _idx, seg, t_in = self.segment_at(t)
        if t_in <= 1e-9 and _idx > 0:
            return self.keypoints[_idx]
        if abs(t_in - seg.T) <= 1e-9:
            return self.keypoints[_idx + 1]
        return seg.axis_pose(t_in)

    def dense_waypoints(self, spacing_mm: float = 5.0,
                        sample_hz: float = 50.0) -> List[JointPose]:
        """把整条轨迹重采样成密集姿势序列（供 composite_run 逐点喂给真机）。

        spacing_mm 以 x 轴位移为主度量；每个关键点本身**精确**出现
        （段末直接写回原关键点，保证 stop 标志与示教值原样保留）。
        """
        dt = 1.0 / max(sample_hz, 1.0)
        out: List[JointPose] = [self.keypoints[0]]
        last = self.keypoints[0]
        for i, seg in enumerate(self.segments):
            end_kp = self.keypoints[i + 1]
            n = max(1, int(math.ceil(seg.T / dt)))
            for k in range(1, n + 1):
                pose = seg.axis_pose(seg.T * k / n)
                if abs(pose.x_mm - last.x_mm) >= spacing_mm:
                    out.append(pose)
                    last = pose
            # 段末：精确落回原关键点（硬约束，不被重采样稀释）
            out.append(end_kp)
            last = end_kp
        # 去掉连续重复（零位移段）
        dedup = [out[0]]
        for pose in out[1:]:
            if (pose.x_mm, pose.y_mm, pose.arm_deg, pose.hand_deg) != \
               (dedup[-1].x_mm, dedup[-1].y_mm, dedup[-1].arm_deg, dedup[-1].hand_deg):
                dedup.append(pose)
        return dedup

    def describe(self) -> str:
        seg_lines = []
        for i, seg in enumerate(self.segments):
            a, b = seg.start, seg.end
            seg_lines.append(
                f"  seg{i}: ({a.x_mm:.0f},{a.y_mm:.0f},{a.arm_deg:.0f},"
                f"{a.hand_deg:.0f}) -> ({b.x_mm:.0f},{b.y_mm:.0f},"
                f"{b.arm_deg:.0f},{b.hand_deg:.0f}) T={seg.T:.2f}s "
                f"v_pass={seg.pass_speed:.0f}")
        return ("JointTrajectory(kp=%d segs=%d T=%.2fs)\n%s"
                % (len(self.keypoints), len(self.segments),
                   self.total_time, "\n".join(seg_lines)))


_AXES = ("x_mm", "y_mm", "arm_deg", "hand_deg")


def _axis_specs() -> dict:
    """每个轴的 JointPose 属性名 / 速度规格 / 加速度规格。"""
    return {
        "x_mm":   (DEFAULT_V_MAX,      DEFAULT_A_MAX),
        "y_mm":   (DEFAULT_V_MAX * 0.6, DEFAULT_A_MAX),
        "arm_deg": (DEFAULT_ARM_DEG_S,  DEFAULT_A_MAX / 4.0),
        "hand_deg": (DEFAULT_HAND_DEG_S, DEFAULT_A_MAX / 4.0),
    }


def _pass_speed(prev: JointPose, way: JointPose, nxt: JointPose) -> float:
    """中间关键点不停车时可穿越的保守上界速度。

    规则：任一刀轴在"进段后半 + 出段前半"内要能从 0 加速到 v 再减到 0，
    v ≤ sqrt(a * min(dist_in, dist_out)) 每条轴都满足 → 取全部轴的最小。
    """
    specs = _axis_specs()
    speeds = []
    for attr, (v_max, a_max) in specs.items():
        d_in = abs(getattr(way, attr) - getattr(prev, attr))
        d_out = abs(getattr(nxt, attr) - getattr(way, attr))
        # 半程加减速空间 = 两侧各一半
        room = min(d_in, d_out) / 2.0
        # 三角形：v² = a·room（从 0 加到 v 再减到 0，各占 room/2）
        v_tri = math.sqrt(max(a_max * room, 0.0))
        speeds.append(min(v_max, v_tri))
    return min(speeds) if speeds else 0.0


def plan_joint_trajectory(keypoints: Sequence[JointPose], *,
                          sample_hz: float = 50.0,
                          max_speed_scale: float = 1.0) -> JointTrajectory:
    """把关键点序列规划成 4-DOF 平滑轨迹。

    - 每个关键点精确经过；`stop=True`（默认）处速度归零停车；
    - `stop=False` 的中间点按 `_pass_speed` 的保守上界不停穿越（角点圆滑）；
    - 每段以最慢轴的时间为段时长，快轴到位后保持（与真机四电机并发一致），
      `sample(t)` 可任意时刻查询。

    max_speed_scale: 全局速度缩放（0~1，现场降速用）。
    """
    if len(keypoints) < 2:
        raise ValueError("至少需要 2 个关键点（start + goal）")
    kps = tuple(keypoints)
    max_speed_scale = _clamp(float(max_speed_scale), 0.0, 1.0)
    specs = _axis_specs()

    segments: List[JointSegment] = []
    for i in range(len(kps) - 1):
        a, b = kps[i], kps[i + 1]
        # 到达 b 时的速度：b 若要求停车或就是终点 → 0；否则可穿越上界
        if b.stop or i + 1 == len(kps) - 1:
            v_pass = 0.0
        else:
            v_pass = _pass_speed(a, b, kps[i + 2]) * max_speed_scale

        profiles = {}
        for attr, (v_max, a_max) in specs.items():
            d = getattr(b, attr) - getattr(a, attr)
            v_max_a = v_max * max_speed_scale
            prof = _trapezoid_v(d, 0.0, v_pass, v_max_a, a_max)
            profiles[attr] = prof
        T = max(p["t_total"] for p in profiles.values())
        if T < 1e-9:
            T = 0.0
        segments.append(JointSegment(start=a, end=b, profiles=profiles,
                                     T=T, pass_speed=v_pass))

    return JointTrajectory(tuple(kps), tuple(segments), sample_hz=sample_hz)
