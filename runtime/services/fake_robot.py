"""离线机械臂/底盘运动学仿真（纯标准库，不依赖任何硬件或推理包）。

用途
----
fake transport 的“真关节姿态”来源。业务层发出的机械臂动作包（composite_run /
composite_pick / goto_position / move_x_position / ...）在这里被解释成带
速度轮廓的关节轨迹，而不是直接把参数写进 state 字典——因此记录器里能拿到
“关节如何从 A 平滑走到 B”的中间姿态样本，而不是一个瞬间出现的假数值。

与 `main/arm/trajectory.py` 的关系
----------------------------------
本模块的多轴梯形速度规划是 `main/arm/trajectory.py` `TrajectoryGenerator` 的
纯 Python 移植（5 段梯形 + 时间归一化同步 + jerk 余量）。runtime 层不允许
import main，所以算法按 BSD 风格移植到本地，附归因注释；数学完全相同，只改
了命名与返回结构，便于 fake service 直接消费采样点做“动作展示”。

单位约定（与 smartcar/whalesbot/vehicle/arm/arm_base.py 一致）
--------------------------------------------------------------
- x / y：米（业务层 composite_run/goto_position/move_x_position 传 m）
- arm_angle / hand_angle：度（绝对值，范围见 Joint 规格）
- x_speed / y_speed：m/s（速度模式，只影响当前速度，需要 advance(dt) 积分）
- y=0 是磁感触底（最下），y<0 是向上；业务层 y 软区间约 [-0.18, 0]

正运动学（可视化模型，非硬件标定）
----------------------------------
- 载台：车架上的十字滑轨位置 (x_mm, y_mm)。
- 肩/肘：肘 = 载台 + L1·(cos arm_angle, sin arm_angle)，arm_angle=0 水平向前，
  +90 向上（复位位）。
- 手爪朝向角：gripper_angle = arm_angle - 90 - hand_angle
  （hand_angle=0 → 手爪垂直向下放料；hand_angle=-90 → 手爪沿臂上折 = UP 安全位）。
- 末端：肘 + L2·(cos gripper_angle, sin gripper_angle)。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional

# ---------------------------------------------------------------------------
# 1. 梯形速度规划（移植自 main/arm/trajectory.py TrajectoryGenerator）
# ---------------------------------------------------------------------------

DEFAULT_JERK = 2000.0  # 对应 main/arm/trajectory.py 的 J_MAX（mm/s³ 或 deg/s³）


def _trapezoid(d_signed: float, v_max: float, a_max: float) -> dict:
    """5 段梯形速度剖面。

    d_signed 为带符号位移。距离太短时退化为三角剖面（没有匀速段）。
    返回 {t_acc, t_run, t_dec, t_total, d_abs, sign, v_peak}。
    """
    d_abs = abs(d_signed)
    sign = 1.0 if d_signed >= 0 else -1.0
    if d_abs < 1e-12:
        return {"t_acc": 0.0, "t_run": 0.0, "t_dec": 0.0,
                "t_total": 0.0, "d_abs": 0.0, "sign": sign, "v_peak": 0.0}
    v_max = max(abs(v_max), 1e-9)
    a_max = max(abs(a_max), 1e-9)
    t_acc = v_max / a_max
    d_tri = v_max * v_max / a_max  # 三角剖面总位移（加速段+减速段）
    if d_abs >= d_tri:
        t_run = (d_abs - d_tri) / v_max
        v_peak = v_max
    else:
        # 三角剖面：达不到 v_max，先算加速时间
        t_acc = math.sqrt(d_abs / a_max)
        t_run = 0.0
        v_peak = a_max * t_acc
    t_dec = t_acc
    return {"t_acc": t_acc, "t_run": t_run, "t_dec": t_dec,
            "t_total": t_acc + t_run + t_dec,
            "d_abs": d_abs, "sign": sign, "v_peak": v_peak}


def _eval_trapezoid(prof: dict, t: float) -> tuple:
    """返回 (s, v)——t 时刻的位移与速度（位移带符号）。"""
    if prof["d_abs"] <= 1e-12:
        return (0.0, 0.0)
    sign = prof["sign"]
    t_acc, t_run, t_dec = prof["t_acc"], prof["t_run"], prof["t_dec"]
    t_total = prof["t_total"]
    v_peak = prof["v_peak"]
    a = v_peak / t_acc if t_acc > 0 else 0.0
    t = max(0.0, min(t, t_total))
    if t <= t_acc:
        s = 0.5 * a * t * t
        v = a * t
    elif t <= t_acc + t_run:
        s_acc = 0.5 * a * t_acc * t_acc
        s = s_acc + v_peak * (t - t_acc)
        v = v_peak
    else:
        t_dec_part = t - t_acc - t_run
        s_acc = 0.5 * a * t_acc * t_acc
        s_run = v_peak * t_run
        s = s_acc + s_run + v_peak * t_dec_part - 0.5 * a * t_dec_part * t_dec_part
        v = v_peak - a * t_dec_part
    return (sign * s, sign * v)


@dataclass
class MultiAxisPlan:
    """多轴同步规划：每个轴一条梯形剖面，整体时长 T = max(T_i) + 2·T_j。"""

    axes: Dict[str, dict] = field(default_factory=dict)  # name -> 轴元数据
    T: float = 0.0        # 总时长（含 jerk 余量）
    j_max: float = DEFAULT_JERK

    def _local_t(self, name: str, t: float) -> float:
        meta = self.axes[name]
        total = meta["profile"]["t_total"]
        if total <= 1e-12:
            return 0.0
        # 时间归一化：全局 t 按比例映射到该轴自己的梯形时长，超时 clamp 在终点
        return min(1.0, t / self.T) * total

    def evaluate(self, t: float) -> Dict[str, float]:
        """全局时间 t 下的各关节位置。"""
        out = {}
        for name, meta in self.axes.items():
            local = self._local_t(name, t)
            s, _v = _eval_trapezoid(meta["profile"], local)
            out[name] = meta["start"] + s
        return out

    def velocity(self, t: float) -> Dict[str, float]:
        out = {}
        for name, meta in self.axes.items():
            local = self._local_t(name, t)
            _s, v = _eval_trapezoid(meta["profile"], local)
            out[name] = v
        return out

    def samples(self, n_max: int = 60) -> List[tuple]:
        """均匀采样 (t, {joint: value})，供 recorder 做动作展示。"""
        if self.T <= 1e-12:
            return [(0.0, self.evaluate(0.0))]
        n = max(1, min(int(n_max), 256))
        out = []
        for i in range(n):
            t = self.T * i / (n - 1)
            out.append((t, self.evaluate(t)))
        return out


def plan_axes(starts: Dict[str, float], targets: Dict[str, float],
              v_max: Dict[str, float], a_max: Dict[str, float],
              j_max: float = DEFAULT_JERK) -> MultiAxisPlan:
    """把多个关节一起规划成同步轨迹（梯形 + jerk 余量）。

    - 每轴先用 (v_max, a_max) 算自己的 5 段梯形；
    - 整体时长 T = max(T_i) + 2·T_j，T_j = a_max/j_max 取最慢轴；
    - 每轴按 `_local_t` 时间归一化，保证到 T 时全部到位。
    """
    axes = {}
    longest = 0.0
    for name in targets:
        start = starts.get(name, 0.0)
        end = targets[name]
        prof = _trapezoid(end - start, v_max.get(name, 0.0), a_max.get(name, 0.0))
        axes[name] = {
            "start": start,
            "end": end,
            "profile": prof,
            "v_max": v_max.get(name, 0.0),
            "a_max": a_max.get(name, 0.0),
        }
        longest = max(longest, prof["t_total"])
    # jerk 余量：对应 TrajectoryGenerator 的 T = max(T_x, T_y) + 2*T_j
    t_j = longest / j_max if j_max > 0 else 0.0
    plan = MultiAxisPlan(axes=axes, T=longest + 2.0 * t_j, j_max=j_max)
    # 全部为零位移的退化情形
    if longest <= 1e-12:
        plan.T = 0.0
    return plan


# ---------------------------------------------------------------------------
# 2. 关节 + 机器人仿真
# ---------------------------------------------------------------------------

DEFAULT_JOINT_SPECS = {
    # name: (lo, hi, v_max, a_max)
    "x_mm":        {"lo": -300.0, "hi": 300.0,  "v_max": 100.0, "a_max": 400.0},
    "y_mm":        {"lo": -200.0, "hi": 0.0,    "v_max": 40.0,  "a_max": 200.0},
    "arm_angle":   {"lo": -150.0, "hi": 150.0,  "v_max": 150.0, "a_max": 300.0},
    "hand_angle":  {"lo": -90.0,  "hi": 10.0,   "v_max": 180.0, "a_max": 360.0},
}

# reset_y 触底归零后的收尾目标（mirror arm_base.POST_RESET_TARGET_M）
POST_RESET_Y_MM = -150.0


def forward_kinematics(x_mm: float, y_mm: float, arm_angle: float,
                       hand_angle: float, l1: float = 0.20, l2: float = 0.12) -> Dict[str, float]:
    """2D 正运动学（可视化模型）：给定四关节值返回末端位置与姿态角。

    供 fake runtime 对轨迹采样点逐帧计算末端位姿（显示“关节如何从 A 平滑走到
    B”时的真实 FK），与 `FakeRobotSim.end_effector_pose` 等价。
    """
    arm = math.radians(arm_angle)
    gripper = math.radians(arm_angle - 90.0 - hand_angle)
    cx = x_mm / 1000.0
    cy = y_mm / 1000.0
    ex = cx + l1 * math.cos(arm)
    ey = cy + l1 * math.sin(arm)
    gx = ex + l2 * math.cos(gripper)
    gy = ey + l2 * math.sin(gripper)
    return {
        "carriage_x_m": cx, "carriage_y_m": cy,
        "elbow_x_m": ex, "elbow_y_m": ey,
        "ee_x_m": gx, "ee_y_m": gy,
        "gripper_deg": math.degrees(gripper),
    }


class Joint:
    """单个关节：当前值 + 限位 + 速度/加速度规格。"""

    def __init__(self, name, lo=-math.inf, hi=math.inf, v_max=0.0, a_max=0.0,
                 value=0.0):
        self.name = name
        self.lo = float(lo)
        self.hi = float(hi)
        self.v_max = float(v_max)
        self.a_max = float(a_max)
        self.value = self.clamp(value)

    def clamp(self, v):
        v = float(v)
        if v < self.lo:
            return self.lo
        if v > self.hi:
            return self.hi
        return v

    def set(self, v):
        self.value = self.clamp(v)
        return self.value


@dataclass
class MotionResult:
    """一次机械臂动作的仿真结果，供 recorder 回放。"""

    action: str
    plan: MultiAxisPlan
    samples: List[tuple] = field(default_factory=list)
    final: Dict[str, float] = field(default_factory=dict)
    duration: float = 0.0


class FakeRobotSim:
    """离线机械臂 + 底盘模型。

    - 关节：x_mm / y_mm / arm_angle / hand_angle，全部走梯形轨迹；
    - 速度模式：x_speed / y_speed 只改速度，需要 advance(dt) 积分；
    - 底盘：move_for 相对位移 → odom；set_chassis_velocity / set_wheel_speeds
      只记 wheels，不积分（与现 fake 行为一致，odom 由 move_for 驱动）；
    - grasp：真空泵布尔状态；
    - feeds：lane / arm / ir / odom 守护线程开/关标志（默认 lane/arm ON，
      与生产 runtime 默认行为一致）。
    """

    def __init__(self, *, joint_specs: Optional[Dict] = None,
                 l1: float = 0.20, l2: float = 0.12):
        specs = joint_specs or DEFAULT_JOINT_SPECS
        self.joints = {
            name: Joint(name, **spec) for name, spec in specs.items()
        }
        # 正运动学连杆长度（m）
        self.l1 = float(l1)
        self.l2 = float(l2)
        self.grasped = False
        self.side = "MID"
        self.t = 0.0
        self._vel = {"x": 0.0, "y": 0.0}  # m/s，速度模式积分用
        self.odom = {"x": 0.0, "y": 0.0, "theta": 0.0, "distance": 0.0}
        self.wheels = [0.0, 0.0, 0.0, 0.0]
        self.feeds = {"lane": True, "arm": True, "ir": False, "odom": True}
        # 传感器注入（供 fixture / 任务 harness 使用）
        self.lane_detections = []
        self.task_detections = []
        self.ir_distances = {"left": None, "right": None}
        self.ocr_result = {"ok": False, "text": "", "order": []}
        # 最近一次多轴规划（动作展示用）
        self.last_plan: Optional[MultiAxisPlan] = None

    # ---------------- 关节访问 ----------------

    def joint(self, name) -> Joint:
        return self.joints[name]

    def arm_state_mm(self) -> Dict[str, float]:
        return {
            "x_mm": self.joints["x_mm"].value,
            "y_mm": self.joints["y_mm"].value,
            "arm_angle": self.joints["arm_angle"].value,
            "hand_angle": self.joints["hand_angle"].value,
        }

    # ---------------- 轨迹动作 ----------------

    def composite_move(self, targets: Dict[str, float],
                       v_max: Optional[Dict[str, float]] = None,
                       a_max: Optional[Dict[str, float]] = None) -> MotionResult:
        """把多个关节同步移动到目标（米/度）。返回带采样的 MotionResult。

        速度模式未结束的 x/y 先停住（清零速度），再进位置模式。
        """
        self._vel = {"x": 0.0, "y": 0.0}
        starts = self.arm_state_mm()
        plans = {}
        # 过滤空目标
        targets = {k: float(v) for k, v in targets.items() if v is not None}
        for name in targets:
            joints = self.joints
            j = joints.get(name)
            if j is None:
                continue
            targets[name] = j.clamp(targets[name])
            plans[name] = (j.value, targets[name])
        starts = {name: p[0] for name, p in plans.items()}
        ends = {name: p[1] for name, p in plans.items()}
        v_lim = {name: (v_max or {}).get(name, self.joints[name].v_max)
                 for name in ends}
        a_lim = {name: (a_max or {}).get(name, self.joints[name].a_max)
                 for name in ends}
        plan = plan_axes(starts, ends, v_lim, a_lim)
        self.last_plan = plan
        for name, end in ends.items():
            self.joints[name].set(end)
        return MotionResult(
            action="composite_move",
            plan=plan,
            samples=plan.samples(),
            final=self.arm_state_mm(),
            duration=plan.T,
        )

    def move_joint(self, name: str, target: float,
                   v_max: Optional[float] = None,
                   a_max: Optional[float] = None) -> MotionResult:
        j = self.joints[name]
        return self.composite_move(
            {name: target},
            v_max={name: v_max} if v_max is not None else None,
            a_max={name: a_max} if a_max is not None else None,
        )

    # ---------------- 速度模式（realtime，不创建 job） ----------------

    def velocity_x(self, v: float):
        self._vel["x"] = float(v)

    def velocity_y(self, v: float):
        self._vel["y"] = float(v)

    def advance(self, dt: float):
        """推进仿真时钟，并对速度模式关节做积分。"""
        dt = float(dt)
        self.t += dt
        if self._vel["x"]:
            j = self.joints["x_mm"]
            j.value = j.clamp(j.value + self._vel["x"] * 1000.0 * dt)
        if self._vel["y"]:
            j = self.joints["y_mm"]
            j.value = j.clamp(j.value + self._vel["y"] * 1000.0 * dt)

    # ---------------- 复位 ----------------

    def reset_y(self) -> MotionResult:
        """触底归零 → 升到 POST_RESET_Y_MM（mirror arm_base.reset_y 收尾）。"""
        self.joints["y_mm"].set(0.0)
        return self.move_joint("y_mm", POST_RESET_Y_MM)

    def reset_x(self, direction: str = "right") -> MotionResult:
        """撞墙定 x 原点（mirror arm_base.reset_x 语义）。"""
        return self.move_joint("x_mm", 0.0)

    def reset_position(self) -> MotionResult:
        """arm→+90 UP，hand→-90 UP，然后触底归零后升到 -150mm（x 不参与，mirror arm_base.reset_position）。"""
        self.composite_move({"arm_angle": 90.0, "hand_angle": -90.0})
        return self.reset_y()

    # ---------------- 底盘 ----------------

    def move_for(self, dx, dy, dtheta):
        self.odom["x"] += float(dx)
        self.odom["y"] += float(dy)
        self.odom["theta"] += float(dtheta)
        self.odom["distance"] += abs(float(dx)) + abs(float(dy))
        return dict(self.odom)

    def set_wheels(self, speeds):
        self.wheels = [float(s) for s in speeds]

    def set_chassis_velocity(self, vx, vy, wz):
        self.wheels = [float(vx)] * 4  # 简化模型：直线速度映射到四轮

    def emergency_stop(self):
        self.wheels = [0.0, 0.0, 0.0, 0.0]
        self.velocity_x(0.0)
        self.velocity_y(0.0)

    # ---------------- 正运动学 ----------------

    def end_effector_pose(self) -> Dict[str, float]:
        """2D 正运动学（可视化模型）：返回末端位置与姿态角。"""
        return forward_kinematics(
            self.joints["x_mm"].value,
            self.joints["y_mm"].value,
            self.joints["arm_angle"].value,
            self.joints["hand_angle"].value,
            l1=self.l1, l2=self.l2,
        )

    def posture_snapshot(self) -> Dict:
        """生产形状的 arm_state（含 active/side/y_limit，供业务快路径）。"""
        return {
            "active": True,
            "mode": "sim",
            "x_mm": self.joints["x_mm"].value,
            "y_mm": self.joints["y_mm"].value,
            "arm_angle": self.joints["arm_angle"].value,
            "hand_angle": self.joints["hand_angle"].value,
            "grasped": self.grasped,
            "side": self.side,
            "y_limit": {"lo_mm": -200.0, "hi_mm": 0.0},
            "t": self.t,
        }


# ---------------------------------------------------------------------------
# 3. 对齐方法注册表（几种对齐策略，全部输出可观察命令包）
# ---------------------------------------------------------------------------
#
# 每种策略的 step() 接收归一化误差（传感器坐标系），输出一份命令包：
#   {"target": "car"|"arm", "name": "set_chassis_velocity"|"composite_run"|...,
#    "args": [...], "kwargs": {...}}
# 这样任务的“对齐行为”也能在 recorder 里以动作包形式被断言/回放。


def _vel_packet(vx=0.0, vy=0.0, wz=0.0):
    return {"target": "car", "name": "set_chassis_velocity",
            "args": [], "kwargs": {"vx": vx, "vy": vy, "wz": wz}}


class ProportionalAligner:
    """比例对齐：v = kp·err，clamp 到 [±v_max]。"""

    def __init__(self, kp=0.6, v_max=0.2, axis="x"):
        self.kp = float(kp)
        self.v_max = float(v_max)
        self.axis = axis

    def step(self, err, dt=0.02):
        v = max(-self.v_max, min(self.v_max, self.kp * err))
        if self.axis == "wz":
            return _vel_packet(wz=v)
        if self.axis == "y":
            return _vel_packet(vy=v)
        return _vel_packet(vx=v)


class PidAligner:
    """PID 对齐（带积分限幅 + 微分低通）。"""

    def __init__(self, kp=0.5, ki=0.1, kd=0.05, v_max=0.2, i_max=0.05,
                 deadband=0.005, axis="x"):
        self.kp, self.ki, self.kd = float(kp), float(ki), float(kd)
        self.v_max, self.i_max, self.deadband = float(v_max), float(i_max), float(deadband)
        self.axis = axis
        self._i = 0.0
        self._prev_err = 0.0

    def step(self, err, dt=0.02):
        if abs(err) < self.deadband:
            err = 0.0
        self._i = max(-self.i_max, min(self.i_max, self._i + err * dt))
        d = (err - self._prev_err) / max(dt, 1e-6)
        self._prev_err = err
        v = self.kp * err + self.ki * self._i + self.kd * d
        v = max(-self.v_max, min(self.v_max, v))
        if self.axis == "wz":
            return _vel_packet(wz=v)
        if self.axis == "y":
            return _vel_packet(vy=v)
        return _vel_packet(vx=v)


class CoarseFineAligner:
    """粗-细（死区）对齐：|err|>coarse → 全速；粗阈值内 → 慢速；细死区内 → 0。"""

    def __init__(self, coarse=0.05, fine=0.005, v_coarse=0.2, v_fine=0.03,
                 axis="x"):
        self.coarse, self.fine = float(coarse), float(fine)
        self.v_coarse, self.v_fine = float(v_coarse), float(v_fine)
        self.axis = axis

    def step(self, err, dt=0.02):
        a = abs(err)
        if a <= self.fine:
            v = 0.0
        elif a <= self.coarse:
            v = self.v_fine if err > 0 else -self.v_fine
        else:
            v = self.v_coarse if err > 0 else -self.v_coarse
        if self.axis == "wz":
            return _vel_packet(wz=v)
        if self.axis == "y":
            return _vel_packet(vy=v)
        return _vel_packet(vx=v)


class StanleyAligner:
    """Stanley 横向跟踪（前视/横切误差 + 航向误差）：

    转向角 = atan2(k_steer·cross_track, v) + heading_err，适用于路径跟随。
    返回带 vx（前进）与 wz（转向）的底盘命令包。
    """

    def __init__(self, v_forward=0.15, k_steer=2.0, max_steer=0.6):
        self.v_forward = float(v_forward)
        self.k_steer = float(k_steer)
        self.max_steer = float(max_steer)

    def step(self, state, dt=0.02):
        # state = (cross_track_error_m, heading_error_rad)
        cte, he = state
        steer = math.atan2(self.k_steer * cte, max(self.v_forward, 1e-3)) + he
        steer = max(-self.max_steer, min(self.max_steer, steer))
        return _vel_packet(vx=self.v_forward, wz=steer)


class HeadingAligner:
    """纯航向对齐：wz = clamp(kp·heading_err)。"""

    def __init__(self, kp=1.5, wz_max=0.8):
        self.kp = float(kp)
        self.wz_max = float(wz_max)

    def step(self, err, dt=0.02):
        v = max(-self.wz_max, min(self.wz_max, self.kp * err))
        return _vel_packet(wz=v)


ALIGNMENT_STRATEGIES = {
    "proportional": ProportionalAligner,
    "pid": PidAligner,
    "coarse_fine": CoarseFineAligner,
    "stanley": StanleyAligner,
    "heading": HeadingAligner,
}


def create_aligner(name: str, **kwargs):
    """按名创建对齐策略。"""
    cls = ALIGNMENT_STRATEGIES.get(name)
    if cls is None:
        raise KeyError("未知对齐策略: %s（可选 %s）"
                       % (name, sorted(ALIGNMENT_STRATEGIES)))
    return cls(**kwargs)
