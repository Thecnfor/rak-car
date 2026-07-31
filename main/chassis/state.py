"""main/chassis/state.py
外环数据 shape。底盘组的控制律只接 dataclass，不接 dict。

所有 dataclass 字段语义：
- 数值型 (float / int) None 含义：feed 未就绪 / 数据缺失
- mode(str) 取值：runtime feed 的 mode 字段（"xxx_feed" / "idle" / "stopped"）
- age_ms(float) None 含义：feed 从未刷新过 / payload 无 updated_at
- is_fresh：age_ms < 500ms 视为新鲜（与外环 50Hz × 2 帧对齐）
- active：对应 feed 守护线程是否在跑（mode == "<name>_feed"）

构造入口：`from_<name>_state_payload(payload)` 接受：
  - 缓存原始 payload: `{"ir_state": {...}}` / `{"odom_state": {...}}`
  - 裸 inner dict: `{...}`
  - None / 非 dict: 视为空,字段全 None（不抛异常）
"""
import time
from dataclasses import dataclass
from typing import Optional, List


def _age_ms_from(updated_at, now: Optional[float] = None) -> Optional[float]:
    """根据 updated_at 时间戳计算 age_ms。None / 非数字 → None。"""
    if not isinstance(updated_at, (int, float)):
        return None
    if now is None:
        now = time.time()
    return max(0.0, (float(now) - float(updated_at)) * 1000.0)


def _unwrap(payload, key: str) -> dict:
    """接受 `{"<key>": {...}}` 或裸 `{...}`,统一返 inner dict。None/非 dict→空 dict。"""
    if isinstance(payload, dict):
        inner = payload.get(key)
        if isinstance(inner, dict):
            return inner
        # 没有 <key> 包裹,但是 dict 也可以当作 inner（兼容裸 shape 调用）
        if not inner:
            return payload
    return {}


def _safe_float(v) -> Optional[float]:
    try:
        return float(v) if v is not None else None
    except Exception:
        return None


# ============ LaneState (原有,2026-07-31 把 age 计算移出,代码本身不变) ============


@dataclass
class LaneState:
    """lane 误差缓存视图，来自 runtime 的 /v1/vision/lane/state。"""

    error_y: Optional[float] = None
    error_angle: Optional[float] = None
    forward: Optional[float] = None
    lateral: Optional[float] = None
    angular: Optional[float] = None
    distance: Optional[float] = None
    mode: Optional[str] = None
    age_ms: Optional[float] = None

    @classmethod
    def from_lane_state_payload(cls, payload: dict, now: Optional[float] = None) -> "LaneState":
        return cls(
            error_y=payload.get("error_y"),
            error_angle=payload.get("error_angle"),
            forward=payload.get("forward_speed"),
            lateral=payload.get("lateral_speed"),
            angular=payload.get("angular_speed"),
            distance=payload.get("distance"),
            mode=payload.get("mode"),
            age_ms=_age_ms_from(payload.get("updated_at"), now=now),
        )

    @property
    def is_fresh(self) -> bool:
        """<500ms 视为新鲜，外环 50Hz × 2 帧还合理。"""
        return self.age_ms is not None and self.age_ms < 500.0

    @property
    def has_error(self) -> bool:
        """error_y / error_angle 都非 None，控制律可以算出 vx/vy/w。"""
        return self.error_y is not None and self.error_angle is not None


# ============ IrState (2026-07-31) ============


@dataclass
class IrState:
    """左右红外距离缓存视图，来自 runtime 的 /v1/realtime/ir/state（2026-07-31）。

    数据源：ir_feed 守护线程 50Hz 喂 streamer.ir_state（meta_lock 路径），
    不进 job_queue、不打 MC602、不抢 car_lock，业务层读缓存延迟 <2ms。

    left/right：用户视角（与 main/chassis/tasks/read_ir.py 语义一致——
                底层 left_sensor/right_sensor 与物理端口的语义经 _FLIP_SIDE 调换）。
    """

    left: Optional[float] = None       # m
    right: Optional[float] = None      # m
    mode: Optional[str] = None         # ir_feed / idle / stopped
    age_ms: Optional[float] = None

    @classmethod
    def from_ir_state_payload(cls, payload, now: Optional[float] = None) -> "IrState":
        inner = _unwrap(payload, "ir_state")
        return cls(
            left=inner.get("left"),
            right=inner.get("right"),
            mode=inner.get("mode"),
            age_ms=_age_ms_from(inner.get("updated_at"), now=now),
        )

    @property
    def active(self) -> bool:
        """ir_feed 守护线程是否在跑（mode == "ir_feed"）。"""
        return self.mode == "ir_feed"

    @property
    def is_fresh(self) -> bool:
        """<500ms 视为新鲜（与 LaneState 同档）。"""
        return self.age_ms is not None and self.age_ms < 500.0


# ============ OdometryState (2026-07-31 扩展) ============


@dataclass
class OdometryState:
    """底盘里程计缓存视图，来自 runtime 的 /v1/realtime/odom/state（2026-07-31）。

    数据源：odom_feed 守护线程 50Hz 喂 streamer.odom_state。
    不进 job_queue、不打 MC602、不抢 car_lock，业务层读缓存延迟 <2ms。
    """

    x: Optional[float] = None
    y: Optional[float] = None
    theta: Optional[float] = None
    distance: Optional[float] = None
    mode: Optional[str] = None
    age_ms: Optional[float] = None

    @classmethod
    def from_odom_state_payload(cls, payload, now: Optional[float] = None) -> "OdometryState":
        inner = _unwrap(payload, "odom_state")
        return cls(
            x=_safe_float(inner.get("x")),
            y=_safe_float(inner.get("y")),
            theta=_safe_float(inner.get("theta")),
            distance=_safe_float(inner.get("distance")),
            mode=inner.get("mode"),
            age_ms=_age_ms_from(inner.get("updated_at"), now=now),
        )

    @property
    def active(self) -> bool:
        return self.mode == "odom_feed"

    @property
    def is_fresh(self) -> bool:
        """<500ms 视为新鲜（与 LaneState 同档）。"""
        return self.age_ms is not None and self.age_ms < 500.0


# ============ WheelsState (2026-07-31 新增)============


@dataclass
class WheelsState:
    """底盘 4 路电机 RPM 缓存视图（fast-path,2026-07-31）。

    数据源：wheels_feed 守护线程 50Hz 喂 streamer.wheels_state。
    不进 job_queue、不打 MC602、不抢 car_lock，业务层读缓存延迟 <2ms。

    注意:目前 lane_feed / arm_feed / task_feed 都是单独缓存,wheels_state
    当前由 SDK 自带的 update_odometry_thread 20Hz 提供;后续若需要单独
    提高频率,会通过新的 wheels_feed 守护线程实现。
    """

    fl_rpm: Optional[float] = None     # 前左 rpm
    fr_rpm: Optional[float] = None     # 前右 rpm
    rl_rpm: Optional[float] = None     # 后左 rpm
    rr_rpm: Optional[float] = None     # 后右 rpm
    mode: Optional[str] = None         # wheels_feed / idle / stopped
    age_ms: Optional[float] = None

    @classmethod
    def from_wheels_state_payload(cls, payload, now: Optional[float] = None) -> "WheelsState":
        inner = _unwrap(payload, "wheels_state")
        return cls(
            fl_rpm=_safe_float(inner.get("fl_rpm")),
            fr_rpm=_safe_float(inner.get("fr_rpm")),
            rl_rpm=_safe_float(inner.get("rl_rpm")),
            rr_rpm=_safe_float(inner.get("rr_rpm")),
            mode=inner.get("mode"),
            age_ms=_age_ms_from(inner.get("updated_at"), now=now),
        )

    @property
    def active(self) -> bool:
        return self.mode == "wheels_feed"

    @property
    def is_fresh(self) -> bool:
        """<500ms 视为新鲜（与 LaneState 同档）。"""
        return self.age_ms is not None and self.age_ms < 500.0


# ============ 旧底层数据结构 (保留,WheelsState 现在仅 sped/encoder) ============


@dataclass
class WheelsRawState:
    """四轮线速度 + 编码器读数（弧度累计）。

    由 `ChassisClient.get_wheel_encoders()` + `set_wheel_speeds` 的回看数据组成,
    调试/跑 trace 用。生产路径用上方 `WheelsState` (RPM cache) 即可。
    """

    speeds: List[float]
    encoders: List[float]
