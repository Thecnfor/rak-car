"""main/arm/api.py
ArmClient：薄封装 RuntimeApiClient + RuntimeWsClient，专给机械臂用。

约定：
  - 只 import main.*，不 import smartcar / runtime
  - 业务单位统一 mm（API 层进车端时换算 m）
  - move_xy / move_x / move_y 底层调 arm.goto_position / arm.move_x_position / arm.move_y_position
    （车端 PID 闭环），同时客户端用 TrajectoryGenerator 做 dry-run 算 t_total 给日志
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    from main.api_client import RuntimeApiClient
    from main.ws_client import RuntimeWsClient
except ImportError:  # pragma: no cover
    from api_client import RuntimeApiClient  # type: ignore
    from ws_client import RuntimeWsClient  # type: ignore

from .state import ArmState, ArmOrigin, SIDES, HANDS
from .trajectory import TrajectoryGenerator, TrajectoryPlan


def _mm_to_m(v_mm: float) -> float:
    return float(v_mm) / 1000.0


def _m_to_mm(v_m) -> float:
    return float(v_m) * 1000.0


def _normalize_side(side: Optional[str]) -> Optional[str]:
    if side is None:
        return None
    s = side.upper()
    if s not in SIDES:
        raise ValueError(f"side 必须是 {SIDES} 之一，收到: {side!r}")
    return s


def _normalize_hand(hand: Optional[str]) -> Optional[str]:
    if hand is None:
        return None
    h = hand.upper()
    if h not in HANDS:
        raise ValueError(f"hand 必须是 {HANDS} 之一，收到: {hand!r}")
    return h


@dataclass
class ArmClient:
    """机械臂专用 client。薄封装 main.api_client / main.ws_client。"""

    http: RuntimeApiClient
    ws: Optional[RuntimeWsClient] = None
    ws_ready: bool = False
    origin: Optional[ArmOrigin] = None
    traj: TrajectoryGenerator = None  # type: ignore

    def __init__(self, http: RuntimeApiClient, ws: Optional[RuntimeWsClient] = None,
                 origin: Optional[ArmOrigin] = None,
                 traj: Optional[TrajectoryGenerator] = None):
        self.http = http
        self.ws = ws
        self.ws_ready = False
        self.origin = origin or ArmOrigin()
        self.traj = traj or TrajectoryGenerator()

    @classmethod
    def connect(cls, load_origin: bool = True) -> "ArmClient":
        http = RuntimeApiClient()
        ws: Optional[RuntimeWsClient] = None
        ready = False
        try:
            ws = RuntimeWsClient()
            ws.connect()
            ready = True
        except Exception:
            ready = False
        client = cls(http=http, ws=ws)
        client.ws_ready = ready
        if load_origin:
            client._load_origin_or_default()
        return client

    # ---- origin 持久化 ----

    def _origin_path(self) -> str:
        # 与 main/arm/__init__.py 同目录
        import os
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, "arm_origin.yaml")

    def _load_origin_or_default(self) -> ArmOrigin:
        import os
        path = self._origin_path()
        if os.path.exists(path):
            try:
                self.origin = self._read_origin_yaml(path)
                return self.origin
            except Exception:
                pass
        self.origin = ArmOrigin()
        return self.origin

    @staticmethod
    def _read_origin_yaml(path: str) -> ArmOrigin:
        # 极简 YAML 解析（项目里其他地方也在用 yaml，这里避免循环依赖）
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return ArmOrigin(
            y_origin_m=float(data.get("y_origin_m", 0.0)),
            x_origin_m=float(data.get("x_origin_m", 0.0)),
            x_wall=str(data.get("x_wall", "left")),
            soft_y_max_m=float(data.get("soft_y_max_m", 0.18)),
            soft_x_min_m=float(data.get("soft_x_min_m", 0.005)),
            soft_x_max_m=float(data.get("soft_x_max_m", 0.30)),
            calibrated_at=str(data.get("calibrated_at", "")),
        )

    def save_origin(self, origin: ArmOrigin) -> None:
        import yaml
        self.origin = origin
        path = self._origin_path()
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                {
                    "y_origin_m": origin.y_origin_m,
                    "x_origin_m": origin.x_origin_m,
                    "x_wall": origin.x_wall,
                    "soft_y_max_m": origin.soft_y_max_m,
                    "soft_x_min_m": origin.soft_x_min_m,
                    "soft_x_max_m": origin.soft_x_max_m,
                    "calibrated_at": origin.calibrated_at,
                },
                f,
                allow_unicode=True,
                sort_keys=False,
            )

    # ---- 底层便捷调用 ----

    def _call_arm(self, name: str, timeout: float = 20.0, *args, **kwargs) -> dict:
        return self.http.execute_arm_action(name, *args, timeout=timeout, **kwargs)

    def _call_car(self, name: str, timeout: float = 20.0, *args, **kwargs) -> dict:
        return self.http.execute_car_action(name, *args, timeout=timeout, **kwargs)

    # ---- 业务动作 ----

    def set_pose(
        self,
        x_mm: Optional[float] = None,
        y_mm: Optional[float] = None,
        side: Optional[str] = None,
        hand: Optional[str] = None,
        timeout: float = 30.0,
    ) -> dict:
        """一次设置 x/y/side/hand，None 表示不动。"""
        side = _normalize_side(side)
        hand = _normalize_hand(hand)
        x_m = _mm_to_m(x_mm) if x_mm is not None else None
        y_m = _mm_to_m(y_mm) if y_mm is not None else None
        self._check_safe(x_mm=x_mm, y_mm=y_mm)
        return self._call_arm(
            "set_arm_pose",
            timeout=timeout,
            x=x_m, y=y_m,
            arm=side, hand=hand,
        )

    def move_xy(
        self,
        x_mm: float,
        y_mm: float,
        v_max_mms: float = 150.0,
        a_max_mms2: float = 400.0,
        timeout: Optional[float] = None,
    ) -> dict:
        """双轴同步移动到 (x_mm, y_mm)。

        底层调 arm.goto_position（车端 PID）。
        客户端用 TrajectoryGenerator 做 dry-run，给出预估时长。
        """
        self._check_safe(x_mm=x_mm, y_mm=y_mm)
        state = self.get_state()
        plan = self.traj.plan_xy(
            x0=state.x_mm, y0=state.y_mm,
            x1=x_mm, y1=y_mm,
            v_max=v_max_mms, a_max=a_max_mms2,
        )
        if timeout is None:
            # dry-run 时间 × 2 + 1s 兜底，最少 5s
            timeout = max(5.0, plan.T * 2.0 + 1.0)
        return self._call_arm(
            "goto_position",
            timeout=timeout,
            x=_mm_to_m(x_mm), y=_mm_to_m(y_mm),
        )

    def move_y(self, y_mm: float, v_max_mms: float = 80.0, timeout: float = 20.0) -> dict:
        self._check_safe(y_mm=y_mm)
        return self._call_arm(
            "move_y_position",
            timeout=timeout,
            target=_mm_to_m(y_mm),
        )

    def move_x(self, x_mm: float, v_max_mms: float = 150.0, timeout: float = 20.0) -> dict:
        self._check_safe(x_mm=x_mm)
        return self._call_arm(
            "move_x_position",
            timeout=timeout,
            target=_mm_to_m(x_mm),
        )

    def set_side(self, side: str, speed: int = 80, timeout: float = 10.0) -> dict:
        side = _normalize_side(side)
        if side is None:
            raise ValueError("set_side 必须给 LEFT/MID/RIGHT")
        return self._call_arm("set_arm_angle", timeout=timeout, angle=side, speed=speed)

    def set_hand(self, hand: str, speed: int = 80, timeout: float = 10.0) -> dict:
        hand = _normalize_hand(hand)
        if hand is None:
            raise ValueError("set_hand 必须给 UP/MID/DOWN")
        return self._call_arm("set_hand_angle", timeout=timeout, angle=hand, speed=speed)

    def grasp(self, on: bool, timeout: float = 10.0) -> dict:
        return self._call_arm("grasp", bool(on), timeout=timeout)

    # ---- reset ----

    def reset_y(self, timeout: float = 30.0) -> dict:
        return self._call_arm("reset_position", timeout=timeout)  # y + x 一起复位

    def reset_x(self, timeout: float = 30.0) -> dict:
        return self._call_arm("reset_x", timeout=timeout)

    def reset_origin(self, x_wall: str = "left", timeout: float = 60.0) -> dict:
        """主动撞一侧墙 + 触底，作为业务坐标系新原点。

        走 arm.reset_position (车端会 reset_y 触底 + reset_x 堵转)；
        然后写 arm_origin.yaml。
        """
        if x_wall not in ("left", "right"):
            raise ValueError("x_wall 必须是 'left' 或 'right'")
        job = self._call_arm("reset_position", timeout=timeout)
        # 重新读一次 y/x 原始坐标，作为新原点
        st = self._read_raw_state()
        new_origin = ArmOrigin(
            y_origin_m=st["raw_y_m"],
            x_origin_m=st["raw_x_m"],
            x_wall=x_wall,
            soft_y_max_m=self.origin.soft_y_max_m if self.origin else 0.18,
            soft_x_min_m=self.origin.soft_x_min_m if self.origin else 0.005,
            soft_x_max_m=self.origin.soft_x_max_m if self.origin else 0.30,
            calibrated_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )
        self.save_origin(new_origin)
        return job

    # ---- 状态读取 ----

    def _read_raw_state(self) -> dict:
        """从车端读原始 y/x 值，单位 m。"""
        try:
            y_job = self._call_arm("y_get_position", timeout=10.0)
            y_val = y_job.get("result") if isinstance(y_job, dict) else None
        except Exception:
            y_val = None
        try:
            x_job = self._call_arm("x_get_position", timeout=10.0)
            x_val = x_job.get("result") if isinstance(x_job, dict) else None
        except Exception:
            x_val = None
        return {"raw_x_m": float(x_val) if x_val is not None else 0.0,
                "raw_y_m": float(y_val) if y_val is not None else 0.0}

    def get_state(self) -> ArmState:
        raw = self._read_raw_state()
        st_job = self._call_car("get_arm_state", timeout=10.0)
        st_data = st_job.get("result") if isinstance(st_job, dict) else {}
        if not isinstance(st_data, dict):
            st_data = {}
        side = str(st_data.get("side", "MID")).upper()
        hand = str(st_data.get("hand_angle", "UP")).upper()
        if hand not in HANDS:
            hand = "UP"
        if side not in SIDES:
            side = "MID"
        origin = self.origin or ArmOrigin()
        return ArmState(
            x_mm=_m_to_mm(raw["raw_x_m"]),
            y_mm=_m_to_mm(raw["raw_y_m"]),
            side=side,
            hand=hand,
            grasping=False,  # 车端没暴露 grasping 字段
            y_origin_valid=bool(st_data.get("y_limit", False)),  # 注意：y_limit 字段语义是 "达到限位"
            x_origin_valid=False,
            soft_y_max_mm=origin.soft_y_max_mm,
            soft_x_min_mm=origin.soft_x_min_mm,
            soft_x_max_mm=origin.soft_x_max_mm,
            raw_x_m=raw["raw_x_m"],
            raw_y_m=raw["raw_y_m"],
            arm_angle=st_data.get("arm_angle"),
            hand_angle=st_data.get("hand_angle"),
        )

    def get_pose_mm(self) -> Tuple[float, float, str, str]:
        st = self.get_state()
        return st.x_mm, st.y_mm, st.side, st.hand

    def get_x_mm(self) -> float:
        return self.get_state().x_mm

    def get_y_mm(self) -> float:
        return self.get_state().y_mm

    # ---- 安全 ----

    def _check_safe(self, x_mm: Optional[float] = None, y_mm: Optional[float] = None) -> None:
        origin = self.origin or ArmOrigin()
        if y_mm is not None and not (0.0 <= y_mm <= origin.soft_y_max_mm):
            raise ValueError(
                f"y_mm={y_mm} 超出软上限 {origin.soft_y_max_mm}mm（触底=0, top={origin.soft_y_max_mm:.0f}mm）"
            )
        if x_mm is not None and not (origin.soft_x_min_mm <= x_mm <= origin.soft_x_max_mm):
            raise ValueError(
                f"x_mm={x_mm} 超出软区间 [{origin.soft_x_min_mm}, {origin.soft_x_max_mm}] mm"
            )

    def emergency_stop(self) -> dict:
        return self.http.emergency_stop()

    def ping(self, timeout: float = 5.0) -> bool:
        try:
            self.http.get_health(timeout=timeout)
            return True
        except Exception:
            return False
