"""main/arm/api/__init__.py — ArmClient 聚合类.

8 mixin 顺序: Safety → Motion/Setters/Composite/Reset/Storage → StateIO → VisServo
(VisServo 在最后, 因为 vision 内部调 motion).
"""
from __future__ import annotations

import logging
import os
import threading
from typing import Optional

from main.api_client import RuntimeApiClient
from main.ws_client import RuntimeWsClient

from ..state import ArmOrigin
from ..trajectory import TrajectoryGenerator
from .composite import CompositeMixin
from .motion import MotionMixin
from .reset_ops import ResetOpsMixin
from .safety import ArmSafetyError, SafetyMixin
from .setters import SettersMixin
from .state_io import StateIOMixin
from .storage import StorageMixin
from .vis_servo import VisServoMixin

logger = logging.getLogger(__name__)


class ArmClient(SafetyMixin, MotionMixin, SettersMixin, CompositeMixin,
                ResetOpsMixin, StorageMixin, StateIOMixin, VisServoMixin):
    """机械臂专用 client. 薄封装 main.api_client / main.ws_client."""

    def __init__(self, http: RuntimeApiClient,
                 ws: Optional[RuntimeWsClient] = None,
                 origin: Optional[ArmOrigin] = None,
                 traj: Optional[TrajectoryGenerator] = None):
        self.http = http
        self.ws = ws
        self.ws_ready = False
        self.origin = origin or ArmOrigin()
        self.traj = traj or TrajectoryGenerator()
        self._vision: Optional[object] = None
        self._storage_side_cache = "UNKNOWN"
        # x_speed safety watchdog 状态 (2026-08-01 补: mixin 拆分 c9fbc99 时漏掉,
        # MotionMixin 内的 x_speed_with_safety / stop_x_speed_safety 需要这些状态)
        self._x_safety_lock = threading.Lock()
        self._x_safety_stop_event: Optional[threading.Event] = None
        self._x_safety_thread: Optional[threading.Thread] = None
        self._x_safety_start_x_mm: Optional[float] = None
        self._x_safety_velocity_ms: float = 0.0

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
        here = os.path.dirname(os.path.abspath(__file__))
        # main/arm/api/__init__.py -> main/arm/arm_origin.yaml (向上 1 层)
        return os.path.join(here, "..", "arm_origin.yaml")

    def _load_origin_or_default(self) -> ArmOrigin:
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
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return ArmOrigin(
            y_origin_m=float(data.get("y_origin_m", 0.0)),
            x_origin_m=float(data.get("x_origin_m", 0.0)),
            x_wall=str(data.get("x_wall", "left")),
            soft_y_max_m=float(data.get("soft_y_max_m", 0.20)),
            step_loss_y_mm=float(data.get("step_loss_y_mm", 2.0)),
            step_loss_x_mm=float(data.get("step_loss_x_mm", 5.0)),
            nozzle_offset_x_norm=float(data.get("nozzle_offset_x_norm", 0.0)),
            nozzle_offset_y_norm=float(data.get("nozzle_offset_y_norm", 0.0)),
            nozzle_offset_map={
                str(k): (float(v[0]), float(v[1]))
                for k, v in (data.get("nozzle_offset_map") or {}).items()
            },
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
                    "step_loss_y_mm": origin.step_loss_y_mm,
                    "step_loss_x_mm": origin.step_loss_x_mm,
                    "nozzle_offset_x_norm": origin.nozzle_offset_x_norm,
                    "nozzle_offset_y_norm": origin.nozzle_offset_y_norm,
                    "nozzle_offset_map": {
                        k: [v[0], v[1]] for k, v in origin.nozzle_offset_map.items()
                    },
                    "calibrated_at": origin.calibrated_at,
                },
                f, allow_unicode=True, sort_keys=False,
            )

    # ---- 底层便捷调用 ----

    def _call_arm(self, name: str, timeout: float = 20.0, *args,
                  sync: bool = True, **kwargs) -> dict:
        # 2026-08-07 0-copy: 任何 arm action 后自动 invalidate 短 TTL 缓存,
        # 避免业务层下一行 _read_*_realtime 读到动作发起前的状态。
        self.invalidate_arm_state_cache()
        return self.http.execute_arm_action(
            name, *args, timeout=timeout, sync=sync, **kwargs
        )

    def _call_car(self, name: str, timeout: float = 20.0, *args,
                  sync: bool = False, **kwargs) -> dict:
        # 2026-08-07 0-copy: 同 _call_arm,加自动 invalidate。
        self.invalidate_arm_state_cache()
        return self.http.execute_car_action(
            name, *args, timeout=timeout, sync=sync, **kwargs
        )
