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

from .state import (
    ArmState,
    ArmOrigin,
    SIDES,
    HANDS,
    STORAGE_SIDES,
    STORAGE_DEFAULT_LEFT_ANGLE,
    STORAGE_DEFAULT_RIGHT_ANGLE,
)
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


def _normalize_storage_side(side: Optional[str]) -> Optional[str]:
    """存储仓二选一档位归一化。和机械臂 SIDES 区别：只有 LEFT/RIGHT 两档。"""
    if side is None:
        return None
    s = side.upper()
    if s not in STORAGE_SIDES:
        raise ValueError(f"storage side 必须是 {STORAGE_SIDES} 之一，收到: {side!r}")
    return s


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
            soft_y_max_m=float(data.get("soft_y_max_m", 0.20)),
            soft_x_min_m=float(data.get("soft_x_min_m", -0.32)),
            soft_x_max_m=float(data.get("soft_x_max_m", 0.32)),
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
        # 业务坐标语义：y_mm=0 在磁感应触底，向下（朝触底）取正值，向上取负值；上限 = -soft_y_max_mm。
        self._check_safe(y_mm=y_mm)
        job = self._call_arm(
            "move_y_position",
            timeout=timeout,
            target=_mm_to_m(y_mm),
        )
        # 磁感应触底兜底：目标接近触底 (y≈0) 但车端 y_limit 仍为 False 时 warn。
        # y_origin_valid 在 get_state 里映射自车端 y_limit（API.md: get_arm_state 返回 y_limit）。
        origin = self.origin or ArmOrigin()
        try:
            state = self.get_state()
            near_bottom = abs(y_mm) <= 0.1 * origin.soft_y_max_mm
            if near_bottom and not state.y_origin_valid:
                print(
                    f"[move_y] 警告: 目标 y={y_mm:.1f}mm 接近触底(0mm)，"
                    f"但车端 y_limit 仍为 False（磁感应未触发）。",
                    flush=True,
                )
            # 丢步补偿：步进电机失步后实际位置 ≠ 目标，超阈值时 warn。
            # 阈值 2mm（≈步距），可在 ArmOrigin 里覆盖。
            self._check_step_loss("y", target_mm=y_mm, actual_mm=state.y_mm,
                                  threshold_mm=origin.step_loss_y_mm)
        except Exception as e:
            print(f"[move_y] 状态校验读取失败: {e}", flush=True)
        return job

    def move_x(self, x_mm: float, v_max_mms: float = 150.0, timeout: float = 20.0) -> dict:
        # 业务坐标语义：x_mm=0 在撞墙参考点（reset_x 堵转后），远离墙为正；区间 [soft_x_min, soft_x_max]。
        # motor_280 是编码器闭环，正常不会丢步，但仍做一次回校以防打滑/卡阻。
        self._check_safe(x_mm=x_mm)
        job = self._call_arm(
            "move_x_position",
            timeout=timeout,
            target=_mm_to_m(x_mm),
        )
        origin = self.origin or ArmOrigin()
        try:
            state = self.get_state()
            self._check_step_loss("x", target_mm=x_mm, actual_mm=state.x_mm,
                                  threshold_mm=origin.step_loss_x_mm)
        except Exception as e:
            print(f"[move_x] 状态校验读取失败: {e}", flush=True)
        return job

    # ---- 丢步/位置偏差校验 ----

    @staticmethod
    def _check_step_loss(axis: str, target_mm: float, actual_mm: float,
                         threshold_mm: float) -> None:
        """对比目标 vs 实际，超阈值 warn（不抛错，由调用方决定是否重试）。"""
        try:
            err = abs(float(actual_mm) - float(target_mm))
        except (TypeError, ValueError):
            return
        if err > threshold_mm:
            print(
                f"[move_{axis}] 警告: 目标={target_mm:.1f}mm 实际={actual_mm:.1f}mm "
                f"偏差={err:.1f}mm > {threshold_mm:.1f}mm（步进/电机可能丢步或堵转）",
                flush=True,
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

    # ---- 存储仓（二选一档位） ----

    def set_storage(self, side: str, timeout: float = 10.0) -> dict:
        """切换车体上的存储仓舵机（独立 PWM 舵机，port=1）。

        只接受两个档位：
          - "LEFT"  → 写死角度 STORAGE_DEFAULT_LEFT_ANGLE（-42°，与初始化复位角度一致）
          - "RIGHT" → 写死角度 STORAGE_DEFAULT_RIGHT_ANGLE（165°）

        底层走 car.set_storage(bool)，它在 car_wrap_2026.sensor_init 阶段已构造。
        之所以走 car（而不是 arm）是因为这块舵机不属于机械臂（arm），属于车体外设。

        返回（业务层常用字段）：
            {
              "ok": bool,            # job 是否 succeeded
              "side": "LEFT"/"RIGHT",# 实际生效的档位（车端回传）
              "flag": 0/1,
              "angle": int,
              "state": bool,         # 透传 set_storage 的 bool 参数
              "raw_job": dict,       # 完整 job dict（保留给调试）
            }
        """
        side = _normalize_storage_side(side)
        if side is None:
            raise ValueError(f"set_storage 必须给 {STORAGE_SIDES}")
        # 注意：car.set_storage(True) → 取 servo_1_angle_list[1] = 165°（RIGHT 档），
        # False → servo_1_angle_list[0] = -42°（LEFT 档）。
        open_flag = side == "RIGHT"
        job = self._call_car("set_storage", timeout=timeout, state=open_flag)

        # 把车端 result 解出来（runtime 已 normalize_value 序列化）。
        # 失败 job 这里 result 通常是 None / 错误字符串。
        result = job.get("result") if isinstance(job, dict) else None
        out = {
            "ok": bool(isinstance(job, dict) and job.get("status") == "succeeded"),
            "side": None,
            "flag": None,
            "angle": None,
            "state": open_flag,
            "raw_job": job,
        }
        if isinstance(result, dict):
            r_side = str(result.get("side", "")).upper()
            if r_side in STORAGE_SIDES:
                out["side"] = r_side
            if "flag" in result:
                try:
                    out["flag"] = int(result["flag"])
                except (TypeError, ValueError):
                    pass
            if "angle" in result:
                try:
                    out["angle"] = int(result["angle"])
                except (TypeError, ValueError):
                    pass
        # 兜底：如果车端没回 side，按请求的 side 填
        if out["side"] is None and out["ok"]:
            out["side"] = side
        # 客户端缓存：让 get_storage() 不用再下发舵机
        if out["side"] in STORAGE_SIDES:
            self._storage_side_cache = out["side"]
        return out

    def get_storage(self) -> str:
        """只读：返回当前存储仓档位 "LEFT" / "RIGHT" / "UNKNOWN"。

        纯客户端缓存：每次 set_storage 成功后本地更新；
        **不会让舵机动作**。ArmClient 重建后状态归零，回到 "UNKNOWN"。
        """
        return getattr(self, "_storage_side_cache", "UNKNOWN")

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
            soft_x_min_m=self.origin.soft_x_min_m if self.origin else -0.32,
            soft_x_max_m=self.origin.soft_x_max_m if self.origin else 0.32,
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
        # y 业务坐标：触底=0，向下（朝触底）取正值，向上（远离触底）取负值；区间 [-soft_y_max_mm, 0]。
        if y_mm is not None and not (-origin.soft_y_max_mm <= y_mm <= 0.0):
            raise ValueError(
                f"y_mm={y_mm} 超出软区间 [-{origin.soft_y_max_mm:.0f}, 0] mm"
                f"（触底=0, 顶部=-{origin.soft_y_max_mm:.0f}mm）"
            )
        if x_mm is not None and not (origin.soft_x_min_mm <= x_mm <= origin.soft_x_max_mm):
            raise ValueError(
                f"x_mm={x_mm} 超出软区间 [{origin.soft_x_min_mm}, {origin.soft_x_max_mm}] mm"
            )

    def emergency_stop(self) -> dict:
        return self.http.emergency_stop()

    def ping(self, timeout: float = 5.0) -> bool:
        try:
            self.http.get_health()
            return True
        except Exception:
            return False
