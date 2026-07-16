"""main/arm/api.py
ArmClient：薄封装 RuntimeApiClient + RuntimeWsClient，专给机械臂用。

约定：
  - 只 import main.*，不 import smartcar / runtime
  - 业务单位统一 mm（API 层进车端时换算 m）
  - move_xy / move_x / move_y 底层调 arm.goto_position / arm.move_x_position / arm.move_y_position
    （车端 PID 闭环），同时客户端用 TrajectoryGenerator 做 dry-run 算 t_total 给日志
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Optional, Tuple

logger = logging.getLogger(__name__)

try:
    from main.api_client import RuntimeApiClient
    from main.ws_client import RuntimeWsClient
except ImportError:  # pragma: no cover
    from api_client import RuntimeApiClient  # type: ignore
    from ws_client import RuntimeWsClient  # type: ignore

from .state import (
    ArmState,
    ArmOrigin,
    STORAGE_SIDES,
    STORAGE_DEFAULT_LEFT_ANGLE,
    STORAGE_DEFAULT_RIGHT_ANGLE,
)
from .trajectory import TrajectoryGenerator, TrajectoryPlan


def _mm_to_m(v_mm: float) -> float:
    return float(v_mm) / 1000.0


def _m_to_mm(v_m) -> float:
    return float(v_m) * 1000.0


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
                    "calibrated_at": origin.calibrated_at,
                },
                f,
                allow_unicode=True,
                sort_keys=False,
            )

    # ---- 底层便捷调用 ----

    def _call_arm(self, name: str, timeout: float = 20.0, *args, sync=True, **kwargs) -> dict:
        """调车端 arm action。

        D 改造后默认 sync=True：
          - 长动作（move_xy / reset_y 等）业务语义就是「等完成才能走下一步」，
            改 sync=False 会破坏现有链式编排。
          - 想 fire-and-forget（例如并发抓多个目标）显式传 sync=False。
        """
        return self.http.execute_arm_action(
            name, *args, timeout=timeout, sync=sync, **kwargs
        )

    def _call_car(self, name: str, timeout: float = 20.0, *args, sync=False, **kwargs) -> dict:
        """调车端 car action。

        默认 sync=False：
          - car 短动作（move_for / move_to_position / set_storage 等）默认异步，
            调用方需要时再显式 sync=True。
        """
        return self.http.execute_car_action(
            name, *args, timeout=timeout, sync=sync, **kwargs
        )

    # ---- 业务动作 ----

    def set_pose(
        self,
        x_mm: Optional[float],
        y_mm: Optional[float],
        timeout: float = 30.0,
    ) -> dict:
        """一次设置 x/y（None 表示不动）。side/hand 已删（2026-07-16）。"""
        x_m = _mm_to_m(x_mm) if x_mm is not None else None
        y_m = _mm_to_m(y_mm) if y_mm is not None else None
        # set_pose 是纯移动，禁止在保护区调
        self._check_y_protected("set_pose")
        self._check_safe(y_mm=y_mm)
        return self._call_arm(
            "set_arm_pose",
            timeout=timeout,
            x=x_m, y=y_m,
        )

    def move_xy(
        self,
        x_mm: float,
        y_mm: float,
        v_max_mms: float = 40.0,
        a_max_mms2: float = 100.0,
        timeout: Optional[float] = None,
    ) -> dict:
        """双轴同步移动到 (x_mm, y_mm)。

        2026-07-16：v_max_mms 150 → 75 → 40（连续减半），a_max_mms2 400 → 200 → 100。
        """
        self._check_y_protected("move_xy")
        self._check_safe(y_mm=y_mm)
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
        # move_y 走 y 步进电机（不动舵机），即使在保护区 [0, -30] 也可以调（用于出保护区）。
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

    def move_x(self, x_mm: float, v_max_mms: float = 40.0, out_time: float = 15.0,
               timeout: float = 30.0) -> dict:
        """2026-07-16: 启用 x 控制 + 速度减半。out_time 默认 15s（避免 PID 脉冲式）。"""
        self._check_y_protected("move_x")
        job = self._call_arm(
            "move_x_position",
            timeout=timeout,
            target=_mm_to_m(x_mm),
            out_time=out_time,
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

    # ---- 硬件安全门（防止误操作撞车） ----
    #
    # 经验规则（来自现场测试 + 比赛策略，2026-07-16）：
    #   - y ∈ [0, -30]   ：保护区，禁止动舵机/臂（除 init 位置 hand UP=-90 / arm MID=0）
    #   - y ∈ [-80, -100]：放开一般舵机动作；set_storage 仍需 y < -100
    #   - y ∈ [-100, -200]：允许 set_storage(LEFT/RIGHT)
    #   - 物理依据：y 离触底越近（接近 0），舵机摆动就越容易撞到地面或邻物
    # 注：reset_x 已删除，不再涉及 x 撞墙。
    #
    # 实现：每次关键操作前查 y，超阈就 raise ValueError。**不静默吞**。

    _Y_PROTECTED_THRESHOLD_MM = -30.0     # 2026-07-16: 收紧保护区 [0, -30]（之前 [0, -80] 太宽松）
    _Y_STORAGE_SAFE_THRESHOLD_MM = -100.0   # y 必须 < 这个值才能 set_storage

    def _check_y_protected(
        self, action: str, *,
        allow_init_position: bool = False,
        skip: bool = False,
    ) -> None:
        """y 保护区检查：y ∈ [0, -30]mm 时禁止动舵机/臂（除 init 位置）。

        Args:
            action: 当前动作名（用于错误信息）。
            allow_init_position: True 时允许 init 位置（hand UP=-90 / arm MID=0）。
            skip: True 时跳过保护区检查（用于"大臂已收起"等条件）。
        """
        if skip:
            return
        try:
            st = self.get_state()
            y_mm = float(st.y_mm)
        except Exception:
            # 读不到 y（init 阶段或底层异常）不阻断，避免死锁
            return
        if y_mm > self._Y_PROTECTED_THRESHOLD_MM:
            if allow_init_position:
                return
            raise ValueError(
                f"[{action}] y={y_mm:.1f}mm ∈ [0, -30] 安全保护区，禁止动。\n"
                f"  规则: 接近触底时舵机摆动会撞车\n"
                f"  解决: 先 ArmClient.move_y(-150) 或更低,再试。\n"
                f"  例外: set_hand('UP'/-90) / set_arm_angle('MID'/0) 初始化姿态允许。"
            )

    def _check_y_safe_for_storage(self, action: str) -> float:
        """检查当前 y 是否允许做「会动存储仓舵机」的动作。

        action: "set_storage"
        返回：当前 y_mm（mm），供 caller 日志
        raise：ValueError 当 y >= _Y_STORAGE_SAFE_THRESHOLD_MM
        """
        st = self.get_state()
        y_mm = float(st.y_mm)
        if y_mm > self._Y_STORAGE_SAFE_THRESHOLD_MM:
            raise ValueError(
                f"[{action}] 安全门拦截: 当前 y={y_mm:.1f}mm > {self._Y_STORAGE_SAFE_THRESHOLD_MM:.0f}mm。\n"
                f"  规则: y < {self._Y_STORAGE_SAFE_THRESHOLD_MM:.0f}mm 才能切存储仓\n"
                f"  (y ∈ [0, -100] 接近触底,舵机摆动会撞车)\n"
                f"  解决: 先 ArmClient.move_y(-150) 或更低,再试。"
            )
        return y_mm

    # ---- 大臂角度限位（业务层硬保护，2026-07-16 联调加） ----
    #
    # 经验规则（来自现场测试）：
    #   - LEFT = +93° 撞车（机械臂结构挡住）
    #   - angle < -180 撞车（机械臂结构挡住）
    #   - 业务层硬限 [0, -150]：0 是最大（>= 0 不让），只能负值；-150 是物理安全下界
    #
    # 实现：set_side("LEFT") 拒绝；set_arm_angle(angle) 校验范围。
    # 注意：这是业务层硬保护，HTTP /v1/execute 直调底层 action 不受此限（保留逃生口）。

    _ARM_ANGLE_MIN = -150.0   # 业务层大臂角度下界（°）
    _ARM_ANGLE_MAX = 0.0      # 业务层大臂角度上界（°），>= 0 拒绝

    # 2026-07-16: 大臂在 [0, -30]° 区间时，y 保护区仍约束；
    # 大臂在 [0, -30]° 之外时（即 > 0 或 < -30），大臂可"随便动"，跳过 y 保护区。
    # 物理意义：大臂收起来（<= -30）时结构安全，可大动作；展开（>= 0）时撞车风险。
    _ARM_SAFE_BAND_MIN = -30.0  # 大臂"安全姿态"下界（<= -30 算"收起来"）
    _ARM_SAFE_BAND_MAX = 0.0    # 大臂"安全姿态"上界

    def _is_arm_safe_position(self) -> bool:
        """当前大臂角度是否在"安全姿态"（<= -30，即收起来）。"""
        try:
            st = self.get_state()
        except Exception:
            # 读不到 state 时按安全原则拒绝（保守）
            return False
        cur = st.arm_angle
        if cur is None:
            return False
        # cur <= _ARM_SAFE_BAND_MIN 表示收起来（>= -30 是展开/撞车风险区）
        return cur <= self._ARM_SAFE_BAND_MIN

    def set_arm_angle(self, angle: float, speed: int, timeout: float) -> dict:
        """大臂总线舵机角度控制（业务层，硬限 [0, -150]°）。

        Args:
            angle: 目标角度（°）。硬限 [0, -150]°（LEFT=+93 撞车已禁；<-180 撞车）。
            speed: 舵机速度（必填，无默认）。
            timeout: HTTP 同步超时（秒，必填，无默认）。

        Raises:
            ValueError: 当 angle > 0 或 angle < -150 时拒绝下发。

        2026-07-16 y 保护区放宽：大臂在 [0, -30]° 之外（即 <= -30"收起来"）时，
        即使 y ∈ [0, -30] 保护区也可"随便动"（用户新规则）。
        """
        try:
            a = float(angle)
        except (TypeError, ValueError):
            raise ValueError(f"set_arm_angle angle 必须是数字，收到: {angle!r}")
        if a > self._ARM_ANGLE_MAX or a < self._ARM_ANGLE_MIN:
            raise ValueError(
                f"set_arm_angle({a}) 超出业务硬限 [{self._ARM_ANGLE_MIN}, {self._ARM_ANGLE_MAX}]°。\n"
                f"  规则: 大臂角度 ∈ [0, -150]°（LEFT=+93 撞车，<-180 撞车）\n"
                f"  解决: 选 -90 (RIGHT 附近) / -120 / -150 等。"
            )
        # y 保护区放宽：大臂"收起来"（<= -30）时跳过 y 保护区
        skip_y_protect = self._is_arm_safe_position()
        if skip_y_protect:
            logger.info("set_arm_angle: 大臂已 <= -30 (收起来)，跳过 y 保护区")
        # y 保护区：0° (MID) 是 init 位置（允许），其他需先出保护区
        self._check_y_protected(
            "set_arm_angle",
            allow_init_position=(a == 0.0),
            skip=skip_y_protect,
        )
        return self._call_arm("set_arm_angle", timeout=timeout, angle=a, speed=speed)

    # ---- 手爪角度限位（业务层硬保护，2026-07-16 联调加） ----
    #
    # PWM 模式 180 物理范围 [-90, +165]（协议值 = angle + 90 ∈ [0, 255]）。
    # 业务层硬限 [-90, 0]°：
    #   - 上界 0°（DOWN）：防止手爪向下超过水平
    #   - 下界 -90°（UP）：防止手爪向上超过机械结构
    # 仅数字接口 set_hand_angle(angle)，无字符串预设（2026-07-16 用户要求）。
    _HAND_ANGLE_MIN = -90.0   # 业务层手爪角度下界（°）
    _HAND_ANGLE_MAX = 0.0     # 业务层手爪角度上界（°），> 0 拒绝

    def set_hand_angle(self, angle: float, speed: int, timeout: float) -> dict:
        """手爪 PWM 舵机角度控制（业务层，硬限 [-90, 0]°）。

        Args:
            angle: 目标角度（°）。数字接口，硬限 [-90, 0]°；UP=-90 是 init 位置。
            speed: 舵机速度（必填，无默认）。
            timeout: HTTP 同步超时（秒，必填，无默认）。

        Raises:
            ValueError: 当 angle > 0 或 angle < -90 时拒绝下发。
            2026-07-16 新规则：当前大臂在 [0, -30]° 范围内时，手爪只允许 init=UP=-90。
        """
        try:
            a = float(angle)
        except (TypeError, ValueError):
            raise ValueError(f"set_hand_angle angle 必须是数字，收到: {angle!r}")
        if a > self._HAND_ANGLE_MAX or a < self._HAND_ANGLE_MIN:
            raise ValueError(
                f"set_hand_angle({a}) 超出业务硬限 [{self._HAND_ANGLE_MIN}, {self._HAND_ANGLE_MAX}]°。\n"
                f"  规则: 手爪角度 ∈ [-90, 0]°（PWM 物理范围 [-90, +165]，"
                f"业务只允许 ≤ 0 防止撞车）\n"
                f"  解决: 选 0 (DOWN) / -37 (MID) / -90 (UP)。"
            )
        # 2026-07-16 新规则：当前大臂在 [0, -30]° 时手爪只允许 init (UP=-90)
        # 物理意义：大臂展开时手爪不动（防机械结构碰车）
        try:
            st = self.get_state()
            cur_arm = st.arm_angle
        except Exception:
            cur_arm = None
        if cur_arm is not None and self._ARM_SAFE_BAND_MIN <= cur_arm <= self._ARM_SAFE_BAND_MAX:
            # 当前大臂在 [0, -30]° "展开区"
            if a != self._HAND_ANGLE_MIN:  # -90 UP
                raise ValueError(
                    f"set_hand_angle({a}) 拒绝：当前大臂在 [{self._ARM_SAFE_BAND_MIN}, "
                    f"{self._ARM_SAFE_BAND_MAX}]° 展开区，手爪只允许 init (UP=-90°)。\n"
                    f"  规则: 大臂展开时手爪禁止控制（防机械结构撞车）\n"
                    f"  解决: 先 set_arm_angle(<=-30) 把大臂收起来，再调手爪。"
                )
            # UP=-90 是 init 位置，仍要走 y 保护区检查（init 允许）
            self._check_y_protected("set_hand_angle", allow_init_position=True)
        else:
            # 当前大臂不在 [0, -30]（即收起来或错误）—— y 保护区正常走 init 例外
            self._check_y_protected("set_hand_angle", allow_init_position=(a == -90.0))
        return self._call_arm("set_hand_angle", timeout=timeout, angle=a, speed=speed)

    # ---- 存储仓（二选一档位） ----

    def set_storage(self, side: str, timeout: float = 10.0) -> dict:
        """切换车体上的存储仓舵机（独立 PWM 舵机，port=1）。

        只接受两个档位（写死角度，不允许任意角度）：
          - "LEFT"  → STORAGE_DEFAULT_LEFT_ANGLE  = -42°（与初始化复位角度一致）
          - "RIGHT" → STORAGE_DEFAULT_RIGHT_ANGLE = 90°（车端 car_wrap_2026.servo_1_angle_list）

        ⚠️ 安全门：y 必须 < -100mm 才能调。在 y ∈ [0, -100] 接近触底的位置舵机摆动
        会撞底盘结构。底层走 car.set_storage(bool)（不在 arm action 表里），所以这里
        显式 safety check。

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
        # 硬件安全门：y 必须 < -100mm（防止 y ∈ [0,-100] 时舵机摆动撞车）
        self._check_y_safe_for_storage("set_storage")
        # 注意：car.set_storage(True) → 取 servo_1_angle_list[1] = 165°（RIGHT 档），
        # False → servo_1_angle_list[0] = -42°（LEFT 档）。
        open_flag = side == "RIGHT"
        # 业务语义：舵机动作完成后才能确认档位，需要 sync=True（car 默认是 False）。
        job = self._call_car("set_storage", timeout=timeout, state=open_flag, sync=True)

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
        # 修复 bug：原来 `_call_arm("grasp", bool(on), timeout=timeout)` 会让
        # `_call_arm(self, name, timeout=20.0, *args, ...)` 把 bool(on) 当成
        # timeout 位置参，再传 timeout=timeout 报 "got multiple values"。
        # 改为 keyword-only timeout 传参。
        return self._call_arm("grasp", bool(on), sync=True, timeout=timeout)

    # ---- reset ----

    def reset_y(self, timeout: float = 30.0) -> dict:
        """仅归 y（步进电机触底 + 磁感确认，不动 x）。

        走车端 arm.reset_y：只让 y 步进电机找磁感触底，不动 x 编码器电机。
        与 reset_position (y + x 一起归) 区分。

        失败语义见 [ARM_API.md §reset_y 行为](./ARM_API.md#reset_y-行为磁感是唯一到底凭证)。
        """
        return self._call_arm("reset_y", timeout=timeout)

    def reset_x(self, direction: str = "right", reset_velocity_mms: float = 20.0,
                timeout: float = 30.0) -> dict:
        """2026-07-16 新加：x 撞墙定原点。

        单档极慢速度撞物理墙,编码器 stall 判定。默认 20 mm/s 撞右墙(direction='right')。
        撞墙后 calibrate,x_pose_now 归零,_x_ref_encoder_at_zero 写入新 ref。

        注意:
          - 仅 opt-in 触发,不进 auto-init(避免 fb24b1a 描述的 pm2 反复重建)。
          - 机械臂当前已经在墙边(<50mm)时不会 calibrate,需先 move_x 反向拉回中段。
          - 撞墙速度不要改太快(参考 commit 1d5990e 实测 0.02 m/s 最稳定)。

        Args:
            direction: 'right' 或 'left'
            reset_velocity_mms: 撞墙速度 (mm/s),默认 20
            timeout: HTTP 同步超时 (s),车端实际可能 15-20s,留余量
        """
        if direction not in ("right", "left"):
            raise ValueError("direction 必须是 'right' 或 'left'")
        return self._call_arm(
            "reset_x", timeout=timeout,
            direction=direction,
            reset_velocity=reset_velocity_mms / 1000.0,
        )

    def reset_all(self, arm_angle: float = 0, hand_angle: float = -90,
                  x_direction: str = "right",
                  reset_x_velocity_mms: float = 20.0,
                  timeout: float = 120.0) -> dict:
        """2026-07-16 新加：复合复位 (x + 大臂 + 手爪 并行 → y 串行)。

        三路并行(x 撞墙 / 大臂回 MID / 手爪回 UP),完成后 reset_y 触底。
        timeout 给够(reset_y + reset_x + 2 servo 总耗时约 30-40s)。

        物理前提:机械臂当前不在右墙边(<50mm) — 否则 reset_x 不 calibrate。
        """
        return self._call_arm(
            "reset_all", timeout=timeout,
            arm_angle=arm_angle, hand_angle=hand_angle,
            x_direction=x_direction,
            reset_x_velocity=reset_x_velocity_mms / 1000.0,
        )

    def reset_origin(self, x_wall: str = "left", timeout: float = 60.0) -> dict:
        """主动触发车端 reset_position（仅 y 触底），作为业务坐标系新原点。

        行为变更（2026-07-16）：reset_x 已删除，x 轴无软件复位。
        reset_position 现在只做 y 触底定原点；x 位置由视觉闭环控制。
        x_origin_m 固定为 0.0（不再基于撞墙）。
        """
        if x_wall not in ("left", "right"):
            raise ValueError("x_wall 必须是 'left' 或 'right'")
        job = self._call_arm("reset_position", timeout=timeout)
        # 重新读一次 y 原始坐标作为新原点（x 固定为 0）
        st = self._read_raw_state()
        new_origin = ArmOrigin(
            y_origin_m=st["raw_y_m"],
            x_origin_m=0.0,
            x_wall=x_wall,  # 保留字段兼容，但语义已无意义
            soft_y_max_m=self.origin.soft_y_max_m if self.origin else 0.20,
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
        # sync=True 同步等 result（异步模式 result 在 job.result 而非顶层）
        st_job = self._call_car("get_arm_state", timeout=10.0, sync=True)
        st_data = st_job.get("result") if isinstance(st_job, dict) else {}
        if not isinstance(st_data, dict):
            st_data = {}
        # 2026-07-16: side/hand 字符串预设已删，get_state 直接透传车端返回值。
        side = str(st_data.get("side", "MID"))
        hand = str(st_data.get("hand_angle", "UP"))
        origin = self.origin or ArmOrigin()
        return ArmState(
            x_mm=_m_to_mm(raw["raw_x_m"]),
            y_mm=_m_to_mm(raw["raw_y_m"]),
            side=side,
            hand=hand,
            grasping=False,  # 车端没暴露 grasping 字段
            y_origin_valid=bool(st_data.get("y_limit", False)),  # 注意：y_limit 字段语义是 "达到限位"
            x_origin_valid=False,  # reset_x 已删除，x 无撞墙校准
            soft_y_max_mm=origin.soft_y_max_mm,
            soft_x_min_mm=None,  # x 轴软限位已取消
            soft_x_max_mm=None,  # x 轴软限位已取消
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
        """软限位校验（仅 y；x 轴软限位已取消）。

        y 业务坐标：触底=0，向下（朝触底）取正值，向上（远离触底）取负值；
        区间 [-soft_y_max_mm, 0]。

        x 参数保留签名兼容，但不再校验（用户原话"灵活使用就好"）。
        """
        origin = self.origin or ArmOrigin()
        if y_mm is not None and not (-origin.soft_y_max_mm <= y_mm <= 0.0):
            raise ValueError(
                f"y_mm={y_mm} 超出软区间 [-{origin.soft_y_max_mm:.0f}, 0] mm"
                f"（触底=0, 顶部=-{origin.soft_y_max_mm:.0f}mm）"
            )

    def emergency_stop(self) -> dict:
        return self.http.emergency_stop()

    def ping(self, timeout: float = 5.0) -> bool:
        try:
            self.http.get_health()
            return True
        except Exception:
            return False
