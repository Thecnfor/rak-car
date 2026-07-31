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
import threading
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

# 2026-07-28: 球检测验证基线 + 助手。task4 业务层有 BALL_VERIFIED_*
# (target1 位姿下球检测期望范围, 蓝黄共用), api.py 这里**再导出 + 暴露
# verify_ball()** 让一般 arm 客户端代码也能做"球是不是在期望范围"的检查
# (不直接依赖 task4 常量文件)。`ArmClient.verify_ball()` 方法是常用入口。
try:
    from main.arm.each_task.task4.constants import (  # type: ignore
        BALL_VERIFIED_CX_MIN, BALL_VERIFIED_CX_MAX,
        BALL_VERIFIED_CY_MIN, BALL_VERIFIED_CY_MAX,
        BALL_VERIFIED_W_MIN, BALL_VERIFIED_W_MAX,
        BALL_VERIFIED_H_MIN, BALL_VERIFIED_H_MAX,
        BALL_VERIFIED_AREA_MIN_VERIFY, BALL_VERIFIED_AREA_MAX_VERIFY,
        BALL_VERIFIED_SCORE_MIN_VERIFY,
        BALL_VERIFIED_ASPECT_MIN, BALL_VERIFIED_ASPECT_MAX,
    )
except ImportError:  # pragma: no cover — 业务层未就绪时给空缺省值, 不阻断 import
    # 2026-07-30: fallback 同步 constants.py 第 9 次 (新最佳) 加测后的 UNION 区间。
    # 2026-07-29 第 8 次 (cx=0.120, aspect=0.923) → 2026-07-30 第 9 次 (cx=0.026, aspect=1.057) 加测,
    # 新球 aspect 跨 1.0 (横宽>纵高), 放宽 H_MIN/ASPECT_MAX。
    # 业务层导入成功时**不会**用这里, 只在 task4 constants.py 缺失时兜底。
    BALL_VERIFIED_CX_MIN = 0.02
    BALL_VERIFIED_CX_MAX = 0.20
    BALL_VERIFIED_CY_MIN = -0.78
    BALL_VERIFIED_CY_MAX = -0.55
    BALL_VERIFIED_W_MIN = 0.35
    BALL_VERIFIED_W_MAX = 0.56
    BALL_VERIFIED_H_MIN = 0.48
    BALL_VERIFIED_H_MAX = 0.65
    BALL_VERIFIED_AREA_MIN_VERIFY = 0.20
    BALL_VERIFIED_AREA_MAX_VERIFY = 0.35
    BALL_VERIFIED_SCORE_MIN_VERIFY = 0.80
    BALL_VERIFIED_ASPECT_MIN = 0.55
    BALL_VERIFIED_ASPECT_MAX = 1.10

from .state import (
    ArmState,
    ArmOrigin,
    STORAGE_SIDES,
    STORAGE_DEFAULT_LEFT_ANGLE,
    STORAGE_DEFAULT_RIGHT_ANGLE,
)
from .trajectory import TrajectoryGenerator, TrajectoryPlan

# 2026-07-31 视觉伺服：延迟到方法体内 import 避免循环依赖
def _import_vision():
    from .vision import ArmVisionClient
    return ArmVisionClient


class ArmSafetyError(ValueError):
    """机械臂安全门拦截时抛的异常。

    业务层入口（move_x / move_y / set_arm_angle / set_hand_angle）目前的保护区检查
    仍统一抛 ``ValueError``；本类作为 ``ValueError`` 的子类提供显式语义，
    方便未来代码按"是否安全门拦截"细分 ``except``（同时不破坏现有
    ``except ValueError`` 的捕获路径）。``__init__.py`` 已对外导出。
    """


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


def pre_init_close_storage(
    http: RuntimeApiClient,
    timeout: float = 10.0,
    closed_angle_deg: float = 98.0,
) -> dict:
    """初始化前预操作：把储存仓舵机打到关闭位（默认 98°）。

    user 2026-07-18 要求：任何 init 入口前都应先关仓，避免开仓状态
    干扰磁感找底。不动 y 轴（忽略 ``test_storage_close.py`` 里的
    y=-150 抬升，那是测试脚本的临时 workaround）。

    实现：走 ``/v1/execute target=car name=set_storage_angle`` 直传
    raw 协议值，``sync=True`` 阻塞轮询到 ``succeeded``；预操作失败
    由调用方 catch（不阻塞 init 主流程，本函数本身只负责下发）。

    注意：set_storage_angle 是 **CAR action**（不是 ARM action，见
    ``runtime/core/actions.py:12``），runtime 收到 ``target=arm`` 会 400。
    跟 ``ArmClient.set_storage_angle`` 走 ``_call_car`` 同款。

    参数:
        http: ``RuntimeApiClient`` 实例。
        timeout: job 超时（秒）。
        closed_angle_deg: 关闭位角度（°），现场标定值，默认 98°。

    返回:
        ``/v1/execute`` 同步返回的 job dict（含 ``status`` / ``result`` / ``error``）。
    """
    return http.execute_car_action(
        "set_storage_angle",
        angle=float(closed_angle_deg),
        timeout=timeout,
        sync=True,
    )


def verify_ball(
    ball: dict,
    *,
    cx_min: float = BALL_VERIFIED_CX_MIN,
    cx_max: float = BALL_VERIFIED_CX_MAX,
    cy_min: float = BALL_VERIFIED_CY_MIN,
    cy_max: float = BALL_VERIFIED_CY_MAX,
    w_min: float = BALL_VERIFIED_W_MIN,
    w_max: float = BALL_VERIFIED_W_MAX,
    h_min: float = BALL_VERIFIED_H_MIN,
    h_max: float = BALL_VERIFIED_H_MAX,
    area_min: float = BALL_VERIFIED_AREA_MIN_VERIFY,
    area_max: float = BALL_VERIFIED_AREA_MAX_VERIFY,
    score_min: float = BALL_VERIFIED_SCORE_MIN_VERIFY,
    aspect_min: float = BALL_VERIFIED_ASPECT_MIN,
    aspect_max: float = BALL_VERIFIED_ASPECT_MAX,
) -> bool:
    """验证 ball dict 是否落在 BALL_VERIFIED_* 期望范围 (target1 位姿下)。

    2026-07-28 加进 api.py: 业务层 (target2 / test_* / step_*) 之外的其他
    代码也能用 arm 客户端做球验证, 不必 import task4.constants。

    默认值取自 `BALL_VERIFIED_*` (target1.py 位姿下 5 次实测基线, 蓝黄共用)。
    全部 7 项**同时**通过才返 True; 任一不通过 → False (静默, 不抛)。

    字段缺失 / 类型错 → 不通过。

    验证项:
      - cx_norm  ∈ [cx_min, cx_max]
      - cy_norm  ∈ [cy_min, cy_max]
      - w_norm   ∈ [w_min, w_max]
      - h_norm   ∈ [h_min, h_max]
      - area     = w*h ∈ [area_min, area_max]
      - score    ≥ score_min
      - aspect   = w/h ∈ [aspect_min, aspect_max]

    Args:
        ball: dict, 期望含 cx_norm / cy_norm / w_norm / h_norm / score 字段。
        其他参数: 阈值覆盖, 默认值 = BALL_VERIFIED_*, 调用方可临时调。

    Returns:
        True = 在范围内 (球检测合理); False = 越界 (噪声框或位姿偏移)。
    """
    try:
        cx = float(ball.get("cx_norm", 0.0))
        cy = float(ball.get("cy_norm", 0.0))
        w = float(ball.get("w_norm", 0.0))
        h = float(ball.get("h_norm", 0.0))
        score = float(ball.get("score", 0.0))
    except (TypeError, ValueError):
        return False
    if w <= 0 or h <= 0:
        return False
    area = w * h
    aspect = w / h
    if not (cx_min <= cx <= cx_max):
        return False
    if not (cy_min <= cy <= cy_max):
        return False
    if not (w_min <= w <= w_max):
        return False
    if not (h_min <= h <= h_max):
        return False
    if not (area_min <= area <= area_max):
        return False
    if score < score_min:
        return False
    if not (aspect_min <= aspect <= aspect_max):
        return False
    return True


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
        # ---- x_speed safety watchdog 状态（per ARM_API §10 + memory [[x-speed-safety-watchdog]]）----
        # latest-wins: 多次调用 x_speed_with_safety 会取消前一个 watchdog。
        # 状态必须用 self._x_safety_lock 串行访问。
        self._x_safety_lock = threading.Lock()
        self._x_safety_thread: Optional[threading.Thread] = None
        self._x_safety_stop_event: Optional[threading.Event] = None
        self._x_safety_start_x_mm: Optional[float] = None
        self._x_safety_velocity_ms: float = 0.0
        # ---- realtime 读取失败原因（见 _read_arm_state_realtime / last_realtime_error）----
        self._last_realtime_error: Optional[str] = None
        # 2026-07-31：vision 懒构造（合并 main 分支的视觉伺服入口）
        self._vision: Optional[object] = None

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
          - sync=False 让 HTTP 调用方立即返回,适合「自己开线程监控」
            或「后台持续动作」。**注意**:runtime 的 arm_queue 单 worker,
            多个 sync=False 调用仍按提交顺序串行执行,**不会真并发**。
            真并发见 composite_pick / composite_release / composite_go_home。
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
        """2026-07-16: 启用 x 控制 + 速度减半。out_time 默认 15s（避免 PID 脉冲式）。

        2026-07-28 修：v_max_mms 之前**接收了但没透传**给 _call_arm，target1/2 传
        v_max_mms=15 实际跑默认 40。修法：v_max_mms 走 kwarg 转发给底层
        move_x_position（arm_base.py:456 接受 v_max_mms kwarg）。
        """
        self._check_y_protected("move_x")
        job = self._call_arm(
            "move_x_position",
            timeout=timeout,
            target=_mm_to_m(x_mm),
            out_time=out_time,
            v_max_mms=v_max_mms,   # 2026-07-28: 透传（之前被吞，target1.x 限速不生效）
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
    #   - y ∈ [-80, -200]：放开一般舵机动作
    #   - 物理依据：y 离触底越近（接近 0），舵机摆动就越容易撞到地面或邻物
    # 注：reset_x 已删除，不再涉及 x 撞墙。
    # 注：set_storage / set_storage_angle **无软限制**（用户原话："这个存储仓舵机不要任何软限制"），
    #     直传 car.set_storage / car.set_storage_angle，撞车风险由 caller 承担。
    #
    # 实现：每次关键操作前查 y，超阈就 raise ValueError。**不静默吞**。

    _Y_PROTECTED_THRESHOLD_MM = -30.0     # 2026-07-16: 收紧保护区 [0, -30]（之前 [0, -80] 太宽松）

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
        except Exception as exc:
            # 2026-07-31 PR#13: 改 fail-closed。原来这里"读不到就放行"
            # 在硬件失控时会放过撞车指令。改为保守拒绝：
            #   - log warning 留现场
            #   - raise ValueError 让 caller 显式选择"跳过保护再下发"
            logger.warning(
                "_check_y_protected: 读不到 state,保守拒绝 (action=%s, err=%s)",
                action, exc,
            )
            raise ValueError(
                f"[{action}] 无法读取 y 状态,保守拒绝。runtime 是否在线?"
            ) from exc
        if y_mm > self._Y_PROTECTED_THRESHOLD_MM:
            if allow_init_position:
                return
            raise ValueError(
                f"[{action}] y={y_mm:.1f}mm ∈ [0, -30] 安全保护区，禁止动。\n"
                f"  规则: 接近触底时舵机摆动会撞车\n"
                f"  解决: 先 ArmClient.move_y(-150) 或更低,再试。\n"
                f"  例外: set_hand('UP'/-90) / set_arm_angle('MID'/0) 初始化姿态允许。"
            )

    # ---- 大臂角度限位（业务层硬保护，2026-07-27 第三次重定义） ----
    #
    # 经验规则（来自现场测试，2026-07-27 联调改）：
    #   - 复位角度 = +90°（展开方向，"初始位"，reset_position 用这个值）
    #   - 最大角度 = -150°（收回方向极限，结构挡住，不能再往下转）
    #   - 业务层硬限 [+90, -150]：上界 +90（复位位），下界 -150（结构极限）
    #   - LEFT=+93° 撞车 / <-150° 撞车 都是经验值，物理安全边界。
    #
    # 历史版本：
    #   - 2026-07-16 初版：硬限 [0, -150]
    #   - 2026-07-27 v2：放宽到 [0, -200]（实测 -180° 不撞车）
    #   - 2026-07-27 v3：用户实测后定义 [+90, -150]（+90 是复位位，-150 是结构极限）
    #
    # 实现：set_side("LEFT") 拒绝；set_arm_angle(angle) 校验范围。
    # 注意：这是业务层硬保护，HTTP /v1/execute 直调底层 action 不受此限（保留逃生口）。

    _ARM_ANGLE_MIN = -150.0   # 业务层大臂角度下界（°），2026-07-27 实测结构极限
    _ARM_ANGLE_MAX = 90.0     # 业务层大臂角度上界（°），2026-07-27 用户实测：+90 是复位位

    # 2026-07-16: 大臂在 [-30, +30]° 区间时，y 保护区仍约束；
    # 大臂在 [-30, +30]° 之外时（<= -30 收起或 >= +30 复位），可"随便动"，跳过 y 保护区。
    # 物理意义：大臂收起来（<= -30）时结构安全，可大动作；展开（>= +30 即接近复位位）也安全。
    _ARM_SAFE_BAND_MIN = -30.0  # 大臂"安全姿态"下界（<= -30 算"收起来"）
    _ARM_SAFE_BAND_MAX = 30.0   # 大臂"安全姿态"上界（>= +30 算"展开到复位附近"）

    def _is_arm_safe_position(self) -> bool:
        """当前大臂角度是否在"安全姿态"（<= -30 收起 或 >= +30 复位附近）。

        大臂在 [-30, +30]° 区间时算"展开状态",需要 y 保护区约束;
        收起来或完全复位都安全,可"随便动"。
        """
        try:
            st = self.get_state()
        except Exception:
            # 读不到 state 时按安全原则拒绝（保守）
            return False
        cur = st.arm_angle
        if cur is None:
            return False
        # cur <= _ARM_SAFE_BAND_MIN 收起 / cur >= _ARM_SAFE_BAND_MAX 复位附近 → 安全
        return cur <= self._ARM_SAFE_BAND_MIN or cur >= self._ARM_SAFE_BAND_MAX

    def set_arm_angle(self, angle: float, speed: int, timeout: float) -> dict:
        """大臂总线舵机角度控制（业务层，硬限 [+90, -150]°，2026-07-27 重定义）。

        Args:
            angle: 目标角度（°）。硬限 [+90, -150]°（LEFT=+93 撞车；<-150 撞车）。
            speed: 舵机速度（必填，无默认）。
            timeout: HTTP 同步超时（秒，必填，无默认）。

        Raises:
            ValueError: 当 angle > +90 或 angle < -150 时拒绝下发。

        2026-07-27：硬限从 [0, -200] 改为 [+90, -150]。+90 是复位角度（reset_position 用),
        -150 是结构极限。
        """
        try:
            a = float(angle)
        except (TypeError, ValueError):
            raise ValueError(f"set_arm_angle angle 必须是数字，收到: {angle!r}")
        if a > self._ARM_ANGLE_MAX or a < self._ARM_ANGLE_MIN:
            raise ValueError(
                f"set_arm_angle({a}) 超出业务硬限 [{self._ARM_ANGLE_MIN}, {self._ARM_ANGLE_MAX}]°。\n"
                f"  规则: 大臂角度 ∈ [+90, -150]°（+90 是复位位，-150 是结构极限）\n"
                f"  解决: 选 +90 (复位) / 0 (MID) / -90 / -150 等。"
            )
        # y 保护区放宽：大臂"收起来"（<= -30）或"复位位"（>= +30）时跳过 y 保护区
        skip_y_protect = self._is_arm_safe_position()
        if skip_y_protect:
            logger.info("set_arm_angle: 大臂已 <= -30 (收起) 或 >= +30 (复位),跳过 y 保护区")
        # y 保护区：+90° (复位位) 或 0° (MID) 是 init 位置（允许），其他需先出保护区
        self._check_y_protected(
            "set_arm_angle",
            allow_init_position=(a == 90.0 or a == 0.0),
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

        ⚠️ **无软限制**（2026-07-17 用户原话："这个存储仓舵机不要任何软限制"）。
        任意 y 位置都会直传车端舵机，撞车风险由 caller 自负。

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
        任意角度（set_storage_angle）会把缓存清成 "UNKNOWN"（不再是两档之一）。
        """
        return getattr(self, "_storage_side_cache", "UNKNOWN")

    def set_storage_angle(self, angle: float, speed: int = 100,
                          timeout: float = 10.0) -> dict:
        """把存储仓舵机转到任意角度（绕开 LEFT/RIGHT 两档写死）。

        与 set_storage(side) 的区别：后者只接受两档写死角度，这里接受任意角度，
        供 main 层自由调试 / 标定。角度语义与底层 ServoPwm(mode=180) 一致：
        协议值 = int(angle / 180 * 180 + 90) = angle + 90，落在 [0, 180] 才合法
        （即 angle ∈ [-90, 90]），超出由舵机自然回弹——这是已知物理 trade-off，
        与 set_storage 的 RIGHT=165° 同理，runtime/底层都不 clamp。

        ⚠️ **无软限制**（2026-07-17 用户原话："这个存储仓舵机不要任何软限制"）。
        任意 y 位置、任意 angle 都会直传车端舵机，撞车风险 / 协议值超界由 caller 自负。
        角度语义与底层 ServoPwm(mode=180) 一致：协议值 = angle + 90，超 [0,180] 由舵机自然回弹。

        底层走车端 car action "set_storage_angle"（runtime 已暴露，接受任意 angle）。

        参数:
            angle: 目标角度（°），见上文合法区间说明。
            speed: 舵机速度，默认 100。
            timeout: job 超时（秒）。

        返回:
            {"ok": bool, "angle": float, "raw_job": dict}
        """
        job = self._call_car(
            "set_storage_angle", timeout=timeout,
            angle=angle, speed=speed, sync=True,
        )
        # 任意角度不属于 LEFT/RIGHT 两档，缓存清成 UNKNOWN 避免 get_storage() 误报。
        self._storage_side_cache = "UNKNOWN"
        return {
            "ok": bool(isinstance(job, dict) and job.get("status") == "succeeded"),
            "angle": float(angle),
            "raw_job": job,
        }

    def grasp(self, on: bool, timeout: float = 10.0) -> dict:
        # ⚠️ 不能走 `_call_arm("grasp", bool(on), sync=True, timeout=timeout)`：
        #   `_call_arm(self, name, timeout=20.0, *args, ...)` 的 `timeout` 是第 2 个
        #   位置形参，`bool(on)` 位置传进去被当成 timeout（=True），再传
        #   timeout=timeout 报 "got multiple values for argument 'timeout'"（target3.py 复现）。
        # ⚠️ 也不能走 `_call_arm("grasp", on=bool(on), ...)`：
        #   runtime ARM_ACTIONS["grasp"] lambda 透传 **kwargs 到 arm_base.grasp(value)，
        #   arm_base.grasp 签名只接 `value`，收到 on/sync/timeout 全 TypeError → job failed，
        #   同步路径下业务层不检查 status 就静默失败（球没吸起来）。
        # 正解：直接调 http.execute_arm_action，它的 timeout 是 keyword-only（`def
        # execute_arm_action(self, name, *args, timeout=None, ...)`），
        # 位置参 `bool(on)` 进 *args，timeout=keyword 正常，到车端 arm_obj.grasp(True) 命中 value。
        return self.http.execute_arm_action(
            "grasp", bool(on),
            timeout=timeout, sync=True,
        )

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

    def reset_all(self, arm_angle: float = 90, hand_angle: float = -90,
                  x_direction: str = "right",
                  reset_x_velocity_mms: float = 20.0,
                  timeout: float = 120.0) -> dict:
        """2026-07-16 新加：复合复位 (x + 大臂 + 手爪 并行 → y 串行)。

        三路并行(x 撞墙 / 大臂回 +90 复位位 / 手爪回 UP),完成后 reset_y 触底。
        大臂默认 2026-07-27 改为 +90°(业务硬限上界 + 复位位,旧版是 0° MID)。
        timeout 给够(reset_y + reset_x + 2 servo 总耗时约 30-40s)。

        物理前提:机械臂当前不在右墙边(<50mm) — 否则 reset_x 不 calibrate。
        """
        return self._call_arm(
            "reset_all", timeout=timeout,
            arm_angle=arm_angle, hand_angle=hand_angle,
            x_direction=x_direction,
            reset_x_velocity=reset_x_velocity_mms / 1000.0,
        )

    # ---- composite (2026-07-31 PR#13) ----
    #
    # 业务层 pick / release / go_home 用这三个方法替换原来的"三步串行"。
    # 设计原则:
    #   - entry-only validation (一次性 _check_y_protected + _check_safe + arm/hand 限位)
    #   - 单次 _call_arm,内部一个 runtime job 内 ThreadPoolExecutor 真并发
    #   - 不要在本 wrapper 调 set_arm_angle / set_hand_angle (它们有 live-state 检查,会与并发冲突)

    def _validate_arm_angle_client(self, angle: float, action: str) -> None:
        """业务层大臂角度硬限 [+90, -150]° 校验。"""
        try:
            a = float(angle)
        except (TypeError, ValueError):
            raise ValueError(f"{action} arm_angle 必须是数字，收到: {angle!r}")
        if a > self._ARM_ANGLE_MAX or a < self._ARM_ANGLE_MIN:
            raise ValueError(
                f"{action} arm_angle({a}) 超出业务硬限 [{self._ARM_ANGLE_MIN}, "
                f"{self._ARM_ANGLE_MAX}]°。\n"
                f"  规则: 大臂角度 ∈ [+90, -150]°（+90 是复位位，-150 是结构极限）\n"
                f"  解决: 选 +90 (复位) / 0 (MID) / -90 / -150 等。"
            )

    def _validate_hand_angle_client(self, angle: float, action: str) -> None:
        """业务层手爪角度硬限 [-90, 0]° 校验。"""
        try:
            a = float(angle)
        except (TypeError, ValueError):
            raise ValueError(f"{action} hand 必须是数字，收到: {angle!r}")
        if a > self._HAND_ANGLE_MAX or a < self._HAND_ANGLE_MIN:
            raise ValueError(
                f"{action} hand({a}) 超出业务硬限 [{self._HAND_ANGLE_MIN}, "
                f"{self._HAND_ANGLE_MAX}]°。\n"
                f"  规则: 手爪角度 ∈ [-90, 0]°（DOWN=0, UP=-90）\n"
                f"  解决: 选 0 (DOWN) / -90 (UP) / 中间值。"
            )

    def composite_pick(
        self,
        arm_angle: float,
        x_mm: float,
        y_mm: float,
        hand: float = 0.0,
        speed: int = 80,
        timeout: float = 30.0,
    ) -> dict:
        """复合抓取（2026-07-31 PR#13）：底层并行 set_arm_angle + goto_position,再串行 hand + grasp。

        业务前置：必须先 move_y(<-30) 出保护区，再调本方法。
        入口校验：arm_angle ∈ [+90, -150]°、hand ∈ [-90, 0]°、y ∈ 软限位。
        返回 {"ok": bool, "steps": {...}} — ok=False 时由 caller 决定后续动作。

        Args:
            arm_angle: 大臂目标角度（°）。
            x_mm / y_mm: xy 目标（mm,业务单位）。
            hand: 手爪角度（°），默认 0=DOWN。
            speed: 舵机速度，默认 80。
            timeout: HTTP 同步超时（秒），默认 30。
        """
        action = "composite_pick"
        self._validate_arm_angle_client(arm_angle, action)
        self._validate_hand_angle_client(hand, action)
        self._check_y_protected(action)
        self._check_safe(y_mm=y_mm)
        return self._call_arm(
            action, timeout=timeout,
            arm_angle=arm_angle, x=_mm_to_m(x_mm), y=_mm_to_m(y_mm),
            hand=hand, speed=speed,
        )

    def composite_release(
        self,
        drop_x_mm: float = 0.0,
        drop_y_mm: float = 30.0,
        hand: float = 0.0,
        speed: int = 80,
        timeout: float = 30.0,
    ) -> dict:
        """复合释放（2026-07-31 PR#13）：保守序列 hand → goto_position → grasp(False)。

        释放场景 hand=DOWN 在大臂展开带时安全门会拒绝,因此串行手爪在前。
        业务前置：必须先 move_y(<-30) 出保护区。

        Returns:
            {"ok": bool, "steps": {"hand": bool, "position": bool, "grasp": bool}}
        """
        action = "composite_release"
        self._validate_hand_angle_client(hand, action)
        self._check_y_protected(action)
        self._check_safe(y_mm=drop_y_mm)
        return self._call_arm(
            action, timeout=timeout,
            drop_x=_mm_to_m(drop_x_mm), drop_y=_mm_to_m(drop_y_mm),
            hand=hand, speed=speed,
        )

    def composite_go_home(
        self,
        hand: float = -90.0,
        arm: float = 0.0,
        speed: int = 80,
        timeout: float = 30.0,
    ) -> dict:
        """复合回原点（2026-07-31 PR#13）：底层并行 set_arm_angle + goto_position,再串行 hand=UP。

        hand=-90 (UP) 是 init 位置,允许在保护区内调,故无需预出保护区。
        返回 {"ok": bool, "steps": {"arm": bool, "position": bool, "hand": bool}}。
        """
        action = "composite_go_home"
        self._validate_arm_angle_client(arm, action)
        self._validate_hand_angle_client(hand, action)
        # hand=UP / arm=0 是 init 位置,_check_y_protected 内置 allow_init_position 处理
        # 但 composite_go_home 是单次 HTTP,job 内顺序是 arm+position 并行 → hand,
        # arm=0 / hand=-90 都在 init,保护区对它们直接放行
        self._check_y_protected(action)
        return self._call_arm(
            action, timeout=timeout,
            hand=hand, arm=arm, speed=speed,
        )

    # ---- 2026-07-31: composite_run / composite_run_reset / vision ----

    def composite_run(
        self,
        *,
        arm: Optional[float] = None,
        x_mm: Optional[float] = None,
        y_mm: Optional[float] = None,
        hand: Optional[float] = None,
        speed: int = 80,
        timeout: float = 30.0,
    ) -> dict:
        """薄封装 arm.composite_run(arm, x, y, hand)，任一 None 跳过。

        业务前置：所有非 None 参数必须先过 _check_y_protected / _check_safe。
        """
        if y_mm is not None:
            self._check_y_protected("composite_run")
            self._check_safe(y_mm=y_mm)
        return self._call_arm(
            "composite_run", timeout=timeout,
            arm=arm,
            x=_mm_to_m(x_mm) if x_mm is not None else None,
            y=_mm_to_m(y_mm) if y_mm is not None else None,
            hand=hand, speed=speed,
        )

    def composite_run_reset(
        self,
        *,
        arm_angle: float = 90.0,
        hand_angle: float = -90.0,
        x_direction: str = "right",
        reset_x_velocity_mms: float = 20.0,
        timeout: float = 60.0,
    ) -> dict:
        """薄封装 arm.composite_run_reset() —— x 撞墙 + arm + hand 并行 + y 触底收尾"""
        return self._call_arm(
            "composite_run_reset", timeout=timeout,
            arm_angle=arm_angle, hand_angle=hand_angle,
            x_direction=x_direction,
            reset_x_velocity=reset_x_velocity_mms / 1000.0,
        )

    @property
    def vision(self):
        """懒构造：首次访问时建 ArmVisionClient"""
        if self._vision is None:
            ArmVisionClient = _import_vision()
            self._vision = ArmVisionClient(self.http)
        return self._vision

    def _make_vision_with_move(self):
        """返回一个 move_fn 已经被 _check_safe 包裹的 vision client（业务层用）。

        2026-07-31: 同时 wrap find_target 和 find_target_realtime —— realtime 路径
        之前未走安全门（HIGH gate-bypass-sibling-path），现统一注入 safe_move_fn。
        """
        ArmVisionClient = _import_vision()
        client = ArmVisionClient(self.http)
        original_find = client.find_target
        original_find_realtime = client.find_target_realtime

        def _safe_move(nx: float, ny: float) -> dict:
            self._check_y_protected("find_target")
            self._check_safe(y_mm=ny)
            return self.move_xy(nx, ny, timeout=5.0)   # 2026-07-31: 5s（伺服高频）

        def _safe_wrap(original, label: str):
            def safe_fn(selector, *, x_mm, y_mm, **kwargs):
                move_fn = kwargs.pop("move_fn", None) or _safe_move
                return original(selector, x_mm=x_mm, y_mm=y_mm, move_fn=move_fn, **kwargs)
            safe_fn.__name__ = label
            return safe_fn

        client.find_target = _safe_wrap(original_find, "safe_find_target")  # type: ignore[method-assign]
        client.find_target_realtime = _safe_wrap(original_find_realtime, "safe_find_target_realtime")  # type: ignore[method-assign]
        return client

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

    # ---- realtime 真值路径（arm_feed 20Hz 守护线程缓存）----
    #
    # 与 _read_raw_state 的区别：
    #   - _read_raw_state 走 y_get_position / x_get_position → 触发底层 calibrate
    #     框架（已坏，实测同位置不同时间读数飘 0.3 / 22.5 / 46.9mm，详见 ARM_API §11）
    #   - 本节两个方法走 /v1/realtime/arm/state（arm_feed 守护线程每 20Hz 刷新），
    #     不进 job_queue、不打 ZMQ、不抢 car_lock，是业务层**唯一**可信 x/y 位置源
    #
    # 失败语义：HTTP 失败 / arm_feed 未启 / 字段 None → 返回 None。
    # 调用方应自己处理 None（一般 raise RuntimeError 让外层兜底退出）。
    #
    # 2026-07-30：以前这两个方法把异常整个吞掉直接 return None，业务层只能
    # 报"arm_feed 可能未启动"，实际最常见的原因是**网络不通**（换网段后
    # main/settings.py 的 IP 没跟着改）。现在统一走 _read_arm_state_realtime()，
    # 失败原因记进 self._last_realtime_error 并 logger.warning，调用方可以读
    # last_realtime_error() 把真实原因带进错误信息。

    def _read_arm_state_realtime(self) -> Optional[dict]:
        """读 /v1/realtime/arm/state 的 arm_state dict。失败返回 None。

        失败原因写进 ``self._last_realtime_error``（供 ``last_realtime_error()``
        读取）并 ``logger.warning``，避免上层把「网络不通」误报成「arm_feed 未启动」。

        Returns:
            arm_state dict（含 x_mm / y_mm / active / ref_encoder ...），失败 → None。
        """
        try:
            resp = self.http.get_arm_state()
        except Exception as exc:
            self._last_realtime_error = (
                f"HTTP 请求失败({type(exc).__name__}: {exc})—— "
                f"多半是 runtime 地址不对或网络不通，"
                f"当前 api_base={getattr(self.http, 'api_base', '?')}"
            )
            logger.warning("realtime arm/state 读取失败: %s", self._last_realtime_error)
            return None
        if not isinstance(resp, dict):
            self._last_realtime_error = f"响应不是 dict: {type(resp).__name__}"
            logger.warning("realtime arm/state %s", self._last_realtime_error)
            return None
        st = resp.get("arm_state")
        if not isinstance(st, dict):
            self._last_realtime_error = "响应里没有 arm_state 字段（runtime 版本不匹配？）"
            logger.warning("realtime arm/state %s", self._last_realtime_error)
            return None
        if not st.get("active", True):
            # active=False 说明 arm_feed 守护线程真的没跑（急停 / cancel 后 _stop_flag）
            self._last_realtime_error = (
                "arm_feed 守护线程未运行(active=False)—— "
                "急停或 cancel_job 后需 POST /v1/control/reset-stop 再重启 feed"
            )
            logger.warning("realtime arm/state %s", self._last_realtime_error)
            return None
        self._last_realtime_error = None
        return st

    def last_realtime_error(self) -> Optional[str]:
        """最近一次 realtime 读取失败的原因（成功后清空，从未失败过 → None）。

        业务层拿到 ``_read_x_mm_realtime`` / ``_read_y_mm_realtime`` 的 None 后，
        应该把本方法的返回值带进自己的错误信息，否则用户看到的永远是
        「arm_feed 可能未启动」这种猜测。
        """
        return getattr(self, "_last_realtime_error", None)

    def _read_x_mm_realtime(self) -> Optional[float]:
        """读机械臂 x 真值（mm，arm_feed 20Hz 缓存）。

        业务层**唯一**可信 x 位置源。详见 ARM_API.md §11。
        x_get_position action 走坏掉的 calibrate 框架，禁用。

        Returns:
            x_mm float，失败 / arm_feed 未启 → None（原因见 ``last_realtime_error()``）。
        """
        st = self._read_arm_state_realtime()
        if st is None:
            return None
        x = st.get("x_mm")
        if x is None:
            self._last_realtime_error = "arm_state.x_mm 为 None（arm_feed 刚启动？）"
            return None
        try:
            return float(x)
        except (TypeError, ValueError):
            self._last_realtime_error = f"arm_state.x_mm 不是数字: {x!r}"
            return None

    def _read_y_mm_realtime(self) -> Optional[float]:
        """读机械臂 y 真值（mm，arm_feed 20Hz 缓存）。

        与 _read_x_mm_realtime 同路径。arm_feed 未启 / 字段 None → 返回 None。

        Returns:
            y_mm float，失败 → None（原因见 ``last_realtime_error()``）。
        """
        st = self._read_arm_state_realtime()
        if st is None:
            return None
        y = st.get("y_mm")
        if y is None:
            self._last_realtime_error = "arm_state.y_mm 为 None（arm_feed 刚启动？）"
            return None
        try:
            return float(y)
        except (TypeError, ValueError):
            self._last_realtime_error = f"arm_state.y_mm 不是数字: {y!r}"
            return None

    # ---- 球检测验证 (BALL_VERIFIED_*, target1 位姿下) ----
    #
    # 2026-07-28 加: 业务层 (target2.fetch_balls 等等) 之外的一般 arm 客户端
    # 调用方也能用 ArmClient.verify_ball(ball) 验证检测结果。默认阈值 = 顶层
    # verify_ball() 的 BALL_VERIFIED_* (target1 位姿下 5 次实测基线, 蓝黄共用)。
    # 阈值可临时覆盖 (例如换位姿校准)。

    def verify_ball(
        self,
        ball: dict,
        *,
        cx_min: float = BALL_VERIFIED_CX_MIN,
        cx_max: float = BALL_VERIFIED_CX_MAX,
        cy_min: float = BALL_VERIFIED_CY_MIN,
        cy_max: float = BALL_VERIFIED_CY_MAX,
        w_min: float = BALL_VERIFIED_W_MIN,
        w_max: float = BALL_VERIFIED_W_MAX,
        h_min: float = BALL_VERIFIED_H_MIN,
        h_max: float = BALL_VERIFIED_H_MAX,
        area_min: float = BALL_VERIFIED_AREA_MIN_VERIFY,
        area_max: float = BALL_VERIFIED_AREA_MAX_VERIFY,
        score_min: float = BALL_VERIFIED_SCORE_MIN_VERIFY,
        aspect_min: float = BALL_VERIFIED_ASPECT_MIN,
        aspect_max: float = BALL_VERIFIED_ASPECT_MAX,
    ) -> bool:
        """验证 ball dict 是否在 BALL_VERIFIED_* 期望范围 (target1 位姿下)。

        委托给模块级 ``verify_ball()`` 函数。详细语义见该函数 docstring。

        Args:
            ball: dict, 期望含 cx_norm / cy_norm / w_norm / h_norm / score 字段。
            其他参数: 阈值覆盖, 默认值 = BALL_VERIFIED_*, 调用方可临时调。

        Returns:
            True = 在范围内 (球检测合理); False = 越界 (噪声框或位姿偏移)。
        """
        return verify_ball(
            ball,
            cx_min=cx_min, cx_max=cx_max,
            cy_min=cy_min, cy_max=cy_max,
            w_min=w_min, w_max=w_max,
            h_min=h_min, h_max=h_max,
            area_min=area_min, area_max=area_max,
            score_min=score_min,
            aspect_min=aspect_min, aspect_max=aspect_max,
        )

    # ---- x_speed safety watchdog（belt-slip 兜底）----
    #
    # 背景：x 轴是 motor_280 + 编码器 + 同步带，belt-slip 下电机在转但车不动。
    # 纯开环 x_speed 不知道车动没动，堵转时空转烧带子/电机。
    #
    # 兜底策略：每次开环 x_speed 时起一个 daemon 线程，周期（默认 100ms）读
    # realtime x_mm，若 max_stale_s 秒内 x 变化 < 0.5mm，自动调 x_speed(0) 停机。
    # 见 ARM_API.md §10 + memory [[x-speed-safety-watchdog]]。
    #
    # latest-wins：再次调用 x_speed_with_safety 会取消前一个 watchdog + 设新速度；
    # 显式 stop_x_speed_safety() 立即停。watchdog 不依赖 _call_arm 同步返回，
    # 完全可以跟其他动作并发。

    def x_speed_with_safety(
        self,
        velocity: float,
        max_stale_s: float = 2.0,
        poll_interval_s: float = 0.1,
        move_threshold_mm: float = 0.5,
        timeout: float = 10.0,
    ) -> dict:
        """开环 x_speed + 后台 watchdog 兜底 belt-slip 堵转。

        Args:
            velocity: 目标速度（m/s，正值向 x 增大方向，负值反向）。
                     与车端 arm.x_speed(velocity) 同单位（m/s）。
            max_stale_s: watchdog 容忍"无位移"最长时间（秒）。超此值自动 x_speed(0)。
            poll_interval_s: watchdog 轮询间隔（秒）。
            move_threshold_mm: 判定"x 有动"的最小位移（mm），避免编码器噪声误判。
            timeout: car action HTTP 超时（秒）。

        Returns:
            ``/v1/execute`` 异步返回的 job dict（sync=False 不等完成）。

        注意：
          - 调用后 motor 立即按 velocity 转；调用方负责在合适时机调
            ``stop_x_speed_safety()`` 或再调一次 ``x_speed_with_safety(0)``。
          - latest-wins：再调一次会自动取消前一个 watchdog + 设新速度。
        """
        v_ms = float(velocity)
        with self._x_safety_lock:
            # 1) 取消前一个 watchdog（保留取消设置，但下面要立刻建新的）
            self._cancel_x_safety_locked()

            # 2) 起新 watchdog
            start_x = self._read_x_mm_realtime()  # 起点（realtime 真值）
            stop_event = threading.Event()
            self._x_safety_stop_event = stop_event
            self._x_safety_start_x_mm = start_x
            self._x_safety_velocity_ms = v_ms

            def _watchdog() -> None:
                last_x = start_x
                last_change_t = time.time()
                # 在内部循环里访问 self，daemon 线程随 client 生命周期共存
                while not stop_event.wait(poll_interval_s):
                    cur = self._read_x_mm_realtime()
                    if cur is None:
                        # 读不到就继续等下一轮（realtime 偶发不可用）
                        continue
                    if abs(cur - last_x) > move_threshold_mm:
                        last_x = cur
                        last_change_t = time.time()
                    elif (time.time() - last_change_t) > max_stale_s:
                        # 卡住超时 → 强停 + 退出
                        try:
                            self._call_arm(
                                "x_speed", timeout=5.0, sync=False, velocity=0.0
                            )
                            logger.warning(
                                "x_speed_with_safety: x_mm %.1fmm 超 %ss 未变，"
                                "已自动 x_speed(0)（belt-slip 兜底）",
                                last_x, max_stale_s,
                            )
                        except Exception as exc:  # pragma: no cover
                            logger.warning(
                                "x_speed_with_safety: 自动停机失败: %s", exc
                            )
                        return

            t = threading.Thread(
                target=_watchdog, daemon=True, name="x-safety-watchdog"
            )
            self._x_safety_thread = t
            t.start()

        # 3) 下发开环速度（异步，不等完成）
        return self._call_arm(
            "x_speed", timeout=timeout, sync=False, velocity=v_ms
        )

    def stop_x_speed_safety(self) -> dict:
        """停 watchdog + 立即 x_speed(0)。

        行为：
          - 取消在跑的 watchdog 线程（latest-wins 的"前一个"被取消语义）。
          - 下发一次 x_speed(0) 异步停止电机。

        Returns:
            ``/v1/execute`` 异步返回的 x_speed(0) job dict。

        注意：即使当前没有 safety session（is_x_safety_active()=False），
        调本方法也安全 —— watchdog 取消 no-op + x_speed(0) 必下。
        """
        with self._x_safety_lock:
            self._cancel_x_safety_locked()
        # 立即下发停车（async，不等完成）
        return self._call_arm(
            "x_speed", timeout=5.0, sync=False, velocity=0.0
        )

    def is_x_safety_active(self) -> bool:
        """watchdog 线程是否在跑。

        Returns:
            True = 上一次 ``x_speed_with_safety()`` 起的 watchdog 还在监控中；
            False = 没在跑（或已被 ``stop_x_speed_safety()`` / 新的
            ``x_speed_with_safety()`` 取消）。
        """
        with self._x_safety_lock:
            t = self._x_safety_thread
            return t is not None and t.is_alive()

    def _cancel_x_safety_locked(self) -> None:
        """取消 watchdog（调用方必须持 ``_x_safety_lock``）。"""
        if self._x_safety_stop_event is not None:
            self._x_safety_stop_event.set()
        self._x_safety_thread = None
        self._x_safety_stop_event = None
        self._x_safety_start_x_mm = None
        self._x_safety_velocity_ms = 0.0

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
