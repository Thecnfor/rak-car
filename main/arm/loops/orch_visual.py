"""main/arm/loops/orch_visual.py
视觉联调编排器：chassis 追踪 + arm 4-DOF + 抓取，一条龙。

典型流水线：
    Stage 1 chassis: track_chassis() 把目标拉到画面中心（两自由度，允许车水平+前后）
    Stage 2 arm:    track_velocity_pick() 臂结构映射 + 吸嘴 setpoint，精准 4-DOF 对齐
    Stage 3 grasp:  y 降到 0 → 吸气

用法：
    from main.arm.loops.orch_visual import VisualOrchestrator
    orch = VisualOrchestrator()
    result = orch.track_and_grasp("h_tu_dou")
    print(result.arrived_chassis, result.arrived_arm, result.grasp_ok)
"""
from __future__ import annotations

import dataclasses
import logging
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple, Union

from ..api import ArmClient
from ..state import ArmOrigin
from .runner import ArmRunner

logger = logging.getLogger(__name__)


# ---- 感知 ----


@dataclass
class OrchFrame:
    """联调过程的一帧状态。"""
    chassis_found: bool = False
    chassis_cx: Optional[float] = None
    chassis_cy: Optional[float] = None
    chassis_vx: Optional[float] = None
    chassis_vy: Optional[float] = None
    arm_found: bool = False
    arm_dx: Optional[float] = None
    arm_dy: Optional[float] = None
    arm_track_hits: int = 0
    elapsed_s: float = 0.0


# ---- 结果 ----


@dataclass
class OrchResult:
    """track_and_grasp() 的返回值。"""
    arrived_chassis: bool = False
    reason_chassis: str = "unknown"
    arrived_arm: bool = False
    reason_arm: str = "unknown"
    grasp_ok: bool = False
    trace: List[OrchFrame] = field(default_factory=list)
    elapsed_s: float = 0.0
    # 来自各阶段的具体结果
    chassis_result: Optional["TrackChassisResult"] = None
    arm_result: Optional[dict] = None


# ---- 主类 ----


class VisualOrchestrator:
    """视觉联调编排器：chassis 追踪 → arm 4-DOF → 抓取。

    默认参数基于 2026-08-02 现场标定：
      - chassis: 画面 cx ↔ 车前后 vx（cx 负→后退），画面 cy ↔ 车横向 vy（cy 负→右移）
      - arm: y=-180 姿态下，画面 cx ← 大臂角，画面 cy ← x 十字位置

    流水线：
      1. chassis track_chassis(): 把目标拉到画面中心（vx/vy 两自由度）
      2. arm track_velocity_pick(): 臂结构映射，精准 4-DOF 对齐到吸嘴 setpoint
      3. y 降到 0 → 吸气

    用法：
        orch = VisualOrchestrator()
        result = orch.track_and_grasp("h_tu_dou")
        print(result.arrived_chassis, result.arrived_arm, result.grasp_ok)
    """

    def __init__(
        self,
        arm: Optional[ArmClient] = None,
        default_timeout_s: float = 30.0,
    ):
        self._arm = arm
        self._default_timeout_s = default_timeout_s
        self._arm_runner: Optional[ArmRunner] = None

    # ---- 懒加载 arm client ----

    @property
    def arm_runner(self) -> ArmRunner:
        if self._arm_runner is None:
            if self._arm is None:
                self._arm = ArmClient.connect()
            self._arm_runner = ArmRunner(self._arm, self._default_timeout_s)
        return self._arm_runner

    @property
    def arm(self) -> ArmClient:
        return self.arm_runner.client

    # ---- Stage 1: chassis 追踪 ----

    def align_chassis(
        self,
        target: Union[str, List[str]],
        *,
        # chassis 参数（透传给 track_chassis）
        sign_vx: int = -1,
        sign_vy: int = +1,
        kp: float = 0.20,
        v_max: float = 0.12,
        v_slew: float = 0.02,
        deadband: float = 0.08,
        hold_frames: int = 5,
        max_lost_frames: int = 60,
        watchdog_ms: float = 2000.0,
        hz: float = 20.0,
        max_seconds: float = 15.0,
        dry_run: bool = False,
        on_tick: Optional[Callable[..., None]] = None,
    ):
        """Stage 1: 底盘追踪，把目标 bbox 拉到画面中心。

        默认参数（2026-08-02 现场稳档）：
          sign_vx=-1: 画面 cx（横向）↔ 车前后（vx），cx 负 → vx 负（后退）
          sign_vy=+1: 画面 cy（纵向）↔ 车横向（vy），cy 负 → vy 正（右移）

        如果换车/摄像头后方向反了：
          sign_vx=+1（cx 负 → 前进）
          sign_vy=-1（cy 负 → 左移）
        """
        from main.chassis import track_chassis, TrackChassisResult

        return track_chassis(
            target=target,
            sign_vx=sign_vx,
            sign_vy=sign_vy,
            kp=kp,
            v_max=v_max,
            v_slew=v_slew,
            deadband=deadband,
            hold_frames=hold_frames,
            max_lost_frames=max_lost_frames,
            watchdog_ms=watchdog_ms,
            hz=hz,
            max_seconds=max_seconds,
            dry_run=dry_run,
            on_tick=on_tick,
        )

    # ---- Stage 2: arm 4-DOF ----

    def align_arm(
        self,
        label: str,
        *,
        # arm 4-DOF 参数（透传给 track_velocity_pick）
        x_start: float = 0.0,
        y_start: float = -180.0,
        arm_start: float = -90.0,
        hand_start: float = 0.0,
        grasp_y_mm: float = 0.0,
        mode: str = "pick",
        sign_arm: float = 1.0,
        sign_x: float = -1.0,
        gain_arm: float = 0.4,
        gain_x: float = 0.08,
        deadzone: float = 0.02,
        max_vel: float = 0.15,
        settle_hits: int = 3,
        hold_s: float = 0.5,
        lift_back: bool = True,
        lock_first: bool = True,
        timeout: float = 30.0,
        hz: float = 20.0,
        no_reset: bool = False,
        dry_run: bool = False,
    ):
        """Stage 2: arm 4-DOF 对齐（臂结构映射版，2026-08-02 现场标定）。

        坐标系（y=-180 姿态下）：
          画面 cx ← 大臂角（dx>0 → arm 更负 → sign_arm=+1）
          画面 cy ← x 十字位置（dy>0 → x 往左 → sign_x=-1）
          y 十字锁 0，手抓固定 0° 朝下

        吸嘴 setpoint：读 origin.nozzle_offset_for(label)，按 label 查表补偿。

        dry_run=True 时：只跑控制律不下发 grasp 动作。
        """
        return self.arm_runner.track_velocity_pick(
            label=label,
            x_start=x_start, y_start=y_start,
            arm_start=arm_start, hand_start=hand_start,
            grasp_y_mm=grasp_y_mm,
            mode=mode,
            sign_arm=sign_arm, sign_x=sign_x,
            gain_arm=gain_arm, gain_x=gain_x,
            deadzone=deadzone, max_vel=max_vel,
            settle_hits=settle_hits, hold_s=hold_s,
            lift_back=lift_back, lock_first=lock_first,
            timeout=timeout, hz=hz,
            no_reset=no_reset,
        )

    # ---- Stage 3: 抓取 ----

    def grasp(
        self,
        *,
        x_mm: float = 0.0,
        y_mm: float = 0.0,
        timeout: float = 5.0,
        dry_run: bool = False,
    ) -> bool:
        """Stage 3: y 降到 y_mm → 吸气。dry_run=True 不真动作。"""
        if dry_run:
            logger.info("grasp dry_run: y=%.1f → grasp", y_mm)
            return True
        try:
            self.arm.move_y(y_mm, timeout=timeout)
            time.sleep(0.2)
            self.arm.grasp(True)
            logger.info("grasp done at y=%.1f mm", y_mm)
            return True
        except Exception as exc:
            logger.error("grasp failed: %s", exc)
            return False

    # ---- 一条龙流水线 ----

    def track_and_grasp(
        self,
        label: str,
        *,
        # === Stage 1: chassis ===
        chassis_max_seconds: float = 15.0,
        chassis_dry_run: bool = False,
        chassis_sign_vx: int = -1,
        chassis_sign_vy: int = +1,
        chassis_kp: float = 0.20,
        chassis_v_max: float = 0.12,
        chassis_deadband: float = 0.08,
        chassis_hold_frames: int = 5,
        chassis_max_lost_frames: int = 60,
        chassis_on_tick=None,
        skip_chassis: bool = False,
        # === Stage 2: arm ===
        arm_timeout: float = 30.0,
        arm_x_start: float = 0.0,
        arm_y_start: float = -180.0,
        arm_arm_start: float = -90.0,
        arm_hand_start: float = 0.0,
        arm_sign_arm: float = 1.0,
        arm_sign_x: float = -1.0,
        arm_gain_arm: float = 0.4,
        arm_gain_x: float = 0.08,
        arm_deadzone: float = 0.02,
        arm_settle_hits: int = 3,
        skip_arm: bool = False,
        arm_dry_run: bool = False,
        # === Stage 3: grasp ===
        grasp_y_mm: float = 0.0,
        grasp_timeout: float = 5.0,
        skip_grasp: bool = False,
        grasp_dry_run: bool = False,
    ) -> OrchResult:
        """一条龙：chassis 追踪 → arm 4-DOF 对齐 → 抓取。

        全程打印进度。任何一个阶段失败不会 abort，后续阶段可以按 skip_* 参数跳过。

        参数：
            label              - 目标 label（h_tu_dou / cylinder_2 / water / ...）
            chassis_max_seconds - Stage 1 超时
            chassis_dry_run   - True 时 Stage 1 不真发车
            chassis_sign_vx / chassis_sign_vy - chassis 轴符号（2026-08-02 现场标定）
            arm_timeout       - Stage 2 超时
            skip_chassis       - True 时跳过 Stage 1
            skip_arm          - True 时跳过 Stage 2
            skip_grasp        - True 时跳过 Stage 3
            grasp_dry_run     - True 时 Stage 3 不真吸气

        用法：
            orch = VisualOrchestrator()
            result = orch.track_and_grasp(
                "h_tu_dou",
                chassis_max_seconds=15.0,
                arm_timeout=30.0,
            )
            print(result.arrived_chassis, result.arrived_arm, result.grasp_ok)
        """
        t0 = time.monotonic()
        result = OrchResult()

        # === Stage 1: chassis ===
        if skip_chassis:
            logger.info("[orch] Stage 1 chassis: SKIPPED")
            result.reason_chassis = "skipped"
        else:
            logger.info(
                "[orch] Stage 1 chassis: track_chassis(%s) max=%.1fs dry_run=%s",
                label, chassis_max_seconds, chassis_dry_run,
            )
            cr = self.align_chassis(
                target=label,
                sign_vx=chassis_sign_vx, sign_vy=chassis_sign_vy,
                kp=chassis_kp, v_max=chassis_v_max,
                deadband=chassis_deadband, hold_frames=chassis_hold_frames,
                max_lost_frames=chassis_max_lost_frames,
                max_seconds=chassis_max_seconds,
                dry_run=chassis_dry_run,
                on_tick=chassis_on_tick,
            )
            result.chassis_result = cr
            result.arrived_chassis = cr.arrived
            result.reason_chassis = cr.reason
            logger.info(
                "[orch] Stage 1 chassis done: arrived=%s reason=%s frames=%d",
                cr.arrived, cr.reason, cr.frames,
            )

        # === Stage 2: arm ===
        if skip_arm:
            logger.info("[orch] Stage 2 arm: SKIPPED")
            result.reason_arm = "skipped"
        else:
            logger.info(
                "[orch] Stage 2 arm: track_velocity_pick(%s) timeout=%.1fs dry_run=%s",
                label, arm_timeout, arm_dry_run,
            )
            ar = self.align_arm(
                label=label,
                x_start=arm_x_start, y_start=arm_y_start,
                arm_start=arm_arm_start, hand_start=arm_hand_start,
                sign_arm=arm_sign_arm, sign_x=arm_sign_x,
                gain_arm=arm_gain_arm, gain_x=arm_gain_x,
                deadzone=arm_deadzone,
                settle_hits=arm_settle_hits,
                timeout=arm_timeout,
                no_reset=True,  # 保持 arm 姿态，Stage 3 要用到
                dry_run=arm_dry_run,
            )
            result.arm_result = ar
            result.arrived_arm = bool(ar.get("ok", False))
            result.reason_arm = ar.get("reason") or ("ok" if ar.get("ok") else "failed")
            logger.info(
                "[orch] Stage 2 arm done: ok=%s reason=%s trace_hits=%d",
                ar.get("ok"), ar.get("reason"), ar.get("trace_hits", 0),
            )

        # === Stage 3: grasp ===
        if skip_grasp:
            logger.info("[orch] Stage 3 grasp: SKIPPED")
        else:
            logger.info("[orch] Stage 3 grasp: y=%.1f mm", grasp_y_mm)
            result.grasp_ok = self.grasp(
                x_mm=0.0, y_mm=grasp_y_mm,
                timeout=grasp_timeout,
                dry_run=grasp_dry_run,
            )
            logger.info("[orch] Stage 3 grasp done: ok=%s", result.grasp_ok)

        result.elapsed_s = time.monotonic() - t0
        logger.info(
            "[orch] ALL DONE: chassis=%s arm=%s grasp=%s elapsed=%.2fs",
            result.arrived_chassis, result.arrived_arm, result.grasp_ok, result.elapsed_s,
        )
        return result

    # ---- 便捷单步入口 ----

    def chassis_only(
        self,
        target: Union[str, List[str]],
        **kwargs,
    ):
        """仅 Stage 1 chassis 追踪。"""
        return self.align_chassis(target, **kwargs)

    def arm_only(self, label: str, **kwargs):
        """仅 Stage 2 arm 4-DOF 对齐。"""
        return self.align_arm(label, **kwargs)


__all__ = [
    "VisualOrchestrator",
    "OrchResult",
    "OrchFrame",
]
