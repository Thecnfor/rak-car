"""main/arm/loops/runner.py
ArmRunner：把 ArmClient + 业务动作 + dry-run 包成同步调用。
"""
from __future__ import annotations

import logging
import time
from typing import Optional

from ..api import ArmClient
from ..state import ArmState

logger = logging.getLogger(__name__)


class ArmRunner:
    """机械臂业务编排器。

    用法：
        client = ArmClient.connect()
        runner = ArmRunner(client)
        runner.move_xy(100, 80)
        runner.pick("LEFT", x_mm=120, y_mm=40)
    """

    def __init__(self, client: ArmClient, default_timeout_s: float = 30.0):
        self.client = client
        self.default_timeout_s = default_timeout_s

    # ---- 基础动作 ----

    def move_xy(
        self,
        x_mm: float,
        y_mm: float,
        v_max_mms: float = 150.0,
        a_max_mms2: float = 400.0,
        timeout: Optional[float] = None,
    ) -> dict:
        st_before = self.client.get_state()
        plan = self.client.traj.plan_xy(
            x0=st_before.x_mm, y0=st_before.y_mm,
            x1=x_mm, y1=y_mm,
            v_max=v_max_mms, a_max=a_max_mms2,
        )
        logger.info(
            "move_xy: (%.1f, %.1f) -> (%.1f, %.1f) mm, "
            "T_plan=%.2fs, peak_vx=%.1f peak_vy=%.1f mm/s",
            st_before.x_mm, st_before.y_mm, x_mm, y_mm,
            plan.T, plan.peak_vx, plan.peak_vy,
        )
        if timeout is None:
            timeout = max(self.default_timeout_s, plan.T * 2.0 + 1.0)
        t0 = time.time()
        job = self.client.move_xy(
            x_mm=x_mm, y_mm=y_mm,
            v_max_mms=v_max_mms, a_max_mms2=a_max_mms2,
            timeout=timeout,
        )
        t_actual = time.time() - t0
        st_after = self.client.get_state()
        logger.info(
            "move_xy done in %.2fs (plan=%.2fs); after=(%.1f, %.1f) mm",
            t_actual, plan.T, st_after.x_mm, st_after.y_mm,
        )
        return job

    def move_x(self, x_mm: float, timeout: Optional[float] = None,
               verify: bool = True) -> dict:
        """移动 x 轴（撞墙=0，远离为正）。

        verify=True 时：move 后对比 actual vs target，
        偏差 > origin.step_loss_x_mm 时 warn（不重发，因为 x 是 motor_280 闭环，
        跑偏通常是机械卡阻，重发没意义）。
        """
        job = self.client.move_x(x_mm=x_mm, timeout=timeout or self.default_timeout_s)
        if verify:
            self._verify_x(x_mm=x_mm)
        return job

    def move_y(self, y_mm: float, timeout: Optional[float] = None,
               verify: bool = True) -> dict:
        """移动 y 轴（触底=0，向下为正、向上为负）。

        驱动层（arm_base.move_y_position）已经自带丢步兜底，调用一次就收敛。
        verify=True 时：上层再读一次 actual，超阈值时 warn（不重发，避免和驱动打架）。
        """
        job = self.client.move_y(y_mm=y_mm, timeout=timeout or self.default_timeout_s)
        if verify:
            self._verify_y(y_mm=y_mm)
        return job

    def _verify_y(self, y_mm: float) -> None:
        """y 上层校验（驱动层已闭环，仅做 sanity check）。"""
        origin = self.client.origin
        threshold = origin.step_loss_y_mm if origin else 2.0
        try:
            state = self.client.get_state()
        except Exception as e:
            logger.warning("verify_y: 读状态失败: %s", e)
            return
        err = y_mm - state.y_mm
        if abs(err) > threshold:
            logger.warning(
                "verify_y: target=%.1f actual=%.1f err=%.1fmm（驱动层应已兜底，"
                "若反复看到建议 reset_y）", y_mm, state.y_mm, err,
            )

    def _verify_x(self, x_mm: float) -> None:
        """x 上层校验（x 是编码器闭环，正常不跑偏；偏差大多是机械卡阻）。"""
        origin = self.client.origin
        threshold = origin.step_loss_x_mm if origin else 5.0
        try:
            state = self.client.get_state()
        except Exception as e:
            logger.warning("verify_x: 读状态失败: %s", e)
            return
        err = x_mm - state.x_mm
        if abs(err) > threshold:
            logger.warning(
                "verify_x: target=%.1f actual=%.1f err=%.1fmm", x_mm, state.x_mm, err,
            )

    def set_arm_angle(self, angle: float, speed: int = 80,
                      timeout: Optional[float] = None) -> dict:
        """大臂角度控制（业务层硬限 [+90, -150]°，2026-07-27 重定义）。

        +90 是复位位（reset_position 用），-150 是结构极限。
        """
        return self.client.set_arm_angle(
            angle, speed=speed,
            timeout=timeout or self.default_timeout_s,
        )

    def set_storage(self, side: str, timeout: Optional[float] = None) -> dict:
        """切换存储仓到 LEFT/RIGHT（写死角度的两档枚举）。"""
        return self.client.set_storage(side, timeout=timeout or self.default_timeout_s)

    def get_storage(self) -> str:
        """只读当前存储仓档位（客户端缓存，不会下发舵机动作）。"""
        return self.client.get_storage()

    def grasp(self, on: bool, timeout: Optional[float] = None) -> dict:
        return self.client.grasp(on, timeout=timeout or self.default_timeout_s)

    def go_home(self) -> dict:
        """回到 y=0, x=0，hand=UP（-90），arm=MID（0）。

        2026-07-31 PR#13：改走 composite_go_home,内部 arm + xy 并行,hand 串行在末尾。
        """
        return self.client.composite_go_home(
            hand=-90.0, arm=0.0, speed=80, timeout=self.default_timeout_s,
        )

    # ---- 复位 ----

    def reset_y(self, timeout: float = 30.0) -> dict:
        """y 步进电机触底复位（车端跑 reset_y，**仅动 y**）。

        仅在 y 跑偏严重（补偿不收敛）时调。
        注：reset_x 已删除（2026-07-16）。x 位置由视觉闭环控制，无软件复位。
        """
        return self.client._call_arm("reset_y", timeout=timeout)

    # ---- 业务组合 ----

    def pick(self, arm_angle: float, x_mm: float, y_mm: float) -> dict:
        """复合抓取 (2026-07-31 PR#13)：底层并行 set_arm_angle + goto_position,再串行 hand + grasp。

        业务前置（必须满足，违反会抛 ValueError）：
          - 当前 y 必须 < -30mm(出保护区)。
            大臂舵机在 y ∈ [0, -30] 摆动会撞车,client wrapper 会拒绝。
          - 大臂角度 arm_angle ∈ [+90, -150]°。
          - 手爪角度 hand ∈ [-90, 0]°。

        Returns:
            {"ok": bool, "steps": {"arm": bool, "position": bool, "hand": bool, "grasp": bool}}
            ok=False 时 caller 决定是否 raise 或继续。
        """
        return self.client.composite_pick(
            arm_angle=arm_angle, x_mm=x_mm, y_mm=y_mm,
            hand=0.0, speed=80, timeout=self.default_timeout_s,
        )

    def release(self, drop_x_mm: float = 0.0, drop_y_mm: float = 30.0) -> dict:
        """复合释放 (2026-07-31 PR#13)：保守序列 hand → goto_position → grasp(False)。

        业务前置：当前 y 必须 < -30mm(出保护区)。
        Returns: {"ok": bool, "steps": {"hand": bool, "position": bool, "grasp": bool}}
        """
        return self.client.composite_release(
            drop_x_mm=drop_x_mm, drop_y_mm=drop_y_mm,
            hand=0.0, speed=80, timeout=self.default_timeout_s,
        )

    # ---- 2026-07-31: 视觉伺服高层组合 ----

    def move_to_vision_target(self, selector, *,
                              x_mm: float, y_mm: float,
                              arm_angle: float = 0.0, hand: float = -90.0,
                              mm_per_norm: float = 30.0,
                              settle_tol_norm: float = 0.05,
                              timeout: float = 10.0):
        """高层组合：composite_run 粗定位 → 视觉伺服精调。

        业务前置：必须在 y < -30mm 保护区外（composite_run 入口会校验）。

        Args:
            selector: TargetSelector（label/group/strategy）
            x_mm, y_mm: 目标位姿（业务单位 mm）
            arm_angle: 大臂目标角度（°）
            hand: 手爪角度（°），默认 -90 = UP（防撞）
            mm_per_norm: bbox 归一化坐标 → mm 转换系数（现场可调）
            settle_tol_norm: 收敛阈值
            timeout: 视觉伺服超时（秒）

        Returns:
            ServoResult（详见 VISION_SERVO_DESIGN.md）
        """
        self.client.composite_run(
            arm=arm_angle, x_mm=x_mm, y_mm=y_mm, hand=hand, timeout=20.0,
        )
        return self.client._make_vision_with_move().find_target(
            selector, x_mm=x_mm, y_mm=y_mm,
            mm_per_norm=mm_per_norm, settle_tol_norm=settle_tol_norm,
            timeout=timeout,
        )

    def pick_by_vision(self, selector, *,
                       x_mm: float, y_mm: float, arm_angle: float = -90.0,
                       settle_tol_norm: float = 0.05,
                       timeout: float = 10.0) -> dict:
        """最高层：粗定位 → 视觉伺服 → composite_pick → grasp。

        业务前置：必须在 y < -30mm 保护区外。
        """
        self.move_to_vision_target(
            selector, x_mm=x_mm, y_mm=y_mm,
            arm_angle=arm_angle, hand=-90.0,
            settle_tol_norm=settle_tol_norm, timeout=timeout,
        )
        return self.client.composite_pick(
            arm_angle=arm_angle, x_mm=x_mm, y_mm=y_mm,
            hand=0.0, speed=80, timeout=30.0,
        )

    # ---- 2026-07-31: 实时（WS push）版本 ----

    def move_to_vision_target_realtime(self, selector, *,
                                        x_mm: float, y_mm: float,
                                        arm_angle: float = 0.0, hand: float = -90.0,
                                        hz: float = 30.0,
                                        mm_per_norm: float = 30.0,
                                        settle_tol_norm: float = 0.05,
                                        timeout: float = 10.0):
        """高层组合：composite_run 粗定位 → 视觉伺服（WS 实时推流）。

        业务前置：必须在 y < -30mm 保护区外（composite_run 入口会校验）。
        """
        self.client.composite_run(
            arm=arm_angle, x_mm=x_mm, y_mm=y_mm, hand=hand, timeout=20.0,
        )
        return self.client._make_vision_with_move().find_target_realtime(
            selector, x_mm=x_mm, y_mm=y_mm,
            hz=hz, mm_per_norm=mm_per_norm,
            settle_tol_norm=settle_tol_norm, timeout=timeout,
        )

    def pick_by_vision_realtime(self, selector, *,
                                 x_mm: float, y_mm: float, arm_angle: float = -90.0,
                                 settle_tol_norm: float = 0.05,
                                 timeout: float = 10.0) -> dict:
        """最高层（实时版）：粗定位 → WS 伺服 → composite_pick → grasp。"""
        self.move_to_vision_target_realtime(
            selector, x_mm=x_mm, y_mm=y_mm,
            arm_angle=arm_angle, hand=-90.0,
            settle_tol_norm=settle_tol_norm, timeout=timeout,
        )
        return self.client.composite_pick(
            arm_angle=arm_angle, x_mm=x_mm, y_mm=y_mm,
            hand=0.0, speed=80, timeout=30.0,
        )

    def track_vision_target(self, selector, *,
                            x_mm: float, y_mm: float,
                            arm_angle: float = 90.0, hand: float = -90.0,
                            hz: float = 30.0,
                            mm_per_norm: float = 30.0,
                            timeout: float = 30.0):
        """持续实时追踪（永不收敛停）：WS 推送驱动，timeout 后返回。

        与 move_to_vision_target_realtime 区别：
          - realtime 版找到目标居中就停；track 版持续跟（即使居中也保持）
          - on_missing_track='wait' 默认（短暂丢失不 abort）

        适用：目标会移动的场景（边走边跟、流水线）。
        """
        # composite_run 粗定位（保持 arm_angle，不强切）
        self.client.composite_run(
            arm=arm_angle, x_mm=x_mm, y_mm=y_mm, hand=hand, timeout=20.0,
        )
        return self.client._make_vision_with_move().find_target_track(
            selector, x_mm=x_mm, y_mm=y_mm,
            hz=hz, mm_per_norm=mm_per_norm,
            timeout=timeout,
        )
