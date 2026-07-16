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
        """大臂角度控制（业务层硬限 [0, -150]°）。

        LEFT=+93 撞车、<-180 撞车，所以业务只允许 ≤ 0 且 ≥ -150。
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
        """回到 y=0, x=0，hand=UP（-90），arm=MID（0）。"""
        self.client.set_hand_angle(-90.0, speed=80, timeout=10.0)
        self.client.set_arm_angle(0.0, speed=80, timeout=10.0)
        return self.move_xy(0.0, 0.0)

    # ---- 复位 ----

    def reset_y(self, timeout: float = 30.0) -> dict:
        """y 步进电机触底复位（车端跑 reset_y，**仅动 y**）。

        仅在 y 跑偏严重（补偿不收敛）时调。
        注：reset_x 已删除（2026-07-16）。x 位置由视觉闭环控制，无软件复位。
        """
        return self.client._call_arm("reset_y", timeout=timeout)

    # ---- 业务组合 ----

    def pick(self, arm_angle: float, x_mm: float, y_mm: float) -> dict:
        """set_arm_angle -> move_xy -> grasp(True)。"""
        self.set_arm_angle(arm_angle)
        self.move_xy(x_mm=x_mm, y_mm=y_mm)
        return self.grasp(True)

    def release(self, drop_x_mm: float = 0.0, drop_y_mm: float = 30.0) -> dict:
        """set_hand_angle(DOWN=0) -> move_xy -> grasp(False)。"""
        self.client.set_hand_angle(0.0, speed=80, timeout=10.0)
        self.move_xy(x_mm=drop_x_mm, y_mm=drop_y_mm)
        return self.grasp(False)
