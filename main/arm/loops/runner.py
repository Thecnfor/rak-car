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

    def move_x(self, x_mm: float, timeout: Optional[float] = None) -> dict:
        return self.client.move_x(x_mm=x_mm, timeout=timeout or self.default_timeout_s)

    def move_y(self, y_mm: float, timeout: Optional[float] = None) -> dict:
        return self.client.move_y(y_mm=y_mm, timeout=timeout or self.default_timeout_s)

    def set_side(self, side: str, timeout: Optional[float] = None) -> dict:
        return self.client.set_side(side, timeout=timeout or self.default_timeout_s)

    def set_hand(self, hand: str, timeout: Optional[float] = None) -> dict:
        return self.client.set_hand(hand, timeout=timeout or self.default_timeout_s)

    def grasp(self, on: bool, timeout: Optional[float] = None) -> dict:
        return self.client.grasp(on, timeout=timeout or self.default_timeout_s)

    def go_home(self) -> dict:
        """回到 y=0, x=0，hand=UP，side=MID。"""
        self.client.set_hand("UP", timeout=10)
        self.client.set_side("MID", timeout=10)
        return self.move_xy(0.0, 0.0)

    # ---- 业务组合 ----

    def pick(self, side: str, x_mm: float, y_mm: float) -> dict:
        """set_side -> move_xy -> grasp(True)。"""
        self.set_side(side)
        self.move_xy(x_mm=x_mm, y_mm=y_mm)
        return self.grasp(True)

    def release(self, drop_x_mm: float = 0.0, drop_y_mm: float = 30.0) -> dict:
        """set_hand(DOWN) -> move_xy -> grasp(False)。"""
        self.set_hand("DOWN")
        self.move_xy(x_mm=drop_x_mm, y_mm=drop_y_mm)
        return self.grasp(False)
