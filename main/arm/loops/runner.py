"""main/arm/loops/runner.py
ArmRunner：把 ArmClient + 业务动作 + dry-run 包成同步调用。
"""
from __future__ import annotations

import dataclasses
import logging
import time
from typing import Optional

from ..api import ArmClient
from ..state import ArmOrigin, ArmState
from ..vision import SelectionStrategy, TargetSelector

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

    # ---- 吸嘴偏移 setpoint（视觉伺服对准"吸嘴正下方"而非画面中心） ----

    def _nozzle_offset(self, label: Optional[str] = None):
        """origin 里已标定的吸嘴偏移；未标定(全 0)返回 None。

        2026-08-02 起按 label 查表：不同尺寸/类别目标检测中心不同
        （cylinders cy≈-0.50, balls cy≈-0.70），单一全局 setpoint 有残余误差。
        label 已知 → 查 nozzle_offset_map；未知 label 回落全局默认。
        """
        origin = self.client.origin or ArmOrigin()
        return origin.nozzle_offset_for(label)

    def _resolve_nozzle_setpoint(self, sx, sy, label: Optional[str] = None):
        """sx/sy 由调用方显式传入；两者皆 None 时回落到 origin 标定值（按 label 查表）。
        返回 (x, y) 或 None（未标定且未显式传 → 不注入，保持旧行为对准画面中心）。"""
        if sx is None and sy is None:
            return self._nozzle_offset(label)
        return (sx if sx is not None else 0.0,
                sy if sy is not None else 0.0)

    @staticmethod
    def _inject_setpoint(kwargs, sp):
        if sp is not None:
            kwargs["setpoint_x_norm"] = sp[0]
            kwargs["setpoint_y_norm"] = sp[1]
        return kwargs

    @staticmethod
    def _maybe_lock_first(selector, lock_first: bool):
        """多目标场景防跳变：默认 HIGHEST_SCORE 每帧重选会来回跳不收敛；
        lock_first=True 且 selector 未显式指定策略/轨迹时，升级为锁定首个目标。"""
        if (lock_first and selector.track_id is None
                and selector.strategy == SelectionStrategy.HIGHEST_SCORE.value):
            return dataclasses.replace(
                selector, strategy=SelectionStrategy.LOCK_FIRST_SEEN.value)
        return selector

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

    def move_x(self, x_mm: float, v_max_mms: Optional[float] = None,
               timeout: Optional[float] = None, verify: bool = True) -> dict:
        """移动 x 轴（撞墙=0，远离为正）。

        v_max_mms (2026-08-04): 可选速度上限 mm/s, None 时走 client.move_x 默认 40.
        verify=True 时：move 后对比 actual vs target，
        偏差 > origin.step_loss_x_mm 时 warn（不重发，因为 x 是 motor_280 闭环，
        跑偏通常是机械卡阻，重发没意义）。
        """
        move_kw = dict(x_mm=x_mm, timeout=timeout or self.default_timeout_s)
        if v_max_mms is not None:
            move_kw["v_max_mms"] = float(v_max_mms)
        job = self.client.move_x(**move_kw)
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
        """大臂角度控制（业务层硬限 [+150, -150]°，2026-08-05 用户放宽 +90→+150）。

        +90 是复位位（reset_position 用），-150 是结构极限，+150 是新对称上界。
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

    def suck(self, timeout: Optional[float] = None) -> dict:
        """吸气保持 (抓取协议 2026-08-02: 气泵开 + 阀门关 → 真空吸住).

        前提: y 已降到 0 (抓取位)。不移动任何电机。
        实测确认 (2026-08-02): SDK grasp(True) = pump.set(False)+valve.set(True),
        电平语义 = 气泵开 + 阀门关 → 建立真空保持吸住, 不一直开气泵.
        """
        return self.client.grasp(True, timeout=timeout or self.default_timeout_s)

    def drop_object(self, timeout: Optional[float] = None) -> dict:
        """释放 (抓取协议 2026-08-02: 只开阀门 → 断真空放下).

        实测确认 (2026-08-02): SDK grasp(False) = pump.set(True)+valve.set(False),
        电平语义 = 气泵关 + 阀门开 → 断开真空, 物体落下.
        """
        return self.client.grasp(False, timeout=timeout or self.default_timeout_s)

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
          - 大臂角度 arm_angle ∈ [+150, -150]°。
          - 手爪角度 hand ∈ [-90, +10]°。

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
                              timeout: float = 10.0,
                              setpoint_x_norm: Optional[float] = None,
                              setpoint_y_norm: Optional[float] = None,
                              **kwargs):
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
            setpoint_x_norm / setpoint_y_norm: 吸嘴偏移 setpoint（目标在吸嘴正下方时
                其 bbox 中心坐标）。默认 None → 自动读 origin 标定值；已标定即把目标
                对准吸嘴正下方，未标定保持对准画面中心。传 (0,0) 显式强制对准画面中心。
            **kwargs: 透传给 find_target (PID: kp/ki/kd; depth: target_real_height_m/focal_length_px)

        Returns:
            ServoResult（详见 VISION_SERVO_DESIGN.md）
        """
        kwargs = self._inject_setpoint(
            kwargs, self._resolve_nozzle_setpoint(
                setpoint_x_norm, setpoint_y_norm, label=selector.label))
        self.client.composite_run(
            arm=arm_angle, x_mm=x_mm, y_mm=y_mm, hand=hand, timeout=20.0,
        )
        return self.client._make_vision_with_move().find_target(
            selector, x_mm=x_mm, y_mm=y_mm,
            mm_per_norm=mm_per_norm, settle_tol_norm=settle_tol_norm,
            timeout=timeout, **kwargs,
        )

    def pick_by_vision(self, selector, *,
                       x_mm: float, y_mm: float, arm_angle: float = -90.0,
                       settle_tol_norm: float = 0.05,
                       timeout: float = 10.0,
                       lock_first: bool = True,
                       setpoint_x_norm: Optional[float] = None,
                       setpoint_y_norm: Optional[float] = None,
                       **kwargs) -> dict:
        """最高层：粗定位 → 视觉伺服 → composite_pick → grasp。

        业务前置：必须在 y < -30mm 保护区外。
        setpoint_x_norm / setpoint_y_norm: 吸嘴偏移 setpoint，默认 None → 读 origin 标定值。
        lock_first: 多目标场景锁定首个检测目标（默认 True）。
        **kwargs: 透传 find_target (PID/depth/4DOF 策略).
        """
        selector = self._maybe_lock_first(selector, lock_first)
        self.move_to_vision_target(
            selector, x_mm=x_mm, y_mm=y_mm,
            arm_angle=arm_angle, hand=-90.0,
            settle_tol_norm=settle_tol_norm, timeout=timeout,
            setpoint_x_norm=setpoint_x_norm, setpoint_y_norm=setpoint_y_norm,
            **kwargs,
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
                                        timeout: float = 10.0,
                                        setpoint_x_norm: Optional[float] = None,
                                        setpoint_y_norm: Optional[float] = None,
                                        **kwargs):
        """高层组合：composite_run 粗定位 → 视觉伺服（WS 实时推流）。

        setpoint_x_norm / setpoint_y_norm: 吸嘴偏移 setpoint，默认 None → 读 origin 标定值。
        **kwargs: 透传 find_target_realtime (PID/depth/4DOF).
        """
        kwargs = self._inject_setpoint(
            kwargs, self._resolve_nozzle_setpoint(
                setpoint_x_norm, setpoint_y_norm, label=selector.label))
        self.client.composite_run(
            arm=arm_angle, x_mm=x_mm, y_mm=y_mm, hand=hand, timeout=20.0,
        )
        return self.client._make_vision_with_move().find_target_realtime(
            selector, x_mm=x_mm, y_mm=y_mm,
            hz=hz, mm_per_norm=mm_per_norm,
            settle_tol_norm=settle_tol_norm, timeout=timeout, **kwargs,
        )

    def pick_by_vision_realtime(self, selector, *,
                                 x_mm: float, y_mm: float, arm_angle: float = -90.0,
                                 settle_tol_norm: float = 0.05,
                                 timeout: float = 10.0,
                                 lock_first: bool = True,
                                 setpoint_x_norm: Optional[float] = None,
                                 setpoint_y_norm: Optional[float] = None,
                                 **kwargs) -> dict:
        """最高层（实时版）：粗定位 → WS 伺服 → composite_pick → grasp.

        setpoint_x_norm / setpoint_y_norm: 吸嘴偏移 setpoint，默认 None → 读 origin 标定值。
        lock_first: 多目标场景锁定首个检测目标（默认 True）。
        **kwargs: 透传 find_target_realtime.
        """
        selector = self._maybe_lock_first(selector, lock_first)
        self.move_to_vision_target_realtime(
            selector, x_mm=x_mm, y_mm=y_mm,
            arm_angle=arm_angle, hand=-90.0,
            settle_tol_norm=settle_tol_norm, timeout=timeout,
            setpoint_x_norm=setpoint_x_norm, setpoint_y_norm=setpoint_y_norm,
            **kwargs,
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
                            timeout: float = 30.0,
                            setpoint_x_norm: Optional[float] = None,
                            setpoint_y_norm: Optional[float] = None,
                            **kwargs):
        """持续实时追踪（永不收敛停）：WS 推送驱动，timeout 后返回。

        setpoint_x_norm / setpoint_y_norm: 吸嘴偏移 setpoint，默认 None → 读 origin 标定值。
        **kwargs: 透传 find_target_track.
        """
        kwargs = self._inject_setpoint(
            kwargs, self._resolve_nozzle_setpoint(
                setpoint_x_norm, setpoint_y_norm, label=selector.label))
        self.client.composite_run(
            arm=arm_angle, x_mm=x_mm, y_mm=y_mm, hand=hand, timeout=20.0,
        )
        return self.client._make_vision_with_move().find_target_track(
            selector, x_mm=x_mm, y_mm=y_mm,
            hz=hz, mm_per_norm=mm_per_norm,
            timeout=timeout, **kwargs,
        )

    # ---- 2026-08-02: velocity 模式追踪 (07/08 封装, 免 arm_queue) ----

    def _set_arm_feed(self, *, stop: bool) -> None:
        """velocity 追踪前让位 / 后恢复 arm_feed (20Hz poll 会占串口).

        2026-08-03 优化: 改成 sync=False, 不阻塞等待 job 结束 ——
        start/stop_arm_feed 在 runtime 内部只是 toggle flag, 物理动作 0ms,
        之前 sync=True + poll_interval 0.5s 起步白白浪费 0.5s×2 = 1s。
        """
        try:
            if stop:
                self.client.http.execute(
                    "car", "stop_arm_feed", kwargs={"force": True},
                    sync=False, timeout=2.0)
            else:
                self.client.http.execute(
                    "car", "start_arm_feed", args=[20.0],
                    sync=False, timeout=2.0)
        except Exception as exc:
            logger.warning("_set_arm_feed(stop=%s) failed: %s", stop, exc)

    def track_velocity(self, label: str, *,
                       x_start: float = 0.0, y_start: float = -130.0,
                       arm_start: float = 0.0, hand_start: float = -90.0,
                       timeout: float = 30.0, hz: float = 20.0,
                       gain: float = 0.05, deadzone: float = 0.02,
                       max_vel: float = 0.15,
                       sign_x: float = -1.0, sign_y: float = 1.0,
                       no_reset: bool = False) -> "VelocityResult":
        """velocity XY 追踪 (示例 07): composite_run 起始位 → 让位 arm_feed →
        追踪 → 恢复 feed + 复位。返回 VelocityResult (hits/misses/elapsed/trace)。

        方向 x_vel=-dx·gain, y_vel=+dy·gain 已按真机实测固化; 异常方向用 sign_* 覆盖。
        """
        self.client.composite_run(arm=arm_start, x_mm=x_start, y_mm=y_start,
                                  hand=hand_start, timeout=20.0)
        try:
            self._set_arm_feed(stop=True)
            return self.client._make_vision_with_move().find_target_velocity(
                label, timeout=timeout, hz=hz, gain=gain, deadzone=deadzone,
                max_vel=max_vel, sign_x=sign_x, sign_y=sign_y)
        finally:
            self._set_arm_feed(stop=False)
            if not no_reset:
                self.client.composite_run(arm=arm_start, x_mm=x_start, y_mm=y_start,
                                          hand=hand_start, timeout=20.0)

    def track_4dof(self, label: str, *,
                   x_start: float = 0.0, y_start: float = -130.0,
                   arm_start: float = 0.0, hand_start: float = -90.0,
                   timeout: float = 30.0, hz: float = 20.0,
                   gain_x: float = 0.05, gain_y: float = 0.05,
                   gain_arm: float = 2.0, gain_hand: float = 2.0,
                   deadzone: float = 0.02, max_vel: float = 0.15,
                   no_reset: bool = False) -> "VelocityResult":
        """4-DOF velocity 追踪 (示例 08, 方向修正后): xy + 大臂 + 手抓 增量联调.

        角度目标从 arm_start/hand_start 起增量累加 (clamp 到 arm[-90,90]/hand[-90,0]),
        全部打包一发 /v1/realtime/arm-velocity。检测丢失 → xy 停, 角度保持。
        """
        self.client.composite_run(arm=arm_start, x_mm=x_start, y_mm=y_start,
                                  hand=hand_start, timeout=20.0)
        try:
            self._set_arm_feed(stop=True)
            return self.client._make_vision_with_move().find_target_4dof(
                label, timeout=timeout, hz=hz,
                gain_x=gain_x, gain_y=gain_y, gain_arm=gain_arm, gain_hand=gain_hand,
                deadzone=deadzone, max_vel=max_vel,
                arm_start=arm_start, hand_start=hand_start)
        finally:
            self._set_arm_feed(stop=False)
            if not no_reset:
                self.client.composite_run(arm=arm_start, x_mm=x_start, y_mm=y_start,
                                          hand=hand_start, timeout=20.0)

    def track_velocity_pick_4dof(self, label: str, *,
                                  x_start: float = -190.0, y_start: float = -180.0,
                                  arm_start: float = -85.0, hand_start: float = -25.0,
                                  grasp_y_mm: float = -20.0,
                                  grasp_x_offset_mm: float = 0.0,
                                  descend_hand_deg: Optional[float] = None,
                                  mode: str = "pick",
                                  timeout: float = 30.0, hz: float = 20.0,
                                  gain_x: float = 0.05, gain_y: float = 0.05,
                                  gain_arm: float = 2.0, gain_hand: float = 2.0,
                                  deadzone: float = 0.02, max_vel: float = 0.15,
                                  hold_y: bool = False,
                                  arm_min: Optional[float] = None,
                                  arm_max: Optional[float] = None,
                                  x_error_source: str = "dx",
                                  setpoint_x_norm: Optional[float] = None,
                                  setpoint_y_norm: Optional[float] = None,
                                  settle_hits: int = 3,
                                  lift_back: bool = True,
                                  two_pass: bool = False,
                                  pass2_y_mm: Optional[float] = None,
                                  pass2_gain_x: Optional[float] = None,
                                  pass2_gain_arm: Optional[float] = None,
                                  pass2_gain_hand: Optional[float] = None,
                                  pass2_max_vel: Optional[float] = None,
                                  pass2_timeout: Optional[float] = None) -> dict:
        """4-DOF 视觉对齐抓取: xy + 大臂 + 手抓 全轴联动 (2026-08-07, task6 蔬菜).

        task6 蔬菜抓取需求: 对齐阶段四轴都要动 (xy 十字 + 大臂 + 手抓), 不是
        track_velocity_pick 的"大臂控 cx + x 十字控 cy"两轴方案。

        对齐 (find_target_4dof): x_vel 控 dx + y_vel 控 dy + arm/hand 增量转向,
        hold_y=False → y 十字也动. 收敛 (末段 settle_hits 帧 |dx|,|dy|<deadzone)
        后 → y 降到 grasp_y + hand 转 descend_hand_deg (并发) → 真空吸 → 抬回.

        grasp_x_offset_mm (2026-08-08): 抓取下降时 X 向 0 方向移动这么多 mm;
        当前 X 已在 [-10,0] 则不动. 0 = 不额外移 X.

        检测丢失 → xy 停, 角度保持 (find_target_4dof 内部).

        Returns: 同 track_velocity_pick 的 dict {ok, reason, trace_hits, settled,
                 end_arm, end_hand, steps}.
        """
        self.client.composite_run(arm=arm_start, x_mm=x_start, y_mm=y_start,
                                  hand=hand_start, timeout=20.0)
        sp = self._resolve_nozzle_setpoint(setpoint_x_norm, setpoint_y_norm, label=label)
        sx, sy = (sp if sp else (0.0, 0.0))
        try:
            self._set_arm_feed(stop=True)
            result = self.client._make_vision_with_move().find_target_4dof(
                label, timeout=timeout, hz=hz,
                gain_x=gain_x, gain_y=gain_y, gain_arm=gain_arm, gain_hand=gain_hand,
                deadzone=deadzone, max_vel=max_vel,
                arm_start=arm_start, hand_start=hand_start,
                hold_y=hold_y,
                arm_min=arm_min, arm_max=arm_max,
                x_error_source=x_error_source,
                setpoint_x_norm=sx, setpoint_y_norm=sy,
            )
        finally:
            self._set_arm_feed(stop=False)

        trace_hits = result.hits

        def _converged(t) -> bool:
            return not t.miss and abs(t.dx) < deadzone and abs(t.dy) < deadzone

        settled = False
        tail = list(result.trace[-30:])
        for start in range(len(tail) - settle_hits + 1):
            if all(_converged(t) for t in tail[start:start + settle_hits]):
                settled = True
                break

        # 两阶段 (2026-08-08): 粗对齐后 Y 先降到 pass2_y_mm 再精对齐 (X 更慢)
        # 触发条件: 粗对齐**看到过目标** (hits>0) 就继续, 不完全收敛也走精对齐
        if two_pass and trace_hits > 0:
            p2x = pass2_gain_x if pass2_gain_x is not None else gain_x
            p2a = pass2_gain_arm if pass2_gain_arm is not None else gain_arm
            p2h = pass2_gain_hand if pass2_gain_hand is not None else gain_hand
            p2v = pass2_max_vel if pass2_max_vel is not None else max_vel
            p2t = pass2_timeout if pass2_timeout is not None else timeout
            # 粗对齐到位后 Y 先降 (靠近目标, 精对齐更准)
            if pass2_y_mm is not None:
                self.client.composite_run(y_mm=pass2_y_mm, speed=100)
                logger.info("  [两阶段] Y 降到 %.0fmm 再精对齐", pass2_y_mm)
            try:
                self._set_arm_feed(stop=True)
                result2 = self.client._make_vision_with_move().find_target_4dof(
                    label, timeout=p2t, hz=hz,
                    gain_x=p2x, gain_y=gain_y, gain_arm=p2a, gain_hand=p2h,
                    deadzone=deadzone, max_vel=p2v,
                    # 精对齐从粗对齐终点接着走, 不重置回起点 (2026-08-08 修)
                    arm_start=result.end_arm or arm_start,
                    hand_start=result.end_hand or hand_start,
                    hold_y=hold_y,
                    arm_min=arm_min, arm_max=arm_max,
                    x_error_source=x_error_source,
                    setpoint_x_norm=sx, setpoint_y_norm=sy,
                )
            finally:
                self._set_arm_feed(stop=False)
            p2_settled = False
            p2_tail = list(result2.trace[-30:])
            for start in range(len(p2_tail) - settle_hits + 1):
                if all(_converged(t) for t in p2_tail[start:start + settle_hits]):
                    p2_settled = True
                    break
            if p2_settled:
                logger.info("  [两阶段] 精对齐收敛 (pass2: gain_x=%.2f max_vel=%.2f)",
                            p2x, p2v)
                result = result2
                trace_hits = result.hits
            else:
                logger.info("  [两阶段] 精对齐未收敛, 用粗对齐结果")

        steps = {"settled": False, "lower": False, "suck": False, "lift": None}
        if not settled:
            return {"ok": False, "reason": "not_settled",
                    "trace_hits": trace_hits, "settled": False,
                    "end_arm": result.end_arm, "end_hand": result.end_hand, "steps": steps}
        steps["settled"] = True

        # 对齐完成 → y 降 grasp_y + hand 转 descend_hand (并发) → 吸气 → 抬回
        try:
            hand_param = float(descend_hand_deg) if descend_hand_deg is not None else None
            # 2026-08-08: grasp_x_offset_mm>0 → 抓取时 X 向 0 方向移 (已在 [-10,0] 不动)
            grasp_x = None
            if grasp_x_offset_mm and abs(grasp_x_offset_mm) > 1e-3:
                try:
                    cur_x = float(self.client.get_state().x_mm)
                    if not (-10.0 <= cur_x <= 0.0):
                        step = grasp_x_offset_mm * (1.0 if cur_x < 0 else -1.0)
                        grasp_x = (cur_x + step) / 1000.0
                        logger.info("    [诊断] grasp x偏移: cur_x=%.0fmm → target=%.0fmm",
                                    cur_x, cur_x + step)
                    else:
                        logger.info("    [诊断] grasp x偏移: cur_x=%.0fmm 已在[-10,0], 不动 X",
                                    cur_x)
                except Exception as exc:
                    logger.info("    [诊断] grasp x偏移: 读 x 失败 (%s), 不动 X", exc)
                    grasp_x = None
            job_down = self.client.http.execute(
                "arm", "composite_run",
                kwargs=dict(arm=None, x=grasp_x, y=float(grasp_y_mm) / 1000.0,
                            hand=hand_param, speed=100, timeout=5.0),
                sync=False,
            )
            jid_down = job_down.get("id") if isinstance(job_down, dict) else None
            if jid_down:
                self.client.http.wait_job(jid_down, timeout=5.0)
            steps["lower"] = True

            if mode == "drop":
                self.client.drop_object()
            else:
                self.client.grasp(True, timeout=5.0)
            steps["suck"] = True

            if lift_back:
                try:
                    self.client.http.execute(
                        "arm", "composite_run",
                        kwargs=dict(arm=None, x=None, y=float(y_start) / 1000.0,
                                    hand=None, speed=100, timeout=5.0),
                        sync=False,
                    )
                    steps["lift"] = True
                except Exception:
                    steps["lift"] = False
        except Exception as exc:
            try:
                self.client.http.execute(
                    "arm", "move_y_position",
                    kwargs=dict(target=float(y_start) / 1000.0, timeout=5.0),
                    sync=False,
                )
            except Exception:
                pass
            return {"ok": False, "reason": f"grasp_failed:{exc}",
                    "trace_hits": trace_hits, "settled": True,
                    "end_arm": result.end_arm, "end_hand": result.end_hand, "steps": steps}
        return {"ok": True, "reason": None, "trace_hits": trace_hits,
                "settled": True, "end_arm": result.end_arm, "end_hand": result.end_hand,
                "steps": steps}

    def track_velocity_pick(self, label: str, *,
                            x_start: float = 0.0, y_start: float = -180.0,
                            arm_start: float = -90.0, hand_start: float = 0.0,
                            grasp_y_mm: float = 0.0,
                            descend_hand_deg: Optional[float] = None,
                            mode: str = "pick",
                            timeout: float = 30.0, hz: float = 20.0,
                            gain_arm: float = 0.4, gain_x: float = 0.08,
                            deadzone: float = 0.02, max_vel: float = 0.15,
                            sign_arm: float = 1.0, sign_x: float = -1.0,
                            arm_min: Optional[float] = None,
                            arm_max: Optional[float] = None,
                            setpoint_x_norm: Optional[float] = None,
                            setpoint_y_norm: Optional[float] = None,
                            lock_first: bool = True,
                            settle_hits: int = 3,
                            hold_s: float = 0.5,
                            lift_back: bool = True,
                            no_reset: bool = False,
                            skip_pose_align: bool = False) -> dict:
        """智能定位抓取 (arm 控 cx + x 十字控 cy + 吸嘴 setpoint, 2026-08-02).

        2026-08-03 优化: skip_pose_align=True 跳过入口处的 composite_run ——
        调用方已经在 S 姿态后再调本方法 (例如 task1_seeding.py 的 (1.5) 步骤),
        重复跑同一个 S 姿态白白浪费 2-3s 物理时间。

        用户协议 (2026-08-02 实机标定):
          本机械结构在 y=-180 标定姿态下: 画面 cx ← 大臂角, 画面 cy ← x 十字位置.
          吸嘴中心 setpoint 在 y=-100 标定 (nozzle_offset_for(label));
          视觉追踪在 y=-180 开始, 大臂增量转把 cx 对准 setpoint_x, x 十字速度把
          cy 对准 setpoint_y (find_target_arm_cross)。y 十字锁 0, 手抓固定 0° 朝下。
          对齐 (末段连续 settle_hits 帧命中且 |dx|,|dy| < deadzone) 后 → y 降到 0 → 吸气。

        方向符号 (实机标定 2026-08-02):
          sign_arm=+1 (dx>0 → arm 更负), sign_x=-1 (dy>0 → x 往左).

        arm_min/arm_max (可选, 2026-08-03):
          覆盖 find_target_arm_cross 默认 [-150, 90] 的大臂 clamp 范围.
          任务二在 +95° 抓取 (超过默认 90), 必须传 arm_min/arm_max 让大臂
          能绕着抓取角增量微调; 不传保持旧行为 (默认 [-150, 90]).

        为什么用 velocity 模式:
          走 POST /v1/realtime/arm-velocity (免 arm_queue, 高频平滑).

        Returns:
            {"ok": bool, "reason": str|None, "trace_hits": int,
             "settled": bool, "lower": bool, "suck": bool, "lift": bool|None,
             "end_arm": float|None, "end_hand": float|None}
        """
        sp = self._resolve_nozzle_setpoint(setpoint_x_norm, setpoint_y_norm, label=label)
        sx, sy = (sp if sp else (0.0, 0.0))
        # 多目标场景: lock_first 默认选离画面中心 (吸嘴) 最近的目标并锁定 track_id,
        # 避免选到远处 marker (2026-08-02 实机多 marker 验证)
        selector = TargetSelector.for_label(
            label,
            strategy=SelectionStrategy.CLOSEST_TO_CENTER.value,
        )
        # 关闭 lock_first 时用 HIGHEST_SCORE (向后兼容)
        if not lock_first:
            selector = TargetSelector.for_label(
                label,
                strategy=SelectionStrategy.HIGHEST_SCORE.value,
            )
        # 2026-08-03 优化: S 姿态 -> 智能抓取 起步前的 composite_run 同步 HTTP
        # 至少 0.5s (poll_interval) + 实际物理动作 ~2s。改成 sync=False + wait_job,
        # 把 HTTP 同步开销压到只一次 poll, 而不是等 execute 整个返回。
        # skip_pose_align=True 时跳过 (调用方已在 S 姿态, 重复跑浪费 ~2-3s)。
        if not skip_pose_align:
            try:
                job = self.client.http.execute(
                    "arm", "composite_run",
                    kwargs=dict(arm=arm_start, x=float(x_start) / 1000.0,
                                y=float(y_start) / 1000.0, hand=hand_start),
                    sync=False,
                )
                job_id = job.get("id") if isinstance(job, dict) else None
                if job_id:
                    self.client.http.wait_job(job_id, timeout=5.0)
            except Exception:
                pass  # 退化到下面 servo, 失败也不致命
        # arm_min/arm_max 可选: 覆盖 find_target_arm_cross 默认 [-150, 90].
        # 任务二大臂在 +95° 抓取 (超过默认 arm_max=90), 不覆盖会把大臂钉死在 90.
        cross_kw = {}
        if arm_min is not None:
            cross_kw["arm_min"] = arm_min
        if arm_max is not None:
            cross_kw["arm_max"] = arm_max
        try:
            self._set_arm_feed(stop=True)
            result = self.client._make_vision_with_move().find_target_arm_cross(
                label, timeout=timeout, hz=hz,
                gain_arm=gain_arm, gain_x=gain_x,
                deadzone=deadzone, max_vel=max_vel,
                arm_start=arm_start,
                sign_arm=sign_arm, sign_x=sign_x,
                setpoint_x_norm=sx, setpoint_y_norm=sy,
                selector=selector, **cross_kw)
        finally:
            self._set_arm_feed(stop=False)

        trace_hits = result.hits
        # 对齐判定 (2026-08-03): 用户诉求 — "已经识别到了就应该走 grasp",
        # 不要因 cx/cy 末段 1-2 帧抖动卡死 not_settled。
        # 判定逻辑降级:
        #   1. 主路径: trace 末段窗口里有 settle_hits 帧全收敛 (cx/cy 都 < deadzone)
        #   2. 兜底:   trace 整体有 >=1 hit 且 cx/cy 末段最近值都在 2*deadzone 内
        #              (即未完全收敛但已经"看得见在 setpoint 附近", 可放 grasp)
        # 只有以上两条都失败才返回 not_settled (此时确实丢目标)。
        def _converged(t) -> bool:
            return not t.miss and abs(t.dx) < deadzone and abs(t.dy) < deadzone

        settled = False
        tail = list(result.trace[-30:])
        for start in range(len(tail) - settle_hits + 1):
            window = tail[start:start + settle_hits]
            if all(_converged(t) for t in window):
                settled = True
                break

        if not settled and trace_hits > 0:
            # 兜底: 找 trace 末段最后一帧未 miss 且 cx/cy 都在 2*deadzone 内
            loose_zone = 2.0 * deadzone
            for t in reversed(tail):
                if t.miss:
                    continue
                if abs(t.dx) < loose_zone and abs(t.dy) < loose_zone:
                    settled = True
                    logger.warning(
                        "track_velocity_pick: 主判定 not_settled 但末段"
                        " dx=%.3f dy=%.3f 在 2*deadzone=%.3f 内, 放 grasp",
                        t.dx, t.dy, loose_zone,
                    )
                    break

        steps = {"settled": False, "lower": False, "suck": False, "lift": None}
        if not settled:
            return {"ok": False, "reason": "not_settled",
                    "trace_hits": trace_hits, "settled": False,
                    "end_arm": result.end_arm, "end_hand": result.end_hand, "steps": steps}
        steps["settled"] = True

        # 对齐完成 → y 降 0 → mode=pick 吸气 / mode=drop 释放
        # 用户 00:45: 缩短 timeout, 吸住后立即抬离, 低延迟!
        # 2026-08-06: hand 转 0° + y 下降并发 (1 次 composite_run 4 轴), 不等手爪到位
        # 串行 set_hand_angle(~0.3-0.5s) + move_y(~0.3s) → 并发只等 move_y ~0.3s.
        # 用户 (2026-08-06) 实测: 手爪 -10°→0° 时 Y 同时降, 吸嘴在最低点前已经 0°
        # (因为 Y 物理 ~0.3s << 手爪 ~0.5s), 安全.
        # 删除所有 sleep: hold_s 直接 0, vacuum_settle_s 走 cfg 0.
        try:
            # 1) hand + y 并发 (1 次 composite_run 4 轴: hand + y)
            target_m = float(grasp_y_mm) / 1000.0
            logger.info("track_velocity_pick: 开始 grasp 段, mode=%s hand+y 并发 (move_y=%.0fmm=%.4fm)",
                        mode, grasp_y_mm, target_m)
            hand_param = float(descend_hand_deg) if descend_hand_deg is not None else None
            job_y_down = self.client.http.execute(
                "arm", "composite_run",
                kwargs=dict(
                    arm=None, x=None,
                    y=target_m,
                    hand=hand_param,
                    speed=100, timeout=5.0,
                ),
                sync=False,
            )
            jid_y_down = job_y_down.get("id") if isinstance(job_y_down, dict) else None
            logger.info("  hand+y 并发 job_id=%s", jid_y_down)

            # 等 y 到位 (grasp 必须等 y 到位才能开阀门, 否则吸嘴没到方块就开阀门)
            if jid_y_down:
                self.client.http.wait_job(jid_y_down, timeout=5.0)
            steps["lower"] = True

            # 2) grasp/drop 是电平动作, ~100ms 即完成
            if mode == "drop":
                logger.info("  drop_object()")
                self.client.drop_object()
                steps["suck"] = True
            else:
                logger.info("  grasp(True)")
                self.client.grasp(True, timeout=5.0)
                steps["suck"] = True

            # 删除 hold_s sleep (用户 2026-08-06: grasp 后立即下一步, 不要停顿)

            # 3) 抬回 y_start (lift_back) — 用户 2026-08-06: 不要停顿, fire-and-forget
            # wait_job 不阻塞返回, 下游立刻可继续 X 移动.
            if lift_back:
                try:
                    target_m_up = float(y_start) / 1000.0
                    logger.info("  move_y(%.4fm) 抬回 fire-and-forget", target_m_up)
                    job_lift = self.client.http.execute(
                        "arm", "composite_run",
                        kwargs=dict(arm=None, x=None, y=target_m_up,
                                    hand=None, speed=100, timeout=5.0),
                        sync=False,
                    )
                    jid_lift = job_lift.get("id") if isinstance(job_lift, dict) else None
                    # fire-and-forget: 不 wait_job, 立即返回. 下游 X 移动并发.
                    steps["lift"] = True
                except Exception:
                    steps["lift"] = False
        except Exception as exc:
            # 2026-08-03 修复: grasp 段失败时先抬回 y_safe, 否则吸嘴留在 y=0 + 真空开着,
            # 主循环走兜底底盘移动会把物体拖走 (用户观察到的"抓起来之后往前走一小段")。
            try:
                safe_y_m = float(y_start) / 1000.0  # -100mm = 安全高度
                self.client.http.execute(
                    "arm", "move_y_position",
                    kwargs=dict(target=safe_y_m, timeout=5.0),
                    sync=False,
                )
            except Exception:
                pass
            return {"ok": False, "reason": f"grasp_failed:{exc}",
                    "trace_hits": trace_hits, "settled": True,
                    "end_arm": result.end_arm, "end_hand": result.end_hand, "steps": steps}
        return {"ok": True, "reason": None, "trace_hits": trace_hits,
                "settled": True, "end_arm": result.end_arm, "end_hand": result.end_hand,
                "steps": steps}

    def pick_by_vision_lower(self, selector, *,
                             x_mm: float, y_mm: float,
                             grasp_y_mm: float = 0.0,
                             arm_angle: float = -90.0, hand: float = 0.0,
                             mm_per_norm: float = 30.0,
                             settle_tol_norm: float = 0.05,
                             timeout: float = 10.0,
                             hold_s: float = 0.4,
                             lift_back: bool = True,
                             lock_first: bool = True,
                             reposition: bool = True,
                             align: bool = True,
                             setpoint_x_norm: Optional[float] = None,
                             setpoint_y_norm: Optional[float] = None,
                             **kwargs) -> dict:
        """识别 → 视觉伺服对准(吸嘴setpoint) → y 下降 lower_mm → 抓取.

        用户约定 (2026-08-01): 识别到目标后直接 y 下降 80mm 去抓;
        中途丢失目标没关系 —— 下降是开环的, 基于对准后的位置。

        流程:
          1. composite_run 粗定位 (arm/hand 姿态 + xy); reposition=False 时跳过,
             直接用当前位姿当伺服起点 (x_mm/y_mm 参数被当前实际位置覆盖)
          2. find_target 视觉伺服, 把目标对准吸嘴 setpoint (origin 注入);
             默认 lock_first=True → 锁定首个检测目标 (多目标场景防来回跳)
             align=False 时跳过伺服, 直接基于当前位姿下降抓 (目标已大致对准,
             大臂略偏不影响吸住 —— 用户约定 2026-08-01)
          3. move_y 降到 grasp_y_mm (默认 0 = 抓取位; 协议 2026-08-01: y 降到 0 才能吸;
             开环下降, 中途丢目标没关系), 下降后验证到位才吸气
          4. 吸气 (关闭阀门+关闭气泵 = grasp(True)), hold hold_s
          5. 可选 lift_back: 回到下降前 y (离开保护区, 方便后续移动)

        安全: 下降进入 y∈[-30,0] 保护区, 但 move_y 只查 _check_safe、grasp 不动电机,
        都不触发 _check_y_protected; 抬回同样安全。若期间想动大臂/手爪会被安全门拒绝。

        Returns:
            {"ok": bool, "reason": str|None, "servo": ServoResult|None,
             "steps": {"servo": bool, "lower": bool, "grasp": bool, "lift": bool|None},
             "y_before": float, "y_lower": float}
        """
        selector = self._maybe_lock_first(selector, lock_first)
        kwargs = self._inject_setpoint(
            kwargs, self._resolve_nozzle_setpoint(
                setpoint_x_norm, setpoint_y_norm, label=selector.label))
        # 1. 粗定位
        if reposition:
            self.client.composite_run(
                arm=arm_angle, x_mm=x_mm, y_mm=y_mm, hand=hand, timeout=20.0,
            )
        else:
            st = self.client.get_state()
            x_mm, y_mm = st.x_mm, st.y_mm
        # 2. 视觉伺服对准 (吸嘴 setpoint); align=False 跳过
        servo = None
        if align:
            try:
                servo = self.client._make_vision_with_move().find_target(
                    selector, x_mm=x_mm, y_mm=y_mm,
                    mm_per_norm=mm_per_norm, settle_tol_norm=settle_tol_norm,
                    timeout=timeout, **kwargs,
                )
            except RuntimeError as exc:
                return {"ok": False, "reason": f"servo_miss:{exc}", "servo": None,
                        "steps": {"servo": False, "lower": False, "grasp": False,
                                  "lift": None},
                        "y_before": None, "y_lower": None}
            if not servo.converged:
                return {"ok": False, "reason": "servo_not_converged", "servo": servo,
                        "steps": {"servo": False, "lower": False, "grasp": False,
                                  "lift": None},
                        "y_before": None, "y_lower": None}
        # 3. 降到 grasp_y_mm (协议: y 降到 0 才能吸; 开环下降, 中途丢目标没关系)
        st = self.client.get_state()
        y_before = st.y_mm
        self.client.move_y(grasp_y_mm, timeout=20.0)
        # 验证到位 (协议强制: 未降到目标 y 不吸气)
        st = self.client.get_state()
        y_err = abs(st.y_mm - grasp_y_mm)
        if y_err > 10.0:
            return {"ok": False, "reason": f"y未到位 err={y_err:.1f}mm",
                    "servo": servo,
                    "steps": {"servo": bool(servo is not None), "lower": False,
                              "grasp": False, "lift": None},
                    "y_before": y_before, "y_lower": grasp_y_mm}
        # 4. 吸气 (关闭阀门+关闭气泵)
        time.sleep(hold_s)
        grasp_job = self.suck(timeout=10.0)
        # 5. 抬回 (离开保护区)
        lift_job = None
        if lift_back:
            lift_job = self.client.move_y(y_before, timeout=15.0)
        return {
            "ok": bool(grasp_job.get("ok")),
            "reason": None,
            "servo": servo,
            "steps": {"servo": bool(servo is not None), "lower": True,
                      "grasp": bool(grasp_job.get("ok")),
                      "lift": bool(lift_job and lift_job.get("status") == "succeeded")
                               if lift_job else None},
            "y_before": y_before, "y_lower": grasp_y_mm,
        }
