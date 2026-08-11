#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
机械臂控制模块

该模块实现了机械臂的运动控制, 包括竖直方向、水平方向的移动, 以及手部的控制。
"""

import math
import time
import numpy as np
import yaml
import os
import sys
from typing import Union
from concurrent.futures import ThreadPoolExecutor, as_completed

# 添加上本地目录
dir_this = os.path.abspath(os.path.dirname(__file__))
sys.path.append(dir_this)
# 添加上两层目录
dir_root = os.path.abspath(os.path.join(dir_this, '..', '..'))
sys.path.append(dir_root)

# 导入自定义模块
from ...tools import get_yaml, limit_val, CountRecord, PID, logger
from .. import (
    AnalogInput, MotorWrap, Key4Btn, ServoPwm,
    ServoBus, StepperWrap, PoutD
)

# 常量定义



POSITION_ERROR_THRESHOLD = 1.5e-3 # 位置误差阈值 (2026-08-02 调大: 滑轨清理后摩擦小, 原 0.4mm 阈值 bang-bang 反复触发抖动; 1.5mm 平衡精度和稳定性)
STOP_CHECK_THRESHOLD = 1e-10 # 停止检查阈值

# reset_y 成功触底后,机械臂自动升到的目标位置 (m)。负值 = 向上,远离磁感。
# 设为 None 表示"触底后停在 0 位不动"(历史默认);设为 -0.15 表示升到磁感上方 150mm。
# reset_position / reset_all 流程会以这个值作为机械臂的"复位后休息位",避免
# "reset_y 完还在最低位,业务第一步 move_y 还要再走一次"。
POST_RESET_TARGET_M = -0.15  # 触底归零后自动走到 -150mm

# 2026-08-01 防伪触发: 磁感是模拟量(无滤波), 可能在非底部位置被噪声误判为触底,
# 把 y_pose_start 覆盖成错误编码值 -> 位置基准漂移 -> 业务 move_y 反复来回走。
# 真触底时电机被磁感物理挡死, 触发瞬间到 dwell 通过(50ms) 编码器位移 < 1mm;
# 噪声伪触发时电机仍在移动, 位移 > 容差。用此容差区分真伪。
Y_RESET_REF_STABILITY_TOL_M = 0.005  # 触发瞬间 ~ dwell 通过, 编码器最大允许位移 (m)
# reset_y 会话内一致性: 本次成功 ref 与上次 (self._y_ref_encoder_at_zero) 偏差超过
# 此值视为伪触发/失步, 拒绝更新基准 (真触底物理位置固定, 多次 reset 应回到同一 ref)。
Y_RESET_REF_CONSISTENCY_TOL_M = 0.02  # 与上次成功触底 ref 的最大偏差 (m)


def get_path_relative(*args):
    """
    获取相对路径

    Args:
        *args: 路径组件

    Returns:
        str: 完整的绝对路径
    """
    local_dir = os.path.abspath(os.path.dirname(__file__))
    return os.path.join(local_dir, *args)


class ArmController:
    """
    机械臂控制类, 负责机械臂的运动控制和状态管理

    Attributes:
        config: 配置参数
        motor_y: 竖直方向步进电机
        motor_x: 水平方向电机
        hand_servo: 手部舵机
        arm_servo: 手臂舵机
        pump: 气泵控制
        valve: 阀门控制
        y_pose_now: 当前竖直位置
        x_pose_now: 当前水平位置
        side: 机械臂方向
    """

    def __init__(self) -> None:
        """
        初始化机械臂控制类
        """
        self.yaml_path = get_path_relative("arm_cfg.yaml")

        with open(self.yaml_path, 'r') as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)

        
        '''机械臂的长度'''
        self.arm_length: float = self.config["arm_length"]
        # 初始化各部分参数
        self.y_params_init(**self.config["vert_cfg"])
        self.x_params_init(**self.config["horiz_cfg"])
        self.hand_params_init(**self.config["hand_cfg"])
        self.position_params_init()
        # 2026-08-03：协作取消接线。runtime cancel_job / emergency_stop 会置
        # car._stop_flag=True 与 car._estop_event.set()（见 my_car/__init__.py
        # 的 self.arm._estop = self._estop_event）。长 PID 循环每帧查本方法，
        # 取消后不再"自然跑完"（旧 README 记载的已知限制）。
        self._stop_flag = False
        # 2026-08-03：runtime 会把这个 provider 指到 car._stop_flag（见
        # my_car/__init__.py）。_must_stop 优先查 provider，独立运行时退回
        # 自身 _stop_flag 属性。
        self._stop_flag_provider = None

    def _must_stop(self):
        """协作停止检查：急停（_estop）或任务取消（_stop_flag）任一命中 → True。
        所有运动循环每帧调用；命中方必须停车后退出。"""
        estop = getattr(self, "_estop", None)
        if estop is not None and estop.is_set():
            return True
        provider = getattr(self, "_stop_flag_provider", None)
        if provider is not None:
            try:
                return bool(provider())
            except Exception:
                return bool(getattr(self, "_stop_flag", False))
        return bool(getattr(self, "_stop_flag", False))


    def y_params_init(self, motor, limit_port, pid, threshold,
                      slow_band_m=0.015, slow_velocity=0.02,
                      top_slow_m=0.020, top_slow_velocity=0.03, **_extra):
        """
        初始化竖直方向电机参数。

        接受 slow_band_m/slow_velocity/top_slow_m/top_slow_velocity 等新参数（默认 0.015/0.02/0.02/0.03）。
        新参数仅用于 y_speed 的分段限幅；旧配置文件里没有这些键时走默认。
        _extra 吸收未来新增键,避免 **vert_cfg unpack 时 TypeError。
        """
        self.motor_y = StepperWrap(**motor)
        self.y_limit_sensor = AnalogInput(limit_port)

        self.y_pose_start = self.motor_y.get_dis()
        self.y_pose_now = 0
        self.y_pid = PID(**pid)
        self.y_velocity_limit = pid['output_limits']
        self.y_distance_change = 0
        self.y_threshold = threshold  # 竖直位置阈值
        self.y_pose_last = 0

        self.y_pid_flag = CountRecord(5)
        self.y_stop_flag = CountRecord(10)
        # 末段减速带：距磁感触发前 slow_band_m 米内，PWM 限幅降到 slow_velocity。
        # 解决"接近磁感时 PID 自然减速 → 编码器不动 → y_stop_check 误判堵转 → reset_y 假到底"的 bug。
        self.y_slow_band_m = float(slow_band_m)
        self.y_slow_velocity = float(slow_velocity)
        # 顶段减速带：|y| > top_slow_m (绝对值) 时，PWM 限幅降到 top_slow_velocity。
        # 顶部是机械硬限位,无传感器,降低失步概率。
        self.y_top_slow_m = float(top_slow_m)
        self.y_top_slow_velocity = float(top_slow_velocity)
        # 丢步核对：reset_y 后记录 ref_encoder，move_y_position 完成后用编码器核对总位移
        self._y_ref_encoder_at_zero = None
        self._y_expected_total_delta = 0.0
        # seek 模式：True 时 y_speed 磁感安全门对正速度放行(允许穿入磁感),
        # 因为 reset_y 必须真正压到磁感才算成功,磁感门挡了它就停不下来了
        self._y_seeking_bottom = False

    def y_reset_check(self):
        """
        检查竖直方向是否到达限位

        Returns:
            bool: 是否到达限位
        """
        return self.y_limit_sensor.read() > 1000  # 磁敏传感器的值大于1000时, 则认为到达限位位置

    def y_stop_check(self):
        """
        检查竖直方向是否停止

        Returns:
            bool: 是否停止
        """
        return self.y_stop_flag(
            abs(self.y_distance_change) < STOP_CHECK_THRESHOLD
        )
    def y_get_position(self):
        self.y_pose_now = (
            self.motor_y.get_dis() - self.y_pose_start
        )
        return self.y_pose_now

    def y_pid_moveto(self, target_pose):
        """
        使用PID控制竖直方向移动

        Args:
            target_pose: 目标位置 (单位: m)

        Returns:
            bool: 是否到达目标位置
        """
        # 记录当前位置, 并更新上次的位置
        self.y_pose_now = (
            self.motor_y.get_dis() - self.y_pose_start
        )
        self.y_distance_change = (
            self.y_pose_now - self.y_pose_last
        )
        self.y_pose_last = self.y_pose_now

        error = target_pose - self.y_pose_now

        # 收敛死区闩锁 (2026-08-09): |err| 一进入阈值内立即停发微速度, 让轴静置,
        # 连续 5 帧自然凑满收敛. 否则欠阻尼 PID (Kp=6, Kd=1.0 → ζ≈0.2) 在 setpoint
        # 附近持续下发微速度, 位置绕目标做极限环 (高电压扭矩大时更明显),
        # "连续 5 帧 |err|<1.5mm" 永远凑不满 → move_y_position 挂死 → 上层
        # wait_job 超时. 停发后轴停在阈值内, 精度与旧"到位判定"一致 (同为 1.5mm).
        if abs(error) < POSITION_ERROR_THRESHOLD:
            velocity = 0.0
        else:
            velocity = self.y_pid(self.y_pose_now)

        self.y_speed(velocity)

        if self.y_pid_flag(abs(error) < POSITION_ERROR_THRESHOLD):
            return True
        else:
            return False

    def reset_y(self):
        """
        重置竖直方向位置：朝磁感方向下压找触底，【磁感触发】是唯一成功凭证。
        触发磁感归零后,自动升到 [arm_base.POST_RESET_TARGET_M](./arm_base.py)
        (默认 -150mm) 收尾（避免 reset 完机械臂还在最低位,业务第一步 move_y
        还要再走一次浪费行程）。

        方向约定（实测）：setpoint>0/velocity>0 = 向下（朝磁感）。

        三段速度曲线，避免 PID 接近时减速被 y_stop_check 误判为堵转：
          1) 远段 (y < -slow_band - top_slow)：SLOW_VELOCITY 0.08 m/s 直驱；
          2) 末段 (y >= -slow_band 且 y < 0)：SLOW_VELOCITY 0.02 m/s 极慢贴底；
          3) 触底 (y_reset_check() 真触发)：保持 0.05s dwell 确认不是抖动，立即归零，
             然后调 move_y_position(POST_RESET_TARGET_M) 升到 -150mm。
        找底期间 y_speed 的磁感门放行（_y_seeking_bottom=True），允许正速度穿入磁感。

        退出条件：
          - 成功：磁感触发后 dwell 通过 → 编码器 ref 记录 + y_pose_start 重置 +
                  move_y_position(POST_RESET_TARGET_M) 收尾 → True；
          - 失败：超时（10s）未触发磁感 → 强制停车 + 报警 + False（**绝不**伪归零）；
          - 急停：_estop 置位 → 立即退出 + False。

        返回：是否成功（bool）。失败时 **不** 更新 y_pose_start，y_pose_now 保持搜索前值，
        后续 move_y_position 会发现偏差并 warn。
        """
        # === 配置 ===
        FAST_VELOCITY = 0.08    # 远段快速接近速度
        SLOW_VELOCITY = self.y_slow_velocity   # 末段贴底速度（0.02 m/s 默认）
        DWELL_TIME = 0.05       # 磁感触发后确认 dwell（秒）
        SEEK_TIMEOUT = 10.0     # 总找底超时
        slow_band = self.y_slow_band_m
        # 触底归零后收尾目标 (m), None 表示停在 0 位
        post_target = POST_RESET_TARGET_M

        # 入口前先把 _y_seeking_bottom 设 True，让 y_speed 对正速度放行
        self._y_seeking_bottom = True
        start = time.time()
        triggered_at = None
        prev_pos = self.y_get_position()
        no_move_since = time.time()   # 编码器持续不动的最早时刻
        NO_MOVE_HARD_TIMEOUT = 2.0    # 长时间不动 → 强制停车报警

        try:
            while True:
                # 1) 急停 / 取消优先
                if self._must_stop():
                    logger.warning("reset_y: 收到急停/取消，中止找底")
                    break
                # 2) 磁感触发 → 记录 dwell 起点
                if self.y_reset_check():
                    if triggered_at is None:
                        triggered_at = time.time()
                        # 记录磁感触发瞬间的编码器位置
                        trigger_ref = self.motor_y.get_dis()
                    elif time.time() - triggered_at >= DWELL_TIME:
                        # 成功！dwell 通过
                        ref = self.motor_y.get_dis()
                        # 2026-08-01 防伪触发(1): 真触底电机被磁感挡死, 触发瞬间到
                        # dwell 通过编码器位移 ~0; 噪声伪触发时电机仍在动, 位移 > 容差。
                        if abs(ref - trigger_ref) > Y_RESET_REF_STABILITY_TOL_M:
                            logger.error(
                                "reset_y: dwell 通过但触发至通过位移 %.1fmm > 容差 %.1fmm, "
                                "疑似磁感噪声伪触发, 拒绝更新 y_pose_start, 继续找底"
                                % (abs(ref - trigger_ref) * 1000.0,
                                   Y_RESET_REF_STABILITY_TOL_M * 1000.0)
                            )
                            triggered_at = None
                            time.sleep(0.01)
                            continue
                        # 2026-08-01 防伪触发(2): 会话内一致性, 与上次成功触底 ref 偏差
                        # 过大视为伪触发/失步, 拒绝覆盖基准 (真触底物理位置固定)。
                        last_ref = getattr(self, "_y_ref_encoder_at_zero", None)
                        if last_ref is not None and \
                                abs(ref - last_ref) > Y_RESET_REF_CONSISTENCY_TOL_M:
                            logger.error(
                                "reset_y: ref=%.4f 与上次成功触底 %.4f 偏差 %.1fmm > 容差 %.1fmm, "
                                "疑似基准漂移/伪触发, 拒绝更新 y_pose_start, 继续找底"
                                % (ref, last_ref, abs(ref - last_ref) * 1000.0,
                                   Y_RESET_REF_CONSISTENCY_TOL_M * 1000.0)
                            )
                            triggered_at = None
                            time.sleep(0.01)
                            continue
                        self.y_pose_start = ref
                        self.y_pose_now = 0
                        self._y_ref_encoder_at_zero = ref
                        self._y_expected_total_delta = 0.0
                        logger.info(
                            "reset_y: 磁感触发+dwell通过,ref_encoder=%.6f,耗时%.2fs"
                            % (ref, time.time() - start)
                        )
                        self.y_speed(0)
                        # 退出 seek 模式后再走收尾位移,避免 move_y_position 期间
                        # _y_seeking_bottom=True 让 y_speed 磁感门失效误推
                        self._y_seeking_bottom = False
                        # 收尾：触底归零后走到 POST_RESET_TARGET_M (默认 -150mm)
                        if post_target is not None:
                            try:
                                self.move_y_position(post_target)
                                logger.info(
                                    "reset_y: 收尾完成,y=%.1fmm"
                                    % (self.y_get_position() * 1000.0)
                                )
                                return True
                            except Exception as exc:
                                logger.error(
                                    "reset_y: 收尾走到 %.0fmm 异常: %s, "
                                    "已归零但未升到 -150mm"
                                    % (post_target * 1000.0, exc)
                                )
                                return False
                        # post_target=None 兼容老语义:停在 0 位
                        return True
                    # dwell 中,维持贴底慢速
                    self.motor_y.set_velocity(0)
                    time.sleep(0.01)
                    continue
                # 还没触发：按当前 y 选档。reset_y 永远往下找底，不走顶段减速分支。
                cur = self.y_get_position()
                # 失步/卡死保护：连续 2s 编码器不动 → 强制停车报警
                if abs(cur - prev_pos) < 1e-5:
                    if time.time() - no_move_since > NO_MOVE_HARD_TIMEOUT:
                        logger.error(
                            "reset_y: 编码器持续 %.1fs 不动, 疑似失步/卡死, 强制停车" %
                            NO_MOVE_HARD_TIMEOUT
                        )
                        break
                else:
                    no_move_since = time.time()
                    prev_pos = cur
                # 速度档位：reset_y 永远正向（向下）找底。
                # 末段（cur >= -slow_band）：极慢贴底；中/远段：快速下压。
                # 【绝不】走顶段减速分支（那是给 move_y_position 往上走用的），
                # 否则机械臂已在 -120mm 时会被错误赋负速度 = 向上 = 撞顶。
                if cur >= -slow_band:
                    v = SLOW_VELOCITY  # 末段：极慢贴底
                else:
                    v = FAST_VELOCITY  # 中/远段
                # 直接走 y_speed：它会按当前 cur 重新选末段限幅 + limit_val
                self.y_speed(v)
                time.sleep(0.01)
                # 总超时
                if time.time() - start > SEEK_TIMEOUT:
                    logger.error("reset_y: 找底 %.1fs 超时未触发磁感, 强制停车" % SEEK_TIMEOUT)
                    break
        finally:
            self._y_seeking_bottom = False
            self.y_speed(0)  # 必停
        return False

    def move_y_position(self, target):
        """
        移动竖直方向指定距离。带软限位 + 丢步核对 + 兜底：
        1) 入口：soft_y_max 软限位 limit_val(基于 arm_origin.soft_y_max_m,默认 0.18m)；
        2) 命令位移记录：本次指令 delta = target - current，记录累积预期位移；
        3) 第一轮 PID 闭环到 < 1mm（或堵转跳出）；
        4) 完成后比对 actual vs target，偏差 > 1mm 时再发一轮 setpoint，最多 2 轮；
        5) 命令/编码器核对：|编码器 delta - 累积预期| > STEP_LOSS_TOL_M（默认 0.005m）→ 报警；
        6) 若仍偏差 > 2mm（步距）则视为异常，仅 warn 不抛错。

        方向约定：target < 0 = 向上, target > 0 = 向下；软区间 [-soft_y_max_m, 0]。
        """
        # 1) 软限位（用 self.y_threshold，但只信任负向区间；若 y_threshold 配置为 [0, 0.2] 错误,
        #    则自动回退到 [-0.18, 0]）
        if self.y_threshold[0] >= 0 and self.y_threshold[1] > 0:
            # 配置错误（[0, 0.2] 这种）,回退默认
            y_lo, y_hi = -0.18, 0.0
        else:
            y_lo, y_hi = self.y_threshold[0], self.y_threshold[1]
        target = limit_val(target, y_lo, y_hi)
        # 2) 命令位移记录
        prev_pos = self.y_get_position()
        # 2026-08-01 防基准污染: 当前 y_pose_now 超出软限位区间外 150mm 视为基准可疑
        # (y_pose_start 被磁感噪声伪触发覆盖), 拒绝执行, 避免在污染基准上无意义运动放大振荡。
        if prev_pos < y_lo - 0.15 or prev_pos > y_hi + 0.15:
            logger.error(
                "move_y_position: 当前 y_pose_now=%.1fmm 超出合理区间 [%.1f,%.1f]mm, "
                "疑似 y_pose_start 被污染, 拒绝执行 (请先 reset_y 校准)"
                % (prev_pos * 1000.0, (y_lo - 0.15) * 1000.0, (y_hi + 0.15) * 1000.0)
            )
            return False
        self._y_expected_total_delta += abs(target - prev_pos)
        # 第一轮（保持原行为）
        self.y_pid.setpoint = target
        while True:
            if self._must_stop():
                logger.info(f"move_y_position: 急停/取消,中止移动到{target}")
                break
            if self.y_pid_moveto(target):
                logger.info(f"移动到高度{target}（PID 收敛）")
                break
            if self.y_stop_check():
                logger.info(f"移到高度{target}过程中检测到停止")
                break
        self.y_speed(0)

        # ---- 丢步/堵转补偿（最多 2 轮） ----
        # 2026-08-01 修复: 兜底阈值 0.001 (1mm) → 0.002 (2mm),与最终误差阈值
        # (line 379 的 0.002 = 2mm) 对齐。原 1mm 阈值在首轮 PID 末段 (Kp=6,误差 0.5mm
        # 对应 0.003 m/s 速度) 会被频繁触发,再发一轮 setpoint 反而放大振荡。
        # 改成 2mm 后只对真物理丢步生效。
        for round_idx in range(2):
            actual = self.y_get_position()
            err = target - actual
            if abs(err) <= 0.002:
                break
            # 2026-08-01 防放大振荡: 未经 reset_y 校准 (基准不可信) 时不做兜底,
            # 否则两轮 setpoint 在漂移基准上互相抵消, 越兜越摆。
            if self._y_ref_encoder_at_zero is None:
                logger.warning(
                    "move_y_position: 未经 reset_y 校准 (ref=None), 跳过丢步兜底, "
                    "err=%.1fmm" % (err * 1000.0)
                )
                break
            # 磁感已触发且 setpoint 已在触底方向 → 已经触底到位
            if self.y_reset_check() and target >= 0.0:
                break
            logger.warning(
                f"move_y_position 丢步兜底 round={round_idx}: "
                f"target={target:.4f} actual={actual:.4f} err={err*1000:.1f}mm, 再发一次"
            )
            self.y_pid.setpoint = target
            while True:
                if self._must_stop():
                    logger.info(f"move_y_position 丢步兜底 round={round_idx}: 急停/取消，中止")
                    break
                if self.y_pid_moveto(target):
                    break
                if self.y_stop_check():
                    break
            self.y_speed(0)

        # ---- 命令/编码器核对（仅在已知 ref 时） ----
        if self._y_ref_encoder_at_zero is not None:
            actual_disp = abs(self.motor_y.get_dis() - self._y_ref_encoder_at_zero)
            disp_err = abs(actual_disp - self._y_expected_total_delta)
            STEP_LOSS_TOL_M = 0.005
            if disp_err > STEP_LOSS_TOL_M:
                logger.warning(
                    f"move_y_position 疑似丢步: 累积预期={self._y_expected_total_delta*1000:.1f}mm "
                    f"编码器={actual_disp*1000:.1f}mm 偏差={disp_err*1000:.1f}mm, 建议重置原点"
                )

        final = self.y_get_position()
        if abs(final - target) > 0.002:
            logger.error(
                f"move_y_position 丢步严重: target={target:.4f} final={final:.4f} "
                f"diff={(final-target)*1000:.1f}mm, 建议重新定原点"
            )
        # 完成后把本次 delta 累加确认（actual vs prev_pos）
        self._y_expected_total_delta += abs(final - prev_pos) - abs(target - prev_pos)

    def x_params_init(self, motor, pid, **_extra):
        """初始化水平方向电机参数。

        x 轴无软件复位、无软限位、无末段/顶段减速带：
          - x 是 motor_280 编码器闭环，正常不跑偏，不需丢步兜底；
          - 软限位已取消（用户原话："灵活使用就好，一般不会超"）；
          - 边界由 PID 主限幅 pid.output_limits（默认 [-0.4, 0.4]）+ 编码器闭环兜底。
        _extra 吸收未来新增键,避免 **horiz_cfg unpack 时 TypeError。
        """
        # 定义水平移动电机,PID参数
        self.motor_x = MotorWrap(**motor)
        self.x_pid = PID(**pid)
        self.x_velocity_limit = pid['output_limits']
        self.x_pose_start = self.motor_x.get_dis()
        self.x_pose_now = 0
        self.x_pose_last = 0

        self.x_distance_change = 0

        self.x_stop_flag = CountRecord(10)
        self.x_pid_flag = CountRecord(5)
        # 丢步核对(与 y 对称):move_x_position 完成后用 ref_encoder 核对总位移
        self._x_ref_encoder_at_zero = None
        self._x_expected_total_delta = 0.0
        # 撞哪侧墙: "left" / "right" / None(未知)（move_x_position 中由 x_stop_check 自动识别）
        # 主动 reset_x 期间置 True,期间 _x_seeking_wall 让外部感知;退出后还原
        self._x_seeking_wall = False

    def x_stop_check(self):
        """
        检查水平方向是否停止

        Returns:
            bool: 是否停止
        """
        return self.x_stop_flag(
            abs(self.x_distance_change) < STOP_CHECK_THRESHOLD
        )
    def x_get_position(self):
        self.x_pose_now = self.motor_x.get_dis() - self.x_pose_start
        return self.x_pose_now

    def x_pid_moveto(self, target_pose):
        """
        使用PID控制水平方向移动

        Args:
            target_pose: 目标位置

        Returns:
            bool: 是否到达目标位置
        """
        self.x_pose_now = (
            self.motor_x.get_dis() - self.x_pose_start
        )
        self.x_distance_change = (
            self.x_pose_now - self.x_pose_last
        )
        self.x_pose_last = self.x_pose_now
        error = target_pose - self.x_pose_now

        velocity = self.x_pid(self.x_pose_now)

        self.x_speed(velocity)

        if self.x_pid_flag(abs(error) < POSITION_ERROR_THRESHOLD):
            return True
        else:
            return False

    def move_x_position(self, target, out_time = 6.0, v_max_mms: float = None):
        """
        移动水平方向指定位置。PID 闭环 (2026-08-12 用户决定替换 bang-bang)。

        原 bang-bang (08-01) 全速 (v_max) 冲到目标 1.5mm 内立刻停 → 皮带回弹/背隙
        导致 "转不到位就停止" (X 短 30-114mm, SDK 却报 ok). 改回 PID:
          近目标自动减速, 停止柔, 回弹小 — 用 "慢速接近 + 可能轻微振荡但最终到位"
          换 "全速冲但停不到位". 控制律 = x_pid (Kp=6, Ki=0, Kd=1.0,
          arm_cfg.yaml horiz_cfg.pid), output_limits 收紧到 ±v_max:
          远段饱和 = bang-bang 快速赶路, 误差 < ~16mm 内 PID 温柔收尾.

        算法:
          - setpoint=target, output_limits=±v_max (默认 40mm/s; composite_run 传
            x_v_max_mms=100 → 100mm/s)
          - 循环调 x_pid_moveto(target): 单步 PID + 连续 5 帧 <1.5mm 到位判定
          - 到位 → x_speed(0); out_time 超时 / 急停 → 兜底退出
          - 10ms 采样 (x_pid.sample_time=0.01 默认)

        Args:
            target: 目标位置 (m)
            out_time: 超时 (s)
            v_max_mms: 速度上限 (mm/s),默认 40。临时收紧 x_pid.output_limits 和
                       x_velocity_limit,try/finally 还原。
        """
        # 1) 命令位移记录
        prev_pos = self.x_get_position()
        self._x_expected_total_delta += abs(target - prev_pos)

        # 临时收紧 PID 限幅 (try/finally 还原, 防污染后续 move_x / goto_position)
        saved_pid_limits = self.x_pid.output_limits
        saved_vel_limit = self.x_velocity_limit
        v_max = 0.04  # 默认 40 mm/s
        if v_max_mms is not None:
            v_max = float(v_max_mms) / 1000.0
        self.x_pid.setpoint = float(target)
        self.x_pid.output_limits = (-v_max, v_max)
        self.x_velocity_limit = (-v_max, v_max)

        end_time = time.time() + out_time
        try:
            while True:
                if self._must_stop():
                    break
                if time.time() > end_time:
                    break
                if self.x_pid_moveto(float(target)):
                    self.x_speed(0)
                    break
                # PID 采样周期 10ms (x_pid.sample_time=0.01 默认)
                time.sleep(0.01)
        finally:
            self.x_speed(0)
            # 还原 PID 限幅,避免临时收紧污染后续 move_x / goto_position 的初始状态
            self.x_pid.output_limits = saved_pid_limits
            self.x_velocity_limit = saved_vel_limit

        # 2) 命令/编码器核对(仅在已知 ref 时)
        if self._x_ref_encoder_at_zero is not None:
            actual_disp = abs(self.motor_x.get_dis() - self._x_ref_encoder_at_zero)
            disp_err = abs(actual_disp - self._x_expected_total_delta)
            STEP_LOSS_TOL_M = 0.005
            if disp_err > STEP_LOSS_TOL_M:
                logger.warning(
                    f"move_x_position 疑似丢步: 累积预期={self._x_expected_total_delta*1000:.1f}mm "
                    f"编码器={actual_disp*1000:.1f}mm 偏差={disp_err*1000:.1f}mm"
                )

        final = self.x_get_position()
        if abs(final - target) > 0.002:
            logger.error(
                f"move_x_position 丢步严重: target={target:.4f} final={final:.4f} "
                f"diff={(final-target)*1000:.1f}mm"
            )
        # 把实际 delta 累加确认
        self._x_expected_total_delta += abs(final - prev_pos) - abs(target - prev_pos)


    def reset_x(self, direction: str = "right", reset_velocity: float = 0.05,
                seek_timeout: float = 25.0):
        """
        主动撞墙定 x 原点。

        2026-08-01 重写:基于"编码器位移是唯一凭证"原则。
        单窗口不动可能是电机启动抖动 → 连续 N 窗口不动才算撞墙。
        """
        v = abs(reset_velocity)
        STALL_WINDOW_S = 0.3   # 窗口时长
        STALL_WINDOW_M = 0.001  # 窗口内位移阈值 (m) — 1mm
        # 2026-08-01: 单窗口不动可能是电机启动/通讯延迟。连续 3 个窗口
        # (≈0.9s) 不动才认定撞墙。0.05 m/s × 0.3s = 15mm >> 1mm 阈值,
        # 电机正常动时下个窗口会清零 stall_count;真撞墙 stall 永远清不零。
        STALL_REQUIRED_WINDOWS = 3

        self._x_seeking_wall = True
        start = time.time()
        window_pos = self.x_get_position()
        window_t0 = time.time()
        stall_count = 0  # 连续 stall 窗口计数

        try:
            while True:
                if self._must_stop():
                    logger.warning("reset_x: 收到急停/取消,中止撞墙")
                    return False
                if time.time() - start > seek_timeout:
                    logger.error(
                        "reset_x: 找墙超时 seek_timeout=%.1fs", seek_timeout
                    )
                    return False

                # 窗口制 stall 检测
                now = time.time()
                if now - window_t0 >= STALL_WINDOW_S:
                    cur = self.x_get_position()
                    moved = abs(cur - window_pos)
                    logger.info(
                        "reset_x dbg: cur=%.4f moved=%.4f stall_count=%d",
                        cur, moved, stall_count,
                    )
                    window_pos = cur
                    window_t0 = now
                    if moved > STALL_WINDOW_M:
                        # 窗口内动过 → 正常驱动,清零 stall 计数
                        stall_count = 0
                    else:
                        # 窗口内几乎没动
                        stall_count += 1
                        if stall_count >= STALL_REQUIRED_WINDOWS:
                            # 连续 N 窗口不动 → 撞墙 calibrate
                            dis = self.motor_x.get_dis()
                            self.x_pose_start = dis
                            self.x_pose_now = 0
                            self._x_ref_encoder_at_zero = dis
                            self._x_expected_total_delta = 0.0
                            self._x_wall = direction
                            logger.info(
                                "reset_x: 撞墙 calibrate,direction=%s,ref=%.6f,"
                                "耗时%.2fs (stall %d 窗口)",
                                direction, dis, time.time() - start,
                                stall_count,
                            )
                            self.motor_x.set_linear(0)
                            return True

                self.x_speed(v if direction == "right" else -v)
                time.sleep(0.01)
        finally:
            self._x_seeking_wall = False
            self.motor_x.set_linear(0)
        return False


    def reset_all(self, arm_angle: float = 90, hand_angle: float = -90,
                  x_direction: str = "right",
                  reset_x_velocity: float = 0.05,
                  timeout: float = 60.0,
                  reset_x: bool = True,
                  do_reset_y: bool = True):
        """
        复合复位:x 撞墙 + 大臂 + 手爪 三路并行,完成后 reset_y 触底串行。

        2026-07-31 改造: 加 reset_x=True 默认参数 (历史默认行为)。
          - reset_x=True  (默认): 与原来完全一致,x 撞墙 + arm + hand 三路并行
          - reset_x=False: 跳过 reset_x 撞墙,只并行 arm + hand,最后 reset_y 串行收尾。
            给 ensure_initialized 复用路径 (auto-init 自愈循环) 用,避免反复撞墙触发
            commit fb24b1a 描述的 PM2 死循环。

        2026-08-03 改造: 加 do_reset_y=True 开关,允许 caller 自己消化 reset_y 后让
          reset_all 只跑并行段。典型用法:init 想把"存储仓归位"提到第一步,
          那 storage 步骤里就要先 reset_y 建零,再 move_y + set_storage_angle,
          此时 reset_all 只需撞墙 + 大臂 + 手爪,并行段后不再 reset_y (do_reset_y=False)。
          - do_reset_y=True  (默认): 与原来完全一致,并行段 + reset_y 串行收尾
          - do_reset_y=False: 只跑并行段,跳过 reset_y。返回里 "y": None 表示跳过。

        为什么不能无条件接入 _create_car_locked / ensure_initialized:
          commit fb24b1a 已根治"reset_x 撞墙 + auto-init 反复调用"的 PM2 死循环。
          此方法由 _create_car_locked 显式调用 (reset_x=True),
          ensure_initialized 复用路径显式传 reset_x=False,跳过撞墙。

        并行原理:
          - x 是 motor_280 编码器电机,大臂/手爪是 PWM/bus 舵机,三者在物理上独立。
          - serial_mc602.lock 串行化串口写入 → Python 层并行,实际串口 FIFO;
            但 set_arm_angle/set_hand_angle 的等待时间里,x_reset 循环可以跑。
          - 撞墙速度 0.02 m/s + 舵机非阻塞 → 不冲突。

        失败语义:任何子步骤异常 logger.warning 不抛,保证 runtime 不会因为单个动作失败
        进入 _should_probe_controller recover 路径。

        Args:
            arm_angle: 大臂目标角度 (°),默认 90=UP
            hand_angle: 手爪目标角度 (°),默认 -90=UP
            x_direction: x 撞墙方向,默认 "right"
            reset_x_velocity: x 撞墙速度 (m/s),默认 0.05
            timeout: 并行阶段总超时 (s)
            reset_x: 是否包含 x 撞墙 (默认 True)
            do_reset_y: 是否在并行阶段之后串行跑 reset_y 触底 (默认 True)

        Returns:
            dict: {"x": bool|None, "arm": bool, "hand": bool, "y": bool|None}
                  reset_x=False 时 "x": None (表示跳过)
                  do_reset_y=False 时 "y": None (表示跳过)
        """
        from concurrent.futures import ThreadPoolExecutor, as_completed
        results = {}

        def _do_x():
            if not reset_x:
                return ("x", None)
            return ("x", self.reset_x(direction=x_direction,
                                       reset_velocity=reset_x_velocity))

        def _do_arm():
            # set_arm_angle 阻塞到舵机到位(MID/UP 约 1s)
            try:
                self.set_arm_angle(arm_angle, speed=80)
                return ("arm", True)
            except Exception as exc:
                logger.warning("reset_all: set_arm_angle 异常: %s" % exc)
                return ("arm", False)

        def _do_hand():
            try:
                self.set_hand_angle(hand_angle, speed=80)
                return ("hand", True)
            except Exception as exc:
                logger.warning("reset_all: set_hand_angle 异常: %s" % exc)
                return ("hand", False)

        try:
            with ThreadPoolExecutor(max_workers=3, thread_name_prefix="reset_all") as ex:
                futs = [ex.submit(_do_x), ex.submit(_do_arm), ex.submit(_do_hand)]
                for fut in as_completed(futs, timeout=timeout):
                    try:
                        name, ok = fut.result()
                        results[name] = bool(ok)
                    except Exception as exc:
                        logger.warning("reset_all: 子步骤异常: %s" % exc)
        except Exception as exc:
            logger.warning("reset_all: 并行阶段异常: %s" % exc)

        # reset_y 串行,最后（不放在线程池里 — 触底磁感应是绝对零点）。
        # do_reset_y=False:跳过 y 收尾,典型用法是 caller 在 reset_all 之前已自行调用
        # reset_y (例如 init 把存储仓归位提前,内部已做 reset_y 建零)。
        if do_reset_y:
            try:
                y_ok = bool(self.reset_y())
            except Exception as exc:
                logger.warning("reset_all: reset_y 异常: %s" % exc)
                y_ok = False
            results["y"] = y_ok
        else:
            results["y"] = None  # 表示跳过, caller 已自行处理 y

        logger.info("reset_all 完成: %s" % results)
        return results


    # ============== 复合动作 (业务层 pick / release / go_home 用) ==============
    #
    # 与 reset_all 同样的设计：
    #   - ThreadPoolExecutor 在一个 runtime job 内部并行驱动多个电机
    #   - arm_queue 单 worker,串行的是 JOB 之间,JOB 内的并发是安全的
    #   - 任何子步骤异常 logger.warning 不抛,避免 runtime 走 _should_probe_controller 自愈路径
    #   - 返回 {"ok": bool, "steps": {...}} 让上层 ArmRunner 自决定后续动作
    #
    # 业务侧说明 (main/arm/README.md §坐标系约定 + 软限位)：
    #   - x 单位米,y 单位米 (与 goto_position 一致)
    #   - arm_angle 单位度,set_arm_angle 直传
    #   - hand 单位度,set_hand_angle 直传
    #   - grasp 是 vacuum 开关

    def composite_pick(
        self,
        arm_angle: float,
        x: float,
        y: float,
        hand: float = 0.0,
        speed: int = 80,
        timeout: float = 30.0,
    ) -> dict:
        """复合抓取：并行 set_arm_angle + goto_position,再串行 set_hand_angle + grasp(True)。

        设计：
          ① ThreadPoolExecutor(2) 并行 set_arm_angle(arm_angle) + goto_position(x, y)
            (大臂舵机 + x/y 电机物理独立)
          ② 等两者都到位
          ③ 串行 set_hand_angle(hand) — 手爪独立,放最后避免与旋转中的大臂撞车
          ④ grasp(True) 开真空泵

        Returns:
            {"ok": bool, "steps": {"arm": bool, "position": bool, "hand": bool, "grasp": bool}}
        """
        results = {"arm": False, "position": False, "hand": False, "grasp": False}

        # ① 并行：大臂角度 + xy 位置
        def _do_arm():
            self.set_arm_angle(arm_angle, speed=speed)
            return ("arm", True)

        def _do_position():
            self.goto_position(x, y)
            return ("position", True)

        try:
            with ThreadPoolExecutor(max_workers=2, thread_name_prefix="composite_pick") as ex:
                futs = [ex.submit(_do_arm), ex.submit(_do_position)]
                for fut in as_completed(futs, timeout=timeout):
                    try:
                        name, ok = fut.result()
                        results[name] = bool(ok)
                    except Exception as exc:
                        logger.warning("composite_pick 子步骤异常: %s" % exc)
        except Exception as exc:
            logger.warning("composite_pick 并行阶段异常: %s" % exc)

        # ② 仅当 position 成功才继续 hand + grasp (避免抓在错误位置)
        if results["position"]:
            try:
                self.set_hand_angle(hand, speed=speed)
                results["hand"] = True
            except Exception as exc:
                logger.warning("composite_pick: set_hand_angle 异常: %s" % exc)
            try:
                self.grasp(True)
                results["grasp"] = True
            except Exception as exc:
                logger.warning("composite_pick: grasp 异常: %s" % exc)
        else:
            logger.warning("composite_pick: position 未到位,跳过 hand + grasp")

        ok = all(results.values())
        return {"ok": ok, "steps": results}


    def composite_release(
        self,
        drop_x: float = 0.0,
        drop_y: float = 0.03,
        hand: float = 0.0,
        speed: int = 80,
        timeout: float = 30.0,
    ) -> dict:
        """复合释放：保守序列 — 先 set_hand_angle,再 goto_position,再 grasp(False)。

        为什么 release 不用并发：
          - hand 放料姿态是 DOWN (0°),与 set_hand_angle 直传;
            同时 set_hand_angle 在大臂展开带 [-30, +30] 时拒绝非 UP,
            因此保守起见 hand 串行在 position 之前(避开"手爪先下但位置未到"的中间态)
          - position 单轴并发节省有限,主要时间花在舵机 wait,不值得加并发复杂度

        Returns:
            {"ok": bool, "steps": {"hand": bool, "position": bool, "grasp": bool}}
        """
        results = {"hand": False, "position": False, "grasp": False}

        # ① hand 串行 (业务侧 _check_y_protected 在 client wrapper 已做)
        try:
            self.set_hand_angle(hand, speed=speed)
            results["hand"] = True
        except Exception as exc:
            logger.warning("composite_release: set_hand_angle 异常: %s" % exc)
            # hand 失败仍继续 — 可能已经在 DOWN

        # ② position 单线程,够用
        try:
            self.goto_position(drop_x, drop_y)
            results["position"] = True
        except Exception as exc:
            logger.warning("composite_release: goto_position 异常: %s" % exc)

        # ③ grasp 关真空
        try:
            self.grasp(False)
            results["grasp"] = True
        except Exception as exc:
            logger.warning("composite_release: grasp 异常: %s" % exc)

        ok = all(results.values())
        return {"ok": ok, "steps": results}


    def composite_go_home(
        self,
        hand: float = -90.0,
        arm: float = 0.0,
        speed: int = 80,
        timeout: float = 30.0,
    ) -> dict:
        """复合回原点：并行 set_arm_angle + goto_position,再串行 set_hand_angle(hand)。

        hand 放最后：
          - hand=-90 (UP) 是安全姿态,即便大臂在 [-30,+30] 也不会撞车
          - 与 reset_all 末尾 hand 串行的设计一致

        Returns:
            {"ok": bool, "steps": {"arm": bool, "position": bool, "hand": bool}}
        """
        results = {"arm": False, "position": False, "hand": False}

        # ① 并行：大臂角度 + xy 位置
        def _do_arm():
            self.set_arm_angle(arm, speed=speed)
            return ("arm", True)

        def _do_position():
            self.goto_position(0.0, 0.0)
            return ("position", True)

        try:
            with ThreadPoolExecutor(max_workers=2, thread_name_prefix="composite_go_home") as ex:
                futs = [ex.submit(_do_arm), ex.submit(_do_position)]
                for fut in as_completed(futs, timeout=timeout):
                    try:
                        name, ok = fut.result()
                        results[name] = bool(ok)
                    except Exception as exc:
                        logger.warning("composite_go_home 子步骤异常: %s" % exc)
        except Exception as exc:
            logger.warning("composite_go_home 并行阶段异常: %s" % exc)

        # ② hand 串行,放最后(参考 reset_all 模式)
        try:
            self.set_hand_angle(hand, speed=speed)
            results["hand"] = True
        except Exception as exc:
            logger.warning("composite_go_home: set_hand_angle 异常: %s" % exc)

        ok = all(results.values())
        return {"ok": ok, "steps": results}

    def composite_run(
        self,
        arm: Union[float, str, None] = None,
        x: Union[float, None] = None,
        y: Union[float, None] = None,
        hand: Union[float, str, None] = None,
        speed: int = 80,
        timeout: float = 30.0,
        y_pid_timeout: float = 10.0,
        x_v_max_mms: float = 100.0,
    ) -> dict:
        """四电机通用并行驱动器：在一路 runtime job 内同时驱动 motor_y / motor_x / arm_servo / hand_servo。

        用法：
          - 任一参数传 None 表示该路跳过；
          - y/x 用 PID 闭环(mm 坐标直接传也行,本方法按 m 计算)；
          - arm/hand 走舵机角度（数字或字符串 "LEFT"/"RIGHT"/"MID"/"UP"/"DOWN"，会查 yaml list）；
          - timeout 是并行阶段总超时，单路超时单独计时不互相阻塞。

        并行可行性（与 composite_pick / composite_go_home 一致）：
          - 物理上 y / x / arm / hand 四轴完全独立,可同时下发；
          - serial_mc602.lock 串行化串口写入 → Python 层并发,串口字节 FIFO,
            舵机/电机指令在串口层被 serialize,但 set_arm_angle 等舵机阻塞在物理到位,
            期间 PID 闭环可以并行跑 — 这是公认收益的并发模式；
          - y 和 x 的 PID 都各自持有独立 setpoint / pose_now,本方法把它们都丢进
            ThreadPoolExecutor 后各自调 goto_position 内部串行,与 composite_pick
            调 self.goto_position 行为一致；
          - reset_y 不能放进这个并行池 — 见 composite_run_reset 说明。

        失败语义：单路子步骤异常 logger.warning + 返回 False,不入 _should_probe_controller;
        全部子路成功才 "ok": True。

        Args:
            arm: 大臂角度 (°) 或字符串,None=跳过。
            x: 水平目标位置 (m),None=跳过。
            y: 竖直目标位置 (m),None=跳过。注意 y 是 PID 闭环,耗时较长。
            hand: 手爪角度 (°) 或字符串,None=跳过。
            speed: 舵机速度 (仅 arm/hand 生效)。
            timeout: 并行阶段总超时 (s)。
            y_pid_timeout: y 闭环单独超时 (s)。

        Returns:
            {"ok": bool, "steps": {"arm": bool, "x": bool, "y": bool, "hand": bool}}
        """
        from concurrent.futures import ThreadPoolExecutor, as_completed
        results = {"arm": False, "x": False, "y": False, "hand": False}

        # 跳过空指令:避免线程池空转,也方便上层任意传 None。
        todo = []
        if arm is not None:
            todo.append(("arm", lambda: self.set_arm_angle(arm, speed=speed)))
        if x is not None:
            todo.append(("x", lambda: self.move_x_position(float(x), v_max_mms=x_v_max_mms)))
        if y is not None:
            todo.append(("y", lambda: self.move_y_position(float(y))))
        if hand is not None:
            todo.append(("hand", lambda: self.set_hand_angle(hand, speed=speed)))

        if not todo:
            return {"ok": True, "steps": results}

        def _wrap(name, fn):
            try:
                fn()
                return (name, True)
            except Exception as exc:
                logger.warning("composite_run: %s 子步骤异常: %s" % (name, exc))
                return (name, False)

        try:
            with ThreadPoolExecutor(
                max_workers=len(todo), thread_name_prefix="composite_run"
            ) as ex:
                futs = [ex.submit(_wrap, name, fn) for name, fn in todo]
                for fut in as_completed(futs, timeout=timeout):
                    try:
                        name, ok = fut.result()
                        results[name] = bool(ok)
                    except Exception as exc:
                        logger.warning("composite_run: 子步骤异常: %s" % exc)
        except Exception as exc:
            logger.warning("composite_run: 并行阶段异常: %s" % exc)

        ok = all(results.values())
        return {"ok": ok, "steps": results}

    def composite_run_reset(
        self,
        arm_angle: float = 90,
        hand_angle: float = -90,
        x_direction: str = "right",
        reset_x_velocity: float = 0.03,
        timeout: float = 60.0,
        reset_x: bool = True,
    ) -> dict:
        """复合复位 + y 最后：x 撞墙 + 大臂 + 手爪 三路并行,reset_y 触底串行收尾。

        设计说明（与 composite_run 一致,但有 reset 专属差别）：
          - 用 x(撞墙) 而非 x(到点),因为 reset 必须以硬件墙为绝对零点；
          - reset_y 串行收尾在并行池外 — 见 reset_all 内注释 [arm_base.py L744]
            "触底磁感应是绝对零点" + 并行失败回滚复杂；
          - 与 reset_all 等价(参数完全相同),但作为独立入口便于上层按名字区分
            "完整复位" vs "运行时多轴并行"。
          - reset_x=False 时跳过 reset_x 撞墙,只并行 arm + hand（与 reset_all 同语义）。
        """
        from concurrent.futures import ThreadPoolExecutor, as_completed
        results = {}

        def _do_x():
            if not reset_x:
                return ("x", None)
            return ("x", self.reset_x(direction=x_direction,
                                       reset_velocity=reset_x_velocity))

        def _do_arm():
            try:
                self.set_arm_angle(arm_angle, speed=80)
                return ("arm", True)
            except Exception as exc:
                logger.warning("composite_run_reset: set_arm_angle 异常: %s" % exc)
                return ("arm", False)

        def _do_hand():
            try:
                self.set_hand_angle(hand_angle, speed=80)
                return ("hand", True)
            except Exception as exc:
                logger.warning("composite_run_reset: set_hand_angle 异常: %s" % exc)
                return ("hand", False)

        try:
            with ThreadPoolExecutor(
                max_workers=3, thread_name_prefix="composite_run_reset"
            ) as ex:
                futs = [ex.submit(_do_x), ex.submit(_do_arm), ex.submit(_do_hand)]
                for fut in as_completed(futs, timeout=timeout):
                    try:
                        name, ok = fut.result()
                        results[name] = bool(ok)
                    except Exception as exc:
                        logger.warning("composite_run_reset: 子步骤异常: %s" % exc)
        except Exception as exc:
            logger.warning("composite_run_reset: 并行阶段异常: %s" % exc)

        try:
            y_ok = bool(self.reset_y())
        except Exception as exc:
            logger.warning("composite_run_reset: reset_y 异常: %s" % exc)
            y_ok = False
        results["y"] = y_ok

        logger.info("composite_run_reset 完成: %s" % results)
        return results


    def hand_params_init(self, hand, hand2, grap):
        """
        初始化手部参数

        Args:
            hand: 手臂舵机配置
            hand2: 手部舵机配置
            grap: 抓取机构配置
        """
        # 手爪舵机(hand2)实际接在 PWM d2 = port=2(末端上下俯仰,PWM 协议)。
        # 历史: commit a0995ec 曾把协议改成 ServoBus(port=3) 并假设手爪在 bus port=3,
        # 实际方向反了 —— 调 set_hand_angle 撞到了大臂(bus port=3),末端手爪物理上不动。
        # 已通过 read_back 总线舵机角度决定性验证(bus port=3 是大臂,port=2 没舵机接)。
        # 修复:hand_servo 改回 ServoPwm(hand2["port"], mode=180),协议匹配 PWM d2;
        # 对应 yaml hand2.port=2(PWM),hand.port=3(Bus 大臂)。
        self.hand_servo = ServoPwm(hand2["port"], mode=180)
        # 2026-07-16: yaml 中 angle_list 已删（用户要求灵活使用，无预设）。
        # SDK 字符串接口仍接受 "UP"/"MID"/"DOWN"/"LEFT"/"RIGHT"，但返回 None 让业务层报错。
        # 业务层只走数字接口（set_arm_angle(-90) / set_hand_angle(-90)）。
        self.hand_angle_list2 = hand2.get("angle_list", {}) or {}
        self.arm_servo = ServoBus(hand["port"])
        self.hand_angle_list = hand.get("angle_list", {}) or {}
        self.pump = PoutD(grap["port_pump"])
        self.valve = PoutD(grap["port_valve"])

    def grasp(self, value: bool):
        """
        控制抓取机构

        Args:
            value: 抓取状态, True为抓取, False为释放
        """
        self.pump.set(not value)
        self.valve.set(value)


    def position_params_init(self):
        """
        初始化位置参数。不再依赖 yaml 持久化，始终以当前编码器值为零点。
        """
        self.pose_enable = False
        self.side = "MID"

        self.x_pose_start = self.motor_x.get_dis()
        self.x_pose_now = 0.0

        self.y_pose_start = self.motor_y.get_dis()
        self.y_pose_now = 0.0

    def save_config(self, pose_enable=True):
        """
        空操作：不再把机械臂位置写入 yaml。
        保留此函数签名以兼容旧调用方，不再有任何副作用。
        """
        pass

    def y_speed(self, velocity):
        """
        设置竖直方向速度

        业务约定（与 main/arm 一致）：y>0=向下（朝触底），y<0=向上（远离触底）。
        业务层 move_y 直传 target 给车端，不取反；车端 motor 方向由 reverse 标志决定。
        velocity 符号与物理方向的具体对应见实测（业务侧只关心 y<0=向上、y>0=向下）。

        磁感安全门：磁感触发时把 velocity 强制置 0，
        防止继续朝磁感方向硬推。

        Args:
            velocity: 速度值
        """
        # === 急停门：外部置位急停时强制 0，任何 y 运动都被此 chokepoint 拦死 ===
        if self._must_stop():
            velocity = 0
        # === 末段减速 / 顶段减速：根据当前位置分档限幅 ===
        # 注意:必须先分档限幅,最后再做 velocity_limit (主限幅)
        # 否则当 slow_velocity < velocity_limit 时,主限幅会把 slow 限制"放大"回去
        cur = self.y_get_position()
        if self.y_slow_band_m > 0 and cur >= -self.y_slow_band_m and cur < 0.0:
            # 已进入末段（接近磁感，y >= -slow_band 且 y < 0）
            velocity = limit_val(velocity, -self.y_slow_velocity, self.y_slow_velocity)
        elif self.y_top_slow_m > 0 and cur <= -self.y_top_slow_m:
            # 已进入顶段（远离磁感，y <= -top_slow）：减速防失步
            velocity = limit_val(velocity, -self.y_top_slow_velocity, self.y_top_slow_velocity)
        # === 磁感安全门：磁感触发 + velocity>0 时 velocity=0，不再朝磁感方向推进 ===
        # seek_bottom 模式下放行(让 reset_y 能真正压到磁感)。注意 seek 结束后立刻置 False。
        if velocity > 0 and self.y_reset_check() and not self._y_seeking_bottom:
            logger.warning("y_speed: 磁感触发，禁止继续推进，velocity=0")
            velocity = 0
        velocity = limit_val(velocity, *self.y_velocity_limit)
        self.motor_y.set_velocity(velocity)

    def x_speed(self, velocity):
        """
        设置水平方向速度

        x 轴无软件软限位、无末段/顶段减速带：仅急停门 + PID 主限幅。
        物理墙保护由 move_x_position 中的 x_stop_check 触发 calibrate 兜底。
        """
        # === 急停门：外部置位急停时强制 0 ===
        if self._must_stop():
            velocity = 0
        velocity = limit_val(velocity, *self.x_velocity_limit)
        self.motor_x.set_linear(velocity)

    def set_position_start(self, y_position):
        """
        设置起始位置

        Args:
            y_position: 竖直位置
        """
        self.y_pose_start = self.y_pose_now
        self.x_pose_start = self.x_pose_now

    def reset_position(self):
        """重置机械臂位置（init 阶段并行复位 + 串行定原点）。

        实现：
          ① 并行：大臂 → +90°（复位位） ＋  手爪 → -90°（UP）
             — 两路舵机物理独立，set_arm_angle 阻塞到位期间手爪可并行到位，
               总耗时从原 ~1.5s 串行降到 ~0.8s 并行。
          ② 串行：reset_y 触底定原点（reset_y 内部:触发磁感 → y_pose_now=0
             → move_y_position(-150mm) 收尾）。

        不放并行：reset_y 不能进并行池 —— 触底磁感是绝对零点 + 并行失败回滚复杂；
        与 composite_run_reset 的设计一致。

        历史重置位（2026-07-27 联调第三次改）：
          - 大臂：+90°（业务硬限上界，复位位）
          - 手爪：UP (-90°)

        历史版本：
          - 2026-07-16 初版：set_arm_angle(0) — 0° 是 MID 位置
          - 2026-07-27 改：set_arm_angle(+90) — +90° 是用户实测的复位位
          - 本次(并行优化)：arm + hand 并行下发，y 串行收尾，x 不参与 reset
            (CLAUDE.md: x 由视觉闭环控制)。

        注意：reset_y 内部已经走到 -150mm,不要再 self.y = 0,
        否则会把刚升到 -150mm 的臂又拉回 0(2026-07-27 bug)。
        """
        from concurrent.futures import ThreadPoolExecutor, as_completed

        results = {"arm": False, "hand": False, "y": False}

        def _do_arm():
            try:
                self.set_arm_angle(90, speed=80)  # 复位位 +90°
                return ("arm", True)
            except Exception as exc:
                logger.warning("reset_position: set_arm_angle 异常: %s" % exc)
                return ("arm", False)

        def _do_hand():
            try:
                self.set_hand_angle(-90, speed=80)  # UP
                return ("hand", True)
            except Exception as exc:
                logger.warning("reset_position: set_hand_angle 异常: %s" % exc)
                return ("hand", False)

        # ① 并行 arm + hand
        try:
            with ThreadPoolExecutor(
                max_workers=2, thread_name_prefix="reset_position"
            ) as ex:
                futs = [ex.submit(_do_arm), ex.submit(_do_hand)]
                for fut in as_completed(futs, timeout=15.0):
                    try:
                        name, ok = fut.result()
                        results[name] = bool(ok)
                    except Exception as exc:
                        logger.warning("reset_position: 子步骤异常: %s" % exc)
        except Exception as exc:
            logger.warning("reset_position: 并行阶段异常: %s" % exc)

        # ② reset_y 串行收尾（绝对零点,不放并行池）
        try:
            y_ok = bool(self.reset_y())
        except Exception as exc:
            logger.warning("reset_position: reset_y 异常: %s" % exc)
            y_ok = False
        results["y"] = y_ok

        logger.info("reset_position 完成: %s" % results)
        return results

    def switch_side(self, side):
        """
        切换机械臂方向

        Args:
            side: 机械臂的方向, LEFT、RIGHT或MID
        """
        if self.side != side:
            self.side = side
            logger.info(f"Changing side to {self.side}")
        else:
            return
        angle_target = self.hand_angle_list[side]
        self.set_arm_angle(angle_target, 80)
        time.sleep(0.5)

    
    
    def set_arm_angle(self, angle: Union[str, int] = "RIGHT", speed=80):
        """
        设置机械臂角度

        Args:
            angle: 目标角度，可以是字符串（"LEFT", "MID", "RIGHT"）或数字
            speed: 速度
        """
        _angle = angle
        if isinstance(_angle, str):
            self.side = _angle
            assert _angle in ("LEFT", "MID", "RIGHT"), "Direction should be LEFT, MID, or RIGHT"
            _angle = self.hand_angle_list[_angle]
        self._arm_angle_last = _angle
        self.arm_servo.set_angle(_angle, speed)

    def set_hand_angle(self, angle: Union[str, int] = "UP", speed=80):
        """
        设置机械臂手角度

        Args:
            angle: 目标角度，可以是字符串（"UP", "MID", "DOWN"）或数字
            speed: 速度
        """
        if isinstance(angle, str):
            assert angle in ("UP","MID","DOWN"), "Direction should be UP, MID, or DOWN"
            angle = self.hand_angle_list2[angle]
        self._hand_angle_last = angle
        self.hand_servo.set_angle(angle, speed)

    def go_for(self, x_offset, y_offset, time_run=None, speed=[0.15, 0.04]):
        """
        移动机械臂到当前位置的相对量

        Args:
            x_offset: 水平偏移
            y_offset: 竖直偏移
            time_run: 运行时间
            speed: 速度 [水平速度, 竖直速度]
        """
        x_pos = self.x_pose_now + x_offset
        y_pos = self.y_pose_now + y_offset
        self.goto_position(x_pos, y_pos, time_run, speed)
    
    def goto_position(self, x=None, y=None,time_run=None, speed= [0.15, 0.04]):
        """
        移动到指定机械臂位置

        Args:
            x: 水平位置
            y: 竖直位置
            time_run: 运行时间
            speed: 速度 [水平速度, 竖直速度]
        """

        # 控制上下限（x 轴软限位已取消，y 轴保留）
        x_pos = x
        y_pos = limit_val(
            y,
            self.y_threshold[0],
            self.y_threshold[1]
        )

        # 获取结束时间和对应速度
        time_start = time.time()
        if time_run is not None:
            assert isinstance(time_run, (int, float)), "Time must be a number"
            # 根据时间求速度
            time_end = time_start + time_run
            y_time = time_run
            x_time = time_run
        elif speed is not None:
            # 根据速度求时间
            if isinstance(speed, (int, float)):
                speed_x = speed
                speed_y = speed
            elif isinstance(speed, (list, tuple)):
                speed_x = speed[0]
                speed_y = speed[1]
            else:
                logger.error("Invalid speed argument")
                return
            x_time = abs(
                x_pos - self.x_pose_now
            ) / speed_x
            y_time = abs(
                y_pos - self.y_pose_now
            ) / speed_y
            time_run = max(x_time, y_time)
        else:
            logger.error("Either time_run or speed must be provided")
            return
        # 超时时间
        time_end = time_start + time_run

        # 定义结束标志和到达位置标记量
        if y is None:
            y_flag = True
        else:
            y_flag = False
        
        if x is None:
            x_flag = True
        else:
            x_flag = False

        # 获取对应的速度和pid位置
        if y_time < 0.1:
            speed_y = 0.1
            y_flag = True
        else:
            speed_y = abs(
                y_pos - self.y_pose_now
            ) / y_time

        self.y_pid.setpoint = y_pos
        self.y_pid.output_limits = (-speed_y, speed_y)

        if x_time < 0.1:
            speed_x = 0.1
            x_flag = True
        else:
            speed_x = abs(
                x_pos - self.x_pose_now
            ) / x_time

        self.x_pid.setpoint = x_pos
        self.x_pid.output_limits = (
            -speed_x, speed_x
        )

        while True:
            # 到达结束标志结束
            if y_flag and x_flag:
                break
            # 2026-08-03 协作取消：急停/任务取消 → 立即停车退出
            if self._must_stop():
                self.x_speed(0)
                self.y_speed(0)
                return
            # 获取剩余时间
            time_remain = time_end - time.time()
            # 超时处理
            if time_remain < -3:
                logger.warning("Timeout")
                # 超时停止
                self.x_speed(0)
                self.y_speed(0)
                break
            if not y_flag:
                if self.y_pid_moveto(y_pos):
                    self.y_speed(0)
                    y_flag = True

                # 触底到位: 目标在上方且磁感触发 → 提前到位退出 + 校准基准。
                # 2026-08-01 修复: 原逻辑磁感一触发(含移动中噪声)就无条件重写
                # y_pose_start/y_pose_now, 视觉伺服 goto_position 持续跟踪时磁感噪声
                # 让位置基准每帧归零 → y 来回走。仅"目标在上方且已触底"才校准。
                if self.y_reset_check():
                    if self.y_pid.setpoint <= self.y_pose_now:
                        y_flag = True
                        self.y_speed(0)
                        self.y_pose_start = self.motor_y.get_dis()
                        self.y_pose_now = 0

            if not x_flag:
                if self.x_pid_moveto(x_pos):
                    self.x_speed(0)
                    x_flag = True
    def set_arm_pose(self,x=None,y=None,arm = None,hand = None):
        '''
        设置机械臂的位位姿

        Args:
            x: 水平位置
            y: 竖直位置
            arm: 手臂角度，可以是字符串（"LEFT", "MID", "RIGHT"）或数字
            hand: 手部角度，可以是字符串（"UP", "MID", "DOWN"）或数字
        
        '''
        self.goto_position(x, y)
        # 注：原先此处 arm -> hand 之间有 time.sleep(1) 死等,
        # 与 PID 闭环无关,纯空转。已删除(2026-07-31 PR#13)。
        if arm is not None:
            self.set_arm_angle(arm)
        if hand is not None:
            self.set_hand_angle(hand)

    # ==================== 便捷属性接口 ====================
    @property
    def y(self) -> float:
        """获取当前竖直位置（单位：mm）"""
        return self.y_get_position() * 1000.0

    @y.setter
    def y(self, mm: float):
        """设置目标竖直位置（单位：mm）"""
        self.move_y_position(mm / 1000.0)

    @property
    def x(self) -> float:
        """获取当前水平位置（单位：mm）"""
        return self.x_get_position() * 1000.0

    @x.setter
    def x(self, mm: float):
        """设置目标水平位置（单位：mm）"""
        self.move_x_position(mm / 1000.0)

    @property
    def angle(self) -> float:
        """获取手臂舵机当前角度"""
        return self._arm_angle_last if hasattr(self, '_arm_angle_last') else 0

    @angle.setter
    def angle(self, val: Union[str, int]):
        """设置手臂舵机角度"""
        self.set_arm_angle(val)

    @property
    def hand_angle(self) -> float:
        """获取手部舵机当前角度"""
        return self._hand_angle_last if hasattr(self, '_hand_angle_last') else 0

    @hand_angle.setter
    def hand_angle(self, val: Union[str, int]):
        """设置手部舵机角度"""
        self.set_hand_angle(val)


if __name__ == '__main__':
    arm = ArmController()
    print(f"机械臂长度: {arm.arm_length}")
    # 自测（reset_x 已删除，x 轴位置由外部/视觉闭环控制）
    print(f"x init: {arm.x_get_position():.4f} m")
    arm.move_x_position(0.1)
    print(f"x after move 0.1: {arm.x_get_position():.4f} m")
    arm.move_x_position(0.2)
    print(f"x after move 0.2: {arm.x_get_position():.4f} m")   

    # start_time = time.time()
    # # arm.grasp(True)
    # arm.reset_position()
    # arm.goto_position(0.15, 0.1)
    # # time.sleep(1)
    # arm.set_arm_angle("LEFT")
    # time.sleep(1)
    # arm.set_hand_angle("DOWN")
    # # arm.grasp(False)
    
    # print(f"移动时间: {time.time() - start_time:.4f}秒")
    # print(f"x: {arm.x_pose_now:.4f}, y: {arm.y_pose_now:.4f}")
