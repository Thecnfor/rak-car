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
from threading import Thread
from typing import Union

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



POSITION_ERROR_THRESHOLD = 4e-4 # 位置误差阈值
STOP_CHECK_THRESHOLD = 1e-10 # 停止检查阈值


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
        self.position_params_init(**self.config["pos_cfg"])


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
        velocity = self.y_pid(self.y_pose_now)

        self.y_speed(velocity)

        if self.y_pid_flag(abs(error) < POSITION_ERROR_THRESHOLD):
            return True
        else:
            return False

    def reset_y(self):
        """
        重置竖直方向位置：朝磁感方向下压找触底，【磁感触发】是唯一成功凭证。

        方向约定（实测）：setpoint>0/velocity>0 = 向下（朝磁感）。

        三段速度曲线，避免 PID 接近时减速被 y_stop_check 误判为堵转：
          1) 远段 (y < -slow_band - top_slow)：SLOW_VELOCITY 0.08 m/s 直驱；
          2) 末段 (y >= -slow_band 且 y < 0)：SLOW_VELOCITY 0.02 m/s 极慢贴底；
          3) 触底 (y_reset_check() 真触发)：保持 0.05s dwell 确认不是抖动，立即归零。
        找底期间 y_speed 的磁感门放行（_y_seeking_bottom=True），允许正速度穿入磁感。

        退出条件：
          - 成功：磁感触发后 dwell 通过 → 编码器 ref 记录 + y_pose_start 重置 + True；
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

        # 入口前先把 _y_seeking_bottom 设 True，让 y_speed 对正速度放行
        self._y_seeking_bottom = True
        start = time.time()
        triggered_at = None
        prev_pos = self.y_get_position()
        no_move_since = time.time()   # 编码器持续不动的最早时刻
        NO_MOVE_HARD_TIMEOUT = 2.0    # 长时间不动 → 强制停车报警

        try:
            while True:
                # 1) 急停优先
                estop = getattr(self, "_estop", None)
                if estop is not None and estop.is_set():
                    logger.warning("reset_y: 收到急停，中止找底")
                    break
                # 2) 磁感触发 → 记录 dwell 起点
                if self.y_reset_check():
                    if triggered_at is None:
                        triggered_at = time.time()
                    elif time.time() - triggered_at >= DWELL_TIME:
                        # 成功！dwell 通过
                        ref = self.motor_y.get_dis()
                        self.y_pose_start = ref
                        self.y_pose_now = 0
                        self._y_ref_encoder_at_zero = ref
                        self._y_expected_total_delta = 0.0
                        logger.info(
                            "reset_y: 磁感触发+dwell通过,ref_encoder=%.6f,耗时%.2fs"
                            % (ref, time.time() - start)
                        )
                        self.y_speed(0)
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
        self.y_speed(0)

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
        self._y_expected_total_delta += abs(target - prev_pos)
        # 第一轮（保持原行为）
        self.y_pid.setpoint = target
        while True:
            if self.y_pid_moveto(target):
                logger.info(f"移动到高度{target}（PID 收敛）")
                break
            if self.y_stop_check():
                logger.info(f"移到高度{target}过程中检测到停止")
                break
        self.y_speed(0)

        # ---- 丢步/堵转补偿（最多 2 轮） ----
        for round_idx in range(2):
            actual = self.y_get_position()
            err = target - actual
            if abs(err) <= 0.001:
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

    def x_params_init(self, motor, pid, threshold,
                      slow_band_m=0.020, slow_velocity=0.04,
                      top_slow_m=0.020, top_slow_velocity=0.06,
                      reset_target_m=0.34, reset_velocity=0.05,
                      reset_dwell_time=0.10, reset_timeout=8.0,
                      # v2 撞墙判据参数(模型驱动)
                      wall_ratio_threshold=0.20,
                      min_velocity_ratio=0.30,
                      wall_window_ms=200,
                      enable_encoder_reset=True,
                      **_extra):
        """初始化水平方向电机参数。

        slow_band_m/slow_velocity/top_slow_m/top_slow_velocity: 分段减速,仅 move_x_position 用。
        reset_* 参数:reset_x 用。撞墙是机械硬限位,无传感器,靠"编码器持续不动 reset_stall_count 次"
        + reset_dwell_time dwell 判定。
        _extra 吸收未来新增键,避免 **horiz_cfg unpack 时 TypeError。
        """
        # 定义水平移动电机,PID参数
        self.motor_x = MotorWrap(**motor)
        self.x_pid = PID(**pid)
        self.x_velocity_limit = pid['output_limits']
        self.x_pose_start = self.motor_x.get_dis()
        self.x_pose_now = 0
        self.x_threshold = threshold
        self.x_pose_last = 0

        self.x_distance_change = 0

        self.x_stop_flag = CountRecord(10)
        self.x_pid_flag = CountRecord(5)
        # 末段减速带
        self.x_slow_band_m = float(slow_band_m)
        self.x_slow_velocity = float(slow_velocity)
        # 顶段减速带
        self.x_top_slow_m = float(top_slow_m)
        self.x_top_slow_velocity = float(top_slow_velocity)
        # reset_x 配置
        self.x_reset_target_m = float(reset_target_m)
        self.x_reset_velocity = float(reset_velocity)
        self.x_reset_dwell_time = float(reset_dwell_time)
        self.x_reset_timeout = float(reset_timeout)
        # v2 撞墙判据(模型驱动 + 物理清零)
        self.x_wall_ratio_threshold = float(wall_ratio_threshold)
        self.x_min_velocity_ratio = float(min_velocity_ratio)
        self.x_wall_window_ms = float(wall_window_ms)
        self.x_enable_encoder_reset = bool(enable_encoder_reset)
        # 兼容老字段名(v1)
        self.x_reset_stall_count = 5  # v2 不再用
        self.x_reset_stall_threshold = 0.0005
        # 丢步核对(与 y 对称):reset_x 后记录 ref_encoder,move_x_position 完成后核对
        self._x_ref_encoder_at_zero = None
        self._x_expected_total_delta = 0.0
        # seek 模式:True 时 x_speed 撞墙门对正速度放行(reset_x 必须真撞墙才算成功)
        self._x_seeking_wall = False
        # 撞哪侧墙: "left" / "right" / None(未知)
        self._x_wall = None

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

    def move_x_position(self, target, out_time = 6.0):
        """
        移动水平方向指定位置。带软限位 + 丢步核对 + 兜底。

        1) 入口:软限位 limit_val (基于 x_threshold,默认 [-0.32, 0.32]);
        2) 命令位移记录:本次指令 delta = target - current,记录累积预期位移;
        3) PID 闭环到 < 1mm 或 out_time 超时或堵转跳出;
        4) 命令/编码器核对:偏差 > 5mm 报警;
        5) 完成后 actual vs target 偏差 > 2mm 报警。

        方向约定:target=0 在撞墙位置,远离墙为正(默认右侧)。
        """
        # 1) 软限位(y_threshold 自动 fallback 已在 y 轴;x 同样:若 [0,0.315] 错配则 [-0.32,0.32])
        if self.x_threshold[0] >= 0 and self.x_threshold[1] > 0:
            x_lo, x_hi = -0.32, 0.32
        else:
            x_lo, x_hi = self.x_threshold[0], self.x_threshold[1]
        target = limit_val(target, x_lo, x_hi)
        # 2) 命令位移记录
        prev_pos = self.x_get_position()
        self._x_expected_total_delta += abs(target - prev_pos)

        end_time = time.time()+out_time
        self.x_pid.setpoint = target
        while True:
            if time.time() > end_time:
                break
            if self.x_pid_moveto(target):
                break
            if self.x_stop_check():
                # 撞墙 calibrate: 与 reset_x 一致用相对零点(不调 motor.reset — 副作用锁电机)
                dis = self.motor_x.get_dis()
                self.x_pose_start = dis
                self._x_ref_encoder_at_zero = dis
                self._x_wall = "left" if dis < 0.15 else "right"
                break
            time.sleep(0.05)
        self.x_speed(0)

        # 3) 命令/编码器核对(仅在已知 ref 时)
        if self._x_ref_encoder_at_zero is not None:
            actual_disp = abs(self.motor_x.get_dis() - self._x_ref_encoder_at_zero)
            disp_err = abs(actual_disp - self._x_expected_total_delta)
            STEP_LOSS_TOL_M = 0.005
            if disp_err > STEP_LOSS_TOL_M:
                logger.warning(
                    f"move_x_position 疑似丢步: 累积预期={self._x_expected_total_delta*1000:.1f}mm "
                    f"编码器={actual_disp*1000:.1f}mm 偏差={disp_err*1000:.1f}mm, 建议重置原点"
                )

        final = self.x_get_position()
        if abs(final - target) > 0.002:
            logger.error(
                f"move_x_position 丢步严重: target={target:.4f} final={final:.4f} "
                f"diff={(final-target)*1000:.1f}mm, 建议重新定原点"
            )
        # 把实际 delta 累加确认
        self._x_expected_total_delta += abs(final - prev_pos) - abs(target - prev_pos)



    def reset_x(self):
        """
        重置水平方向位置:慢速撞右墙,v2 模型驱动 + 物理清零。

        与 y 轴核心区别:x 没有磁感/限位传感器,只能靠 encoder 推断撞墙。
        三层撞墙判据(任一持续 wall_window_ms 命中即认定):
          1) **位移比** = 实际位移 / 期望位移 (期望 = velocity * dt)
             - 正常: 比值 ≈ 1.0(实测速度 ≈ 命令速度)
             - 撞墙/失步: 比值 → 0(堵转不动) — **主判据**
          2) **速度比** = 滑窗实测速度 / 命令速度
             - 步进电机失步时编码器位置仍变化(走空了),位移比可能仍是 1.0,
               但滑窗速度会因为 "命令速度 = 期望速度" 而差距大 → **防丢步误判**
          3) 撞墙后 **物理清零编码器** (Motor.reset())
             - encoder_2.reset() 让编码器真正归零,x_pose_start/x_pose_now 直接 0
             - 替代旧的"x_pose_start = motor_x.get_dis() - 0.31" 数学调整,
               消除"行程常量"假设误差。

        失败不伪归零(返回 False)。超时 / 急停 / 撞墙条件不满足都算失败。
        """
        # === 配置 ===
        VELOCITY = self.x_reset_velocity
        DWELL = self.x_reset_dwell_time
        TIMEOUT = self.x_reset_timeout
        RATIO_THRESH = self.x_wall_ratio_threshold
        VEL_RATIO_THRESH = self.x_min_velocity_ratio
        WINDOW_MS = self.x_wall_window_ms
        ENABLE_ENC_RESET = bool(self.x_enable_encoder_reset)

        self._x_seeking_wall = True
        start = time.time()
        # 速度滑窗(用于速度对照判据)
        samples = []   # [(t, pos), ...] 最多保留 window_ms 内
        triggered_at = None
        start_pos = self.x_get_position()
        logger.info("reset_x: 开始,起点 x=%.4f m (ref=%.4f),_x_wall=%s"
                    % (start_pos, self._x_ref_encoder_at_zero or -1, self._x_wall))

        try:
            self.x_speed(VELOCITY)
            while True:
                # 急停优先
                estop = getattr(self, "_estop", None)
                if estop is not None and estop.is_set():
                    logger.warning("reset_x: 收到急停,中止找墙")
                    break
                # 全局超时:在循环顶部,任何分支外都生效 — 之前版本只在撞墙分支检查,
                # 导致车不动时不进任何分支 = 死循环,runtime 一直打 auto_init。
                t_now = time.time()
                if t_now - start > TIMEOUT:
                    logger.error(
                        "reset_x: 全局超时 %.1fs,推了 %.1fmm (期望 ≥ 50mm),强制停车"
                        % (TIMEOUT, abs(self.x_get_position() - start_pos) * 1000)
                    )
                    break
                cur = self.x_get_position()
                samples.append((t_now, cur))
                # 滑窗裁剪
                while samples and (t_now - samples[0][0]) * 1000.0 > WINDOW_MS:
                    samples.pop(0)
                # 至少要 window 充满才判定(避免单点抖动)
                if len(samples) < 3:
                    self.x_speed(VELOCITY)
                    time.sleep(0.01)
                    continue
                # 预触发距离闸:车必须真的走出 ≥ MIN_PRE_TRIGGER_DISP 才能判撞墙。
                # 否则电机扭矩不够编码器不动 → ratio=0 → 假撞墙,reset 提前误退出。
                if abs(self.x_get_position() - start_pos) < MIN_PRE_TRIGGER_DISP:
                    self.x_speed(VELOCITY)
                    time.sleep(0.01)
                    continue
                t_start, pos_start = samples[0]
                t_end, pos_end = samples[-1]
                dt = t_end - t_start
                if dt <= 0:
                    self.x_speed(VELOCITY)
                    time.sleep(0.01)
                    continue
                actual_disp = abs(pos_end - pos_start)
                expected_disp = abs(VELOCITY) * dt
                ratio = actual_disp / expected_disp if expected_disp > 1e-9 else 0.0
                # 判据 1: 位移比过低(撞墙/失步)
                hit_ratio = ratio < RATIO_THRESH
                # 判据 2: 速度比过低(补强)
                actual_speed = actual_disp / dt
                hit_speed = actual_speed < abs(VELOCITY) * VEL_RATIO_THRESH
                if hit_ratio or hit_speed:
                    if triggered_at is None:
                        triggered_at = t_now
                        # 撞墙瞬间:**直接 motor_x.set_linear(0)**,不走 x_speed。
                        # 原因:motor_280 是 DC 电机无刹车,x_speed(0) 仍会被
                        # x_speed 内部的 soft-limit / velocity_limit 链路处理后再下发;
                        # 而用户实测"顶到墙之后还会转"——说明 set_speed(0) 后电机仍
                        # 因 PID 闭环 / 串口响应延迟继续转几下。直接 set_linear(0)
                        # 绕过所有包装,MC602 立刻收到 0 命令 + 内部 PID 也立即停。
                        self.motor_x.set_linear(0)
                        logger.info(
                            "reset_x: 撞墙触发,r=%.2f v=%.4f → motor.set_linear(0) 硬停"
                            % (ratio, actual_speed)
                        )
                    elif t_now - triggered_at >= DWELL:
                        # 撞墙成功 → 相对零点:撞墙时的编码器值作为 ref,x_pose_start = ref
                        # 注:不再调 motor.reset() — ctl_id=2 走 encoder_2.reset 后电机可能被锁,
                        # 后续 set_angular 无视 → 推 0mm,看似撞墙实则锁死。相对零点更稳。
                        ref = self.motor_x.get_dis()
                        self.x_pose_start = ref
                        self._x_ref_encoder_at_zero = ref
                        # 物理清零后:编码器值就是 0,直接 x_pose_start = 0
                        if ENABLE_ENC_RESET:
                            self.x_pose_start = 0.0
                        else:
                            ref = self.motor_x.get_dis()
                            self.x_pose_start = ref
                        self.x_pose_now = 0
                        self.x_pose_last = 0
                        self._x_ref_encoder_at_zero = self.x_pose_start
                        self._x_expected_total_delta = 0.0
                        self._x_wall = "right"
                        logger.info(
                            "reset_x: 撞右墙+model判据(ratio=%.2f,speed=%.4f)+dwell通过,耗时%.2fs,推了%.1fmm,ref_encoder=%.6f"
                            % (ratio, actual_speed, time.time() - start,
                               abs(self.x_get_position() - start_pos) * 1000, ref)
                        )
                        # 双保险:最后再发一次 0,确保 return 前电机真停了
                        self.motor_x.set_linear(0)
                        self.x_speed(0)
                        return True
                    # dwell 中(已触发,等 dwell 满):保持硬停
                    self.motor_x.set_linear(0)
                    time.sleep(0.01)
                    continue
                # 未撞墙:持续推全速
                triggered_at = None  # 重置触发计时
                self.x_speed(VELOCITY)
                time.sleep(0.01)
                # 超时检查已移到循环顶部(对所有分支生效),这里不再重复。
        finally:
            self._x_seeking_wall = False
            self.x_speed(0)
        return False

    def hand_params_init(self, hand, hand2, grap):
        """
        初始化手部参数

        Args:
            hand: 手臂舵机配置
            hand2: 手部舵机配置
            grap: 抓取机构配置
        """
        # 手爪舵机(hand2)实际接在 bus_servo port=3(总线舵机协议,不是 PWM)。
        # 旧实现用 ServoPwm 调 hand2.port=2 → 协议错配,实际不发命令。
        # 改成 ServoBus(跟 arm_servo 同一协议)。
        self.hand_servo = ServoBus(hand2["port"])
        self.hand_angle_list2 = hand2["angle_list"]
        self.arm_servo = ServoBus(hand["port"])
        self.hand_angle_list = hand["angle_list"]
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


    def position_params_init(self, pose_enable, pose_horiz, pose_vert, side):
        """
        初始化位置参数

        Args:
            pose_enable: 是否启用位置
            pose_horiz: 水平位置
            pose_vert: 竖直位置
            side: 方向
        """
        self.pose_enable = pose_enable
        self.y_pose_start = (
            self.motor_y.get_dis() - pose_vert
        )
        self.y_pose_now = pose_vert
        self.x_pose_start = (
            self.motor_x.get_dis() - pose_horiz
        )
        self.x_pose_now = pose_horiz
        self.side = side

    def save_config(self, pose_enable=True):
        """
        保存配置到YAML文件

        Args:
            pose_enable: 是否启用位置
        """
        self.config["pos_cfg"] = {
            "pose_enable": pose_enable,
            "pose_horiz": self.x_pose_now,
            "pose_vert": self.y_pose_now,
            "side": self.side
        }
        with open(self.yaml_path, 'w') as stream:
            yaml.dump(self.config, stream, sort_keys=False)

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
        estop = getattr(self, "_estop", None)
        if estop is not None and estop.is_set():
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

        Args:
            velocity: 速度值
        """
        # === 急停门：外部置位急停时强制 0 ===
        estop = getattr(self, "_estop", None)
        if estop is not None and estop.is_set():
            velocity = 0
        # === 软限位边界自然停 ===
        # 旧实现用 _x_wall 撞墙门,在 x_wall 已知侧永远钳 velocity → move_x_position
        # 会立刻触发 x_stop_check 死循环(encoder 不动 → 触发 calibrate → 又设 _x_wall)。
        # 新实现:用软限位 [x_threshold[0], x_threshold[1]] 作为边界,边界处 velocity 强制 0。
        # 这样 move_x 自由推,reset_x(设 target=0.34 > x_threshold[1])也能撞过去。
        cur = self.x_get_position()
        x_lo, x_hi = self.x_threshold[0], self.x_threshold[1]
        BOUNDARY = 0.005  # 边界缓冲 5mm
        if cur >= x_hi - BOUNDARY and velocity > 0:
            velocity = 0  # 已到正向软上限
        elif cur <= x_lo + BOUNDARY and velocity < 0:
            velocity = 0  # 已到反向软下限
        # === 末段减速 / 顶段减速：根据当前位置分档限幅 ===
        # 注意:必须先分档限幅,最后再做 velocity_limit (主限幅)
        # 否则当 slow_velocity < velocity_limit 时,主限幅会把 slow 限制"放大"回去
        wall_pos = self.x_reset_target_m
        if self.x_slow_band_m > 0 and cur >= (wall_pos - self.x_slow_band_m):
            velocity = limit_val(velocity, -self.x_slow_velocity, self.x_slow_velocity)
        elif self.x_top_slow_m > 0 and cur <= -(self.x_top_slow_m):
            velocity = limit_val(velocity, -self.x_top_slow_velocity, self.x_top_slow_velocity)
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
        self.save_config()

    def reset_position(self):
        """
        重置机械臂位置
        """
        thread_reset_y = Thread(target=self.reset_y)
        thread_reset_x = Thread(target=self.reset_x)

        self.set_hand_angle("UP")
        self.set_arm_angle("RIGHT")
        thread_reset_y.daemon = True
        thread_reset_x.daemon = True
        thread_reset_y.start()
        thread_reset_x.start()
        thread_reset_y.join()
        thread_reset_x.join()
        self.x = 0
        self.y = 0
        self.save_config()

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

        # 控制上下限
        x_pos = limit_val(
            x,
            self.x_threshold[0],
            self.x_threshold[1]
        )
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

        # 开始移动前, 位置信息定义, 如果中间中断此时位置信息无用
        self.save_config(pose_enable=False)

        while True:
            # 到达结束标志结束
            if y_flag and x_flag:
                break
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

                # 重置初始化位置
                if self.y_reset_check():
                    if self.y_pid.setpoint <= self.y_pose_now:
                        y_flag = True
                        self.y_speed(0)
                    self.y_pose_start = self.motor_y.get_dis()
                    self.y_pose_now = 0
                    self.save_config()

            if not x_flag:
                if self.x_pid_moveto(x_pos):
                    self.x_speed(0)
                    x_flag = True

        self.save_config()
        # logger.debug(
        #     f"机械臂移动完成，当前位置状态: x: {self.x_pose_now:.4f}, y: {self.y_pose_now:.4f}, hand: {self.side}。 "
        # )
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
        # time.sleep(0.2)
        if arm is not None:
            self.set_arm_angle(arm)
            time.sleep(1)
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
    arm.reset_x()
    arm.x_move_to_position(0.1)
    print(arm.x_get_positon())
    arm.x_move_to_position(0.2)
    print(arm.x_get_positon())
    arm.x_move_to_position(0.3)
    print(arm.x_get_positon())   

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
