#!/usr/bin/python
# -*- coding: utf-8 -*-
"""MyCar 聚合类：`class MyCar(MecanumDriver, *Mixins)`。

原来 3059 行的 my_car.py 按职责拆成 mixin 子模块（用户决策：FeedsMixin 继承），
本文件只保留聚合类骨架 + 生命周期（__init__ / close / camera_init）——这两处
必须留在最外层聚合类：`super(MyCar, self).close()` 要沿 MRO 命中 MecanumDriver，
放进 mixin 会跳过它。

子模块一览：
- pid.py            纯 PID 类 + limit（无状态）
- state_mixin      急停 / 协作取消 / arm_state / 调试 / walk_lane_test
- sensors_mixin    传感器 / 存储仓 / 射击 / 灯光 / 数码管 / IR
- hardware_io_mixin 实时硬件控制（底盘速度 / 电机 / 编码器 / 舵机 / 模拟量）
- detection_mixin  视觉推理 / OCR / ERNIE / 检测结果
- motion_mixin     运动 / 车道保持 / 目标定位
- feeds            5 个守护线程缓存（lane / arm / task / ir / odom）

注意：模块级遗留（filter_chinese_letter / sellect_program / kill_other_python /
test_for_animal / `__main__` 块）是无调用死代码，随模块→包迁移被移除
（Phase 4 清理项）。
"""
import os
import threading
import time

from smartcar import Camera, logger
from smartcar.whalesbot.tools import get_yaml
from smartcar.whalesbot.vehicle import (
    ArmController,
    Beep,
    MecanumDriver,
    ScreenShow,
)

from .detection_mixin import DetectionMixin
from .feeds import FeedsMixin
from .hardware_io_mixin import HardwareIOMixin
from .motion_mixin import MotionMixin
from .sensors_mixin import SensorsMixin
from .state_mixin import StateMixin

__all__ = ["MyCar"]


class MyCar(
    MecanumDriver,
    StateMixin,
    SensorsMixin,
    HardwareIOMixin,
    DetectionMixin,
    MotionMixin,
    FeedsMixin,
):
    """
    智能车控制类

    该类继承自MecanumDriver，实现了智能车的完整控制功能，包括传感器初始化、PID控制、摄像头控制、
    目标检测、车道保持等功能。
    """

    STOP_PARAM: bool = True

    def __init__(self, cap_front=None, cap_side=None, streamer=None):
        """
        初始化智能车

        初始化智能车的各个组件，包括底盘、传感器、摄像头、PID控制器等。
        """
        # 调用继承的初始化
        start_time = time.time()
        super(MyCar, self).__init__()
        logger.info("my car init ok {}".format(time.time() - start_time))
        # 显示
        self.display = ScreenShow()

        self._shared_cap_front = cap_front
        self._shared_cap_side = cap_side
        self._owns_cameras = cap_front is None and cap_side is None
        self._owns_streamer = streamer is None
        self.streamer = streamer
        self.arm = ArmController()

        # 获取 rak-car 根目录（runtime/services 的上两级）
        # car_wrap_2026.py 原在根目录，现在搬到 runtime/services/，config_car.yml 仍在根目录
        self.path_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
        self.yaml_path = os.path.join(self.path_dir, "config_car.yml")
        # 获取配置
        cfg = get_yaml(self.yaml_path)
        # 根据配置设置sensor
        self.sensor_init(cfg)

        self.car_pid_init(cfg)
        self.ring = Beep()
        self.camera_init(cfg)
        # paddle推理初始化
        self.paddle_infer_init()
        # 文心一言分析初始化
        self.ernie_bot_init()

        # 相关临时变量设置
        # 程序结束标志
        self._stop_flag = False
        # 急停事件：置位后 arm 的 y_speed/x_speed 被 chokepoint 强制 0，
        # 正在跑的 reset_y / move_* 循环因电机不动而收敛退出。
        # 与 _stop_flag 一起构成"协作式取消"，可被 runtime 无锁抢占。
        self._estop_event = threading.Event()
        self.arm._estop = self._estop_event
        # 硬件急停标志:emergency_stop() 触发;与 _stop_flag 不同 —— 硬件 loop 必须
        # 响应(让底盘/arm 停转),但 lane_feed/arm_feed/task_feed 守护线程不受影响,
        # 上位机视觉/推理持续对外提供数据流。需要 reset_stop 复位。
        self._hardware_stop = False
        # 物理按键板 2026 年未启用（仅 arm jog 用，不再触发 _stop_flag）。
        self._end_flag = False

        # lane 误差缓存守护线程：客户端外环用，车端不动轮速
        self._lane_feed_thread = None
        self._lane_feed_stop = None
        self._lane_feed_lock = threading.Lock()
        # 2026-07-31：左右 IR 距离缓存守护线程（与 _lane_feed_* 同构）。
        # 数据源 MyCar.get_all_ir_distance() —— 内部是 MC602 字节往返。
        # 后台跑后 main 业务层 /v1/realtime/ir/state 几乎是 0 队列延迟 + 0 car_lock。
        self._ir_feed_thread = None
        self._ir_feed_stop = None
        self._ir_feed_lock = threading.Lock()
        # 2026-07-31：底盘里程计缓存守护线程（与 _lane_feed_* 同构）。
        # 数据源 MyCar.get_odometry() + get_distance() —— 内存读，廉价但同步路径
        # 抢 _ref_lock + job_queue，慢；后台喂 50Hz 后业务层读 0 延迟。
        self._odom_feed_thread = None
        self._odom_feed_stop = None
        self._odom_feed_lock = threading.Lock()

        self.beep()

    def camera_init(self, cfg):
        """
        初始化摄像头

        根据配置初始化前置摄像头和侧面摄像头。

        参数:
            cfg: 配置字典，包含摄像头的配置信息
        """
        # 初始化前后摄像头设置
        if self._shared_cap_front is not None:
            self.cap_front = self._shared_cap_front
        else:
            self.cap_front = Camera(cfg["camera"]["front"])
        # 侧面摄像头
        if self._shared_cap_side is not None:
            self.cap_side = self._shared_cap_side
        else:
            self.cap_side = Camera(cfg["camera"]["side"])

    def close(self):
        """
        关闭方法

        关闭所有线程和资源，包括按键线程、摄像头和流处理器。
        """
        self._stop_flag = False
        self._end_flag = True
        # 2026-08-01 修复: 原来只 stop_lane_feed(且不传 force → NOOP), 漏停
        # arm/task/ir/odom 四个守护线程。auto-init 重建 MyCar 时旧 feed 线程继续跑,
        # 与新 car 的 feed 线程双写 streamer 缓存 → arm_state 交替正常/异常。
        # close 是销毁整个实例, 必须 force=True 真正停掉全部 5 个 feed 线程。
        # (stop_*_feed 默认 force=False 是 no-op, 见 feeds.py _stop_feed)
        for _feed in ("lane_feed", "arm_feed", "task_feed", "ir_feed", "odom_feed"):
            try:
                getattr(self, "stop_%s" % _feed)(force=True)
            except Exception:
                pass
        # 按键线程已移除
        try:
            super(MyCar, self).close()
        except Exception:
            pass
        if self._owns_cameras:
            self.cap_front.close()
            self.cap_side.close()
        if self._owns_streamer and self.streamer is not None:
            self.streamer.stop()
        # self.grap_cam.close()
