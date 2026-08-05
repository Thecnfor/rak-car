#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""CarRuntimeService 聚合类（原 runtime_service.py 的 1633 行按职责拆分）。

主类只保留：__init__（状态 + 5 个后台线程启动）、锁层次、realtime 转发
（_realtime_gate 同步路径）、状态 / 快照 / actions / infer 查询。
行为按职责拆到 4 个 mixin：
- ControllerWatcherMixin   控制器健康巡检
- LifecycleMixin           创建 / 复用 / 关闭 / 摄像头 / auto-init / wait-ready
- JobsMixin                双 worker 队列 / dispatch / 协作取消
- LoopsMixin               内存降档 / feed watchdog

`runtime.services.runtime_service` 保留为薄兼容层 re-export，`app.py` 及
任何外部 `from runtime.services.runtime_service import CarRuntimeService`
零改动。
"""
import logging
import queue
import threading
import time

from runtime.core import settings
from runtime.core.actions import ARM_ACTIONS, CAR_ACTIONS
from runtime.hardware.controller_session import get_controller_session
from runtime.services.inference_service import InferBackendService

from ._common import normalize_value
from .controller_watcher import ControllerWatcherMixin
from .jobs_mixin import JobsMixin
from .lifecycle_mixin import LifecycleMixin
from .loops_mixin import LoopsMixin

# 2026-07-17: runtime_service.py 用 logging.getLogger(__name__),但 uvicorn 没
# basicConfig,root logger 空 → logger.info() 静默被吞。
# 方案:basicConfig 只在 root 还没 handler 时生效(幂等),不影响 smartcar 自己的
# "my_logger" logger(have handler)。失败兜底保持原 behavior。
if not logging.getLogger().handlers:
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s %(filename)s, line %(lineno)d, %(levelname)s:%(message)s',
    )
logger = logging.getLogger(__name__)


class CarRuntimeService(
    ControllerWatcherMixin,
    LifecycleMixin,
    JobsMixin,
    LoopsMixin,
):
    def __init__(self):
        self.car = None
        self._car_class = None
        self._camera_cfg = None
        self.shared_front_camera = None
        self.shared_side_camera = None
        self.infer_service = InferBackendService()
        self.controller_session = get_controller_session()
        self.controller_generation = None
        # 锁层次重构（runtime 并发优化）：
        #   - `_ref_lock`：只保护 `self.car` 引用替换（init / recover / close）。
        #     持锁时间极短（只读 self.car 或写 self.car = ...），worker / 长动作都不应持它。
        #   - `_realtime_gate`：realtime 端点（set_wheel_speeds 等）入口处微秒级取 self.car 引用，
        #     真正的硬件调用在锁外执行，靠 SDK 的 `serial_mc602.lock` 串行字节流。
        #   - 旧 `car_lock`（RLock）已删除，保留同名 property 抛错，确保漏改的代码路径立即暴露。
        self._ref_lock = threading.Lock()
        self._realtime_gate = threading.Lock()
        # 两个独立 worker 队列（runtime 并发优化）：
        #   - arm_queue：机械臂长动作专用（arm.goto_position / move_xy 等 1-3s 闭环）。
        #     arm worker 卡在 PID 闭环里不影响 car_queue。
        #   - car_queue：底盘 / 任务 / system 短动作。短动作不被 arm 长动作排在同一个 worker 后面。
        # 两条队列物理隔离，互不阻塞；底层硬件字节流仍由 SDK 的 serial_mc602.lock 串行。
        self.job_lock = threading.Lock()
        self.jobs = {}
        # 2026-08-04：底盘三速"最后指令"缓存。lane_feed 的 forward/lateral/
        # angular_speed 字段从未被填（外环在客户端跑，车端只收轮速），调试页
        # 要显示底盘速度只能记最后一次 /v1/realtime/chassis-velocity 指令
        self._chassis_cmd_lock = threading.Lock()
        self._chassis_cmd = {"vx": None, "vy": None, "wz": None, "updated_at": None}
        # 2026-08-04：命令历史环（方向诊断用）：记录最近 60 条三速指令，
        # 用户复现"按键方向不对"时读 /v1/realtime/chassis/command 的 history
        # 对照物理行为，区分"页面发错"还是"车动错"。
        self._chassis_cmd_history = []
        # D.6 协作退出事件：与 jobs 字典分开存放，避免 JSON 序列化时碰到
        # `threading.Event`（不可序列化）抛错。key 与 jobs[job_id]["id"] 对齐。
        self.job_stop_events: dict = {}
        self.arm_queue: queue.Queue = queue.Queue()
        self.car_queue: queue.Queue = queue.Queue()
        self.arm_worker = threading.Thread(
            target=self._worker_loop, args=("arm",), daemon=True
        )
        self.car_worker = threading.Thread(
            target=self._worker_loop, args=("car",), daemon=True
        )
        self.arm_worker.start()
        self.car_worker.start()
        self.camera_lock = threading.Lock()
        self.init_lock = threading.Lock()
        self.initializing = False
        self.last_error = None
        self.last_init_at = None
        self.last_controller_probe = None
        self.current_job_id = None
        self.stop_after_action = settings.get_stop_after_action_default()
        self.auto_init_requested = False
        self.auto_heal_armed = settings.get_auto_init_on_start()
        self.auto_init_retry_interval = settings.get_auto_init_retry_interval()
        self.action_ready_timeout = settings.get_action_ready_timeout()
        self.action_ready_poll_interval = settings.get_action_ready_poll_interval()
        self.stream_service = None
        self.auto_init_kwargs = {
            "reset_arm": settings.get_reset_arm_on_auto_init(),
            "reset_position": settings.get_reset_position_on_init(),
        }
        self.auto_init_supervisor = threading.Thread(
            target=self._auto_init_loop,
            daemon=True,
        )
        self.auto_init_supervisor.start()
        self.camera_supervisor = None
        self.camera_supervisor_started = False
        # 2026-08-01：内存压力降档（详见 .trae/specs/system-arch-optimization/spec.md）。
        # ResourceProbeThread 每 30s 读 psutil RSS，超过阈值按既定顺序降档；恢复条件
        # 满足后反向恢复。feeds.degraded 字段记入 /v1/health。
        self._resource_lock = threading.Lock()
        self._feeds_degraded = []  # 公开字段（degraded list，按降档顺序追加）
        self._feed_default_hz = {
            "lane": 50.0,
            "arm": 20.0,
            "task": 30.0,
            "ir": 50.0,
            "odom": 50.0,
        }
        self._feed_current_hz = dict(self._feed_default_hz)
        # 降档顺序：ir → odom → arm → task → lane（lane 永不降档，但留 slot 兼容未来）
        self._degrade_order = ["ir", "odom", "arm", "task", "lane"]
        self._resource_probe_thread = threading.Thread(
            target=self._resource_probe_loop,
            name="rak-car-resource-probe",
            daemon=True,
        )
        self._resource_probe_thread.start()

        # 2026-08-01：feed watchdog。
        # lane_feed / task_feed / arm_feed 等守护线程必须 24x7 跑（外环依赖
        # lane_state 实时误差）；守护线程死后不会再自动起来，这条线程专门负责
        # 探测 + 自动 restart。
        # 策略：每 15s 检查一次，判定标准：
        #   - car.<feed>_feed_thread 不存在或 is_alive()=False → 立即 restart
        #   - car.<feed>_feed_health.alive=False 且 explicit stop（不可恢复）→ 不动
        #   - health.last_iter_at 距今 > 5s（说明 feed 卡在推理 / ZMQ 永久 EAGAIN）
        #     → restart（stop_event + 新线程）。这种情况通常意味着 ClintInterface
        #     socket 永久死掉了，重建 client 是最干净的修复。
        # env: RAK_CAR_FEED_WATCHDOG_INTERVAL_S（默认 15.0）
        self._feed_watchdog_thread = threading.Thread(
            target=self._feed_watchdog_loop,
            name="rak-car-feed-watchdog",
            daemon=True,
        )
        self._feed_watchdog_thread.start()

    @property
    def car_lock(self):
        """已废弃：长动作不应再全程持锁。

        新代码请用：
          - `_ref_lock` — init / recover / 改 self.car 引用的入口
          - `_realtime_gate` — realtime 端点入口微秒级取 self.car 引用
        如果你的代码原本 `with self.car_lock:` 包住一长段动作（典型场景：worker 跑
        `arm.move_xy` 1-3s 闭环），请改成 `with self._ref_lock:` 在最外圈瞬时拿
        到 car 引用后立即释放。
        """
        raise RuntimeError(
            "car_lock 已废弃,长动作不应再持锁。请改用 self._ref_lock（init/引用替换）"
            " 或 self._realtime_gate（realtime 入口）。"
            " grep 'self.car_lock' 找遗留。"
        )

    def set_stream_service(self, stream_service):
        self.stream_service = stream_service

    # === 实时硬件控制（_realtime_gate 同步路径，绕过 job_queue，50Hz 友好） ===
    #
    # B 改造：所有 realtime 端点不再持 RLock，只在入口处瞬时取 self.car 引用。
    # 硬件字节流仍由 SDK 的 `serial_mc602.lock` 串行。
    # 这样 arm 长动作（move_xy 1-3s 闭环）不再挡住 lane 外环的 set_wheel_speeds。

    def _realtime_check_locked(self):
        if self.car is None:
            raise RuntimeError("car 未初始化")

    def set_wheel_speeds(self, speeds):
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.set_wheel_speeds(speeds)

    def set_chassis_velocity(self, vx, vy, wz, duration=None):
        """
        上层外环专用：(vx, vy, wz) → 4 轮速直发，绕开 set_velocity 的里程计耦合。

        走 _realtime_gate 同步路径，50Hz 友好。里程计照常由 odometer_thread 自动更新
        （它读 wheels_chassis.get_linear()）。
        """
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        vx = float(vx)
        vy = float(vy)
        wz = float(wz)
        # 直接复用 chassis 的 IK（避免重复实现 mecanum 公式）
        wheel_speeds = list(
            car.chassis.calculate_wheel_velocities(vx, vy, wz)
        )
        # 直发轮速，绕开 set_velocity（set_velocity 会反复 lock + set）
        car.wheels_chassis.set_linear([float(s) for s in wheel_speeds])
        with self._chassis_cmd_lock:
            self._chassis_cmd = {
                "vx": vx, "vy": vy, "wz": wz, "updated_at": time.time(),
            }
            self._chassis_cmd_history.append(
                {"vx": vx, "vy": vy, "wz": wz, "t": time.time()}
            )
            if len(self._chassis_cmd_history) > 60:
                del self._chassis_cmd_history[: len(self._chassis_cmd_history) - 60]
        return {
            "vx": vx,
            "vy": vy,
            "wz": wz,
            "duration": duration,
            "wheel_speeds": wheel_speeds,
        }

    def set_arm_velocity(self, x_vel=None, y_vel=None, arm_angle=None, hand_angle=None):
        """arm 4-DOF 实时命令 — 绕开 arm_queue, 供视觉伺服连续追踪用。

        与 set_chassis_velocity 同构: _realtime_gate 微秒级取 car 引用, 直发,
        不进 job_queue、不持 car_lock。

        支持的轴 (全部可选, None = 该轴不动):
          - x_vel / y_vel: 十字滑台速度 (m/s), 走 arm_base.x_speed / y_speed
          - arm_angle:     大臂角度 (°), 走 set_arm_angle (舵机异步转到位)
          - hand_angle:    手抓角度 (°), 走 set_hand_angle (舵机异步转到位)

        角度软限位 (2026-08-01 用户约定):
          - 大臂: [-90, +90]  (-90=朝 x 左, +90=朝 x 右)
          - 手抓: [-90, 0]    (-90=看正面/水平, 0=朝下)
        十字速度软限位: x ∈ [-0.30, 0] m, y ∈ [-0.20, 0] m (触底=0)。

        注意: set_arm_angle/set_hand_angle 立即返回 (舵机异步转), 不阻塞;
        但读位置 x_get_position/y_get_position 走串口 (~ms), 高频时建议 <=20Hz。
        y 额外有磁感安全门 (arm_base 内置)。
        """
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        out = {}
        # 角度软限位 (°) — arm 对齐业务层 [-150, +150] (2026-08-06).
        # 实时路径走 _realtime_gate 不进 job_queue, 跳过 main/arm/api/safety.py 校验,
        # 这里的常量是 main/arm/api/safety.py:SafetyMixin._ARM_ANGLE_MIN/_MAX/_HAND_ANGLE_MIN/_MAX 的镜像;
        # runtime 与 main 是两个独立包, 不跨包 import 常量, 改任何一边另一处必须同步.
        ARM_MIN, ARM_MAX = -150.0, 150.0
        HAND_MIN, HAND_MAX = -90.0, 10.0
        # 十字速度软限位 (m)
        X_MIN_M, X_MAX_M = -0.30, 0.0
        Y_MIN_M, Y_MAX_M = -0.20, 0.0
        if arm_angle is not None:
            arm_angle = max(ARM_MIN, min(ARM_MAX, float(arm_angle)))
            car.arm.set_arm_angle(arm_angle, speed=80)
            out["arm_angle"] = arm_angle
        if hand_angle is not None:
            hand_angle = max(HAND_MIN, min(HAND_MAX, float(hand_angle)))
            car.arm.set_hand_angle(hand_angle, speed=80)
            out["hand_angle"] = hand_angle
        if x_vel is not None:
            x_vel = float(x_vel)
            try:
                x_pos = car.arm.x_get_position()
            except Exception:
                x_pos = None
            if x_pos is not None:
                if x_vel > 0 and x_pos >= X_MAX_M:
                    x_vel = 0.0
                elif x_vel < 0 and x_pos <= X_MIN_M:
                    x_vel = 0.0
            car.arm.x_speed(x_vel)
            out["x_vel"] = x_vel
        if y_vel is not None:
            y_vel = float(y_vel)
            try:
                y_pos = car.arm.y_get_position()
            except Exception:
                y_pos = None
            if y_pos is not None:
                if y_vel > 0 and y_pos >= Y_MAX_M:
                    y_vel = 0.0
                elif y_vel < 0 and y_pos <= Y_MIN_M:
                    y_vel = 0.0
            car.arm.y_speed(y_vel)
            out["y_vel"] = y_vel
        return out

    def get_wheel_encoders(self):
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.get_wheel_encoders()

    def get_chassis_command(self):
        """最后一次 /v1/realtime/chassis-velocity 指令缓存 + 最近历史（诊断用）。"""
        with self._chassis_cmd_lock:
            out = dict(self._chassis_cmd)
            out["history"] = list(self._chassis_cmd_history)
            return out

    def get_lane_state(self):
        """外环专用：读 streamer 缓存的 lane_state。

        数据来源：`lane_feed` 守护线程（runtime 启动后默认 50Hz）通过
        `car.streamer.set_lane_state(...)` 持续刷新的内存缓存。

        不进 job_queue、不打 ZMQ、不抢任何 runtime 锁——只取 `meta_lock`（极快）。
        因此 50Hz+ 外环轮询安全，不会和 lane_feed 守护线程或 MJPEG 推流抢锁。

        `stream_service` 由 `runtime.api.app` 在路由注册前 `set_stream_service`
        注入，正常启动后不会为 None；若 runtime 尚未注入则返回 503。
        """
        if self.stream_service is None:
            raise RuntimeError("stream_service 未注入（runtime 启动异常）")
        return self.stream_service.get_lane_state()

    def get_arm_state(self):
        """调试/UI 专用：读 streamer 缓存的 arm_state（机械臂 y/x 位置）。

        数据来源：`arm_feed` 守护线程（runtime 启动后默认 20Hz）通过
        `car.streamer.set_arm_state(...)` 持续刷新的内存缓存。

        不进 job_queue、不打 ZMQ、不抢任何 runtime 锁——只取 `meta_lock`（极快）。
        20Hz+ 轮询安全。
        """
        if self.stream_service is None:
            raise RuntimeError("stream_service 未注入（runtime 启动异常）")
        return self.stream_service.get_arm_state()

    def get_task_state(self):
        """边走边看专用：读 streamer 缓存的 task_state（侧摄目标检测）。

        数据来源：`task_feed` 守护线程（runtime 启动后默认 30Hz）通过
        `car.streamer.set_task_state(...)` 持续刷新的内存缓存。

        不进 job_queue、不打 ZMQ、不抢任何 runtime 锁——只取 `meta_lock`（极快）。
        让业务层"边走边看"侧摄目标成为可能（之前 /v1/vision/task 是 sync 5-15s 阻塞）。

        返回字段：
          - active: bool (task_feed 是否在跑)
          - mode: str ("task_feed" / "tracking" / "idle" / "stopped")
          - detections: list[{cls_id, det_id, label, score, bbox_norm{...}}]
          - count: int
          - updated_at: float (unix time)
        """
        if self.stream_service is None:
            raise RuntimeError("stream_service 未注入（runtime 启动异常）")
        return self.stream_service.get_task_state()

    def get_ir_state(self):
        """外环/触发判定专用：读 streamer 缓存的 ir_state（左右 IR 距离）。

        数据来源：`ir_feed` 守护线程（runtime 启动后默认 50Hz）通过
        `car.streamer.set_ir_state(...)` 持续刷新的内存缓存。

        不进 job_queue、不打 ZMQ、不抢任何 runtime 锁——只取 `meta_lock`（极快）。
        50Hz+ 轮询安全，与 lane_feed / arm_feed / task_feed 同构。

        返回字段：
          - active: bool (ir_feed 是否在跑)
          - mode: str ("ir_feed" / "idle" / "stopped")
          - left: float | None (m,用户视角左)
          - right: float | None (m,用户视角右)
          - updated_at: float (unix time)
        """
        if self.stream_service is None:
            raise RuntimeError("stream_service 未注入（runtime 启动异常）")
        return self.stream_service.get_ir_state()

    def get_odom_state(self):
        """外环/触发判定专用：读 streamer 缓存的 odom_state（底盘里程计）。

        数据来源：`odom_feed` 守护线程（runtime 启动后默认 50Hz）通过
        `car.streamer.set_odom_state(...)` 持续刷新的内存缓存。

        不进 job_queue、不打 ZMQ、不抢任何 runtime 锁——只取 `meta_lock`（极快）。
        50Hz+ 轮询安全，与 lane_feed / ir_feed 同构。

        返回字段：
          - active: bool (odom_feed 是否在跑)
          - mode: str ("odom_feed" / "idle" / "stopped")
          - x, y, theta: float | None (m, m, rad)
          - distance: float | None (m,本轮累积行驶距离)
          - updated_at: float (unix time)
        """
        if self.stream_service is None:
            raise RuntimeError("stream_service 未注入（runtime 启动异常）")
        return self.stream_service.get_odom_state()

    # === 2026-07-31：IR / odometer 同步直读（_realtime_gate 路径,绕过 job_queue） ===
    #
    # 用于 main 业务层在没有 feed 缓存可用时（feed 未启动 / 异常退出）的 fallback。
    # 也用于 arm 长动作期间需要再拉一次"最新未缓存值"的场景。
    def get_ir_distance_sync(self, side="left"):
        """step B：单次读 IR,走 _realtime_gate,不进 job_queue。

        实测延迟:HTTP RTT + (1-2 次 MC602 字节往返 ~10-30ms)。
        """
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.get_ir_distance(side=side)

    def get_all_ir_distance_sync(self):
        """step B：单次读两侧 IR,走 _realtime_gate,不进 job_queue。"""
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.get_all_ir_distance()

    def get_odometry_sync(self, show_info=False):
        """step B：单次读里程计,走 _realtime_gate,不进 job_queue。

        实测延迟:几乎只有 HTTP RTT（get_odometry 内部 _lock 微秒级），
        比走 job_queue 快一个数量级。
        """
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.get_odometry(show_info=show_info)

    def get_distance_sync(self):
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.get_distance()

    def get_feed_status(self):
        """2026-07-31: 给 /v1/health 用的 feed summary（一次性扫所有 cache）。

        返回:{lane_state, arm_state, task_state, ir_state, odom_state} 各自的 active / mode / updated_at。
        调试用,不抢任何锁,只读 streamer meta_lock 5 次。

        2026-08-01：附加 feed health 心跳(线程 alive + last_iter_at + err_count),
        watchdog 巡检后能在 /v1/health 里直接看 lane/task feed 是否被自动 restart 过。
        """
        if self.stream_service is None:
            return {"ir_state": {}, "odom_state": {}, "lane_state": {}, "arm_state": {}, "task_state": {}}
        out = {}
        for name, fn in (
            ("lane_state", self.stream_service.get_lane_state),
            ("arm_state", self.stream_service.get_arm_state),
            ("task_state", self.stream_service.get_task_state),
            ("ir_state", self.stream_service.get_ir_state),
            ("odom_state", self.stream_service.get_odom_state),
        ):
            try:
                st = fn()
                out[name] = {
                    "active": st.get("active"),
                    "mode": st.get("mode"),
                    "updated_at": st.get("updated_at"),
                }
            except Exception:
                out[name] = {"active": False, "mode": "idle", "updated_at": None}
        # 2026-08-01：附加 health 心跳。car 引用可能在 init / recover 切换
        # 瞬间变 None — getattr default 安全。
        try:
            car = self.car
        except Exception:
            car = None
        if car is not None:
            health_map = {
                "lane_state": getattr(car, "_lane_feed_health", None),
                "task_state": getattr(car, "_task_feed_health", None),
                "arm_state":  getattr(car, "_arm_feed_health", None),
                "ir_state":   getattr(car, "_ir_feed_health", None),
                "odom_state": getattr(car, "_odom_feed_health", None),
            }
            for feed_name, health in health_map.items():
                if health is None:
                    continue
                out[feed_name]["health"] = {
                    "alive": health.get("alive"),
                    "iter_count": health.get("iter_count"),
                    "ok_count": health.get("ok_count"),
                    "err_count": health.get("err_count"),
                    "last_iter_at": health.get("last_iter_at"),
                    "last_ok_at": health.get("last_ok_at"),
                    "last_err": health.get("last_err"),
                    "last_err_at": health.get("last_err_at"),
                }
        return out

    def set_single_motor(self, port, speed, reverse=1):
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.set_single_motor(port, speed, reverse=reverse)

    def get_encoder(self, port, reverse=1):
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.get_encoder(port, reverse=reverse)

    def set_stepper_rad(self, port, rad, time=0.5, reverse=1, perimeter=0.008):
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.set_stepper_rad(port, rad, time, reverse, perimeter)

    def set_bus_servo(self, port, angle, speed=100):
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.set_bus_servo(port, angle, speed)

    def read_bus_servo(self, port):
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.read_bus_servo(port)

    def read_analog(self, port):
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.read_analog(port)

    def read_analog2(self, port):
        with self._realtime_gate:
            self._realtime_check_locked()
            car = self.car
        return car.read_analog2(port)

    def emergency_stop(self):
        # 关键：不持 car_lock。worker 跑长动作（reset_position / move_* / 巡线）时
        # car_lock 被占，若这里也抢 car_lock 会排队等长动作结束 → 急停失效。
        # 停车指令走串口层自带锁，与 worker 并发安全；car.emergency_stop 内部
        # 置 _hardware_stop / _estop 让正在跑的循环协作退出，并直接停三轴。
        # **不**置 _stop_flag → 上位机视觉/推理 (lane_feed / arm_feed / task_feed) 不受影响。
        car = self.car
        if car is None:
            return False
        try:
            return bool(car.emergency_stop())
        except AttributeError:
            # 兼容旧 car（无 emergency_stop 方法）：至少置 _hardware_stop + 停底盘
            car._hardware_stop = True
            try:
                car.stop()
            except Exception:
                pass
            return True

    def reset_stop_flag(self):
        # 同样不持 car_lock，保证急停恢复不被卡住的长动作阻塞
        car = self.car
        if car is None:
            return False
        try:
            return bool(car.clear_stop())
        except AttributeError:
            car._stop_flag = False
            return True

    def set_stop_mode(self, enabled):
        self.stop_after_action = bool(enabled)
        with self._realtime_gate:
            car = self.car
        if car is not None:
            car.STOP_PARAM = self.stop_after_action
        return self.stop_after_action

    def get_state(self):
        controller_snapshot = self._sync_controller_health_state()
        with self.job_lock:
            jobs = list(self.jobs.values())
        queued_count = sum(job["status"] == "queued" for job in jobs)
        infer_snapshot = self.infer_service.get_state()
        camera_snapshot = (
            self.stream_service.get_status() if self.stream_service is not None else None
        )
        return {
            "initialized": self._is_car_ready(controller_snapshot),
            "initializing": self.initializing,
            "last_error": self.last_error,
            "last_init_at": self.last_init_at,
            "current_job_id": self.current_job_id,
            "queued_jobs": queued_count,
            "stop_after_action": self.stop_after_action,
            "stop_flag": getattr(self.car, "_stop_flag", None) if self.car else None,
            "streamer_url": settings.get_public_stream_base(),
            "controller_probe": self.last_controller_probe,
            "controller_session": controller_snapshot,
            "infer_service": infer_snapshot,
            "camera_stream": camera_snapshot,
            # 2026-07-31：feed 守护线程状态（lane / arm / task / ir / odom）。
            # 给 /v1/health 调试用，一眼看清 feed 是否卡住 / 卡了哪个。
            "feeds": {
                **self.get_feed_status(),
                # 2026-08-01：当前被 ResourceProbeThread 降档的 feed 列表，
                # 顺序即降档触发顺序（ir → odom → arm → task）。
                "degraded": self.get_feeds_degraded(),
            },
            # 2026-08-01：runtime 进程内存画像（psutil RSS），给 /v1/health 用。
            "resource": {
                "rss_mb": self._read_self_rss_mb(),
                "pressure_mb": settings.get_car_memory_pressure_mb(),
                "hard_limit_mb": settings.get_car_rss_limit_mb(),
            },
            "components": {
                "controller": {
                    "ready": controller_snapshot.get("state") == "PROGRAM_READY",
                    "state": controller_snapshot.get("state"),
                    "mode": controller_snapshot.get("mode"),
                    "detail": (controller_snapshot.get("last_probe") or {}).get("detail")
                    or controller_snapshot.get("detail"),
                },
                "infer": {
                    "ready": infer_snapshot.get("status") == "ready",
                    "state": infer_snapshot.get("status"),
                    "detail": infer_snapshot.get("last_error"),
                },
                "camera": {
                    "ready": bool(camera_snapshot and camera_snapshot.get("active_cams")),
                    "state": camera_snapshot.get("status") if camera_snapshot else "unknown",
                    "detail": camera_snapshot.get("cameras") if camera_snapshot else None,
                },
            },
        }

    def get_runtime_snapshot(self):
        with self._ref_lock:
            car = self.car
        if car is None:
            return None
        return {
            "odometry": normalize_value(car.get_odometry()),
            "distance": normalize_value(car.get_distance()),
            "stop_after_action": car.STOP_PARAM,
            "stop_flag": car._stop_flag,
        }

    def get_car_for_stream(self):
        # 不持 car_lock：capture_loop 在机械臂长动作期间会被卡住 5+s，
        # 导致 stream 缓存不更新,前端 MJPEG/frame 一直吐最后那帧(画面卡死)。
        # self.car 是单实例属性，读取是 GIL 原子操作,无撕裂风险。
        # 切换 self.car 仅发生在 init/recover 流程,且会同步停止 capture_loop 一拍。
        return self.car

    def list_actions(self):
        # 2026-07-16 删 "task": 任务逻辑由 main 编排，runtime 只暴露 car/arm action。
        return {
            "car": sorted(CAR_ACTIONS.keys()),
            "arm": sorted(ARM_ACTIONS.keys()),
            "system": [
                "init",
                "close",
                "set_stop_mode",
                "reset_stop_flag",
                "emergency_stop",
            ],
        }

    def get_infer_state(self):
        return self.infer_service.get_state()

    def infer_drop_oldest(self, timeout_s=None):
        """2026-08-01：让推理后端按 LRU 卸载非 eager 模型（详见 system-arch-optimization spec）。

        通过每个后端端口发 DROPX! 命令实现；前端 /v1/infer/drop-oldest 入口。
        """
        try:
            results = self.infer_service.drop_oldest(timeout_s=timeout_s)
        except Exception as exc:  # pragma: no cover
            return {"ok": False, "error": str(exc), "results": []}
        return {"ok": True, "results": results}
