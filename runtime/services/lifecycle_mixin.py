#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""生命周期 Mixin（从 runtime_service.py 拆出）。

负责 MyCar 的创建 / 复用 / 关闭、共享摄像头、auto-init 自愈循环、
`_wait_until_ready` / `_recover_controller_runtime` 编排。依赖聚合类提供
`self.controller_session`、`self._ref_lock`、`self.init_lock` 等属性，以及
`ControllerWatcherMixin` 的 `_ensure_controller_ready` / `_is_car_ready` /
`_probe_controller` / `_sync_controller_health_state` 等。
"""
import gc
import logging
import os
import threading
import time
import traceback
from pathlib import Path

import yaml

from ._common import _debug_emit

logger = logging.getLogger(__name__)


class LifecycleMixin:
    """CarRuntimeService 的 MyCar 生命周期行为。"""

    def start_background_services(self):
        self.infer_service.start_background()
        with self.camera_lock:
            if self.camera_supervisor_started:
                return
            self.camera_supervisor_started = True
            self.camera_supervisor = threading.Thread(
                target=self._camera_supervisor_loop,
                daemon=True,
            )
            self.camera_supervisor.start()

    def _camera_supervisor_loop(self):
        while True:
            try:
                self.ensure_shared_cameras()
                return
            except Exception:
                time.sleep(1.0)

    def _load_camera_cfg(self):
        if self._camera_cfg is not None:
            return self._camera_cfg
        config_path = Path(__file__).resolve().parents[2] / "config_car.yml"
        with config_path.open("r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file) or {}
        camera_cfg = config.get("camera") or {}
        self._camera_cfg = {
            "front": int(camera_cfg.get("front", 1)),
            "side": int(camera_cfg.get("side", 2)),
        }
        return self._camera_cfg

    def ensure_shared_cameras(self):
        with self.camera_lock:
            cfg = self._load_camera_cfg()
            if self.shared_front_camera is None:
                from smartcar.whalesbot.tools.camera import Camera

                self.shared_front_camera = Camera(cfg["front"])
            if self.shared_side_camera is None:
                from smartcar.whalesbot.tools.camera import Camera

                self.shared_side_camera = Camera(cfg["side"])
            return self.shared_front_camera, self.shared_side_camera

    def _close_shared_cameras_locked(self):
        unique_cameras = []
        for camera in (self.shared_front_camera, self.shared_side_camera):
            if camera is not None and camera not in unique_cameras:
                unique_cameras.append(camera)
        self.shared_front_camera = None
        self.shared_side_camera = None
        for camera in unique_cameras:
            try:
                camera.close()
            except Exception:
                pass

    def _remember_shared_cameras(self, car):
        if car is None:
            return
        if self.shared_front_camera is None:
            self.shared_front_camera = getattr(car, "cap_front", None)
        if self.shared_side_camera is None:
            self.shared_side_camera = getattr(car, "cap_side", None)

    def get_stream_camera(self, cam_id):
        cam_name = str(cam_id).lower()
        if cam_name in {"cam1", "front"}:
            return self.shared_front_camera
        if cam_name in {"cam2", "side"}:
            return self.shared_side_camera
        return None

    def _ensure_feeds_running(self, car):
        """幂等启动循线和触发所需的低成本缓存 feed。

        arm/task feed 需要摄像头或串口高频轮询，改为由订阅或任务显式启动，
        避免 runtime 初始化后常驻占用推理和串口资源。
        """
        try:
            car.start_lane_feed(
                hz=float(os.environ.get("RAK_CAR_LANE_FEED_HZ", "30"))
            )
        except Exception as exc:
            logger.warning("lane_feed auto-start failed: {}".format(exc))
        try:
            car.start_ir_feed(
                hz=float(os.environ.get("RAK_CAR_IR_FEED_HZ", "20"))
            )
        except Exception as exc:
            logger.warning("ir_feed auto-start failed: {}".format(exc))
        try:
            car.start_odom_feed(
                hz=float(os.environ.get("RAK_CAR_ODOM_FEED_HZ", "20"))
            )
        except Exception as exc:
            logger.warning("odom_feed auto-start failed: {}".format(exc))

    def _create_car_locked(self, reset_arm=False, reset_position=True):
        session = self._ensure_controller_ready()
        # 2026-08-03: 必须先保证模块级 singleton serial_wrap 已经稳定连接,
        # 否则 MyCar.__init__ 走到 ArmController.y_params_init → motor_y.get_dis()
        # → serial.get_anwser 会立刻 raise ControllerNotReadyError("控制器未连接"),
        # mark_offline 回调把 controller_session 设回 DISCONNECTED, 进死循环。
        # controller_session.ensure_ready 内的 sync_with_probe 一进来就 _close_locked
        # 重置 connect_flag, singleton 自己又有 auto_connect 异步重连,两条路径相互踩;
        # 这里 poll 等待 singleton 在调用方抢到 _ref_lock 之前稳定到 connect_flag=True。
        self._wait_serial_wrap_ready(timeout=5.0)
        # 2026-08-03: 构造 MyCar 期间禁 GC。GC finalizer 可能 finalize 泄漏的
        # zmq.Context, __del__ → destroy() → term() 永等未关 socket —— 若卡在构造
        # 线程里, initializing 永真, 自愈循环整体瘫痪 (证据:
        # .dbg/mc602-download-stuck-pyspy-20260803.txt)。MyCar.close 已显式清理
        # zmq, 这里是第二层保险: finalizer 推迟到构造结束后再跑, 卡也卡不到 init。
        _gc_was_enabled = gc.isenabled()
        gc.disable()
        try:
            car = self._get_car_class()(
                cap_front=self.shared_front_camera,
                cap_side=self.shared_side_camera,
                streamer=self.stream_service,
            )
        finally:
            if _gc_was_enabled:
                gc.enable()
        self._remember_shared_cameras(car)
        car.STOP_PARAM = self.stop_after_action
        # 2026-08-08: 蜂鸣是装饰性动作, 蜂鸣器无响应会 raise 把 init 卡死
        # (实测: 每 ~3s 重建 MyCar, initialized 永 False, 无任何进度日志)。
        # 包一层 try, 失败只 warning 不阻断 init。
        try:
            car.beep()
        except Exception as exc:
            logger.warning("init beep 失败(继续): %s" % exc)
        time.sleep(1)
        # 2026-08-03 init 新顺序（用户要求）：
        #   ① 存储仓归位 (舵机 close 到 98°)
        #   ② reset-all (含 y 校准 + x 撞墙 ‖ 大臂 +90° ‖ 手爪 UP)
        #   ③ 里程计清零 (init_car_position)
        # 关键设计:move_y_position 必须 in reset_y 之后,
        # 因此 reset_y 放在 ② (reset_all, do_reset_y=True) 里, 后续 move_y
        # 才能拿到正确的 y_pose_start; ① 不动 y 轴。
        init_x_v = float(os.getenv("RAK_CAR_RESET_X_VELOCITY", "0.05"))
        # ① 存储仓归位 (仅舵机 close 到 98°, 不动 y 轴)
        try:
            car.set_storage_angle(98, speed=5)
            logger.info("init storage close (98°) 完成")
        except Exception as exc:  # pragma: no cover - 不让 init 失败
            logger.warning("init storage close 失败: %s" % exc)
        # ② reset-all: y 校准 + x 撞墙 + 大臂 + 手爪 (并行段)
        try:
            reset_res = car.arm.reset_all(
                arm_angle=90,        # 复位位 +90°
                hand_angle=-90,      # UP
                x_direction="right", # 默认撞右墙
                reset_x_velocity=init_x_v,
                timeout=60.0,
                do_reset_y=True,     # y 校准在 ②, 后续 move_y 才有正确基准
            )
            logger.info("init reset_all (含 y 校准) 完成: %s", reset_res)
        except Exception as exc:
            self.last_error = "arm reset_all 失败: {}".format(exc)
            logger.warning("init 时 reset_all 失败: %s" % exc)
        # ③ 里程计清零
        if reset_position:
            # 机械臂归位 + 里程计清零 打包在 car.init_car_position() 里,
            # 一个调用同时完成两件事, 避免分开调时一个异常导致另一个遗漏。
            # reset_arm=True (默认): 包含 arm.reset_position(); 复用 car 路径
            # 仍会单独走 ensure_initialized 的 reset_arm 分支, 不重复归位 arm。
            try:
                init_res = car.init_car_position(reset_arm=False)
                logger.info(
                    "init_car_position 完成 (arm_reset=%s, odometry_reset=%s)",
                    init_res.get("arm_reset"), init_res.get("odometry_reset"),
                )
            except Exception as exc:
                self.last_error = "init_car_position 失败: {}".format(exc)
                logger.warning("init_car_position 失败: %s" % exc)
        self.car = car
        self.controller_generation = session.get("generation")
        self.last_init_at = time.time()
        self.last_error = None
        self.auto_heal_armed = True
        # 默认启动 5 路守护线程（lane/arm/task/ir/odom feed）
        self._ensure_feeds_running(car)
        return car

    def _ensure_infer_ready(self):
        return self.infer_service.ensure_ready()

    def ensure_initialized(self, reset_arm=False, force=False, reset_position=True):
        with self.init_lock:
            self.initializing = True
            self.last_error = None
            try:
                session = self._ensure_controller_ready()
                self._ensure_infer_ready()
                self.ensure_shared_cameras()
                with self._ref_lock:
                    if self.car is not None and force:
                        self._safe_close_locked()
                    if (
                        self.car is not None
                        and self.controller_generation is not None
                        and self.controller_generation != session.get("generation")
                    ):
                        self._safe_close_locked()
                    if self.car is None:
                        return self._create_car_locked(
                            reset_arm=reset_arm,
                            reset_position=reset_position,
                        )
                    self.car.STOP_PARAM = self.stop_after_action
                    if reset_arm:
                        # 2026-08-03：复用现有 car 的"完整复位"路径与 _create_car_locked 对齐
                        # "三步顺序：存储仓 (舵机 close) → reset_all (含 y 校准) → odom"。
                        # 复用路径仍然 reset_x=False 跳过撞墙,防 auto-init 自愈循环反复撞墙触发
                        # commit fb24b1a 描述的 PM2 死循环。
                        try:
                            self.car.set_storage_angle(98, speed=5)
                            logger.info("ensure_initialized storage close (98°) 完成")
                        except Exception as exc:  # pragma: no cover
                            logger.warning("ensure_initialized storage close 失败: %s" % exc)
                        try:
                            init_x_v = float(os.getenv("RAK_CAR_RESET_X_VELOCITY", "0.05"))
                            reset_res = self.car.arm.reset_all(
                                arm_angle=90,
                                hand_angle=-90,
                                x_direction="right",
                                reset_x_velocity=init_x_v,
                                timeout=60.0,
                                reset_x=False,    # 复用路径:跳过撞墙
                                do_reset_y=True,  # y 校准在 reset_all 内完成
                            )
                            logger.info("ensure_initialized reset_all (含 y) 完成: %s", reset_res)
                        except Exception as exc:
                            logger.warning("ensure_initialized reset_all 失败: %s" % exc)
                    if reset_position:
                        self.car.reset_position()
                    # 2026-07-27：复用现有 car 的 ensure_initialized 路径不再补 reset_x。
                    # 这样执行任务、健康检查、短暂重入时不会触发 x 自动撞墙归零。
                    # x 自动复位只保留在 _create_car_locked() 的真正 init 路径；
                    # 手动复位仍然显式调用 arm.reset_x / arm.reset_all。
                    self.controller_generation = session.get("generation")
                    # 复用现有 car 时也确保 5 路守护线程在跑（幂等）
                    self._ensure_feeds_running(self.car)
                    return self.car
            except Exception:
                self.last_error = traceback.format_exc()
                raise
            finally:
                self.initializing = False

    def start_auto_init(self):
        self.auto_init_requested = True
        self.auto_heal_armed = True

    def _auto_init_loop(self):
        while True:
            if self.current_job_id is not None or self.initializing:
                time.sleep(self.auto_init_retry_interval)
                continue
            if not self.auto_heal_armed:
                time.sleep(self.auto_init_retry_interval)
                continue
            snapshot = self._probe_controller()
            if self._is_car_ready(snapshot):
                time.sleep(self.auto_init_retry_interval)
                continue
            try:
                if snapshot.get("state") == "PROGRAM_READY":
                    self.ensure_initialized(**self.auto_init_kwargs)
            except Exception:
                pass
            time.sleep(self.auto_init_retry_interval)

    def _safe_close_locked(self):
        if self.car is None:
            return
        car = self.car
        # 2026-08-03: 先翻引用再关——realtime gate 立刻看到 car=None,
        # 不会把指令打到正在销毁的旧车上。
        self.car = None
        self.controller_generation = None

        def _close_worker():
            try:
                car.stop()
            except Exception:
                pass
            try:
                car.close()
            except Exception:
                pass

        # close 超时护栏: car.close 若 hang (串口/电机/zmq 卡死), 不能无限堵住
        # _ref_lock 与后续重建。15s 上限覆盖 5 个 feed join(timeout=5) 的正常慢路径;
        # 超时后旧实例线程在后台自然收尾, 资源依赖 MyCar.close 显式清理兜底。
        closer = threading.Thread(
            target=_close_worker, daemon=True, name="car_close_guard",
        )
        closer.start()
        closer.join(timeout=15.0)
        if closer.is_alive():
            logger.error(
                "car.close 超过 15s 未返回, 放弃等待; 旧实例线程/socket 可能泄漏"
            )

    def close(self, disable_auto_init=True):
        if disable_auto_init:
            self.auto_init_requested = False
            self.auto_heal_armed = False
        with self._ref_lock:
            self._safe_close_locked()
        with self.camera_lock:
            self._close_shared_cameras_locked()

    def shutdown(self):
        self.close()
        self.infer_service.stop()

    def _wait_serial_wrap_ready(self, timeout=5.0):
        """poll 等模块级 singleton serial_wrap 稳定到 connect_flag=True, 必要时主动 sync_with_probe。

        controller_session 的 probe 每次都 _close_locked 重置 connect_flag,
        singleton 自己又有 auto_connect_until_ready 异步重连;两条路径在 race。
        MyCar.__init__ 走到 ArmController 一步就直接读 serial, 必须先等稳定。
        poll 超时再没连上,就主动 sync_with_probe() 拉一次, 拿不到就让上层报错。
        """
        try:
            from smartcar.whalesbot.vehicle.base.serial_wrap import serial_wrap
        except Exception:
            return
        deadline = time.time() + float(timeout)
        while time.time() < deadline:
            if (
                getattr(serial_wrap, "connect_flag", False)
                and getattr(serial_wrap, "dev", None) is not None
                and getattr(serial_wrap, "is_open", False)
            ):
                return
            time.sleep(0.05)
        # 超时仍未稳定 — 主动拉一次 probe, 让 singleton 重新进入 program 模式
        try:
            logger.warning(
                "serial_wrap singleton 未在 %.1fs 内稳定, 主动 sync_with_probe()",
                timeout,
            )
            serial_wrap.sync_with_probe(probe_result=None)
        except Exception as exc:
            logger.warning("serial_wrap.sync_with_probe 失败: %s", exc)

    def _wait_until_ready(self, reset_position=False, timeout=None):
        timeout = self.action_ready_timeout if timeout is None else float(timeout)
        deadline = time.time() + timeout
        last_exc = None
        self.start_auto_init()
        #region debug-point runtime-init-queue-ready
        _debug_emit(
            "H1",
            "runtime_service._wait_until_ready",
            "进入 wait_until_ready",
            {
                "reset_position": reset_position,
                "timeout": timeout,
                "current_job_id": self.current_job_id,
                "initializing": self.initializing,
                "controller_state": self.controller_session.snapshot().get("state"),
            },
        )
        #endregion debug-point runtime-init-queue-ready
        while time.time() < deadline:
            try:
                snapshot = self._sync_controller_health_state()
                if snapshot.get("state") != "PROGRAM_READY":
                    snapshot = self.controller_session.ensure_ready(
                        timeout=min(0.8, max(0.1, deadline - time.time()))
                    )
                if snapshot.get("state") != "PROGRAM_READY":
                    raise RuntimeError(
                        "控制器尚未进入 program 模式: {}".format(
                            (snapshot.get("last_probe") or {}).get("detail")
                            or snapshot.get("detail")
                        )
                    )
                return self.ensure_initialized(reset_position=reset_position)
            except Exception as exc:
                last_exc = exc
                #region debug-point runtime-init-queue-ready
                _debug_emit(
                    "H1",
                    "runtime_service._wait_until_ready",
                    "ensure_initialized 失败，继续等待",
                    {
                        "exc_type": type(exc).__name__,
                        "exc_repr": repr(exc),
                        "initializing": self.initializing,
                        "current_job_id": self.current_job_id,
                    },
                )
                #endregion debug-point runtime-init-queue-ready
                if self._should_probe_controller(exc):
                    self.controller_session.note_io_failure(exc)
                    snapshot = self._sync_controller_health_state()
                    if snapshot.get("state") == "PROGRAM_READY":
                        snapshot = self.controller_session.snapshot()
                time.sleep(self.action_ready_poll_interval)
        detail = str(last_exc) if last_exc is not None else "未知错误"
        raise RuntimeError(f"等待小车就绪超时: {detail}")

    def _recover_controller_runtime(self, exc):
        detail = f"运行时控制器异常: {type(exc).__name__}: {exc}"
        self.controller_session.note_io_failure(detail)
        # _safe_close_locked 会改 self.car 引用，必须走 _ref_lock 串行保护。
        with self._ref_lock:
            self._safe_close_locked()
        self.start_auto_init()
        snapshot = self._sync_controller_health_state()
        self.last_error = "{}; state={}".format(
            detail,
            (snapshot.get("last_probe") or {}).get("detail")
            or snapshot.get("detail"),
        )
        return snapshot
