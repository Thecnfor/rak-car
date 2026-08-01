#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""生命周期 Mixin（从 runtime_service.py 拆出）。

负责 MyCar 的创建 / 复用 / 关闭、共享摄像头、auto-init 自愈循环、
`_wait_until_ready` / `_recover_controller_runtime` 编排。依赖聚合类提供
`self.controller_session`、`self._ref_lock`、`self.init_lock` 等属性，以及
`ControllerWatcherMixin` 的 `_ensure_controller_ready` / `_is_car_ready` /
`_probe_controller` / `_sync_controller_health_state` 等。
"""
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

    def _create_car_locked(self, reset_arm=False, reset_position=True):
        session = self._ensure_controller_ready()
        car = self._get_car_class()(
            cap_front=self.shared_front_camera,
            cap_side=self.shared_side_camera,
            streamer=self.stream_service,
        )
        self._remember_shared_cameras(car)
        car.STOP_PARAM = self.stop_after_action
        car.beep()
        time.sleep(1)
        # init 阶段统一做一次 arm.reset_all():
        #   x 撞墙 + 大臂 +90° + 手爪 UP —— 三路并行（物理独立，串口字节 FIFO 串行化）
        #   reset_y 触底串行收尾在最后（触底磁感是绝对零点，不进并行池）
        # 2026-07-31 调整：复用 ensure_initialized 的复用路径在 reset_arm=True 时也走 reset_all,
        # 但**不**单独补 reset_x —— 复用路径下 x 由视觉闭环控制，避免 auto-init 反复撞墙
        # (commit fb24b1a 描述的 PM2 死循环)。
        try:
            init_x_v = float(os.getenv("RAK_CAR_RESET_X_VELOCITY", "0.05"))
            reset_res = car.arm.reset_all(
                arm_angle=90,        # 复位位 +90°
                hand_angle=-90,      # UP
                x_direction="right", # 默认撞右墙
                reset_x_velocity=init_x_v,
                timeout=60.0,
            )
            logger.info("init reset_all 完成: %s", reset_res)
        except Exception as exc:
            self.last_error = "arm reset_all 失败: {}".format(exc)
            logger.warning("init 时 reset_all 失败: %s" % exc)
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
        # 2026-07-30 init 时把存储仓舵机转到 close 物理位（98°），与 reset 同步。
        # 参照 test/test_storage_close.py：先抬 y 到 -150mm 离开保护区，再发舵机。
        # 走下层同步方法（car.arm.move_y_position / car.set_storage_angle），
        # 不绕 HTTP / ArmClient 业务 wrapper，失败仅 log warn，不阻断 init。
        try:
            car.arm.move_y_position(-0.150)
            car.set_storage_angle(98, speed=5)
            logger.info("init storage close (98°) 完成")
        except Exception as exc:  # pragma: no cover - 不让 init 失败
            logger.warning("init storage close 失败: %s" % exc)
        self.car = car
        self.controller_generation = session.get("generation")
        self.last_init_at = time.time()
        self.last_error = None
        self.auto_heal_armed = True
        # 默认启 lane_feed 守护线程：比赛阶段 lane_state 必须持续更新
        # 供外环消费。start_lane_feed 幂等，重复调用立即返回。
        try:
            car.start_lane_feed(hz=50.0)
        except Exception as exc:  # pragma: no cover - 不让 init 失败
            logger.warning("lane_feed auto-start failed: {}".format(exc))
        # 默认启 arm_feed 守护线程:持续刷新 streamer.arm_state(y/x 位置),
        # 供 WS subscribe_arm_state 实时推送,调试机械臂必备
        try:
            car.start_arm_feed(hz=20.0)
        except Exception as exc:
            logger.warning("arm_feed auto-start failed: {}".format(exc))
        # 默认启 task_feed 守护线程:持续刷新 streamer.task_state(侧摄目标检测),
        # 供 WS subscribe_task_detection 实时推送,"边走边看"侧摄目标的必需组件
        try:
            car.start_task_feed(hz=30.0)
        except Exception as exc:
            logger.warning("task_feed auto-start failed: {}".format(exc))
        # 2026-07-31 默认启 ir_feed 守护线程(50Hz,与 lane_feed 同档):
        # 供 main/chassis/tasks/read_ir.py 与 orchestrator._wait_until_triggered
        # 走缓存读,避免每次进 job_queue。hz 由 env RAK_CAR_IR_FEED_HZ 覆盖。
        try:
            car.start_ir_feed(
                hz=float(os.environ.get("RAK_CAR_IR_FEED_HZ", str(50.0)))
            )
        except Exception as exc:
            logger.warning("ir_feed auto-start failed: {}".format(exc))
        # 2026-07-31 默认启 odom_feed 守护线程(50Hz):同上模式,
        # 喂底盘里程计缓存,给 main/chassis/api.py.get_odometry 走 fast-path。hz 由 env 覆盖。
        try:
            car.start_odom_feed(
                hz=float(os.environ.get("RAK_CAR_ODOM_FEED_HZ", str(50.0)))
            )
        except Exception as exc:
            logger.warning("odom_feed auto-start failed: {}".format(exc))
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
                        # 复用现有 car 的"完整复位"路径：arm + hand 并行 → y 串行收尾。
                        # 与 _create_car_locked 走同一入口 (reset_all),保证两条 init 路径语义一致。
                        # 注：复用路径显式传 reset_x=False 跳过撞墙,防止 auto_init 自愈循环
                        # 反复撞墙触发 commit fb24b1a 描述的 PM2 死循环;只有真正创建新 car 的
                        # _create_car_locked 才默认 reset_x=True 撞墙定原点。
                        try:
                            init_x_v = float(os.getenv("RAK_CAR_RESET_X_VELOCITY", "0.05"))
                            reset_res = self.car.arm.reset_all(
                                arm_angle=90,
                                hand_angle=-90,
                                x_direction="right",
                                reset_x_velocity=init_x_v,
                                timeout=60.0,
                                reset_x=False,  # 复用路径:跳过撞墙
                            )
                            logger.info("ensure_initialized reset_all 完成: %s", reset_res)
                        except Exception as exc:
                            logger.warning("ensure_initialized reset_all 失败: %s" % exc)
                    if reset_position:
                        self.car.reset_position()
                    # 2026-07-27：复用现有 car 的 ensure_initialized 路径不再补 reset_x。
                    # 这样执行任务、健康检查、短暂重入时不会触发 x 自动撞墙归零。
                    # x 自动复位只保留在 _create_car_locked() 的真正 init 路径；
                    # 手动复位仍然显式调用 arm.reset_x / arm.reset_all。
                    self.controller_generation = session.get("generation")
                    # 复用现有 car 时也确保 lane_feed 跑着（幂等）
                    try:
                        self.car.start_lane_feed(hz=50.0)
                    except Exception as exc:  # pragma: no cover
                        logger.warning("lane_feed auto-start (reused) failed: {}".format(exc))
                    # arm_feed 同理
                    try:
                        self.car.start_arm_feed(hz=20.0)
                    except Exception as exc:
                        logger.warning("arm_feed auto-start (reused) failed: {}".format(exc))
                    # task_feed 同理
                    try:
                        self.car.start_task_feed(hz=30.0)
                    except Exception as exc:
                        logger.warning("task_feed auto-start (reused) failed: {}".format(exc))
                    # 2026-07-31：ir_feed / odom_feed 同理（复用现有 car 时也确保在跑）
                    try:
                        self.car.start_ir_feed(
                            hz=float(os.environ.get("RAK_CAR_IR_FEED_HZ", str(50.0)))
                        )
                    except Exception as exc:
                        logger.warning("ir_feed auto-start (reused) failed: {}".format(exc))
                    try:
                        self.car.start_odom_feed(
                            hz=float(os.environ.get("RAK_CAR_ODOM_FEED_HZ", str(50.0)))
                        )
                    except Exception as exc:
                        logger.warning("odom_feed auto-start (reused) failed: {}".format(exc))
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
        try:
            self.car.stop()
        except Exception:
            pass
        try:
            self.car.close()
        except Exception:
            pass
        self.car = None
        self.controller_generation = None

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
