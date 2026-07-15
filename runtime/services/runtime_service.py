#!/usr/bin/python3
# -*- coding: utf-8 -*-
import importlib
import json
import os
import queue
import threading
import time
import traceback
import uuid
import urllib.request
from pathlib import Path

import yaml

from runtime.core import settings
from runtime.core.actions import ARM_ACTIONS, CAR_ACTIONS, TASK_ACTION_NAMES
from runtime.hardware.controller_session import get_controller_session
from runtime.services.inference_service import InferBackendService

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


#region debug-point runtime-init-queue-session
def _debug_emit(hypothesis_id, location, msg, data=None):
    api_url = os.environ.get("DEBUG_SERVER_URL") or os.environ.get("TRAE_DEBUG_API_URL")
    if not api_url:
        return
    payload = {
        "sessionId": "runtime-init-queue",
        "hypothesisId": hypothesis_id,
        "location": location,
        "msg": msg,
        "data": data or {},
    }
    try:
        req = urllib.request.Request(
            api_url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        urllib.request.urlopen(req, timeout=0.2).read()
    except Exception:
        pass
#endregion debug-point runtime-init-queue-session


def normalize_value(value):
    if np is not None:
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, np.ndarray):
            return value.tolist()
    if isinstance(value, (list, tuple)):
        return [normalize_value(item) for item in value]
    if isinstance(value, dict):
        return {str(key): normalize_value(val) for key, val in value.items()}
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


class CarRuntimeService:
    def __init__(self):
        self.car = None
        self._car_class = None
        self._task_module = None
        self._camera_cfg = None
        self.shared_front_camera = None
        self.shared_side_camera = None
        self.infer_service = InferBackendService()
        self.controller_session = get_controller_session()
        self.controller_generation = None
        self.car_lock = threading.RLock()
        self.camera_lock = threading.Lock()
        self.init_lock = threading.Lock()
        self.job_lock = threading.Lock()
        self.jobs = {}
        self.job_queue = queue.Queue()
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()
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

    def set_stream_service(self, stream_service):
        self.stream_service = stream_service

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

    def _is_controller_related_error(self, detail):
        if not detail:
            return False
        text = str(detail).lower()
        keywords = (
            "控制器",
            "下位机",
            "ttyusb",
            "serial",
            "controller",
            "program 模式",
            "program",
            "bootloader",
            "未找到控制器串口",
            "控制器恢复超时",
            "控制器探测失败",
        )
        return any(keyword in text for keyword in keywords)

    def _sync_controller_health_state(self, snapshot=None):
        snapshot = snapshot or self.controller_session.snapshot()
        self.last_controller_probe = snapshot.get("last_probe")
        if (
            snapshot.get("state") == "PROGRAM_READY"
            and self._is_controller_related_error(self.last_error)
        ):
            self.last_error = None
        return snapshot

    def _trim_jobs(self, keep=None):
        if keep is None:
            keep = settings.JOB_HISTORY_LIMIT
        if len(self.jobs) <= keep:
            return
        removable_ids = [
            job_id
            for job_id, job in self.jobs.items()
            if job["status"] in {"succeeded", "failed"}
        ]
        while len(self.jobs) > keep and removable_ids:
            self.jobs.pop(removable_ids.pop(0), None)

    def _set_job(self, job_id, **updates):
        with self.job_lock:
            self.jobs[job_id].update(updates)

    def _get_car_class(self):
        if self._car_class is None:
            self._car_class = importlib.import_module("car_wrap_2026").MyCar
        return self._car_class

    def _get_task_module(self):
        if self._task_module is None:
            self._task_module = importlib.import_module("car_task_function")
        return self._task_module

    def _bind_task_car(self, car):
        self._get_task_module().bind_car(car)

    def _probe_controller(self):
        return self._sync_controller_health_state(self.controller_session.snapshot())

    def _is_generation_stale(self, snapshot=None):
        snapshot = snapshot or self.controller_session.snapshot()
        if self.car is None or self.controller_generation is None:
            return False
        return self.controller_generation != snapshot.get("generation")

    def _is_car_ready(self, snapshot=None):
        snapshot = snapshot or self.controller_session.snapshot()
        return (
            self.car is not None
            and snapshot.get("state") == "PROGRAM_READY"
            and not self._is_generation_stale(snapshot)
        )

    def _ensure_controller_ready(self):
        snapshot = self.controller_session.ensure_ready(
            timeout=self.action_ready_timeout
        )
        return self._sync_controller_health_state(snapshot)

    def _mark_controller_offline(self, detail=None):
        self.controller_session.mark_offline(detail, mode="unknown")
        with self.car_lock:
            self._safe_close_locked()
        if detail:
            self.last_error = detail
        self._sync_controller_health_state()

    def _create_car_locked(self, reset_arm=False, reset_position=True):
        session = self._ensure_controller_ready()
        car = self._get_car_class()(
            cap_front=self.shared_front_camera,
            cap_side=self.shared_side_camera,
            streamer=self.stream_service,
        )
        self._remember_shared_cameras(car)
        car.STOP_PARAM = self.stop_after_action
        self._bind_task_car(car)
        car.beep()
        time.sleep(1)
        if reset_arm:
            car.arm.reset_position()
        else:
            # 即使外部没要求 reset_arm,每次 init 仍要 y 轴归零:
            # 控制器重启/USB 重连后 y 编码器基准点丢失,不清零 move_y 会在错误基准上跑。
            # 失败不抛(可能机械臂在不允许动的状态),仅记 last_error。
            try:
                car.arm.reset_y()
            except Exception as exc:
                self.last_error = "arm reset_y 失败: {}".format(exc)
                logger.warning("init 时 reset_y 失败: %s" % exc)
        if reset_position:
            car.reset_position()
        self.car = car
        self.controller_generation = session.get("generation")
        self.last_init_at = time.time()
        self.last_error = None
        self.auto_heal_armed = True
        # 默认启 lane_feed 守护线程：比赛阶段 lane_state 必须持续更新
        # 供外环消费。start_lane_feed 幂等，重复调用立即返回。
        try:
            car.start_lane_feed(hz=20.0)
        except Exception as exc:  # pragma: no cover - 不让 init 失败
            logger.warning("lane_feed auto-start failed: {}".format(exc))
        # 默认启 arm_feed 守护线程:持续刷新 streamer.arm_state(y/x 位置),
        # 供 WS subscribe_arm_state 实时推送,调试机械臂必备
        try:
            car.start_arm_feed(hz=20.0)
        except Exception as exc:
            logger.warning("arm_feed auto-start failed: {}".format(exc))
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
                with self.car_lock:
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
                    self._bind_task_car(self.car)
                    if reset_arm:
                        self.car.arm.reset_position()
                    if reset_position:
                        self.car.reset_position()
                    self.controller_generation = session.get("generation")
                    # 复用现有 car 时也确保 lane_feed 跑着（幂等）
                    try:
                        self.car.start_lane_feed(hz=20.0)
                    except Exception as exc:  # pragma: no cover
                        logger.warning("lane_feed auto-start (reused) failed: {}".format(exc))
                    # arm_feed 同理
                    try:
                        self.car.start_arm_feed(hz=20.0)
                    except Exception as exc:
                        logger.warning("arm_feed auto-start (reused) failed: {}".format(exc))
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
        with self.car_lock:
            self._safe_close_locked()
        with self.camera_lock:
            self._close_shared_cameras_locked()

    def shutdown(self):
        self.close()
        self.infer_service.stop()

    def emergency_stop(self):
        # 关键：不持 car_lock。worker 跑长动作（reset_position / move_* / 巡线）时
        # car_lock 被占，若这里也抢 car_lock 会排队等长动作结束 → 急停失效。
        # 停车指令走串口层自带锁，与 worker 并发安全；car.emergency_stop 内部
        # 置 _stop_flag/_estop 让正在跑的循环协作退出，并直接停三轴。
        car = self.car
        if car is None:
            return False
        try:
            return bool(car.emergency_stop())
        except AttributeError:
            # 兼容旧 car（无 emergency_stop）：至少置标志 + 停底盘
            car._stop_flag = True
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

    # === 实时硬件控制（car_lock 同步路径，绕过 job_queue，50Hz 友好） ===

    def _realtime_check_locked(self):
        if self.car is None:
            raise RuntimeError("car 未初始化")

    def set_wheel_speeds(self, speeds):
        with self.car_lock:
            self._realtime_check_locked()
            return self.car.set_wheel_speeds(speeds)

    def set_chassis_velocity(self, vx, vy, wz, duration=None):
        """
        上层外环专用：(vx, vy, wz) → 4 轮速直发，绕开 set_velocity 的里程计耦合。

        走 car_lock 同步路径，50Hz 友好。里程计照常由 odometer_thread 自动更新
        （它读 wheels_chassis.get_linear()）。
        """
        with self.car_lock:
            self._realtime_check_locked()
            vx = float(vx)
            vy = float(vy)
            wz = float(wz)
            # 直接复用 chassis 的 IK（避免重复实现 mecanum 公式）
            wheel_speeds = list(
                self.car.chassis.calculate_wheel_velocities(vx, vy, wz)
            )
            # 直发轮速，绕开 set_velocity（set_velocity 会反复 lock + set）
            self.car.wheels_chassis.set_linear([float(s) for s in wheel_speeds])
            return {
                "vx": vx,
                "vy": vy,
                "wz": wz,
                "duration": duration,
                "wheel_speeds": wheel_speeds,
            }

    def get_wheel_encoders(self):
        with self.car_lock:
            self._realtime_check_locked()
            return self.car.get_wheel_encoders()

    def get_lane_state(self):
        """外环专用：读 streamer 缓存的 lane_state。

        数据来源：`lane_feed` 守护线程（runtime 启动后默认 20Hz）通过
        `car.streamer.set_lane_state(...)` 持续刷新的内存缓存。

        不进 job_queue、不打 ZMQ、不抢 car_lock——只取 `meta_lock`（极快）。
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

        不进 job_queue、不打 ZMQ、不抢 car_lock——只取 `meta_lock`（极快）。
        20Hz+ 轮询安全。
        """
        if self.stream_service is None:
            raise RuntimeError("stream_service 未注入（runtime 启动异常）")
        return self.stream_service.get_arm_state()

    def set_single_motor(self, port, speed, reverse=1):
        with self.car_lock:
            self._realtime_check_locked()
            return self.car.set_single_motor(port, speed, reverse=reverse)

    def get_encoder(self, port, reverse=1):
        with self.car_lock:
            self._realtime_check_locked()
            return self.car.get_encoder(port, reverse=reverse)

    def set_stepper_rad(self, port, rad, time=0.5, reverse=1, perimeter=0.008):
        with self.car_lock:
            self._realtime_check_locked()
            return self.car.set_stepper_rad(port, rad, time, reverse, perimeter)

    def set_bus_servo(self, port, angle, speed=100):
        with self.car_lock:
            self._realtime_check_locked()
            return self.car.set_bus_servo(port, angle, speed)

    def read_bus_servo(self, port):
        with self.car_lock:
            self._realtime_check_locked()
            return self.car.read_bus_servo(port)

    def read_analog(self, port):
        with self.car_lock:
            self._realtime_check_locked()
            return self.car.read_analog(port)

    def read_analog2(self, port):
        with self.car_lock:
            self._realtime_check_locked()
            return self.car.read_analog2(port)

    def set_stop_mode(self, enabled):
        with self.car_lock:
            self.stop_after_action = bool(enabled)
            if self.car is not None:
                self.car.STOP_PARAM = self.stop_after_action
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
        with self.car_lock:
            if self.car is None:
                return None
            self._bind_task_car(self.car)
            return {
                "odometry": normalize_value(self.car.get_odometry()),
                "distance": normalize_value(self.car.get_distance()),
                "stop_after_action": self.car.STOP_PARAM,
                "stop_flag": self.car._stop_flag,
            }

    def get_car_for_stream(self):
        # 不持 car_lock：capture_loop 在机械臂长动作期间会被卡住 5+s，
        # 导致 stream 缓存不更新,前端 MJPEG/frame 一直吐最后那帧(画面卡死)。
        # self.car 是单实例属性，读取是 GIL 原子操作,无撕裂风险。
        # 切换 self.car 仅发生在 init/recover 流程,且会同步停止 capture_loop 一拍。
        return self.car

    def list_actions(self):
        return {
            "task": sorted(TASK_ACTION_NAMES),
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

    def submit_job(self, target, name, args=None, kwargs=None):
        args = args or []
        kwargs = kwargs or {}
        target = str(target)
        name = str(name)
        valid_actions = self.list_actions()
        if target not in valid_actions:
            raise KeyError(f"不支持的 target: {target}")
        if name not in valid_actions[target]:
            raise KeyError(f"不支持的动作: {target}.{name}")
        job_id = uuid.uuid4().hex[:12]
        job = {
            "id": job_id,
            "target": target,
            "name": name,
            "args": normalize_value(args),
            "kwargs": normalize_value(kwargs),
            "status": "queued",
            "submitted_at": time.time(),
            "started_at": None,
            "finished_at": None,
            "result": None,
            "error": None,
        }
        with self.job_lock:
            self.jobs[job_id] = job
            self._trim_jobs()
        self.job_queue.put(job_id)
        return job

    def wait_job(self, job_id, timeout=None, poll_interval=None):
        timeout = (
            settings.DEFAULT_JOB_WAIT_TIMEOUT if timeout is None else float(timeout)
        )
        poll_interval = (
            settings.DEFAULT_POLL_INTERVAL
            if poll_interval is None
            else float(poll_interval)
        )
        deadline = time.time() + timeout
        while time.time() < deadline:
            job = self.get_job(job_id)
            if job is None:
                raise KeyError(f"任务不存在: {job_id}")
            if job["status"] in {"succeeded", "failed"}:
                return job
            time.sleep(poll_interval)
        raise TimeoutError(f"等待任务超时: {job_id}")

    def submit_job_and_wait(self, target, name, args=None, kwargs=None, timeout=None):
        job = self.submit_job(target=target, name=name, args=args, kwargs=kwargs)
        return self.wait_job(job["id"], timeout=timeout)

    def list_jobs(self):
        with self.job_lock:
            return list(self.jobs.values())

    def get_job(self, job_id):
        with self.job_lock:
            return self.jobs.get(job_id)

    def _dispatch_task(self, name, args, kwargs):
        task_module = self._get_task_module()
        return getattr(task_module, name)(*args, **kwargs)

    def _dispatch_car(self, name, args, kwargs):
        self._bind_task_car(self.car)
        return CAR_ACTIONS[name](self.car, *args, **kwargs)

    def _dispatch_arm(self, name, args, kwargs):
        self._bind_task_car(self.car)
        return ARM_ACTIONS[name](self.car.arm, *args, **kwargs)

    def _dispatch_system(self, name, _args, kwargs):
        if name == "init":
            self.ensure_initialized(
                reset_arm=kwargs.get("reset_arm", False),
                force=kwargs.get("force", False),
                reset_position=kwargs.get(
                    "reset_position",
                    settings.get_reset_position_on_init(),
                ),
            )
            return self.get_state()
        if name == "close":
            self.close()
            return {"closed": True}
        if name == "set_stop_mode":
            return {
                "stop_after_action": self.set_stop_mode(
                    kwargs.get("enabled", False)
                )
            }
        if name == "reset_stop_flag":
            return {"stop_flag": self.reset_stop_flag()}
        if name == "emergency_stop":
            return {"stopped": self.emergency_stop()}
        raise KeyError(f"不支持的系统动作: {name}")

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
        with self.car_lock:
            self._safe_close_locked()
        self.start_auto_init()
        snapshot = self._sync_controller_health_state()
        self.last_error = "{}; state={}".format(
            detail,
            (snapshot.get("last_probe") or {}).get("detail")
            or snapshot.get("detail"),
        )
        return snapshot

    def _dispatch_target_locked(self, target, name, args, kwargs):
        self._bind_task_car(self.car)
        if self.car is not None:
            self.car.STOP_PARAM = self.stop_after_action
        if target == "task":
            return self._dispatch_task(name, args, kwargs)
        if target == "car":
            return self._dispatch_car(name, args, kwargs)
        if target == "arm":
            return self._dispatch_arm(name, args, kwargs)
        raise KeyError(f"不支持的 target: {target}")

    def _dispatch(self, target, name, args, kwargs):
        if target == "system":
            return self._dispatch_system(name, args, kwargs)
        self._wait_until_ready(reset_position=False)
        try:
            with self.car_lock:
                return self._dispatch_target_locked(target, name, args, kwargs)
        except Exception as exc:
            if not self._should_probe_controller(exc):
                raise
            snapshot = self._recover_controller_runtime(exc)
            raise RuntimeError(
                "下位机掉线，已转入后台自愈: {}".format(
                    (snapshot.get("last_probe") or {}).get("detail")
                    or snapshot.get("detail")
                )
            ) from exc

    def _should_probe_controller(self, exc):
        text = traceback.format_exception_only(type(exc), exc)
        message = "".join(text).lower()
        keywords = (
            "input/output error",
            "device disconnected",
            "device reports readiness",
            "serial",
            "ttyusb",
            "controller",
            "控制器",
            "下位机",
            "未就绪",
            "broken pipe",
            "timed out",
            "timeout",
            "no such file",
            "resource temporarily unavailable",
            "controllernotreadyerror",
            "controllernoresponseerror",
            "controllertransporterror",
        )
        return any(keyword in message for keyword in keywords)

    def _handle_dispatch_failure(self, target, exc):
        if target == "system":
            return
        if not self._should_probe_controller(exc):
            return
        self._sync_controller_health_state()

    def _worker_loop(self):
        while True:
            job_id = self.job_queue.get()
            job = self.get_job(job_id)
            if job is None:
                self.job_queue.task_done()
                continue
            self.current_job_id = job_id
            self._set_job(
                job_id,
                status="running",
                started_at=time.time(),
                error=None,
            )
            #region debug-point runtime-init-queue-worker
            _debug_emit(
                "H2",
                "runtime_service._worker_loop",
                "worker 开始执行任务",
                {
                    "job_id": job_id,
                    "target": job["target"],
                    "name": job["name"],
                    "queued_size": self.job_queue.qsize(),
                },
            )
            #endregion debug-point runtime-init-queue-worker
            try:
                result = self._dispatch(
                    job["target"],
                    job["name"],
                    job["args"],
                    job["kwargs"],
                )
                self._set_job(
                    job_id,
                    status="succeeded",
                    result=normalize_value(result),
                    finished_at=time.time(),
                )
                #region debug-point runtime-init-queue-worker
                _debug_emit(
                    "H2",
                    "runtime_service._worker_loop",
                    "worker 任务成功",
                    {
                        "job_id": job_id,
                        "target": job["target"],
                        "name": job["name"],
                    },
                )
                #endregion debug-point runtime-init-queue-worker
            except Exception as exc:
                self._handle_dispatch_failure(job["target"], exc)
                self._set_job(
                    job_id,
                    status="failed",
                    error=traceback.format_exc(),
                    finished_at=time.time(),
                )
                #region debug-point runtime-init-queue-worker
                _debug_emit(
                    "H2",
                    "runtime_service._worker_loop",
                    "worker 任务失败",
                    {
                        "job_id": job_id,
                        "target": job["target"],
                        "name": job["name"],
                        "exc_type": type(exc).__name__,
                        "exc_repr": repr(exc),
                    },
                )
                #endregion debug-point runtime-init-queue-worker
            finally:
                self.current_job_id = None
                self.job_queue.task_done()
