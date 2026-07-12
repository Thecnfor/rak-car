#!/usr/bin/python3
# -*- coding: utf-8 -*-
import importlib
import queue
import threading
import time
import traceback
import uuid
from pathlib import Path

import yaml

from runtime.core import settings
from runtime.core.actions import ARM_ACTIONS, CAR_ACTIONS, TASK_ACTION_NAMES
from runtime.hardware.controller_session import get_controller_session

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


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
        self.controller_session.mark_offline(detail)
        with self.car_lock:
            self._safe_close_locked()
        if detail:
            self.last_error = detail
        self._sync_controller_health_state()

    def _create_car_locked(self, reset_arm=False, reset_position=True):
        session = self._ensure_controller_ready()
        self.ensure_shared_cameras()
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
        if reset_position:
            car.reset_position()
        self.car = car
        self.controller_generation = session.get("generation")
        self.last_init_at = time.time()
        self.last_error = None
        self.auto_heal_armed = True
        return car

    def ensure_initialized(self, reset_arm=False, force=False, reset_position=True):
        with self.init_lock:
            self.initializing = True
            self.last_error = None
            try:
                session = self._ensure_controller_ready()
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

    def emergency_stop(self):
        with self.car_lock:
            if self.car is None:
                return False
            self.car._stop_flag = True
            self.car.stop()
            return True

    def reset_stop_flag(self):
        with self.car_lock:
            if self.car is None:
                return False
            self.car._stop_flag = False
            return True

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
        with self.car_lock:
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
        while time.time() < deadline:
            try:
                return self.ensure_initialized(reset_position=reset_position)
            except Exception as exc:
                last_exc = exc
                if self._should_probe_controller(exc):
                    snapshot = self._probe_controller()
                    last_probe = snapshot.get("last_probe") or {}
                    if snapshot.get("state") != "PROGRAM_READY":
                        self._mark_controller_offline(
                            "等待动作执行时检测到下位机离线: {}".format(
                                last_probe.get("detail") or snapshot.get("detail")
                            )
                        )
                time.sleep(self.action_ready_poll_interval)
        detail = str(last_exc) if last_exc is not None else "未知错误"
        raise RuntimeError(f"等待小车就绪超时: {detail}")

    def _recover_controller_runtime(self, exc):
        detail = f"运行时控制器异常: {type(exc).__name__}: {exc}"
        self.controller_session.note_io_failure(detail)
        with self.car_lock:
            self._safe_close_locked()
        snapshot = self.controller_session.ensure_ready(
            timeout=self.action_ready_timeout
        )
        if snapshot.get("state") == "PROGRAM_READY":
            self.last_error = None
        else:
            self.last_error = "{}; probe={}".format(
                detail,
                (snapshot.get("last_probe") or {}).get("detail")
                or snapshot.get("detail"),
            )
        return self._sync_controller_health_state(snapshot)

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
            if snapshot.get("state") != "PROGRAM_READY":
                raise RuntimeError(
                    "下位机掉线，等待恢复: {}".format(
                        (snapshot.get("last_probe") or {}).get("detail")
                        or snapshot.get("detail")
                    )
                ) from exc
            self._wait_until_ready(reset_position=False)
            with self.car_lock:
                return self._dispatch_target_locked(target, name, args, kwargs)

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
        snapshot = self._probe_controller()
        if snapshot.get("state") != "PROGRAM_READY":
            self._mark_controller_offline(
                "运行时检测到下位机离线: {}".format(
                    (snapshot.get("last_probe") or {}).get("detail")
                    or snapshot.get("detail")
                )
            )

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
            except Exception as exc:
                self._handle_dispatch_failure(job["target"], exc)
                self._set_job(
                    job_id,
                    status="failed",
                    error=traceback.format_exc(),
                    finished_at=time.time(),
                )
            finally:
                self.current_job_id = None
                self.job_queue.task_done()
