"""Jetson 单进程 runtime client。

接口尽量兼容 RuntimeApiClient，但动作直接进入 CarRuntimeService，完全跳过
HTTP、WebSocket、JSON 和 socket。"""
from __future__ import annotations

import time
from typing import Any, Optional

from .settings import load_settings


class LocalRuntimeClient:
    """CarRuntimeService 的进程内适配器。"""

    def __init__(self, service=None, settings=None):
        self.settings = settings or load_settings()
        if service is None:
            from runtime.services.local_runtime import start_local_runtime
            service = start_local_runtime()
        self.service = service
        self.api_base = "local://runtime"
        self.api_prefix = "/v1"

    def build_url(self, path):
        return path

    def get(self, path, timeout=None):
        """GET 直读 shim：把业务层常见的 realtime 只读端点映射到 service getter。

        生产由 FastAPI 路由返回 `{"ok": True, "<xxx>_state": ...}`；这里同形返回，
        任务代码（如 task1/task2 的 `client.get("/v1/realtime/vision/task")`）
        无需感知 transport 差异。
        """
        if path.endswith("/realtime/vision/task"):
            return {"ok": True, "task_state": self.service.get_task_state()}
        if path.endswith("/realtime/odom/state"):
            return {"ok": True, "odom_state": self.service.get_odom_state()}
        if path.endswith("/lane/state"):
            return {"ok": True, "lane_state": self.service.get_lane_state()}
        if path.endswith("/realtime/arm/state"):
            return {"ok": True, "arm_state": self.service.get_arm_state()}
        if path.endswith("/realtime/ir/state"):
            return {"ok": True, "ir_state": self.service.get_ir_state()}
        raise NotImplementedError("本地 transport 不支持 GET: {}".format(path))

    def post(self, path, payload=None, timeout=None):
        if path.endswith("/arm-velocity"):
            payload = payload or {}
            return self.realtime_arm_velocity(
                payload.get("x_vel"), payload.get("y_vel"),
                payload.get("arm_angle"), payload.get("hand_angle"),
            )
        if path.endswith("/realtime/chassis-velocity"):
            payload = payload or {}
            return self.realtime_chassis_velocity(
                payload.get("vx", 0.0), payload.get("vy", 0.0),
                payload.get("wz", 0.0),
            )
        if path.endswith("/realtime/chassis-align"):
            payload = payload or {}
            return self.service.chassis_align(**payload)
        raise NotImplementedError("本地 transport 不支持路径调用: {}".format(path))

    def wait_until_ready(self, timeout=None, poll_interval=None):
        timeout = self.settings.wait_timeout if timeout is None else float(timeout)
        deadline = time.monotonic() + timeout
        while True:
            state = self.service.get_state()
            if state.get("initialized"):
                return {"ok": True, "state": state}
            if time.monotonic() >= deadline:
                raise TimeoutError("等待本地 runtime 初始化超时")
            time.sleep(0.05 if poll_interval is None else float(poll_interval))

    def execute(self, target, name, args=None, kwargs=None, timeout=None, sync=False):
        if sync:
            return self.service.submit_job_and_wait(
                target, name, args=args or [], kwargs=kwargs or {}, timeout=timeout
            )
        return self.service.submit_job(target, name, args=args or [], kwargs=kwargs or {})

    def create_job(self, target, name, args=None, kwargs=None):
        return self.service.submit_job(target, name, args=args or [], kwargs=kwargs or {})

    def get_job(self, job_id):
        job = self.service.get_job(job_id)
        if job is None:
            raise KeyError("任务不存在: {}".format(job_id))
        return job

    def list_jobs(self):
        return {"jobs": self.service.list_jobs()}

    def wait_job(self, job_id, timeout=None, poll_interval=None):
        return self.service.wait_job(job_id, timeout=timeout, poll_interval=poll_interval)

    def cancel_job(self, job_id):
        return {"stopped": self.service.cancel_job(job_id), "job_id": job_id}

    def call(self, target, name, *args, timeout=None, **kwargs):
        return self.execute(target, name, args=list(args), kwargs=kwargs, timeout=timeout)

    def execute_task(self, name, *args, timeout=None, sync=False, **kwargs):
        return self.execute("task", name, args=list(args), kwargs=kwargs,
                            timeout=timeout, sync=sync)

    def execute_car_action(self, name, *args, timeout=None, sync=False, **kwargs):
        return self.execute("car", name, args=list(args), kwargs=kwargs,
                            timeout=timeout, sync=sync)

    def execute_arm_action(self, name, *args, timeout=None, sync=False, **kwargs):
        return self.execute("arm", name, args=list(args), kwargs=kwargs,
                            timeout=timeout, sync=sync)

    def run_task(self, name, *args, **kwargs):
        return self.create_job("task", name, args=list(args), kwargs=kwargs)

    def run_car_action(self, name, *args, **kwargs):
        return self.create_job("car", name, args=list(args), kwargs=kwargs)

    def run_arm_action(self, name, *args, **kwargs):
        return self.create_job("arm", name, args=list(args), kwargs=kwargs)

    def get_health(self, snapshot=False):
        return {"ok": True, "state": self.service.get_state()}

    def get_actions(self):
        return {"actions": self.service.list_actions()}

    def get_config(self):
        return {"transport": "local", "api_prefix": self.api_prefix}

    def get_runtime(self):
        return self.service.get_runtime_snapshot() or {}

    def init_runtime(self, force=False, reset_arm=False, reset_position=True):
        job = self.execute("system", "init", kwargs={
            "force": force, "reset_arm": reset_arm, "reset_position": reset_position,
        }, sync=True)
        return {"ok": True, "job": job}

    def set_stop_mode(self, enabled):
        return {"stop_after_action": self.service.set_stop_mode(enabled)}

    def reset_stop_flag(self):
        return {"cleared": self.service.reset_stop_flag()}

    def emergency_stop(self):
        return {"stopped": self.service.emergency_stop()}

    def close_runtime(self):
        if getattr(self.service, "is_fake", False):
            close = getattr(self.service, "close", None)
            if close is not None:
                close()
            return {"closed": True}
        from runtime.services.local_runtime import shutdown_local_runtime
        shutdown_local_runtime()
        return {"closed": True}

    # ---- realtime：不创建 job，不轮询 ----
    def realtime_wheel_speeds(self, speeds):
        return self.service.set_wheel_speeds(list(speeds))

    def realtime_wheel_encoders(self):
        return {"encoders": self.service.get_wheel_encoders()}

    def realtime_chassis_velocity(self, vx, vy, wz=0.0, duration=None):
        return self.service.set_chassis_velocity(vx, vy, wz, duration=duration)

    def realtime_arm_velocity(self, x_vel=None, y_vel=None, arm_angle=None, hand_angle=None):
        return self.service.set_arm_velocity(x_vel, y_vel, arm_angle, hand_angle)

    def realtime_motor_speed(self, port, speed, reverse=1):
        return self.service.set_single_motor(port, speed, reverse=reverse)

    def realtime_encoder(self, port, reverse=1):
        return {"encoder": self.service.get_encoder(port, reverse=reverse)}

    def realtime_stepper_rad(self, port, rad, time=0.5, reverse=1, perimeter=0.008):
        return self.service.set_stepper_rad(port, rad, time, reverse, perimeter)

    def realtime_bus_servo_angle(self, port, angle, speed=100):
        return self.service.set_bus_servo(port, angle, speed)

    def realtime_bus_servo_read(self, port):
        return {"angle": self.service.read_bus_servo(port)}

    def realtime_analog(self, port):
        return {"value": self.service.read_analog(port)}

    def realtime_analog2(self, port):
        return {"value": self.service.read_analog2(port)}

    def realtime_lane_state(self):
        return {"lane_state": self.service.get_lane_state()}

    def subscribe_lane(self, callback, hz=50.0):
        return self._subscribe_state(callback, self.realtime_lane_state, hz)

    def subscribe_task_detection(self, callback, hz=30.0):
        return self._subscribe_state(callback, self.get_task_state, hz)

    def _subscribe_state(self, callback, reader, hz):
        import threading
        stop_event = threading.Event()
        period = 1.0 / max(float(hz), 1.0)

        def loop():
            while not stop_event.is_set():
                try:
                    callback(reader())
                except Exception:
                    pass
                stop_event.wait(period)

        thread = threading.Thread(target=loop, name="local-state-subscription", daemon=True)
        thread.start()
        return stop_event.set

    def get_lane_state(self):
        return self.realtime_lane_state()

    def get_arm_state(self):
        return {"arm_state": self.service.get_arm_state()}

    def get_task_state(self):
        return {"task_state": self.service.get_task_state()}

    def get_ir_state(self):
        return {"ir_state": self.service.get_ir_state()}

    def get_odom_state(self):
        return {"odom_state": self.service.get_odom_state()}

    def get_vision_task_cache(self):
        return self.get_task_state()

    def request_vision_task(self, *, sort_pos=(0.0, 0.0), limit_x=1.0,
                            limit_y=1.0, timeout=20.0):
        result = self.execute_car_action(
            "get_detection_results", sort_pos=list(sort_pos),
            limit_x=limit_x, limit_y=limit_y, timeout=timeout, sync=True,
        )
        detections = result.get("result", []) if isinstance(result, dict) else []
        return {"ok": True, "count": len(detections), "detections": detections}

    def get_ir_distance_sync(self, side="left"):
        return self.service.get_ir_distance_sync(side=side)

    def get_all_ir_distance_sync(self):
        return self.service.get_all_ir_distance_sync()

    def get_odometry_sync(self, show_info=False):
        return self.service.get_odometry_sync(show_info=show_info)

    def get_distance_sync(self):
        return self.service.get_distance_sync()

    def _current_car(self):
        with self.service._ref_lock:
            car = self.service.car
        if car is None:
            raise RuntimeError("car 未初始化")
        return car

    def start_lane_feed(self, hz=50.0):
        return self._current_car().start_lane_feed(hz=hz)

    def stop_lane_feed(self, force=False):
        with self.service._ref_lock:
            car = self.service.car
        return {"stopped": False} if car is None else car.stop_lane_feed(force=force)

    def start_arm_feed(self, hz=20.0):
        return self._current_car().start_arm_feed(hz=hz)

    def stop_arm_feed(self, force=False):
        with self.service._ref_lock:
            car = self.service.car
        return {"stopped": False} if car is None else car.stop_arm_feed(force=force)

    def wait_wheels_stopped(self, settle_s=0.2, timeout_s=1.0):
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            e1 = self.realtime_wheel_encoders().get("encoders", [])
            time.sleep(float(settle_s))
            e2 = self.realtime_wheel_encoders().get("encoders", [])
            if len(e1) < 4 or len(e2) < 4:
                return True
            if sum(abs(float(e2[i]) - float(e1[i])) for i in range(4)) < 1.0:
                return True
        return False


def create_runtime_client(settings=None, transport=None):
    """按 RAK_CAR_TRANSPORT 选择 local/http；默认 local。"""
    import os
    mode = (transport or os.getenv("RAK_CAR_TRANSPORT", "local")).lower()
    if mode == "local":
        return LocalRuntimeClient(settings=settings)
    if mode == "fake":
        from runtime.services.fake_runtime import get_fake_runtime
        return LocalRuntimeClient(service=get_fake_runtime(), settings=settings)
    if mode == "http":
        from .api_client import RuntimeApiClient
        return RuntimeApiClient(settings=settings)
    raise ValueError("不支持的 RAK_CAR_TRANSPORT: {}".format(mode))
