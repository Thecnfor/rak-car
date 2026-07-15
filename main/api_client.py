#!/usr/bin/python3
# -*- coding: utf-8 -*-
import time

try:
    import requests
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 requests 依赖，请先执行: python3 -m pip install requests"
    ) from exc

try:
    from .settings import load_settings
except ImportError:  # pragma: no cover
    from settings import load_settings


class RuntimeApiClient:
    def __init__(self, settings=None):
        self.settings = settings or load_settings()

    @property
    def api_base(self):
        return self.settings.api_base

    @property
    def api_prefix(self):
        return self.settings.api_prefix

    def build_url(self, path):
        return f"{self.api_base}{path}"

    def _request(self, method, path, payload=None, timeout=None):
        timeout = timeout or self.settings.request_timeout
        response = requests.request(
            method=method,
            url=self.build_url(path),
            json=payload,
            timeout=timeout,
        )
        response.raise_for_status()
        return response.json() if response.content else {}

    def get(self, path, timeout=None):
        return self._request("GET", path, timeout=timeout)

    def post(self, path, payload=None, timeout=None):
        return self._request("POST", path, payload=payload, timeout=timeout)

    def _deadline(self, timeout=None):
        timeout = self.settings.wait_timeout if timeout is None else float(timeout)
        return time.time() + timeout

    def _is_retryable_request_error(self, exc):
        if isinstance(exc, requests.ConnectionError):
            return True
        if isinstance(exc, requests.Timeout):
            return True
        return False

    def get_health(self, snapshot=False):
        suffix = "?snapshot=1" if snapshot else ""
        return self.get(f"{self.api_prefix}/health{suffix}")

    def get_actions(self):
        return self.get(f"{self.api_prefix}/actions")

    def get_config(self):
        return self.get(f"{self.api_prefix}/config")

    def get_runtime(self):
        return self.get(f"{self.api_prefix}/runtime")

    def list_jobs(self):
        return self.get(f"{self.api_prefix}/jobs")

    def create_job(self, target, name, args=None, kwargs=None):
        payload = {
            "target": target,
            "name": name,
            "args": args or [],
            "kwargs": kwargs or {},
        }
        response = self.post(f"{self.api_prefix}/jobs", payload=payload)
        return response["job"]

    def execute(self, target, name, args=None, kwargs=None, timeout=None, sync=False):
        """通用执行：D 改造后默认异步（立即返回 job dict，status=queued）。

        异步模式（默认）：
          - 不轮询，调用方自行 `get_job(job_id)` 查状态或 `cancel_job(job_id)` 取消。
          - 适合：上层编排一边跑一边发 wheel_speeds / 想拿 job_id 做并行编排。
        同步模式（sync=True）：
          - 内部 polling 到 succeeded/failed，行为与改造前完全一致。
          - 适合：链式编排（`move_xy → grasp → release`），业务层要等结果才能下一步。

        用 `execute_arm_action` / `execute_car_action` / `call` 的旧调用方，arm 长动作
        （reset_y / reset_x / reset_origin / move_xy / move_x / move_y / set_side /
        set_hand / set_storage / grasp）已显式加 sync=True（见 main/arm/api.py），行为
        不变。
        """
        payload = {
            "target": target,
            "name": name,
            "args": args or [],
            "kwargs": kwargs or {},
            "sync": sync,
        }
        if timeout is not None:
            payload["timeout"] = timeout

        if sync:
            # 同步：阻塞轮询到 succeeded/failed（保留改造前行为）
            deadline = self._deadline(timeout)
            last_exc = None
            while time.time() < deadline:
                try:
                    response = self.post(
                        f"{self.api_prefix}/execute",
                        payload=payload,
                        timeout=min((deadline - time.time()) + 5.0, self.settings.wait_timeout + 5.0),
                    )
                    return response["job"]
                except requests.RequestException as exc:
                    last_exc = exc
                    if not self._is_retryable_request_error(exc):
                        raise
                    time.sleep(self.settings.poll_interval)
            raise TimeoutError(f"调用 execute 超时: {target}.{name}: {last_exc}")

        # 异步：单次 POST，立即返回 job dict（status=queued）
        response = self.post(
            f"{self.api_prefix}/execute",
            payload=payload,
            timeout=self.settings.request_timeout,
        )
        return response["job"]

    def cancel_job(self, job_id):
        """D.6 协作取消 job（D 路径新增）。

        立即返回 True/False，不等 SDK 完成。SDK 在下个 PID 循环检测到
        `_stop_flag` 后协作退出，job 状态置 failed。
        """
        response = self.post(
            f"{self.api_prefix}/jobs/{job_id}/stop",
            payload={},
            timeout=self.settings.request_timeout,
        )
        return response

    def call(self, target, name, *args, timeout=None, **kwargs):
        return self.execute(
            target=target,
            name=name,
            args=list(args),
            kwargs=kwargs,
            timeout=timeout,
        )

    def get_job(self, job_id):
        response = self.get(f"{self.api_prefix}/jobs/{job_id}")
        return response["job"]

    def wait_job(self, job_id, timeout=None, poll_interval=None):
        timeout = timeout or self.settings.wait_timeout
        poll_interval = poll_interval or self.settings.poll_interval
        start_time = time.time()
        while True:
            job = self.get_job(job_id)
            if job["status"] in {"succeeded", "failed"}:
                return job
            if time.time() - start_time > timeout:
                raise TimeoutError(f"等待任务超时: {job_id}")
            time.sleep(poll_interval)

    def wait_until_ready(self, timeout=None, poll_interval=None):
        deadline = self._deadline(timeout)
        poll_interval = poll_interval or 1.0
        last_exc = None
        while True:
            if time.time() > deadline:
                detail = f": {last_exc}" if last_exc is not None else ""
                raise TimeoutError(f"等待小车初始化超时{detail}")
            try:
                health = self.get_health(snapshot=False)
                state = health["state"]
                if state["initialized"]:
                    return health
                if state["last_error"]:
                    print(f"等待恢复... last_error={state['last_error']}")
                else:
                    print(
                        f"等待初始化... initialized={state['initialized']} "
                        f"initializing={state['initializing']}"
                    )
            except requests.RequestException as exc:
                last_exc = exc
                if not self._is_retryable_request_error(exc):
                    raise
                print(f"等待服务监听... {exc}")
            time.sleep(poll_interval)

    def init_runtime(self, force=False, reset_arm=False, reset_position=True):
        return self.post(
            f"{self.api_prefix}/control/init",
            payload={
                "force": force,
                "reset_arm": reset_arm,
                "reset_position": reset_position,
            },
        )

    def set_stop_mode(self, enabled):
        return self.post(
            f"{self.api_prefix}/control/stop-mode",
            payload={"enabled": enabled},
        )

    def reset_stop_flag(self):
        return self.post(f"{self.api_prefix}/control/reset-stop", payload={})

    def emergency_stop(self):
        return self.post(f"{self.api_prefix}/control/emergency-stop", payload={})

    def close_runtime(self):
        return self.post(f"{self.api_prefix}/control/close", payload={})

    # === 实时硬件直达（不走 /v1/execute，不进 job_queue） ===

    def realtime_wheel_speeds(self, speeds):
        return self.post(
            f"{self.api_prefix}/realtime/wheels/speeds",
            payload={"speeds": list(speeds)},
        )

    def realtime_wheel_encoders(self):
        return self.get(f"{self.api_prefix}/realtime/wheels/encoders")

    def realtime_motor_speed(self, port, speed, reverse=1):
        return self.post(
            f"{self.api_prefix}/realtime/motor/speed",
            payload={
                "port": int(port),
                "speed": float(speed),
                "reverse": int(reverse),
            },
        )

    def realtime_encoder(self, port, reverse=1):
        return self._request(
            "GET",
            f"{self.api_prefix}/realtime/encoder?port={int(port)}&reverse={int(reverse)}",
        )

    def realtime_stepper_rad(
        self, port, rad, time=0.5, reverse=1, perimeter=0.008
    ):
        return self.post(
            f"{self.api_prefix}/realtime/stepper/rad",
            payload={
                "port": int(port),
                "rad": float(rad),
                "time": float(time),
                "reverse": int(reverse),
                "perimeter": float(perimeter),
            },
        )

    def realtime_bus_servo_angle(self, port, angle, speed=100):
        return self.post(
            f"{self.api_prefix}/realtime/bus-servo/angle",
            payload={
                "port": int(port),
                "angle": float(angle),
                "speed": int(speed),
            },
        )

    def realtime_bus_servo_read(self, port):
        return self._request(
            "GET",
            f"{self.api_prefix}/realtime/bus-servo/angle?port={int(port)}",
        )

    def realtime_analog(self, port):
        return self._request(
            "GET", f"{self.api_prefix}/realtime/analog?port={int(port)}"
        )

    def realtime_analog2(self, port):
        return self._request(
            "GET", f"{self.api_prefix}/realtime/analog2?port={int(port)}"
        )

    def realtime_lane_state(self):
        """外环最常用：读 lane_feed 守护线程缓存的 lane_state。

        不进 job_queue、不打 ZMQ、不抢 car_lock——只取 streamer 的 meta_lock。
        50Hz+ 外环轮询安全；和数据源（lane_feed，runtime 默认 20Hz）的
        更新频率解耦，所以轮询再快也只会读到同一份最新缓存。

        返回 `{"lane_state": {"error_y": ..., "error_angle": ..., "active": ..., ...}}`。
        `error_y`/`error_angle` 为 None 时说明 lane_feed 未运行或刚刚启动。
        """
        return self._request("GET", f"{self.api_prefix}/realtime/lane/state")

    def get_arm_state(self):
        """读机械臂实时 y/x 位置(arm_feed 守护线程缓存,默认 20Hz 刷新)。

        与 get_lane_state 完全同构:不进 job_queue、不打 ZMQ、不抢 car_lock,
        只取 streamer 的 meta_lock(极快)。

        返回 `{"arm_state": {"y_m": ..., "x_m": ..., "y_mm": ..., "x_mm": ..., "ref_encoder": ..., "active": ...}}`。
        字段为 None 时说明 arm_feed 未运行或刚启动。
        """
        return self._request("GET", f"{self.api_prefix}/realtime/arm/state")

    def get_task_state(self):
        """读侧摄目标检测缓存(task_feed 守护线程,默认 10Hz 刷新)。

        "边走边看"侧摄目标的必需组件 —— 之前 /v1/vision/task 是 sync POST
        （5-15s 阻塞）,"边走边看"做不到。现在轮询本端点即可拿到最近一次检测结果。

        返回 `{"task_state": {"active": ..., "mode": ..., "detections": [...], "count": N, "updated_at": ...}}`。
        detections 是 list[dict],每个 dict 含 cls_id / det_id / label / score / bbox_norm。
        字段为 None 时说明 task_feed 未运行或刚启动。
        """
        return self._request("GET", f"{self.api_prefix}/realtime/vision/task")

    def run_task(self, name, *args, **kwargs):
        return self.create_job("task", name, args=list(args), kwargs=kwargs)

    def run_car_action(self, name, *args, **kwargs):
        return self.create_job("car", name, args=list(args), kwargs=kwargs)

    def run_arm_action(self, name, *args, **kwargs):
        return self.create_job("arm", name, args=list(args), kwargs=kwargs)

    def execute_task(self, name, *args, timeout=None, sync=False, **kwargs):
        # sync 是 execute 的元参数，不能漏到 action 的 kwargs 里（SDK action 不接受）。
        return self.execute(
            "task",
            name,
            args=list(args),
            kwargs=kwargs,
            timeout=timeout,
            sync=sync,
        )

    def execute_car_action(self, name, *args, timeout=None, sync=False, **kwargs):
        return self.execute(
            "car",
            name,
            args=list(args),
            kwargs=kwargs,
            timeout=timeout,
            sync=sync,
        )

    def execute_arm_action(self, name, *args, timeout=None, sync=False, **kwargs):
        return self.execute(
            "arm",
            name,
            args=list(args),
            kwargs=kwargs,
            timeout=timeout,
            sync=sync,
        )
