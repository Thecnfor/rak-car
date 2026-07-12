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

    def execute(self, target, name, args=None, kwargs=None, timeout=None):
        payload = {
            "target": target,
            "name": name,
            "args": args or [],
            "kwargs": kwargs or {},
        }
        if timeout is not None:
            payload["timeout"] = timeout
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

    def run_task(self, name, *args, **kwargs):
        return self.create_job("task", name, args=list(args), kwargs=kwargs)

    def run_car_action(self, name, *args, **kwargs):
        return self.create_job("car", name, args=list(args), kwargs=kwargs)

    def run_arm_action(self, name, *args, **kwargs):
        return self.create_job("arm", name, args=list(args), kwargs=kwargs)

    def execute_task(self, name, *args, timeout=None, **kwargs):
        return self.execute(
            "task",
            name,
            args=list(args),
            kwargs=kwargs,
            timeout=timeout,
        )

    def execute_car_action(self, name, *args, timeout=None, **kwargs):
        return self.execute(
            "car",
            name,
            args=list(args),
            kwargs=kwargs,
            timeout=timeout,
        )

    def execute_arm_action(self, name, *args, timeout=None, **kwargs):
        return self.execute(
            "arm",
            name,
            args=list(args),
            kwargs=kwargs,
            timeout=timeout,
        )
