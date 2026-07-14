#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import time
import uuid

try:
    from websocket import (
        WebSocketConnectionClosedException,
        WebSocketTimeoutException,
        create_connection,
    )
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 websocket-client 依赖，请先执行: python3 -m pip install -r "
        "/home/jetson/workspace/rak-car/main/requirements.txt"
    ) from exc

try:
    from .settings import load_settings
except ImportError:  # pragma: no cover
    from settings import load_settings


def build_ws_url(api_base, api_prefix):
    base = api_base.rstrip("/")
    if base.startswith("https://"):
        ws_base = "wss://" + base[len("https://") :]
    elif base.startswith("http://"):
        ws_base = "ws://" + base[len("http://") :]
    else:
        ws_base = base
    return f"{ws_base}{api_prefix}/ws"


class RuntimeWsClient:
    def __init__(self, settings=None):
        self.settings = settings or load_settings()
        self.ws_url = build_ws_url(self.settings.api_base, self.settings.api_prefix)
        self._conn = None
        self._welcome = None

    @property
    def welcome(self):
        return self._welcome

    def connect(self, timeout=None, force=False):
        if self._conn is not None and not force:
            return self._welcome
        self.close()
        timeout = self.settings.request_timeout if timeout is None else float(timeout)
        self._conn = create_connection(self.ws_url, timeout=timeout)
        self._conn.settimeout(timeout)
        welcome = self._recv_json()
        self._welcome = welcome
        return welcome

    def close(self):
        if self._conn is None:
            return
        try:
            self._conn.close()
        except Exception:
            pass
        self._conn = None

    def _recv_json(self):
        if self._conn is None:
            raise RuntimeError("WebSocket 尚未连接")
        message = self._conn.recv()
        return json.loads(message)

    def request(self, op, request_timeout=None, auto_reconnect=True, **payload):
        request_timeout = (
            self.settings.request_timeout
            if request_timeout is None
            else float(request_timeout)
        )
        request_id = payload.pop("request_id", str(uuid.uuid4())[:8])
        body = {"op": op, "request_id": request_id}
        body.update(payload)
        last_exc = None
        for attempt in range(2 if auto_reconnect else 1):
            try:
                self.connect(timeout=request_timeout, force=(attempt > 0))
                self._conn.settimeout(request_timeout)
                self._conn.send(json.dumps(body, ensure_ascii=False))
                response = self._recv_json()
                if response.get("request_id") == request_id:
                    return response
                return response
            except (
                OSError,
                RuntimeError,
                WebSocketTimeoutException,
                WebSocketConnectionClosedException,
            ) as exc:
                last_exc = exc
                self.close()
                if attempt == 0 and auto_reconnect:
                    time.sleep(self.settings.poll_interval)
                    continue
                raise
        raise RuntimeError(str(last_exc))

    def ping(self):
        return self.request("ping")

    def health(self, snapshot=False, timeout=None):
        return self.request(
            "health",
            request_timeout=timeout,
            snapshot=1 if snapshot else 0,
        )

    def runtime(self, timeout=None):
        return self.request("runtime", request_timeout=timeout)

    def actions(self, timeout=None):
        return self.request("actions", request_timeout=timeout)

    def execute(self, target, name, args=None, kwargs=None, timeout=None):
        payload = {
            "target": target,
            "name": name,
            "args": args or [],
            "kwargs": kwargs or {},
        }
        if timeout is not None:
            payload["timeout"] = timeout
        return self.request("execute", request_timeout=timeout, **payload)

    def create_job(self, target, name, args=None, kwargs=None, timeout=None):
        return self.request(
            "create_job",
            request_timeout=timeout,
            target=target,
            name=name,
            args=args or [],
            kwargs=kwargs or {},
        )

    # === 实时硬件直达 op（car_lock 同步路径，不进 job_queue） ===

    def realtime_wheel_speeds(self, speeds, timeout=None):
        return self.request(
            "realtime/wheel_speeds",
            request_timeout=timeout,
            speeds=list(speeds),
        )

    def realtime_wheel_encoders(self, timeout=None):
        return self.request("realtime/wheel_encoders", request_timeout=timeout)

    def realtime_lane_state(self, timeout=None):
        resp = self.request("realtime/lane_state", request_timeout=timeout)
        return (resp.get("data") or {}).get("lane_state") or {}

    def realtime_motor_speed(self, port, speed, reverse=1, timeout=None):
        return self.request(
            "realtime/motor_speed",
            request_timeout=timeout,
            port=int(port),
            speed=float(speed),
            reverse=int(reverse),
        )

    def realtime_encoder(self, port, reverse=1, timeout=None):
        return self.request(
            "realtime/encoder",
            request_timeout=timeout,
            port=int(port),
            reverse=int(reverse),
        )

    def realtime_stepper_rad(
        self, port, rad, time=0.5, reverse=1, perimeter=0.008, timeout=None
    ):
        return self.request(
            "realtime/stepper_rad",
            request_timeout=timeout,
            port=int(port),
            rad=float(rad),
            time=float(time),
            reverse=int(reverse),
            perimeter=float(perimeter),
        )

    def realtime_bus_servo_angle(self, port, angle, speed=100, timeout=None):
        return self.request(
            "realtime/bus_servo_angle",
            request_timeout=timeout,
            port=int(port),
            angle=float(angle),
            speed=int(speed),
        )

    def realtime_bus_servo_read(self, port, timeout=None):
        return self.request(
            "realtime/bus_servo_read",
            request_timeout=timeout,
            port=int(port),
        )

    def realtime_analog(self, port, timeout=None):
        return self.request(
            "realtime/analog", request_timeout=timeout, port=int(port)
        )

    def realtime_analog2(self, port, timeout=None):
        return self.request(
            "realtime/analog2", request_timeout=timeout, port=int(port)
        )


if __name__ == "__main__":
    client = RuntimeWsClient()
    print(json.dumps(client.connect(), ensure_ascii=False, indent=2))
    print(json.dumps(client.health(), ensure_ascii=False, indent=2))
