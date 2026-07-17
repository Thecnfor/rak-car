#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import threading
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

    def realtime_lane_state(self, timeout=None):
        """外环最常用：读 lane_feed 守护线程缓存的 lane_state。

        不进 job_queue、不打 ZMQ、不抢 car_lock——只取 streamer 的 meta_lock。
        50Hz+ 外环轮询安全；和数据源（lane_feed，runtime 默认 50Hz，2026-07-16 上调）的
        更新频率解耦，所以轮询再快也只会读到同一份最新缓存。

        返回 `{"lane_state": {"error_y": ..., "error_angle": ..., "active": ..., ...}}`。
        `error_y`/`error_angle` 为 None 时说明 lane_feed 未运行或刚刚启动。
        """
        return self.request("realtime/lane_state", request_timeout=timeout)

    # === 推送订阅 ===

    def subscribe_lane(self, on_state, hz=20.0):
        """订阅 lane_state 推送——服务端按 `updated_at` 变化主动推，免客户端轮询。

        行为：
          - 内部**独立开一条** WebSocket 连接（不复用主连接），避免推送帧
            和主连接的请求/响应相互干扰。
          - 服务端按 `lane_feed` 的更新节奏（默认 50Hz，2026-07-16 上调）推送 `lane_state` dict。
          - 调用 `on_state(lane_state_dict)`；on_state 抛异常不会中断订阅。

        参数：
          on_state: callable(dict) -> None；lane_state 字典，回调里只读。
          hz: 服务端订阅频率提示（实际频率受 lane_feed 限制）。

        返回：unsubscribe() callable。多次调用安全（幂等）。

        用法：
          client = RuntimeWsClient(); client.connect()
          stop = client.subscribe_lane(lambda s: print(s['error_y']))
          # ... 运行若干秒 ...
          stop()  # 断开订阅连接
        """
        return self._subscribe_push(
            slot_attr="_lane_subscriber",
            subscribe_op="subscribe_lane",
            push_op="lane_state",
            on_state=on_state,
            hz=hz,
        )

    def subscribe_arm_state(self, on_state, hz=20.0):
        """订阅 arm_state 推送——机械臂 y/x 实时位置。

        行为与 `subscribe_lane` 完全一致:
          - 独立 WS 连接,服务端按 `arm_feed` 节奏(默认 20Hz)推 `arm_state` dict
          - 字段:`y_m`/`x_m`(SDK m),`y_mm`/`x_mm`(业务 mm),`ref_encoder`(丢步核对)

        用法:
          stop = client.subscribe_arm_state(lambda s: print(s['y_mm'], s['x_mm']))
          # ...
          stop()
        """
        return self._subscribe_push(
            slot_attr="_arm_subscriber",
            subscribe_op="subscribe_arm_state",
            push_op="arm_state",
            on_state=on_state,
            hz=hz,
        )

    def subscribe_task_detection(self, on_state, hz=10.0):
        """订阅侧摄目标检测推送——"边走边看"侧摄目标。

        服务端 task_feed 守护线程默认 10Hz 推 `task_state` dict(同 lane/arm 模式):
          - 独立 WS 连接
          - 字段:`active`,`mode`,`detections` (list[{cls_id, det_id, label, score, bbox_norm}]),
            `count`,`updated_at`

        之前 /v1/vision/task 是 sync POST（5-15s 阻塞）,"边走边看"做不到。
        现在业务层可以一边发轮速一边收 detection,真正实现实时闭环。

        用法:
          stop = client.subscribe_task_detection(lambda s: print(s['label'], s['score']))
          # ...
          stop()
        """
        return self._subscribe_push(
            slot_attr="_task_subscriber",
            subscribe_op="subscribe_task_detection",
            push_op="task_state",
            on_state=on_state,
            hz=hz,
        )

    def _subscribe_push(self, slot_attr, subscribe_op, push_op, on_state, hz):
        """通用推送订阅,被 subscribe_lane / subscribe_arm_state 共用。"""
        existing = getattr(self, slot_attr, None)
        if existing is not None and existing.is_alive():
            return existing.stop
        sub = _PushSubscriber(
            ws_url=self.ws_url,
            on_state=on_state,
            poll_interval=max(1.0 / max(float(hz), 1.0), 0.001),
            subscribe_op=subscribe_op,
            push_op=push_op,
        )
        sub.start()
        setattr(self, slot_attr, sub)
        return sub.stop

    @property
    def lane_subscription_active(self):
        sub = getattr(self, "_lane_subscriber", None)
        return sub is not None and sub.is_alive()

    @property
    def arm_subscription_active(self):
        sub = getattr(self, "_arm_subscriber", None)
        return sub is not None and sub.is_alive()


class _PushSubscriber:
    """独立 WebSocket 连接,通用推送订阅(lane_state / arm_state 共用)。

    独立连接的设计目的:避免推送帧和主连接的 req/rep 流相互抢占——
    websocket-client 是单 conn 单 recv,独立连接让两条流零干扰。
    服务端 asyncio 同时跑 N 条 WS 连接的代价可忽略。
    """

    def __init__(self, ws_url, on_state, poll_interval, subscribe_op, push_op):
        self._ws_url = ws_url
        self._on_state = on_state
        self._poll_interval = poll_interval
        self._subscribe_op = subscribe_op
        self._push_op = push_op
        self._stop_event = threading.Event()
        self._thread = None
        self._conn = None
        self.push_count = 0
        self.error_count = 0

    def start(self):
        self._thread = threading.Thread(
            target=self._run, name="ws-subscriber-" + self._subscribe_op, daemon=True
        )
        self._thread.start()

    def is_alive(self):
        return self._thread is not None and self._thread.is_alive()

    def stop(self):
        self._stop_event.set()
        conn = self._conn
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self):
        try:
            self._conn = create_connection(self._ws_url, timeout=2.0)
            # server 立刻发 welcome,先吃掉
            try:
                self._conn.settimeout(2.0)
                self._conn.recv()
            except Exception:
                pass
            # 发订阅请求;服务端的 ack 也会通过同一个连接回,先吃掉
            self._conn.send(
                json.dumps({"op": self._subscribe_op, "hz": 1.0 / self._poll_interval})
            )
            try:
                self._conn.settimeout(2.0)
                ack = self._conn.recv()
                ack_data = json.loads(ack)
                if not ack_data.get("ok"):
                    return
            except Exception:
                return
            # 主循环:等推送
            while not self._stop_event.is_set():
                try:
                    self._conn.settimeout(1.0)
                    raw = self._conn.recv()
                except WebSocketTimeoutException:
                    continue
                except (OSError, WebSocketConnectionClosedException):
                    break
                except Exception:
                    self.error_count += 1
                    if self.error_count > 5:
                        break
                    continue
                try:
                    data = json.loads(raw)
                except Exception:
                    continue
                if data.get("op") != self._push_op:
                    continue
                self.push_count += 1
                payload = data.get("data") or {}
                try:
                    self._on_state(payload)
                except Exception:
                    # 回调抛异常不能让订阅线程死
                    self.error_count += 1
        finally:
            try:
                if self._conn is not None:
                    self._conn.close()
            except Exception:
                pass


if __name__ == "__main__":
    client = RuntimeWsClient()
    print(json.dumps(client.connect(), ensure_ascii=False, indent=2))
    print(json.dumps(client.health(), ensure_ascii=False, indent=2))
