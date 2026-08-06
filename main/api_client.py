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
        # 2026-08-03：持久 Session = HTTP keep-alive。旧实现每请求 requests.request()
        # 新建 TCP 连接,50Hz 外环每次多付一个 RTT 的握手成本（~20-40ms LAN）。
        # Session 内部连接池线程安全（每次调用从池取连接）。
        self._session = self._build_session()

    def _build_session(self):
        """装配：连接池 + Retry。

        改造点（不暴露给调用方）：
          1) 给默认 adapter 挂 Retry：幂等 GET 自动重试 1 次，幂等 method
             pool（HEAD/GET/OPTIONS）也覆盖。POST 等非幂等默认不重试，避免
             重复下指令伤硬件。
          2) pool_maxsize 调到 16，单 host 不会出现"pool full, use non-pooled
             connection"那条 warning 偷跑（50Hz × 3 端点 = 150 req/s，外加
             orchestrator/health 等并发，10 个连接偏紧）。
          3) Retry 不抬高 connect timeout — 留给调用方的 _request(timeout=...)。
        """
        from requests.adapters import HTTPAdapter
        from urllib3.util.retry import Retry

        session = requests.Session()
        retry = Retry(
            total=1,
            connect=1,
            read=0,
            redirect=0,
            status=1,
            allowed_methods=frozenset(["HEAD", "GET", "OPTIONS"]),
            status_forcelist=(502, 503, 504),
            backoff_factor=0.05,
            raise_on_status=False,
        )
        adapter = HTTPAdapter(
            pool_connections=4,
            pool_maxsize=16,
            max_retries=retry,
        )
        session.mount("http://", adapter)
        session.mount("https://", adapter)
        return session

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
        response = self._session.request(
            method=method,
            url=self.build_url(path),
            json=payload,
            timeout=timeout,
        )
        response.raise_for_status()
        return response.json() if response.content else {}

    def get(self, path, timeout=None):
        return self._request("GET", path, timeout=timeout)

    # 2026-07-31: 视觉伺服封装需要的 vision 调用方法（VISION_SERVO_DESIGN.md §3）。
    # - request_vision_task: 单次同步推理（带 bbox_pixels + filter）
    # - get_vision_task_cache: 读 task_feed 30Hz 缓存（视觉伺服主路径）
    def request_vision_task(
        self,
        *,
        sort_pos=(0.0, 0.0),
        limit_x: float = 1.0,
        limit_y: float = 1.0,
        timeout: float = 20.0,
    ):
        """POST /v1/vision/task — 同步单次推理（含 bbox_pixels）。

        返回 runtime JSON 原样 dict，由 vision.py 层负责解析。
        """
        return self._request(
            "POST",
            f"{self.api_prefix}/vision/task",
            payload={
                "sort_pos": [float(sort_pos[0]), float(sort_pos[1])],
                "limit_x": float(limit_x),
                "limit_y": float(limit_y),
                "timeout": float(timeout),
            },
            timeout=timeout + 5.0,
        )

    def get_vision_task_cache(self):
        """GET /v1/realtime/vision/task — 读 task_feed 30Hz 缓存。"""
        return self._request("GET", f"{self.api_prefix}/realtime/vision/task")

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

        异步模式（默认 sync=False）：
          - 不轮询，调用方自行 `get_job(job_id)` 查状态或 `cancel_job(job_id)` 取消。
          - 适合：上层编排一边跑一边发 wheel_speeds / 想拿 job_id 做并行编排。

        同步模式（sync=True）：
          - 阻塞返回 job dict（status=succeeded/failed）。
          - 2026-08-07 优化: 旧版 ``sync=True`` 在 deadline 内**重复 POST 同一 payload**
            到 ``/v1/execute`` —— 每次重试 runtime 都会创建新 job_, 抢 job_queue、
            jobs 表 30+ 写入/清理。改为 ``sync=False + wait_job`` 1 次 POST + N 次 GET,
            省 N-1 次 POST 字节往返 + runtime 重复 job 调度。
          - 适合：链式编排（`move_xy → grasp → release`），业务层要等结果才能下一步。

        用 `execute_arm_action` / `execute_car_action` / `call` 的旧调用方，arm 长动作
        （reset_y / reset_origin / move_xy / move_x / move_y / set_side /
        set_hand / set_storage / grasp）已显式加 sync=True（见 main/arm/api.py），行为
        不变。注：reset_x 已删除（2026-07-16）。
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
            # 2026-08-07: 改 sync=True 走 sync=False + wait_job 范式。
            # 旧实现重复 POST /v1/execute 同 payload，每次 runtime 收单进 job_queue
            # AND 写入 jobs 表——3s 物理动作 + 0.1s poll_interval = ~30 次 POST /
            # ~30 个重复 job entry。现在 1 次 POST + N 次 GET /v1/jobs/{id},
            # GET 极轻量、不进 job_queue、runtime 直接读 status。
            # Backward compat: 仍返回 dict ({"status": ..., "result": ...} 等 job 字段)。
            deadline = self._deadline(timeout)
            last_exc = None
            # 首次 POST 拿 job_id
            try:
                response = self.post(
                    f"{self.api_prefix}/execute",
                    payload={**payload, "sync": False},
                    timeout=self.settings.request_timeout,
                )
                job = response.get("job") if isinstance(response, dict) else None
                if not isinstance(job, dict) or not job.get("id"):
                    return job or {}
                job_id = job["id"]
            except requests.RequestException as exc:
                last_exc = exc
                if not self._is_retryable_request_error(exc):
                    raise
                raise TimeoutError(f"调用 execute 起始 POST 失败: {target}.{name}: {last_exc}")
            # 后续纯 GET 轮询
            return self.wait_job(
                job_id, timeout=timeout,
                poll_interval=self.settings.poll_interval,
            )

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
        """轮询 job 状态到 succeeded/failed。

        2026-08-07 优化: 默认 ``poll_interval`` 提到 0.3s（settings 默认）。
        早期 0.1s 间隔意味着 3s 物理动作产生 ~30 次 GET, 浪费 CPU + 网卡
        字节往返; 0.3s 间隔意味着 10 次 GET, 最坏 0.3s 延迟感知 (肉眼看
        composite_run 物理动作 ~2-3s, 0.3s 粒度无差别)。
        外部显式传 ``poll_interval`` 不变 (task1_seeding 等自己管理轮询)。
        """
        timeout = timeout or self.settings.wait_timeout
        poll_interval = poll_interval if poll_interval is not None else self.settings.poll_interval
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
        50Hz+ 外环轮询安全；和数据源（lane_feed，runtime 默认 50Hz，2026-07-16 上调）的
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

    # === 2026-07-31：左右 IR / 底盘里程计 fast-path（与 get_lane_state / get_arm_state 同构）===
    def get_ir_state(self):
        """读 ir_feed 守护线程缓存的左右 IR 距离（runtime 默认 50Hz 刷新）。

        不进 job_queue、不打 ZMQ、不抢 car_lock——只取 streamer 的 meta_lock。
        这是 main/chassis/tasks/read_ir.py 的 fast-path:业务层不再每次
        触发 /v1/execute → car_queue → MC602 字节往返。

        返回 `{"ir_state": {"active": ..., "mode": ..., "left": m, "right": m, "updated_at": ...}}`。
        left/right 为 None 时说明 ir_feed 未运行或刚启动。
        """
        return self._request("GET", f"{self.api_prefix}/realtime/ir/state")

    def get_odom_state(self):
        """读 odom_feed 守护线程缓存的底盘里程计（runtime 默认 50Hz 刷新）。

        不进 job_queue、不打 ZMQ、不抢 car_lock——只取 streamer 的 meta_lock。
        这是 main/chassis/api.py.get_odometry 的 fast-path:业务层不再每次
        触发 /v1/execute → car_queue → car_lock。

        返回 `{"odom_state": {"active": ..., "mode": ..., "x": m, "y": m, "theta": rad, "distance": m, "updated_at": ...}}`。
        """
        return self._request("GET", f"{self.api_prefix}/realtime/odom/state")

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
