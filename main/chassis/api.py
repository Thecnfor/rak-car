"""main/chassis/api.py
只暴露底盘组真正会用到的 API 子集，不重复 main/API_REFERENCE.md 的全部接口。

约定：
- 只 import main.*，不 import smartcar / runtime
- 优先用 ws 长连接下发实时轮速；ws 不通则回退到 http realtime/* 接口

fast-path 约定（2026-07-31）：底盘外环频繁读的 IR / 里程计走 feed 缓存，
   不进 job_queue、不打 MC602、不抢 car_lock，单次 <2ms。
   详见 state.py 顶部注释 + runtime README § ir_feed / odom_feed。
"""
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

import time

from .state import LaneState, IrState, OdometryState

try:
    from main.api_client import RuntimeApiClient
    from main.ws_client import RuntimeWsClient
except ImportError:  # pragma: no cover
    from api_client import RuntimeApiClient  # type: ignore
    from ws_client import RuntimeWsClient  # type: ignore


@dataclass
class ChassisClient:
    """底盘专用 client。

    lane_state 两种读取模式（2026-08-09）：
      - **req/resp（旧）**：外环每帧 `read_lane()` → WS realtime_lane_state 一问一答
      - **推送（新，默认优先）**：`start_lane_subscription()` 起一条独立订阅连接，
        服务端 `lane_feed` 每帧推 `lane_state`，订阅线程写共享缓存，外环
        `read_lane()` 直接读缓存（零每帧 RTT）。订阅断线自动回退 req/resp。
    """

    http: RuntimeApiClient
    ws: RuntimeWsClient
    ws_ready: bool = False
    # ---- lane_state 推送共享缓存（订阅线程写 / 外环读，GIL 下引用赋值原子）----
    _lane_sub_stop: Optional[object] = None
    _latest_lane_state: Optional[dict] = None
    _latest_lane_mono: Optional[float] = None

    @classmethod
    def connect(cls) -> "ChassisClient":
        http = RuntimeApiClient()
        ws = RuntimeWsClient()
        ready = False
        try:
            ws.connect()
            ready = True
        except Exception:
            ready = False
        return cls(http=http, ws=ws, ws_ready=ready)

    # ---- lane_state 推送订阅（替代 50Hz req/resp 轮询）----

    def start_lane_subscription(self, hz: float = 50.0) -> bool:
        """起一条独立 WS 订阅连接，服务端 lane_feed 每帧推 lane_state。

        订阅线程 `on_state` 把最新帧写进共享缓存 `_latest_lane_state`，
        外环 `read_lane()` 零 RTT 读缓存。幂等：已订阅则直接返回 True。

        返回 True 表示订阅线程已启动（实际连通要等线程握手，`read_lane`
        内部按 `ws.lane_subscription_active` 判断新鲜度，未通自动回退 req/resp）。
        """
        try:
            stop = self.ws.subscribe_lane(self._on_lane_push, hz=hz)
            self._lane_sub_stop = stop
            return True
        except Exception:
            return False

    def stop_lane_subscription(self) -> None:
        """停订阅连接（幂等）。close() 收尾时调用。"""
        stop = self._lane_sub_stop
        self._lane_sub_stop = None
        if stop is not None:
            try:
                stop()
            except Exception:
                pass

    def _on_lane_push(self, state_dict: dict) -> None:
        """订阅线程回调：只做引用替换（外环读侧无锁安全）。"""
        self._latest_lane_state = state_dict
        self._latest_lane_mono = time.monotonic()

    def _lane_from_push_cache(self) -> Optional[LaneState]:
        """推送缓存新鲜且订阅线程存活 → 返回 LaneState；否则 None。

        新鲜度用本地 `time.monotonic()` 算（服务端 updated_at 是另一台机的
        wall clock，跨机时钟可能不同步——本地单调时钟最稳）。
        """
        if not self.ws.lane_subscription_active:
            return None
        if self._latest_lane_state is None or self._latest_lane_mono is None:
            return None
        age_ms = (time.monotonic() - self._latest_lane_mono) * 1000.0
        if age_ms >= 500.0:
            return None
        state = LaneState.from_lane_state_payload(self._latest_lane_state)
        state.age_ms = age_ms
        return state

    # ---- 业务动作 ----

    def start_lane_feed(self, hz: float = 50.0, timeout: float = 10.0):
        """车端：开一个守护线程只刷 lane_state 缓存，不下发轮速。

        注意：runtime init 默认已经启 lane_feed（50Hz），业务侧一般不需要调。
        timeout 留给未来同步语义扩展；当前服务端是瞬时返回。
        """
        return self.http.call("car", "start_lane_feed", hz=hz, timeout=timeout)

    def stop_lane_feed(self, timeout: float = 5.0):
        return self.http.call("car", "stop_lane_feed", timeout=timeout)

    def stop_wheel_speeds(self):
        """手动零速（兜底/异常路径用）。正常退出请调 ``close()``。

        ``close()`` 会自动发零速 + 收 ws —— 调用方在 finally 里只调 close()
        就够了，不要既调 stop_wheel_speeds 又调 close（重复发零速无副作用但不必要）。
        """
        return self.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])

    def emergency_stop(self):
        return self.http.emergency_stop()

    def move_for(
        self,
        dx_m: float = 0.0,
        dy_m: float = 0.0,
        timeout: float = 30.0,
        max_velocity_ms: float = 0.20,
    ) -> dict:
        """底盘相对位移 move_for (sync 阻塞到位后返回)。

        dx_m>0 前进 / <0 后退 (车体本地 x 偏移, 单位 m)。
        dy_m 是车体本地 y 偏移 (横移, 单位 m), 符号与 vy 一致:
            error_y>0 (车在线右) → dy_m>0 左移回中。
        走 runtime CAR_ACTIONS["move_for"] → 车端 car.move_for, 一次性位置闭环,
        走完自动停。theta_offset 恒 0 —— 直道回正不旋转, 保持 odom theta 为 0。
        返回 /v1/execute 同步 job dict (status/result/error)。

        Raises:
            RuntimeError: move_for job 失败 (status != succeeded)。
        """
        offset = [float(dx_m), float(dy_m), 0.0]  # [x_offset, y_offset, theta_offset] 单位 m
        job = self.http.execute_car_action(
            "move_for",
            offset,
            timeout=timeout,
            sync=True,
            max_velocities=[max_velocity_ms, max_velocity_ms, 3.14159 / 3],
        )
        ok = isinstance(job, dict) and job.get("status") == "succeeded"
        if not ok:
            status = job.get("status") if isinstance(job, dict) else type(job).__name__
            err = job.get("error") if isinstance(job, dict) else None
            raise RuntimeError("move_for failed: status=%r error=%r" % (status, err))
        return job

    # ---- 状态读取 ----

    def get_lane_state(self) -> dict:
        return self.http.get(f"{self.http.api_prefix}/vision/lane/state")

    def read_lane(self) -> LaneState:
        """外环每帧调这个：**推送缓存优先**，订阅断了回退 req/resp（WS→HTTP）。

        优先级（2026-08-09）：
          1. 推送订阅缓存（`start_lane_subscription()` 起的独立连接，零每帧 RTT）
          2. WS realtime_lane_state 一问一答
          3. HTTP /v1/vision/lane/state

        空 LaneState 的 has_error 为 False，控制律会自然输出零速，
        所以调用方不需要自己 try/except。
        """
        cached = self._lane_from_push_cache()
        if cached is not None:
            return cached
        try:
            if self.ws_ready:
                payload = self.ws.realtime_lane_state() or {}
            else:
                payload = self.get_lane_state() or {}
        except Exception:
            return LaneState()
        return LaneState.from_lane_state_payload(payload)

    def get_odometry(self, timeout: float = 5.0) -> Tuple[float, float, float]:
        """读底盘里程计。fast-path 走 odom_feed 缓存（runtime 默认 50Hz 刷新），
        fallback 走原 job_queue + car_lock 的慢路径。

        性能（fast-path）：
          - 不进 job_queue、不打 MC602、不抢 car_lock，只读 streamer meta_lock
          - 实测 50Hz+ 轮询安全，单次 round-trip <2ms
        适用：外环 / orchestrator 触发判定 / 业务层任何频繁读里程计的场景。

        返回：(x, y, theta) 单位 m / m / rad。
        """
        try:
            payload = self.http.get_odom_state() or {}
        except Exception:
            payload = {}
        odom = payload.get("odom_state") if isinstance(payload, dict) else None
        if isinstance(odom, dict) and odom.get("active") and odom.get("x") is not None:
            return float(odom["x"]), float(odom["y"]), float(odom["theta"])
        # fallback：odom_feed 未启动 / 异常退出 / 数据缺失 → 走原慢路径
        pos = self.http.call("car", "get_odometry", timeout=timeout)
        # get_odometry 返回 numpy array，走 normalize 后是 list
        if isinstance(pos, dict) and "result" in pos:
            pos = pos["result"]
        if hasattr(pos, "tolist"):
            pos = pos.tolist()
        return float(pos[0]), float(pos[1]), float(pos[2])

    def get_odometry_state(self) -> OdometryState:
        """读里程计缓存的 dataclass 视图（fast-path,return OdometryState）。

        等价于 `self.http.get_odom_state()` + `OdometryState.from_odom_state_payload`。
        需要字段访问(.x / .y / .theta / .distance / .age_ms / .is_fresh)的场景,
        比 get_odometry() 返回 tuple 更可读。

        失败（feed 未就绪 / 网络异常）→ OdometryState 全 None 字段(x/y/theta/distance/age_ms=None,
        mode="idle" 或 None),不抛异常。
        """
        try:
            payload = self.http.get_odom_state()
        except Exception:
            payload = None
        return OdometryState.from_odom_state_payload(payload or {})

    def get_ir_state(self) -> IrState:
        """读左右 IR 距离缓存的 dataclass 视图（fast-path,return IrState）。

        等价于 `self.http.get_ir_state()` + `IrState.from_ir_state_payload`。
        需要字段访问(.left / .right / .age_ms / .is_fresh / .active)的场景。

        失败（feed 未就绪 / 网络异常）→ IrState 全 None 字段,不抛异常。
        """
        try:
            payload = self.http.get_ir_state()
        except Exception:
            payload = None
        return IrState.from_ir_state_payload(payload or {})

    def get_ir(self, *, side: Optional[str] = None, timeout: float = 5.0):
        """读左右 IR 距离（双侧 / 单侧）。

        与 main/chassis/tasks/read_ir.read_ir 语义一致(用户视角 left/right)：
          - side=None(默认) → 返回 {"right": float, "left": float}(m),键是用户视角
          - side="left" / "right" → 返回 float(m)

        fast-path 走 ir_feed 缓存(单侧 / 双侧都是同一次 HTTP,无差别)，
        fallback `car.get_all_ir_distance` / `car.get_ir_distance` 同步直读
        走 runtime 的 _realtime_gate,不再造 job(2026-07-31)。
        """
        state = self.get_ir_state()
        # 缓存有数据 → 返回(任何 dict 都不需要 fallback)
        if state.active and (state.left is not None or state.right is not None):
            if side is None:
                # 与 read_ir 同步:dict 键是用户视角 {right, left},IrState 内部是用户视角 left/right
                return {"right": state.right, "left": state.left}
            s = str(side).lower()
            if s == "left":
                return state.left
            if s == "right":
                return state.right
            return None
        # fallback：ir_feed 未启动 / 异常退出 → 走慢路径(同步直读,_realtime_gate)
        if side is None:
            try:
                irs = self.http.call("car", "get_all_ir_distance", timeout=timeout)
                if isinstance(irs, dict) and "result" in irs:
                    irs = irs["result"]
                if isinstance(irs, dict):
                    # 调换：底层 left_sensor ↔ 用户视角 right
                    return {"right": irs.get("left"), "left": irs.get("right")}
                return irs
            except Exception:
                return None
        # 单侧 fallback
        flipped_side = "right" if str(side).lower() == "left" else "left" if str(side).lower() == "right" else str(side)
        try:
            ir = self.http.call(
                "car", "get_ir_distance",
                args=[flipped_side],
                timeout=timeout,
            )
            if isinstance(ir, dict) and "result" in ir:
                ir = ir["result"]
            return float(ir) if ir is not None else None
        except Exception:
            return None

    def get_wheel_encoders(self, timeout: float = 5.0):
        return self.http.realtime_wheel_encoders()

    # ---- 实时下发（优先 ws） ----

    def set_wheel_speeds(self, speeds: Iterable[float], timeout: float = 5.0):
        speeds = [float(s) for s in speeds]
        if self.ws_ready:
            try:
                r = self.ws.realtime_wheel_speeds(speeds, timeout=timeout)
                # 2026-08-07: runtime WS handler 出错时 ok=False 静默返回, 不 raise,
                # 外环会误以为已下发 → 车不走。显式检查, 失败回退 HTTP。
                if isinstance(r, dict) and not r.get("ok"):
                    raise RuntimeError(r.get("error", "ws ok=False"))
                return r
            except Exception:
                self.ws_ready = False
        return self.http.realtime_wheel_speeds(speeds)

    def set_chassis_velocity(self, vx: float, vy: float, wz: float = 0.0,
                             timeout: float = 5.0):
        """(vx, vy, wz) 直发 — runtime 内部 IK 反算 4 轮速。

        优先 ws 长连接（免每请求 TCP 握手）;ws 不通回退 HTTP keep-alive。
        """
        if self.ws_ready:
            try:
                r = self.ws.realtime_chassis_velocity(
                    vx, vy, wz, timeout=timeout)
                if isinstance(r, dict) and not r.get("ok"):
                    raise RuntimeError(r.get("error", "ws ok=False"))
                return r
            except Exception:
                self.ws_ready = False
        return self.http.post(
            f"{self.http.api_prefix}/realtime/chassis-velocity",
            payload={"vx": float(vx), "vy": float(vy), "wz": float(wz)},
            timeout=timeout,
        )

    # === runtime 侧巡线导航环（进程内闭环，2026-08-11） ===
    # 50Hz lane-follow 控制环下沉到 runtime：读 streamer 缓存 + 直发轮速，
    # 客户端只发低频生命周期调用（每次 mission / wait-key 启动一次）。
    # 旧 runtime（无 /realtime/lane-nav/* 端点）→ HTTP 404，调用方回退客户端
    # DoubleLoopRunner。

    def start_lane_nav(self, *, hz=50.0, controller_type="straight",
                       turn_cfg=None, watchdog_ms=500.0, lost_line_ms=None,
                       crossroad_turn=None, timeout=10.0) -> dict:
        """启动 runtime 巡线导航环（幂等：已跑返回 already_running）。"""
        payload = {
            "hz": float(hz),
            "controller_type": controller_type,
            "turn_cfg": turn_cfg or {},
            "watchdog_ms": watchdog_ms,
            "lost_line_ms": lost_line_ms,
            "crossroad_turn": crossroad_turn,
        }
        return self.http.post(
            f"{self.http.api_prefix}/realtime/lane-nav/start",
            payload=payload, timeout=timeout,
        )

    def pause_lane_nav(self, timeout=5.0) -> dict:
        """暂停 runtime 导航环（同步等零速 ack，进任务点前调用）。"""
        return self.http.post(
            f"{self.http.api_prefix}/realtime/lane-nav/pause",
            payload={"timeout": 1.0}, timeout=timeout,
        )

    def resume_lane_nav(self, timeout=5.0) -> dict:
        """恢复 runtime 导航环（loop 已死自动重建重启）。"""
        return self.http.post(
            f"{self.http.api_prefix}/realtime/lane-nav/resume",
            payload={}, timeout=timeout,
        )

    def stop_lane_nav(self, force=True, timeout=5.0) -> dict:
        """停止 runtime 导航环 + 兜底零速。"""
        return self.http.post(
            f"{self.http.api_prefix}/realtime/lane-nav/stop",
            payload={"force": bool(force)}, timeout=timeout,
        )

    def lane_nav_state(self, timeout=5.0) -> dict:
        """读 runtime 导航环状态 + 心跳（health.iter_count 每帧递增）。"""
        return self.http.get(
            f"{self.http.api_prefix}/realtime/lane-nav/state", timeout=timeout,
        )


    def chassis_align(self, **kwargs) -> dict:
        """底盘视觉对齐（下沉到 runtime）。

        单次 HTTP POST 到 /v1/realtime/chassis-align，阻塞 1-15s 直到
        arrived / timeout / watchdog / no_target，返回完整结果 dict。

        参数全部透传给 runtime ChassisAlignController，详见
        runtime/services/chassis_align.py::ChassisAlignController.__init__。
        """
        payload = dict(kwargs)
        # setpoint_cxcy 元组 → list（HTTP 可序列化）
        if "setpoint_cxcy" in payload:
            payload["setpoint_cxcy"] = list(payload["setpoint_cxcy"])
        return self.http.post(
            f"{self.http.api_prefix}/realtime/chassis-align",
            payload=payload,
            timeout=payload.get("max_seconds", 10.0) + 5.0,
        )

    def set_single_motor(self, port: int, speed: float, reverse: int = 1, timeout: float = 5.0):
        if self.ws_ready:
            try:
                return self.ws.realtime_motor_speed(port, speed, reverse=reverse, timeout=timeout)
            except Exception:
                self.ws_ready = False
        return self.http.realtime_motor_speed(port, speed, reverse=reverse)

    def ping(self, timeout: float = 5.0) -> bool:
        try:
            self.http.get_health(timeout=timeout)
            return True
        except Exception:
            return False

    def close(self) -> None:
        """退出收尾（#5）：自动发零速 + 停推送订阅 + 关 ws 长连接。

        流程：先发 [0,0,0,0]（即使 ws 断了也走 HTTP 兜底）→ 停独立订阅连接
        → 再关 ws 主连接。失败无所谓（进程要退了）。
        """
        # 自动发零速（#5）：以前 DoubleLoopRunner finally 与 stop_wheel_speeds 双重发，
        # 现在收敛到 close()。多次调用 close() 也安全（smoother 已清零，不会乱跳）。
        try:
            self.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])
        except Exception:
            pass
        # 推送订阅是独立连接（_PushSubscriber），不随 ws.close() 关闭——必须显式停
        self.stop_lane_subscription()
        try:
            self.ws.close()
        except Exception:
            pass
        self.ws_ready = False
