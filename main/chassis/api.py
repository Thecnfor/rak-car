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

from .state import LaneState, IrState, OdometryState

try:
    from main.api_client import RuntimeApiClient
    from main.ws_client import RuntimeWsClient
except ImportError:  # pragma: no cover
    from api_client import RuntimeApiClient  # type: ignore
    from ws_client import RuntimeWsClient  # type: ignore


@dataclass
class ChassisClient:
    """底盘专用 client。"""

    http: RuntimeApiClient
    ws: RuntimeWsClient
    ws_ready: bool = False

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

    # ---- 状态读取 ----

    def get_lane_state(self) -> dict:
        return self.http.get(f"{self.http.api_prefix}/vision/lane/state")

    def read_lane(self) -> LaneState:
        """外环每帧调这个：ws 通就走 ws，不通回退 http，异常返回空 LaneState。

        空 LaneState 的 has_error 为 False，控制律会自然输出零速，
        所以调用方不需要自己 try/except。
        """
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
                return self.ws.realtime_wheel_speeds(speeds, timeout=timeout)
            except Exception:
                self.ws_ready = False
        return self.http.realtime_wheel_speeds(speeds)

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
        """退出收尾（#5）：自动发零速 + 关 ws 长连接。

        流程：先发 [0,0,0,0]（即使 ws 断了也走 HTTP 兜底）→ 再关 ws。
        失败无所谓（进程要退了）。
        """
        # 自动发零速（#5）：以前 DoubleLoopRunner finally 与 stop_wheel_speeds 双重发，
        # 现在收敛到 close()。多次调用 close() 也安全（smoother 已清零，不会乱跳）。
        try:
            self.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])
        except Exception:
            pass
        try:
            self.ws.close()
        except Exception:
            pass
        self.ws_ready = False
