"""main/chassis/api.py
只暴露底盘组真正会用到的 API 子集，不重复 main/API_REFERENCE.md 的全部接口。

约定：
- 只 import main.*，不 import smartcar / runtime
- 优先用 ws 长连接下发实时轮速；ws 不通则回退到 http realtime/* 接口
"""
from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

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
        """车端：开一个守护线程只刷 lane_state 缓存，不下发轮速。"""
        return self.http.call("car", "start_lane_feed", hz=hz, timeout=timeout)

    def stop_lane_feed(self, timeout: float = 5.0):
        return self.http.call("car", "stop_lane_feed", timeout=timeout)

    def stop_wheel_speeds(self):
        """零速：客户端外环退出前必须发。"""
        return self.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])

    def emergency_stop(self):
        return self.http.emergency_stop()

    # ---- 状态读取 ----

    def get_lane_state(self) -> dict:
        return self.http.get(f"{self.http.api_prefix}/vision/lane/state")

    def get_odometry(self, timeout: float = 5.0) -> Tuple[float, float, float]:
        pos = self.http.call("car", "get_odometry", timeout=timeout)
        # get_odometry 返回 numpy array，走 normalize 后是 list
        if isinstance(pos, dict) and "result" in pos:
            pos = pos["result"]
        if hasattr(pos, "tolist"):
            pos = pos.tolist()
        return float(pos[0]), float(pos[1]), float(pos[2])

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
