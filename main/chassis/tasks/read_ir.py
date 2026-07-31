"""main/chassis/tasks/read_ir.py
读取红外距离传感器（左右两侧）。

- side=None（默认）→ 同时读两侧，返回 {"right": float, "left": float}（单位 m）
- side="left" | "right" → 单侧返回 float（单位 m）

底层数据来源（2026-07-31 升级）：
  - fast-path：ir_feed 守护线程 50Hz 喂 streamer.ir_state 缓存，
    通过 /v1/realtime/ir/state 拉（不进 job_queue、不打 MC602、不抢 car_lock）。
  - fallback：feed 未就绪 / 异常的极小窗口，回退到原 car.get_all_ir_distance()
    同步 HTTP（_realtime_gate 路径，仅 1-2 次字节往返）。

底层接口：
- car.get_all_ir_distance() / car.get_ir_distance(side)
均注册在 runtime/core/actions.py 的 CAR_ACTIONS 里。
"""
from typing import Optional, Union

from ..api import ChassisClient


IrReading = Union[float, dict]


# 调换 left / right
_FLIP_SIDE = {"left": "right", "right": "left"}


def _flip_ir_dict(data: dict) -> dict:
    """把 {"left": L, "right": R} 调换为 {"right": L, "left": R}。"""
    return {
        "right": data.get("left"),
        "left": data.get("right"),
    }


def _read_ir_fast(api: ChassisClient) -> Optional[dict]:
    """从 ir_feed 缓存拉一次两侧 IR；失败或 feed 未就绪返回 None。"""
    try:
        payload = api.http.get_ir_state()
    except Exception:
        return None
    if not isinstance(payload, dict):
        return None
    ir = payload.get("ir_state")
    if not isinstance(ir, dict) or not ir.get("active"):
        return None
    left = ir.get("left")
    right = ir.get("right")
    if left is None and right is None:
        return None
    return {"left": left, "right": right}


def read_ir(
    api: ChassisClient,
    *,
    side: Optional[str] = None,
    timeout: float = 5.0,
) -> IrReading:
    """读取 IR 距离传感器（对外语义：side 是用户视角的左/右，底层已调换）。

    Args:
        api: ChassisClient（main/chassis/api.py）。
        side: "left" / "right" 读单侧；None（默认）读双侧。
        timeout: 慢路径（fallback）HTTP 调用超时秒数。

    Returns:
        单侧：float（m）。双侧：{"right": float, "left": float}（m，键为用户视角）。

    路径：
      1) fast-path：/v1/realtime/ir/state 缓存读（推荐，<2ms）
      2) fallback：car.get_all_ir_distance (sync, _realtime_gate，~10-30ms)
      3) 兼容旧行为：失败时返回 None（业务层已有 try/except 兜底）
    """
    if side is None:
        fast = _read_ir_fast(api)
        if fast is not None:
            return _flip_ir_dict(fast)
        # fallback：旧 execute 路径（保留语义兼容；runtime 升级后通常不会走到这里）
        try:
            job = api.http.execute(
                "car", "get_all_ir_distance", timeout=timeout, sync=True
            )
            result = job["result"]
            if isinstance(result, dict):
                return _flip_ir_dict(result)
            return result
        except Exception:
            return None

    # 单侧：fast-path 拿双侧后裁
    fast = _read_ir_fast(api)
    if fast is not None:
        val = fast.get(_FLIP_SIDE.get(str(side), "left"))
        return float(val) if val is not None else 0.0
    flipped_side = _FLIP_SIDE.get(str(side), str(side))
    try:
        job = api.http.execute(
            "car",
            "get_ir_distance",
            args=[flipped_side],
            timeout=timeout,
            sync=True,
        )
        return job["result"]
    except Exception:
        return None
