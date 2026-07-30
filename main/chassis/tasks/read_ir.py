"""main/chassis/tasks/read_ir.py
读取红外距离传感器（左右两侧）。

- side=None（默认）→ 同时读两侧，返回 {"right": float, "left": float}（单位 m）
- side="left" | "right" → 传返回 float（单位 m）

底层接口：
- car.get_ir_distance(side="left"|"right")
- car.get_all_ir_distance()
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
        timeout: HTTP 调用超时秒数。

    Returns:
        单侧：float（m）。双侧：{"right": float, "left": float}（m，键为用户视角）。
    """
    if side is None:
        job = api.http.execute("car", "get_all_ir_distance", timeout=timeout, sync=True)
        result = job["result"]
        if isinstance(result, dict):
            return _flip_ir_dict(result)
        return result
    else:
        flipped_side = _FLIP_SIDE.get(str(side), str(side))
        job = api.http.execute(
            "car",
            "get_ir_distance",
            args=[flipped_side],
            timeout=timeout,
            sync=True,
        )
        return job["result"]