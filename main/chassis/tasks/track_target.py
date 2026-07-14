"""main/chassis/tasks/track_target.py
对侧摄检测目标做对齐：把车端的 car.move_to_detection_target 包一层。
不需要外环；这是内环事件。
"""
from typing import Optional

from ..api import ChassisClient


def track_target(
    api: ChassisClient,
    *,
    label: Optional[str] = None,
    delta_x: float = 0.0,
    delta_y: Optional[float] = 0.0,
    time_out: float = 3.0,
) -> dict:
    """调用 car.move_to_detection_target。返回 job 字典；timeout 兜底。"""
    job = api.http.call(
        "car",
        "move_to_detection_target",
        label=label,
        delta_x=delta_x,
        delta_y=delta_y,
        time_out=time_out,
        timeout=max(time_out + 5.0, 10.0),
    )
    return job
