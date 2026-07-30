"""main/chassis/tasks/monitor_ir.py
持续读取 IR 距离传感器，命中阈值时回调。

设计目的：
- 车端某些子任务（典型：`auto_seeding`）需要被外部事件打断并恢复，
  打断信号只能从底盘侧的"红外靠近"事件里取（典型：右侧 IR 超阈值）。
- 这个任务**只负责采样 + 命中回调**，不直接跳流程；调用方拿到回调后
  自行实现"暂停主循环 → 跑子任务 → 恢复"的逻辑。

读侧约定：
- 本任务复用 `tasks/read_ir.py` 的 `read_ir(api, side=...)` 接口，分别
  以 side="left" / side="right" 调一次，得到两个 float（m）；
- 阈值比较是 `value > threshold_m`；IR 测距一般是"距离越大越远"，
  若物理语义是"靠近触发"，把判断改成 `<` 即可。

用法::

    from main.chassis import ChassisClient
    from main.chassis.tasks.monitor_ir import monitor_ir

    api = ChassisClient.connect()

    def on_signal(side, value):
        # side  = "left" | "right"
        # value = 当前距离（m）
        if side == "right":
            ...  # 右侧靠墙 → 暂停巡线、跑 auto_seeding

    monitor_ir(
        api,
        threshold_m=0.10,
        hz=20.0,
        max_seconds=120.0,
        on_alert=on_signal,
    )

边界：
- 不下发轮速（不像 `follow_lane`），只是一个常驻采样循环
- 任何异常路径都不强制 `stop_wheel_speeds()`——
  调用方自己负责轮速兜底
"""
from __future__ import annotations

import time
from typing import Callable, Optional

from ..api import ChassisClient
from .read_ir import read_ir


# 回调签名：side 是哪一侧命中，value 是该侧当前距离（m）。
IRAlertCallback = Callable[[str, float], None]
# 每帧回调：left/right 两个 float（m），便于 TUI / 调试日志。
IRTickCallback = Callable[[float, float], None]


def monitor_ir(
    api: ChassisClient,
    *,
    threshold_m: float = 0.25,
    hz: float = 20.0,
    max_seconds: Optional[float] = None,
    on_alert: Optional[IRAlertCallback] = None,
    on_tick: Optional[IRTickCallback] = None,
    timeout: float = 2.0,
) -> None:
    """持续采样 IR，命中阈值时调 on_alert（每帧每侧最多一次）。

    Args:
        api: ChassisClient。
        threshold_m: 单边触发阈值（m）。`value < threshold_m` 即命中。
        hz: 采样频率。底盘场景建议 10-50Hz，过高会浪费 HTTP 调用。
        max_seconds: 最大运行时长，None 表示一直跑（依赖 Ctrl-C 终止）。
        on_alert: 阈值命中回调，签名 `(side, value)`；本帧哪一侧命中就
            对应侧调一次（最多两次）。
        on_tick: 每帧都会调，签名 `(left, right)`，两 float（m），
            任一侧读失败传 None。
        timeout: 单次 HTTP 读 IR 的超时秒数。
    """
    if hz <= 0:
        raise ValueError("hz must be > 0")
    dt = 1.0 / hz
    deadline = None if max_seconds is None else time.monotonic() + max_seconds

    try:
        while True:
            t0 = time.monotonic()

            if deadline is not None and t0 >= deadline:
                break

            left: Optional[float] = None
            right: Optional[float] = None
            try:
                left = float(read_ir(api, side="left", timeout=timeout))
            except Exception:
                left = None
            try:
                right = float(read_ir(api, side="right", timeout=timeout))
            except Exception:
                right = None

            # 每帧回调：给上层做 TUI / 日志
            if on_tick is not None:
                try:
                    on_tick(  # type: ignore[arg-type]
                        left if left is not None else float("nan"),
                        right if right is not None else float("nan"),
                    )
                except Exception:
                    pass

            # 阈值命中回调：两侧独立判断、独立触发
            if on_alert is not None:
                if left is not None and left < threshold_m:
                    try:
                        on_alert("left", left)
                    except Exception:
                        pass
                if right is not None and right < threshold_m:
                    try:
                        on_alert("right", right)
                    except Exception:
                        pass

            elapsed = time.monotonic() - t0
            sleep_s = dt - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)
    except KeyboardInterrupt:
        # Ctrl-C 直接退出，不重抛；调用方一般在外层还有 finally 兜底轮速。
        pass