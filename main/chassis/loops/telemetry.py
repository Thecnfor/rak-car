"""main/chassis/loops/telemetry.py
外环的每帧调试输出。打印逻辑与控制律解耦，没有控制律 / 外环的代码不能跑。

现状：现场主要靠 `CurvatureAdaptiveOuterLoop.debug_snapshot()` 暴露内部量。
P / Stanley / PurePursuit 没这个接口，``lane_trace`` 退化打通用字段。
"""
from __future__ import annotations

from typing import Callable, List, Optional

from ..state import LaneState


def lane_trace(
    outer=None,
    every_n: int = 1,
) -> Callable[[LaneState, List[float]], None]:
    """给 ``DoubleLoopRunner(on_tick=...)`` 用的回调。

    每 ``every_n`` 帧打一行：lane 误差 + 4 轮线速度 + 控制律 debug_snapshot()。
    控制器没有 debug_snapshot() 时只打通用字段，打印宽度会缩短。
    """
    counter = {"n": 0}
    has_dbg = hasattr(outer, "debug_snapshot")

    def _on_tick(state: LaneState, wheels: List[float]) -> None:
        counter["n"] += 1
        if every_n > 1 and counter["n"] % every_n != 0:
            return
        v1, v2, v3, v4 = (wheels + [0.0] * 4)[:4]
        if has_dbg:
            try:
                dbg = outer.debug_snapshot()
            except Exception:
                dbg = {}
            print(
                f"ey={state.error_y!s:>10}  ea={state.error_angle!s:>10}  "
                f"kappa={dbg.get('kappa_ema', 0.0):.3f}  "
                f"dkappa={dbg.get('dkappa_ema', 0.0):.3f}  "
                f"axis_mix={dbg.get('axis_mix', 0.0):.3f}  "
                f"vy_keep={dbg.get('vy_keep', 0.0):.3f}  "
                f"ey_int={dbg.get('ey_int', 0.0):+.4f}  "
                f"streak={dbg.get('straight_streak_ms', 0.0):>5.0f}ms  "
                f"v1={v1:>8.4f}  v2={v2:>8.4f}  v3={v3:>8.4f}  v4={v4:>8.4f}"
            )
        else:
            print(
                f"ey={state.error_y!s:>10}  ea={state.error_angle!s:>10}  "
                f"v1={v1:>8.4f}  v2={v2:>8.4f}  v3={v3:>8.4f}  v4={v4:>8.4f}"
            )

    return _on_tick
