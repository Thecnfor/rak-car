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

    debug_snapshot() 带 ``"type"`` 字段时按类型切换模板：
      * ``"orthogonal"`` → 十字正交：两通道 P/I + 死区 + vx/vy/ω
      * 其他或无 type → 兼容 CurvatureAdaptiveOuterLoop（老格式）
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
            if dbg.get("type") == "orthogonal":
                # 十字正交专用：突出 "两通道独立" 的调试量
                vy_p = dbg.get("vy_p_term", 0.0)
                vy_i = dbg.get("vy_i_term", 0.0)
                vy   = dbg.get("vy", 0.0)
                ey_dz = dbg.get("vy_dz", 0.0)
                o_p  = dbg.get("omega_p_term", 0.0)
                o_i  = dbg.get("omega_i_term", 0.0)
                om   = dbg.get("omega", 0.0)
                ea_dz = dbg.get("omega_dz", 0.0)
                vx   = dbg.get("vx", 0.0)
                lock = "LOCK" if dbg.get("locked_vx", False) else "CRUS"
                print(
                    f"[{lock}] ey={state.error_y!s:>10} ea={state.error_angle!s:>10}  "
                    f"vx={vx:+.3f}  "
                    f"vy(P/I/dz)= {vy:+.4f} = {vy_p:+.4f} + {vy_i:+.4f}  (dz={ey_dz:+.5f})  |  "
                    f"ω(P/I/dz) = {om:+.4f} = {o_p:+.4f} + {o_i:+.4f}  (dz={ea_dz:+.5f})  |  "
                    f"v1={v1:>8.4f} v2={v2:>8.4f} v3={v3:>8.4f} v4={v4:>8.4f}"
                )
            else:
                # 老模板：curvature_adaptive / stanley / P 的调试量
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
