"""main/chassis/tasks/back_to_line.py
"丢线恢复"：外环感知 lane_state 长时间归零时，退一步直行。
这里只暴露一个高层函数，底盘同学可以直接调。
"""
import time


def back_to_line(api, *, straight_seconds: float = 0.6, vx: float = 0.2) -> None:
    """不依赖外环：直接零号位 + 直走 straight_seconds，自动 zero out 收尾。"""
    try:
        # 麦轮"零号位"用同一套 4 轮速度设定：
        api.set_wheel_speeds([vx, -vx, -vx, vx])  # 直走（按 inverse 一致性自行核对）
        time.sleep(straight_seconds)
    finally:
        api.stop_wheel_speeds()
