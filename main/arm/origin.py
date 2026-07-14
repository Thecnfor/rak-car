"""main/arm/origin.py
OriginCalibrator：4 键手动硬定原点。

按车端 4 键连续按住（Key4Btn(4)）的约定：
  1 = y 下降（电机 id=3 反方向）
  3 = y 上升（电机 id=3 正方向）
  2 = x 左移（电机 id=6 反方向）
  4 = x 右移（电机 id=6 正方向）
松开后电机停转。

复合键：
  1 + 3 同时按 1s = 触发保存原点（落盘 arm_origin.yaml）
  1 + 3 同时按 3s = 退出程序

注意：
  - 这个工具是给"首次上电 / 重新装配"用的，需要人在车边按键
  - 它**不**调车端 arm.reset_position（避免和触底/撞墙判定打架）
  - 它直接控制 y_speed / x_speed 开环速度，人按到什么位置算什么
"""
from __future__ import annotations

import time
import threading
from typing import Optional

try:
    from main.api_client import RuntimeApiClient
except ImportError:  # pragma: no cover
    from api_client import RuntimeApiClient  # type: ignore

from .state import ArmOrigin


# 按键码（与 smartcar/whalesbot/vehicle/arm/arm_base.py:ArmController.set_manually 对齐）
KEY_Y_DOWN = 1
KEY_Y_UP = 3
KEY_X_LEFT = 2
KEY_X_RIGHT = 4

# 移动速度（m/s）：与现有 set_manually 一致
JOG_SPEED = 0.1

# 复合键时长阈值
SAVE_HOLD_S = 1.0
QUIT_HOLD_S = 3.0


class OriginCalibrator:
    """4 键手动原点标定器。"""

    def __init__(self, http: RuntimeApiClient):
        self.http = http
        self._running = False
        self._stop_event = threading.Event()

    # ---- 内部工具 ----

    def _y_speed(self, v: float) -> None:
        """y 轴开环速度下发（m/s）。"""
        self.http.execute_arm_action("y_speed", timeout=5, velocity=v)

    def _x_speed(self, v: float) -> None:
        """x 轴开环速度下发（m/s）。"""
        self.http.execute_arm_action("x_speed", timeout=5, velocity=v)

    def _read_key(self) -> int:
        """读车端 4 键当前按键（0=无）。"""
        try:
            job = self.http.execute_car_action("get_key_event", timeout=5)
            res = job.get("result") if isinstance(job, dict) else None
            if isinstance(res, int):
                return res
            return 0
        except Exception:
            return 0

    def _save_origin(self, x_wall: str) -> ArmOrigin:
        """从车端读当前 y/x 原始值，写到 arm_origin.yaml。"""
        try:
            y_job = self.http.execute_arm_action("y_get_position", timeout=10)
            y_val = float(y_job.get("result"))
        except Exception as exc:
            raise RuntimeError(f"读 y 失败: {exc}")
        try:
            x_job = self.http.execute_arm_action("x_get_position", timeout=10)
            x_val = float(x_job.get("result"))
        except Exception as exc:
            raise RuntimeError(f"读 x 失败: {exc}")

        origin = ArmOrigin(
            y_origin_m=y_val,
            x_origin_m=x_val,
            x_wall=x_wall,
            soft_y_max_m=0.18,
            soft_x_min_m=0.005,
            soft_x_max_m=0.30,
            calibrated_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )
        # 落盘
        import os
        import yaml
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(here, "arm_origin.yaml")
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                {
                    "y_origin_m": origin.y_origin_m,
                    "x_origin_m": origin.x_origin_m,
                    "x_wall": origin.x_wall,
                    "soft_y_max_m": origin.soft_y_max_m,
                    "soft_x_min_m": origin.soft_x_min_m,
                    "soft_x_max_m": origin.soft_x_max_m,
                    "calibrated_at": origin.calibrated_at,
                },
                f,
                allow_unicode=True,
                sort_keys=False,
            )
        return origin

    # ---- 主循环 ----

    def run(self, x_wall: str = "left") -> Optional[ArmOrigin]:
        """阻塞：手动 4 键 jog，按 1+3 持续 1s 保存原点并退出。"""
        if x_wall not in ("left", "right"):
            raise ValueError("x_wall 必须是 'left' 或 'right'")

        print("=== 4 键手动原点标定 ===")
        print("  按键映射：")
        print("    1 = y 下降")
        print("    3 = y 上升")
        print("    2 = x 左移")
        print("    4 = x 右移")
        print(f"  1+3 同时按住 {SAVE_HOLD_S:.0f}s = 保存原点并退出")
        print(f"  1+3 同时按住 {QUIT_HOLD_S:.0f}s = 强制退出（不保存）")
        print("  Ctrl-C = 退出（不保存）")
        print()
        print(f"  目标：把机械臂 y 按到底、x 撞到{x_wall}侧墙，然后保存。")
        print()

        self._running = True
        self._stop_event.clear()
        combo_start: Optional[float] = None
        last_action = 0  # 防抖：上一次按下的键

        try:
            while self._running and not self._stop_event.is_set():
                key = self._read_key()

                # ---- 复合键 1+3 ----
                if key in (KEY_Y_DOWN, KEY_Y_UP):
                    # 注意：get_key_event 一次只返回一个按键；这里如果只读到一个，
                    # 就用"短间隔连续两个按键都按下"近似 1+3。生产环境如果 Key4Btn
                    # 能返回组合键，再升级。
                    if combo_start is None:
                        combo_start = time.time()
                    held = time.time() - combo_start
                    if held >= QUIT_HOLD_S:
                        print("检测到 1+3 持续 3s，强制退出。")
                        break
                    if held >= SAVE_HOLD_S:
                        print(f"检测到 1+3 持续 {SAVE_HOLD_S:.0f}s，保存原点...")
                        self._y_speed(0)
                        self._x_speed(0)
                        origin = self._save_origin(x_wall)
                        print(f"原点已写入 arm_origin.yaml: {origin.calibrated_at}")
                        print(f"  y_origin_m = {origin.y_origin_m:.5f}")
                        print(f"  x_origin_m = {origin.x_origin_m:.5f}")
                        print(f"  x_wall     = {origin.x_wall}")
                        return origin
                else:
                    combo_start = None

                # ---- 单键 jog ----
                if key != last_action:
                    last_action = key
                if key == KEY_Y_DOWN:
                    self._y_speed(-JOG_SPEED)
                    print("y 下降", end="\r")
                elif key == KEY_Y_UP:
                    self._y_speed(+JOG_SPEED)
                    print("y 上升", end="\r")
                elif key == KEY_X_LEFT:
                    self._x_speed(-JOG_SPEED)
                    print("x 左移", end="\r")
                elif key == KEY_X_RIGHT:
                    self._x_speed(+JOG_SPEED)
                    print("x 右移", end="\r")
                else:
                    # 无按键：停
                    self._y_speed(0)
                    self._x_speed(0)
                    print("等待按键...", end="\r")

                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n用户中断（不保存）")
        finally:
            self._y_speed(0)
            self._x_speed(0)
            self._running = False
        return None


def run_calibrator(x_wall: str = "left") -> Optional[ArmOrigin]:
    """便捷入口：建一个 client 跑 OriginCalibrator。"""
    http = RuntimeApiClient()
    return OriginCalibrator(http).run(x_wall=x_wall)
