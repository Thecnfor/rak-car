#!/usr/bin/python3
"""task4 / open_storage —— 开仓专用 (单动作, 不动 y, 不做 gate)

职责单一: 只做 "开仓" 这一动作 (set_storage_angle 75°)。
调用方应自己保证 y 已到位。

2026-07-31 改造: 删掉之前 y gate 预检 + auto-move 那套逻辑
  - 之前: 读 realtime y, 不在 [-205, -145] 就 auto_move 抬到 -175
  - 现在: 不读 y, 不动 y, 直接 set_storage_angle 75°
  - 理由: 现场已确认 y 到位策略由调用方负责, open_storage 只管"舵机怎么转"
  - 调用方负责 (典型顺序):
      1) client.move_y(-175)        # 把 y 抬到开仓区间
      2) step_open_storage(client, runner)

用法:
  # 单动作入口
  step_open_storage(client, runner)

输出:
  - {"ok": True, "storage": "OPEN", "angle_deg": 75}
"""
from __future__ import annotations

import os
import sys

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402

try:
    # 包内跑: `python -m main.arm.each_task.task4.open_storage`
    from .constants import (  # noqa: E402
        STORAGE_OPEN_ANGLE_DEG,
        STORAGE_OPEN_SPEED,
        LOG_PREFIX_TASK4,
    )
except ImportError:  # pragma: no cover — 直接 `python open_storage.py` 时无包上下文
    from main.arm.each_task.task4.constants import (  # type: ignore
        STORAGE_OPEN_ANGLE_DEG,
        STORAGE_OPEN_SPEED,
        LOG_PREFIX_TASK4,
    )


def step_open_storage(
    client: ArmClient,
    runner: ArmRunner,
    *,
    angle_deg: int = STORAGE_OPEN_ANGLE_DEG,
    speed: int = STORAGE_OPEN_SPEED,
    timeout: float = 10.0,
) -> dict:
    """开仓 (set_storage_angle 75°) —— 纯舵机动作, 不碰 y。

    2026-07-31: 删掉 y gate 预检 + auto-move 那套 (旧版本会读 realtime y,
    不在 [-205, -145] 就自动抬到 -175)。现在调用方负责把 y 摆到位,
    open_storage 只管"舵机怎么转"。

    Args:
        client: ArmClient 实例 (保留传参以兼容旧调用方; 本函数未直接使用)
        runner: ArmRunner 实例 (用于 set_storage_angle)
        angle_deg: 开仓角度 (默认 75°, 与 STORAGE_OPEN_ANGLE_DEG 一致)
        speed: 舵机速度 (默认 5, ARM_API §6.1)
        timeout: HTTP 同步超时

    Returns:
        dict: {"ok": True, "storage": "OPEN", "angle_deg": 75}

    Raises:
        RuntimeError: set_storage_angle job failed 时抛出 (带车端 error)
    """
    print(f"{LOG_PREFIX_TASK4} [open_storage] 开始开仓 (angle={angle_deg}°)")

    # ---- 直传舵机: set_storage_angle 走 runner, 触发 ARMClient 自身 gate ----
    runner.set_storage_angle(angle_deg, speed=speed, timeout=timeout)

    print(
        f"{LOG_PREFIX_TASK4} [open_storage] ✅ 开仓完成 "
        f"(angle={angle_deg}°, speed={speed})"
    )
    return {
        "ok": True,
        "storage": "OPEN",
        "angle_deg": angle_deg,
    }


def main() -> None:
    """独立跑入口 —— 直接开仓。"""
    client = ArmClient.connect()
    runner = ArmRunner(client)
    step_open_storage(client, runner)


if __name__ == "__main__":
    main()