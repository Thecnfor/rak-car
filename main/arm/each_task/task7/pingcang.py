"""task7 / pingcang —— **让储存仓角度转到 90°** (raw 协议值直传)。

(pingcang = "拼仓"/"平仓" 的拼音, task7 调仓用; 单独抽出来供 6 个位置按需调)

行为:
  - 走车端 ``car.set_storage_angle(angle, speed=100)``, **target=car 不是 arm**。
  - 默认 angle=**+90°** (用户 2026-08-03 指定); speed=100 默认。
  - sync=True 阻塞等 job 完成 (舵机到位后才算完)。
  - 不走 ArmClient.set_storage_angle (其内部也是 ``_call_car`` 但默认 sync=False 异步),
    显式调 ``client.http.execute_car_action(..., sync=True)`` —— 与 task7/dipan.py
    同款模式。

⚠️ **为什么用 set_storage_angle 而不是 set_storage** (见 ARM_API §6):
  - ``set_storage(side)`` 写死 LEFT=-42° / RIGHT=165° 两档, 不能传中间值。
  - ``set_storage_angle(angle, speed=100)`` 绕开两档, 直传 raw 协议值, 支持任意角。
  - 90° 不在 LEFT/RIGHT 两档里, 所以**必须**走 ``set_storage_angle``。

⚠️ **execute_car_action 调用 pattern 不能照搬 dipan.py** (2026-08-03 现场踩坑):
  - dipan.py 走 ``execute_car_action("move_for", [dist_m, 0.0, 0.0], ...)`` —— 看似
    通用, 其实**只对 move_for 有效**, 因为 ``move_for(position_offset, ...)`` 是
    **单参 list** (`[x, y, theta]`) 签名, 包成 list 才对。
  - ``set_storage_angle(angle, speed=100)`` 是**两个独立位置参**, 不能 wrap 成 list。
    如果写成 ``execute_car_action("set_storage_angle", [angle, speed], ...)``, runtime
    会调成 ``car.set_storage_angle([angle, speed])`` → ``angle`` 收到 list,
    ``speed`` 默认 100, 末端 ``int(angle)`` 报 "int() argument must be ... not 'list'"。
  - **正确写法**: angle 走位置参, speed 走 kwarg (与 ``ArmClient.set_storage_angle``
    内部 ``self._call_car("set_storage_angle", ..., angle=angle, speed=speed, sync=True)``
    一致)。

⚠️ **协议层 signed byte 范围** (ARM_API §6.2):
  - mc602 servo_pwm angle 字节是 signed byte, 合法区间 ``angle ∈ [-128, 127]``。
  - **90° 在合法区间内** ✓ (90 < 127)。
  - 超出范围 → 抛 ``struct.error`` (SDK 内部 ``bbBb`` 格式解包失败)。

⚠️ **无业务软限制** (ARM_API §6.2):
  - 2026-07-17 取消了 y 安全门; ``set_storage_angle`` 任意 y 位置都直传。
  - **物理碰撞由 caller 自负**: 跑前必须确认当前 y / x / arm / hand 位姿
    不会让大臂/手爪在舵机转动期间撞到车上结构。
  - 推荐跑前: y 在保护区 [0, -80] 外 (y ≤ -80), 大臂复位位 +90°, 手爪 UP -90°。

⚠️ **get_storage() 调完返回 "UNKNOWN"** (ARM_API §6.2):
  - 任意角度不属于 LEFT/RIGHT 两档, 客户端缓存逻辑识别不了。
  - 这是 **预期行为**, 不是 bug; 重建 client 后 cache 自然清空。

⚠️ **跑比赛前必须现场标定** (ARM_API §6.2):
  - 90° 是用户**当前调试**的值, **不一定是比赛最终角度**。
  - 现场扫协议值, 找到"开仓最大开角"的 raw 协议值后写业务脚本;
  - **不要假设** 旧角度常量还有效, 舵机机械结构会随校准变化。

⚠️ **本文件自包含** (与 task7/{dipan,target,position*}.py 同款):
  只依赖 ``main.arm.ArmClient``, 不 import task7 包内任何模块。
  原因: task5 包曾被外部清空过一次 (见 [[task5-rebuild-2026-07-22]]),
  自包含可保证 ``python pingcang.py`` 直接跑不受影响。

跑法:
    python main/arm/each_task/task7/pingcang.py              # 默认 angle=90°
    python -m main.arm.each_task.task7.pingcang
    python main/arm/each_task/task7/pingcang.py --angle 60   # 调成 60°
    python main/arm/each_task/task7/pingcang.py --angle -42  # 等效 LEFT 档
    python main/arm/each_task/task7/pingcang.py --speed 50   # 舵机慢一点更稳
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient  # noqa: E402


# ---------- 默认参数 ----------

LOG_PREFIX: str = "[task7/pingcang]"

DEFAULT_ANGLE_DEG: float = 90.0
"""默认储存仓角度 (raw 协议值, 单位度)。用户 2026-08-03 指定 +90°;
合法区间 [-128, 127] (mc602 servo_pwm signed byte)。"""

DEFAULT_SPEED: int = 100
"""舵机速度 (1-100), 默认 100 = 全速。慢一点更稳可 ``--speed 50``。"""

DEFAULT_TIMEOUT_S: float = 10.0
"""job 同步超时。舵机到位一般 < 2s, 给 10s 兜底 (含网络 + job_queue 等待)。"""


def _run(client: ArmClient, angle_deg: float, speed: int, timeout: float) -> dict:
    """下发一次 set_storage_angle, 同步等 job 完成。

    Args:
        client: ArmClient (取 .http 走车端 action)。
        angle_deg: 目标角度 (raw 协议值, 度)。合法区间 [-128, 127]。
        speed: 舵机速度 (1-100)。
        timeout: HTTP 同步超时 (秒)。

    Returns:
        ``/v1/execute`` 同步返回的 job dict (含 status/result/error)。

    Raises:
        ValueError: angle_deg 超出 signed byte 合法区间 [-128, 127]。
        RuntimeError: job status != succeeded (含 status/result 详情)。
    """
    # 协议层校验: signed byte 合法区间 [-128, 127]
    if not (-128 <= angle_deg <= 127):
        raise ValueError(
            f"{LOG_PREFIX} angle={angle_deg}° 超出 mc602 servo_pwm signed byte "
            f"合法区间 [-128, 127], 会触发 struct.error。"
        )

    # 业务提示: 角度不属于 LEFT/RIGHT 两档
    is_standard = angle_deg in (-42.0, 165.0)
    note = "(非 LEFT/RIGHT 两档, 调完 get_storage() 返回 UNKNOWN)" if not is_standard else "(标准档)"

    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  目标: 储存仓 → {angle_deg:+.0f}°  speed={speed}  timeout={timeout:.1f}s  {note}")

    t0 = time.time()
    # ⚠️ sync=True 阻塞等 job 完成; 默认 False 异步会让下一步在舵机还没
    # 到位时跑 (舵机到位是机械动作, 必须等)。
    # ⚠️ target="car" 不是 "arm"! set_storage_angle 走的是 car.set_storage_angle,
    # 见 ARM_API §3.2 / §6.2。业务层调 ArmClient.set_storage_angle 内部也走 car。
    # ⚠️ angle 走位置参, speed 走 kwarg! 不能 wrap 成 [angle, speed] (那样 runtime
    # 会调成 car.set_storage_angle([angle, speed]) → int(angle) 报 list 错误,
    # 见 2026-08-03 现场踩坑 + ARM_API §3.2 签名 set_storage_angle(angle, speed=100)。
    job = client.http.execute_car_action(
        "set_storage_angle",
        angle_deg,                   # 位置参 (单值, 不能 wrap 成 list!)
        speed=speed,                 # 关键字参 (避免与 move_for 单参 list 模式混淆)
        sync=True,
        timeout=timeout,
    )
    dt = time.time() - t0

    ok = isinstance(job, dict) and job.get("status") == "succeeded"
    status = job.get("status") if isinstance(job, dict) else None
    result = job.get("result") if isinstance(job, dict) else None
    error = job.get("error") if isinstance(job, dict) else None

    print(f"  结果: status={status!r}  耗时={dt:.2f}s  "
          f"result={result}  error={error}")

    if not ok:
        raise RuntimeError(
            f"{LOG_PREFIX} set_storage_angle 失败 (status={status!r}, "
            f"result={result!r}, error={error!r})"
        )

    print(f"========== {LOG_PREFIX} 完成 (储存仓 → {angle_deg:+.0f}°, {dt:.2f}s) ==========\n")
    return job


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task7 pingcang: 储存仓角度 (默认 +90°, raw 协议值直传)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--angle", type=float, default=DEFAULT_ANGLE_DEG,
                   help="储存仓目标角度 (度, 合法区间 [-128, 127])")
    p.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                   help="舵机速度 (1-100)")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                   help="HTTP 同步超时 (秒)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    _run(client, angle_deg=args.angle, speed=args.speed, timeout=args.timeout)
    return 0


if __name__ == "__main__":
    sys.exit(main())