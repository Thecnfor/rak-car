"""task7 / aaashouzhua —— **让手爪末端角度设为 -10°**。

(命名约定: ``aaa`` 前缀 = 任务起点预备动作, 与 ``the_final.py`` 的 ``the``
前缀命名类比; ``pingcang`` 是储存仓预备 (舵机), ``aaashouzhua`` 是手爪末端
预备 (舵机))

行为:
  - 走 ``ArmClient.set_hand_angle(angle, speed, timeout)`` —— **业务层**入口,
    内部自动校验硬限 + y 保护区 (不必手写 ValueError 检查)。
  - 默认 angle=**-10°** (用户 2026-08-06 指定); speed=80 默认; timeout=10 默认。
  - 同步阻塞等 job 完成。

⚠️ **set_hand_angle 业务硬限** (见 ``ARM_API.md`` §1.1):
  - 合法区间 ``angle ∈ [-90, 0]``。
  - ``angle > 0`` 或 ``angle < -90`` → 抛 ``ValueError``。
  - **-10° 在合法区间内** ✓。

⚠️ **y 保护区** (见 ``ARM_API.md`` §7.1):
  - 保护区 ``y ∈ [0, -80] mm``: 手爪摆动会撞车, 除 ``UP`` (-90) 外都拦截。
  - **set_hand_angle 非 -90 (UP) 会校验 y**: 若当前 y ∈ [0, -80] → 抛异常。
  - 跑前必须确认 y 不在保护区 (``y ≤ -80``)。
  - **建议跑前位姿**: y=-150 (典型工作位) + 大臂任意 + 手爪 UP 起手 → 再调 -10°。

⚠️ **不要绕开业务层直调**:
  - ``set_hand_angle`` 走业务层, 业务层会校验硬限 + y 保护区。
  - 与 ``pingcang.py`` 不同 (它绕开 ``ArmClient.set_storage_angle`` 是因为
    MEMORY 已知 ``api.py:720-729`` 缺 ``job =`` 赋值, 调一次崩 NameError)。
  - 本文件用业务层 (无已知 bug), 不要 ``client.http.execute_arm_action(...)`` 直调。

⚠️ **跑比赛前必须现场标定** (与 ``pingcang.py`` 同款):
  - -10° 是用户**当前调试**值, **不一定是比赛最终角度**。
  - 现场扫协议值, 找到"合适抓取姿态"对应的 raw 协议值后写业务脚本;
  - **不要假设** 旧角度常量还有效, 舵机机械结构会随校准变化。

⚠️ **本文件自包含** (与 ``task7/{pingcang, dipan, target, position*}.py`` 同款):
  只依赖 ``main.arm.ArmClient``, 不 import task7 包内任何模块。
  原因: task5 包曾被外部清空过一次 (见 ``[[task5-rebuild-2026-07-22]]``),
  自包含可保证 ``python aaashouzhua.py`` 直接跑不受影响。

跑法:
    python main/arm/each_task/task7/aaashouzhua              # 默认 angle=-10°
    python -m main.arm.each_task.task7.aaashouzhua
    python main/arm/each_task/task7/aaashouzhua.py --angle -45  # 改成 -45°
    python main/arm/each_task/task7/aaashouzhua.py --speed 50    # 慢一点
    python main/arm/each_task/task7/aaashouzhua.py --timeout 5   # 短超时
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

LOG_PREFIX: str = "[task7/aaashouzhua]"

DEFAULT_ANGLE_DEG: float = 10
"""默认手爪末端角度 (度)。用户 2026-08-06 指定 -10°;
合法区间 [-90, 0] (业务硬限, 超出抛 ValueError)。"""

DEFAULT_SPEED: int = 80
"""舵机速度 (1-100), 默认 80 = ARM_API §1.1 示例同款。"""

DEFAULT_TIMEOUT_S: float = 10.0
"""job 同步超时。舵机到位一般 < 2s, 给 10s 兜底 (含网络 + job_queue 等待)。"""


def _run(client: ArmClient, angle_deg: float, speed: int, timeout: float) -> dict:
    """下发一次 set_hand_angle, 同步等 job 完成。

    Args:
        client: ArmClient 实例 (业务层入口)。
        angle_deg: 目标角度 (度)。合法区间 [-90, 0] (业务硬限)。
        speed: 舵机速度 (1-100)。
        timeout: HTTP 同步超时 (秒)。

    Returns:
        ``ArmClient.set_hand_angle`` 返回值 (job dict, 含 status/result/error)。

    Raises:
        ValueError: angle_deg 超出 [-90, 0] 业务硬限, 或当前 y ∈ [0, -80] 保护区
                    (手爪摆动会撞车, 业务层自动拦截)。
        RuntimeError: job status != succeeded (set_hand_angle 内部不抛, 这里捕获重抛)。
    """
    # 业务层校验提示: 角度合法区间
    is_up = (angle_deg == -90.0)
    note = "(UP, 保护区允许)" if is_up else "(保护区外, y 必须 ≤ -80)"

    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  目标: 手爪末端 → {angle_deg:+.0f}°  speed={speed}  "
          f"timeout={timeout:.1f}s  {note}")

    t0 = time.time()
    # 走业务层 ArmClient.set_hand_angle:
    #   - 内部 _validate_hand_angle_client 校验 [-90, 0] 硬限
    #   - 内部 _check_y_protected 校验 y 不在保护区 (除非 -90 UP)
    #   - 内部 _call_arm 走 /v1/execute 同步 job
    job = client.set_hand_angle(
        angle=angle_deg,
        speed=speed,
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
            f"{LOG_PREFIX} set_hand_angle 失败 (status={status!r}, "
            f"result={result!r}, error={error!r})"
        )

    print(f"========== {LOG_PREFIX} 完成 (手爪末端 → {angle_deg:+.0f}°, {dt:.2f}s) ==========\n")
    return job


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task7 aaashouzhua: 手爪末端角度 (默认 -10°, 合法 [-90, 0])",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--angle", type=float, default=DEFAULT_ANGLE_DEG,
                   help="手爪末端目标角度 (度, 合法区间 [-90, 0])")
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