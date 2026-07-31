"""task5 / dipan —— **让底盘向后移动 166mm**。

(底盘 = "dipan"; 单独抽出来供 task5 各阶段按需调, 例如:
  - 取完果实从场地仓回退腾位
  - 摆位前微调车体位置
  - 视觉闭环期间给球让位置)

行为:
  - 走车端 ``car.move_for`` (相对位姿位移, [x,y,theta] 单位米)。
  - 默认 x=-0.166m (向后 166mm); y=0 / theta=0, 即纯直线后退, 不横移 / 不转向。
  - sync=True 阻塞等 job 完成 (move_for 是闭环位移, 必须等到位才能走下一步)。
  - 不走 ArmClient._call_car (默认 sync=False 异步), 显式调
    ``client.http.execute_car_action(..., sync=True)`` — 与 task4/target4.py
    前移 5cm 离散 step 同款模式。

⚠️ **方向约定 (从 task4/target4.py 推断, 2026-07-30)**:
  - ``move_for`` 的 [x, y, theta] 第一项 = 车体本地 x 偏移 (m)。
  - 正值 = 车头方向前进, 负值 = 车头方向后退 (即车尾向车头原本前进的反向移动)。
  - 用户口述 "向后 145mm" → ``[-0.145, 0.0, 0.0]``。
  - 若你看到的实际运动方向相反 (车向前窜), 先单独用 ``--dist 50`` 做冒烟,
    再决定要不要在脚本里翻转符号; **不要靠改参数默认值, 那是埋雷**。

⚠️ **默认 timeout 给 20s**: 145mm 在中等速度下大概 2-4s 到位, 留余量给 PID 闭环。
  - 超时 → job status=failed, 脚本直接 raise 让外层处理 (不静默吞)。

⚠️ **里程计飘移**: ``move_for`` 是闭环位移 (odometry 积分), 现场实测会有 cm 级
  累积误差。若需要 mm 级精度, 改用 ``move_to_position`` (绝对目标) + 视觉闭环,
  或在调用方用 GET /v1/realtime/chassis/state 复核实际位置。

⚠️ **本文件自包含** (与 task5 其他脚本同款, task5/__init__.py 提到的
  "曾被外部清空" 防御): 只依赖 ``main.arm.ArmClient``, 不 import task5
  包内任何模块。

跑法:
    python main/arm/each_task/task5/dipan.py                # 默认后退 166mm
    python -m main.arm.each_task.task5.dipan
    python main/arm/each_task/task5/dipan.py --dist 200     # 后退 200mm
    python main/arm/each_task/task5/dipan.py --dist -100    # 前进 100mm
    python main/arm/each_task/task5/dipan.py --vel 0.10     # 限速 0.10 m/s
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

LOG_PREFIX: str = "[task5/dipan]"

DEFAULT_DIST_MM: float = -166.0
"""默认位移 (mm)。负值 = 向后 (用户 2026-07-30 要求); 正值 = 向前。
对应 ``move_for`` 调用: x_m = dist_mm / 1000。"""

DEFAULT_TIMEOUT_S: float = 20.0
"""job 同步超时。166mm 在 0.10 m/s 下约 1.66s, 留余量给 PID 闭环。
脚本会按 ``max(5.0, abs(dist_m)/max_vel + 2)`` 自适应放大。"""

DEFAULT_MAX_VELOCITY_MS: float = 0.10
"""默认最大线速度 (m/s)。10cm/s 在直线 14.5cm 上 ≈ 1.5s 跑完,
对编码器积分友好 (不容易过冲); 现场要更快可 ``--vel 0.20``。"""


def _run(client: ArmClient, dist_mm: float,
         max_velocity_ms: float, timeout: float) -> dict:
    """下发一次 move_for, 同步等 job 完成。

    Args:
        client: ArmClient (取 .http 走车端 action)。
        dist_mm: 目标位移 (mm)。负值 = 后退。
        max_velocity_ms: 限速 (m/s), 透传给 move_for.max_velocities。
        timeout: HTTP 同步超时 (秒)。

    Returns:
        ``/v1/execute`` 同步返回的 job dict (含 status/result/error)。

    Raises:
        RuntimeError: job status != succeeded (含 status/result 详情)。
    """
    dist_m = dist_mm / 1000.0
    direction = "向后" if dist_m < 0 else ("向前" if dist_m > 0 else "原地")
    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  目标: {direction} {abs(dist_mm):.0f}mm  "
          f"(x_offset={dist_m:+.3f}m)  max_v={max_velocity_ms:.2f}m/s  "
          f"timeout={timeout:.1f}s")

    t0 = time.time()
    # ⚠️ sync=True 阻塞等闭环完成; 默认 False 异步会让下一步在底盘还没
    # 停下时跑 (target4.py 注释明示 "车端走完自动停", 业务语义就是要等)。
    job = client.http.execute_car_action(
        "move_for",
        [dist_m, 0.0, 0.0],          # [x, y, theta] —— 纯 x 直线, 不横移 / 不转向
        max_velocities=[max_velocity_ms, max_velocity_ms, 0.0],  # xy 限速, theta 留 0
        sync=True,
        timeout=timeout,
    )
    dt = time.time() - t0

    ok = isinstance(job, dict) and job.get("status") == "succeeded"
    status = job.get("status") if isinstance(job, dict) else None
    result = job.get("result") if isinstance(job, dict) else None
    error = job.get("error") if isinstance(job, dict) else None

    print(f"  结果: status={status!r}  耗时={dt:.2f}s  "
          f"actual={result}  error={error}")

    if not ok:
        raise RuntimeError(
            f"{LOG_PREFIX} move_for 失败 (status={status!r}, "
            f"result={result!r}, error={error!r})"
        )

    print(f"========== {LOG_PREFIX} 完成 ({direction} {abs(dist_mm):.0f}mm, {dt:.2f}s) ==========\n")
    return job


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task5 dipan: 底盘直线位移 (默认后退 166mm)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--dist", type=float, default=DEFAULT_DIST_MM,
                   help="位移 (mm)。负值=后退, 正值=前进, 0=原地 (no-op)。")
    p.add_argument("--vel", type=float, default=DEFAULT_MAX_VELOCITY_MS,
                   dest="max_velocity",
                   help="最大线速度 (m/s), 透传给 move_for.max_velocities")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                   help="HTTP 同步超时 (秒)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    # 自适应 timeout: |dist| / vel + 2s 兜底, 最少 5s
    adaptive_timeout = max(5.0, abs(args.dist) / 1000.0 / max(args.max_velocity, 0.01) + 2.0)
    timeout = args.timeout if args.timeout != DEFAULT_TIMEOUT_S else adaptive_timeout
    _run(client, dist_mm=args.dist,
         max_velocity_ms=args.max_velocity, timeout=timeout)
    return 0


if __name__ == "__main__":
    sys.exit(main())