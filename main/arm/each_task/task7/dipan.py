"""task7 / dipan —— **让底盘向前移动 60cm** (= 600mm)。

(底盘 = "dipan"; 单独抽出来供 task7 各阶段按需调, 例如:
  - task6 取完订单后进入配送位
  - 摆位前微调车体位置
  - 视觉闭环期间给动作让位置)

行为:
  - 走车端 ``car.move_for`` (相对位姿位移, [x,y,theta] 单位米)。
  - 默认 x=+0.600m (向前 60cm); y=0 / theta=0, 即纯直线前进, 不横移 / 不转向。
  - sync=True 阻塞等 job 完成 (move_for 是闭环位移, 必须等到位才能走下一步)。
  - 不走 ArmClient._call_car (默认 sync=False 异步), 显式调
    ``client.http.execute_car_action(..., sync=True)`` — 与 task5/dipan.py
    / task4/target4.py 同款模式。

⚠️ **方向约定 (从 task5/dipan.py 推断, 2026-08-03 与之对齐)**:
  - ``move_for`` 的 [x, y, theta] 第一项 = 车体本地 x 偏移 (m)。
  - 正值 = 车头方向前进, 负值 = 车头方向后退。
  - 用户口述 "向前 60cm" → ``[+0.600, 0.0, 0.0]``。
  - 若你看到的实际运动方向相反 (车往后窜), 先单独用 ``--dist -50`` 做冒烟
    反向验证, **不要靠改参数默认值埋雷**。

⚠️ **默认 timeout 给 15s**: 60cm 在 0.10 m/s 下约 6s, 加 PID 闭环余量 = 8s 起步。
  - 脚本会按 ``max(5.0, |dist_m|/max_vel + 2)`` 自适应放大。
  - 60cm 量级比 task5 单次 166mm / task4 单次 50mm 都大不少, 若 PID 闭环长可
    ``--timeout 30`` 兜底。
  - 超时 → job status=failed, 脚本直接 raise 让外层处理 (不静默吞)。

⚠️ **里程计飘移**: ``move_for`` 是闭环位移 (odometry 积分), 60cm 量级现场实测
  累积误差可能到 1-3cm (比小步位移放大)。若需要 mm 级精度, 改用 ``move_to_position``
  (绝对目标) + 视觉闭环, 或在调用方用 GET /v1/realtime/chassis/state 复核。

⚠️ **大位移分段建议**: 若底盘或地面有打滑 / 中途卡顿, 60cm 一次 move_for 比
  分段 (e.g. 6×100mm) 风险高 —— 一段出问题整段重来。现场先冒烟一次再决定是否
  拆多段调用。

⚠️ **本文件自包含** (与 task5/dipan.py / task5 其他脚本同款):
  只依赖 ``main.arm.ArmClient``, 不 import task7 包内任何模块。

跑法:
    python main/arm/each_task/task7/dipan.py                # 默认前进 60cm
    python -m main.arm.each_task.task7.dipan
    python main/arm/each_task/task7/dipan.py --dist 1000    # 前进 1000mm (1m)
    python main/arm/each_task/task7/dipan.py --dist -600    # 后退 60cm
    python main/arm/each_task/task7/dipan.py --vel 0.08     # 限速 0.08 m/s (更稳)
    python main/arm/each_task/task7/dipan.py --timeout 30  # 大位移给更宽超时
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

LOG_PREFIX: str = "[task7/dipan]"

DEFAULT_DIST_MM: float = 315
"""默认位移 (mm)。正值 = 向前 60cm (= 600mm, 用户 2026-08-03 要求);
负值 = 向后。对应 ``move_for`` 调用: x_m = dist_mm / 1000。"""

DEFAULT_TIMEOUT_S: float = 15.0
"""job 同步超时。60cm 在 0.10 m/s 下约 6s, 加 2s PID 闭环 = 8s 起步。
脚本会按 ``max(5.0, |dist_m|/max_vel + 2)`` 自适应放大, 这个值是用户显式
``--timeout`` 兜底时的安全上限。"""

DEFAULT_MAX_VELOCITY_MS: float = 0.10
"""默认最大线速度 (m/s)。10cm/s 对 60cm ≈ 6s 跑完,
对编码器积分友好 (不容易过冲); 现场要更快可 ``--vel 0.20``。"""


def _run(client: ArmClient, dist_mm: float,
         max_velocity_ms: float, timeout: float) -> dict:
    """下发一次 move_for, 同步等 job 完成。

    Args:
        client: ArmClient (取 .http 走车端 action)。
        dist_mm: 目标位移 (mm)。正值 = 前进, 负值 = 后退。
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
    # 停下时跑 (target4.py / task5/dipan.py 注释明示业务语义就是要等)。
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
        description="task7 dipan: 底盘直线位移 (默认前进 60cm)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--dist", type=float, default=DEFAULT_DIST_MM,
                   help="位移 (mm)。正值=前进, 负值=后退, 0=原地 (no-op)。")
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
