"""task7 / position1 —— **位置 1** 的位姿序列 (3 阶段: 后退 → 7 步臂 (含 1 次放气) → 前进)。

按用户 2026-08-04 v4 指定顺序 (与 position2.py v4 实际运行参数和顺序一致):

  Phase 1: 底盘后退 130mm (13cm)
            ↓
  Phase 2: 7 步动作 (与 position2.py v4 完全同款, 含 1 次放气 Step 2.5, 用户"自己写"指示内联不 import)
            2.0 move_y(-190mm)                   y 抬高完全出保护区
            2.1 set_arm_angle(+90°)              大臂归为 +90° (y_up 之后立刻归位)
            2.2 set_hand_angle(-40°)             手爪 -40° (mid mode, 跟 position2 同款)
            2.3 move_y(-135mm)                   y 降回工作深度
            2.4 move_x_with_split(-225mm)        x 滑到位置 1 (-225mm, 跟 position2 同款, push 货物到位)
            2.5 runner.drop_object()             🆕 v4 新增: 放气 (断开真空, 货物落到目标位)
            2.6 move_x_with_split(0mm)           x 回 0 位 (撞墙 calibrate, 原 Step 2.5 前移)
            ↓
  Phase 3: 底盘前进 130mm (13cm)

⚠️ **变更历史**:
  - **v1 (2026-08-03)**: 7 步原版 (含 suck() 2.0 + drop_object() 2.5)。
  - **v2 (2026-08-04 中午)**: 删掉 suck() 和 drop_object(), 原 7 步 → 5 步。
  - **v3 (2026-08-04 下午)**: 在 y_up 之后插入 set_arm_angle(+90°), 5 步 → 6 步。
    大臂 +90° 是业务硬限上界 [-150, +90] 内的合法值,跟 position4/5/6 同款,
    在 y=-190 (保护区外) 调用 _check_safe 不会拦截。
  - **v4 (2026-08-04 晚上)**: 在 Step 2.4 (x_to push) 和原 Step 2.5 (x_return) 之间加回
    drop_object() (放气), 6 步 → 7 步。序号后移 1 (2.5→2.6), return dict 加回
    ``drop_result`` (无 suck_result)。语义: push 货物到位后立即放气 → 真空断开 →
    货物落到目标位, 然后再 x 回 0 (撞墙 calibrate)。

⚠️ **底盘移动 (Phase 1/3)**:
  - 走车端 ``car.move_for`` (相对位姿位移, [x_m, 0, 0])。
  - 正值 = 车头方向前进, 负值 = 车头方向后退 (与 task7/dipan.py 同款)。
  - 本脚本 CLI 接收的 --back / --forward 始终是 **正 mm** (用户语义),
    内部 Phase 1 转负号 = 后退; Phase 3 保持正号 = 前进。
  - 130mm / 0.10 m/s ≈ 1.3s, 自适应 timeout = max(5.0, |dist|/vel + 2.0),
    用户 --timeout 兜底。
  - sync=True 阻塞等闭环完成 (后续臂动作必须等底盘停下, 否则位置错位)。
  - 不走 ArmClient._call_car (默认 sync=False 异步会抢跑)。
  - 失败 → job status=failed, 脚本直接 raise 让外层处理 (不静默吞)。

⚠️ **臂序列 (Phase 2) 与 position2.py v4 实际运行一致** (用户 2026-08-04 改):
  - y_up=-190, arm=+90°, hand=-40, y_down=-135, x_to=-225, x_return=0
  - 含 1 次 drop_object() (Step 2.5), 跟 position2 v4 7 步动作同款 (但**不加回 suck()**,
    本脚本前提: 货物已在手里)
  - 全部 ARM_API §7 软限位 / 保护区解释见 position2.py docstring
  - 手爪 UP 必须 ``client.set_hand_angle(angle, speed, timeout=...)``:
    ArmRunner 没有 set_hand_angle (只有 set_storage), 且 timeout 是必填位置参
    (与 set_arm_angle 不同)。见 [[armrunner-set-hand-angle-gotcha]]
  - 🆕 v4 加回 1 次 drop_object() (在 Step 2.5), 但**不加回 suck()** (本脚本前提:
    货物已在手里)。其他吸/放气动作仍由上层单独触发。

⚠️ **为什么不直接 import position2.py**:
  task7 包内模块被外部清空过一次 (见 [[task5-rebuild-2026-07-22]] 同款教训),
  本脚本自包含: 只依赖 ``main.arm`` + ``main.arm.each_task.common``,
  不 import task7 包内任何模块。Phase 2 的 7 步动作按用户"自己写"指示内联实现。

⚠️ **底盘位移精度**: ``move_for`` 是闭环位移 (odometry 积分), 130mm 量级现场
  实测累积误差可能到 1-3cm。若需要 mm 级精度, 改用 ``move_to_position`` (绝对
  目标) + 视觉闭环, 或在调用方用 GET /v1/realtime/chassis/state 复核。

跑法:
    python main/arm/each_task/task7/position1.py
    python -m main.arm.each_task.task7.position1
    python main/arm/each_task/task7/position1.py --back 200 --forward 200   # 改底盘距离
    python main/arm/each_task/task7/position1.py --vel 0.05                 # 更稳
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.each_task.common import move_x_with_split  # noqa: E402


# ---------- 默认参数 ----------

LOG_PREFIX: str = "[task7/position1]"

# ==== Phase 1 / 3: 底盘移动 (跟前 version 一致, 保持不变) ====
DEFAULT_BACK_MM: float = 130.0
"""Phase 1 后退距离 (mm)。用户 2026-08-03 指定 13cm = 130mm。
CLI 接收正值, 内部转负号 → move_for x_m = -dist_mm/1000 (后退)。
负值会被 ``abs()`` 强制取正再取负, 避免误传前进。"""

DEFAULT_FORWARD_MM: float = 130.0
"""Phase 3 前进距离 (mm)。用户 2026-08-03 指定 13cm = 130mm。
CLI 接收正值, 内部直接用 → move_for x_m = +dist_mm/1000 (前进)。"""

DEFAULT_CHASSIS_TIMEOUT_S: float = 10.0
"""底盘 move_for HTTP 同步超时兜底 (秒)。
130mm / 0.10 m/s ≈ 1.3s; 脚本按 max(5.0, |dist|/vel + 2.0) 自适应放大,
用户 ``--timeout`` 是这个自适应值的下限。"""

DEFAULT_CHASSIS_VELOCITY_MS: float = 0.10
"""底盘最大线速度 (m/s), 与 task7/dipan.py 默认一致。"""

# ==== Phase 2: 臂序列 (与 position2.py 实际运行参数 + 顺序一致, 2026-08-04 改) ====
POS_Y_UP_MM: float = -190.0
"""Phase 2.1: y 抬到 -190mm (完全出保护区 [0, -30], 给 set_hand_angle 留余地)。
⚠️ ≤ soft_y_max=-200 才不超业务硬限。"""

POS_HAND_DEG: float = -40.0
"""Phase 2.3: 手爪 -40° (mid mode, 与 position2.py 实际运行值一致, 2026-08-04 改)。
⚠️ 改名历史: 原 POS_HAND_UP_DEG=-45, 2026-08-04 跟 position2 统一改成 POS_HAND_DEG=-40。
⚠️ 必须走 ``client.set_hand_angle(angle, speed, timeout=...)`` (ArmRunner 没 set_hand_angle,
   timeout 必填位置参)。见 [[armrunner-set-hand-angle-gotcha]]"""

POS_ARM_DEG: float = 90.0
"""Phase 2.1: 大臂归为 +90° (用户 2026-08-04 要求 y_up 之后立刻归位)。

⚠️ +90° 是业务硬限上界 [-150, +90] 内的合法值,跟 position4/5/6 同款 (它们都有这一步)。
⚠️ 必须 y ≤ -80 (保护区外) 才能调 set_arm_angle(非 0/MID),所以本步在 y_up=-190 之后。
⚠️ 走 ``runner.set_arm_angle(angle, speed=...)`` (ArmRunner 有这个方法,默认 timeout 80s)。"""

POS_Y_DOWN_MM: float = -135.0
"""Phase 2.4: y 降回 -135mm (工作深度, 与 position2.py 实际值一致)。
⚠️ 与 task7/target.py setup (-80mm) **故意不同**: 位置 1/2/3 横向较远,
   货物位置比标准位高, y 抬高适配。"""

POS_X_TO_MM: float = -225.0
"""Phase 2.5: x 滑到位置 1 (-225mm, 与 position2.py 实际运行值一致, 2026-08-04 改)。
⚠️ 必须 ≥ x_min_m=-320 软限位; -225 距下界还有 95mm 余量。"""

POS_X_RETURN_MM: float = 0.0
"""Phase 2.6: x 回 0 位 (撞墙 calibrate, 重置编码器零点, 与 position2.py 同款)。"""

ANGLE_SPEED: int = 80
"""大臂 + 手爪舵机速度 (与 task7 其他脚本一致)。"""


# ---------- 底盘 move_for 内联 ----------

def _chassis_move_for(client: ArmClient, dist_mm: float,
                      max_velocity_ms: float, timeout: float,
                      log_prefix: str) -> dict:
    """底盘相对位姿位移 (move_for)。sync=True 阻塞等闭环完成。

    Args:
        client: ArmClient (.http.execute_car_action)
        dist_mm: 位移 mm; 正值 = 前进, 负值 = 后退 (move_for 自身符号约定)
        max_velocity_ms: 限速 m/s, 透传给 move_for.max_velocities
        timeout: HTTP 同步超时秒
        log_prefix: 打印前缀

    Returns:
        ``/v1/execute`` 同步返回的 job dict (status=succeeded 时)。

    Raises:
        RuntimeError: job status != succeeded (含 status/result/error 详情)。
    """
    dist_m = dist_mm / 1000.0
    direction = "向后" if dist_m < 0 else ("向前" if dist_m > 0 else "原地")
    print(f"  {log_prefix} {direction} {abs(dist_mm):.0f}mm  "
          f"(x_offset={dist_m:+.3f}m)  max_v={max_velocity_ms:.2f}m/s  "
          f"timeout={timeout:.1f}s")
    t0 = time.time()
    job = client.http.execute_car_action(
        "move_for",
        [dist_m, 0.0, 0.0],                       # [x, y, theta] 纯 x 直线
        max_velocities=[max_velocity_ms, max_velocity_ms, 0.0],
        sync=True,
        timeout=timeout,
    )
    dt = time.time() - t0

    ok = isinstance(job, dict) and job.get("status") == "succeeded"
    status = job.get("status") if isinstance(job, dict) else None
    result = job.get("result") if isinstance(job, dict) else None
    error = job.get("error") if isinstance(job, dict) else None
    print(f"  {log_prefix} 结果: status={status!r}  耗时={dt:.2f}s  "
          f"actual={result}  error={error}")
    if not ok:
        raise RuntimeError(
            f"{log_prefix} move_for 失败 (status={status!r}, "
            f"result={result!r}, error={error!r})"
        )
    return job


# ---------- 主流程 ----------

def run(client: ArmClient, runner: ArmRunner,
        back_mm: float = DEFAULT_BACK_MM,
        forward_mm: float = DEFAULT_FORWARD_MM,
        max_velocity_ms: float = DEFAULT_CHASSIS_VELOCITY_MS,
        timeout: float = DEFAULT_CHASSIS_TIMEOUT_S) -> dict:
    """3 阶段执行: 后退 → 7 步臂 (含 1 次放气 Step 2.5) → 前进 (与 position2.py v4 一致)。

    ⚠️ **2026-08-04 v4 改**: 在 Step 2.4 (x_to push) 和原 Step 2.5 (x_return) 之间加回
       drop_object() (放气), 6 步 → 7 步。语义: push 货物到位后立即放气 → 真空断开
       → 货物落到目标位, 然后再 x 回 0 (撞墙 calibrate)。
    ⚠️ **2026-08-04 v3 改 (历史)**: 在 y_up 之后插入 set_arm_angle(+90°), 5 步 → 6 步。
    ⚠️ **2026-08-04 v2 改 (历史)**: 删掉 suck() + drop_object(), 原 7 步 → 5 步;
       v4 又加回 drop_object() 一次, 但**不加回 suck()** (本脚本前提: 货物已在手里)。

    Args:
        client: ArmClient (move_x + set_hand_angle + http.execute_car_action)
        runner: ArmRunner (move_y + set_arm_angle + set_hand_angle 替代 set_hand + **drop_object (Step 2.5)**)
        back_mm: Phase 1 后退距离 (mm, 正值, 默认 130)
        forward_mm: Phase 3 前进距离 (mm, 正值, 默认 130)
        max_velocity_ms: 底盘限速 (m/s, 默认 0.10)
        timeout: 底盘 move_for HTTP 同步超时下限 (秒, 默认 10; 脚本自适应放大)

    Returns:
        {
            "ok": True,
            "back_mm": float,
            "forward_mm": float,
            "back_job": dict,              # Phase 1 job
            "forward_job": dict,           # Phase 3 job
            "y_up_mm": -190.0,
            "y_down_mm": -135.0,
            "hand_deg": -40.0,
            "x_to_mm": -225.0,
            "x_return_mm": 0.0,
            "x_to_result": dict,           # Phase 2.4 split 结果
            "drop_result": dict,           # 🆕 v4 Phase 2.5 放气 (y=-135 + x=-225)
            "x_return_result": dict,       # Phase 2.6 split 结果 (v4 由原 2.5 前移)
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (back → 7 步臂 (含 1 次放气) → forward) ==========")
    print(f"  Phase 1: 后退 {back_mm:.0f}mm")
    print(f"  Phase 2: 7 步动作 (与 position2.py v4 实际运行参数 + 顺序一致, 含 1 次放气)")
    print(f"  Phase 3: 前进 {forward_mm:.0f}mm")

    # ==== Phase 1: 底盘后退 (底盘, 不变) ====
    # 强制转负: 即便用户传了 --back -50 (误) 也变成 -130 (后退), 避免误前进撞墙。
    back_signed_mm = -abs(back_mm)
    back_timeout = max(timeout, 5.0, abs(back_signed_mm) / 1000.0 / max(max_velocity_ms, 0.01) + 2.0)
    print(f"\n  ── Phase 1: 底盘后退 {abs(back_signed_mm):.0f}mm ──")
    back_job = _chassis_move_for(
        client, back_signed_mm,
        max_velocity_ms, back_timeout,
        log_prefix=f"  {LOG_PREFIX} phase1",
    )

    # ==== Phase 2: 7 步动作 (与 position2.py v4 同款, 内联实现, 含 1 次放气 Step 2.5) ====
    print(f"\n  ── Phase 2: 7 步动作 (与 position2 v4 同款, 含 1 次放气) ──")

    # 2.0 y 抬高 (出保护区, 给 set_*_angle 和 move_x 留余地)
    print(f"  [2.0] move_y({POS_Y_UP_MM}mm)    y 抬高完全出保护区")
    runner.move_y(POS_Y_UP_MM, verify=True)

    # 2.1 大臂归为 +90° (y_up 之后立刻归位, 用户 2026-08-04 要求)
    # ⚠️ +90° 是业务硬限上界 [-150, +90] 内合法值, 在 y=-190 (保护区外) 调 _check_safe 不会拦截。
    # ⚠️ 跟 position4/5/6 同款 (它们都有这一步)。走 runner.set_arm_angle (ArmRunner 有这个方法)。
    print(f"  [2.1] set_arm_angle({POS_ARM_DEG}°)   大臂归为 +{POS_ARM_DEG:.0f}° (y=-190, 保护区外)")
    runner.set_arm_angle(POS_ARM_DEG, speed=ANGLE_SPEED)

    # 2.2 手爪 -40° (mid mode, 跟 position2 同款)
    # ⚠️ ArmRunner 没有 set_hand_angle (只有 set_storage), 必须走 client.set_hand_angle,
    #    且 timeout 是必填位置参 (与 set_arm_angle 不同)。见 [[armrunner-set-hand-angle-gotcha]]
    print(f"  [2.2] set_hand_angle({POS_HAND_DEG}°)   手爪到 mid mode")
    client.set_hand_angle(
        POS_HAND_DEG, speed=ANGLE_SPEED,
        timeout=runner.default_timeout_s,
    )

    # 2.3 y 降回工作深度 (手爪 OUT 后安全降)
    print(f"  [2.3] move_y({POS_Y_DOWN_MM}mm)    y 降回工作深度")
    runner.move_y(POS_Y_DOWN_MM, verify=True)

    # 2.4 x 滑到位置 1 (-225mm, 跟 position2 同款, push 货物到位)
    print(f"  [2.4] move_x_with_split({POS_X_TO_MM}mm)  x → 位置 1 (split 兜底)")
    x_to_result = move_x_with_split(
        client, runner, POS_X_TO_MM,
        log_prefix=f"  {LOG_PREFIX} phase2.4",
    )

    # 🆕 2.5 (v4 新增): 放气 (断开真空, 货物落到目标位)
    # ⚠️ 必须**在 x 推到位之后** (Step 2.4 后), **x 归零之前** (Step 2.6 前)。
    # runner.drop_object() 走 runner, 不走 client (跟 suck/drop_object 同款, 见
    # main/arm/loops/runner.py:185 drop_object)。
    print(f"  [2.5] drop_object()  🆕 v4 放气 (断开真空, 货物落目标位, y=-135 + x=-225)")
    drop_result = runner.drop_object()

    # 2.6 x 回 0 位 (撞墙 calibrate, v4 由原 Step 2.5 前移)
    print(f"  [2.6] move_x_with_split({POS_X_RETURN_MM}mm) x → 0 位 (split 撞墙 calibrate)")
    x_return_result = move_x_with_split(
        client, runner, POS_X_RETURN_MM,
        log_prefix=f"  {LOG_PREFIX} phase2.6",
    )

    # ==== Phase 3: 底盘前进 (底盘, 不变) ====
    # 强制转正: 即便用户传了 --forward -50 (误) 也变成 +130 (前进)。
    forward_signed_mm = abs(forward_mm)
    forward_timeout = max(timeout, 5.0, abs(forward_signed_mm) / 1000.0 / max(max_velocity_ms, 0.01) + 2.0)
    print(f"\n  ── Phase 3: 底盘前进 {forward_signed_mm:.0f}mm ──")
    forward_job = _chassis_move_for(
        client, forward_signed_mm,
        max_velocity_ms, forward_timeout,
        log_prefix=f"  {LOG_PREFIX} phase3",
    )

    dt = time.time() - t0
    print(f"========== {LOG_PREFIX} 完成 ({dt:.2f}s) ==========\n")

    return {
        "ok": True,
        "back_mm": back_mm,
        "forward_mm": forward_mm,
        "back_job": back_job,
        "forward_job": forward_job,
        "y_up_mm": POS_Y_UP_MM,
        "y_down_mm": POS_Y_DOWN_MM,
        "hand_deg": POS_HAND_DEG,
        "x_to_mm": POS_X_TO_MM,
        "x_return_mm": POS_X_RETURN_MM,
        "x_to_result": x_to_result,
        "drop_result": drop_result,                       # 🆕 v4 新增: Step 2.5 放气
        "x_return_result": x_return_result,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: --back / --forward / --vel / --timeout。

    --back / --forward 接收正 mm (用户语义), 内部自动转符号:
      - --back  → move_for x_m = -|back|/1000  (后退)
      - --forward → move_for x_m = +|forward|/1000  (前进)
    即便用户传负值也会被 abs() 强制取正, 避免误传撞墙。
    """
    p = argparse.ArgumentParser(
        description=(
            "task7 position1: 后退 13cm → 7 步臂 (含 1 次放气, 与 position2 v4 同款) → 前进 13cm"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--back", type=float, default=DEFAULT_BACK_MM,
                   help="Phase 1 后退距离 (mm, 默认 130 = 13cm, 强制正值)")
    p.add_argument("--forward", type=float, default=DEFAULT_FORWARD_MM,
                   help="Phase 3 前进距离 (mm, 默认 130 = 13cm, 强制正值)")
    p.add_argument("--vel", type=float, default=DEFAULT_CHASSIS_VELOCITY_MS,
                   dest="max_velocity",
                   help="底盘最大线速度 (m/s, 默认 0.10)")
    p.add_argument("--timeout", type=float, default=DEFAULT_CHASSIS_TIMEOUT_S,
                   help="底盘 move_for HTTP 同步超时下限 (秒, 默认 10)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner,
        back_mm=args.back,
        forward_mm=args.forward,
        max_velocity_ms=args.max_velocity,
        timeout=args.timeout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
