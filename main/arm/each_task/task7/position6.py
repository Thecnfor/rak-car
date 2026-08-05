"""task7 / position6 —— **位置 6** 的位姿序列 (3 阶段: 前进 → 10 步动作 (含 1 次放气) → 后退)。

按用户 2026-08-04 v3 指定顺序 (与 position5.py v3 实际运行参数和顺序一致):

  Phase 1: 底盘前进 130mm (13cm)
            ↓
  Phase 2: 10 步动作 (与 position5.py v3 完全同款, **含 1 次放气** Step 2.06, 用户"自己写"指示内联不 import)
            2.00 move_y(-190mm)                  y 抬高完全出保护区
            2.01 set_arm_angle(+90°)              大臂到 90° (复位位)
            2.02 set_hand_angle(-30°)             手爪 -30° (mid mode, 跟 position5 同款)
            2.03 move_x_with_split(-180mm)       x 在 y=-190 时滑到中间位 -180mm
            2.04 move_y(-65mm)                   y 降到 -65mm (工作深度)
            2.05 move_x_with_split(-215mm)       x 在 y=-65 时滑到最终位 -215mm (push 货物)
            2.06 runner.drop_object()             🆕 v3 新增: 放气 (断开真空, 货物落到目标位)
            2.07 move_x_with_split(-180mm)       x 回到 x_mid (-180, y=-65, 复用 POS_X_MID_MM, 原 2.06 前移)
            2.08 move_y(-190mm)                  y 上升回 -190 (y=-65 → -190, 复用 POS_Y_UP_MM, 原 2.07 前移)
            2.09 move_x_with_split(0mm)          x 归零 (撞墙 calibrate, 在 y=-190 时, 原 2.08 前移)
            ↓
  Phase 3: 底盘后退 130mm (13cm)

⚠️ **变更历史**:
  - **v1 (2026-08-03)**: 11 步原版 (含 suck() 2.00 + drop_object() 2.07)。
  - **v2 (2026-08-04 中午)**: 删掉 suck() 和 drop_object(), 原 11 步 → 9 步。
    序号前移 1 (2.01→2.00, 2.02→2.01, ... 2.10→2.08)。
  - **v3 (2026-08-04 晚上)**: 在 Step 2.05 和 Step 2.06 之间加回 drop_object() (放气),
    9 步 → 10 步。序号后移 1 (2.06→2.07, 2.07→2.08, 2.08→2.09), return dict 加回
    ``drop_result`` (无 suck_result)。语义: push 货物到位后立即放气 → 真空断开 →
    货物落到目标位, 然后再撤退 + 归零。

⚠️ **底盘方向与 position4.py 相反** (与 position3.py 同款):
  - position1 / position4: 先 **back** → arm → **forward**
  - position3 / position6: 先 **forward** → arm → **back**
  - CLI 仍然用 `--forward` / `--back` 两个 mm 距离, 语义不变 (forward=前进,
    back=后退), 只是 phase 顺序相反:
      - position4: Phase 1=back, Phase 3=forward
      - position6: Phase 1=forward, Phase 3=back

⚠️ **底盘移动 (Phase 1/3)**:
  - 走车端 ``car.move_for`` (相对位姿位移, [x_m, 0, 0])。
  - 正值 = 车头方向前进, 负值 = 车头方向后退 (与 task7/dipan.py / position4 同款)。
  - 本脚本 CLI 接收的 --forward / --back 始终是 **正 mm** (用户语义),
    内部 Phase 1 用 +forward → 前进; Phase 3 用 -back → 后退。
  - 130mm / 0.10 m/s ≈ 1.3s, 自适应 timeout = max(5.0, |dist|/vel + 2.0),
    用户 --timeout 兜底。
  - sync=True 阻塞等闭环完成 (后续臂动作必须等底盘停下, 否则位置错位)。
  - 不走 ArmClient._call_car (默认 sync=False 异步会抢跑)。
  - 失败 → job status=failed, 脚本直接 raise 让外层处理 (不静默吞)。

⚠️ **臂序列 (Phase 2) 与 position5.py v3 实际运行一致** (用户 2026-08-04 改):
  - **10 步动作** (含 1 次放气) = 9 步机械臂 + 1 次放气
  - 5 Phase: 准备 (2.00-2.03) + 投递 (2.04-2.05) + 🆕 放气 (2.06) + 撤退 (2.07-2.08) + 归零 (2.09)
  - 常量: y_up=-190, arm=+90°, hand=-30°, x_mid=-180, y_down=-65,
          x_final=-215, x_return=0
  - 见 position5.py docstring 完整解释 (顺序关键 / 业务硬限 / 保护区外)
  - 手爪 UP 必须 ``client.set_hand_angle(angle, speed, timeout=...)``:
    ArmRunner 没有 set_hand_angle (只有 set_storage), 且 timeout 是必填位置参
    (与 set_arm_angle 不同)。见 [[armrunner-set-hand-angle-gotcha]]
  - 🆕 v3 加回 1 次 drop_object() (在 Step 2.06), 但**不加回 suck()** (本脚本前提:
    货物已在手里)。其他吸/放气动作仍由上层单独触发。

⚠️ **为什么不直接 import position4.py / position5.py**:
  task7 包内模块被外部清空过一次 (见 [[task5-rebuild-2026-07-22]] 同款教训),
  本脚本自包含: 只依赖 ``main.arm`` + ``main.arm.each_task.common``,
  不 import task7 包内任何模块。Phase 2 的 11 步动作按用户"自己写"指示内联实现。

⚠️ **底盘位移精度**: ``move_for`` 是闭环位移 (odometry 积分), 130mm 量级现场
  实测累积误差可能到 1-3cm。若需要 mm 级精度, 改用 ``move_to_position`` (绝对
  目标) + 视觉闭环, 或在调用方用 GET /v1/realtime/chassis/state 复核。

跑法:
    python main/arm/each_task/task7/position6.py
    python -m main.arm.each_task.task7.position6
    python main/arm/each_task/task7/position6.py --forward 200 --back 200   # 改底盘距离
    python main/arm/each_task/task7/position6.py --x-final -240              # 改臂最终位
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

LOG_PREFIX: str = "[task7/position6]"

# ==== Phase 1 / 3: 底盘移动 (跟前 version 一致, 保持不变) ====
# 注意: position6 是先前进 → 再后退, phase 顺序与 position4.py 相反。
# 但 CLI 仍然用 --forward / --back 两个语义清晰的标志, 不引入新词。

DEFAULT_FORWARD_MM: float = 130.0
"""Phase 1 前进距离 (mm, **先** 走的那个)。用户 2026-08-03 指定 13cm = 130mm。
CLI 接收正值, 内部直接用 → move_for x_m = +dist_mm/1000 (前进)。
负值会被 ``abs()`` 强制取正再转换, 避免误后退错过摆位。"""

DEFAULT_BACK_MM: float = 130.0
"""Phase 3 后退距离 (mm, **再** 走的那个)。用户 2026-08-03 指定 13cm = 130mm。
CLI 接收正值, 内部转负号 → move_for x_m = -dist_mm/1000 (后退)。
负值会被 ``abs()`` 强制取正再取负, 避免误前进撞墙。"""

DEFAULT_CHASSIS_TIMEOUT_S: float = 10.0
"""底盘 move_for HTTP 同步超时兜底 (秒)。
130mm / 0.10 m/s ≈ 1.3s; 脚本按 max(5.0, |dist|/vel + 2.0) 自适应放大,
用户 ``--timeout`` 是这个自适应值的下限。"""

DEFAULT_CHASSIS_VELOCITY_MS: float = 0.10
"""底盘最大线速度 (m/s), 与 task7/dipan.py / position4.py 默认一致。"""

# ==== Phase 2: 10 步动作 (与 position5.py v3 实际运行参数 + 顺序一致, 2026-08-04 改) ====
POS_Y_UP_MM: float = -190.0
"""Phase 2.00 / 2.08: y 抬到 -190mm (出保护区, 给 set_*_angle 和 move_x 留安全余量)。

⚠️ ≤ soft_y_max=-200 才不超业务硬限。
⚠️ 2.00 (Phase A 抬高) + 2.08 (Phase C 撤退抬 y, v3 由原 2.07 前移) 复用同一个常量,
   现场改值自动同步两处。"""

POS_ARM_DEG: float = 90.0
"""Phase 2.01: 大臂 +90° (复位位, 业务硬限上界 [-150, +90])。"""

POS_HAND_DEG: float = -30.0
"""Phase 2.02: 手爪 -30° (mid mode, 与 position5.py 实际运行值一致, 2026-08-04 改)。

⚠️ 改名历史: 原 POS_HAND_DEG=-45, 2026-08-04 跟 position5 统一改成 -30。
⚠️ 必须走 ``client.set_hand_angle(angle, speed, timeout=...)`` (ArmRunner 没 set_hand_angle,
   timeout 必填位置参)。见 [[armrunner-set-hand-angle-gotcha]]"""

POS_X_MID_MM: float = -180.0
"""Phase 2.03 / 2.07: x 中间位 (出保护区后第一段 x 缓冲 + 撤退回归)。

⚠️ 沿用 position5.py 的 -180mm, 现场可微调。
⚠️ 必须 ≥ x_min_m=-320 软限位; -180 距下界还有 140mm 余量。
⚠️ 2.03 + 2.07 (v3 由原 2.06 前移) 复用同一个常量, 现场改值自动同步两处。"""

POS_Y_DOWN_MM: float = -65.0
"""Phase 2.04: y 降到 -65mm (工作深度, 保护区外, 与 position5.py 实际运行值一致)。

⚠️ -65 距离保护区下边界 -30 还有 35mm 余量。"""

POS_X_FINAL_MM: float = -215.0
"""Phase 2.05: x 在 y=-65 时滑到最终位 -215mm (push 货物, 与 position5.py 实际值一致)。

⚠️ 必须 ≥ x_min_m=-320 软限位; -215 距下界还有 105mm 余量。"""

POS_X_RETURN_MM: float = 0.0
"""Phase 2.09: x 在 y=-190 时归零 (撞墙 calibrate, 跟 position5 同款, v3 由原 2.08 前移)。"""

ANGLE_SPEED: int = 80
"""大臂 + 手爪舵机速度, 默认 80 (与 task7 其他脚本一致)。"""


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
        forward_mm: float = DEFAULT_FORWARD_MM,
        back_mm: float = DEFAULT_BACK_MM,
        max_velocity_ms: float = DEFAULT_CHASSIS_VELOCITY_MS,
        timeout: float = DEFAULT_CHASSIS_TIMEOUT_S) -> dict:
    """3 阶段执行: 前进 → 10 步动作 (含 1 次放气 Step 2.06) → 后退 (与 position4 顺序相反)。

    ⚠️ **2026-08-04 v3 改**: 在 Step 2.05 和 Step 2.06 之间加回 drop_object() (放气)。
       9 步 → 10 步。语义: push 货物到位后立即放气 → 真空断开 → 货物落到目标位,
       然后再撤退 + 归零。
    ⚠️ **2026-08-04 v2 改 (历史)**: 删掉 suck() + drop_object(), 原 11 步 → 9 步;
       v3 又加回 drop_object() 一次, 但**不加回 suck()** (本脚本前提: 货物已在手里)。

    Args:
        client: ArmClient (move_x_with_split + set_hand_angle + http.execute_car_action)
        runner: ArmRunner (move_y + set_arm_angle + **drop_object (Step 2.06)**)
        forward_mm: Phase 1 前进距离 (mm, 正值, 默认 130)
        back_mm: Phase 3 后退距离 (mm, 正值, 默认 130)
        max_velocity_ms: 底盘限速 (m/s, 默认 0.10)
        timeout: 底盘 move_for HTTP 同步超时下限 (秒, 默认 10; 脚本自适应放大)

    Returns:
        {
            "ok": True,
            "forward_mm": float,
            "back_mm": float,
            "forward_job": dict,               # Phase 1 job
            "back_job": dict,                  # Phase 3 job
            "y_up_mm": -190.0,
            "arm_deg": 90.0,
            "hand_deg": -30.0,
            "x_mid_mm": -180.0,
            "y_down_mm": -65.0,
            "x_final_mm": -215.0,
            "x_return_mm": 0.0,
            "x_mid_result": dict,              # Phase 2.03 split
            "x_final_result": dict,            # Phase 2.05 split
            "drop_result": dict,               # 🆕 v3 Phase 2.06 放气 (y=-65 + x=-215)
            "x_mid_return_result": dict,       # Phase 2.07 split (v3 由原 2.06 前移)
            "x_return_result": dict,           # Phase 2.09 split (v3 由原 2.08 前移)
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (forward → 10 步动作 (含 1 次放气) → backward) ==========")
    print(f"  Phase 1: 前进 {forward_mm:.0f}mm")
    print(f"  Phase 2: 10 步动作 (与 position5.py v3 实际运行参数 + 顺序一致, 含 1 次放气)")
    print(f"  Phase 3: 后退 {back_mm:.0f}mm")

    # ==== Phase 1: 底盘前进 (底盘, 不变) ====
    forward_signed_mm = abs(forward_mm)  # 强制转正, 即便用户传 --forward -50 也变成 +130 前进
    forward_timeout = max(timeout, 5.0, abs(forward_signed_mm) / 1000.0 / max(max_velocity_ms, 0.01) + 2.0)
    print(f"\n  ── Phase 1: 底盘前进 {forward_signed_mm:.0f}mm ──")
    forward_job = _chassis_move_for(
        client, forward_signed_mm,
        max_velocity_ms, forward_timeout,
        log_prefix=f"  {LOG_PREFIX} phase1",
    )

    # ==== Phase 2: 10 步动作 (与 position5.py v3 同款, 内联实现, 含 1 次放气 Step 2.06) ====
    print(f"\n  ── Phase 2: 10 步动作 (与 position5.py v3 同款, 含 1 次放气) ──")

    # 2.00 y 抬高 (出保护区, 给后续 set_*_angle 和 move_x 留余地)
    print(f"  [2.00] move_y({POS_Y_UP_MM}mm)   y 出保护区")
    runner.move_y(POS_Y_UP_MM, verify=True)

    # 2.01 大臂 +90° (复位位, 保护区允许)
    print(f"  [2.01] set_arm_angle({POS_ARM_DEG}°)  大臂到复位位")
    runner.set_arm_angle(POS_ARM_DEG, speed=ANGLE_SPEED)

    # 2.02 手爪 -30° (mid mode, 跟 position5 同款)
    # ⚠️ ArmRunner 没有 set_hand_angle, 必须走 client.set_hand_angle, timeout 必填。
    print(f"  [2.02] set_hand_angle({POS_HAND_DEG}°)  手爪到 mid mode")
    client.set_hand_angle(
        POS_HAND_DEG, speed=ANGLE_SPEED,
        timeout=runner.default_timeout_s,
    )

    # 2.03 x_mid 在 y=-190 时滑到中间位 -180mm (出保护区第一段 x 缓冲)
    print(f"  [2.03] move_x_with_split({POS_X_MID_MM}mm)  x_mid → 中间位 (y=-190, split 兜底)")
    x_mid_result = move_x_with_split(
        client, runner, POS_X_MID_MM,
        log_prefix=f"  {LOG_PREFIX} phase2.03",
    )

    # 2.04 y 降到 -65mm (工作深度, 保护区外)
    print(f"  [2.04] move_y({POS_Y_DOWN_MM}mm)   y → 工作深度 (保护区外)")
    runner.move_y(POS_Y_DOWN_MM, verify=True)

    # 2.05 x 在 y=-65 时滑到最终位 -215mm (push 货物)
    print(f"  [2.05] move_x_with_split({POS_X_FINAL_MM}mm)  x_final → 最终位 (y=-65, split 兜底)")
    x_final_result = move_x_with_split(
        client, runner, POS_X_FINAL_MM,
        log_prefix=f"  {LOG_PREFIX} phase2.05",
    )

    # 🆕 2.06 (v3 新增): 放气 (断开真空, 货物落到目标位)
    # ⚠️ 必须**在 x 推到位之后** (2.05 后), **x 撤退之前** (2.07 前)。
    # runner.drop_object() 走 runner, 不走 client (跟 suck/drop_object 同款, 见
    # main/arm/loops/runner.py:185 drop_object)。
    print(f"  [2.06] drop_object()  🆕 v3 放气 (断开真空, 货物落目标位, y=-65 + x=-215)")
    drop_result = runner.drop_object()

    # 2.07 x 回到 x_mid (-180) (y=-65 时, 防止归零冲过头撞到已投放的货物/撞墙, 原 2.06 前移)
    print(f"  [2.07] move_x_with_split({POS_X_MID_MM}mm)  x_mid 回归 (y=-65, 复用 POS_X_MID_MM, split 兜底)")
    x_mid_return_result = move_x_with_split(
        client, runner, POS_X_MID_MM,
        log_prefix=f"  {LOG_PREFIX} phase2.07",
    )

    # 2.08 y 上升回 -190mm (出工作深度, 给 x 归零让路, 原 2.07 前移)
    print(f"  [2.08] move_y({POS_Y_UP_MM}mm)   y 出工作深度 (复用 POS_Y_UP_MM)")
    runner.move_y(POS_Y_UP_MM, verify=True)

    # 2.09 x 在 y=-190 时归零 (撞墙 calibrate, 原 2.08 前移)
    print(f"  [2.09] move_x_with_split({POS_X_RETURN_MM}mm)  x_return → 0 位 (y=-190, split 撞墙 calibrate)")
    x_return_result = move_x_with_split(
        client, runner, POS_X_RETURN_MM,
        log_prefix=f"  {LOG_PREFIX} phase2.09",
    )

    # ==== Phase 3: 底盘后退 (底盘, 不变) ====
    back_signed_mm = -abs(back_mm)  # 强制转负, 即便用户传 --back -50 也变成 -130 后退
    back_timeout = max(timeout, 5.0, abs(back_signed_mm) / 1000.0 / max(max_velocity_ms, 0.01) + 2.0)
    print(f"\n  ── Phase 3: 底盘后退 {abs(back_signed_mm):.0f}mm ──")
    back_job = _chassis_move_for(
        client, back_signed_mm,
        max_velocity_ms, back_timeout,
        log_prefix=f"  {LOG_PREFIX} phase3",
    )

    dt = time.time() - t0
    print(f"========== {LOG_PREFIX} 完成 ({dt:.2f}s) ==========\n")

    return {
        "ok": True,
        "forward_mm": forward_mm,
        "back_mm": back_mm,
        "forward_job": forward_job,
        "back_job": back_job,
        "y_up_mm": POS_Y_UP_MM,
        "arm_deg": POS_ARM_DEG,
        "hand_deg": POS_HAND_DEG,
        "x_mid_mm": POS_X_MID_MM,
        "y_down_mm": POS_Y_DOWN_MM,
        "x_final_mm": POS_X_FINAL_MM,
        "x_return_mm": POS_X_RETURN_MM,
        "x_mid_result": x_mid_result,
        "x_final_result": x_final_result,
        "drop_result": drop_result,                       # 🆕 v3 新增: Step 2.06 放气
        "x_mid_return_result": x_mid_return_result,
        "x_return_result": x_return_result,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 底盘 `--forward` / `--back` + 臂 7 个常量 override (与 position5 v3 同款)。

    `--forward` / `--back` 接收正 mm (用户语义), 内部自动转符号:
      - --forward  → move_for x_m = +|forward|/1000  (前进)  ← Phase 1
      - --back     → move_for x_m = -|back|/1000     (后退)  ← Phase 3
    即便用户传负值也会被 abs() 强制取正再转换, 避免误传撞墙。

    与 position4.py 的区别仅 phase 顺序:
      - position4: Phase 1=back, Phase 3=forward
      - position6: Phase 1=forward, Phase 3=back
    Phase 2 臂序列与 position5 v3 完全同款 (10 步动作, 含 1 次放气 Step 2.06)。
    """
    p = argparse.ArgumentParser(
        description=(
            "task7 position6: 前进 13cm → 10 步动作 (含 1 次放气, 与 position5 v3 同款) → 后退 13cm"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--forward", type=float, default=DEFAULT_FORWARD_MM,
                   help="Phase 1 前进距离 (mm, 默认 130 = 13cm, 强制正值)")
    p.add_argument("--back", type=float, default=DEFAULT_BACK_MM,
                   help="Phase 3 后退距离 (mm, 默认 130 = 13cm, 强制正值)")
    p.add_argument("--vel", type=float, default=DEFAULT_CHASSIS_VELOCITY_MS,
                   dest="max_velocity",
                   help="底盘最大线速度 (m/s, 默认 0.10)")
    p.add_argument("--timeout", type=float, default=DEFAULT_CHASSIS_TIMEOUT_S,
                   help="底盘 move_for HTTP 同步超时下限 (秒, 默认 10)")
    # Phase 2 臂序列参数 (与 position5 v3 同款, 7 个常量)
    p.add_argument("--y-up", type=float, default=POS_Y_UP_MM,
                   help="Phase 2.00/2.08 y 抬高目标 (mm, 默认 -190, 复用)")
    p.add_argument("--arm", type=float, default=POS_ARM_DEG,
                   help="Phase 2.01 大臂角度 (°, 默认 +90)")
    p.add_argument("--hand", type=float, default=POS_HAND_DEG,
                   help="Phase 2.02 手爪角度 (°, 默认 -30=mid mode, 跟 position5 同款)")
    p.add_argument("--x-mid", type=float, default=POS_X_MID_MM,
                   help="Phase 2.03/2.07 x 中间位 (mm, 默认 -180, 复用, 改值同步两处, v3 由 2.06 前移)")
    p.add_argument("--y-down", type=float, default=POS_Y_DOWN_MM,
                   help="Phase 2.04 工作深度 (mm, 默认 -65=保护区外, 跟 position5 同款)")
    p.add_argument("--x-final", type=float, default=POS_X_FINAL_MM,
                   help="Phase 2.05 最终 x (mm, 默认 -215, 在 y=-65 时调用, 跟 position5 同款)")
    p.add_argument("--x-return", type=float, default=POS_X_RETURN_MM,
                   dest="x_return",
                   help="Phase 2.09 归零 x (mm, 默认 0=撞墙, 在 y=-190 时调用, v3 由 2.08 前移)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    # 让 CLI 覆盖生效
    global POS_Y_UP_MM, POS_ARM_DEG, POS_HAND_DEG
    global POS_X_MID_MM, POS_Y_DOWN_MM, POS_X_FINAL_MM, POS_X_RETURN_MM
    POS_Y_UP_MM = args.y_up
    POS_ARM_DEG = args.arm
    POS_HAND_DEG = args.hand
    POS_X_MID_MM = args.x_mid
    POS_Y_DOWN_MM = args.y_down
    POS_X_FINAL_MM = args.x_final
    POS_X_RETURN_MM = args.x_return
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner,
        forward_mm=args.forward,
        back_mm=args.back,
        max_velocity_ms=args.max_velocity,
        timeout=args.timeout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
