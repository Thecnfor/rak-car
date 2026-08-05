"""task6 / getzuowu2 —— **任务六抓作物 2** 6 步序列 (5 步臂 + 1 步底盘前进)。

按用户 2026-08-04 指定顺序:

  Step 1: runner.move_y(-190mm)                y 抬高到 -190mm (距 soft_y_max=-200 留 10mm 余量)
            ↓
  Step 2: 底盘前进 15cm (150mm)                  move_for x_m = +0.150 (底盘 +15cm, 臂位姿不变)
            ↓
  Step 3: move_x_with_split(-170mm)            x 滑到 -170mm (距下界 -320 留 150mm 余量)
            ↓
  Step 4: runner.set_arm_angle(-95°)           大臂旋转至 -95° (推牌姿势, 业务硬限 [-150, +90]° 内)
            ↓
  Step 5: client.set_hand_angle(0°)            手爪舵机 0° (DOWN, 业务硬限上界 0°)
            ↓
  Step 6: runner.move_y(-10mm)                 y 下降到 -10mm ⚠️ **终态进入保护区 [0, -80]**
            ↓
  终态: y=-10 + arm=-95° + x=-170 + hand=0° + 底盘前进 15cm

⚠️ **业务硬限 / 保护区逐项核对** (走前要核对, 见 ARM_API §1.1 / §7):
  - y=-190 ≤ soft_y_max=-200 ✓               距上限 10mm 余量
  - x=-170 ∈ [-320, +220] mm ✓                距下界 -320 还有 150mm, 距上界 +220 还有 390mm
  - arm=-95 ∈ [-150, +90]° ✓                  距上界 +90° 远, 距下界 -150° 55°
  - hand=0 ∈ [-90, 0]° ✓                       正好上界 (DOWN 位)
  - **y=-10 ∈ [0, -80] (保护区内!)** ⚠️⚠️⚠️      终态进入保护区, 后续 wrapper 调用受限

⚠️ **顺序关键** (用户硬指定, 不可调换):
  - Step 1 y=-190 抬高到保护区外 + 离上限 10mm, 让后续 arm/x/hand 全在保护区外执行。
  - Step 2 底盘前进 15cm: 此时 y=-190 (臂已抬到保护区外, 安全), 臂位姿不变。
                      底盘前进相当于"车先往前开一截", 后续 Step 3-6 都在底盘 +15cm
                      后的新世界坐标系执行。**注意顺序**: 跟 getzuowu1.py (y→x→底盘)
                      不同, 本脚本是 y→**底盘**→x, 即 "**先开车再伸臂**" (drive-then-reach)。
  - Step 3 x=-170: y=-190 保护区外, move_x_with_split wrapper 放行。
  - Step 4 arm=-95°: y=-190 保护区外, wrapper 放行。
  - Step 5 hand=0° (DOWN): 此时 y=-190 保护区外, wrapper 放行。
  - Step 6 y=-10: move_y 从不被保护区拦, 但 **终点 y=-10 进入保护区**,
                   后续 set_arm_angle / set_hand_angle / move_x 走 wrapper 会 fail-closed。

⚠️ **底盘前进 (Step 2)**:
  - 走车端 ``car.move_for`` (相对位姿位移, [x_m, 0, 0])。
  - 正值 = 车头方向前进, 负值 = 车头方向后退 (move_for 自身符号约定)。
  - 150mm / 0.10 m/s ≈ 1.5s, 自适应 timeout = max(5.0, |dist|/vel + 2.0),
    用户 --timeout 兜底。
  - sync=True 阻塞等闭环完成 (后续臂动作必须等底盘停下, 否则位置错位)。
  - 不走 ArmClient._call_car (默认 sync=False 异步会抢跑)。
  - 失败 → job status=failed, 脚本直接 raise 让外层处理 (不静默吞)。
  - CLI 接收正 mm (用户语义), 内部保持正号 → move_for x_m = +dist_mm/1000 (前进)。
  - 即便用户传负值也会被 ``abs()`` 强制取正再前进, 避免误传后退。

⚠️ **为什么 hand=0° 走 client 不走 runner**:
  ArmRunner 没有 ``set_hand_angle`` (只有 ``set_storage_angle``), 必须走
  ``client.set_hand_angle(angle, speed, timeout=...)``, 且 ``timeout`` 是必填位置参
  (与 ``set_arm_angle`` 默认值不同)。见 [[armrunner-set-hand-angle-gotcha]]。

⚠️ **底盘位移精度**: ``move_for`` 是闭环位移 (odometry 积分), 150mm 量级现场
  实测累积误差可能到 1-3cm。若需要 mm 级精度, 改用 ``move_to_position`` (绝对
  目标) + 视觉闭环, 或在调用方用 GET /v1/realtime/chassis/state 复核。

⚠️ **y=-10 终态在保护区内** (跟 getzuowu1.py 改 y_low=-15 同款风险):
  Step 6 把 y 从 -190 降到 -10, 跨入保护区的"浅"位置 (-10 ∈ [0, -80])。
  之后:
    - ``runner.set_arm_angle(非 0/MID)`` → **wrapper fail-closed 拒**
    - ``client.set_hand_angle(非 -90)``  → **wrapper fail-closed 拒**
    - ``runner.move_x(...)``            → **wrapper fail-closed 拒**
  后续要再动臂/手/x 必须先 ``runner.move_y(-150)`` 出保护区 (move_y 自己不被拦)。
  若需要"贴近地面"动作 (抓作物/推杆), 此终态正好, 但**不要再 set_arm/hand/x**。

⚠️ **与 task6/{target1, position1, position2, getzuowu1}.py 的区别**:
  - target1.py = 底盘后退 9cm + 4 步臂 (y→arm→hand→x) + OCR 写 liaobiao1, 终态 y=-143
  - position1.py = 5 步纯臂, x=-18, 终态 y=-95 (近点, 无底盘)
  - position2.py = 5 步纯臂, x=-75, 终态 y=-95 (中点, 无底盘)
  - getzuowu1.py = 6 步 (y→x→底盘→arm→hand→y), x=-220, 底盘前进 23cm, 终态 y=-100,
                  模式 "**伸臂后开车**" (reach-then-drive, 底盘在 x 后)
  - **getzuowu2.py = 6 步 (y→底盘→x→arm→hand→y), x=-170, 底盘前进 15cm,
                  终态 y=-10 ⚠️ 进保护区, 模式 "先开车再伸臂" (drive-then-reach, 底盘在 x 前)**

⚠️ **本文件自包含** (与 task6/{tuigan, wenzishibie, position1, position2, getzuowu1}.py +
   task7/{position*.py} 同款):
   只依赖 ``main.arm.ArmClient`` + ``main.arm.ArmRunner`` +
   ``main.arm.each_task.common.move_x_with_split``,
   不 import task6 包内任何模块。原因: task5 包曾被外部清空过一次
   (见 [[task5-rebuild-2026-07-22]]), 自包含可保证 ``python getzuowu2.py``
   直接跑不受影响。

跑法:
    python main/arm/each_task/task6/getzuowu2.py
    python -m main.arm.each_task.task6.getzuowu2
    python main/arm/each_task/task6/getzuowu2.py --forward 100          # 底盘少前进 5cm
    python main/arm/each_task/task6/getzuowu2.py --x-target -150        # x 不那么远
    python main/arm/each_task/task6/getzuowu2.py --y-low -50            # y 不到那么浅 (留在保护区外)
    python main/arm/each_task/task6/getzuowu2.py --vel 0.05             # 底盘更慢更稳
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


# ---------- 默认参数 (用户 2026-08-04 现场指定) ----------

LOG_PREFIX: str = "[task6/getzuowu2]"

# ==== Step 1: y 抬高 (出保护区, 离上限留 10mm 余量) ====
POS_Y_HIGH_MM: float = -190.0
"""Step 1: y 抬到 -190mm (距业务硬限上界 soft_y_max=-200 留 10mm 余量)。

⚠️ -190 ≤ soft_y_max=-200 ✓ (10mm buffer)
⚠️ -190 ≤ -80 ✓ (保护区外, 后续 arm/x/hand/底盘 全安全)
⚠️ 起点 y 若已在保护区 (y > -80), move_y(-190) 自动抬出保护区, 无需软抢答。"""

# ==== Step 2: 底盘前进 15cm (150mm) ====
DEFAULT_FORWARD_MM: float = 150.0
"""Step 2: 底盘前进距离 (mm)。用户 2026-08-04 指定 15cm = 150mm。
CLI 接收正值, 内部直接用 → move_for x_m = +dist_mm/1000 (前进)。
负值会被 ``abs()`` 强制取正再前进, 避免误传后退。

⚠️ 此时 y=-190 (臂抬到保护区外), 底盘前进相当于"先开车",
   后续 Step 3-6 都在底盘 +15cm 后的新位置执行。
⚠️ 比 getzuowu1.py 的 23cm 短 8cm: getzuow1 先伸臂再开车 (臂 + 底盘都贡献距离),
   本脚本先开车再伸臂 (底盘先到位, 臂只伸到中距离)。"""

DEFAULT_CHASSIS_TIMEOUT_S: float = 10.0
"""底盘 move_for HTTP 同步超时兜底 (秒)。
150mm / 0.10 m/s ≈ 1.5s; 脚本按 max(5.0, |dist|/vel + 2.0) 自适应放大,
用户 ``--timeout`` 是这个自适应值的下限。"""

DEFAULT_CHASSIS_VELOCITY_MS: float = 0.10
"""底盘最大线速度 (m/s), 与 task7/dipan.py / position*.py 默认一致。"""

# ==== Step 3: x 滑到 -170mm (中远点, 距下界 150mm 余量) ====
POS_X_TARGET_MM: float = -170.0
"""Step 3: x 滑到 -170mm (中远点, 距下界 -320 留 150mm 余量)。

⚠️ -170 ∈ [-320, +220] mm ✓ (距下界 150mm, 距上界 390mm)
⚠️ 走 ``move_x_with_split`` (belt-slip / wall_hit / overshoot 检测),
   兜底 ARM_API §9.1。
⚠️ 与 getzuowu1.py (x=-220) / position1.py (x=-18) / position2.py (x=-75) 都不同,
   是"中远点", 适配"先开车再伸臂"模式: 底盘已前进 15cm 缩短距离, x 不用伸太远。"""

# ==== Step 4: 大臂到 -95° (推牌姿势, 跟 task6/target1.py 同款) ====
POS_ARM_DEG: float = -95.0
"""Step 4: 大臂旋转至 -95° (推牌姿势, 业务硬限 [-150, +90]° 内)。

⚠️ -95 ∈ [-150, +90]° ✓ (距上界 +90° 远, 距下界 -150° 还有 55°)
⚠️ 与 task6/target1.py / getzuowu1.py 的 arm=-95° 完全一致 (推牌姿势)。
⚠️ runner.set_arm_angle 默认 speed=80, 大扭矩动作; 用户 CLI 暂不暴露 speed。"""

# ==== Step 5: 手爪到 0° (DOWN 位, 业务硬限上界) ====
POS_HAND_DEG: float = 0.0
"""Step 5: 手爪舵机 0° (DOWN 位, 正好业务硬限上界 [-90, 0]°)。

⚠️ hand=0° = DOWN, 是保护区 y ∈ [0, -80] 内 set_hand_angle **唯一例外** (待定, 见
   safety.py 实际行为)。但本脚本 Step 5 时 y=-190 始终在保护区外, wrapper 放行
   无问题, 不需要 tuigan.py 那种底层直调。
⚠️ 必须走 ``client.set_hand_angle(angle, speed, timeout=...)`` (ArmRunner 没
   set_hand_angle, timeout 必填位置参)。见 [[armrunner-set-hand-angle-gotcha]]"""

# ==== Step 6: y 下降到 -10mm (终态 ⚠️ 进入保护区) ====
POS_Y_LOW_MM: float = -10.0
"""Step 6: y 下降到 -10mm ⚠️ **终态进入保护区 [0, -80] 浅位**。

⚠️ -10 ∈ [0, -80] 保护区内!
⚠️ 终态保持 y=-10 + arm=-95° + x=-170 + hand=0° + 底盘 +15cm, 适合"贴地抓作物"姿态。
⚠️ y 从 -190 → -10 是 +180mm 下降, runner.move_y 内部 PID 闭环, 终点 -10 是
   保护区内"较浅"位置 (距 0 还有 10mm)。
⚠️ **进保护区后, 后续 wrapper 调用受限**:
    - runner.set_arm_angle(非 0/MID) → fail-closed 拒
    - client.set_hand_angle(非 -90)  → fail-closed 拒
    - runner.move_x(...)             → fail-closed 拒
    要再动臂/手/x 必须先 runner.move_y(-150) 出保护区 (move_y 自己不被拦)。
⚠️ 跟 getzuowu1.py 改 y_low=-15 同款风险 (均进保护区), 用户 2026-08-04 故意保留。
⚠️ 与 position1/2.py 的 y_low=-95 不同 (留 15mm 余量在保护区外), 本脚本 -10 进保护区。"""

# ==== 时序常量 ====
ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度, 默认 80。与 task6/{target1, target2, tuigan, position1, position2,
getzuowu1}.py、task7/*.py 一致。"""


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
        y_high_mm: float = POS_Y_HIGH_MM,
        forward_mm: float = DEFAULT_FORWARD_MM,
        x_target_mm: float = POS_X_TARGET_MM,
        arm_deg: float = POS_ARM_DEG,
        hand_deg: float = POS_HAND_DEG,
        y_low_mm: float = POS_Y_LOW_MM,
        max_velocity_ms: float = DEFAULT_CHASSIS_VELOCITY_MS,
        timeout: float = DEFAULT_CHASSIS_TIMEOUT_S) -> dict:
    """6 步序列 (顺序固定 y → 底盘 → x → arm → hand → y, 用户 2026-08-04 硬指定)。

    与 task6/getzuowu1.py 的核心区别:
      - getzuowu1 = 6 步 (y → **x → 底盘** → arm → hand → y), 模式 "**伸臂后开车**"
                     (reach-then-drive), x=-220 远点, 底盘 23cm, 终态 y=-100
      - **getzuowu2 = 6 步 (y → **底盘 → x** → arm → hand → y)**, 模式 "**先开车再伸臂**"
                     (drive-then-reach), x=-170 中远点, 底盘 15cm, **终态 y=-10 ⚠️ 进保护区**

    与 task6/target1.py 的核心区别:
      - target1 = 底盘后退 9cm + 4 步臂 (y→arm→hand→x) + OCR, 顺序 "**先底盘后臂**" (back-then-reach)
      - getzuowu2 = **底盘前进 15cm + 5 步臂 (y→底盘→x→arm→hand→y)**, 顺序 "**先臂后底盘**" (臂先抬, 底盘后开)

    Args:
        client:        ArmClient (move_x_with_split + set_hand_angle + http.execute_car_action)
        runner:        ArmRunner (move_y + set_arm_angle)
        y_high_mm:     Step 1 y 抬高目标 (mm, 默认 -190, 距上限 10mm)
        forward_mm:    Step 2 底盘前进距离 (mm, 正值, 默认 150 = 15cm)
        x_target_mm:   Step 3 x 目标位置 (mm, 默认 -170, 距下界 150mm)
        arm_deg:       Step 4 大臂角度 (°, 默认 -95, 推牌姿势)
        hand_deg:      Step 5 手爪角度 (°, 默认 0 = DOWN)
        y_low_mm:      Step 6 y 下降目标 (mm, 默认 **-10**, ⚠️ 进保护区)
        max_velocity_ms: 底盘限速 (m/s, 默认 0.10)
        timeout:       底盘 move_for HTTP 同步超时下限 (秒, 默认 10)

    Returns:
        {
            "ok": True,
            "y_high_mm": float,
            "forward_mm": float,
            "x_target_mm": float,
            "arm_deg":   float,
            "hand_deg":  float,
            "y_low_mm":  float,
            "forward_job": dict,         # Step 2 底盘 job
            "x_result":  dict,           # Step 3 split 结果
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (6 步: y → 底盘 → x → arm → hand → y) ==========")
    print(f"  Step 1: y={y_high_mm:.0f}mm")
    print(f"  Step 2: 底盘前进 {forward_mm:.0f}mm = {forward_mm/10:.1f}cm")
    print(f"  Step 3: x={x_target_mm:.0f}mm")
    print(f"  Step 4: arm={arm_deg:.0f}°")
    print(f"  Step 5: hand={hand_deg:.0f}°")
    print(f"  Step 6: y={y_low_mm:.0f}mm  ⚠️ 进保护区 [0, -80]")

    # ==== Step 1: y 抬高到 -190 (出保护区 + 离上限 10mm) ====
    print(f"\n  ── Step 1: y → {y_high_mm:.0f}mm (出保护区, 距上限 10mm) ──")
    runner.move_y(y_high_mm, verify=True)

    # ==== Step 2: 底盘前进 15cm (y=-190 保护区外, 同步等闭环) ====
    # 强制转正: 即便用户传了 --forward -50 (误) 也变成 +150 (前进)。
    forward_signed_mm = abs(forward_mm)
    forward_timeout = max(timeout, 5.0, abs(forward_signed_mm) / 1000.0 / max(max_velocity_ms, 0.01) + 2.0)
    print(f"\n  ── Step 2: 底盘前进 {forward_signed_mm:.0f}mm (y=-190 保护区外, 同步等闭环) ──")
    forward_job = _chassis_move_for(
        client, forward_signed_mm,
        max_velocity_ms, forward_timeout,
        log_prefix=f"  {LOG_PREFIX} step2",
    )

    # ==== Step 3: x 滑到 -170 (底盘已前进, y=-190 保护区外, split 兜底) ====
    print(f"\n  ── Step 3: x → {x_target_mm:.0f}mm (中远点, 距下界 -320 留 150mm, split 兜底) ──")
    x_result = move_x_with_split(
        client, runner, x_target_mm,
        log_prefix=f"  {LOG_PREFIX} step3",
    )

    # ==== Step 4: 大臂旋转至 -95° (y=-190 保护区外, wrapper 放行) ====
    print(f"\n  ── Step 4: arm → {arm_deg:.0f}° (推牌姿势, 距上界 +90° 远) ──")
    runner.set_arm_angle(arm_deg, speed=ANGLE_SPEED)

    # ==== Step 5: 手爪 0° (DOWN, y=-190 保护区外, 走 client) ====
    # ⚠️ ArmRunner 没有 set_hand_angle, 必须走 client.set_hand_angle,
    #    且 timeout 是必填位置参 (与 set_arm_angle 默认值不同)。
    #    见 [[armrunner-set-hand-angle-gotcha]]
    print(f"\n  ── Step 5: hand → {hand_deg:.0f}° (DOWN, y=-190 保护区外) ──")
    client.set_hand_angle(
        hand_deg, speed=ANGLE_SPEED,
        timeout=runner.default_timeout_s,
    )

    # ==== Step 6: y 下降到 -10 (⚠️ 终态进保护区 [0, -80]) ====
    print(f"\n  ── Step 6: y → {y_low_mm:.0f}mm ⚠️ 进保护区 [0, -80] ──")
    runner.move_y(y_low_mm, verify=True)

    dt = time.time() - t0
    print(f"\n========== {LOG_PREFIX} 完成 ({dt:.2f}s) ==========")
    print(f"  终态: y={y_low_mm:.0f}mm + arm={arm_deg:.0f}° + "
          f"x={x_target_mm:.0f}mm + hand={hand_deg:.0f}° + 底盘 +{forward_signed_mm:.0f}mm")
    print(f"  ⚠️ y=-10 ⚠️⚠️⚠️ 已在保护区 [0, -80] 内, 后续 set_arm/hand/x 走 wrapper 会 fail-closed!")
    print(f"     要再动臂/手/x 必须先 runner.move_y(-150) 出保护区。\n")

    return {
        "ok": True,
        "y_high_mm": y_high_mm,
        "forward_mm": forward_mm,
        "x_target_mm": x_target_mm,
        "arm_deg": arm_deg,
        "hand_deg": hand_deg,
        "y_low_mm": y_low_mm,
        "forward_job": forward_job,
        "x_result": x_result,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 6 个序列常量 + 2 个底盘常量全暴露, 默认值与模块常量一致。

    参数命名跟模块常量对齐:
      --y-high   / --forward   / --x-target   / --arm   / --hand   / --y-low
      --vel      / --timeout
    """
    p = argparse.ArgumentParser(
        description=(
            "task6 getzuowu2: 6 步序列 "
            "(y=-190 → 底盘+15cm → x=-170 → arm=-95° → hand=0° → y=-10⚠️进保护区)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--y-high", type=float, default=POS_Y_HIGH_MM,
                   dest="y_high",
                   help="Step 1 y 抬高目标 (mm, 默认 -190, 距 soft_y_max=-200 留 10mm)")
    p.add_argument("--forward", type=float, default=DEFAULT_FORWARD_MM,
                   help="Step 2 底盘前进距离 (mm, 默认 150 = 15cm, 强制正值)")
    p.add_argument("--x-target", type=float, default=POS_X_TARGET_MM,
                   dest="x_target",
                   help=("Step 3 x 目标位置 (mm, 默认 -170, 中远点, 距下界 -320 留 150mm, "
                         "必须在 [-320, +220])"))
    p.add_argument("--arm", type=float, default=POS_ARM_DEG,
                   help="Step 4 大臂角度 (°, 默认 -95, 推牌姿势, 业务硬限 [-150, +90]° 内)")
    p.add_argument("--hand", type=float, default=POS_HAND_DEG,
                   help="Step 5 手爪角度 (°, 默认 0 = DOWN)")
    p.add_argument("--y-low", type=float, default=POS_Y_LOW_MM,
                   dest="y_low",
                   help=("Step 6 y 下降目标 (mm, 默认 **-10**, ⚠️ 进保护区 [0, -80]; "
                         "要留在保护区外请传 ≤ -80)"))
    p.add_argument("--vel", type=float, default=DEFAULT_CHASSIS_VELOCITY_MS,
                   dest="max_velocity",
                   help="底盘最大线速度 (m/s, 默认 0.10)")
    p.add_argument("--timeout", type=float, default=DEFAULT_CHASSIS_TIMEOUT_S,
                   help="底盘 move_for HTTP 同步超时下限 (秒, 默认 10)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args()
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner,
        y_high_mm=args.y_high,
        forward_mm=args.forward,
        x_target_mm=args.x_target,
        arm_deg=args.arm,
        hand_deg=args.hand,
        y_low_mm=args.y_low,
        max_velocity_ms=args.max_velocity,
        timeout=args.timeout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
