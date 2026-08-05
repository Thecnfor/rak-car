"""task6 / target1 —— **任务六位置 1 抓取** 的 3 阶段序列 (底盘后退 + 4 步纯臂 + 文字识别写 liaobiao1)。

按用户 2026-08-04 v2 指定顺序 (x/y 改动, arm/hand 数 值不变, 顺序 / 底盘不动):

  Phase 1: 底盘后退 90mm (9cm)            ← **不变**
            ↓
  Phase 2: 4 步纯臂序列 (顺序固定: y → arm → hand → x)
            2.0 move_y(-143mm)               y 抬高到 -143mm (距 soft_y_max=-200 还有 57mm 余量)
            2.1 set_arm_angle(-95°)          大臂到 -95° (推牌姿势, 业务硬限 [+90, -150]° 内, 不变)
            2.2 set_hand_angle(-55°)         手爪到 -55° (推杆姿态, 业务硬限 [-90, 0]° 内, 不变)
            2.3 move_x_with_split(-121mm)    x 滑到位置 1 (-121mm, 距 x_min=-320 还有 199mm 余量)

⚠️ **v2 改动记录** (用户 2026-08-04 现场微调):
  - y: -200 → **-143** (旧值顶业务硬限上界 0mm 余量太危险, 留 57mm 余量更稳)
  - x: -211 → **-121** (旧值距 -320 还有 109mm, 新值留 199mm)
  - arm/hand/底盘/步骤顺序: **不动**

⚠️ **业务硬限** (走前要核对, 见 ARM_API §1.1 / §7):
  - y=-143 ≤ soft_y_max=-200 ✓ (距上限 57mm 余量, 比 v1 留余量)
  - arm=-95 ∈ [-150, +90]° ✓ (距上界 185° 远, 距下界 55°)
  - hand=-55 ∈ [-90, 0]° ✓ (距上界 35° 远, 距下界 -90° 还有 35°)
  - x=-121 ∈ [-320, +220] mm ✓ (距下界 -320 199mm 余量, 距上界 +220 341mm)
  - y=-143 ≤ -80 ✓ (保护区外 → set_hand_angle(-55°) 不会被 safety 拦)

⚠️ **底盘移动 (Phase 1)**:
  - 走车端 ``car.move_for`` (相对位姿位移, [x_m, 0, 0])。
  - 正值 = 车头方向前进, 负值 = 车头方向后退。
  - 90mm / 0.10 m/s ≈ 0.9s, 自适应 timeout = max(5.0, |dist|/vel + 2.0),
    用户 --timeout 兜底。
  - sync=True 阻塞等闭环完成 (后续臂动作必须等底盘停下, 否则位置错位)。
  - 不走 ArmClient._call_car (默认 sync=False 异步会抢跑)。
  - 失败 → job status=failed, 脚本直接 raise 让外层处理 (不静默吞)。

⚠️ **为什么 set_hand_angle(-55°) 不需要像 tuigan.py 那样走底层直调**:
  - tuigan.py 是在 Y=0 (触底) 时调 set_hand_angle, Y=0 ∈ [0, -80] 保护区
    → safety.py:76 fail-closed 必拒 → 必须绕 wrapper。
  - 本脚本 y=-143 始终在保护区外 (-143 ≤ -80 ✓), 走 ``client.set_hand_angle``
    wrapper 完全合法, 不用绕。

⚠️ **顺序关键**:
  - 第 2.0 步 y 抬到 -143 是为了让后续 set_arm_angle(-95°) / set_hand_angle(-55°)
    / move_x_with_split(-121) 都在 **保护区外** 完成。
    保护区 y ∈ [0, -80]mm 内: set_arm_angle(非 MID/0) / set_hand_angle(非 -90)
    / move_x 都会被 _check_safe 拦截, 所以必须先抬 y。
  - 第 2.1/2.2 步 arm / hand 在 y=-143 都安全 (保护区外 + 业务硬限内)。
  - 第 2.3 步 x 移到 -121 必须在 y ≤ -80 才能调 (y=-143 满足 ✓), 否则 SDK 拦截。
  - **没有第 2.4 步 y 下降** (跟 task7/get_position1.py v2+ 不同) —
    用户指定的 4 步是 "底盘 → y → arm → hand → x", 终态保持 y=-143。
    后续若要 y 下降到工作深度, 调用方负责再调 runner.move_y(...)

⚠️ **本文件自包含** (与 task6/{tuigan, wenzishibie}.py + task7/{position*.py} 同款):
   只依赖 ``main.arm.ArmClient`` + ``main.arm.ArmRunner`` +
   ``main.arm.each_task.common.move_x_with_split``,
   不 import task6 包内任何模块。原因: task5 包曾被外部清空过一次
   (见 [[task5-rebuild-2026-07-22]]), 自包含可保证 ``python target1.py``
   直接跑不受影响。

跑法:
    python main/arm/each_task/task6/target1.py
    python -m main.arm.each_task.task6.target1
    python main/arm/each_task/task6/target1.py --back 100              # 底盘后退 10cm
    python main/arm/each_task/task6/target1.py --y -190                # y 留更宽余量
    python main/arm/each_task/task6/target1.py --x -150                # x 不到那么远
    python main/arm/each_task/task6/target1.py --x-target -100         # x 走 -100 (近点)
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

# Phase 3 加 (2026-08-04 用户新需求): 调 wenzishibie 文字识别 → 自动写 liebiao.liaobiao1
# 突破早期 "自包含" 约定 (单文件 self-contained), 因为 wenzishibie 是 task6 包内
# 姐妹文件, 与 target1.py 在同一目录, 包从未被外部清空过 (task5 清空见
# [[task5-rebuild-2026-07-22]]), 风险低。仍不依赖 main.arm.__init__ 或其他子包。
#
# v6 简化: 入库 (append_liaobiao1) 现在由 wenzishibie.run(target_list="liaobiao1")
#          统一负责, target1 只重塑返回 shape, 这里不再直接 import append_liaobiao1。
from main.arm.each_task.task6 import wenzishibie as _task6_wenzishibie  # noqa: E402


# ---------- 默认参数 ----------

LOG_PREFIX: str = "[task6/target1]"

# ==== Phase 1: 底盘后退 ====
DEFAULT_BACK_MM: float = 90.0
"""Phase 1 后退距离 (mm)。用户 2026-08-04 指定 9cm = 90mm。
CLI 接收正值, 内部转负号 → move_for x_m = -dist_mm/1000 (后退)。
负值会被 ``abs()`` 强制取正再取负, 避免误传前进。

⚠️ v2 改动记录: 底盘距离未变。"""

DEFAULT_CHASSIS_TIMEOUT_S: float = 10.0
"""底盘 move_for HTTP 同步超时兜底 (秒)。
90mm / 0.10 m/s ≈ 0.9s; 脚本按 max(5.0, |dist|/vel + 2.0) 自适应放大,
用户 ``--timeout`` 是这个自适应值的下限。"""

DEFAULT_CHASSIS_VELOCITY_MS: float = 0.10
"""底盘最大线速度 (m/s), 与 task7/dipan.py / position*.py 默认一致。"""

# ==== Phase 2: 4 步纯臂序列 (用户 2026-08-04 v2 指定) ====
POS_Y_MM: float = -143.0
"""Phase 2.0: y 抬到 -143mm (距业务硬限上界 -200 还有 57mm 余量)。

⚠️ v2 改动: 旧值 -200 顶上限 0mm 余量太危险, v2 改 -143 留 57mm 余量。
⚠️ y ≤ -80 (保护区外) → 后续 set_arm_angle(-95°) / set_hand_angle(-55°) /
   move_x_with_split(-121) 全都安全, 不会被 _check_safe 拦截。"""

POS_ARM_DEG: float = -95.0
"""Phase 2.1: 大臂到 -95° (推牌姿势, 业务硬限 [+90, -150]° 内)。

⚠️ v2 改动: 数值不变。
⚠️ arm=-95 距上界 +90 还有 185° (远), 距下界 -150 还有 55° (安全)。
⚠️ 大扭矩动作, 默认 speed=80 (脚本走 runner.set_arm_angle 默认); 现场
   可调 speed (见 runner.set_arm_angle 文档)。"""

POS_HAND_DEG: float = -55.0
"""Phase 2.2: 手爪到 -55° (推杆姿态, 业务硬限 [-90, 0]° 内)。

⚠️ v2 改动: 数值不变。
⚠️ 手爪 -55° 不是 UP (-90°) 也不是 DOWN (0°), 是"推杆扫牌"中间位,
   跟 task6/tuigan.py 的 PUSH_BAR_HAND_DEG=-55° 同款。
⚠️ 必须走 ``client.set_hand_angle(angle, speed, timeout=...)``
   (ArmRunner 没有 set_hand_angle, timeout 必填位置参)。
   见 [[armrunner-set-hand-angle-gotcha]]"""

POS_X_TARGET_MM: float = -121.0
"""Phase 2.3: x 滑到位置 1 (-121mm)。

⚠️ v2 改动: 旧值 -211 → 新值 -121, 距下界 -320 还有 199mm 余量。
⚠️ 走 move_x_with_split (belt-slip / wall_hit / overshoot 检测),
   兜底 ARM_API §9.1。"""

ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度, 默认 80。与 task6/tuigan.py、task7/*.py 一致。"""


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


# ---------- Phase 3: 文字识别 + 写入 liaobiao1 (2026-08-04 新增) ----------

def _ocr_append_to_liaobiao1(log_prefix: str) -> dict:
    """Phase 3: 调 ``_task6_wenzishibie.run(target_list="liaobiao1")`` → 自动入库。

    v6 简化 (2026-08-04): 不再自己做 ``_validate_order_result`` + ``append_liaobiao1``,
        改让 wenzishibie.run() 统一负责"识别 + 校验 + 入库", 我们只重塑返回 shape。
        调用方无感: target1/run() 的 Phase 3 拿到的 dict 形状完全不变。

    失败语义 (与 v5 一致): 拉帧 / 网络 / 校验失败 → wenzishibie **不写入**, 这里 **不抛**。

    Args:
        log_prefix: 打印前缀, 例如 ``"  [task6/target1] phase3"``。

    Returns:
        ``{"ok": bool, "name": str|None, "goods": str|None,
           "record": dict|None, "raw": dict, "error": str|None, "warnings": list}``
        - ok=True 时 record 为 wenzishibie 已自动 append 的 record (从 appended_record)
        - ok=False 时 record=None
    """
    print(f"  {log_prefix} 调 wenzishibie.run(target_list='liaobiao1') ... "
          f"(cam2 + ERNIE + 自动入库)")
    try:
        ocr = _task6_wenzishibie.run(target_list="liaobiao1")
    except Exception as exc:                                # noqa: BLE001
        msg = f"wenzishibie.run() 异常: {exc!r}"
        print(f"  ⚠️ {log_prefix} {msg}")
        return {"ok": False, "name": None, "goods": None,
                "record": None, "raw": {}, "error": msg, "warnings": []}

    if not isinstance(ocr, dict) or not ocr.get("ok"):
        err = (ocr.get("error") if isinstance(ocr, dict) else None) or "ERNIE 调用失败"
        print(f"  ⚠️ {log_prefix} ERNIE 调用失败, 不写入: {err}")
        return {"ok": False, "name": None, "goods": None,
                "record": None,
                "raw": ocr if isinstance(ocr, dict) else {},
                "error": err, "warnings": []}

    validated = ocr.get("validated") or {}
    warnings = validated.get("warnings") or []

    # v5 加: 软警告 (黑名单命中/已归一化) 透传打印, 不影响 valid
    for w in warnings:
        print(f"  ⚠️ {log_prefix} 软警告: {w}")

    if not validated.get("valid"):
        # 校验失败: wenzishibie 已不写入
        errs = "; ".join(validated.get("errors", []) or ["校验失败"])
        print(f"  ⚠️ {log_prefix} 校验失败, 不写入: {errs}")
        return {"ok": False, "name": None, "goods": None,
                "record": None, "raw": ocr,
                "error": errs, "warnings": warnings}

    # 校验通过: wenzishibie 已自动 append_liaobiao1, 取出 result
    name = validated["name"]
    goods = validated["goods"]
    record = ocr.get("appended_record")
    appended_to = ocr.get("appended_to")
    print(f"  ✅ {log_prefix} OCR 成功 → wenzishibie 已 append {appended_to!r}: "
          f"人名={name!r} 蔬菜={goods!r}")
    return {"ok": True, "name": name, "goods": goods,
            "record": record, "raw": ocr, "error": None,
            "warnings": warnings}


# ---------- 主流程 ----------

def run(client: ArmClient, runner: ArmRunner,
        back_mm: float = DEFAULT_BACK_MM,
        y_mm: float = POS_Y_MM,
        arm_deg: float = POS_ARM_DEG,
        hand_deg: float = POS_HAND_DEG,
        x_target_mm: float = POS_X_TARGET_MM,
        max_velocity_ms: float = DEFAULT_CHASSIS_VELOCITY_MS,
        timeout: float = DEFAULT_CHASSIS_TIMEOUT_S) -> dict:
    """3 阶段执行: 底盘后退 9cm → 4 步纯臂 (y → arm → hand → x) → 文字识别写 liaobiao1。

    Args:
        client: ArmClient (move_x_with_split + set_hand_angle + http.execute_car_action)
        runner: ArmRunner (move_y + set_arm_angle)
        back_mm: Phase 1 后退距离 (mm, 正值, 默认 90)
        y_mm: Phase 2.0 y 抬高目标 (mm, 默认 -143, 距硬限上界 -200 留 57mm 余量)
        arm_deg: Phase 2.1 大臂角度 (°, 默认 -95)
        hand_deg: Phase 2.2 手爪角度 (°, 默认 -55)
        x_target_mm: Phase 2.3 x 目标位置 (mm, 默认 -121)
        max_velocity_ms: 底盘限速 (m/s, 默认 0.10)
        timeout: 底盘 move_for HTTP 同步超时下限 (秒, 默认 10)

    Returns:
        {
            "ok": True,
            "back_mm": float,
            "y_mm": float,
            "arm_deg": float,
            "hand_deg": float,
            "x_target_mm": float,
            "back_job": dict,              # Phase 1 job
            "x_result": dict,              # Phase 2.3 split 结果
            "ocr_result": dict,            # Phase 3 OCR + liaobiao1 写入结果
                                          # {"ok", "name"|None, "goods"|None,
                                          #  "record"|None, "raw", "error"|None}
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (底盘后退 {DEFAULT_BACK_MM:.0f}mm → 4 步臂) ==========")
    print(f"  Phase 1: 底盘后退 {back_mm:.0f}mm")
    print(f"  Phase 2: y({y_mm:.0f}) → arm({arm_deg:.0f}°) → hand({hand_deg:.0f}°) → x({x_target_mm:.0f}mm)")

    # ==== Phase 1: 底盘后退 (底盘) ====
    # 强制转负: 即便用户传了 --back -50 (误) 也变成 -90 (后退), 避免误前进撞墙。
    back_signed_mm = -abs(back_mm)
    back_timeout = max(timeout, 5.0, abs(back_signed_mm) / 1000.0 / max(max_velocity_ms, 0.01) + 2.0)
    print(f"\n  ── Phase 1: 底盘后退 {abs(back_signed_mm):.0f}mm ──")
    back_job = _chassis_move_for(
        client, back_signed_mm,
        max_velocity_ms, back_timeout,
        log_prefix=f"  {LOG_PREFIX} phase1",
    )

    # ==== Phase 2: 4 步纯臂 (底盘不动, 顺序固定 y → arm → hand → x) ====
    print(f"\n  ── Phase 2: 4 步纯臂序列 ──")

    # 2.0 y 抬高 (出保护区, 距上限 57mm 余量, 不再顶硬限)
    print(f"  [2.0] move_y({y_mm}mm)              y → 留 57mm 余量")
    runner.move_y(y_mm, verify=True)

    # 2.1 大臂到 -95° (保护区 y={y_mm}mm 外, 业务硬限内)
    print(f"  [2.1] set_arm_angle({arm_deg}°)         大臂到推牌姿势")
    runner.set_arm_angle(arm_deg, speed=ANGLE_SPEED)

    # 2.2 手爪到 -55° (保护区 y={y_mm}mm 外, 业务硬限内, 走 client 不是 runner)
    # ⚠️ ArmRunner 没有 set_hand_angle (只有 set_storage), 必须走 client.set_hand_angle,
    #    且 timeout 是必填位置参 (与 set_arm_angle 不同)。见 [[armrunner-set-hand-angle-gotcha]]
    print(f"  [2.2] set_hand_angle({hand_deg}°)        手爪到推杆姿态")
    client.set_hand_angle(
        hand_deg, speed=ANGLE_SPEED,
        timeout=runner.default_timeout_s,
    )

    # 2.3 x 滑到位置 1 (-121mm, y={y_mm}mm 时保护区外)
    print(f"  [2.3] move_x_with_split({x_target_mm}mm)  x → 位置 1 (split 兜底)")
    x_result = move_x_with_split(
        client, runner, x_target_mm,
        log_prefix=f"  {LOG_PREFIX} phase2.3",
    )

    dt = time.time() - t0
    print(f"\n========== {LOG_PREFIX} Phase 2 完成 ({dt:.2f}s) ==========")
    print(f"  终态: 底盘后退 {abs(back_signed_mm):.0f}mm + "
          f"y={y_mm:.0f}mm + arm={arm_deg:.0f}° + hand={hand_deg:.0f}° + x={x_target_mm:.0f}mm")
    print(f"  ⚠️ y=-143 距上限 -200 留 57mm 余量, 后续 move_y 仍可微调。")
    print(f"     需要继续操作可直接调 runner.move_y(...) (走保护区外路径)。")

    # ===== Phase 3: 文字识别 + 写入 liaobiao1 (2026-08-04 新增) =====
    # 调 wenzishibie.run() (cam2 + ERNIE) → 校验通过 → append_liaobiao1(name, goods)。
    # ⚠️ OCR 通常耗时 5-30s (含 5 次重试 × 2s 间隔), 不算超时, 业务层接受等待。
    # ⚠️ OCR 失败 (拉帧 / 网络 / 校验) 不抛异常, 只 log warn + 不写入。
    print(f"\n  ── Phase 3: 文字识别 → 写入 liaobiao1 ──")
    ocr_result = _ocr_append_to_liaobiao1(log_prefix=f"  {LOG_PREFIX} phase3")
    print(f"  Phase 3 结果: ok={ocr_result['ok']}  "
          f"name={ocr_result['name']!r}  goods={ocr_result['goods']!r}  "
          f"error={ocr_result['error']!r}")

    dt_total = time.time() - t0
    print(f"\n========== {LOG_PREFIX} 完成 ({dt_total:.2f}s, 含 Phase 3 OCR) ==========\n")

    return {
        "ok": True,
        "back_mm": back_mm,
        "y_mm": y_mm,
        "arm_deg": arm_deg,
        "hand_deg": hand_deg,
        "x_target_mm": x_target_mm,
        "back_job": back_job,
        "x_result": x_result,
        "ocr_result": ocr_result,                   # Phase 3 新增: {"ok", "name", "goods", "record", "raw", "error"}
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: --back / --y / --arm / --hand / --x-target / --vel / --timeout。

    --back 接收正 mm (用户语义), 内部自动转负号 → move_for x_m = -|back|/1000。
    即便用户传负值也会被 abs() 强制取正再取负, 避免误传前进撞墙。
    """
    p = argparse.ArgumentParser(
        description=(
            "task6 target1 v2: 底盘后退 9cm → y(-143) → arm(-95°) → hand(-55°) → x(-121mm)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--back", type=float, default=DEFAULT_BACK_MM,
                   help="Phase 1 后退距离 (mm, 默认 90 = 9cm, 强制正值)")
    p.add_argument("--y", type=float, default=POS_Y_MM,
                   help="Phase 2.0 y 抬高目标 (mm, 默认 -143, 距硬限上界 -200 留 57mm 余量)")
    p.add_argument("--arm", type=float, default=POS_ARM_DEG,
                   help="Phase 2.1 大臂角度 (°, 默认 -95)")
    p.add_argument("--hand", type=float, default=POS_HAND_DEG,
                   help="Phase 2.2 手爪角度 (°, 默认 -55=推杆姿态)")
    p.add_argument("--x-target", type=float, default=POS_X_TARGET_MM,
                   dest="x_target",
                   help="Phase 2.3 x 目标位置 (mm, 默认 -121, 必须在 [-320, +220])")
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
        y_mm=args.y,
        arm_deg=args.arm,
        hand_deg=args.hand,
        x_target_mm=args.x_target,
        max_velocity_ms=args.max_velocity,
        timeout=args.timeout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
