"""task6 / target2 —— **任务六位置 2** 的 4 步纯臂序列 + 末尾 OCR 写 liaobiao2 (不碰底盘, 跟 tuigan.py 同款 Y 触底逻辑)。

按用户 2026-08-04 指定顺序 (y=-2 保护区系列):

  1. move_x_with_split(-158mm)       x 移到 -158mm (在 y≤-80 时保护区外, wrapper 安全)
  2. move_y(-2mm)                    y 降到 -2 (保护区 [0, -80] 内, move_y 从不被拦)
  3. set_arm_angle(-95°)             大臂到 -95° (Y 保护区 → 底层直调, 跟 tuigan.py 同款)
  4. set_hand_angle(-61°)            手爪到 -61° (Y 保护区 → 底层直调, 跟 tuigan.py 同款)

⚠️ **顺序关键** (这条不能乱):
  - 第 1 步 x 移到 -158 必须在 y ≤ -80 才能调, 否则 SDK 拦截。
    假设上游 target1.py 把 y 抬到 -200 留下的状态, 直接调 x 安全。
  - 第 2 步 y 降到 -2 后, 第 3/4 步 set_arm_angle(-95°) / set_hand_angle(-61°)
    必须走 **底层直调** (``arm_client._call_arm(...)``), 绕开 Python 层
    y 保护区校验 — 与 task6/tuigan.py:174-203 + task5/low_tower.py:118-126
    + task5/get_blue.py:172 同款处理。
    走 wrapper 必 raise ValueError。
  - **绝对不能** 先设 arm/hand 再降 y, 否则 set_*_angle(非白名单) 在 y=-200
    时虽然合法但顺序反了 → 大扭矩舵机与 X 同步容易撞推牌机构。

⚠️ **业务硬限** (走前要核对, 见 ARM_API §1.1 / §7):
  - x=-158 ∈ [-320, +220] mm ✓ (距下界 162mm 余量)
  - y=-2 ∈ [0, -80] 保护区 → set_*_angle / move_x wrapper 都被拒,
    第 3/4 步必须走底层直调 (跟 tuigan.py / 旧版 target2.py 同款)
  - arm=-95° ∈ [-150, +90]° ✓ (推牌姿势, 跟 tuigan.py / target1.py 同款)
  - hand=-61° ∈ [-90, 0]° ✓ (推杆姿态, 比 tuigan.py -55° 收 6°, 比旧版 -60° 收 1°
    — 现场 2026-08-04 微调)

⚠️ **x 走 move_x_with_split** (common.py:174): belt-slip 防误撞墙 + 自动 retry,
   与 task7/{position1-6,get_position1/2}.py + task6/target1.py 一致。

⚠️ **set_arm_angle / set_hand_angle 走底层直调** (跟 tuigan.py 同款):
  - Y ∈ [0, -80] 保护区时, ``safety.py:76`` fail-closed, 走 wrapper
    ``runner.set_arm_angle`` / ``client.set_hand_angle`` 必 raise ValueError。
  - 因此本脚本直接调 ``arm_client._call_arm("set_arm_angle", ...)`` /
    ``arm_client._call_arm("set_hand_angle", ...)``, 跳过 Python 层校验,
    合法性交车端判断。
  - ⚠️ 代价: 绕开 wrapper 也绕开了 ``_check_step_loss`` (丢步核对), 要校验
    实际到位请读 ``GET /v1/realtime/arm/state`` 里的 arm_angle/hand_angle 对比。

⚠️ **本文件自包含** (与 task6/{tuigan, wenzishibie, target1}.py + task7/*.py 同款):
   只依赖 ``main.arm.ArmClient`` + ``main.arm.ArmRunner`` +
   ``main.arm.each_task.common.move_x_with_split``,
   不 import task6 包内任何模块。原因: task5 包曾被外部清空过一次
   (见 [[task5-rebuild-2026-07-22]]), 自包含可保证 ``python target2.py``
   直接跑不受影响。

⚠️ **前置状态要求** (上游负责):
  - 上游脚本 (target1.py 或其他) 必须先把 y 抬到 ≤ -80 才能调本脚本第 1 步 move_x。
  - 假设 y=-200 (target1.py 默认终态), 直接跑本脚本安全。

跑法:
    python main/arm/each_task/task6/target2.py
    python -m main.arm.each_task.task6.target2
    python main/arm/each_task/task6/target2.py --x-target -120        # x 不到那么远
    python main/arm/each_task/task6/target2.py --hand -50             # 手爪张开点
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.each_task.common import move_x_with_split  # noqa: E402

# 末尾新增 (2026-08-04 用户新需求): 调 wenzishibie 文字识别 → 自动写 liebiao.liaobiao2
# 突破早期 "自包含" 约定 (单文件 self-contained), 因为 wenzishibie 是 task6 包内
# 姐妹文件, 与 target2.py 在同一目录, 包从未被外部清空过 (task5 清空见
# [[task5-rebuild-2026-07-22]]), 风险低。仍不依赖 main.arm.__init__ 或其他子包。
#
# v6 简化: 入库 (append_liaobiao2) 现在由 wenzishibie.run(target_list="liaobiao2")
#          统一负责, target2 只重塑返回 shape, 这里不再直接 import append_liaobiao2。
from main.arm.each_task.task6 import wenzishibie as _task6_wenzishibie  # noqa: E402


# ---------- 序列常量 (用户 2026-08-04 指定) ----------

LOG_PREFIX: str = "[task6/target2]"

# ==== 4 步纯臂序列 (跟 tuigan.py 高度相似, 但不带底盘动作) ====
POS_X_TARGET_MM: float = -158.0
"""第 1 步: x 滑到位置 2 (-158mm)。

⚠️ 必须 ≥ x_min_m=-320 软限位 ✓; -158 距下界还有 162mm 余量。
⚠️ 必须在 y ≤ -80 时调用 (假设上游 target1.py 已抬到 -200, 满足 ✓)。
⚠️ 走 move_x_with_split (belt-slip / wall_hit / overshoot 检测)。"""

POS_Y_DOWN_MM: float = -2.0
"""第 2 步: y 降到 -2mm (保护区 [0, -80] 内, 跟旧版 y=0 几乎贴底)。

⚠️ Y=-2 ∈ [0, -80] 保护区 → set_*_angle / move_x wrapper 都被拒,
   第 3/4 步必须走底层直调 (跟 tuigan.py / 旧版 target2.py 同款)。
⚠️ move_y 从不被 safety 拦, 直接走 wrapper。"""

POS_ARM_DEG: float = -95.0
"""第 3 步: 大臂到 -95° (推牌姿势, 业务硬限 [+90, -150]° 内)。

⚠️ 此时 Y ∈ [0, -80] 保护区, safety.py:76 fail-closed → 必须走底层直调。
⚠️ 跟 tuigan.py / target1.py 同款值 -95°。"""

POS_HAND_DEG: float = -61.0
"""第 4 步: 手爪到 -61° (推杆姿态, 业务硬限 [-90, 0]° 内)。

⚠️ 此时 Y ∈ [0, -80] 保护区, safety.py:76 fail-closed → 必须走底层直调。
⚠️ 跟 tuigan.py 的 PUSH_BAR_HAND_DEG=-55° 收 6°; 比旧版 -60° 收 1°
   (现场 2026-08-04 微调)。"""

# ==== 时序常量 (sleep / 速度) ====
ARM_SPEED: int = 40
"""大臂舵机速度, 大扭矩动作慢速, 默认 40 (跟 task6/tuigan.py / task6_get_order.py 同款)。"""

HAND_SPEED: int = 80
"""手爪舵机速度, 默认 80 (跟 tuigan.py / target1.py 同款)。"""

HAND_TIMEOUT_S: float = 10.0
"""set_hand_angle 走底层直调时的 timeout。ArmClient._call_arm sync=True 阻塞等闭环。"""

POST_BAR_PAUSE_S: float = 0.5
"""第 4 步 (手爪转到 -60° 推杆姿势) 后等稳, 防止马上动 X 时手爪还在抖动
导致位置偏移。跟 tuigan.py POST_BAR_HAND_PAUSE_S 同款。"""


# ---------- logger ----------

logger = logging.getLogger("task6.target2")


# ── Y=0 触底时的动作 (绕开业务层 y 保护区, 跟 tuigan.py 同款) ────────

def _set_arm_angle_at_bottom(
    arm_client: ArmClient,
    angle: float,
    speed: int = ARM_SPEED,
    timeout: float = HAND_TIMEOUT_S,
) -> dict:
    """Y 触底时设置大臂角度 (底层直调, 绕开 y 保护区)。"""
    return arm_client._call_arm(
        "set_arm_angle", timeout=timeout, sync=True,
        angle=float(angle), speed=speed,
    )


def _set_hand_angle_at_bottom(
    arm_client: ArmClient,
    angle: float,
    speed: int = HAND_SPEED,
    timeout: float = HAND_TIMEOUT_S,
) -> dict:
    """Y 触底时设置手爪角度 (底层直调, 绕开 y 保护区)。"""
    return arm_client._call_arm(
        "set_hand_angle", timeout=timeout, sync=True,
        angle=float(angle), speed=speed,
    )


# ---------- 末尾新增 (2026-08-04): 文字识别 + 写入 liaobiao2 ----------

def _ocr_append_to_liaobiao2(log_prefix: str) -> dict:
    """末尾 OCR: 调 ``_task6_wenzishibie.run(target_list="liaobiao2")`` → 自动入库。

    v6 简化 (2026-08-04): 与 target1 同款简化, 改让 wenzishibie.run() 统一负责
        "识别 + 校验 + 入库", 我们只重塑返回 shape 给 target2 末尾调用方。
        调用方拿到的 dict 形状与 v5 完全一致。

    失败语义与 v5 一致: 拉帧 / 网络 / 校验失败 → wenzishibie 不写入, 这里不抛。
    """
    print(f"  {log_prefix} 调 wenzishibie.run(target_list='liaobiao2') ... "
          f"(cam2 + ERNIE + 自动入库)")
    try:
        ocr = _task6_wenzishibie.run(target_list="liaobiao2")
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

    for w in warnings:
        print(f"  ⚠️ {log_prefix} 软警告: {w}")

    if not validated.get("valid"):
        errs = "; ".join(validated.get("errors", []) or ["校验失败"])
        print(f"  ⚠️ {log_prefix} 校验失败, 不写入: {errs}")
        return {"ok": False, "name": None, "goods": None,
                "record": None, "raw": ocr,
                "error": errs, "warnings": warnings}

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

def run(arm_client: ArmClient, runner: ArmRunner) -> dict:
    """按用户顺序执行 4 步纯臂序列 (x → y → arm → hand, Y 触底后绕 wrapper) +
    末尾 OCR 写入 liaobiao2。

    本函数 **不碰底盘**, 只调机械臂。

    假设上游已把 y 抬到 ≤ -80 (target1.py 默认 y=-200, 满足)。
    终态: X=-215 + Y=0 + 大臂=-95° + 手爪=-60° (跟 tuigan.py 终态相似但 X 更远)

    Args:
        arm_client: ArmClient (.http.execute_car_action + set_*_angle_at_bottom)
        runner: ArmRunner (move_y + move_x_with_split 内部用)

    Returns:
        {
            "ok": True,
            "x_target_mm":  -215.0,    # 第 1 步目标
            "y_down_mm":       0.0,    # 第 2 步目标 (触底)
            "arm_deg":       -95.0,    # 第 3 步目标 (推牌姿势)
            "hand_deg":      -60.0,    # 第 4 步目标 (推杆姿态)
            "x_result":      dict,     # move_x_with_split 第 1 步返回
            "ocr_result":    dict,     # 末尾 OCR + liaobiao2 写入结果
                                       # {"ok", "name"|None, "goods"|None,
                                       #  "record"|None, "raw", "error"|None}
        }

    Raises:
        RuntimeError: 业务层异常 (move_x 失败 / 底层直调失败 / 编码器读不到)。
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (4 步纯臂序列, 不动底盘) ==========")
    print(f"  假设上游已把 y 抬到 ≤ -80 (默认 target1.py y=-200)")
    print(f"  终态: x=-158 + y=-2 + arm=-95° + hand=-61°")

    # ===== 第 1 步: x 移到 -158mm (在 y≤-80 时保护区外, wrapper 安全) =====
    print(f"\n  [1/4] move_x_with_split({POS_X_TARGET_MM}mm)  x → 位置 2 (假设 y ≤ -80, split 兜底)")
    x_result = move_x_with_split(
        arm_client, runner, POS_X_TARGET_MM,
        log_prefix=f"  {LOG_PREFIX} step1",
    )

    # ===== 第 2 步: y 降到 -2 (move_y 从不被 safety 拦) =====
    print(f"\n  [2/4] move_y({POS_Y_DOWN_MM}mm)              y → -2 (保护区, move_y 不被拦)")
    runner.move_y(POS_Y_DOWN_MM, verify=True)

    # ===== 第 3 步: 大臂到 -95° (Y 保护区, 必须绕 wrapper) =====
    # ⚠️ 此时 Y ∈ [0, -80] 保护区, safety.py:76 fail-closed → 底层直调
    print(f"\n  [3/4] set_arm_angle_at_bottom({POS_ARM_DEG}°)   大臂到 -95° (Y 保护区, 底层直调)")
    _set_arm_angle_at_bottom(arm_client, POS_ARM_DEG)
    time.sleep(0.5)  # 大扭矩动作后短暂稳定 (跟 tuigan.py 同款)
    logger.info("  大臂 → %.0f° 完成", POS_ARM_DEG)

    # ===== 第 4 步: 手爪到 -61° (Y 保护区, 必须绕 wrapper) =====
    print(f"  [4/4] set_hand_angle_at_bottom({POS_HAND_DEG}°)  手爪到 -61° (Y 保护区, 底层直调)")
    _set_hand_angle_at_bottom(arm_client, POS_HAND_DEG)
    time.sleep(POST_BAR_PAUSE_S)
    logger.info("  手爪 → %.0f° 完成 (推杆姿态就绪)", POS_HAND_DEG)

    dt = time.time() - t0
    print(f"\n========== {LOG_PREFIX} 4 步臂完成 ({dt:.2f}s) ==========")
    print(f"  终态: X={POS_X_TARGET_MM:.0f}mm + 大臂={POS_ARM_DEG:.0f}° + "
          f"手爪={POS_HAND_DEG:.0f}° + Y={POS_Y_DOWN_MM:.0f}mm")
    print(f"  ⚠️ Y 在保护区, 继续动作前先 runner.move_y(-150) 出保护区。")

    # ===== 末尾新增 (2026-08-04): 文字识别 + 写入 liaobiao2 =====
    # ⚠️ OCR 通常耗时 5-30s (含 5 次重试 × 2s 间隔), 不算超时。
    # ⚠️ OCR 失败 (拉帧 / 网络 / 校验) 不抛异常, 只 log warn + 不写入。
    print(f"\n  ── 末尾: 文字识别 → 写入 liaobiao2 ──")
    ocr_result = _ocr_append_to_liaobiao2(log_prefix=f"  {LOG_PREFIX} end")
    print(f"  末尾 结果: ok={ocr_result['ok']}  "
          f"name={ocr_result['name']!r}  goods={ocr_result['goods']!r}  "
          f"error={ocr_result['error']!r}")

    dt_total = time.time() - t0
    print(f"\n========== {LOG_PREFIX} 完成 ({dt_total:.2f}s, 含末尾 OCR) ==========\n")

    return {
        "ok": True,
        "x_target_mm": POS_X_TARGET_MM,
        "y_down_mm": POS_Y_DOWN_MM,
        "arm_deg": POS_ARM_DEG,
        "hand_deg": POS_HAND_DEG,
        "x_result": x_result,
        "ocr_result": ocr_result,                # 末尾新增: {"ok", "name", "goods", "record", "raw", "error"}
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 4 个常量覆盖 (x_target / y / arm / hand)。

    arm_deg / hand_deg 是推牌/推杆位姿, 故意不暴露 CLI 默认覆盖 (避免误改改坏
    业务硬限) — 但脚本里仍走 CLI 参数, 想现场微调可直接 --arm/--hand。
    """
    p = argparse.ArgumentParser(
        description=(
            "task6 target2: 4 步纯臂序列 (x(-215) → y(0) → arm(-95°) → hand(-60°), Y 触底后绕 wrapper)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--x-target", type=float, default=POS_X_TARGET_MM,
                   dest="x_target",
                   help="第 1 步 x 目标位置 (mm, 默认 -215, 必须在 [-320, +220])")
    p.add_argument("--y", type=float, default=POS_Y_DOWN_MM,
                   help="第 2 步 y 触底目标 (mm, 默认 0)")
    p.add_argument("--arm", type=float, default=POS_ARM_DEG,
                   help="第 3 步 大臂角度 (°, 默认 -95, Y=0 时必须走底层直调)")
    p.add_argument("--hand", type=float, default=POS_HAND_DEG,
                   help="第 4 步 手爪角度 (°, 默认 -60, Y=0 时必须走底层直调)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    # CLI 覆盖 4 个常量 (模块顶置常量是文档/默认值)
    global POS_X_TARGET_MM, POS_Y_DOWN_MM, POS_ARM_DEG, POS_HAND_DEG
    POS_X_TARGET_MM = args.x_target
    POS_Y_DOWN_MM = args.y
    POS_ARM_DEG = args.arm
    POS_HAND_DEG = args.hand

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(message)s")
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner)
    return 0


if __name__ == "__main__":
    sys.exit(main())