"""task6 / tuigan —— **任务六推杆部分** 自包含版本 (不动底盘, 不做识别/抓菜)。

本源文件 **完全复制** 自 ``main/task/task6_get_order.py`` 的 ``_enter_push_bar_pose``
函数 + 两个 ``_at_bottom`` helper。删掉了原文件的:
  - 阶段 3  LLM × 2 轮读单 (``order_read_run``)
  - 阶段 4  视觉识别 + 抓菜 + 投放 (``veggie_detect_run`` + ``_pick_one_veggie``)
  - 阶段 5  运送待命姿态 (stub)
  - `_enter_push_bar_pose` 的 **第三部分 "调整到读单姿态"** (f-j 5 步,
    是为后续识别/读单做的姿态准备, 跟"识别"绑定, 故一并删除)
  - `_load_task6_config` 加载
  - `_chassis_move_for` 底盘移动 helper

⚠️ **不再包含第三部分"调整到读单姿态"**:
  用户 2026-08-04 明确"只要推杆部分, 不用识别部分"。
  原 task6_get_order.py 里的第三部分 (f-j 5 步: X→-150 + Y→-80 +
  arm→-85° + hand→-55° + X→-140) 是为后续 cam2 拍订单牌 + LLM OCR
  做姿态准备 — 跟"识别"绑定, 故一并删除。后续做识别请另外开脚本
  调 cam2 + order_read_run()。

**保留的部分** = **推杆本身 (两部分, 共 7 步)**:

  ===== 第一部分: 推杆姿态准备 (X→-200 → arm→-95° → hand→-90° → Y→0 → hand→-55°) =====
    步骤 0  (软抢答) runner.move_y(-150)   仅当 cur_y > -30mm (保护区残留时) 抬出保护区
    步骤 a  runner.move_x(-200)
    步骤 b  runner.set_arm_angle(-95°, speed=40) + sleep(2.0)
    步骤 c  arm_client.set_hand_angle(-90°, speed=80, timeout=10) + sleep(1.0)
    步骤 d  runner.move_y(0)
    步骤 e  _set_hand_angle_at_bottom(-55°) (Y 触底, 底层直调) + sleep(0.5)

  ===== 第二部分: 扫牌 (Y 保持 0, **最终终态**) =====
    步骤 6  _move_x_at_bottom(-120, v_max_mms=100)  Y 触底, 底层直调

⚠️ **动作顺序硬约束 (C1, 见 task6_get_order.py docstring L19-21)**:
   第一部分必须 **X → arm(+2s) → hand(+1s) → Y** 顺序执行, 中间 sleep 不能省。
   否则多个大电流舵机同时启动会触发硬件保护, Jetson 瞬时掉电 / cam2 USB 断连。

⚠️ **Y 触底保护区 (C2)**:
   Y=0 (> -30mm) 时, ``safety.py:76`` fail-closed, ``set_*_angle`` / ``move_x``
   走 wrapper 必 raise。所以第二部分 (扫牌) 走 **底层直调** ``arm_client._call_arm(...)``,
   合法性交车端判断 —— 跟原 ``_set_hand_angle_at_bottom`` / ``_move_x_at_bottom``
   实现一致。
   代价: 绕过 wrapper = 绕过丢步核对; 要校验实际到位请自行读
   ``GET /v1/realtime/arm/state`` 里的 x_mm 对比 (见 ARM_API §1.1)。

⚠️ **本文件自包含** (与 task7/{position1,position2,get_position1,get_position2}.py
   同款): 只依赖 ``main.arm.ArmClient`` + ``main.arm.ArmRunner``,
   不 import task6 包内任何模块。原因: task5/target.py 等曾被外部清空过
   (见 [[task5-rebuild-2026-07-22]]), 自包含可保证 ``python tuigan.py``
   直接跑不受影响。

⚠️ **参数全部顶置到模块常量** (跟原 ``_load_task6_config()`` 走 yaml 不同 —
   这里走纯 Python 默认值)。CLI 暴露各常量可选覆盖, 方便现场调参。

⚠️ **终态**: 跑完二部分后, 状态 = X=-120mm + 大臂=-95° + 手爪=-55° + Y=0。
   这个终态下能继续做的事: ``move_x_with_split`` / 调底盘 / 起新任务。
   不能继续做的事: ``set_*_angle`` / ``move_y`` 走 wrapper (Y=0 保护区拦);
   要做这些必须先 ``runner.move_y(-150)`` 出保护区。

跑法:
    python main/arm/each_task/task6/tuigan.py
    python -m main.arm.each_task.task6.tuigan
    python main/arm/each_task/task6/tuigan.py --push-x -250 --sweep-end -100
    python main/arm/each_task/task6/tuigan.py --sweep-speed 60     # 推慢一点
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


# ---------- 序列常量 (从 task6_get_order.py push_bar_pose 抽出来) ----------

LOG_PREFIX: str = "[task6/tuigan]"

# ==== 第一部分: 推杆姿态准备 ====
PUSH_X_MM: float = -200.0
"""第一部分步骤 a: X 收到 -200mm (PID 闭环, 业务 wrapper)。

⚠️ -200 ∈ [-320, +220] 软限位内 ✓, 距下界 -320 还有 120mm 余量, 横向足够。"""

PUSH_ARM_DEG: float = -95.0
"""第一部分步骤 b: 大臂到 -95° (推牌姿势, 业务硬限 [+90, -150]° 内)。

⚠️ 大扭矩动作, speed=40 慢, sleep 2.0s 等舵机稳 — 顺序硬约束 1b, 不能省。"""

PUSH_HAND_DEG: float = -90.0
"""第一部分步骤 c: 手爪到 -90° (UP / 复位位)。

⚠️ 此时 Y=-150 (步骤 0 抬的或外部已就位), 在保护区外 → wrapper 放行。"""

PUSH_Y_MM: float = 0.0
"""第一部分步骤 d: Y 下降到 0 (触底限位, 推牌时 Y 必须贴地)。

⚠️ 此时 set_hand(-90°) 走 wrapper 已 OK (wrapper 在 Y=-150 走完才到这步, 但
   set_hand_angle 在保护区 [0, -30] 内是允许 -90° 的)。然后步骤 e 才走 -55°
   (非 -90°, 这才需要绕开 wrapper)。"""

PUSH_BAR_HAND_DEG: float = -55.0
"""第一部分步骤 e: 手爪到 -55° (推杆姿态就绪, 手爪作为推牌杆)。

⚠️ 此时 Y 已 = 0 (> -30, 保护区里), ``safety.py:76`` 会拒绝 set_hand_angle。
   **必须** 走 ``_set_hand_angle_at_bottom`` 底层直调, 绕开 Python 层 y 校验。"""

PUSH_SAFE_Y_MM: float = -150.0
"""步骤 0 软抢答: 当检测到 cur_y > -30 时, 把 y 抬到这个值再继续第一部分。

⚠️ 防止上一轮任务 / 上一次崩溃把 y 留在触底附近, 此时 move_x 直接被 safety 拦。
   -150 ∈ 保护区外 ✓, -150 ≤ -80 (硬限外) ✓。"""

# ==== 第二部分: 扫牌 (最终终态) ====
SWEEP_X_END_MM: float = -120.0
"""第二部分步骤 6: X 从 PUSH_X_MM 扫到 SWEEP_X_END_MM (-120mm)。

⚠️ 80mm 行程, 推牌距离。-120 也在 [-320, +220] 范围内 ✓。
   sweep 时 Y 保持 0 — 因此 ``_move_x_at_bottom`` 必须走底层直调。"""

SWEEP_SPEED_MMS: float = 100.0
"""第二部分步骤 6: X 扫动速度, mm/s。

⚠️ 原 task6_get_order.py 之前只在日志里记 100mm/s 但实际跑默认 40mm/s, 现已
   补上 v_max_mms 真实透传给 SDK。100mm/s 是 PID 闭环驱动电机, belt-slip 状态
   下可能跑不到, 但任务六推牌场景不需要 mm 级精度。

   调慢可 ``--sweep-speed 60``, 跑满不会假撞墙, belt-slip 走满 seek_timeout
   超时返回 (见 ARM_API §9.1)。"""

# ==== 时序常量 (sleep / 速度) ====
ARM_SPEED_LOW: int = 40
"""大臂舵机速度, 大扭矩动作慢速, 默认 40 (跟原代码一致)。"""

HAND_SPEED_NORMAL: int = 80
"""手爪舵机速度, 默认 80 (跟原代码一致)。"""

HAND_TIMEOUT_S: float = 10.0
"""set_hand_angle 走 wrapper 时的显式 timeout (ArmRunner 没有 set_hand_angle,
走 ``arm_client.set_hand_angle``, timeout 是必填位置参, 见
[[armrunner-set-hand-angle-gotcha]])。"""

ARM_STABILIZE_S: float = 2.0
"""大臂转完后稳定等待时间 (第一部分)。原代码注释明示是顺序硬约束的
"硬约束 1b": 大扭矩动作 sleep 不能省, 否则后续舵机瞬时叠加电流会掉电。"""

HAND_STABILIZE_S: float = 1.0
"""手爪转完后稳定等待 (第一部分)。

⚠️ 跟 ARM_STABILIZE_S 一起构成"大电流动作顺序硬约束": 步骤 b 后 2s +
步骤 c 后 1s 共 3s 的稳定等待, 才能走步骤 d (Y 下降)。"""

POST_BAR_HAND_PAUSE_S: float = 0.5
"""步骤 e (手爪转到 -55° 推杆姿势) 后等稳, 防止马上开始扫牌时手爪还在抖动
导致扫到的推牌轨迹偏移。"""


# ---------- logger ----------

logger = logging.getLogger("task6.tuigan")


# ── Y=0 触底时的动作 (绕开业务层 y 保护区) ────────────────────────
#
# ⚠️ 完全照抄自 task6_get_order.py L106-152, 注释也照搬, 没改一行。
# ⚠️ 与 task5/low_tower.py:118-126 (手爪 0° DOWN) 和
#    task5/get_blue.py:172 同款处理。

def _set_hand_angle_at_bottom(
    arm_client: ArmClient,
    angle: float,
    speed: int = HAND_SPEED_NORMAL,
    timeout: float = HAND_TIMEOUT_S,
) -> dict:
    """Y 触底时设置手爪角度 (底层直调, 绕开 y 保护区)."""
    return arm_client._call_arm(
        "set_hand_angle", timeout=timeout, sync=True,
        angle=float(angle), speed=speed,
    )


def _move_x_at_bottom(
    arm_client: ArmClient,
    x_mm: float,
    v_max_mms: float = SWEEP_SPEED_MMS,
    out_time: float = 15.0,
    timeout: float = 30.0,
) -> dict:
    """Y 触底时移动 X (底层直调, 绕开 y 保护区).

    注意: 绕开 wrapper 也就绕开了 ``move_x`` 的丢步核对 (_check_step_loss),
    需要校验实际到位时请自行读 realtime x_mm 对比。
    """
    return arm_client._call_arm(
        "move_x_position", timeout=timeout, sync=True,
        target=float(x_mm) / 1000.0, out_time=out_time,
        v_max_mms=float(v_max_mms),
    )


# ---------- 主流程: 两部分 推杆+扫牌 ----------

def run(arm_client: ArmClient, runner: ArmRunner) -> dict:
    """等价 task6_get_order.py 的 ``_enter_push_bar_pose`` 的**前两部分** (推杆本身)。

    执行顺序严格按 C1 顺序硬约束:
      第一部分: 推杆姿态准备 (步骤 0 + a-e, 6 个动作)
      第二部分: 扫牌 (步骤 6, 1 个动作) ← **最终终态**

    ⚠️ **不再做第三部分"调整到读单姿态"** — 已删除 (见文件顶部 docstring)。

    Args:
        arm_client: ArmClient
        runner: ArmRunner

    Returns:
        {
            "ok": True,
            "phase1": {                  # 推杆姿态准备
                "x_mm": PUSH_X_MM,
                "arm_deg": PUSH_ARM_DEG,
                "hand_deg": PUSH_HAND_DEG,
                "y_mm": PUSH_Y_MM,
                "bar_hand_deg": PUSH_BAR_HAND_DEG,
            },
            "phase2": {                  # 扫牌 (终态)
                "x_end_mm": SWEEP_X_END_MM,
                "speed_mms": SWEEP_SPEED_MMS,
            },
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run ========")

    # ==== 第一部分: 推杆姿态准备 ====
    logger.info(
        "阶段 1: 推杆姿态准备 (x=%.0f arm=%.0f hand=%.0f y=%.0f)",
        PUSH_X_MM, PUSH_ARM_DEG, PUSH_HAND_DEG, PUSH_Y_MM,
    )

    # 第零步: 确保 y 在保护区外 (否则下面第一个 move_x 直接被拦)
    # 上一轮任务 / 上一次崩溃可能把 y 留在触底附近, 此时 move_x 会 raise。
    # move_y 本身从不被安全门拦, 可以无条件先抬。
    try:
        cur_y = float(arm_client.get_state().y_mm)
    except Exception:
        cur_y = 0.0
    if cur_y > -30.0:
        logger.info("  y=%.1fmm 在保护区内, 先抬到 %.0fmm 再动 X",
                    cur_y, PUSH_SAFE_Y_MM)
        runner.move_y(PUSH_SAFE_Y_MM)

    # a) X 轴: PID 闭环移动到位
    runner.move_x(PUSH_X_MM)
    logger.info("  X → %.0f mm 完成", PUSH_X_MM)

    # b) 大臂旋转 (大扭矩动作, 单独执行 + 长等待稳定)
    runner.set_arm_angle(PUSH_ARM_DEG, speed=ARM_SPEED_LOW)
    time.sleep(ARM_STABILIZE_S)
    logger.info("  大臂 → %.0f° 完成", PUSH_ARM_DEG)

    # c) 手爪旋转到 PUSH_HAND_DEG
    arm_client.set_hand_angle(PUSH_HAND_DEG, speed=HAND_SPEED_NORMAL,
                              timeout=HAND_TIMEOUT_S)
    time.sleep(HAND_STABILIZE_S)
    logger.info("  手爪 → %.0f° 完成", PUSH_HAND_DEG)

    # d) Y 轴: PID 闭环下降到底部限位 (必须最后执行 Y)
    runner.move_y(PUSH_Y_MM)
    logger.info("  Y → %.0f mm 完成 (扫动前 Y 已触底)", PUSH_Y_MM)

    # e) 手爪转到 PUSH_BAR_HAND_DEG (与推牌杆配合的推杆姿态)
    #    此时 Y 已触底 (=0), 走 wrapper 必被 y 保护区拒 → 底层直调
    _set_hand_angle_at_bottom(arm_client, PUSH_BAR_HAND_DEG)
    time.sleep(POST_BAR_HAND_PAUSE_S)
    logger.info("  手爪 → %.0f° 完成 (推杆姿态就绪)", PUSH_BAR_HAND_DEG)

    logger.info("推杆姿态就绪 (X=%.0f arm=%.0f hand=-45 Y=%.0f)",
                PUSH_X_MM, PUSH_ARM_DEG, PUSH_Y_MM)

    # ==== 第二部分: X 扫动推牌 (Y 保持触底, 推杆最后一步) ====
    logger.info(
        "阶段 2: X 扫动推牌 %.0f → %.0f @ %.0f mm/s (Y=%.0f) —— 最终终态",
        PUSH_X_MM, SWEEP_X_END_MM, SWEEP_SPEED_MMS, PUSH_Y_MM,
    )
    # Y 仍触底 (=0), 走 wrapper 必被 y 保护区拒 → 底层直调
    _move_x_at_bottom(arm_client, SWEEP_X_END_MM, v_max_mms=SWEEP_SPEED_MMS)
    logger.info("  扫动完成, X=%.0f mm", SWEEP_X_END_MM)

    logger.info("推杆 + 扫牌完成 (识别部分不在本任务范围, 另开脚本)")

    dt = time.time() - t0
    print(f"========== {LOG_PREFIX} 完成 ({dt:.2f}s) ========")
    print(f"  终态: X={SWEEP_X_END_MM:.0f}mm + 大臂={PUSH_ARM_DEG:.0f}° + "
          f"手爪={PUSH_BAR_HAND_DEG:.0f}° + Y={PUSH_Y_MM:.0f}mm")
    print(f"  ⚠️ Y 在保护区, 继续动作前先 runner.move_y(-150) 出保护区。\n")

    return {
        "ok": True,
        "phase1": {
            "x_mm": PUSH_X_MM,
            "arm_deg": PUSH_ARM_DEG,
            "hand_deg": PUSH_HAND_DEG,
            "y_mm": PUSH_Y_MM,
            "bar_hand_deg": PUSH_BAR_HAND_DEG,
        },
        "phase2": {
            "x_end_mm": SWEEP_X_END_MM,
            "speed_mms": SWEEP_SPEED_MMS,
        },
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 8 个常量覆盖, 全部走 argparse.ArgumentDefaultsHelpFormatter。

    参数命名跟模块常量名对齐:
      第一部分: --push-x / --push-arm / --push-hand / --push-y / --bar-hand / --safe-y
      第二部分: --sweep-end / --sweep-speed
    """
    p = argparse.ArgumentParser(
        description=(
            "task6 tuigan: **推杆部分** (姿态准备 + 扫牌), 不动底盘, 不做识别/读单/抓菜"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    # 第一部分
    p.add_argument("--push-x", type=float, default=PUSH_X_MM,
                   dest="push_x",
                   help="第一部分步骤 a: X 推到位置 (mm, 默认 -200)")
    p.add_argument("--push-arm", type=float, default=PUSH_ARM_DEG,
                   dest="push_arm",
                   help="第一部分步骤 b: 大臂角度 (°, 默认 -95)")
    p.add_argument("--push-hand", type=float, default=PUSH_HAND_DEG,
                   dest="push_hand",
                   help="第一部分步骤 c: 手爪角度 (°, 默认 -90 = UP)")
    p.add_argument("--push-y", type=float, default=PUSH_Y_MM,
                   dest="push_y",
                   help="第一部分步骤 d: Y 下降目标 (mm, 默认 0 = 触底)")
    p.add_argument("--bar-hand", type=float, default=PUSH_BAR_HAND_DEG,
                   dest="bar_hand",
                   help="第一部分步骤 e: 推杆姿态手爪角度 (°, 默认 -55)")
    p.add_argument("--safe-y", type=float, default=PUSH_SAFE_Y_MM,
                   dest="safe_y",
                   help="步骤 0 软抢答 y 抬升目标 (mm, 默认 -150)")
    # 第二部分
    p.add_argument("--sweep-end", type=float, default=SWEEP_X_END_MM,
                   dest="sweep_end",
                   help="第二部分步骤 6: X 扫动终点 (mm, 默认 -120)")
    p.add_argument("--sweep-speed", type=float, default=SWEEP_SPEED_MMS,
                   dest="sweep_speed",
                   help="第二部分步骤 6: X 扫动速度 (mm/s, 默认 100)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    # CLI 覆盖所有 8 个常量 (模块顶置常量是文档/默认值)
    global PUSH_X_MM, PUSH_ARM_DEG, PUSH_HAND_DEG, PUSH_Y_MM, PUSH_BAR_HAND_DEG
    global PUSH_SAFE_Y_MM
    global SWEEP_X_END_MM, SWEEP_SPEED_MMS
    PUSH_X_MM = args.push_x
    PUSH_ARM_DEG = args.push_arm
    PUSH_HAND_DEG = args.push_hand
    PUSH_Y_MM = args.push_y
    PUSH_BAR_HAND_DEG = args.bar_hand
    PUSH_SAFE_Y_MM = args.safe_y
    SWEEP_X_END_MM = args.sweep_end
    SWEEP_SPEED_MMS = args.sweep_speed

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(message)s")
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner)
    return 0


if __name__ == "__main__":
    sys.exit(main())
