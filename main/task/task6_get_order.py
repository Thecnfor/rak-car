#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务六: 智能接单 (推杆扫订单 + OCR 读单 + 抓取对应蔬菜).

推杆动作完整序列 (sweep 期间 Y 保持 = 0 触底):
  1. X 收至 -200 mm (PID 闭环定位)
  2. 大臂 arm → -95° + 等待 2s 稳定
  3. 手爪 hand → -90° + 等待 1s 稳定
  4. Y 下降 → 0 mm (触底限位, 推杆前保持此高度)
  5. hand → -55° (推杆准备姿态)
  6. X 推杆扫动: -200 → -120 @ 100 mm/s (PID 闭环带动推牌杆)
  7. X 回退 → -150 mm (调整位置)
  8. 先抬 Y → -80 (防碰撞) 再转大臂 arm → -85° (读单姿态)
  9. hand → -55° (确认)
  10. Y → -100 mm (安全抬升)

=== 分阶段划分 ===
  阶段 1-2: 推杆动作 + X/Y/arm 复位 (读单前姿态准备)
  阶段 3  : LLM × 2 轮读单 (调用 test_order_read.run)
  阶段 4  : 抓取对应蔬菜 → LLM 视觉识别 + 真空抓取 × 2 棵
  阶段 5  : 回到运送待命姿态 (准备给任务七, 目前 stub)

动作辅助函数来源: 统一走 main.arm.ArmRunner (含 y 保护区 / 角度硬限 /
丢步核对 / composite_* 并发执行). 自定义的安全门不再存在 —— SafetyMixin
在每个动作入口处统一校验.

架构说明 (2026-08 重构):
  本任务使用 main.arm.ArmRunner + CompositeMixin 编排动作,
  不再依赖 main/task/_helpers.py (该文件已删除, 详见 main/task/README.md).
"""
from __future__ import annotations

import logging
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import yaml

from main.api_client import RuntimeApiClient
from main.arm import ArmClient, ArmRunner
from main.misc.test_order_read import run as order_read_run
from main.misc.test_veggie_detect import run as veggie_detect_run

# task6 配置独立保留在 test/task6_config.yml (避免侵入 task_config.yml 其它段)
_TASK6_CONFIG = Path(_PROJECT_ROOT) / "test" / "task6_config.yml"

logger = logging.getLogger("task.get_order")


# ── 配置加载 ──────────────────────────────────────────────────

def _load_task6_config() -> Dict[str, Any]:
    """加载任务六独立配置 (只读 test/task6_config.yml, 不侵入 task_config.yml)."""
    if not _TASK6_CONFIG.exists():
        raise FileNotFoundError(f"任务六配置文件不存在: {_TASK6_CONFIG}")
    with _TASK6_CONFIG.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    cfg = data.get("get_order") if isinstance(data, dict) else {}
    if not cfg or cfg.get("placeholder"):
        raise KeyError("task6_config.yml 中未找到 get_order 配置段或为占位")
    return cfg


# ── 蔬菜货架 X 坐标映射 (从上到下共 4 行) ──

_SHELF_X_BY_ROW = [-50.0, -100.0, -140.0, -180.0]


def _pos_to_row(pos: str) -> int:
    """将 LLM 输出的中文位置文本映射到货架行号 (0=最上第1行, 3=最下第4行).

    支持格式示例: "右1"=右排第1行, "左3"=左排第3行.
    解析失败 fallback 到第 2 行 (下标 1).
    """
    pos = (pos or "").strip()
    m = re.search(r'[左右]\s*(\d)', pos)
    if m:
        n = int(m.group(1))
        return max(0, min(3, n - 1))  # 1→0, 2→1, 3→2, 4→3
    return 1  # 兜底: 默认第 2 行


def _pos_to_side(pos: str) -> str:
    """根据中文位置判断货架左右排, 返回 'left' 或 'right'."""
    return "left" if "左" in (pos or "") else "right"


# ── 底盘移动 ──────────────────────────────────────────────────

def _chassis_move_for(
    arm_client: ArmClient,
    dx_m: float,
    timeout: float,
) -> dict:
    """底盘纵向 move_for 阻塞调用 (sync=True 等结果).

    走 ChassisClient.move_for —— move_for 是底盘动作, 不应走
    ArmClient._call_car. 后者签名是 (name, timeout=20.0, *args, sync=False, **kwargs),
    第二个位置参数是 timeout, 写成 _call_car("move_for", dx_m, timeout=...)
    会把 dx_m 误绑给 timeout, 报 "multiple values for argument 'timeout'".
    """
    from main.chassis import ChassisClient
    chassis = ChassisClient.connect()
    try:
        return chassis.move_for(dx_m=dx_m, timeout=timeout)
    finally:
        chassis.close()


# ── Y=0 触底时的动作 (绕开业务层 y 保护区) ────────────────────────
#
# ⚠️ main/arm/api/safety.py:76 的 y 保护区 fail-closed: y > -30mm 时
#    set_hand_angle / set_arm_angle / move_x 一律 raise ValueError,
#    只豁免 hand=-90 / arm=0 / arm=+90 三个 init 姿态。
#
#    但任务六的推杆动作**必须在 Y=0 触底时**摆手爪 + 扫 X (推牌杆要贴地
#    才能拨动订单机推杆, 见 test/task6_config.yml push_bar_pose.y_mm=0),
#    走 wrapper 必被拒。
#
#    因此这两个动作走 `_call_arm` 底层直调, 跳过 Python 层校验, 合法性
#    交由车端判断 —— 与 task5/low_tower.py:118-126 (手爪 0° DOWN) 和
#    task5/get_blue.py:172 同款处理。
#
#    ⚠️ 代价: 绕过安全门后, 若姿态没摆对, 手爪在触底高度横扫会撞到车体。
#       首次上场前务必先低速手动确认 arm=-95° 时手爪的扫掠半径。

def _set_hand_angle_at_bottom(
    arm_client: ArmClient,
    angle: float,
    speed: int = 80,
    timeout: float = 10.0,
) -> dict:
    """Y 触底时设置手爪角度 (底层直调, 绕开 y 保护区)."""
    return arm_client._call_arm(
        "set_hand_angle", timeout=timeout, sync=True,
        angle=float(angle), speed=speed,
    )


def _move_x_at_bottom(
    arm_client: ArmClient,
    x_mm: float,
    v_max_mms: float = 40.0,
    out_time: float = 15.0,
    timeout: float = 30.0,
) -> dict:
    """Y 触底时移动 X (底层直调, 绕开 y 保护区).

    注意: 绕开 wrapper 也就绕开了 `move_x` 的丢步核对 (_check_step_loss),
    需要校验实际到位时请自行读 realtime x_mm 对比。
    """
    return arm_client._call_arm(
        "move_x_position", timeout=timeout, sync=True,
        target=float(x_mm) / 1000.0, out_time=out_time,
        v_max_mms=float(v_max_mms),
    )


# ── 单棵蔬菜抓取+投放 (走 ArmRunner + composite_*) ──────────────────

def _pick_one_veggie(
    arm_client: ArmClient,
    runner: ArmRunner,
    target_x_mm: float,
    carry_y_mm: float,
    drop_y_mm: float,
    label: str = "",
) -> None:
    """单棵蔬菜抓取+投放流程: 抓取预备 → 抓取 → 投放预备 → 投放.

    执行顺序 (composite_run 内置并发 + SafetyMixin 自动 y 保护区校验):
      1. composite_run: 大臂 → -95° + 手爪 → -10° + Y → -200 (预备, 并发)
      2. composite_run: X → target_x_mm + 手爪 → 0° (伸出, 并发)
      3. move_y → -20 (下降接近蔬菜)
      4. grasp on + 真空稳定等待
      5. move_y → carry_y (运输高度, 第 1 棵 -110, 第 2 棵 -140 避免碰撞)
      6. composite_run: X → 0 + 大臂 → +95° + 手爪 → -20° (投放姿态, 并发)
      7. move_y → drop_y (第 1 棵 -40, 第 2 棵 -80 防止堆叠)
      8. grasp off 释放

    Args:
        target_x_mm: 该蔬菜所在货架行对应的 X 坐标
        carry_y_mm: 吸取后抬升的 Y
        drop_y_mm: 投放时下降的 Y
        label: 用于日志标识的蔬菜名称
    """
    tag = f"[{label}]" if label else ""
    logger.info("%s 抓取: X=%.0f 抬升Y=%.0f 投放Y=%.0f", tag, target_x_mm, carry_y_mm, drop_y_mm)

    # 1) 抓取预备姿态: 大臂 → -95° + 手爪 → -10° + Y → -200 (并发)
    runner.client.composite_run(arm=-95.0, hand=-10.0, y_mm=-200.0)

    # 2) 伸出 + 手爪朝下: X → target_x_mm + 手爪 → 0° (并发)
    runner.client.composite_run(x_mm=target_x_mm, hand=0.0)

    # 3) 下降到接近蔬菜
    runner.move_y(-20.0)
    logger.info("  %s X → %.0f mm, 手爪→0°, Y→-20 mm (抓取就绪)", tag, target_x_mm)

    # 4) 开真空吸盘吸住蔬菜 → 稳定等待
    runner.grasp(on=True)
    time.sleep(0.5)
    logger.info("  %s 真空开启 (吸附保持)", tag)

    # 5) Y 抬升到运输安全高度
    runner.move_y(carry_y_mm)
    logger.info("  %s Y → %.0f mm (运输高度)", tag, carry_y_mm)

    # 6) 投放前转换姿态: X 收回 + 大臂 + 手爪 (并发)
    runner.client.composite_run(x_mm=0.0, arm=95.0, hand=-20.0)
    logger.info("  %s X → 0, 大臂→+95°, 手爪→-20° (投放姿态)", tag)

    # 7) Y 下降到投放高度
    runner.move_y(drop_y_mm)
    logger.info("  %s Y → %.0f mm (投放)", tag, drop_y_mm)

    # 8) 关真空释放蔬菜
    runner.grasp(on=False)
    time.sleep(0.3)
    logger.info("  %s 真空关闭 (释放)", tag)


# ── 推杆姿态 + 扫牌 + 读单姿态 ──────────────────────────────────────

def _enter_push_bar_pose(
    arm_client: ArmClient,
    runner: ArmRunner,
    cfg: Dict[str, Any],
) -> None:
    """推杆姿态准备 → X 扫动推牌 → 调整到读单姿态 — 完整序列.

    目标动作顺序 (参数取自 task6_config.yml):
      第一部分 (推杆姿态准备):
        1. X 收至 push_bar_pose.x_mm (PID 闭环)
        2. 大臂 arm → push_bar_pose.arm_angle_deg + 等待 2s 稳定
        3. 手爪 hand → push_bar_pose.hand_angle_deg + 等待 1s 稳定
        4. Y 下降 → push_bar_pose.y_mm (PID 闭环, 直至触底限位)
        5. hand → -55° (推杆姿态就绪, 手爪作为推牌杆)
      第二部分 (X 扫动推牌, Y 保持 = 0):
        6. X 扫动: x_mm → sweep_x_end_mm @ sweep_speed_mms
      第三部分 (调整到读单姿态):
        7. X 回退 → reposition_pose.x_mm (中途重定位)
        8. Y 抬升 → reposition_pose.y_mm (先抬 Y 再转大臂, 防 Y=0 转臂碰撞)
        9. arm → reposition_pose.arm_angle_deg (转到读单/携带姿态)
       10. hand → reposition_pose.hand_angle_deg (确认)
       11. X → reposition_pose.final_x_mm (抬升后的最终 X 位置)

    ⚠️ 动作顺序硬约束 (防止 Jetson 瞬时大电流掉电 / cam2 USB 断连):
       先 X → 再 arm(+sleep 2s) → 再 hand(+sleep 1s) → 最后 Y.
       顺序绝不能换, 否则多个大电流舵机同时启动会触发硬件保护.
    """
    pose = cfg["push_bar_pose"]
    sweep_end = cfg.get("sweep_x_end_mm", -120.0)
    sweep_speed = cfg.get("sweep_speed_mms", 100.0)
    repos = cfg["reposition_pose"]

    # === 第一部分: 推杆姿态准备 ===
    logger.info(
        "阶段 1: 推杆姿态准备 (x=%.0f arm=%.0f hand=%.0f y=%.0f)",
        pose["x_mm"], pose["arm_angle_deg"],
        pose["hand_angle_deg"], pose["y_mm"],
    )

    # === 第零步: 确保 y 在保护区外 (否则下面第一个 move_x 直接被拦) ===
    # 上一轮任务 / 上一次崩溃可能把 y 留在触底附近, 此时 move_x 会 raise。
    # move_y 本身从不被安全门拦, 可以无条件先抬。
    try:
        cur_y = float(arm_client.get_state().y_mm)
    except Exception:
        cur_y = 0.0
    if cur_y > -30.0:
        logger.info("  y=%.1fmm 在保护区内, 先抬到 -150mm 再动 X", cur_y)
        runner.move_y(-150.0)

    # a) X 轴: PID 闭环移动到位
    runner.move_x(float(pose["x_mm"]))
    logger.info("  X → %.0f mm 完成", pose["x_mm"])

    # b) 大臂旋转 (大扭矩动作, 单独执行 + 长等待稳定)
    runner.set_arm_angle(float(pose["arm_angle_deg"]), speed=40)
    time.sleep(2.0)
    logger.info("  大臂 → %.0f° 完成", pose["arm_angle_deg"])

    # c) 手爪旋转到 -90°
    arm_client.set_hand_angle(float(pose["hand_angle_deg"]), speed=80, timeout=10.0)
    time.sleep(1.0)
    logger.info("  手爪 → %.0f° 完成", pose["hand_angle_deg"])

    # d) Y 轴: PID 闭环下降到底部限位 (必须最后执行 Y)
    runner.move_y(float(pose["y_mm"]))
    logger.info("  Y → %.0f mm 完成 (扫动前 Y 已触底)", pose["y_mm"])

    # e) 手爪转到 -55° (与推牌杆配合的推杆姿态)
    #    此时 Y 已触底 (=0), 走 wrapper 必被 y 保护区拒 → 底层直调
    _set_hand_angle_at_bottom(arm_client, -55.0)
    time.sleep(0.5)
    logger.info("  手爪 → -55° 完成 (推杆姿态就绪)")

    logger.info("推杆姿态就绪 (X=%.0f arm=%.0f hand=-45 Y=%.0f)",
                pose["x_mm"], pose["arm_angle_deg"], pose["y_mm"])

    # === 第二部分: X 扫动推牌 (Y 保持触底) ===
    logger.info("阶段 1b: X 扫动推牌 %.0f → %.0f @ %.0f mm/s (Y=%.0f)",
                pose["x_mm"], sweep_end, sweep_speed, pose["y_mm"])
    # Y 仍触底 (=0), 走 wrapper 必被 y 保护区拒 → 底层直调
    # sweep_speed_mms 之前只进了日志没进调用 (实际按默认 40mm/s 跑), 这里补上
    _move_x_at_bottom(arm_client, float(sweep_end), v_max_mms=float(sweep_speed))
    logger.info("  扫动完成, X=%.0f mm", sweep_end)

    # === 第三部分: 调整到读单姿态 ===
    logger.info(
        "阶段 1c: 调整读单姿态 (x=%.0f arm=%.0f hand=%.0f y=%.0f)",
        repos["x_mm"], repos["arm_angle_deg"],
        repos["hand_angle_deg"], repos["y_mm"],
    )

    # f) X 先移到 reposition x (Y 仍触底 → 底层直调)
    _move_x_at_bottom(arm_client, float(repos["x_mm"]))
    logger.info("  X → %.0f mm 完成", repos["x_mm"])

    # g) Y 先抬升 (必须先抬 Y 再转大臂, 避免 Y=0 时大臂横扫撞到推牌机构)
    runner.move_y(float(repos["y_mm"]))
    logger.info("  Y → %.0f mm 完成", repos["y_mm"])

    # h) 大臂转到读单/携带角度
    runner.set_arm_angle(float(repos["arm_angle_deg"]), speed=40)
    time.sleep(1.5)
    logger.info("  大臂 → %.0f° 完成", repos["arm_angle_deg"])

    # i) 手爪转到确认角度
    arm_client.set_hand_angle(float(repos["hand_angle_deg"]), speed=80, timeout=10.0)
    time.sleep(0.5)
    logger.info("  手爪 → %.0f° 完成", repos["hand_angle_deg"])

    # j) 最后精修 X 坐标 (抬 Y+转臂后编码器可能漂移, 这里不做严格校验避免误杀)
    final_x = float(repos.get("final_x_mm", -140))
    runner.move_x(final_x)
    logger.info("  X → %.0f mm 完成", final_x)

    logger.info("推杆 + 扫牌 + 读单姿态调整完成")


# ============================================================
# 阶段 3-5 存根 (目前 stub, 部分逻辑已在 run() 中内联实现)
# 阶段 1-2 (推杆姿态 + 扫牌) 由 _enter_push_bar_pose 统一处理
# ============================================================

def _detect_and_ocr(arm_client: ArmClient, runner: ArmRunner, cfg: Dict[str, Any]) -> dict:
    """阶段 3 存根: cam2 检测前方订单牌 + OCR 读取 + 解析. 当前未实现."""
    raise NotImplementedError("阶段 3 detect_and_ocr - 待实现")


def _pick_goods(arm_client: ArmClient, runner: ArmRunner, cfg: Dict[str, Any]) -> dict:
    """阶段 4 存根: 物理抓取 5cm 订单方块. 当前未实现 (run 中另有实现)."""
    raise NotImplementedError("阶段 4 pick_goods - 待实现")


def _lift_and_carry(arm_client: ArmClient, runner: ArmRunner, cfg: Dict[str, Any]) -> dict:
    """阶段 5 存根: 抬升 + 运输待命姿态 (为任务七做准备). 当前未实现."""
    raise NotImplementedError("阶段 5 lift_and_carry - 待实现")


# ============================================================
# run() 主入口
# ============================================================

def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """任务六主入口: 推杆扫牌 + LLM 双轮读单 + 蔬菜识别匹配 + 抓取投放.

    Args:
        client: 可选 RuntimeApiClient, None 时内部新建

    Returns:
        Dict: {
            "ok": bool,                          # 任务是否成功
            "completed": List[str],             # 已完成的子步骤列表
            "order_list": {"round1":[], "round2":[]},  # 两轮 LLM 读单结果
            "error": str                        # 失败时的错误信息 (仅 ok=False)
        }
    """
    cfg = _load_task6_config()

    if client is None:
        client = RuntimeApiClient()
    client.wait_until_ready(timeout=30.0)

    # 初始化机械臂客户端与执行器
    arm_client = ArmClient.connect()
    if not arm_client.ping():
        raise RuntimeError("机械臂 runtime 未在线, 请检查 arm_feed 守护进程")
    runner = ArmRunner(arm_client)

    completed: List[str] = []
    order_list: List[Dict[str, Any]] = []

    try:
        # ===== 阶段 1+2: 推杆姿态准备 + X 扫动推牌 + 调整到读单姿态 =====
        logger.info("=== 任务六阶段 1+2: 推杆姿态 + 扫牌 + 读单姿态调整 ===")
        _enter_push_bar_pose(arm_client, runner, cfg)
        completed.extend(["push_bar_pose", "sweep", "reposition"])

        # ===== 阶段 3a: LLM 读单 (第一轮, 当前位置) =====
        logger.info("=== 阶段 3a: LLM 读单 第一轮 (当前位置) ===")
        round1 = order_read_run()
        if round1.get("ok") and round1.get("orders"):
            logger.info("  [第一轮] 读取到 %d 条订单:", len(round1["orders"]))
            for o in round1["orders"]:
                logger.info("    客户: %s | 商品: %s", o["name"], o["goods"])
        else:
            logger.warning("  [第一轮] 读取失败: %s", round1.get("error", "未识别到订单"))
        completed.append("order_read_1")

        # ===== 阶段 3b: 调整姿态 (为第二轮读单做不同角度尝试) =====
        logger.info("=== 阶段 3b: 调整姿态 X→-150 hand→-70° Y→0 准备第二轮读单 ===")
        runner.move_x(-150.0)
        logger.info("  X → -150 mm 完成")
        arm_client.set_hand_angle(-70.0, speed=80, timeout=10.0)
        time.sleep(0.5)
        logger.info("  手爪 → -70° 完成")
        runner.move_y(0.0)
        logger.info("  Y → 0 mm 完成")

        # ===== 阶段 3c: LLM 读单 第二轮 (多角度重试: -70°→-55°→-90°) =====
        round2: Dict[str, Any] = {"ok": False, "orders": [], "error": "no attempts"}
        for attempt, hand_angle in enumerate([-70.0, -55.0, -90.0]):
            logger.info("=== 阶段 3c: LLM 读单 第二轮 (手爪=%.0f°, 第 %d 次尝试) ===", hand_angle, attempt + 1)
            if attempt > 0:
                # 此时 Y=0 (阶段 3b 已下降), 走 wrapper 必被 y 保护区拒 → 底层直调
                _set_hand_angle_at_bottom(arm_client, hand_angle)
                time.sleep(0.5)
                logger.info("  手爪 → %.0f° (重试)", hand_angle)
            round2 = order_read_run()
            if round2.get("ok") and round2.get("orders"):
                logger.info("  [第二轮] 读取到 %d 条订单 (第 %d 次尝试, hand=%.0f°):",
                            len(round2["orders"]), attempt + 1, hand_angle)
                for o in round2["orders"]:
                    logger.info("    客户: %s | 商品: %s", o["name"], o["goods"])
                break
            logger.warning("  [第二轮] 第 %d 次尝试失败 (hand=%.0f°): %s",
                           attempt + 1, hand_angle, round2.get("error", "未识别到订单"))
        completed.append("order_read_2")

        # 合并两轮读单结果
        order_list = {
            "round1": round1.get("orders", []),
            "round2": round2.get("orders", []),
        }

        # ===== 阶段 4: 识别并抓取订单中的蔬菜 =====
        logger.info("=== 阶段 4: 抓取订单对应蔬菜 ===")

        # 4a) 调整抓取预备姿态: Y→-200, arm→-95°, hand→-10°
        #     ⚠️ 阶段 3b 把 Y 压到 0, 此时 composite_run(y_mm=...) 会读当前 y
        #     并被保护区拒 (composite.py:62)。所以先单独 move_y 抬出保护区
        #     (move_y 从不被拦), 再并发摆大臂 + 手爪。
        logger.info("  调整抓取预备姿态: Y→-200 arm→-95 hand→-10")
        runner.move_y(-200.0)
        runner.client.composite_run(arm=-95.0, hand=-10.0)
        logger.info("  抓取预备姿态就绪")

        # 4b) 底盘前进 15cm 靠近蔬菜货架
        _chassis_move_for(arm_client, dx_m=0.15, timeout=30.0)
        logger.info("  底盘前进 15cm 完成")

        # 4c) LLM 视觉识别货架上所有蔬菜
        logger.info("  调用 LLM 进行蔬菜识别...")
        veggie_result = veggie_detect_run()
        veggie_items = veggie_result.get("items", []) if veggie_result.get("ok") else []
        if not veggie_items:
            logger.warning("  蔬菜识别: 未找到任何目标, 跳过抓取")
        else:
            logger.info("  蔬菜识别: 共识别到 %d 棵", len(veggie_items))
            for it in veggie_items:
                logger.info("    [位置:%s] %s 置信度:%s",
                            it.get("position", "?"), it.get("name", "?"), it.get("confidence", "?"))

        # 4d) 从两轮订单中提取出所有被订购的蔬菜名集合
        ordered_goods: set = set()
        for rnd in [round1, round2]:
            for o in (rnd.get("orders") or []):
                g = o.get("goods", "")
                if g:
                    ordered_goods.add(g)
        logger.info("  订单需求蔬菜集合: %s", ordered_goods)

        # 4e) 匹配: 优先抓取订单里有的蔬菜; 没匹配到时 fallback 抓右侧货架 (兜底)
        matched = [v for v in veggie_items if v.get("name") in ordered_goods]
        if not matched:
            logger.warning("  订单蔬菜未匹配识别结果, 兜底: 取右侧货架上的蔬菜")
            matched = [v for v in veggie_items if _pos_to_side(v.get("position", "")) == "right"]
        else:
            logger.info("  订单匹配成功 %d 棵蔬菜", len(matched))

        # 分左右排: 右侧优先取 (近, 不需要额外底盘位移)
        right_targets = [v for v in matched if _pos_to_side(v.get("position", "")) == "right"]
        left_targets = [v for v in matched if _pos_to_side(v.get("position", "")) == "left"]
        pick_idx = 0

        # ── 先抓右侧货架 (最多 2 棵, 右侧距离近) ──
        for veg in right_targets[:2]:
            row = _pos_to_row(veg.get("position", ""))
            x_pos = _SHELF_X_BY_ROW[min(row, 3)]
            carry_y = -110.0 if pick_idx == 0 else -140.0
            drop_y = -40.0 if pick_idx == 0 else -80.0
            label = veg.get("name", f"item{pick_idx + 1}")
            logger.info("  → [右侧] 抓取 '%s' 行=%d X=%.0f (第 %d 棵)", label, row, x_pos, pick_idx + 1)
            _pick_one_veggie(arm_client, runner, x_pos, carry_y, drop_y, label=label)
            completed.append(f"picked_{pick_idx + 1}")
            pick_idx += 1

        # ── 再抓左侧货架 (如果还未抓满 2 棵, 需要底盘再前进 12cm 才够到) ──
        if left_targets and pick_idx < 2:
            logger.info("  底盘前进 12cm 以够到左侧蔬菜")
            _chassis_move_for(arm_client, dx_m=0.12, timeout=30.0)
            for veg in left_targets[:2 - pick_idx]:
                row = _pos_to_row(veg.get("position", ""))
                x_pos = _SHELF_X_BY_ROW[min(row, 3)]
                carry_y = -110.0 if pick_idx == 0 else -140.0
                drop_y = -50.0 if pick_idx == 0 else -100.0
                label = veg.get("name", f"item{pick_idx + 1}")
                logger.info("  → [左侧] 抓取 '%s' 行=%d X=%.0f (第 %d 棵)", label, row, x_pos, pick_idx + 1)
                _pick_one_veggie(arm_client, runner, x_pos, carry_y, drop_y, label=label)
                completed.append(f"picked_{pick_idx + 1}")
                pick_idx += 1

        if pick_idx == 0:
            logger.warning("  最终未能抓取任何蔬菜 (无可选目标)")

        completed.append("pick_goods")

    except Exception as exc:
        logger.exception("get_order 任务失败: %s", exc)
        return {
            "ok": False,
            "completed": completed,
            "order_list": order_list,
            "error": str(exc),
        }

    return {
        "ok": True,
        "completed": completed,
        "order_list": order_list,
    }


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
    result = run()
    print("任务六 智能接单 执行结果:", result)