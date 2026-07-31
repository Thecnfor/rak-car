#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/tasks/get_order.py

Task 6: get_order (smart order pickup / 智能接单).

Full motion sequence (Y=0 during sweep):
  1. X → -200 mm (PID)
  2. arm → -95° + settle 2s
  3. hand → -90° + settle 1s
  4. Y → 0 mm (touch bottom)          ← sweep 时 Y 保持 0
  5. hand → -45° (push-bar ready)
  6. X sweep: -200 → -120 at 100 mm/s (推杆, PID 闭环)
  7. X → -150 mm (reposition)
  8. arm → -85° (carry-ready)
  9. hand → -45° (confirm)
  10. Y → -100 mm (safe lift)

Phases:
  Phase 1-2: push-bar pose + sweep + reposition
  Phase 3: order reading ×2 via LLM (→ test_order_read.run)
  Phase 4: pick goods — veggie detect + vacuum pick ×2
  Phase 5: carry pose for Task 7 (stub)

Motion helpers: 统一从 main.task._helpers 取 (与 task1/task2 共用)。
"""
from __future__ import annotations

import logging
import re
import sys
import time
from pathlib import Path
from typing import Any, Dict, Optional

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

from main.api_client import RuntimeApiClient
from main.task._helpers import (
    _ensure_runtime,
    _wait_infer_ready,
    _move_x,
    _move_y,
    _set_arm_angle,
    _set_hand_angle,
    _grasp,
    _chassis_move_for,
    _move_x_checked,
    _read_x_mm,
)
from main.misc.test_order_read import run as order_read_run
from main.misc.test_veggie_detect import run as veggie_detect_run

import yaml

# task6 配置独立保留在 test/task6_config.yml (避免侵入 task_config.yml 其它段)
_TASK6_CONFIG = Path(_PROJECT_ROOT) / "test" / "task6_config.yml"


def _load_task6_config() -> Dict[str, Any]:
    """只读 test/task6_config.yml, 不碰 task_config.yml."""
    if not _TASK6_CONFIG.exists():
        raise FileNotFoundError(f"任务六配置文件不存在: {_TASK6_CONFIG}")
    with _TASK6_CONFIG.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    cfg = data.get("get_order") if isinstance(data, dict) else {}
    if not cfg or cfg.get("placeholder"):
        raise KeyError("task6_config.yml 中没有 get_order 段")
    return cfg

logger = logging.getLogger("task.get_order")


# ── 货架 X 位置（从上到下 4 行）──
_SHELF_X_BY_ROW = [-50.0, -100.0, -140.0, -180.0]


def _pos_to_row(pos: str) -> int:
    """将 LLM 输出的位置文字映射到货架行号 (0=最上1, 3=最下4).

    格式: "右1"=右边第一排第1行, "左3"=左边第二排第3行.
    """
    pos = (pos or "").strip()
    m = re.search(r'[左右]\s*(\d)', pos)
    if m:
        n = int(m.group(1))
        return max(0, min(3, n - 1))  # 1→0, 2→1, 3→2, 4→3
    return 1  # fallback


def _pos_to_side(pos: str) -> str:
    """返回 'left' 或 'right'."""
    return "left" if "左" in (pos or "") else "right"


def _pick_one_veggie(client, target_x_mm: float, carry_y_mm: float,
                     drop_y_mm: float, label: str = ""):
    """取一个蔬菜: X到位→末端0°→Y降→吸→Y抬→X→0→末端-90°大臂+95°→Y降→放.

    Args:
        target_x_mm: 该蔬菜对应的货架 X 位置
        carry_y_mm: 吸取后抬升 Y（第一个 -90, 第二个 -140）
        drop_y_mm: 投放时下降 Y（第一个 -40, 第二个 -80）
        label: 日志标记
    """
    tag = f"[{label}]" if label else ""
    logger.info("%s pick: X=%.0f carry_y=%.0f drop_y=%.0f", tag, target_x_mm, carry_y_mm, drop_y_mm)

    # 0) 恢复取菜姿势: arm→-95°, hand→-10°, Y→-200
    _set_arm_angle(client, -95.0, speed=40)
    time.sleep(1.5)
    _set_hand_angle(client, -10.0, speed=80)
    time.sleep(0.5)
    _move_y(client, -200.0)
    logger.info("  %s pose reset: arm=-95 hand=-10 Y=-200", tag)

    # 1) X 到位
    _move_x(client, target_x_mm, v_max_mms=40.0)
    logger.info("  %s X -> %.0f mm", tag, target_x_mm)

    # 2) 先转末端 0°, 再降 Y 到 -20
    _set_hand_angle(client, 0.0, speed=80)
    time.sleep(0.3)
    _move_y(client, -20.0)
    logger.info("  %s hand -> 0 deg, Y -> -20 mm (pick ready)", tag)

    # 3) 吸泵: 开气阀吸气→关阀保持真空
    _grasp(client, True)
    time.sleep(0.5)
    logger.info("  %s grasp ON (vacuum hold)", tag)

    # 4) Y 抬升
    _move_y(client, carry_y_mm)
    logger.info("  %s Y -> %.0f mm (carry)", tag, carry_y_mm)

    # 5) 投放: 先 X→0, 再转大臂+末端
    _move_x(client, 0.0, v_max_mms=80.0)
    logger.info("  %s X -> 0 mm", tag)
    _set_arm_angle(client, 95.0, speed=40)
    time.sleep(1.5)
    _set_hand_angle(client, -20.0, speed=80)
    time.sleep(0.5)
    logger.info("  %s arm -> +95 deg, hand -> -20 deg (drop pose)", tag)

    # 6) Y 下降投放
    _move_y(client, drop_y_mm)
    logger.info("  %s Y -> %.0f mm (drop)", tag, drop_y_mm)

    # 7) 放气
    _grasp(client, False)
    time.sleep(0.3)
    logger.info("  %s grasp OFF (release)", tag)


def _enter_push_bar_pose(client, cfg):
    """Push-bar pose → sweep → reposition — full sequence.

    Target sequence (values from task_config.yml):
      1. X → push_bar_pose.x_mm (PID)
      2. arm → push_bar_pose.arm_angle_deg + settle 2s
      3. hand → push_bar_pose.hand_angle_deg + settle 1s
      4. Y → push_bar_pose.y_mm (PID, touch bottom)
      5. hand → -55° (push-bar ready)
      6. X sweep: x_mm → sweep_x_end_mm at sweep_speed_mms (推杆, Y=0)
      7. X → reposition_pose.x_mm (reposition)
      8. Y → reposition_pose.y_mm (先抬升, 防止转臂碰撞)
      9. arm → reposition_pose.arm_angle_deg (carry-ready)
      10. hand → reposition_pose.hand_angle_deg (confirm)
      11. X → reposition_pose.final_x_mm (final position after lift)

    Motion order (critical to avoid Jetson brown-out / cam2 USB disconnect):
      X first, then arm + sleep 2s, then hand + sleep 1s, then Y.
    """
    pose = cfg["push_bar_pose"]
    v_x = cfg["v_max_arm_x_mms"]
    sweep_end = cfg.get("sweep_x_end_mm", -120.0)
    sweep_speed = cfg.get("sweep_speed_mms", 100.0)
    repos = cfg["reposition_pose"]

    # === push-bar pose ===
    logger.info(
        "Phase 1: push-bar pose (x=%.0f arm=%.0f hand=%.0f y=%.0f)",
        pose["x_mm"], pose["arm_angle_deg"],
        pose["hand_angle_deg"], pose["y_mm"],
    )

    # a) X axis: PID closed-loop (blocks until reached)
    _move_x(client, pose["x_mm"], v_max_mms=v_x)
    logger.info("  X -> %.0f mm done", pose["x_mm"])

    # b) arm rotation
    _set_arm_angle(client, pose["arm_angle_deg"], speed=40)
    time.sleep(2.0)  # big rotation settle
    logger.info("  arm -> %.0f deg done", pose["arm_angle_deg"])

    # c) hand rotation to -90°
    _set_hand_angle(client, pose["hand_angle_deg"], speed=80)
    time.sleep(1.0)
    logger.info("  hand -> %.0f deg done", pose["hand_angle_deg"])

    # d) Y axis: PID closed-loop to bottom
    _move_y(client, pose["y_mm"])
    logger.info("  Y -> %.0f mm done (sweep 前 Y 已触底)", pose["y_mm"])

    # e) hand → -55° (push-bar ready)
    _set_hand_angle(client, -55.0, speed=80)
    time.sleep(0.5)
    logger.info("  hand -> -55 deg done (push-bar ready)")

    logger.info("Push-bar pose ready (X=%.0f arm=%.0f hand=-45 Y=%.0f)",
                pose["x_mm"], pose["arm_angle_deg"], pose["y_mm"])

    # === sweep: X at Y=0, PID closed-loop ===
    logger.info("Phase 1b: sweep X %.0f → %.0f at %.0f mm/s (Y=%.0f)",
                pose["x_mm"], sweep_end, sweep_speed, pose["y_mm"])
    _move_x(client, sweep_end, v_max_mms=sweep_speed)
    logger.info("  sweep done, X=%.0f mm", sweep_end)

    # === reposition ===
    logger.info(
        "Phase 1c: reposition (x=%.0f arm=%.0f hand=%.0f y=%.0f)",
        repos["x_mm"], repos["arm_angle_deg"],
        repos["hand_angle_deg"], repos["y_mm"],
    )

    # f) X → repos.x_mm (先移到 -170)
    _move_x(client, repos["x_mm"], v_max_mms=v_x)
    logger.info("  X -> %.0f mm done", repos["x_mm"])

    # g) Y → repos.y_mm (先抬升 Y 再转大臂，防止 Y=0 时转臂撞到)
    _move_y(client, repos["y_mm"])
    logger.info("  Y -> %.0f mm done", repos["y_mm"])

    # h) arm → repos.arm_angle_deg
    _set_arm_angle(client, repos["arm_angle_deg"], speed=40)
    time.sleep(1.5)
    logger.info("  arm -> %.0f deg done", repos["arm_angle_deg"])

    # i) hand → repos.hand_angle_deg (confirm)
    _set_hand_angle(client, repos["hand_angle_deg"], speed=80)
    time.sleep(0.5)
    logger.info("  hand -> %.0f deg done", repos["hand_angle_deg"])

    # j) X → final_x_mm (Y 抬升+转臂后编码器可能偏移, 不用校验防误杀)
    final_x = repos.get("final_x_mm", -140)
    _move_x(client, final_x, v_max_mms=v_x)
    logger.info("  X -> %.0f mm done", final_x)

    logger.info("Full push-bar + sweep + reposition complete")


# ============================================================
# Phase 3-5: subsequent steps (stubs only)
# Phase 1 (push-bar pose + sweep) is handled by _enter_push_bar_pose
# ============================================================

def _detect_and_ocr(client, cfg):
    """Phase 3: cam2 detect front order + OCR read + parse. Stub."""
    raise NotImplementedError("Phase 3 detect_and_ocr - to be implemented")


def _pick_goods(client, cfg):
    """Phase 4: physically pick 5cm cube from order shelf. Stub."""
    raise NotImplementedError("Phase 4 pick_goods - to be implemented")


def _lift_and_carry(client, cfg):
    """Phase 5: lift + carry pose (prepare for Task 7). Stub."""
    raise NotImplementedError("Phase 5 lift_and_carry - to be implemented")


# ============================================================
# run() entry point
# ============================================================

def run(client: Optional[RuntimeApiClient] = None):
    """Task 6 main entry. Runs push-bar pose + sweep + reposition.

    Returns: {"ok": bool, "completed": [...], "order_list": [], "error": str}
    """
    cfg = _load_task6_config()

    if client is None:
        client = RuntimeApiClient()
    _ensure_runtime(client)
    _wait_infer_ready(client, timeout_s=30.0)

    completed = []
    order_list = []

    try:
        # ===== Phase 1+2: push-bar pose + sweep + reposition =====
        logger.info("=== Task 6: push-bar pose + sweep + reposition ===")
        _enter_push_bar_pose(client, cfg)
        completed.append("push_bar_pose")
        completed.append("sweep")
        completed.append("reposition")

        # ===== Phase 3: order detection via LLM (round 1) =====
        logger.info("=== Phase 3a: order reading round 1 (current position) ===")
        round1 = order_read_run()
        if round1.get("ok") and round1.get("orders"):
            logger.info("  [round1] %d orders:", len(round1["orders"]))
            for o in round1["orders"]:
                logger.info("    %s ← %s → %s号楼", o["name"], o["goods"], o["address"])
        else:
            logger.warning("  [round1] failed: %s", round1.get("error", "no orders"))
        completed.append("order_read_1")

        # ===== Phase 3b: adjust pose for second reading =====
        logger.info("=== Phase 3b: X→-150 hand→-70° Y→0 for round 2 ===")
        _move_x(client, -150.0, v_max_mms=80.0)
        logger.info("  X -> -150 mm done")
        _set_hand_angle(client, -70.0, speed=80)
        time.sleep(0.5)
        logger.info("  hand -> -70 deg done")
        _move_y(client, 0.0)
        logger.info("  Y -> 0 mm done")

        # ===== Phase 3c: order detection via LLM (round 2) with retry =====
        round2 = {"ok": False, "orders": [], "error": "no attempts"}
        for attempt, hand_angle in enumerate([-70.0, -55.0, -90.0]):
            logger.info("=== Phase 3c: order reading round 2 (hand=%.0f°) ===", hand_angle)
            if attempt > 0:
                _set_hand_angle(client, hand_angle, speed=80)
                time.sleep(0.5)
                logger.info("  hand -> %.0f deg (retry)", hand_angle)
            round2 = order_read_run()
            if round2.get("ok") and round2.get("orders"):
                logger.info("  [round2] %d orders (attempt %d, hand=%.0f°):",
                            len(round2["orders"]), attempt + 1, hand_angle)
                for o in round2["orders"]:
                    logger.info("    %s ← %s → %s号楼", o["name"], o["goods"], o["address"])
                break
            logger.warning("  [round2] attempt %d failed (hand=%.0f°): %s",
                           attempt + 1, hand_angle, round2.get("error", "no orders"))
        completed.append("order_read_2")

        # merge both rounds into order_list
        order_list = {
            "round1": round1.get("orders", []),
            "round2": round2.get("orders", []),
        }

        # ===== Phase 4: pick goods =====
        logger.info("=== Phase 4: pick goods ===")

        # 4a) 调整姿态: Y→-200, arm→-95°, hand→-10°
        logger.info("  adjusting pick pose: Y→-200 arm→-95 hand→-10")
        _move_y(client, -200.0)
        _set_arm_angle(client, -95.0, speed=40)
        time.sleep(1.5)
        _set_hand_angle(client, -10.0, speed=80)
        time.sleep(0.5)
        logger.info("  pick pose ready")

        # 4b) 底盘前进 18cm
        _chassis_move_for(client, dx_m=0.15, timeout=30.0)
        logger.info("  chassis forward 15cm done")

        # 4c) 蔬菜识别
        logger.info("  running veggie detection via LLM...")
        veggie_result = veggie_detect_run()
        veggie_items = veggie_result.get("items", []) if veggie_result.get("ok") else []
        if not veggie_items:
            logger.warning("  veggie detection: no items found, skip pick")
        else:
            logger.info("  veggie detection: %d items found", len(veggie_items))
            for it in veggie_items:
                logger.info("    [%s] %s conf=%s", it.get("position", "?"), it.get("name", "?"), it.get("confidence", "?"))

        # 4d) 从订单中提取需要的蔬菜列表
        ordered_goods = set()
        for rnd in [round1, round2]:
            for o in (rnd.get("orders") or []):
                g = o.get("goods", "")
                if g:
                    ordered_goods.add(g)
        logger.info("  ordered goods: %s", ordered_goods)

        # 4e) 匹配: 只取订单里有的蔬菜
        matched = [v for v in veggie_items if v.get("name") in ordered_goods]
        if not matched:
            logger.warning("  no veggie matches order, falling back to right-side items")
            matched = [v for v in veggie_items if _pos_to_side(v.get("position", "")) == "right"]
        else:
            logger.info("  matched %d veggies from order", len(matched))

        # 分左右, 右侧优先取
        right_targets = [v for v in matched if _pos_to_side(v.get("position", "")) == "right"]
        left_targets = [v for v in matched if _pos_to_side(v.get("position", "")) == "left"]
        pick_idx = 0

        # ── 先取右侧 ──
        for veg in right_targets[:2]:
            row = _pos_to_row(veg.get("position", ""))
            x_pos = _SHELF_X_BY_ROW[min(row, 3)]
            carry_y = -110.0 if pick_idx == 0 else -140.0
            drop_y = -40.0 if pick_idx == 0 else -80.0
            label = veg.get("name", f"item{pick_idx+1}")
            logger.info("  → [right] picking '%s' row=%d X=%.0f (pick %d)", label, row, x_pos, pick_idx+1)
            _pick_one_veggie(client, x_pos, carry_y, drop_y, label=label)
            completed.append(f"picked_{pick_idx+1}")
            pick_idx += 1

        # ── 左侧: 底盘前进 13cm 再取 ──
        if left_targets and pick_idx < 2:
            logger.info("  chassis forward 12cm for left-side veggies")
            _chassis_move_for(client, dx_m=0.12, timeout=30.0)
            for veg in left_targets[:2 - pick_idx]:
                row = _pos_to_row(veg.get("position", ""))
                x_pos = _SHELF_X_BY_ROW[min(row, 3)]
                carry_y = -110.0 if pick_idx == 0 else -140.0
                drop_y = -50.0 if pick_idx == 0 else -100.0
                label = veg.get("name", f"item{pick_idx+1}")
                logger.info("  → [left] picking '%s' row=%d X=%.0f (pick %d)", label, row, x_pos, pick_idx+1)
                _pick_one_veggie(client, x_pos, carry_y, drop_y, label=label)
                completed.append(f"picked_{pick_idx+1}")
                pick_idx += 1

        if pick_idx == 0:
            logger.warning("  no targets to pick")

        completed.append("pick_goods")

    except Exception as exc:
        logger.exception("get_order failed: %s", exc)
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
    print("get_order result:", result)