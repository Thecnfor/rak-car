#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务五: 作物颜色分拣 (按颜色分高/低仓放置) —— **自包含实现**。

本文件**完整实现** task5 端到端 4 阶段流水线, **不引用** main.arm.each_task.task5
包内任何兄弟模块 (the_final / new_target / target_all / from_*_to_*.py / dipan)。
仅依赖基础设施 ``main.arm`` (ArmClient/ArmRunner) + 跨包复用
``main.arm.each_task.task4.target2.fetch_balls`` (侧摄球识别工具)。

⚙️ 业务流程 (5 阶段):
  ┌──────────────────────────────────────────────────────────────────────┐
  │ Phase 0: 底盘前进 315mm (预备, 2026-08-08 用户新增)                  │
  │ Phase 1: 4机联动 + 模型识别高仓颜色 → label = "blue" / "yellow"      │
  │ Phase 2: 4机联动 + 全色识别 + Python层黄蓝分桶计数                   │
  │ Phase 3a/d: matching 色 → 高塔 / opposite 色 → 低塔 (按球数循环)      │
  │ Phase 3c: 底盘后退 166mm (move_for [-0.166, 0, 0])                   │
  └──────────────────────────────────────────────────────────────────────┘

  - Phase 0: 底盘前进 315mm (移到识别位姿)
  - 高仓 = blue  → 蓝球进高塔 (N=count_blue 次) → 后退 → 黄球进低塔 (M=count_yellow 次)
  - 高仓 = yellow → 黄球进高塔 (N=count_yellow 次) → 后退 → 蓝球进低塔 (M=count_blue 次)

⚠️ **设计原则**:
  - **不引用** each_task/task5 包内任何兄弟模块 — 业务逻辑全部内联
  - 允许依赖 ``main.arm`` 基础设施 (ArmClient/ArmRunner + http.execute_*)
  - 允许跨包复用 ``task4/target2.fetch_balls`` (侧摄球识别, 已踩坑的解析逻辑)
  - 业务硬限 (composite_run 4 轴全传 / grasp 走 runner.* / move_y 保护区绕过)
    全部按 ARM_API.md §1.1 + setters.py:45 在本文件内重写

⚠️ **历史沿革 (本文件自我演进)**:
  - v0 (2026-08-08 之前): 薄封装 the_final.main
  - v1 (2026-08-08): 自包含, 4 阶段流水线全部内联, 不依赖 the_final 等兄弟模块

跑法:
    # 推荐 (包导入, sys.path 自动解决):
    from main.task import TASK_RUNNERS
    TASK_RUNNERS[5](client=None)

    # 单独跑:
    cd /home/jetson/workspace/rak-car
    python -m main.task.task5_sort
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Any, Dict, Optional

# ⚠️ 必须先加 repo 根进 sys.path, 再 import main.* (直接 python main/task/task5_sort.py 跑时
#    sys.path[0] 是 main/task/, 找不到 main 包)
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.api_client import RuntimeApiClient  # noqa: E402
from main.arm import ArmClient, ArmRunner  # noqa: E402


# ============================================================
# 流程常量 (沿用 task5 the_final.py 现场标定值)
# ============================================================

LOG_PREFIX: str = "[task5_sort]"

# ---------- Phase 1 (高仓识别) ----------
PHASE1_ARM_DEG: float = 90.0
PHASE1_X_MM: float = -28.0
PHASE1_Y_MM: float = -121.0
PHASE1_HAND_DEG: float = -58.0
PHASE1_DETECT_TIMEOUT_S: float = 20.0
PHASE1_SCORE_MIN: float = 0.50

# ---------- Phase 2 (全色识别) ----------
PHASE2_ARM_DEG: float = 90.0
PHASE2_X_MM: float = -40.0
PHASE2_Y_MM: float = -200.0
PHASE2_HAND_DEG: float = 0.0
PHASE2_DETECT_TIMEOUT_S: float = 3.0
PHASE2_DETECT_HZ: float = 5.0
# task5 位姿实测标定 (target_blue.py 2026-07-29):
PHASE2_SCORE_MIN: float = 0.60
PHASE2_AREA_MIN: float = 0.10
PHASE2_AREA_MAX: float = 0.24
PHASE2_ASPECT_TOL: float = 0.8

# ---------- Phase 3a (取球 + 投高塔) ----------
# 取球位姿 (new_get_blue / new_get_yellow 同款)
PICK_ARM_DEG: float = 85.0
PICK_Y_PRE_MM: float = -135.0   # 4 机联动 y (出保护区 55mm)
PICK_HAND_DEG: float = 10.0
PICK_GRASP_Y_MM: float = -66.0  # 吸气后下探
PICK_X_BLUE_MM: float = 0.0     # 蓝球 bin 位置
PICK_X_YELLOW_MM: float = -62.0 # 黄球 bin 位置

# 高塔投球位姿 (high_tower 同款)
HIGH_MOVE_Y_MM: float = -115.0          # 步骤 1: 出保护区
HIGH_COMPOSITE_Y_MM: float = -185.0     # 步骤 2
HIGH_COMPOSITE_X_MM: float = -50.0
HIGH_COMPOSITE_ARM_DEG: float = 90.0
HIGH_COMPOSITE_HAND_DEG: float = -82.0
HIGH_GRASP_X_MM: float = -145         # 步骤 3: 伸进塔
HIGH_RETRACT_X_MM: float = -70.0        # 步骤 5: 退回

# ---------- Phase 3d (取球 + 投低塔) ----------
LOW_MOVE_Y_MM: float = -176.0           # 步骤 1: 出保护区
LOW_COMPOSITE_Y_MM: float = -176.0
LOW_COMPOSITE_X_MM: float = -135.0
LOW_COMPOSITE_ARM_DEG: float = 90.0
LOW_COMPOSITE_HAND_DEG: float = 0.0     # DOWN (与 high_tower -82° UP 不同)

# ---------- Phase 3c (底盘后退) ----------
RETREAT_DIST_MM: float = -166.0         # ⚠️ 硬编码, 不读外部常量
RETREAT_MAX_VEL_MS: float = 0.10
RETREAT_TIMEOUT_FLOOR_S: float = 15.0   # floor=15s 防网络/队列抖动

# ---------- Phase 0 (底盘前进预备, 2026-08-08 用户新增) ----------
ADVANCE_DIST_MM: float = 325.0          # ⚠️ 硬编码, 不读外部常量
ADVANCE_MAX_VEL_MS: float = 0.10
ADVANCE_TIMEOUT_FLOOR_S: float = 15.0   # floor=15s 防网络/队列抖动

# ---------- 通用 ----------
COMPOSITE_SPEED: int = 80
COMPOSITE_TIMEOUT_S: float = 30.0
ANGLE_SPEED: int = 80
MOVE_X_V_MAX_MMS: float = 80.0
MOVE_TIMEOUT_S: float = 30.0
GRASP_TIMEOUT_S: float = 10.0

# ---------- 末尾收尾 4机联动 (2026-08-09 用户新增) ----------
FINAL_ARM_DEG: float = -80.0
FINAL_X_MM: float = -150.0
FINAL_Y_MM: float = -150.0
FINAL_HAND_DEG: float = -60.0


# ============================================================
# Phase 1: 4机联动 + 模型识别高仓颜色
# ============================================================

def _phase1_detect_high_tower(client: ArmClient, runner: ArmRunner,
                              phase1_pose_ready: bool = False) -> Dict[str, Any]:
    """Phase 1: 摆位 + 模型识别高仓色标。

    业务流程:
      [1/2] composite_run 4 机联动 to (arm=90°, x=-28, y=-121, hand=-58°)
      [2/2] 调 client.http.request_vision_task() 过滤 label_blue/label_yellow,
            取 score 最高者映射为 "blue" / "yellow" / "unknown"

    Returns:
        {"ok": True, "label": "blue"/"yellow"/"unknown",
         "step1_composite": dict, "label_info": dict,
         "final_pose": {"x_mm", "y_mm", "arm_deg", "hand_deg"}}

    Raises:
        RuntimeError: composite_run 失败 或 HTTP 调用失败。
    """
    print(f"\n========== {LOG_PREFIX} Phase 1: 4机联动 + 模型识别高仓色标 ==========")
    print(f"  目标: arm={PHASE1_ARM_DEG}° x={PHASE1_X_MM}mm y={PHASE1_Y_MM}mm "
          f"hand={PHASE1_HAND_DEG}° → 模型识别 (score_min={PHASE1_SCORE_MIN})")

    # [1/2] 4机联动 composite_run
    if phase1_pose_ready:
        print("  [1/2] ✅ 已由 task4→task5 handoff 完成 Phase 1 入场姿态，跳过重复 composite_run")
        step1 = {"status": "succeeded", "result": {"ok": True, "steps": {
            "arm": True, "x": True, "y": True, "hand": True,
        }}, "handoff": True}
    else:
        print(f"  [1/2] composite_run: arm={PHASE1_ARM_DEG:+.0f}° x={PHASE1_X_MM:.0f}mm "
              f"y={PHASE1_Y_MM:.0f}mm hand={PHASE1_HAND_DEG:+.0f}° "
              f"speed={COMPOSITE_SPEED} timeout={COMPOSITE_TIMEOUT_S:.0f}s")
        step1 = client.composite_run(
            arm=PHASE1_ARM_DEG,
            x_mm=PHASE1_X_MM,
            y_mm=PHASE1_Y_MM,
            hand=PHASE1_HAND_DEG,
            speed=COMPOSITE_SPEED,
            timeout=COMPOSITE_TIMEOUT_S,
        )
    ok1 = (isinstance(step1, dict)
           and step1.get("status") == "succeeded"
           and isinstance(step1.get("result"), dict)
           and step1["result"].get("ok", False))
    if not ok1:
        print(f"  [1/2] ❌ composite_run 失败: {step1}")
        raise RuntimeError(f"{LOG_PREFIX} Phase 1 composite_run 失败: {step1}")
    steps1 = step1["result"].get("steps", {}) if isinstance(step1.get("result"), dict) else {}
    if phase1_pose_ready:
        print(f"  [1/2] ✅ handoff Phase 1 入场姿态已确认  steps={steps1}")
    else:
        print(f"  [1/2] ✅ 4 轴并发到位  steps={steps1}")

    # [2/2] 模型识别 label
    print(f"  [2/2] 调 client.http.request_vision_task (timeout={PHASE1_DETECT_TIMEOUT_S}s)")
    # ⚠️ request_vision_task() 不接受 score_min kwarg — score 过滤在本文件做
    label_info = client.http.request_vision_task(
        timeout=PHASE1_DETECT_TIMEOUT_S,
    )
    # label_info 是 dict (runtime 原始返回); 兼容 list 形式
    if isinstance(label_info, dict):
        detections = label_info.get("detections", [])
    elif isinstance(label_info, list):
        detections = label_info
    else:
        detections = []
    # 本地按 score_min + label 过滤
    label_candidates = [d for d in detections
                        if d.get("label") in ("label_blue", "label_yellow")
                        and d.get("score", 0.0) >= PHASE1_SCORE_MIN]
    if not label_candidates:
        label = "unknown"
        winner = {}
    else:
        winner = max(label_candidates, key=lambda d: d.get("score", 0.0))
        model_label = winner.get("label", "")
        label = "blue" if "blue" in model_label else ("yellow" if "yellow" in model_label else "unknown")
    print(f"  [2/2] label={label!r}  winner={winner}")

    return {
        "ok": True,
        "label": label,
        "step1_composite": step1,
        "label_info": winner if winner else {},
        "final_pose": {
            "x_mm": PHASE1_X_MM, "y_mm": PHASE1_Y_MM,
            "arm_deg": PHASE1_ARM_DEG, "hand_deg": PHASE1_HAND_DEG,
        },
    }


# ============================================================
# Phase 2: 4机联动 + 全色识别 + Python 层黄蓝分桶
# ============================================================

def _phase2_count_balls(client: ArmClient, runner: ArmRunner) -> Dict[str, Any]:
    """Phase 2: 摆位 + 全色识别 + 黄蓝分桶计数。

    业务流程:
      [1/2] composite_run 4 机联动 to (arm=90°, x=-40, y=-200, hand=0°)
      [2/2] 调 fetch_balls(client.http, color_filter=None) 全色识别
            按 b["color"] 分桶 (count_yellow, count_blue, count_unknown)

    Returns:
        {"ok": True, "balls": list[dict],
         "counts": {count_total, count_yellow, count_blue, count_unknown},
         "final_pose": {...}}
    """
    print(f"\n========== {LOG_PREFIX} Phase 2: 4机联动 + 全色识别 + 黄蓝分桶 ==========")
    print(f"  目标: composite_run (arm={PHASE2_ARM_DEG}° x={PHASE2_X_MM}mm "
          f"y={PHASE2_Y_MM}mm hand={PHASE2_HAND_DEG}°) → 全色识别")

    # [1/2] composite_run
    print(f"  [1/2] composite_run: arm={PHASE2_ARM_DEG:+.0f}° x={PHASE2_X_MM:.0f}mm "
          f"y={PHASE2_Y_MM:.0f}mm hand={PHASE2_HAND_DEG:+.0f}°")
    step1 = client.composite_run(
        arm=PHASE2_ARM_DEG,
        x_mm=PHASE2_X_MM,
        y_mm=PHASE2_Y_MM,
        hand=PHASE2_HAND_DEG,
        speed=COMPOSITE_SPEED,
        timeout=COMPOSITE_TIMEOUT_S,
    )
    ok1 = (isinstance(step1, dict)
           and step1.get("status") == "succeeded"
           and isinstance(step1.get("result"), dict)
           and step1["result"].get("ok", False))
    if not ok1:
        print(f"  [1/2] ❌ composite_run 失败: {step1}")
        raise RuntimeError(f"{LOG_PREFIX} Phase 2 composite_run 失败: {step1}")
    print(f"  [1/2] ✅ 4 轴并发到位")

    # [2/2] 全色识别 (跨包复用 task4/target2.fetch_balls)
    print(f"  [2/2] 全色识别 (fetch_balls color_filter=None, ≤{PHASE2_DETECT_TIMEOUT_S}s)")
    try:
        from main.arm.each_task.task4.target2 import fetch_balls  # 跨包复用
    except Exception as e:
        print(f"  [2/2] [WARN] 无法 import task4/target2.fetch_balls: {e}")
        balls = []
    else:
        period = 1.0 / PHASE2_DETECT_HZ if PHASE2_DETECT_HZ > 0 else 0.2
        deadline = time.time() + PHASE2_DETECT_TIMEOUT_S
        balls = []
        rounds = 0
        while True:
            rounds += 1
            try:
                balls = fetch_balls(
                    client.http,
                    color_filter=None,
                    score_min=PHASE2_SCORE_MIN,
                    area_min=PHASE2_AREA_MIN,
                    area_max=PHASE2_AREA_MAX,
                    aspect_tol=PHASE2_ASPECT_TOL,
                )
            except Exception as e:
                print(f"  [2/2] [WARN] fetch_balls 异常: {e}")
                balls = []
            if balls:
                break
            if time.time() >= deadline:
                break
            time.sleep(period)

    # Python 层黄蓝分桶
    counts = {
        "count_total": len(balls),
        "count_yellow": sum(1 for b in balls if b.get("color") == "yellow"),
        "count_blue": sum(1 for b in balls if b.get("color") == "blue"),
        "count_unknown": sum(1 for b in balls
                              if b.get("color") not in ("yellow", "blue")),
    }
    print(f"  [2/2] ✅ 识别 {counts['count_total']} 个球 (黄={counts['count_yellow']} "
          f"蓝={counts['count_blue']} unknown={counts['count_unknown']}, 轮询 {rounds} 次)")
    for i, b in enumerate(balls):
        print(f"    [{i}] color={str(b.get('color')):7s} cx={b.get('cx_norm', 0.0):+.3f} "
              f"cy={b.get('cy_norm', 0.0):+.3f} score={b.get('score', 0.0):.3f}")

    return {
        "ok": True,
        "balls": balls,
        "counts": counts,
        "step1_composite": step1,
        "final_pose": {
            "x_mm": PHASE2_X_MM, "y_mm": PHASE2_Y_MM,
            "arm_deg": PHASE2_ARM_DEG, "hand_deg": PHASE2_HAND_DEG,
        },
    }


# ============================================================
# Phase 3a: 取球 (取色 X) → 投高塔 (5 步)
# ============================================================

def _pick_ball_cycle_high(client: ArmClient, runner: ArmRunner, color: str) -> Dict[str, Any]:
    """单球: 取色 (蓝/黄) → 投高塔。

    业务流程 (3 + 5 = 8 步):
      Phase A (3 步): 4机联动取色位姿 → 吸气 → 下探 (球吸住)
      Phase B (5 步): y 抬出保护区 → 4机联动投球位姿 → x 伸进塔 →
                     释放真空 → x 退回

    Args:
        color: "blue" 或 "yellow", 决定取球 x 位置 (蓝 0, 黄 -62)

    Returns:
        {"ok": True, "final_pose": {...}}
    """
    x_pick = PICK_X_BLUE_MM if color == "blue" else PICK_X_YELLOW_MM
    print(f"\n========== {LOG_PREFIX} 取{color}球 → 投高塔 (8 步) ==========")

    # ---- Phase A: 取球 (3 步) ----
    print(f"  [A1/3] composite_run 取{color}位姿: arm={PICK_ARM_DEG:+.0f}° "
          f"x={x_pick:.0f}mm y={PICK_Y_PRE_MM:.0f}mm hand={PICK_HAND_DEG:+.0f}°")
    a1 = client.composite_run(
        arm=PICK_ARM_DEG, x_mm=x_pick, y_mm=PICK_Y_PRE_MM, hand=PICK_HAND_DEG,
        speed=COMPOSITE_SPEED, timeout=COMPOSITE_TIMEOUT_S,
    )
    if not (isinstance(a1, dict) and a1.get("status") == "succeeded"
            and isinstance(a1.get("result"), dict) and a1["result"].get("ok", False)):
        raise RuntimeError(f"{LOG_PREFIX} 取球 composite_run 失败: {a1}")
    print(f"  [A2/3] runner.grasp(True) 吸气")
    a2 = runner.grasp(True, timeout=GRASP_TIMEOUT_S)
    print(f"  [A3/3] runner.move_y({PICK_GRASP_Y_MM}mm) 下探到 grasp_y")
    a3 = runner.move_y(PICK_GRASP_Y_MM, timeout=MOVE_TIMEOUT_S)

    # ---- Phase B: 投高塔 (5 步) ----
    print(f"  [B1/5] runner.move_y({HIGH_MOVE_Y_MM}mm) 出保护区")
    b1 = runner.move_y(HIGH_MOVE_Y_MM, timeout=MOVE_TIMEOUT_S)
    print(f"  [B2/5] composite_run 投球位姿: arm={HIGH_COMPOSITE_ARM_DEG:+.0f}° "
          f"x={HIGH_COMPOSITE_X_MM:.0f}mm y={HIGH_COMPOSITE_Y_MM:.0f}mm "
          f"hand={HIGH_COMPOSITE_HAND_DEG:+.0f}°")
    b2 = client.composite_run(
        arm=HIGH_COMPOSITE_ARM_DEG, x_mm=HIGH_COMPOSITE_X_MM,
        y_mm=HIGH_COMPOSITE_Y_MM, hand=HIGH_COMPOSITE_HAND_DEG,
        speed=COMPOSITE_SPEED, timeout=COMPOSITE_TIMEOUT_S,
    )
    if not (isinstance(b2, dict) and b2.get("status") == "succeeded"
            and isinstance(b2.get("result"), dict) and b2["result"].get("ok", False)):
        raise RuntimeError(f"{LOG_PREFIX} 投高塔 composite_run 失败: {b2}")
    print(f"  [B3/5] runner.move_x({HIGH_GRASP_X_MM}mm) 伸进塔")
    b3 = runner.move_x(HIGH_GRASP_X_MM, v_max_mms=MOVE_X_V_MAX_MMS, timeout=MOVE_TIMEOUT_S)
    print(f"  [B4/5] runner.grasp(False) 释放真空 (球入塔)")
    b4 = runner.grasp(False, timeout=GRASP_TIMEOUT_S)
    print(f"  [B5/5] runner.move_x({HIGH_RETRACT_X_MM}mm) 退回")
    b5 = runner.move_x(HIGH_RETRACT_X_MM, v_max_mms=MOVE_X_V_MAX_MMS, timeout=MOVE_TIMEOUT_S)

    return {
        "ok": True,
        "phase_a": {"step1": a1, "step2_grasp": a2, "step3_move_y": a3},
        "phase_b": {"step1": b1, "step2_composite": b2, "step3_move_x": b3,
                    "step4_grasp": b4, "step5_move_x": b5},
        "final_pose": {
            "x_mm": HIGH_RETRACT_X_MM, "y_mm": HIGH_COMPOSITE_Y_MM,
            "arm_deg": HIGH_COMPOSITE_ARM_DEG, "hand_deg": HIGH_COMPOSITE_HAND_DEG,
        },
    }


# ============================================================
# Phase 3d: 取球 (取色 X) → 投低塔 (3 步)
# ============================================================

def _pick_ball_cycle_low(client: ArmClient, runner: ArmRunner, color: str) -> Dict[str, Any]:
    """单球: 取色 (蓝/黄) → 投低塔。

    业务流程 (3 + 3 = 6 步):
      Phase A (3 步): 4机联动取色位姿 → 吸气 → 下探 (球吸住)
      Phase B (3 步): y 抬出保护区 → 4机联动投球位姿 → 释放真空 (球入低塔)
        注: 低塔无 x 推进 / 回退 (low_tower.py 同款)

    Args:
        color: "blue" 或 "yellow", 决定取球 x 位置 (蓝 0, 黄 -62)

    Returns:
        {"ok": True, "final_pose": {...}}
    """
    x_pick = PICK_X_BLUE_MM if color == "blue" else PICK_X_YELLOW_MM
    print(f"\n========== {LOG_PREFIX} 取{color}球 → 投低塔 (6 步) ==========")

    # ---- Phase A: 取球 (3 步) ----
    print(f"  [A1/3] composite_run 取{color}位姿: arm={PICK_ARM_DEG:+.0f}° "
          f"x={x_pick:.0f}mm y={PICK_Y_PRE_MM:.0f}mm hand={PICK_HAND_DEG:+.0f}°")
    a1 = client.composite_run(
        arm=PICK_ARM_DEG, x_mm=x_pick, y_mm=PICK_Y_PRE_MM, hand=PICK_HAND_DEG,
        speed=COMPOSITE_SPEED, timeout=COMPOSITE_TIMEOUT_S,
    )
    if not (isinstance(a1, dict) and a1.get("status") == "succeeded"
            and isinstance(a1.get("result"), dict) and a1["result"].get("ok", False)):
        raise RuntimeError(f"{LOG_PREFIX} 取球 composite_run 失败: {a1}")
    print(f"  [A2/3] runner.grasp(True) 吸气")
    a2 = runner.grasp(True, timeout=GRASP_TIMEOUT_S)
    print(f"  [A3/3] runner.move_y({PICK_GRASP_Y_MM}mm) 下探到 grasp_y")
    a3 = runner.move_y(PICK_GRASP_Y_MM, timeout=MOVE_TIMEOUT_S)

    # ---- Phase B: 投低塔 (3 步) ----
    print(f"  [B1/3] runner.move_y({LOW_MOVE_Y_MM}mm) 出保护区")
    b1 = runner.move_y(LOW_MOVE_Y_MM, timeout=MOVE_TIMEOUT_S)
    print(f"  [B2/3] composite_run 投球位姿: arm={LOW_COMPOSITE_ARM_DEG:+.0f}° "
          f"x={LOW_COMPOSITE_X_MM:.0f}mm y={LOW_COMPOSITE_Y_MM:.0f}mm "
          f"hand={LOW_COMPOSITE_HAND_DEG:+.0f}°")
    b2 = client.composite_run(
        arm=LOW_COMPOSITE_ARM_DEG, x_mm=LOW_COMPOSITE_X_MM,
        y_mm=LOW_COMPOSITE_Y_MM, hand=LOW_COMPOSITE_HAND_DEG,
        speed=COMPOSITE_SPEED, timeout=COMPOSITE_TIMEOUT_S,
    )
    if not (isinstance(b2, dict) and b2.get("status") == "succeeded"
            and isinstance(b2.get("result"), dict) and b2["result"].get("ok", False)):
        raise RuntimeError(f"{LOG_PREFIX} 投低塔 composite_run 失败: {b2}")
    print(f"  [B3/3] runner.grasp(False) 释放真空 (球入低塔)")
    b3 = runner.grasp(False, timeout=GRASP_TIMEOUT_S)

    return {
        "ok": True,
        "phase_a": {"step1": a1, "step2_grasp": a2, "step3_move_y": a3},
        "phase_b": {"step1": b1, "step2_composite": b2, "step3_grasp": b3},
        "final_pose": {
            "x_mm": LOW_COMPOSITE_X_MM, "y_mm": LOW_COMPOSITE_Y_MM,
            "arm_deg": LOW_COMPOSITE_ARM_DEG, "hand_deg": LOW_COMPOSITE_HAND_DEG,
        },
    }


# ============================================================
# Phase 0: 底盘前进 315mm (预备, 2026-08-08 用户新增)
# ============================================================

def _advance_315mm(client: ArmClient) -> Dict[str, Any]:
    """底盘前进 315mm (硬编码 +315mm, 不依赖 dipan.DEFAULT_DIST_MM)。

    走 client.http.execute_car_action("move_for", [+0.315, 0, 0], sync=True),
    限速 0.10 m/s, timeout 自适应 (floor=15s 防网络/队列抖动)。

    ⚠️ **与 _retreat_166mm 同款**: dist_mm / max_vel / timeout 全本地硬编码,
       不读 dipan.DEFAULT_*。task5 流水线对每个底盘距离都有强约定, 跟随外部常量
       漂移会导致车冲过头或不到位。
    """
    dist_m = ADVANCE_DIST_MM / 1000.0  # +0.315
    adaptive_timeout = max(
        ADVANCE_TIMEOUT_FLOOR_S,
        abs(ADVANCE_DIST_MM) / 1000.0 / max(ADVANCE_MAX_VEL_MS, 0.01) + 10.0,
    )
    direction = "向后" if dist_m < 0 else ("向前" if dist_m > 0 else "原地")
    print(f"\n========== {LOG_PREFIX} Phase 0: 底盘{direction} {abs(ADVANCE_DIST_MM):.0f}mm "
          f"(x_offset={dist_m:+.3f}m, max_v={ADVANCE_MAX_VEL_MS:.2f}m/s, "
          f"timeout={adaptive_timeout:.1f}s) ==========")
    t0 = time.perf_counter()
    job = client.http.execute_car_action(
        "move_for",
        [dist_m, 0.0, 0.0],
        max_velocities=[ADVANCE_MAX_VEL_MS, ADVANCE_MAX_VEL_MS, 0.0],
        sync=True,
        timeout=adaptive_timeout,
    )
    dt = time.perf_counter() - t0
    ok = isinstance(job, dict) and job.get("status") == "succeeded"
    status = job.get("status") if isinstance(job, dict) else None
    error = job.get("error") if isinstance(job, dict) else None
    print(f"  ✅ 底盘{direction} {abs(ADVANCE_DIST_MM):.0f}mm 完成 "
          f"(status={status!r}, 耗时={dt:.2f}s)")
    if not ok:
        raise RuntimeError(
            f"{LOG_PREFIX} Phase 0 底盘前进失败 (status={status!r}, error={error!r})"
        )
    return job


# ============================================================
# Phase 3c: 底盘后退 166mm (硬编码, 不读外部常量)
# ============================================================

def _retreat_166mm(client: ArmClient) -> Dict[str, Any]:
    """底盘后退 166mm (硬编码 -166mm, 不依赖 dipan.DEFAULT_DIST_MM)。

    走 client.http.execute_car_action("move_for", [-0.166, 0, 0], sync=True),
    限速 0.10 m/s, timeout 自适应 (floor=15s 防网络/队列抖动)。
    """
    dist_m = RETREAT_DIST_MM / 1000.0  # -0.166
    adaptive_timeout = max(
        RETREAT_TIMEOUT_FLOOR_S,
        abs(RETREAT_DIST_MM) / 1000.0 / max(RETREAT_MAX_VEL_MS, 0.01) + 10.0,
    )
    direction = "向后" if dist_m < 0 else ("向前" if dist_m > 0 else "原地")
    print(f"\n========== {LOG_PREFIX} Phase 3c: 底盘{direction} {abs(RETREAT_DIST_MM):.0f}mm "
          f"(x_offset={dist_m:+.3f}m, max_v={RETREAT_MAX_VEL_MS:.2f}m/s, "
          f"timeout={adaptive_timeout:.1f}s) ==========")
    t0 = time.perf_counter()
    job = client.http.execute_car_action(
        "move_for",
        [dist_m, 0.0, 0.0],
        max_velocities=[RETREAT_MAX_VEL_MS, RETREAT_MAX_VEL_MS, 0.0],
        sync=True,
        timeout=adaptive_timeout,
    )
    dt = time.perf_counter() - t0
    ok = isinstance(job, dict) and job.get("status") == "succeeded"
    status = job.get("status") if isinstance(job, dict) else None
    error = job.get("error") if isinstance(job, dict) else None
    print(f"  ✅ 底盘{direction} {abs(RETREAT_DIST_MM):.0f}mm 完成 "
          f"(status={status!r}, 耗时={dt:.2f}s)")
    if not ok:
        raise RuntimeError(
            f"{LOG_PREFIX} Phase 3c 底盘后退失败 (status={status!r}, error={error!r})"
        )
    return job


# ============================================================
# Phase 4: 末尾收尾 4机联动 (2026-08-09 用户新增)
# ============================================================

def _final_composite_pose(client: ArmClient) -> Dict[str, Any]:
    """task5 末尾收尾: 4机联动 4 轴并行一次到位。

    业务流程:
      [1/1] composite_run 4 机联动 to (arm=-80°, x=-150, y=-150, hand=-60°)

    设计: pipeline 跑完后固定摆这个位姿, 作为"分拣完成 → 离场" 的过渡姿态
    (后续 waypoint 会由 orchestrator 接 home reset 接管)。4 轴全传有效值,
    不依赖 SDK 是否支持 partial (composite_run 不接受 None 偏量)。

    Args:
        client: ArmClient 实例。

    Returns:
        composite_run 原始返回 dict (含 status/result/steps)。

    Raises:
        RuntimeError: composite_run 失败 (status != "succeeded")。
    """
    print(f"\n========== {LOG_PREFIX} Phase 4: 末尾 4机联动 收尾位姿 ==========")
    print(f"  目标: composite_run arm={FINAL_ARM_DEG:+.0f}° "
          f"x={FINAL_X_MM:.0f}mm y={FINAL_Y_MM:.0f}mm "
          f"hand={FINAL_HAND_DEG:+.0f}° (speed={COMPOSITE_SPEED}, "
          f"timeout={COMPOSITE_TIMEOUT_S:.0f}s)")
    t0 = time.perf_counter()
    res = client.composite_run(
        arm=FINAL_ARM_DEG,
        x_mm=FINAL_X_MM,
        y_mm=FINAL_Y_MM,
        hand=FINAL_HAND_DEG,
        speed=COMPOSITE_SPEED,
        timeout=COMPOSITE_TIMEOUT_S,
    )
    dt = time.perf_counter() - t0
    ok = (isinstance(res, dict)
          and res.get("status") == "succeeded"
          and isinstance(res.get("result"), dict)
          and res["result"].get("ok", False))
    status = res.get("status") if isinstance(res, dict) else None
    steps = (res.get("result") or {}).get("steps", {}) if isinstance(res, dict) else {}
    if ok:
        print(f"  ✅ 4 轴并发到位  steps={steps}  (status={status!r}, 耗时={dt:.2f}s)")
    else:
        print(f"  ❌ composite_run 失败: status={status!r}  res={res}")
        raise RuntimeError(f"{LOG_PREFIX} Phase 4 末尾 4机联动 失败: {res}")
    return res


# ============================================================
# 主入口: 4 阶段编排
# ============================================================

def _run_pipeline(client: ArmClient, runner: ArmRunner,
                  prev_ball_counts: Optional[Dict[str, int]] = None,
                  phase1_pose_ready: bool = False) -> Dict[str, Any]:
    """task5 端到端 4 阶段流水线 (内联实现)。

    Args:
        prev_ball_counts: task4 采集到的球色统计 {"blue": N, "yellow": M},
                           提供则跳过 Phase 2 全色识别, 直接用此统计。

    Returns:
        {"ok": True, "high_color": str, "counts": dict,
         "phase3a_runs": list, "phase3c_retreat": dict, "phase3d_runs": list,
         "final_pose": {...}}
    """
    t_total = time.perf_counter()

    print(f"\n{'='*70}\n  {LOG_PREFIX} run (端到端分拣入库流水线, 4 阶段)\n{'='*70}")
    print(f"  Phase 0: 底盘前进 315mm (预备)")
    print(f"  Phase 1: 4机联动 + 模型识别高仓颜色")
    print(f"  Phase 2: 4机联动 + 全色识别 + 黄蓝分桶")
    print(f"  Phase 3: 底盘 + 高/低塔循环 (matching → 高, opposite → 低)")

    # ========== Phase 0: 底盘前进 315mm (预备, 2026-08-08 用户新增) ==========
    phase0_advance = _advance_315mm(client)

    # ========== Phase 1: 识别高仓颜色 ==========
    phase1 = _phase1_detect_high_tower(client, runner,
                                       phase1_pose_ready=phase1_pose_ready)
    high_color = phase1.get("label", "unknown")
    print(f"\n  ✅ Phase 1 完成  高仓颜色 = {high_color!r}")
    if high_color not in ("blue", "yellow"):
        raise RuntimeError(
            f"{LOG_PREFIX} Phase 1 高仓颜色无法识别 ({high_color!r}), "
            f"仅支持 'blue' / 'yellow'"
        )

    # ========== Phase 2: 识别球数 ==========
    if prev_ball_counts:
        # task4 已采集统计, 跳过 Phase 2 全色识别
        n_blue = int(prev_ball_counts.get("blue", 0))
        n_yellow = int(prev_ball_counts.get("yellow", 0))
        n_total = n_blue + n_yellow
        counts = {
            "count_total": n_total,
            "count_yellow": n_yellow,
            "count_blue": n_blue,
            "count_unknown": 0,
        }
        print(f"\n  ✅ Phase 2 跳过 (使用 task4 统计: 黄={n_yellow} 蓝={n_blue})")
    else:
        phase2 = _phase2_count_balls(client, runner)
        counts = phase2["counts"]
        n_blue = counts["count_blue"]
        n_yellow = counts["count_yellow"]
        n_total = counts["count_total"]
        print(f"\n  ✅ Phase 2 完成  球数: 总 {n_total}, 黄 {n_yellow}, 蓝 {n_blue}, "
              f"unknown {counts['count_unknown']}")

    # ========== Phase 3: 分拣 + 底盘后退 ==========
    if high_color == "blue":
        # 蓝球进高塔
        phase3a_runs = []
        for i in range(n_blue):
            print(f"\n  [{LOG_PREFIX}] Phase 3a 第 {i+1}/{n_blue} 次: 蓝球进高塔")
            phase3a_runs.append(_pick_ball_cycle_high(client, runner, "blue"))
        phase3c_retreat = _retreat_166mm(client)
        # 黄球进低塔
        phase3d_runs = []
        for i in range(n_yellow):
            print(f"\n  [{LOG_PREFIX}] Phase 3d 第 {i+1}/{n_yellow} 次: 黄球进低塔")
            phase3d_runs.append(_pick_ball_cycle_low(client, runner, "yellow"))
    else:  # yellow
        # 黄球进高塔
        phase3a_runs = []
        for i in range(n_yellow):
            print(f"\n  [{LOG_PREFIX}] Phase 3a 第 {i+1}/{n_yellow} 次: 黄球进高塔")
            phase3a_runs.append(_pick_ball_cycle_high(client, runner, "yellow"))
        phase3c_retreat = _retreat_166mm(client)
        # 蓝球进低塔
        phase3d_runs = []
        for i in range(n_blue):
            print(f"\n  [{LOG_PREFIX}] Phase 3d 第 {i+1}/{n_blue} 次: 蓝球进低塔")
            phase3d_runs.append(_pick_ball_cycle_low(client, runner, "blue"))

    final_pose = (phase3d_runs[-1]["final_pose"] if phase3d_runs
                  else (phase3a_runs[-1]["final_pose"] if phase3a_runs
                        else phase2["final_pose"]))

    # ========== Phase 4: 末尾收尾 4机联动 (2026-08-09 用户新增) ==========
    phase4_final = _final_composite_pose(client)
    final_pose = {
        "x_mm": FINAL_X_MM, "y_mm": FINAL_Y_MM,
        "arm_deg": FINAL_ARM_DEG, "hand_deg": FINAL_HAND_DEG,
    }

    elapsed = time.perf_counter() - t_total
    print(f"\n{'='*70}\n  {LOG_PREFIX} 完成 (高仓={high_color}, 球数 黄={n_yellow} 蓝={n_blue}, "
          f"高塔={len(phase3a_runs)} 次, 低塔={len(phase3d_runs)} 次, "
          f"总耗时 {elapsed:.3f}s)\n{'='*70}\n")

    return {
        "ok": True,
        "high_color": high_color,
        "phase1_high_tower_label": phase1,
        "phase2_ball_counts": phase2,
        "counts": counts,
        "phase3a_runs": phase3a_runs,
        "phase3c_retreat": phase3c_retreat,
        "phase3d_runs": phase3d_runs,
        "phase4_final_composite": phase4_final,
        "final_pose": final_pose,
    }


# ============================================================
# 入口: TASK_RUNNERS 兼容
# ============================================================

def run(client: Optional[RuntimeApiClient] = None,
        prev_ball_counts: Optional[Dict[str, int]] = None,
        phase1_pose_ready: bool = False) -> Dict[str, Any]:
    """任务五主入口: 自包含实现 4 阶段流水线 (TASK_RUNNERS 兼容)。

    与 the_final/main.py 不同, 本函数**自包含**, 不调用 the_final.main,
    不引用 each_task/task5 包内任何兄弟模块。

    Args:
        client: 形式参数, 实际被忽略 (本函数内部自建 ArmClient / ArmRunner)。
        prev_ball_counts: task4 采集到的球色统计 {"blue": N, "yellow": M},
                           提供则跳过 Phase 2 全色识别。
        phase1_pose_ready: task4→task5 巡航 handoff 已完成 Phase 1 入场姿态，
                           True 时 Phase 1 仅识别高仓色标，不重复摆臂。
    Returns:
        Dict: {"ok": bool, "task": "task5_sort", "rc": 0/1, "detail": str}
    """
    arm_client = ArmClient.connect()
    if not arm_client.ping():
        raise RuntimeError("机械臂 runtime 未在线, 请检查 arm_feed 守护进程")
    runner = ArmRunner(arm_client)

    try:
        result = _run_pipeline(arm_client, runner,
                               prev_ball_counts=prev_ball_counts,
                               phase1_pose_ready=phase1_pose_ready)
        return {
            "ok": True,
            "task": "task5_sort",
            "rc": 0,
            "detail": (f"task5 完成: 高仓={result['high_color']}, "
                       f"球数 黄={result['counts']['count_yellow']} "
                       f"蓝={result['counts']['count_blue']}"),
        }
    except Exception as e:
        return {
            "ok": False,
            "task": "task5_sort",
            "rc": 1,
            "detail": f"task5 失败: {type(e).__name__}: {str(e)[:200]}",
        }


def main(argv=None) -> int:
    """CLI 入口: 直接跑 task5 流水线。"""
    parser = argparse.ArgumentParser(
        description=(
            "task5_sort v1 (自包含): 端到端分拣入库流水线 (4 阶段)\n"
            "  Phase 1: 4机联动 + 模型识别高仓颜色\n"
            "  Phase 2: 4机联动 + 全色识别 + 黄蓝分桶\n"
            "  Phase 3: 底盘 + 高/低塔循环\n"
            "  不引用 each_task/task5 包内任何兄弟模块"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    args = parser.parse_args(argv)
    result = run()
    print(f"\n{LOG_PREFIX} main 退出: {result}")
    return result["rc"]


if __name__ == "__main__":
    sys.exit(main())