"""task5 / new_target —— 4 机联动摆位 + 用 runtime 视觉模型识别高仓色标 label (yellow/blue)。

业务要求 (2026-08-07 用户):
  1. 仿 ``new_get_blue.py`` / ``new_get_yellow.py`` 的 4 机联动 composite_run 模式,
     把臂一次性摆到 "观察位姿":
         arm = +90°
         x   = -28 mm  (中位偏左, 2026-08-07 现场拍板)
         y   = -121 mm (出保护区, 高位看色标, 2026-08-07 现场拍板)
         hand = -58° (mid 偏后, 不挡画面下沿, 2026-08-07 现场拍板)
  2. 调用 ``client.http.request_vision_task()`` (POST /v1/vision/task) 触发一次侧视模型推理,
     在返回的 ``detections`` 中筛 ``label="label_blue"`` / ``label="label_yellow"``,
     按 score 排序取最高分者:
         - ``label_blue``  → label = "blue"
         - ``label_yellow`` → label = "yellow"
         - 没有 / 都不过阈值 → "unknown"
  3. 返回 winner 的 label + score + bbox (调试)

⚠️ **v2 改: 不再使用 HSV 算法** (2026-08-07 用户拍板)
  - 旧 detect_high_tower_color 走 cv2 HSV 阈值 + RETR_CCOMP 内洞优选
  - Image #2 现场: 地面黄色球 (~3000px) 像素多盖过高塔色标 (~1000-2000px) → 误判 yellow
  - 加 ``AUTO_TAG_MAX_CENTER_Y_RATIO=0.75`` 位置过滤 + 蓝色 H 范围放宽 [90, 135]
    仍不能完全解决 (色标候选根本没找到时无能为力)
  - **改用 runtime /v1/vision/task 模型**: 模型在 Image #2 上以 0.92 置信度识别 blue,
    直接用模型 label 输出即可, 不再做二次 HSV 推断

⚠️ **composite_run 业务硬限 (ARM_API.md §1.1 / setters.py:45)**:
  - arm = +90 ∈ [-150, +150]° ✓
  - hand = -58 ∈ [-90, +10]° ✓ (mid 位)
  - y = -121 ∈ [-200, 0] mm ✓ (出保护区 [0, -80] 41mm)
  - x = -28 ∈ [-320, +220] mm ✓ (中位偏左, 软限位内)

⚠️ **composite_run 不接受 None 轴 (2026-08-06 现场实测)**:
  - 业务层 composite.py:56-68 虽然把 None 透传给底层, 但 SDK 不识别 None →
    `result.steps={None轴: False, 有值轴: True}`, 整个 job `result.ok=False`
  - **正确用法**: 4 轴全传有效值, "不动的轴"靠"传相同值"实现 (SDK 内部走 no-op)

⚠️ 本文件**自包含**: 只依赖 `main.arm` (ArmClient/ArmRunner),
   **不 import task5 包内其它模块** (target.py / grasp_5 / *_tower)。
   沿用 new_get_*.py 的自包含约定 — task5 辅助文件曾被外部动作清空过,
   自包含保证 `python new_target.py` 直接跑不受影响。

跑法:
    python main/arm/each_task/task5/new_target.py
    python -m main.arm.each_task.task5.new_target
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from main.arm import ArmClient, ArmRunner  # noqa: E402


# ---------- 观察位姿常量 (4 机联动目标) ----------

LOG_PREFIX: str = "[task5/new_target]"

NEW_TARGET_X_MM: float = -28.0
"""观察 x 目标 (-28mm, 中位偏左 28mm)。

2026-08-07 现场拍板: 实测正中 (0mm) 看色标略偏, 左移 28mm 让色标进入画面中央区域。
距物理墙 (-300mm 量级) 较远, 不撞墙。"""

NEW_TARGET_Y_MM: float = -121.0
"""观察 y 目标 (-121mm, 出保护区 [0, -80] 41mm, 高位看色标)。

2026-08-07 现场拍板: 比 target.py v3+ 默认 -100 更低 21mm (离色标更近, 画面占比更大)。

物理沿革:
  - 触底 (0) → 抬 121mm: 给色标稳定视角, 避免手爪吸盘遮画面下沿
  - 比下探位 (-135) 高 14mm: 比取球位略高, 给识别留稳定余量
"""

NEW_TARGET_ARM_DEG: float = 90.0
"""大臂 +90° (复位位, 业务硬限上界)。

大臂顶起让相机能直接看高仓色标; 不依赖保护区允许 (在保护区外)。
2026-08-07 现场维持 90° (与之前一致, 仅 xy/hand 调整)。
"""

NEW_TARGET_HAND_DEG: float = -58.0
"""手爪 -58° (mid 偏后 13°, 不挡画面下沿)。

2026-08-07 现场拍板: 比 target.py v3+ 默认 -45 更收 13° (实测 -45 时手爪前端略入画面底部)。
不是 init 例外位 (-90 是 init), 但在业务硬限 [-90, +10] 内合法。
"""

COMPOSITE_TIMEOUT_S: float = 30.0
"""4 机联动 composite_run 同步超时 (秒)。4 轴并发到位一般 ~2-3s, 给 30s 兜底
(含网络 + job_queue + SDK 内部 4 路 as_completed)。"""

ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度 + xy PID speed, 默认 80。与 task5/target.py / new_get_* 一致。"""


# ---------- 模型识别常量 ----------

# 业务层认可的高仓色标 label 集合 (model class names)
LABEL_BLUE: str = "label_blue"
LABEL_YELLOW: str = "label_yellow"
"""模型输出的 label_blue / label_yellow 字符串。

来源: runtime 视觉模型 class 配置 (config_car.yml → infer_cfg.task.classes)。
不在此白名单的 detections 一律忽略。
"""

# 模型输出 label → 业务 label 映射
MODEL_LABEL_TO_COLOR: dict = {
    LABEL_BLUE: "blue",
    LABEL_YELLOW: "yellow",
}
"""模型 label (str) → 业务 label (str) 映射。

⚠️ 这是识别逻辑的唯一映射点。模型说 label_blue 就是 blue,
模型说 label_yellow 就是 yellow, 不做二次推断。
"""

LABEL_SCORE_MIN: float = 0.50
"""最低 score 阈值 (winner.score < 此值 → unknown)。

兜底防误检: 模型可能给一些边界框很低分 (e.g. 0.3) — 不应作为最终判定。
"""

DETECT_TIMEOUT_S: float = 20.0
"""POST /v1/vision/task 单次推理 HTTP 超时 (秒)。

runtime 默认 timeout=20, 给 20s 兜底。模型冷启动可能 5-15s。"""


# ---------- 模型识别主函数 ----------

def detect_high_tower_label(
    client: ArmClient,
    *,
    timeout: float = DETECT_TIMEOUT_S,
    score_min: float = LABEL_SCORE_MIN,
    sort_pos: tuple = (0.0, 0.0),
    limit_x: float = 1.0,
    limit_y: float = 1.0,
) -> dict:
    """调用 runtime 视觉模型识别高仓色标 label: yellow / blue / unknown。

    算法 (2026-08-07 v2 简化):
      1. ``client.http.request_vision_task()`` → POST /v1/vision/task
         触发一次侧视目标检测 (cam2)
      2. 在返回的 ``detections`` 中过滤 ``label in (label_blue, label_yellow)``
      3. 按 ``score`` 排序, 取最高分者:
         - winner.label == "label_blue"  → label = "blue"
         - winner.label == "label_yellow" → label = "yellow"
         - 没有候选 / winner.score < score_min → "unknown"

    Args:
        client: ArmClient (用 ``client.http.request_vision_task()`` 调 vision API)
            ArmClient 是薄封装, 实际方法在 ``client.http`` (RuntimeApiClient) 上。
        timeout: 推理 HTTP 超时 (秒)
        score_min: winner.score 最低阈值, < 此值 → unknown
        sort_pos: 传给 /v1/vision/task 的 sort_pos 字段 (用于按距离排序)
        limit_x: 传给 /v1/vision/task 的 limit_x 字段 (归一化坐标限幅)
        limit_y: 传给 /v1/vision/task 的 limit_y 字段

    Returns:
        {
            "color": "blue" | "yellow" | "unknown",     # 业务 label
            "label": str | None,                          # 模型原始 label (e.g. "label_blue")
            "score": float,                               # winner.score
            "bbox_norm": dict | None,                     # winner.bbox_norm
            "bbox_pixels": dict | None,                   # winner.bbox_pixels
            "all_label_detections": list[dict],           # 所有 label_blue/label_yellow 候选
            "total_detections": int,                      # 模型本次返回的所有 detection 数
            "elapsed_ms": float,                          # 总耗时
            "raw_response_ok": bool,                      # 模型原始 ok 字段
        }
    """
    t0 = time.perf_counter()

    # 1. 触发模型推理 (POST /v1/vision/task)
    # ⚠️ request_vision_task 在 RuntimeApiClient 上 (api_client.py:54), 不在 ArmClient 上。
    #    ArmClient.http 是底层 RuntimeApiClient 实例, 走 client.http.request_vision_task()。
    resp = client.http.request_vision_task(
        sort_pos=sort_pos,
        limit_x=limit_x,
        limit_y=limit_y,
        timeout=timeout,
    )
    raw_ok = bool(resp.get("ok", False))
    if not raw_ok:
        raise RuntimeError(
            f"{LOG_PREFIX} /v1/vision/task 失败: ok={raw_ok} resp={resp}"
        )

    detections: list = resp.get("detections", []) or []

    # 2. 过滤 label_blue / label_yellow
    label_candidates: list = []
    for det in detections:
        model_label = det.get("label", "")
        if model_label in MODEL_LABEL_TO_COLOR:
            label_candidates.append(det)

    # 3. 按 score 降序, 取最高分
    label_candidates.sort(key=lambda d: float(d.get("score", 0.0)), reverse=True)

    color = "unknown"
    winner: Optional[dict] = None
    winner_label: Optional[str] = None
    winner_score: float = 0.0
    winner_bbox_norm: Optional[dict] = None
    winner_bbox_pixels: Optional[dict] = None

    if label_candidates:
        winner = label_candidates[0]
        winner_score = float(winner.get("score", 0.0))
        winner_label = winner.get("label")
        if winner_score >= score_min and winner_label in MODEL_LABEL_TO_COLOR:
            color = MODEL_LABEL_TO_COLOR[winner_label]
            winner_bbox_norm = winner.get("bbox_norm")
            winner_bbox_pixels = winner.get("bbox_pixels")

    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    return {
        "color": color,
        "label": winner_label,
        "score": winner_score,
        "bbox_norm": winner_bbox_norm,
        "bbox_pixels": winner_bbox_pixels,
        "all_label_detections": label_candidates,
        "total_detections": len(detections),
        "elapsed_ms": elapsed_ms,
        "raw_response_ok": raw_ok,
    }


# ---------- 主入口 ----------

def run(client: ArmClient, runner: ArmRunner,
        x_mm: float = NEW_TARGET_X_MM,
        y_mm: float = NEW_TARGET_Y_MM,
        arm_deg: float = NEW_TARGET_ARM_DEG,
        hand_deg: float = NEW_TARGET_HAND_DEG,
        *,
        detect_timeout: float = DETECT_TIMEOUT_S,
        score_min: float = LABEL_SCORE_MIN) -> dict:
    """4 机联动 composite_run 摆位 + 模型推理识别 label。

    业务流程 (仿 new_get_*.py 的 3 步拆 + 末尾识别):
      1. **4 机联动** composite_run(arm=90°, x=-28, y=-121, hand=-58°)
         仿 ``main/task/task1_seeding.py::_switch_to_place_pose`` 模式,
         4 轴并发到位 (~2-3s)。
      2. **模型识别** detect_high_tower_label() 调 ``client.http.request_vision_task()``,
         过滤 ``label_blue`` / ``label_yellow``, 取 score 最高者映射为 blue / yellow。

    Args:
        client: ArmClient (composite_run + request_vision_task 在这里)
        runner: ArmRunner (本流程不直接用, 但保留参数对齐 new_get_* 签名)
        x_mm: composite_run 目标 x (mm), 默认 -28
        y_mm: composite_run 目标 y (mm), 默认 -121
        arm_deg: composite_run 目标大臂角度 (°), 默认 90
        hand_deg: composite_run 目标手爪角度 (°), 默认 -58
        detect_timeout: 模型推理 HTTP 超时 (秒), 默认 20
        score_min: winner.score 最低阈值, 默认 0.50

    Returns:
        {
            "ok": True,                     # 4 机联动 + detect 都成功
            "step1_composite": dict,        # 4 机联动 composite_run 原始 job dict
            "label_info": dict,             # detect_high_tower_label 完整结果
            "label": str,                   # 简化字段: "yellow" / "blue" / "unknown"
            "final_pose": {                 # 终态 (预期值, 不重读 state)
                "x_mm": float,
                "y_mm": float,
                "arm_deg": float,
                "hand_deg": float,
            },
        }

    Raises:
        RuntimeError: Step 1 composite_run 失败 (status != "succeeded"),
            或 Step 2 /v1/vision/task 失败。
    """
    print(f"\n========== {LOG_PREFIX} run (摆位 + 模型识别 label) ==========")
    print(f"  目标: arm={arm_deg}° x={x_mm}mm y={y_mm}mm hand={hand_deg}° "
          f"→ 模型识别 (score_min={score_min})")

    # ========== Step 1: 4 机联动 composite_run (仿 main/task/task1_seeding.py) ==========
    # 仿 _switch_to_place_pose / _init_step2_s_pose 同款 4 轴并发模式。
    # ⚠️ composite_run 不接受 None 轴, 4 轴全传有效值, "不动的轴"靠"传相同值"实现。
    # ⚠️ composite_run 内部不调 _check_y_protected (composite.py:60 拍板), 所以
    #    hand=-58° 在 y=-121 时不会被保护区拦截 (虽然 y=-121 本就在保护区外)。
    print(f"  [1/2] composite_run (4 机联动): arm={arm_deg:+.0f}° x={x_mm:.0f}mm "
          f"y={y_mm:.0f}mm hand={hand_deg:+.0f}°  speed={ANGLE_SPEED} "
          f"timeout={COMPOSITE_TIMEOUT_S:.0f}s")
    step1 = client.composite_run(
        arm=arm_deg,
        x_mm=x_mm,
        y_mm=y_mm,
        hand=hand_deg,
        speed=ANGLE_SPEED,
        timeout=COMPOSITE_TIMEOUT_S,
    )
    ok1 = (
        isinstance(step1, dict)
        and step1.get("status") == "succeeded"
        and isinstance(step1.get("result"), dict)
        and step1["result"].get("ok", False)
    )
    if not ok1:
        # ⚠️ 通用踩坑: job["result"]["ok"] 不是 job["ok"] — job dict 和
        # composite_run SDK 返回的 result dict 是嵌套结构, 详见
        # [[composite-run-no-partial-2026-08-06]]
        print(f"  [1/2] ❌ composite_run 失败: {step1}")
        raise RuntimeError(
            f"{LOG_PREFIX} Step 1 composite_run 4 机联动失败: {step1}"
        )
    # 检查 4 轴全部 ok (现场实测 SDK 会把 None 轴判 False, 所以这里再核一次 steps)
    steps = step1["result"].get("steps", {}) if isinstance(step1.get("result"), dict) else {}
    print(f"  [1/2] ✅ 4 轴并发到位 (~2-3s)  steps={steps}")

    # ========== Step 2: detect_high_tower_label (调 /v1/vision/task 模型识别) ==========
    print(f"\n  [2/2] detect_high_tower_label  POST /v1/vision/task "
          f"timeout={detect_timeout}s score_min={score_min}")
    try:
        label_info = detect_high_tower_label(
            client, timeout=detect_timeout, score_min=score_min,
        )
    except Exception as e:
        print(f"  [2/2] ❌ 模型识别失败: {type(e).__name__}: {e}")
        raise

    label = label_info["color"]
    print(f"        label        = {label}")
    print(f"        model_label  = {label_info['label']!r}")
    print(f"        score        = {label_info['score']:.3f}")
    print(f"        bbox_norm    = {label_info['bbox_norm']}")
    print(f"        bbox_pixels  = {label_info['bbox_pixels']}")
    print(f"        total_det    = {label_info['total_detections']}  "
          f"label_det={len(label_info['all_label_detections'])}  "
          f"elapsed={label_info['elapsed_ms']:.0f}ms")
    # 调试: 列出所有候选 label_blue / label_yellow (按 score 排序)
    if len(label_info["all_label_detections"]) > 1:
        print(f"        candidates   = (按 score 降序):")
        for i, det in enumerate(label_info["all_label_detections"][:5]):
            print(f"          [{i}] label={det.get('label')!r}  "
                  f"score={float(det.get('score', 0.0)):.3f}  "
                  f"bbox={det.get('bbox_pixels')}")

    print(f"\n========== {LOG_PREFIX} 完成: label={label} "
          f"(arm={arm_deg}° x={x_mm}mm y={y_mm}mm hand={hand_deg}°) ==========\n")
    return {
        "ok": True,
        "step1_composite": step1,
        "label_info": label_info,
        "label": label,
        "final_pose": {
            "x_mm": x_mm,
            "y_mm": y_mm,
            "arm_deg": arm_deg,
            "hand_deg": hand_deg,
        },
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description=(
            "task5 new_target v2: 4 机联动 composite_run 摆位 + runtime 视觉模型识别 label\n"
            "  仿 new_get_*.py 的 4 机联动模式\n"
            "  → POST /v1/vision/task 调模型 → 过滤 label_blue/label_yellow → 输出 blue/yellow/unknown\n"
            "  默认: arm=+90° x=-28 y=-121 hand=-58° → 模型识别 score_min=0.5"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--x", type=float, default=NEW_TARGET_X_MM, help="composite_run 目标 x (mm)")
    p.add_argument("--y", type=float, default=NEW_TARGET_Y_MM, help="composite_run 目标 y (mm)")
    p.add_argument("--arm", type=float, default=NEW_TARGET_ARM_DEG,
                   help="composite_run 目标大臂角度 (°)")
    p.add_argument("--hand", type=float, default=NEW_TARGET_HAND_DEG,
                   help="composite_run 目标手爪角度 (°)")
    p.add_argument("--detect-timeout", type=float, default=DETECT_TIMEOUT_S,
                   dest="detect_timeout", help="模型推理 HTTP 超时 (秒)")
    p.add_argument("--score-min", type=float, default=LABEL_SCORE_MIN,
                   dest="score_min", help="winner.score 最低阈值, < 此值判 unknown")
    return p


def main(argv=None) -> int:
    t_total_start = time.perf_counter()
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    if not client.ping():
        raise RuntimeError("机械臂 runtime 未在线, 请检查 arm_feed 守护进程")
    runner = ArmRunner(client)
    result = run(client, runner,
        x_mm=args.x, y_mm=args.y,
        arm_deg=args.arm, hand_deg=args.hand,
        detect_timeout=args.detect_timeout,
        score_min=args.score_min)
    elapsed = time.perf_counter() - t_total_start
    print(f"========== {LOG_PREFIX} 识别结果: label={result['label']}  "
          f"总耗时: {elapsed:.3f} s ==========")
    return 0


if __name__ == "__main__":
    sys.exit(main())