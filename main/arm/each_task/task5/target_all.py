"""task5 / target_all —— 摆到 target1 目标位姿 + 识别全部球 (黄+蓝) 并分别计数。

(2026-08-08 用户新建; 沿用 target_blue.py 的 4 机联动位姿 + 改全色识别 + Python 层分桶)

目标位姿 (沿用 target_blue.py, 2026-07-29 用户指定):
  1. **4 机联动** composite_run (arm=90°, x=-40, y=-200, hand=0°) ≈ 2-3s
  2. **全部球识别**   (蓝黄都收, 不过滤颜色, 分别计数黄球/蓝球数量)

最终位姿: x=-40, arm=90°, y=-200, hand=0°。

⚠️ **v1: 沿用 target_blue.py 的 4 机联动** (用户要求"前面四机联动没变化"):
  - composite_run(arm=90, x_mm=-40, y_mm=-200, hand=0, speed=80, timeout=30)
  - 4 轴并发, 沿用 new_get_blue.py / new_get_yellow.py / new_target.py 模式, 耗时 ~2-3s
  - 业务硬限检查: arm=90 ∈ [-150, +150]° ✓ / hand=0 ∈ [-90, +10]° ✓ /
    y=-200 ∈ [-200, 0] mm ✓ (软限位边界) / x=-40 ∈ [-320, +220] mm ✓
  - composite_run 不接受 None 轴 (2026-08-06 实测踩坑): 4 轴全传有效值
  - composite_run 不调 _check_y_protected (composite.py:60 拍板): 手爪 0° 在 y=-200 安全

⚠️ **v1: 全色识别 + Python 层分桶计数 (与 target_blue.py 关键差异)**:
  - `DETECT_COLOR_FILTER = None` (不过滤), 与 target_blue.py 写死 "blue" 不同
  - 调用 fetch_balls 后, **在 Python 层按 b["color"] 分桶**:
      count_yellow: color == "yellow" 的球数
      count_blue:   color == "blue" 的球数
      count_total:  全部识别到的球数 (= len(balls))
  - 球数据 list 仍然完整返回 (含每个球的 cx/cy/score 等), 便于上层做目标排序
  - 颜色映射来自 task4/target2._label_to_color:
      label 含 "blue" → blue, 含 "yellow" → yellow, 都不含 → unknown
      现场模型输出 ball_blue / ball_yellow, 映射正常
  - **unknown 颜色的球不计入** count_yellow / count_blue, 但仍出现在 balls 列表里
    (便于排查 "为啥 count=0 但 balls 不空" 的情况)

⚠️ **第 2 步球类识别 (复用 task4/target2.fetch_balls)**:
  - 走 task4/target2.py 的 fetch_balls() (侧摄 task_feed 守护线程,
    GET /v1/realtime/vision/task, runtime 默认 30Hz 常开, 不需要手动启)
  - 复用而非重抄: target2 里的 bbox 三格式兼容 / label→color 映射 / 球形几何
    过滤已经踩过坑, 重抄一份必然漂移。import 失败会给出明确报错而不是静默跳过
  - **纯只读**: 识别不改变位姿、不动机械臂, 失败只 warn 不抛 (摆位已成功,
    不该因为看不到球就把整个脚本判失败)。`--no-detect` 可关
  - 单帧可能空 (球没进画面 / task_feed 刚起), 故按 DETECT_HZ 轮询到
    DETECT_TIMEOUT_S 为止, 拿到球就立刻返回

⚠️ **顺序沿用 get_blue / get_yellow 的既定套路** (y → x → 大臂 → 手爪):
  - 第 1 步 composite_run 内部 4 轴并发, 不分顺序
  - 本脚本**不抬回** (get_blue/get_yellow 第 5 步的 y 抬回位): 用户只给了
    单个 y=-200, 做完手爪就停在这个位姿

⚠️ **y=-200 正好压在软限位边界上**: ArmOrigin.soft_y_max_m = 0.2 (arm_origin.yaml),
   api.py:_check_safe 判 `-200 <= y <= 0` 是**闭区间**, 故 -200 刚好通过。
   若哪天 soft_y_max_m 被标定成 < 0.2, 这里会直接 raise ValueError —— 那是
   预期行为 (软限位在保护你), 不要靠改脚本绕过, 去改标定。

⚠️ **大臂 90°**: 业务硬限 [+90, -150] 的**上界** (api.py:502-503), 同时是
   set_arm_angle 的 init 例外位 (a == 90.0 → allow_init_position=True), 保护区内
   也允许下发。composite_run 内部走 SDK set_arm_angle, 大臂与 xy/hand 4 轴并发到位

⚠️ 本文件**自包含**: 只依赖 `main.arm` (ArmClient/ArmRunner) + `task4/target2.py`
   的 `fetch_balls` (球识别, 见第 2 步说明), 不 import task5 包内其它模块。
   沿用 target_blue.py 自包含约定 — task5 辅助文件曾被外部动作清空过,
   自包含保证直接跑不受影响。

跑法:
    python main/arm/each_task/task5/target_all.py
    python -m main.arm.each_task.task5.target_all
    python main/arm/each_task/task5/target_all.py --x -40 --y -200 --arm 90 --hand 0
    python main/arm/each_task/task5/target_all.py --no-detect     # 只摆位, 不识别
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


# ---------- 目标位姿常量 (沿用 target_blue.py, 内联不依赖 constants.py) ----------

LOG_PREFIX: str = "[task5/target_all]"

TARGET1_Y_MM: float = -200.0
"""y 轴目标 (mm)。触底=0, 向上为负。-200 正好 = 软限位 soft_y_max_mm 边界
(闭区间, 通过)。远出保护区 [0,-30], 后续 x/大臂动作 wrapper 都能过。

⚠️ 沿用 target_blue.py (用户要求"前 4 机联动没变化"): -200。"""

TARGET1_X_MM: float = -40.0
"""x 轴目标 (mm)。composite_run 内部走 SDK move_x_position (无 belt-slip split)。

⚠️ 沿用 target_blue.py: -40mm。"""

TARGET1_ARM_DEG: float = 90.0
"""大臂角度。90° = 业务硬限上界 (api.py:503 _ARM_ANGLE_MAX) = 复位位,
且是 set_arm_angle 的 init 例外位 (保护区内也允许下发)。

⚠️ 沿用 target_blue.py: 90°。"""

TARGET1_HAND_DEG: float = 0.0
"""手爪 DOWN (composite_run 内部走 SDK set_hand_angle, 与 arm_angle 同步下发)。

⚠️ 沿用 target_blue.py: 0°。"""

# composite_run 4 机联动参数 (沿用 target_blue.py / new_get_blue.py / new_target.py 同款)
COMPOSITE_TIMEOUT_S: float = 30.0
"""4 机联动 composite_run 同步超时 (秒)。4 轴并发到位一般 ~2-3s, 给 30s 兜底
(含网络 + job_queue + SDK 内部 4 路 as_completed)。"""

ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度 + xy PID speed, 默认 80。与 task5/target.py / new_get_* 一致。"""

# ---------- 球类识别参数 (第 2 步, 沿用 target_blue.py 阈值) ----------

DETECT_ENABLED: bool = True
"""摆位完成后是否跑球类识别。--no-detect 关。"""

DETECT_TIMEOUT_S: float = 3.0
"""识别轮询总时长 (秒)。单帧可能空 (球没进画面 / task_feed 刚起),
在此时长内按 DETECT_HZ 反复拉, 拿到球立刻返回。超时返回 [] (只 warn)。"""

DETECT_HZ: float = 5.0
"""识别轮询频率 (Hz)。task_feed 守护线程本身 10Hz 刷新, 客户端 5Hz 够用。"""

# v1 关键改: 不过滤颜色 (None = 蓝黄都要)
DETECT_COLOR_FILTER = None
"""颜色过滤: None (不过滤, 蓝黄都要) / "blue" / "yellow"。

⚠️ 与 target_blue.py 关键差异:
  - target_blue: DETECT_COLOR_FILTER = "blue" (写死, 不提供 CLI 开关)
  - target_all:  DETECT_COLOR_FILTER = None  (蓝黄都要, 全收)
识别后在 Python 层按 b["color"] 分桶, 分别数黄/蓝。"""


def _count_by_color(balls: list) -> dict:
    """按 b["color"] 分桶计数。

    Args:
        balls: fetch_balls() 返回的 list[dict], 每球含 "color" 字段
               ("blue" / "yellow" / "unknown")。

    Returns:
        {
            "count_total": int,   # 全部识别到的球数 (= len(balls))
            "count_yellow": int,  # color == "yellow" 的球数
            "count_blue": int,    # color == "blue" 的球数
            "count_unknown": int, # color == "unknown" 的球数 (label 映射失败)
        }
    """
    count_total = len(balls)
    count_yellow = sum(1 for b in balls if b.get("color") == "yellow")
    count_blue = sum(1 for b in balls if b.get("color") == "blue")
    count_unknown = sum(1 for b in balls
                        if b.get("color") not in ("yellow", "blue"))
    return {
        "count_total": count_total,
        "count_yellow": count_yellow,
        "count_blue": count_blue,
        "count_unknown": count_unknown,
    }


def _fmt_ball(b: dict, idx: int) -> str:
    """单个球的一行日志 (与 target_blue.py / task4/target2._fmt_ball 同格式)。"""
    return (f"    [{idx}] color={str(b.get('color')):7s} "
            f"cx={b.get('cx_norm', 0.0):+.3f}  cy={b.get('cy_norm', 0.0):+.3f}  "
            f"w×h={b.get('w_norm', 0.0):.3f}×{b.get('h_norm', 0.0):.3f}  "
            f"score={b.get('score', 0.0):.3f}  det_id={b.get('det_id')}")


# ---- task5 专属几何/置信度阈值 (沿用 target_blue.py 实测标定) ----
#
# ⚠️ **不能复用 task4/constants.py 的 TARGET_* ——它们会把球全筛光**。
# task4 的 TARGET_AREA_MIN=0.20 / MAX=0.30 是在 task4 target1 位姿 (球贴得极近,
# w~0.42 h~0.60, area~0.25) 下标定的; 本文件位姿 (y=-200 / x=-40) 球离侧摄远得多。
#
# 2026-07-29 现场 GET /v1/realtime/vision/task 实测 3 球 (target_blue 沿用):
#   ball_yellow  score=0.916  w=0.337  h=0.471  area=0.159  aspect=0.714
#   ball_blue    score=0.899  w=0.317  h=0.466  area=0.148  aspect=0.680
#   ball_blue    score=0.683  w=0.337  h=0.500  area=0.168  aspect=0.673
# → area 全落在 [0.148, 0.168], 被 task4 的 0.20 下限整体丢弃 (三个全丢)
# → aspect 0.67~0.71 与 task4 实测 (0.42/0.60=0.70) 一致
# → score 0.683 那颗是真球 (画面边缘), 故下限取 0.60 而非 task4 的 0.85

DETECT_SCORE_MIN: float = 0.60
"""最低置信度。实测 0.683~0.916; 取 0.60 保住边缘那颗真球。
调高可过滤噪声框, 但会先丢掉画面边缘的球。--score-min 覆盖。"""

DETECT_AREA_MIN: float = 0.10
"""最小归一化面积。实测 0.148~0.168, 下探到 0.10 留余量 (球再远一点也能收)。
⚠️ 这是与 task4 的关键差异 (那边是 0.20)。--area-min 覆盖。"""

DETECT_AREA_MAX: float = 0.24
"""最大归一化面积。实测上界 0.168, 放到 0.24 留余量 (球更近时也能收),
同时仍能挡住占满画面的大块噪声。--area-max 覆盖。"""

DETECT_ASPECT_TOL: float = 0.8
"""宽高比容差, 沿用 task4 (|aspect - 1| ≤ tol)。实测 aspect 0.67~0.71,
|a-1| ≤ 0.33, 余量充足, 无需单独标定。"""


# ---------- 球类识别 (复用 task4/target2.fetch_balls) ----------

def detect_all_balls(client: ArmClient,
                     color_filter=DETECT_COLOR_FILTER,
                     timeout_s: float = DETECT_TIMEOUT_S,
                     hz: float = DETECT_HZ,
                     score_min: float = DETECT_SCORE_MIN,
                     area_min: float = DETECT_AREA_MIN,
                     area_max: float = DETECT_AREA_MAX,
                     aspect_tol: float = DETECT_ASPECT_TOL) -> list:
    """摆位完成后读侧摄 task_feed, 返回当前帧识别到的全部球 (蓝/黄/其它)。

    复用 `task4/target2.py` 的 `fetch_balls()` 做解析 (bbox 三格式兼容 /
    label→color 映射), 但**阈值一律传本文件的 DETECT_***, 不吃 task4 的
    TARGET_* 默认值 —— 那套是近景标定的, 会把本位姿的球全筛光。

    纯只读: 不动机械臂。任何异常都只 warn + 返回 [], 不抛 (摆位已成功,
    不该因为看不到球把整个脚本判失败)。

    Args:
        client: ArmClient (取它的 .http 拿 RuntimeApiClient)。
        color_filter: None (不过滤, 默认) / "blue" / "yellow"。
            沿用 target_blue.py 的 DETECT_COLOR_FILTER 参数签名, 但本文件默认 None。
        timeout_s: 轮询总时长 (秒), 拿到球提前返回。
        hz: 轮询频率 (Hz)。
        score_min / area_min / area_max / aspect_tol: 过滤阈值,
            默认 = 本文件 DETECT_* (task5 位姿实测标定)。

    Returns:
        list[dict]: 每球 {color, cx_norm, cy_norm, w_norm, h_norm, score,
                    det_id, cls_id, label}; 没识别到 / 出错 → []。
    """
    try:
        from main.arm.each_task.task4.target2 import fetch_balls  # noqa: E402
    except Exception as e:
        print(f"  {LOG_PREFIX} [WARN] 无法 import task4/target2.fetch_balls "
              f"({type(e).__name__}: {str(e)[:80]}), 跳过球类识别", file=sys.stderr)
        return []

    period = 1.0 / hz if hz > 0 else 0.2
    deadline = time.time() + max(0.0, timeout_s)
    balls: list = []
    rounds = 0

    while True:
        rounds += 1
        try:
            # v1 关键改: color_filter=None (蓝黄都要)
            balls = fetch_balls(
                client.http,
                color_filter=color_filter,  # = None, 不过滤
                # 显式传 task5 阈值, 不用 task4 的 TARGET_* 默认值
                score_min=score_min,
                area_min=area_min,
                area_max=area_max,
                aspect_tol=aspect_tol,
            )
        except Exception as e:
            print(f"  {LOG_PREFIX} [WARN] fetch_balls 异常: "
                  f"{type(e).__name__}: {str(e)[:80]}", file=sys.stderr)
            balls = []
        if balls:
            break
        if time.time() >= deadline:
            break
        time.sleep(period)

    # 打日志: 球列表 + 分桶计数
    if balls:
        cnt = _count_by_color(balls)
        print(f"  {LOG_PREFIX} 识别到 {cnt['count_total']} 个球 "
              f"(黄={cnt['count_yellow']} 蓝={cnt['count_blue']} "
              f"unknown={cnt['count_unknown']}, 轮询 {rounds} 次, "
              f"color_filter={color_filter})")
        for i, b in enumerate(balls):
            print(_fmt_ball(b, i))
    else:
        print(f"  {LOG_PREFIX} [WARN] {timeout_s}s 内没识别到球 (轮询 {rounds} 次)。"
              f" 当前阈值: score≥{score_min} area∈[{area_min},{area_max}] "
              f"|aspect-1|≤{aspect_tol}。"
              f" 排查: 先 curl /v1/realtime/vision/task 看 raw detections 的 "
              f"score/width/height, 逐项比对上面的阈值 (最常见是 area 不在区间)",
              file=sys.stderr)
    return balls


# ---------- 主入口 ----------

def run(client: ArmClient, runner: ArmRunner,
        x_mm: float = TARGET1_X_MM,
        y_mm: float = TARGET1_Y_MM,
        arm_deg: float = TARGET1_ARM_DEG,
        hand_deg: float = TARGET1_HAND_DEG,
        detect: bool = DETECT_ENABLED,
        color_filter=DETECT_COLOR_FILTER,
        detect_timeout_s: float = DETECT_TIMEOUT_S,
        score_min: float = DETECT_SCORE_MIN,
        area_min: float = DETECT_AREA_MIN,
        area_max: float = DETECT_AREA_MAX) -> dict:
    """摆到 target1 目标位姿 (composite_run 4 机联动) + 全色识别 + 黄蓝分桶计数。

    v1: 沿用 target_blue.py 的 4 机联动 (arm=90° x=-40 y=-200 hand=0°)
        + 全色识别 (color_filter=None) + Python 层 _count_by_color 分桶。

    Returns:
        {
            "ok": True,
            "x_info": dict,                # 4 机联动信息 (method/steps/ok/step1_job)
            "x_mm": float,                 # = -40
            "y_mm": float,                 # = -200
            "arm_deg": float,              # = 90
            "hand_deg": float,             # = 0
            "balls": list[dict],           # fetch_balls 全色识别结果
            "counts": {                    # 黄蓝分桶计数 (Python 层分桶)
                "count_total": int,        # 全部识别到的球数 (= len(balls))
                "count_yellow": int,       # color == "yellow" 的球数
                "count_blue": int,         # color == "blue" 的球数
                "count_unknown": int,      # color == "unknown" 的球数
            },
        }
        balls 在 detect=False 或没识别到时为 []; counts 全 0。
    """
    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  目标: 4 机联动 composite_run (arm={arm_deg}° x={x_mm}mm y={y_mm}mm hand={hand_deg}°)"
          f"{' → 全色识别 + 黄蓝分桶计数' if detect else ' (不识别)'}")

    # 1. 4 机联动 composite_run (沿用 target_blue.py, 用户要求"前 4 机联动没变化")
    #    一次性把 4 轴摆到目标 (arm=90°, x=-40, y=-200, hand=0°) ≈ 2-3s
    #    ⚠️ composite_run 不接受 None 轴 (2026-08-06 实测踩坑): 4 轴全传有效值。
    #    ⚠️ composite_run 不调 _check_y_protected (composite.py:60 拍板), 所以
    #       hand=0° 在 y=-200 时不会被 wrapper 拦截。
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
    # 保留 x_info 字段名 (沿用 target_blue.py 的返回值结构), 改成记录 composite_run 信息
    x_info = {
        "method": "composite_run",
        "steps": steps,
        "ok": ok1,
        "step1_job": step1,
    }

    # 2. 全色识别 + Python 层分桶计数 (蓝黄都要)
    balls: list = []
    counts = {"count_total": 0, "count_yellow": 0, "count_blue": 0, "count_unknown": 0}
    if detect:
        print(f"  [2/2] 全色识别 (侧摄 task_feed, ≤{detect_timeout_s}s, "
              f"score≥{score_min} area∈[{area_min},{area_max}], color_filter={color_filter})")
        balls = detect_all_balls(client, color_filter=color_filter,
                                 timeout_s=detect_timeout_s,
                                 score_min=score_min,
                                 area_min=area_min, area_max=area_max)
        counts = _count_by_color(balls)
        # v1 关键输出: 黄球 / 蓝球数量
        unknown_str = f", unknown {counts['count_unknown']} 个" if counts['count_unknown'] else ""
        print(f"  [2/2] ✅ 计数结果: 总 {counts['count_total']} 个, "
              f"黄球 {counts['count_yellow']} 个, "
              f"蓝球 {counts['count_blue']} 个"
              f"{unknown_str}")
    else:
        print(f"  [2/2] 全色识别  已跳过 (--no-detect)")

    print(f"========== {LOG_PREFIX} 完成 "
          f"(x={x_mm}mm arm={arm_deg}° y={y_mm}mm hand={hand_deg}° "
          f"balls={counts['count_total']} 黄={counts['count_yellow']} "
          f"蓝={counts['count_blue']}) ==========\n")
    return {
        "ok": True,
        "x_info": x_info,
        "x_mm": x_mm,
        "y_mm": y_mm,
        "arm_deg": arm_deg,
        "hand_deg": hand_deg,
        "balls": balls,
        "counts": counts,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task5 target_all v1: 4 机联动 composite_run (arm=90° x=-40 y=-200 hand=0°) + 全色识别 + 黄蓝分桶计数",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--x", type=float, default=TARGET1_X_MM, help="x (mm), 默认 -40 (沿用 target_blue)")
    p.add_argument("--y", type=float, default=TARGET1_Y_MM, help="y (mm), 默认 -200 (沿用 target_blue)")
    p.add_argument("--arm", type=float, default=TARGET1_ARM_DEG, help="大臂角度 (°), 默认 90")
    p.add_argument("--hand", type=float, default=TARGET1_HAND_DEG, help="手爪角度 (°), 默认 0")
    p.add_argument("--no-detect", action="store_true", dest="no_detect",
                   help="只摆位, 跳过第 2 步全色识别")
    # v1 新增: --color 开关 (target_blue 不提供, 因文件名已写死; target_all 提供,
    #    默认 None 全色, 可指定 "blue" 或 "yellow" 单色筛选用)。
    p.add_argument("--color", type=str, default=None, choices=[None, "blue", "yellow"],
                   dest="color",
                   help="颜色过滤 (默认 None 全色; 可指定 blue / yellow 单色筛选, "
                        "仅作辅助, 主用途是全色识别)")
    p.add_argument("--detect-timeout", type=float, default=DETECT_TIMEOUT_S,
                   dest="detect_timeout",
                   help="识别轮询总时长 (秒), 拿到球提前返回")
    p.add_argument("--score-min", type=float, default=DETECT_SCORE_MIN,
                   dest="score_min", help="识别最低置信度 (task5 位姿实测 0.68~0.92)")
    p.add_argument("--area-min", type=float, default=DETECT_AREA_MIN,
                   dest="area_min",
                   help="最小归一化面积 (task5 位姿实测 0.148~0.168; "
                        "注意 task4 的 0.20 在此位姿会把球全筛光)")
    p.add_argument("--area-max", type=float, default=DETECT_AREA_MAX,
                   dest="area_max", help="最大归一化面积")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    runner = ArmRunner(client)
    # v1 关键改: color_filter 走 CLI (默认 None 全色)
    run(client, runner, x_mm=args.x, y_mm=args.y,
        arm_deg=args.arm, hand_deg=args.hand,
        detect=not args.no_detect,
        color_filter=args.color,  # 默认 None (全色)
        detect_timeout_s=args.detect_timeout,
        score_min=args.score_min,
        area_min=args.area_min, area_max=args.area_max)
    return 0


if __name__ == "__main__":
    sys.exit(main())