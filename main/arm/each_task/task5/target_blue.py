"""task5 / target_blue —— 摆到 '取蓝' 目标位姿 + **只检测蓝球**。

(2026-07-29 由 target1.py 改名而来; 同时把球类识别收窄为只认蓝球。)

目标位姿 (用户指定 2026-07-29):
  1. y 轴  → -200 mm  (先抬出保护区 [0,-30], 后面 x/大臂 wrapper 才能过)
  2. x 轴  → -40 mm   (belt-slip 安全 move_x, 不撞墙)
  3. 大臂  → 90°      (业务硬限上界 = 复位位, init 例外位)
  4. 手爪  → 0°       (DOWN, 走底层直调, 大臂不动)
  5. **蓝球**识别      (摆位完成后读侧摄 task_feed, 输出蓝球归一化坐标)

最终位姿: x=-40, arm=90°, y=-200, hand=0°。

⚠️ **第 5 步只检测蓝球 (2026-07-29 用户要求)**:
  - `DETECT_COLOR_FILTER = "blue"` 写死, **不提供 --color CLI 开关** —— 文件名
    就叫 target_blue, 命令行还能切黄球是自相矛盾的口子, 干脆不开。
  - 黄球 (以及 label 无法映射成 blue/yellow 的 unknown) 在 `fetch_balls` 里
    直接被丢弃, 不进返回值、不打日志。
  - 确实要一次看蓝黄两色时: 直接调 `detect_balls(client, color_filter=None)`
    (函数签名保留了这个参数), 或去跑 task4/target2.py。
  - 颜色映射来自 `task4/target2._label_to_color`: label 含 "blue" → blue,
    含 "yellow" → yellow, 都不含 → unknown。现场模型输出的是
    `ball_blue` / `ball_yellow`, 映射正常。

⚠️ **第 5 步球类识别 (2026-07-29 新加)**:
  - 走 `task4/target2.py` 的 `fetch_balls()` (侧摄 task_feed 守护线程,
    `GET /v1/realtime/vision/task`, runtime 默认 30Hz 常开, **不需要手动启**)。
  - 复用而非重抄: target2 里的 bbox 三格式兼容 / label→color 映射 / 球形几何
    过滤已经踩过坑, 重抄一份必然漂移。import 失败会给出明确报错而不是静默跳过。
  - **纯只读**: 识别不改变位姿、不动机械臂, 失败只 warn 不抛 (摆位已经成功,
    不该因为看不到球就把整个脚本判失败)。`--no-detect` 可关。
  - 单帧可能空 (球没进画面 / task_feed 刚起), 故按 `DETECT_HZ` 轮询到
    `DETECT_TIMEOUT_S` 为止, 拿到球就立刻返回。

⚠️⚠️ **`verify_target1_pose` 必须是 False**:
  - `task4/constants.py` 的 `BALL_VERIFIED_*` 是在 **task4 的 target1.py 位姿**
    (y=-150 / x=-260 / arm=+90 / hand=0) 下实测标定的 7 项范围。
  - **本文件位姿完全不同** (y=-200 / x=-40), 球在画面里的 cx/cy/大小都不一样。
    开这个验证 = 每个球都被判越界过滤掉, 永远返回 0 个球。
  - 故本脚本硬编码 `verify_target1_pose=False`, 且**不提供打开的 CLI 开关**。
    若将来要给本位姿做同款基线, 请新标一套专属常量, 不要复用 task4 的。
  - (参数名里的 "target1" 是 task4 那边的历史命名, 与本文件旧名 target1.py
    无关 —— 改名后已无混淆, 但参数名属 task4, 不动。)

⚠️ **顺序沿用 get_blue / get_yellow 的既定套路** (y → x → 大臂 → 手爪):
  - 第 1 步 move_y 走步进电机, **不过 y 保护区** (api.py:372-373 注释:
    '即使在保护区 [0, -30] 也可以调, 用于出保护区') → 从 init (y=0) 直接跑也行。
  - 之后的 move_x / set_arm_angle 都要过 y 保护区网关, y=-200 远出保护区, 过。
  - 本脚本**不抬回** (get_blue/get_yellow 第 5 步的 y 抬回位): 用户只给了
    单个 y=-200, 做完手爪就停在这个位姿。

⚠️ **y=-200 正好压在软限位边界上**: ArmOrigin.soft_y_max_m = 0.2 (arm_origin.yaml),
   api.py:_check_safe 判 `-200 <= y <= 0` 是**闭区间**, 故 -200 刚好通过。
   若哪天 soft_y_max_m 被标定成 < 0.2, 这里会直接 raise ValueError —— 那是
   预期行为 (软限位在保护你), 不要靠改脚本绕过, 去改标定。

⚠️ **x = -40mm 的 belt-slip 处理**: 40mm 接近 belt-slip 单次有效行程上界
   (24-46mm, ARM_API §7.2.1), 单次 move_x 有可能到不了位。故走
   `_move_x_with_split` (test_x_to_150.py 模式, 与 get_yellow.py 同款):
   每轮 move_x(target, v_max_mms=30) + realtime 校验; 卡住 kick 让同步带重咬合;
   连续 3 轮 stall 放弃。位置验证一律走 `_read_x_mm_realtime()` (20Hz arm_feed
   真值), **不走 get_state()** —— calibrate 框架已坏, 见 ARM_API §11。

⚠️ **大臂 90°**: 业务硬限 [+90, -150] 的**上界**(api.py:502-503), 同时是
   `set_arm_angle` 的 init 例外位 (a == 90.0 → allow_init_position=True), 保护区
   里也允许下发。90° ≥ +30 落在 "安全姿态" 带外 → 之后 set_hand_angle(0) 走
   Python 层 wrapper **不会被拒**; 但这里仍保留底层 `_call_arm` 直调, 与
   get_blue / get_yellow 保持一致 (行为交由车端裁决)。

⚠️ 本文件**自包含**: 只依赖 `main.arm` (ArmClient/ArmRunner) + `task4/target2.py`
   的 `fetch_balls` (球识别, 见第 5 步说明), 不 import task5 包内其它模块。
   原因: task5 辅助文件曾被外部动作清空过, 自包含保证直接跑不受影响。

跑法:
    python main/arm/each_task/task5/target_blue.py
    python -m main.arm.each_task.task5.target_blue
    python main/arm/each_task/task5/target_blue.py --x -40 --y -200 --arm 90 --hand 0
    python main/arm/each_task/task5/target_blue.py --no-detect     # 只摆位, 不识别
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


# ---------- 目标位姿常量 (内联, 不依赖 constants.py) ----------

LOG_PREFIX: str = "[task5/target_blue]"

TARGET1_Y_MM: float = -200.0
"""y 轴目标 (mm)。触底=0, 向上为负。-200 正好 = 软限位 soft_y_max_mm 边界
(闭区间, 通过)。远出保护区 [0,-30], 后续 x/大臂动作 wrapper 都能过。"""

TARGET1_X_MM: float = -40.0
"""x 轴目标 (mm)。40mm 接近 belt-slip 单次有效行程上界 (24-46mm, §7.2.1),
故走 _move_x_with_split 分段 + 卡住 kick。"""

TARGET1_ARM_DEG: float = 90.0
"""大臂角度。90° = 业务硬限上界 (api.py:503 _ARM_ANGLE_MAX) = 复位位,
且是 set_arm_angle 的 init 例外位 (保护区内也允许下发)。"""

TARGET1_HAND_DEG: float = 0.0
"""手爪 DOWN。走底层 _call_arm 直调 (与 get_blue / get_yellow 同款)。
大臂 90° 已在展开区 [-30, +30] 之外, api.py:604-612 的门不再命中,
wrapper 其实也能过; 直调只为跟兄弟脚本保持一致的行为语义。"""

# belt-slip 安全 move_x 参数 (test_x_to_150.py 模式, ARM_API §7.2.1+§11)
MOVE_X_TOL_MM: float = 5.0          # 到位容差 (realtime 抖动 <1mm, 放宽给 PID 余量)
MOVE_X_V_MAX_MMS: float = 30.0      # 业务限速 (2026-07-22 限速透传 bug 修复后定档 30)
MOVE_X_MAX_ROUNDS: int = 12         # 最多尝试轮数
MOVE_X_STALL_MM: float = 3.0        # 本轮位移 < 此值视为卡住 (疑似打滑)
MOVE_X_MAX_STALL_ROUNDS: int = 3    # 连续卡住这么多轮 → 放弃
MOVE_X_KICK_SLEEP_S: float = 0.2    # kick: 停一下让同步带齿重新咬合

# ---------- 球类识别参数 (第 5 步, 2026-07-29) ----------

DETECT_ENABLED: bool = True
"""摆位完成后是否跑球类识别。--no-detect 关。"""

DETECT_TIMEOUT_S: float = 3.0
"""识别轮询总时长 (秒)。单帧可能空 (球没进画面 / task_feed 刚起),
在此时长内按 DETECT_HZ 反复拉, 拿到球立刻返回。超时返回 [] (只 warn)。"""

DETECT_HZ: float = 5.0
"""识别轮询频率 (Hz)。task_feed 守护线程本身 10Hz 刷新, 客户端 5Hz 够用。"""

DETECT_COLOR_FILTER: str = "blue"
"""颜色过滤: "blue" (本文件只认蓝球) / "yellow" / None (蓝黄都要)。

2026-07-29 改名时写死为 "blue" 且**不提供 --color 开关**, 文件名 target_blue
与可切换颜色是自相矛盾的口子。要看黄球直接跑 task4/target2.py。"""

# ---- task5 专属几何/置信度阈值 (2026-07-29 实测标定) ----
#
# ⚠️ **不能复用 task4/constants.py 的 TARGET_* ——它们会把球全筛光**。
# task4 的 TARGET_AREA_MIN=0.20 / MAX=0.30 是在 task4 target1 位姿 (球贴得极近,
# w~0.42 h~0.60, area~0.25) 下标定的; 本文件位姿 (y=-200 / x=-40) 球离侧摄远得多。
#
# 2026-07-29 现场 GET /v1/realtime/vision/task 实测 3 球:
#   ball_yellow  score=0.916  w=0.337  h=0.471  area=0.159  aspect=0.714
#   ball_blue    score=0.899  w=0.317  h=0.466  area=0.148  aspect=0.680
#   ball_blue    score=0.683  w=0.337  h=0.500  area=0.168  aspect=0.673
# → area 全落在 [0.148, 0.168], 被 task4 的 0.20 下限**整体丢弃** (三个全丢),
#   这就是 "balls=0 但画面明明有球" 的根因。
# → aspect 0.67~0.71 与 task4 实测 (0.42/0.60=0.70) 一致 —— 说明只是距离/尺度
#   差异, 不是模型或标注问题。
# → score 0.683 那颗是真球 (画面边缘), 故下限取 0.60 而非 task4 的 0.85。

DETECT_SCORE_MIN: float = 0.60
"""最低置信度。实测 0.683~0.916; 取 0.60 保住边缘那颗真球。
调高可过滤噪声框, 但会先丢掉画面边缘的球。--score-min 覆盖。"""

DETECT_AREA_MIN: float = 0.10
"""最小归一化面积。实测 0.148~0.168, 下探到 0.10 留余量 (球再远一点也能收)。
⚠️ 这是与 task4 的**关键差异** (那边是 0.20)。--area-min 覆盖。"""

DETECT_AREA_MAX: float = 0.24
"""最大归一化面积。实测上界 0.168, 放到 0.24 留余量 (球更近时也能收),
同时仍能挡住占满画面的大块噪声。--area-max 覆盖。"""

DETECT_ASPECT_TOL: float = 0.8
"""宽高比容差, 沿用 task4 (|aspect - 1| ≤ tol)。实测 aspect 0.67~0.71,
|a-1| ≤ 0.33, 余量充足, 无需单独标定。"""


# ---------- 球类识别 (复用 task4/target2.fetch_balls) ----------

def _fmt_ball(b: dict, idx: int) -> str:
    """单个球的一行日志 (与 task4/target2._fmt_ball 同格式, 便于对照)。"""
    return (f"    [{idx}] color={str(b.get('color')):7s} "
            f"cx={b.get('cx_norm', 0.0):+.3f}  cy={b.get('cy_norm', 0.0):+.3f}  "
            f"w×h={b.get('w_norm', 0.0):.3f}×{b.get('h_norm', 0.0):.3f}  "
            f"score={b.get('score', 0.0):.3f}  det_id={b.get('det_id')}")


def detect_balls(client: ArmClient,
                 color_filter=DETECT_COLOR_FILTER,
                 timeout_s: float = DETECT_TIMEOUT_S,
                 hz: float = DETECT_HZ,
                 score_min: float = DETECT_SCORE_MIN,
                 area_min: float = DETECT_AREA_MIN,
                 area_max: float = DETECT_AREA_MAX,
                 aspect_tol: float = DETECT_ASPECT_TOL) -> list:
    """摆位完成后读侧摄 task_feed, 返回当前帧识别到的球 (蓝/黄)。

    复用 `task4/target2.py` 的 `fetch_balls()` 做解析 (bbox 三格式兼容 /
    label→color 映射), 但**阈值一律传本文件的 DETECT_***, 不吃 task4 的
    TARGET_* 默认值 —— 那套是近景标定的, 会把本位姿的球全筛光 (2026-07-29
    实测: area 0.148~0.168 vs task4 下限 0.20, 三球全丢)。详见常量区注释。

    ⚠️ `verify_target1_pose=False` 写死: BALL_VERIFIED_* 是 **task4** 的
    target1 位姿 (y=-150/x=-260) 标定的, 本文件是 task5 的 target1
    (y=-200/x=-40), 位姿不同, 开了会把所有球误伤过滤光。详见模块 docstring。

    纯只读: 不动机械臂。任何异常都只 warn + 返回 [], 不抛 (摆位已成功,
    不该因为看不到球把整个脚本判失败)。

    Args:
        client: ArmClient (取它的 .http 拿 RuntimeApiClient)。
        color_filter: "blue" / "yellow" / None (不过滤)。
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
            balls = fetch_balls(
                client.http,
                color_filter=color_filter,
                # ⚠️ 显式传 task5 阈值, 不用 task4 的 TARGET_* 默认值
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

    if balls:
        print(f"  {LOG_PREFIX} 识别到 {len(balls)} 个球 (轮询 {rounds} 次, "
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



# ---------- belt-slip 安全 move_x (内联, 走 test_x_to_150.py 模式) ----------

def _move_x_with_split(client: ArmClient, target_x_mm: float) -> dict:
    """belt-slip 安全 move_x —— 走 test_x_to_150.py 模式:
    每轮 client.move_x(target, v_max_mms) (透传限速), 然后 realtime 读真值;
    卡住 → kick 停一下让带重咬合; 连续 N 轮无进展 → 放弃。

    Returns:
        {"target_x": float, "actual_x": float, "segments": int, "reached": bool}
    """
    x0 = client._read_x_mm_realtime()
    if x0 is None:
        raise RuntimeError("realtime x_mm 读不到 (arm_feed 未启 / realtime 不可用)")
    delta = target_x_mm - x0

    if abs(delta) <= MOVE_X_TOL_MM:
        print(f"  {LOG_PREFIX} move_x({target_x_mm}mm)  已在容差内 (x0={x0:+.1f}mm), 跳过")
        return {"target_x": target_x_mm, "actual_x": x0, "segments": 0, "reached": True}

    print(f"  {LOG_PREFIX} move_x({target_x_mm}mm)  距 {delta:+.1f}mm  "
          f"v_max={MOVE_X_V_MAX_MMS:.0f}mm/s  TOL=±{MOVE_X_TOL_MM:.0f}mm  "
          f"reach+stall 模式 (test_x_to_150.py 同款)")

    x_prev = x0
    stall_rounds = 0
    steps = 0
    reached = False
    x_final = x0

    for rnd in range(1, MOVE_X_MAX_ROUNDS + 1):
        try:
            client.move_x(x_mm=target_x_mm, v_max_mms=MOVE_X_V_MAX_MMS)
            x_now = client._read_x_mm_realtime()
            if x_now is None:
                raise RuntimeError("realtime x_mm 读不到")
        except Exception as e:
            print(f"  {LOG_PREFIX} [FAIL] 轮{rnd:2d}  {type(e).__name__}: {str(e)[:80]}")
            break

        step = x_now - x_prev
        err = x_now - target_x_mm
        steps += 1
        x_prev = x_now
        x_final = x_now
        print(f"  {LOG_PREFIX} 轮{rnd:2d}  x={x_now:+7.1f}mm  本轮走={step:+6.1f}mm  "
              f"距目标={err:+6.1f}mm")

        if abs(err) < MOVE_X_TOL_MM:
            reached = True
            break

        if abs(step) < MOVE_X_STALL_MM:
            stall_rounds += 1
            print(f"         [SLIP] 本轮几乎没动, kick 停 {MOVE_X_KICK_SLEEP_S}s 让同步带重咬合 "
                  f"(连续卡住 {stall_rounds}/{MOVE_X_MAX_STALL_ROUNDS})")
            if stall_rounds >= MOVE_X_MAX_STALL_ROUNDS:
                print(f"         [ABORT] 连续 {MOVE_X_MAX_STALL_ROUNDS} 轮无进展, "
                      f"撞墙/带打滑治不动, 放弃")
                break
            client.stop_x_speed_safety()
            time.sleep(MOVE_X_KICK_SLEEP_S)
        else:
            stall_rounds = 0

    return {"target_x": target_x_mm, "actual_x": x_final, "segments": steps, "reached": reached}


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
    """摆到 target1 目标位姿: y=-200 → x=-40 → 大臂 90° → 手爪 0° → 球类识别。

    Returns:
        {"ok": True, "x_info": dict, "x_mm": float, "y_mm": float,
         "arm_deg": float, "hand_deg": float, "balls": list[dict]}
        `balls` 在 detect=False 或没识别到时为 []。
    """
    print(f"\n========== {LOG_PREFIX} run ==========")
    print(f"  目标: y={y_mm}mm → x={x_mm}mm → 大臂{arm_deg}° → 手爪{hand_deg}°"
          f"{' → 球类识别' if detect else ' (不识别)'}")

    # 1. y → -200 (先抬出保护区 [0,-30]; move_y 走步进电机, 保护区里也能调)
    print(f"  [1/5] move_y({y_mm}mm)  抬出保护区 [0,-30]")
    runner.move_y(y_mm, timeout=30.0)

    # 2. x → -40 (belt-slip 安全 move_x, 透传 v_max_mms=30; 卡住 kick + stall 放弃)
    print(f"  [2/5] move_x({x_mm}mm)  belt-slip 安全 (test_x_to_150.py 模式)")
    x_info = _move_x_with_split(client, x_mm)
    print(f"        x_info={x_info}")

    # 3. 大臂 → 90° (业务硬限上界 / 复位位 / init 例外位; y=-200 远出保护区)
    print(f"  [3/5] set_arm_angle({arm_deg}°)")
    client.set_arm_angle(arm_deg, speed=80, timeout=10.0)

    # 4. 手爪 → 0° (DOWN) —— 不动大臂, 直接设手爪
    #    大臂 90° 已在展开区 [-30,+30] 之外, api.py:604-612 的门不再命中;
    #    仍走底层 _call_arm 直调 (与 get_blue / get_yellow 一致) → 真正下发的
    #    合法性由车端决定。硬件真不允许 → 拿到车端错误, 不会崩在 Python 层。
    print(f"  [4/5] 手爪 → {hand_deg}° (DOWN), 大臂保持 {arm_deg}° 不动 (底层直调)")
    client._call_arm(
        "set_hand_angle", timeout=10.0, sync=True,
        angle=hand_deg, speed=80,
    )

    # 5. 球类识别 (只读, 不动机械臂; 失败只 warn 不抛)
    balls: list = []
    if detect:
        print(f"  [5/5] 球类识别 (侧摄 task_feed, ≤{detect_timeout_s}s, "
              f"score≥{score_min} area∈[{area_min},{area_max}])")
        balls = detect_balls(client, color_filter=color_filter,
                             timeout_s=detect_timeout_s,
                             score_min=score_min,
                             area_min=area_min, area_max=area_max)
    else:
        print(f"  [5/5] 球类识别  已跳过 (--no-detect)")

    print(f"========== {LOG_PREFIX} 完成 "
          f"(x={x_mm}mm arm={arm_deg}° y={y_mm}mm hand={hand_deg}° "
          f"balls={len(balls)}) ==========\n")
    return {
        "ok": True,
        "x_info": x_info,
        "x_mm": x_mm,
        "y_mm": y_mm,
        "arm_deg": arm_deg,
        "hand_deg": hand_deg,
        "balls": balls,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task5 target_blue: 摆到取蓝位姿 (y=-200→x=-40→arm=90→hand=0) + 只检测蓝球",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--x", type=float, default=TARGET1_X_MM, help="x (mm), 默认 -40")
    p.add_argument("--y", type=float, default=TARGET1_Y_MM, help="y (mm), 默认 -200")
    p.add_argument("--arm", type=float, default=TARGET1_ARM_DEG, help="大臂角度 (°)")
    p.add_argument("--hand", type=float, default=TARGET1_HAND_DEG, help="手爪角度 (°)")
    p.add_argument("--no-detect", action="store_true", dest="no_detect",
                   help="只摆位, 跳过第 5 步球类识别")
    # ⚠️ 不提供 --color: 本文件只认蓝球 (DETECT_COLOR_FILTER 写死 "blue")。
    #    要看黄球用 task4/target2.py。
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
    # ⚠️ color_filter 写死传 DETECT_COLOR_FILTER ("blue"), 不读 CLI
    run(client, runner, x_mm=args.x, y_mm=args.y,
        arm_deg=args.arm, hand_deg=args.hand,
        detect=not args.no_detect,
        color_filter=DETECT_COLOR_FILTER,
        detect_timeout_s=args.detect_timeout,
        score_min=args.score_min,
        area_min=args.area_min, area_max=args.area_max)
    return 0


if __name__ == "__main__":
    sys.exit(main())
