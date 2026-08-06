"""task7 / the_final_position5 —— **位置 5 (下中)** 投递脚本 (7 步臂, 含 1 次放气)。

按用户 2026-08-06 v4 重写, 现场实测 (v4+) 已通过。
本文件是 ``position5.py`` 的 1:1 镜像 (命名风格跟 ``the_final.py`` 编排器一致):

  ========== [task7/the_final_position5] run (7 步臂: composite_run → 双机联动 → push → drop → pull → y_up → return) ==========
    [1/7] composite_run: arm=+90° x=-165mm y=-160mm hand=+0°  speed=80 timeout=30s
    [1/7] ✅ 4 轴并发到位 (~2-3s)

    [2/7] composite_run (双机联动, 4 轴全传): arm=+90° (保持) x=-165mm (保持) y=-85mm (改) hand=-20° (改)  speed=80 timeout=30s
           ⚠️ y=-85 在保护区 [0, -80] 外 5mm (刚出保护区边 5mm, 余量比 -92 紧), composite_run 内部不查 y 保护区 (拍板)
    [2/7] ✅ y+hand 并发到位 (arm/x 保持)

    [3/7] runner.move_x(-230.0mm)  x 推到投递位 (从 -165 → -230, 距离 65mm)
    [4/7] runner.drop_object()  放气 (断开真空, 货物落目标位, y=-85 + x=-230)

    [5/7] runner.move_x(-170.0mm)  x 撤退回中间位 (从 -230 → -170, 距离 60mm)

    [6/7] runner.move_y(-160mm)  y 上升回 y_up (从 -85 → -160, 距离 75mm)

    [7/7] runner.move_x(0.0mm)  x 归零 (撞墙 calibrate, 从 -170 → 0, 距离 170mm, y=-160 保护区外)

  ⚠️ Step 6 print 输出 "y_up" 但常量名是 ``POS_Y_INIT_MM`` (= y_init / y_up 同义, 是保护区外的安全 y 位)。

vs 旧版 v3 (10 步):
  - Phase A 准备 (4 步: y_up → arm → hand → x_mid) → 1 步 composite_run (4 机联动 Step 1)
  - Phase B/C/D (6 步: y_down → x_final → drop → x_mid → y_up → x_return)
    → 6 步新流程 (composite_run 双机联动 → x_push → drop → x_pull → y_up → x_return)
  - 去掉 move_x_with_split (用户 2026-07-31 报 "belt-slip 已修复", 简化)
  - 总耗时: 旧版 ~15s → 新版 ~5-6s

⚠️ **业务硬限** (走前要核对, 见 ARM_API §1.1 / §7 + setters.py):
  - y=-160 ≤ soft_y_max=-200 ✓ (**保护区 [0, -80] 外 80mm**, 给 Step 6 move_y 留余量)
  - y=-85 ∈ soft_y_max ✓ (**保护区 [0, -80] 外 5mm**, 比 -74 安全但比 -92 紧)
  - x=-165 ∈ [-320, +220] ✓ (距下界 155mm)
  - x=-230 ∈ [-320, +220] ✓ (距下界 90mm)
  - x=-170 ∈ [-320, +220] ✓ (距下界 150mm)
  - arm=+90 ∈ [-150, +150]° ✓ (复位位)
  - hand=0° ∈ [-90, +10]° ✓ (init, 业务硬限上界内)
  - hand=-20° ∈ [-90, +10]° ✓ (mid mode)

⚠️ **Step 2 双机联动** (🆕 用户 2026-08-06 新设计, 4 机联动的"值变化版"):
  - **逻辑上**只 y + hand 改值 (双机), arm + x 保持 Step 1 终态 (复用, 不动)。
  - **API 上** 4 轴必须**全传有效值** — 2026-08-06 现场实测确认: composite_run SDK
    **不接受 None 轴**, 传 ``composite_run(y_mm=-85, hand=-20)`` (arm/x 默认 None) 会导致
    ``result.steps={'arm': False, 'x': False}`` (None 被 SDK 当无效值拒绝), 整个 job 失败。
  - **正确调用**: ``composite_run(arm=+90, x_mm=-165, y_mm=-85, hand=-20)`` —
    SDK 内部对"目标 == 当前"的 arm/x 走 no-op 或快速确认, **不会真动**; y + hand 并发到位。
  - 这是用户设计的"小步迭代"模式: 4 机先到位 (y 在保护区外), 再单独 y+hand 切到工作姿态。

⚠️ **Step 2 后 arm 处在保护区外 + 非 UP 手** (y=-85 ∉ [0, -80] + hand=-20°):
  - composite_run 内部**不调用** _check_y_protected (composite.py:60 拍板"不怕撞车"),
    所以 Step 2 的 2 轴并发 (y: -160→-85 + hand: 0→-20) 不会被拦截。
  - Step 3/5/7 runner.move_x() 不查保护区; Step 4 runner.drop_object() 不查保护区;
    Step 6 runner.move_y(-160) 是**纯 y 平移** (不走 set_*_angle), 不查保护区。
  - 所以 Step 2 后 y=-85 (保护区**外** 5mm) + hand=-20° (非 UP) **完全安全**。
  - (旧版 y=-74 时在保护区内, 风险更大; v4+ 改 -85 出保护区后无需担心 hand 状态。)

⚠️ **顺序关键** (放气位置不能乱):
  - Step 4 drop_object() 必须在 Step 3 x 推到投递位 (-230) **之后**, Step 5 x 撤退 (-170) **之前**。
  - 太早 (push 之前) → 货物在工作位之前掉 → 扔歪
  - 太晚 (撤退之后) → 真空一直吸住, 撤退拖拽货物 → 撞墙/撞货物
  - Step 6 move_y(-160) 在 Step 5 之后, 把 hand 带出保护区, 防止后续意外触发 _check_y_protected。

⚠️ **不走 move_x_with_split** (与 task1_seeding / get_position1 v4 / target v3 / position1-3 v5 / the_final_position5 同款):
  - 用户 2026-07-31 报告 belt-slip 已修复, 业务层简化直接用 runner.move_x。
  - 本脚本是 state 过渡 (投递), 不需要 belt-slip retry 兜底。

⚠️ **2026-08-06 修 (现场实测踩坑)**:
  - **composite_run 不支持偏量调用**。业务层代码 ``composite.py:56-68`` 把 None 透传给 SDK,
    但 SDK 端不识别 None, 会把 ``arm=None/x=None`` 当无效值拒绝, 导致 steps 里 arm/x=False,
    整个 job ``result.ok=False`` (虽然 HTTP/queue 层 status=succeeded)。
  - 修法: 任何 composite_run 调用必须 **4 轴全传有效值**; "不动的轴"靠"传相同值"实现。
  - 适用于所有 composite_run 调用点 (target.py / get_position1-2.py / position1-3 v5.py /
    the_final.py 等); 之前那些**正好传了全部 4 轴**, 没踩到这个坑。
  - 想做"部分轴变化"时, 用 **"全传 + 部分值不变"** 模式 (本文件 Step 2 范例)。

⚠️ **本文件自包含** (与 task7/{target, get_position1 v4, get_position2 v3, position1-3 v5, the_final_position5, dipan}.py 同款):
  只依赖 ``main.arm.ArmClient`` + ``main.arm.ArmRunner``,
  **不 import** ``main.arm.each_task.common.move_x_with_split`` (本版不用),
  也不 import task7 包内任何模块 (包含 ``position5`` 也不引用 —— 本文件是镜像不是 alias)。

⚠️ **跟 position5.py 关系** (1:1 镜像, 命名风格同 the_final.py):
  - position5.py: 老文件名, v4 是 2026-08-06 现场实测通过的 7 步投递脚本
  - **the_final_position5.py**: 1:1 镜像, 文件名跟 task7/the_final.py 编排器命名风格一致
  - 两个文件**逻辑完全相同**, 区别只是文件名 + LOG_PREFIX + 内部 docstring 引用
  - 现场用哪个都行; 如果将来要把位置 5 单独抽出用, 推荐用 ``the_final_position5.py``
  - **不要**为了对齐结构去引用 ``from . import position5`` —— 违反自包含约定

⚠️ **改版历史**:
  - v1 (2026-08-03): 11 步原版 (含 suck() Step 0 + drop_object() Step 7)。
  - v2 (2026-08-04 中午): 删掉 suck() 和 drop_object(), 原 11 步 → 9 步。
  - v3 (2026-08-04 晚上): 在 Step 6 和 Step 7 之间加回 drop_object() (放气), 9 步 → 10 步。
  - **v4 (2026-08-06, 当前)**: 7 步大改 —— Step 1 composite_run(4 机联动) +
    Step 2 composite_run(双机联动, 4 轴全传 + arm/x 复用 Step 1 终态值) +
    简化 x/y 序列 (去掉 move_x_with_split, 走 runner.move_x)。
  - **v4+ (2026-08-06 同日修)**: Step 2 实测踩坑后修正 — composite_run SDK **不接受 None 轴**,
    改成"4 轴全传有效值 + arm/x 复用相同值"模式。详见 [[composite-run-no-partial-2026-08-06]]。
  - **🆕 the_final_position5 (2026-08-06)**: 1:1 镜像 position5.py, 命名跟 the_final.py 一致。

跑法:
    python main/arm/each_task/task7/the_final_position5.py
    python -m main.arm.each_task.task7.the_final_position5
    python main/arm/each_task/task7/the_final_position5.py --x-push -250    # 改 Step 3 push x
    python main/arm/each_task/task7/the_final_position5.py --x-pull -150    # 改 Step 5 pull x
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


# ---------- 终态常量 (用户 2026-08-06 v4 重写, composite_run + 双机联动) ----------

LOG_PREFIX: str = "[task7/the_final_position5]"

# ====== Step 1: 4 机联动 composite_run 终态 ======

POS_X_INIT_MM: float = -165.0
"""Step 1 composite_run 的 x_mm 终态 (-165mm)。

⚠️ 必须 ∈ [-320, +220] 软限位 ✓; 距下界 -320 有 155mm 余量, 充裕。
⚠️ Step 2 双机联动**复用相同值** (值不变 = SDK no-op), 此值保持到 Step 7 之前。"""

POS_Y_INIT_MM: float = -160.0
"""Step 1 composite_run 的 y_mm 终态 (-160mm)。

⚠️ **保护区 [0, -80] 外 80mm**, 给 Step 6 move_y(-160) 留余量。
⚠️ ≤ soft_y_max=-200 ✓; 距上限 40mm, 偏紧但合法。"""

POS_ARM_DEG: float = 90.0
"""Step 1 composite_run 的 arm 角度终态 (+90°, 复位位)。

⚠️ 业务硬限 [-150, +150]° ✓; +90 是 init 位置 (保护区允许)。
⚠️ Step 2 双机联动**复用相同值** (值不变 = SDK no-op), 此值保持到 Step 7 之前。
⚠️ 与 position1/2/3 v5 / target.py / get_position1 v4 同款 reset 位 (+90°)。"""

POS_HAND_INIT_DEG: float = 0.0
"""Step 1 composite_run 的 hand 角度终态 (0°, init 边界)。

⚠️ 业务硬限 [-90, +10]° ✓; 0° 是 init 位置。
⚠️ Step 2 双机联动会把 hand 改成 POS_HAND_DOWN_DEG=-20°。"""

# ====== Step 2: 双机联动 composite_run (4 轴全传, arm/x 复用 Step 1 终态值, 只 y + hand 改值) ======

POS_Y_DOWN_MM: float = -85.0
"""Step 2 composite_run 的 y_mm 终态 (-85mm, 工作深度)。

⚠️ **保护区 [0, -80] 外 5mm** (用户 2026-08-06 改 -74→-92→-85, 出保护区更安全)。
⚠️ composite_run 内部**不调** _check_y_protected, 所以 y=-85 + hand=-20° 合法。
⚠️ Step 6 move_y(-160) 仍保留, 原因是把 y 归位到 init 值, 防止后续脚本误读 state。
⚠️ ≤ soft_y_max=-200 ✓。"""

POS_HAND_DOWN_DEG: float = -20.0
"""Step 2 composite_run 的 hand 角度终态 (-20°, mid mode)。

⚠️ 业务硬限 [-90, +10]° ✓; -90 < -20 < +10, 合法。
⚠️ 旧版 v3 用 -30°, v4 跟用户 2026-08-06 重设计改成 -20° (双机联动 mid mode)。
⚠️ 与 position1-3 v5 的 -66° 故意不同: position5 是"下排中", mid mode 比 position1-3 更 UP。
⚠️ Step 6 之后 hand 状态被抬 y -160 隔离, 后续 set_*_angle 不会触发 _check_y_protected。"""

# ====== Step 3/5/7: move_x 目标 ======

POS_X_PUSH_MM: float = -230.0
"""Step 3 runner.move_x 目标 (-230mm, 投递位置)。

⚠️ ∈ [-320, +220] ✓; 距下界 -320 有 90mm 余量。
⚠️ 从 Step 1 x=-165 → Step 3 x=-230 距离 65mm (沿 x 远离 0 方向推)。
⚠️ Step 4 drop_object 在此位置放气。
⚠️ 与旧版 v3 POS_X_FINAL_MM=-215 微调 (用户 2026-08-06 跟 position1-3 v5 风格统一)。"""

POS_X_PULL_MM: float = -170.0
"""Step 5 runner.move_x 目标 (-170mm, 撤退中间位)。

⚠️ ∈ [-320, +220] ✓; 距下界 -320 有 150mm 余量, 充裕。
⚠️ 从 Step 3 x=-230 → Step 5 x=-170 距离 60mm (沿 x 靠近 0 方向撤退)。
⚠️ 旧版 v3 撤退回 x_mid=-180 (与 Phase A 进给 x_mid 复用), v4 改成 -170 (新值, 用户 2026-08-06 重设计, 减少 10mm 撤退距离)。
⚠️ 用途: 防止 Step 7 x 归零直接 -230→0 冲过头撞到已投放的货物。"""

POS_X_RETURN_MM: float = 0.0
"""Step 7 runner.move_x 目标 (0mm, 撞墙 calibrate 位)。

⚠️ 撞墙是 calibrate, 重置编码器零点; 后续视觉闭环用此作为起点。
⚠️ 从 Step 5 x=-170 → Step 7 x=0 距离 170mm, 较远但单步可达。
⚠️ Step 6 y=-160 时调用 (y 在保护区外), move_x 不查保护区 ✓。"""

# ====== 通用 ======

ANGLE_SPEED: int = 80
"""大臂 / 手爪舵机速度 + xy PID speed, 默认 80。
与 task7/{target, get_position1 v4, position1-3 v5, the_final_position5}.py 一致。"""

COMPOSITE_TIMEOUT_S: float = 30.0
"""Step 1/2 composite_run 同步超时 (秒)。2-4 轴并发到位一般 ~2-3s, 给 30s 兜底。"""

X_MOVE_TIMEOUT_S: float = 10.0
"""Step 3/5/7 runner.move_x 同步超时 (秒)。单 x 移动一般 ~1-2s, 给 10s 兜底。"""

Y_MOVE_TIMEOUT_S: float = 10.0
"""Step 6 runner.move_y 同步超时 (秒)。单 y 移动 75mm (-85→-160) 一般 ~1-2s, 给 10s 兜底。"""


# ---------- 主流程 ----------

def run(client: ArmClient, runner: ArmRunner) -> dict:
    """按用户顺序执行 7 步臂 (含 1 次放气 Step 4): composite_run → 双机联动 → push → drop → pull → y_up → return。

    本函数 **不碰底盘**, 只调机械臂。终态: x=0, y=-160, arm=+90°, hand=-20°
    (注意 hand 留在 -20° 而非 -90° UP, 但 y=-160 在保护区外, 后续 set_*_angle 安全)。

    Args:
        client: ArmClient (composite_run 在这里)
        runner: ArmRunner (move_x + move_y + drop_object 在这里)

    Returns:
        {
            "ok":                  True / False,    # Step 1+2 composite_run 都 ok (后续 5 步走 SDK 同步, 异常就抛)
            "step1_composite":     dict,            # Step 1 composite_run(arm=+90°, x=-165, y=-160, hand=0°) 原始 job dict
            "step2_composite":     dict,            # 🆕 Step 2 composite_run(arm=+90°, x=-165, y=-85, hand=-20°)
                                                  #    双机联动 (4 轴全传, arm/x 复用 Step 1 终态值) 原始 job dict
            "step3_x_push":        dict,            # Step 3 move_x(-230) 原始 job dict
            "step4_drop":          dict,            # Step 4 drop_object 原始 job dict
            "step5_x_pull":        dict,            # Step 5 move_x(-170) 原始 job dict
            "step6_y_up":          dict,            # Step 6 move_y(-160) 原始 job dict
            "step7_x_return":      dict,            # Step 7 move_x(0) 原始 job dict
            "final_pose": {                          # 终态 (预期值, 不重读 state)
                "x_mm": 0.0,
                "y_mm": -160.0,
                "arm_deg": 90.0,
                "hand_deg": -20.0,
            },
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (7 步臂: composite_run → 双机联动 → push → drop → pull → y_up → return) ==========")

    # ========== Step 1: 4 机联动 composite_run ==========
    # 仿 task1_seeding.py 模式, 4 轴并发到位。y=-160 在保护区 [0, -80] **外** 80mm。
    print(f"  [1/7] composite_run: arm={POS_ARM_DEG:+.0f}° x={POS_X_INIT_MM:.0f}mm "
          f"y={POS_Y_INIT_MM:.0f}mm hand={POS_HAND_INIT_DEG:+.0f}°  speed={ANGLE_SPEED} "
          f"timeout={COMPOSITE_TIMEOUT_S:.0f}s")
    step1 = client.composite_run(
        arm=POS_ARM_DEG,
        x_mm=POS_X_INIT_MM,
        y_mm=POS_Y_INIT_MM,
        hand=POS_HAND_INIT_DEG,
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
        print(f"  [1/7] ❌ composite_run 失败: {step1}")
        return {
            "ok": False,
            "failed_step": "step1_composite_run",
            "step1_composite": step1,
            "step2_composite": None,
            "step3_x_push": None,
            "step4_drop": None,
            "step5_x_pull": None,
            "step6_y_up": None,
            "step7_x_return": None,
            "final_pose": None,
        }
    print(f"  [1/7] ✅ 4 轴并发到位 (~2-3s)")

    # ========== Step 2: 双机联动 composite_run (4 轴全传, arm/x 值不变 = 不动) ==========
    # 🆕 用户 2026-08-06 新设计: 4 机联动的"值变化版", 只 y + hand 改值, arm + x 复用 Step 1 终态。
    # 验证 (2026-08-06 现场实测): composite_run SDK **不接受 None 轴**, 传 ``arm=None/x=None``
    # 会导致 steps={'arm': False, 'x': False} (值无效被拒)。修法: 4 轴全传有效值, SDK 内部对
    # "目标 == 当前" 的轴走 no-op 或快速确认, **不会真动**; y + hand 并发到位。
    # y: -160 → -85 (进入保护区外 5mm, 安全)
    # hand: 0° → -20° (mid mode)
    # composite_run 内部不调 _check_y_protected (composite.py:60 拍板), 所以合法。
    print(f"\n  [2/7] composite_run (双机联动, 4 轴全传): arm={POS_ARM_DEG:+.0f}° (保持) "
          f"x={POS_X_INIT_MM:.0f}mm (保持) y={POS_Y_DOWN_MM:.0f}mm (改) "
          f"hand={POS_HAND_DOWN_DEG:+.0f}° (改)  speed={ANGLE_SPEED} "
          f"timeout={COMPOSITE_TIMEOUT_S:.0f}s")
    print(f"         ⚠️ y=-85 在保护区 [0, -80] 外 5mm (刚出保护区边 5mm, 余量比 -92 紧), composite_run 内部不查 y 保护区 (拍板)")
    step2 = client.composite_run(
        arm=POS_ARM_DEG,         # 复用 Step 1 终态 (值不变 = 不动)
        x_mm=POS_X_INIT_MM,      # 复用 Step 1 终态 (值不变 = 不动)
        y_mm=POS_Y_DOWN_MM,      # y: -160 → -85
        hand=POS_HAND_DOWN_DEG,  # hand: 0° → -20°
        speed=ANGLE_SPEED,
        timeout=COMPOSITE_TIMEOUT_S,
    )
    ok2 = (
        isinstance(step2, dict)
        and step2.get("status") == "succeeded"
        and isinstance(step2.get("result"), dict)
        and step2["result"].get("ok", False)
    )
    if not ok2:
        print(f"  [2/7] ❌ composite_run (双机联动) 失败: {step2}")
        return {
            "ok": False,
            "failed_step": "step2_composite_run_dual",
            "step1_composite": step1,
            "step2_composite": step2,
            "step3_x_push": None,
            "step4_drop": None,
            "step5_x_pull": None,
            "step6_y_up": None,
            "step7_x_return": None,
            "final_pose": None,
        }
    print(f"  [2/7] ✅ y+hand 并发到位 (arm/x 保持)")

    # ========== Step 3: runner.move_x(-230) 推投递位 ==========
    print(f"\n  [3/7] runner.move_x({POS_X_PUSH_MM}mm)  x 推到投递位 (从 {POS_X_INIT_MM:.0f} → {POS_X_PUSH_MM:.0f}, "
          f"距离 {abs(POS_X_PUSH_MM - POS_X_INIT_MM):.0f}mm)")
    step3 = runner.move_x(
        x_mm=POS_X_PUSH_MM,
        timeout=X_MOVE_TIMEOUT_S,
        verify=True,
    )

    # ========== Step 4: runner.drop_object() 放气 (投递关键步骤) ==========
    # ⚠️ 必须**在 x 推到位 (Step 3) 之后**, **x 撤退 (Step 5) 之前**。
    # y=-85 + x=-230 + hand=-20°: 手爪在工作深度 + 投递位 + mid mode, 真空断开 → 货物落目标位。
    print(f"  [4/7] runner.drop_object()  放气 (断开真空, 货物落目标位, y={POS_Y_DOWN_MM:.0f} + x={POS_X_PUSH_MM:.0f})")
    step4 = runner.drop_object(timeout=X_MOVE_TIMEOUT_S)

    # ========== Step 5: runner.move_x(-170) 撤退回 x_pull 中间位 ==========
    # 防止 Step 7 x 归零直接 -230→0 冲过头撞到已投放的货物。
    print(f"\n  [5/7] runner.move_x({POS_X_PULL_MM}mm)  x 撤退回中间位 (从 {POS_X_PUSH_MM:.0f} → {POS_X_PULL_MM:.0f}, "
          f"距离 {abs(POS_X_PULL_MM - POS_X_PUSH_MM):.0f}mm)")
    step5 = runner.move_x(
        x_mm=POS_X_PULL_MM,
        timeout=X_MOVE_TIMEOUT_S,
        verify=True,
    )

    # ========== Step 6: runner.move_y(-160) y 上升回 y_up (把 hand 带出保护区) ==========
    # 当前状态: y=-85 (保护区外 5mm) + hand=-20° (非 UP)。move_y 是纯 y 平移 (不走 set_*_angle),
    # 不查保护区 ✓。把 y 抬到 -160 后, hand 即使是非 UP 状态, 也不在保护区, 安全。
    # 为什么做这一步: Step 7 x 归零虽然不查保护区, 但 Step 7 之后状态 y=-85+hand=-20° 留着
    # 容易被后续业务脚本意外触发 _check_y_protected; 提前抬 y 隔离。
    print(f"\n  [6/7] runner.move_y({POS_Y_INIT_MM:.0f}mm)  y 上升回 y_up (把 hand 带出保护区, "
          f"从 {POS_Y_DOWN_MM:.0f} → {POS_Y_INIT_MM:.0f}, 距离 {abs(POS_Y_INIT_MM - POS_Y_DOWN_MM):.0f}mm)")
    step6 = runner.move_y(
        y_mm=POS_Y_INIT_MM,
        timeout=Y_MOVE_TIMEOUT_S,
        verify=True,
    )

    # ========== Step 7: runner.move_x(0) x 归零 (撞墙 calibrate) ==========
    print(f"\n  [7/7] runner.move_x({POS_X_RETURN_MM}mm)  x 归零 (撞墙 calibrate, 从 {POS_X_PULL_MM:.0f} → 0, "
          f"距离 {abs(POS_X_RETURN_MM - POS_X_PULL_MM):.0f}mm, y={POS_Y_INIT_MM:.0f} 保护区外)")
    step7 = runner.move_x(
        x_mm=POS_X_RETURN_MM,
        timeout=X_MOVE_TIMEOUT_S,
        verify=True,
    )

    dt = time.time() - t0
    print(f"\n========== {LOG_PREFIX} 完成 ({dt:.2f}s) ==========")
    print(f"  终态: y={POS_Y_INIT_MM:.0f}mm (保护区 [0, -80] **外** 80mm, hand 状态被隔离) "
          f"x=0mm (撞墙 calibrate) "
          f"arm={POS_ARM_DEG:+.0f}° hand={POS_HAND_DOWN_DEG:+.0f}°")
    print(f"     注意: hand=-20° (非 UP) 但 y=-160 在保护区外, 后续 set_*_angle 安全。\n")

    return {
        "ok": True,
        "step1_composite": step1,                  # Step 1 composite_run(4 机联动)
        "step2_composite": step2,                  # 🆕 Step 2 composite_run(双机联动, 4 轴全传, arm/x 复用 Step 1 终态)
        "step3_x_push": step3,                     # Step 3 move_x(-230)
        "step4_drop": step4,                       # Step 4 drop_object
        "step5_x_pull": step5,                     # Step 5 move_x(-170)
        "step6_y_up": step6,                       # Step 6 move_y(-160)
        "step7_x_return": step7,                   # Step 7 move_x(0)
        "final_pose": {
            "x_mm": POS_X_RETURN_MM,
            "y_mm": POS_Y_INIT_MM,
            "arm_deg": POS_ARM_DEG,
            "hand_deg": POS_HAND_DOWN_DEG,
        },
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 暴露 3 个 move_x 目标常量 (push/pull/return) 供现场微调。

    y_init / y_down / arm_deg / hand_init / hand_down 是 composite_run 终态,
    故意不暴露给 CLI (避免误改撞 y 保护区或业务硬限)。
    想改这 5 个值, 请编辑本文件顶置常量。
    """
    p = argparse.ArgumentParser(
        description=(
            "task7 the_final_position5 v4: 7 步大改 —— composite_run(arm=+90° x=-165 y=-160 hand=0°)\n"
            "  → composite_run(arm=+90° x=-165 y=-85 hand=-20°) [双机联动, 4 轴全传, arm/x 复用同值]\n"
            "  → move_x(-230) → drop_object → move_x(-170) → move_y(-160) → move_x(0)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--x-push", type=float, default=POS_X_PUSH_MM,
                   help="Step 3 move_x 的目标 (mm, 默认 -230, 投递位置, 必须在 [-320, +220])")
    p.add_argument("--x-pull", type=float, default=POS_X_PULL_MM,
                   help="Step 5 move_x 的目标 (mm, 默认 -170, 撤退中间位, 必须在 [-320, +220])")
    p.add_argument("--x-return", type=float, default=POS_X_RETURN_MM,
                   help="Step 7 move_x 的目标 (mm, 默认 0, 撞墙 calibrate 位)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    # 用 CLI 覆盖 3 个可变常量 (顶置常量是文档/默认值)
    global POS_X_PUSH_MM, POS_X_PULL_MM, POS_X_RETURN_MM
    POS_X_PUSH_MM = args.x_push
    POS_X_PULL_MM = args.x_pull
    POS_X_RETURN_MM = args.x_return
    client = ArmClient.connect()
    runner = ArmRunner(client)
    run(client, runner)
    return 0


if __name__ == "__main__":
    sys.exit(main())