"""task7 / the_final_position6 —— **位置 6 (右中)** 投递编排器 (3 阶段: 前进 → the_final_position5 → 后退)。

按用户 2026-08-06 新建 (命名风格同 the_final.py 编排器):

  Phase 1: 底盘前进 130mm (13cm)                                          ← 不变
            ↓
  Phase 2: 调用 the_final_position5.run() (7 步臂, 含 1 次放气 Step 2.3)
            2.0 composite_run(arm=+90°, x=-165, y=-160, hand=+0°)        4 机联动
            2.1 composite_run(双机联动) y:-160→-85, hand:0°→-20°         (4 轴全传)
            2.2 runner.move_x(-230)                                        push 投到投递位
            2.3 runner.drop_object()                                       🆕 放气 (货物落目标位)
            2.4 runner.move_x(-170)                                        pull 撤退中间位
            2.5 runner.move_y(-160)                                        y 上升回 y_up
            2.6 runner.move_x(0)                                           x 归零撞墙 calibrate
            ↓
  Phase 3: 底盘后退 130mm (13cm)                                          ← 不变

vs 旧版 position6.py (如果有):
  - 旧版 7 步臂全 inline, 自包含
  - 新版委托 ``the_final_position5.run()`` 处理 7 步臂, **避免逻辑重复**
  - 命名风格同 ``the_final.py`` 编排器, 跟 ``position5.py`` 形成对照 (the_final_position5 是 1:1 镜像)
  - 底盘 Phase 1/3 走 ``car.move_for`` (同 ``position3.py v5`` —— **方向相反于 position1**)

⚠️ **底盘移动 (Phase 1/3) 保持不变** (用户 2026-08-06 明确指示):
  - Phase 1 前进 / Phase 3 后退; 与 the_final_position4.py 的方向**相反** (后者是先退后进)。
  - 走车端 ``car.move_for`` (相对位姿位移, [x_m, 0, 0])。
  - 正值 = 车头方向前进, 负值 = 车头方向后退。
  - 本脚本 CLI 接收的 --forward / --back 始终是 **正 mm** (用户语义),
    内部 Phase 1 直接用 = 前进; Phase 3 转负号 = 后退。
  - 130mm / 0.10 m/s ≈ 1.3s, 自适应 timeout = max(5.0, |dist|/vel + 2.0),
    用户 ``--timeout`` 兜底。
  - sync=True 阻塞等闭环完成 (后续臂动作必须等底盘停下, 否则位置错位)。
  - 不走 ArmClient._call_car (默认 sync=False 异步会抢跑)。
  - 失败 → job status=failed, 脚本直接 raise 让外层处理 (不静默吞)。

⚠️ **跟 the_final_position4.py 唯一区别: 底盘方向相反**:
  - the_final_position4: 后退 → work → 前进  (适合"位置 4 左边", 车先远离 4 号位置再回)
  - **the_final_position6: 前进 → work → 后退**  (适合"位置 6 右边", 车先靠近 6 号位置再回)
  - 其他 7 步臂 + 业务硬限 + 顺序约束**完全相同** (委托 the_final_position5)。
  - **不要**为了对齐结构去复制 the_final_position4.py 的代码 —— 同款代码用 import 复用。

⚠️ **中间臂序列委托给 the_final_position5** (重要):
  - **不要**在本文件复制 7 步臂代码 —— 违反 DRY, 改 the_final_position5 时容易漏改。
  - **不要**用 ``from .the_final_position5 import ...`` 走包导入 —— task7 不一定
    有 __init__.py。本脚本用 sys.path 注入 task7 目录后**直接模块导入**。
  - the_final_position5.run() 返回 dict 含 ``step1..step7`` 各步原始 job,
    编排器原样透传给上游, **不解析/不修改** (上游自己决定哪些步骤失败需要 fallback)。

⚠️ **业务硬限 + 顺序约束** (放气位置不能乱) 详见 the_final_position5.py docstring:
  - 7 步臂全部 ARM_API §1.1 / §7 软限位 / 保护区约束, 走前要核对。
  - 关键约束: Step 2.3 drop_object() 必须在 Step 2.2 push 之后、Step 2.4 pull 之前。
  - y=-85 在保护区 [0, -80] **外** 5mm, composite_run 内部不查 y 保护区 (拍板)。

⚠️ **本文件定位**:
  - **不**算 self-contained 脚本 (与 position1-3 v5.py / position5 v4+ / get_position1 v4 等不同)。
  - 是**编排器** —— 跟 the_final.py 一类, 委托子任务 (the_final_position5)。
  - 命名风格统一: ``the_final_position{N}.py`` = "位置 N 投递全流程" 编排器。
  - the_final_position5.py 是 1:1 镜像 position5.py (单独跑也行);
    **the_final_position4 / the_final_position6 必须** 委托 the_final_position5。

⚠️ **改版历史**:
  - **v1 (2026-08-06)**: 首次新建 —— 3 阶段编排器, 委托 the_final_position5.run()。
    底盘 Phase 1/3 走 _chassis_move_for, 与 the_final_position4.py 方向**相反**。

跑法:
    python main/arm/each_task/task7/the_final_position6.py
    python -m main.arm.each_task.task7.the_final_position6
    python main/arm/each_task/task7/the_final_position6.py --forward 200 --back 200   # 改底盘距离
    python main/arm/each_task/task7/the_final_position6.py --vel 0.05                 # 更稳
"""
from __future__ import annotations

import argparse
import os
import sys
import time

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

# 注入 task7 目录到 sys.path, 让 the_final_position5 可作为模块直接 import
_TASK7_DIR = os.path.dirname(os.path.abspath(__file__))
if _TASK7_DIR not in sys.path:
    sys.path.insert(0, _TASK7_DIR)

from main.arm import ArmClient, ArmRunner  # noqa: E402
from the_final_position5 import run as the_final_position5_run  # noqa: E402


# ---------- 默认参数 ----------

LOG_PREFIX: str = "[task7/the_final_position6]"

# ==== Phase 1 / 3: 底盘移动 (跟前 version 一致, 保持不变) ====

DEFAULT_FORWARD_MM: float = 122
"""Phase 1 前进距离 (mm)。用户 2026-08-06 指定 13cm = 130mm。
CLI 接收正值, 内部直接用 → move_for x_m = +dist_mm/1000 (前进)。
负值会被 ``abs()`` 强制取正, 避免误传后退撞墙。"""

DEFAULT_BACK_MM: float = 122
"""Phase 3 后退距离 (mm)。用户 2026-08-06 指定 13cm = 130mm。
CLI 接收正值, 内部转负号 → move_for x_m = -dist_mm/1000 (后退)。
负值会被 ``abs()`` 强制取正再取负, 避免误传前进撞墙。"""

DEFAULT_CHASSIS_TIMEOUT_S: float = 10.0
"""底盘 move_for HTTP 同步超时兜底 (秒)。
130mm / 0.10 m/s ≈ 1.3s; 脚本按 max(5.0, |dist|/vel + 2.0) 自适应放大,
用户 ``--timeout`` 是这个自适应值的下限。"""

DEFAULT_CHASSIS_VELOCITY_MS: float = 0.10
"""底盘最大线速度 (m/s), 与 task7/dipan.py / position1.py v5 / the_final_position4 默认一致。"""

ARM_PHASE_TIMEOUT_S: float = 90.0
"""Phase 2 the_final_position5.run() 同步超时 (秒)。
7 步臂 + 1 次放气, 现场实测 ~33s (含双机联动 composite_run 2 次 × ~2-3s, move_x/y 5 次 × ~1-2s,
drop_object 1 次), 给 90s 兜底。
注意: 这是**编排器等待 Phase 2 返回**的总超时, 不是 SDK 单步超时 (SDK 单步用各自 timeout)。
当前 the_final_position5.run() 不接受 timeout 参数, 此值**暂时用不到**, 保留参数为将来扩展。"""


# ---------- 底盘 move_for 内联 ----------

def _chassis_move_for(client: ArmClient, dist_mm: float,
                      max_velocity_ms: float, timeout: float,
                      log_prefix: str) -> dict:
    """底盘相对位姿位移 (move_for)。sync=True 阻塞等闭环完成。

    与 the_final_position4.py / position1.py v5 的 _chassis_move_for 完全相同
    (复制过来避免包依赖, 跟 the_final_position4 完全同款)。

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


# ---------- 主流程 ----------

def run(client: ArmClient, runner: ArmRunner,
        forward_mm: float = DEFAULT_FORWARD_MM,
        back_mm: float = DEFAULT_BACK_MM,
        max_velocity_ms: float = DEFAULT_CHASSIS_VELOCITY_MS,
        timeout: float = DEFAULT_CHASSIS_TIMEOUT_S,
        arm_timeout: float = ARM_PHASE_TIMEOUT_S) -> dict:
    """3 阶段执行: 前进 → the_final_position5.run() (7 步臂, 含 1 次放气) → 后退。

    ⚠️ **2026-08-06 新建**: 编排器模式 —— 底盘 Phase 1/3 走 move_for,
       中间 Phase 2 委托 ``the_final_position5.run()`` 处理 7 步臂 (含 1 次放气)。

    ⚠️ **底盘方向跟 the_final_position4 相反**:
       - the_final_position4: 后退 → work → 前进
       - **the_final_position6: 前进 → work → 后退**

    Args:
        client: ArmClient (底盘 move_for + the_final_position5 内部 composite_run 都用它)
        runner: ArmRunner (委托给 the_final_position5.run(), 本编排器不用)
        forward_mm: Phase 1 前进距离 (mm, 正值, 默认 130)
        back_mm: Phase 3 后退距离 (mm, 正值, 默认 130)
        max_velocity_ms: 底盘限速 (m/s, 默认 0.10)
        timeout: 底盘 move_for HTTP 同步超时下限 (秒, 默认 10; 脚本自适应放大)
        arm_timeout: Phase 2 the_final_position5.run() 同步超时 (秒, 默认 90; 现场实测 ~33s)
                     当前 the_final_position5.run() 不接受 timeout 参数, 暂时用不到。

    Returns:
        {
            "ok":              True / False,
            "forward_mm":      float,            # Phase 1 距离 (正值, 用户语义)
            "back_mm":         float,            # Phase 3 距离 (正值, 用户语义)
            "forward_job":     dict,             # Phase 1 move_for 原始 job dict
            "back_job":        dict,             # Phase 3 move_for 原始 job dict
            "phase2_result":   dict,             # 🆕 Phase 2 the_final_position5.run() 原始返回 dict
                                                #    含 step1..step7 + final_pose
            "final_pose": {                       # 🆕 终态 (Phase 2 完成后, 等于 the_final_position5 终态)
                "x_mm":    0.0,
                "y_mm":    -160.0,
                "arm_deg": 90.0,
                "hand_deg": -20.0,
            },
        }
    """
    t_overall = time.time()
    print(f"\n========== {LOG_PREFIX} run (前进 → the_final_position5 → 后退) ==========")
    print(f"  Phase 1: 前进 {forward_mm:.0f}mm")
    print(f"  Phase 2: 调用 the_final_position5.run() (7 步臂, 含 1 次放气, 总超时 {arm_timeout:.0f}s)")
    print(f"  Phase 3: 后退 {back_mm:.0f}mm")

    # ==== Phase 1: 底盘前进 ====
    # 强制转正: 即便用户传了 --forward -50 (误) 也变成 +130 (前进), 避免误后退撞墙。
    forward_signed_mm = abs(forward_mm)
    forward_timeout = max(timeout, 5.0, abs(forward_signed_mm) / 1000.0 / max(max_velocity_ms, 0.01) + 2.0)
    print(f"\n  ── Phase 1: 底盘前进 {forward_signed_mm:.0f}mm ──")
    forward_job = _chassis_move_for(
        client, forward_signed_mm,
        max_velocity_ms, forward_timeout,
        log_prefix=f"  {LOG_PREFIX} phase1",
    )

    # ==== Phase 2: 调用 the_final_position5.run() (7 步臂) ====
    # 委托子任务, 本编排器**不复制** 7 步臂代码。
    # the_final_position5.run() 内部走 SDK 同步, 异常就抛 (本编排器不静默吞)。
    print(f"\n  ── Phase 2: 调用 the_final_position5.run() (7 步臂, 含 1 次放气) ──")
    t_arm = time.time()
    phase2_result = the_final_position5_run(
        client, runner,
        # 注意: the_final_position5.run() 不接受 timeout 参数,
        # 它内部已经用各自的 COMPOSITE_TIMEOUT_S / X_MOVE_TIMEOUT_S / Y_MOVE_TIMEOUT_S。
        # 所以 arm_timeout 暂时用不到 (保留参数为将来扩展)。
    )
    arm_dt = time.time() - t_arm
    if not isinstance(phase2_result, dict) or not phase2_result.get("ok", False):
        print(f"  ── Phase 2 ❌ the_final_position5.run() 失败: {phase2_result}")
        return {
            "ok": False,
            "failed_step": "phase2_the_final_position5",
            "forward_job": forward_job,
            "back_job": None,
            "phase2_result": phase2_result,
            "final_pose": None,
        }
    print(f"  ── Phase 2 ✅ the_final_position5.run() 完成 ({arm_dt:.2f}s) ──")

    # ==== Phase 3: 底盘后退 ====
    # 强制转负: 即便用户传了 --back -50 (误) 也变成 -130 (后退), 避免误前进撞墙。
    back_signed_mm = -abs(back_mm)
    back_timeout = max(timeout, 5.0, abs(back_signed_mm) / 1000.0 / max(max_velocity_ms, 0.01) + 2.0)
    print(f"\n  ── Phase 3: 底盘后退 {abs(back_signed_mm):.0f}mm ──")
    back_job = _chassis_move_for(
        client, back_signed_mm,
        max_velocity_ms, back_timeout,
        log_prefix=f"  {LOG_PREFIX} phase3",
    )

    overall_dt = time.time() - t_overall
    print(f"\n========== {LOG_PREFIX} 完成 ({overall_dt:.2f}s) ==========")
    print(f"  终态 (the_final_position5): y=-160mm (保护区 [0, -80] 外 80mm) "
          f"x=0mm (撞墙 calibrate) "
          f"arm=+90° hand=-20°")
    print(f"     注意: hand=-20° (非 UP) 但 y=-160 在保护区外, 后续 set_*_angle 安全。\n")

    return {
        "ok": True,
        "forward_mm": forward_mm,
        "back_mm": back_mm,
        "forward_job": forward_job,
        "back_job": back_job,
        "phase2_result": phase2_result,            # 🆕 透传 the_final_position5.run() 返回 dict
        "final_pose": phase2_result.get("final_pose"),  # 🆕 终态从 the_final_position5 取
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: --forward / --back / --vel / --timeout。

    --forward / --back 接收正 mm (用户语义), 内部自动转符号:
      - --forward → move_for x_m = +|forward|/1000  (前进, Phase 1)
      - --back  → move_for x_m = -|back|/1000  (后退, Phase 3)
    即便用户传负值也会被 abs() 强制取正, 避免误传撞墙。
    """
    p = argparse.ArgumentParser(
        description=(
            "task7 the_final_position6 v1: 3 阶段编排器\n"
            "  Phase 1: 底盘前进 13cm\n"
            "  Phase 2: the_final_position5.run()  (7 步臂, 含 1 次放气)\n"
            "  Phase 3: 底盘后退 13cm"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--forward", type=float, default=DEFAULT_FORWARD_MM,
                   help="Phase 1 前进距离 (mm, 默认 130 = 13cm, 强制正值)")
    p.add_argument("--back", type=float, default=DEFAULT_BACK_MM,
                   help="Phase 3 后退距离 (mm, 默认 130 = 13cm, 强制正值)")
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
        forward_mm=args.forward,
        back_mm=args.back,
        max_velocity_ms=args.max_velocity,
        timeout=args.timeout)
    return 0


if __name__ == "__main__":
    sys.exit(main())