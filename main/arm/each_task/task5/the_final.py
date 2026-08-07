"""task5 / the_final —— 端到端分拣入库流水线 (4 阶段)。

业务流程 (2026-08-08 用户指定):

  ┌──────────────────────────────────────────────────────────────────────┐
  │ [1/4] new_target.run()        摆位 + 模型识别高仓颜色 → blue/yellow  │
  │ [2/4] target_all.run()        摆位 + 全色识别 + 黄蓝分桶计数          │
  │ [3a/3b] **底盘 + 高塔循环**    matching 色 → 高塔 (调用次数 = 同色球数)│
  │ [3c] 后退 166mm               move_for([-0.166, 0, 0]) (调 dipan 默认)│
  │ [3d] **底盘 + 低塔循环**      opposite 色 → 低塔 (调用次数 = 反色球数)│
  └──────────────────────────────────────────────────────────────────────┘

  - 高仓 = blue  → 蓝球进高塔 (N=count_blue 次) → 后退 → 黄球进低塔 (M=count_yellow 次)
  - 高仓 = yellow → 黄球进高塔 (N=count_yellow 次) → 后退 → 蓝球进低塔 (M=count_blue 次)

⚠️ **设计原则 (用户要求: 不重写内部逻辑)**:
  - **不直接调用** new_target / target_all / from_*_to_*.py / dipan 内部的动作
  - **调用它们的 run() / _run() 函数**: 端到端流程已封装好, 业务硬限 / composite_run
    4 轴全传 / grasp 走 runner.* / move_y 保护区绕过 全部在内部处理
  - 本文件只负责**顶层编排 + 错误透传**, 不重写任何具体步骤

⚠️ **底盘后退 166mm 复用 dipan.py**:
  - dipan.py 已有 ``client.http.execute_car_action("move_for", [-0.166, 0, 0], sync=True)``
    完整逻辑 (默认参数 / 限速 / timeout 自适应)
  - 本文件直接 import DEFAULT_DIST_MM / DEFAULT_MAX_VELOCITY_MS / DEFAULT_TIMEOUT_S 常量,
    用 client.http.execute_car_action 调一次, **不走 dipan._run()** (私有接口)
  - 这是 2026-07-30 用户拍板的"向后 166mm", 与 __init__.py 提到的"默认 dipan 后撤 165mm"
    差 1mm (165 vs 166), 按用户本次明示用 166。

⚠️ **循环调用 from_*_to_*.run()**:
  - 每个 run() 内部是**单球端到端**: 4 机联动 → 吸气 → 下探 (取) → 抬升 → 4 机联动 → 投
  - 循环 N 次 = 处理 N 个同色球 (假设球都在同一 bin 位置, 每次从 bin 拿同一颗; 这是
    task5 业务层的现有约定 — task4 蓝 x=0 / 黄 x=-65, 不区分多颗)
  - 一次循环耗时 ~8-12s (3+5 步 for 蓝→高, 3+3 步 for 黄→低)
  - 任一循环失败 → RuntimeError 透传, 终止整个流水线 (不继续后面的球)

⚠️ **业务硬限**: 全部来自 5 个底层 run() 内部 (new_target / target_all / from_*_to_*),
   本文件不重写, 故不重复列硬限。详见各 run() 文档。

⚠️ **本文件自包含 (task5 约定)**: 只依赖 ``main.arm`` (ArmClient/ArmRunner)
   + import task5 兄弟模块的 run() 函数 / 常量, 不重写任何内部步骤。
   沿用 task5 业务层自包含约定 — task5 辅助文件曾被外部动作清空过,
   本文件 import 5 个稳定的兄弟 run() 而非重写, 保证直接跑不受影响。

跑法:
    python main/arm/each_task/task5/the_final.py
    python -m main.arm.each_task.task5.the_final
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# 1) 把 repo 根加进 sys.path, 让 "main.arm" / "main.arm.each_task.task5.*" 可解析
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

# 2) 业务硬限: 只 import task5 兄弟模块的 run() 函数 / 常量
from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.each_task.task5 import (  # noqa: E402
    new_target,        # Phase 1: 识别高仓颜色
    target_all,        # Phase 2: 识别球数 (黄/蓝 分桶)
    from_blue_to_high, # 高仓=蓝 → 蓝球进高塔 (Phase 3a)
    from_yellow_to_high, # 高仓=黄 → 黄球进高塔 (Phase 3a 反)
    from_blue_to_low,  # 高仓=黄 → 蓝球进低塔 (Phase 3d)
    from_yellow_to_low, # 高仓=蓝 → 黄球进低塔 (Phase 3d 反)
    dipan,             # Phase 3c: 底盘后退 166mm (用其常量, 不调 _run 私有)
)


# ---------- 流程常量 ----------

LOG_PREFIX: str = "[task5/the_final]"


# ---------- 主入口 ----------

def run(client: ArmClient, runner: ArmRunner) -> dict:
    """端到端分拣入库流水线 (4 阶段)。

    业务流程:
      Phase 1: ``new_target.run(client, runner)``
                → 4 机联动 + 模型识别 → label = "blue" / "yellow" / "unknown"
      Phase 2: ``target_all.run(client, runner)``
                → 4 机联动 + 全色识别 + Python 层黄蓝分桶计数
                → counts = {count_yellow, count_blue, ...}
      Phase 3: 根据 high_color 分支:
        ─── 高仓 = "blue" ───
          Phase 3a: 循环 from_blue_to_high.run() N 次 (N = count_blue)
          Phase 3c: 底盘 move_for([-0.166, 0, 0]) 后退 166mm
          Phase 3d: 循环 from_yellow_to_low.run() M 次 (M = count_yellow)
        ─── 高仓 = "yellow" ───
          Phase 3a: 循环 from_yellow_to_high.run() N 次 (N = count_yellow)
          Phase 3c: 底盘 move_for([-0.166, 0, 0]) 后退 166mm
          Phase 3d: 循环 from_blue_to_low.run() M 次 (M = count_blue)
        ─── 高仓 = "unknown" / 其它 ───
          RuntimeError: 高仓颜色无法识别, 终止流水线

    Args:
        client: ArmClient (composite_run + http + move_for 都在这里)
        runner: ArmRunner (move_y + move_x + grasp 在这里)

    Returns:
        {
            "ok": bool,                              # 全部阶段成功
            "phase1_high_tower_label": dict,         # new_target.run 完整返回值
            "high_color": str,                       # "blue" / "yellow" (Phase 1 简化)
            "phase2_ball_counts": dict,              # target_all.run 完整返回值
            "counts": {                              # Phase 2 简化 (球数)
                "count_total": int,
                "count_yellow": int,
                "count_blue": int,
                "count_unknown": int,
            },
            "phase3a_runs": list[dict],              # 高塔循环每次 run() 完整返回值
            "phase3c_retreat": dict,                 # move_for job dict
            "phase3d_runs": list[dict],              # 低塔循环每次 run() 完整返回值
            "final_pose": {                          # 终态 (来自最后一次 Phase 3d run)
                "x_mm": float,
                "y_mm": float,
                "arm_deg": float,
                "hand_deg": float,
            },
        }

    Raises:
        RuntimeError: 任一阶段失败 (Phase 1/2/3a/3c/3d), 错误透传, 终止流水线。
            - Phase 1: new_target.run 失败 (composite_run / 模型推理)
            - Phase 2: target_all.run 失败 (composite_run)
            - Phase 3a/d: from_*_to_*.run 单次循环失败
            - Phase 3c: move_for 失败 (HTTP / 底盘故障)
            - Phase 1 高仓颜色为 "unknown" → 主动 raise
    """
    t_total_start = time.perf_counter()

    print(f"\n========== {LOG_PREFIX} run (端到端分拣入库流水线, 4 阶段) ==========")
    print(f"  Phase 1: new_target.run  (识别高仓颜色)")
    print(f"  Phase 2: target_all.run  (识别球数: 黄/蓝 分桶)")
    print(f"  Phase 3: 底盘 + 高/低塔循环 (matching → 高, opposite → 低)")

    # ========== Phase 1: 识别高仓颜色 ==========
    print(f"\n--- Phase 1: new_target.run ---")
    phase1 = new_target.run(client, runner)
    if not isinstance(phase1, dict) or not phase1.get("ok", False):
        print(f"  ❌ Phase 1 失败: {phase1}")
        raise RuntimeError(
            f"{LOG_PREFIX} Phase 1 new_target.run 失败, 终止流水线 (Phase 2/3 未执行)"
        )
    high_color = phase1.get("label", "unknown")
    print(f"  ✅ Phase 1 完成  高仓颜色 = {high_color!r}")
    if high_color not in ("blue", "yellow"):
        raise RuntimeError(
            f"{LOG_PREFIX} Phase 1 高仓颜色无法识别 ({high_color!r}), "
            f"仅支持 'blue' / 'yellow', 终止流水线"
        )

    # ========== Phase 2: 识别球数 (黄/蓝 分桶) ==========
    print(f"\n--- Phase 2: target_all.run ---")
    phase2 = target_all.run(client, runner)
    if not isinstance(phase2, dict) or not phase2.get("ok", False):
        print(f"  ❌ Phase 2 失败: {phase2}")
        raise RuntimeError(
            f"{LOG_PREFIX} Phase 2 target_all.run 失败, 终止流水线 (Phase 3 未执行)"
        )
    counts = phase2.get("counts", {})
    n_blue = int(counts.get("count_blue", 0))
    n_yellow = int(counts.get("count_yellow", 0))
    n_total = int(counts.get("count_total", 0))
    print(f"  ✅ Phase 2 完成  球数: 总 {n_total} 个, 黄 {n_yellow}, 蓝 {n_blue}, "
          f"unknown {counts.get('count_unknown', 0)}")

    # ========== Phase 3: 根据 high_color 分支 ==========
    if high_color == "blue":
        phase3a_runs = _loop_run(client, runner,
                                  runner_module=from_blue_to_high,
                                  count=n_blue,
                                  phase_label="3a",
                                  color_label="蓝",
                                  tower_label="高")
        phase3c_retreat = _retreat_166mm(client)
        phase3d_runs = _loop_run(client, runner,
                                  runner_module=from_yellow_to_low,
                                  count=n_yellow,
                                  phase_label="3d",
                                  color_label="黄",
                                  tower_label="低")
    else:  # high_color == "yellow"
        phase3a_runs = _loop_run(client, runner,
                                  runner_module=from_yellow_to_high,
                                  count=n_yellow,
                                  phase_label="3a",
                                  color_label="黄",
                                  tower_label="高")
        phase3c_retreat = _retreat_166mm(client)
        phase3d_runs = _loop_run(client, runner,
                                  runner_module=from_blue_to_low,
                                  count=n_blue,
                                  phase_label="3d",
                                  color_label="蓝",
                                  tower_label="低")

    # 终态 = Phase 3d 最后一次 run 的 final_pose (失败时 final_pose 来自最后一次成功)
    final_pose = (phase3d_runs[-1].get("final_pose", {}) if phase3d_runs
                  else (phase3a_runs[-1].get("final_pose", {}) if phase3a_runs
                        else phase2.get("final_pose", {})))

    elapsed = time.perf_counter() - t_total_start
    print(f"\n========== {LOG_PREFIX} 完成 "
          f"(高仓={high_color}, 球数 黄={n_yellow} 蓝={n_blue}, "
          f"高塔={len(phase3a_runs)} 次, 低塔={len(phase3d_runs)} 次, "
          f"总耗时 {elapsed:.3f}s) ==========\n")

    return {
        "ok": True,
        "phase1_high_tower_label": phase1,
        "high_color": high_color,
        "phase2_ball_counts": phase2,
        "counts": counts,
        "phase3a_runs": phase3a_runs,
        "phase3c_retreat": phase3c_retreat,
        "phase3d_runs": phase3d_runs,
        "final_pose": final_pose,
    }


# ---------- 辅助函数 (本文件私有) ----------

def _loop_run(client: ArmClient, runner: ArmRunner,
              runner_module, count: int,
              phase_label: str, color_label: str, tower_label: str) -> list:
    """循环调用 ``runner_module.run(client, runner)`` 共 ``count`` 次。

    Args:
        client: ArmClient
        runner: ArmRunner
        runner_module: 任一 from_*_to_*.py 模块 (from_blue_to_high 等)
        count: 循环次数 (= 对应颜色的球数)
        phase_label: "3a" / "3d" (用于日志)
        color_label: "黄" / "蓝" (中文, 用于日志)
        tower_label: "高" / "低" (中文, 用于日志)

    Returns:
        list[dict]: 每次 run() 的完整返回值, 按顺序。

    Raises:
        RuntimeError: 任一次循环失败, 透传原始错误, 终止流水线 (不继续后面的球)。
            若 count=0 直接返回 [] 不报错。
    """
    print(f"\n--- Phase {phase_label}: {runner_module.__name__.split('.')[-1]}.run × {count} "
          f"({color_label}球进{tower_label}塔) ---")
    if count <= 0:
        print(f"  ⏭️  count={count}, 跳过循环")
        return []
    results: list = []
    for i in range(1, count + 1):
        print(f"\n  [{phase_label}.{i}/{count}] {runner_module.__name__.split('.')[-1]}.run(client, runner)")
        result = runner_module.run(client, runner)
        if not isinstance(result, dict) or not result.get("ok", False):
            print(f"  [{phase_label}.{i}/{count}] ❌ 失败: {result}")
            raise RuntimeError(
                f"{LOG_PREFIX} Phase {phase_label} 第 {i}/{count} 次 "
                f"{runner_module.__name__.split('.')[-1]}.run 失败, 终止流水线"
            )
        results.append(result)
        pose = result.get("final_pose", {})
        print(f"  [{phase_label}.{i}/{count}] ✅ 完成  终态: "
              f"x={pose.get('x_mm')} y={pose.get('y_mm')} "
              f"arm={pose.get('arm_deg')}° hand={pose.get('hand_deg')}°")
    return results


def _retreat_166mm(client: ArmClient) -> dict:
    """底盘后退 166mm (硬编码 -166mm, 不依赖 dipan.DEFAULT_DIST_MM)。

    走 ``client.http.execute_car_action("move_for", [-0.166, 0, 0], sync=True)``,
    默认限速 0.10 m/s, timeout 自适应 (max(5.0, 0.166/0.10 + 2) = 5.0s)。

    ⚠️ **dist_mm / max_vel / timeout 全部硬编码**, 不读 dipan.DEFAULT_*:
      - task5 分拣流水线对后退距离有强约定 (166mm = 从高塔位到低塔位的标定值),
        不允许跟随 dipan.DEFAULT_DIST_MM 漂移 (曾经被外部动作改成 +325,
        导致车向前窜 325mm 而不是向后 166mm)
      - max_vel / timeout 用同样策略, 与 dipan.py 默认值一致但本地硬编码

    Args:
        client: ArmClient (取 .http 走车端 action)

    Returns:
        ``/v1/execute`` 同步返回的 job dict (含 status/result/error)。

    Raises:
        RuntimeError: move_for 失败 (status != succeeded)。
    """
    # ⚠️ 硬编码: 166mm 后退 = -0.166m, 不读 dipan.DEFAULT_DIST_MM
    dist_mm: float = -166.0
    max_vel: float = 0.10
    timeout_default: float = 20.0  # 与 dipan.DEFAULT_TIMEOUT_S 一致, 仅用于自适应计算参考
    # 自适应 timeout (与 dipan.py main() 同款, 但 floor 提高到 15s)
    # ⚠️ **floor=15s 而非 5s**: dipan.py 默认 floor=5s, 现场实测 166mm 在网络/
    # 队列抖动时常跑到第 5s 还没完成 (job_queue 排队 + 加减速 + 网络),
    # 触发 TimeoutError。the_final 是流水线编排, 不允许在中间超时, 故放宽到 15s
    adaptive_timeout = max(15.0, abs(dist_mm) / 1000.0 / max(max_vel, 0.01) + 10.0)
    actual_timeout = adaptive_timeout  # 本文件不暴露 --timeout, 全部用 adaptive

    dist_m = dist_mm / 1000.0
    # 由 dist_m 实际符号决定方向 (而不是硬编码字面量, 与实际行为一致)
    direction = "向后" if dist_m < 0 else ("向前" if dist_m > 0 else "原地")
    print(f"\n--- Phase 3c: 底盘{direction} {abs(dist_mm):.0f}mm "
          f"(x_offset={dist_m:+.3f}m, max_v={max_vel:.2f}m/s, "
          f"timeout={actual_timeout:.1f}s) ---")
    t0 = time.perf_counter()
    job = client.http.execute_car_action(
        "move_for",
        [dist_m, 0.0, 0.0],          # [x, y, theta] —— 纯 x 直线, 不横移 / 不转向
        max_velocities=[max_vel, max_vel, 0.0],
        sync=True,
        timeout=actual_timeout,
    )
    dt = time.perf_counter() - t0

    ok = isinstance(job, dict) and job.get("status") == "succeeded"
    status = job.get("status") if isinstance(job, dict) else None
    error = job.get("error") if isinstance(job, dict) else None

    print(f"  ✅ 底盘{direction} {abs(dist_mm):.0f}mm 完成 (status={status!r}, 耗时={dt:.2f}s)")

    if not ok:
        raise RuntimeError(
            f"{LOG_PREFIX} Phase 3c 底盘后退失败 (status={status!r}, error={error!r})"
        )
    return job


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 顶层编排层**不暴露** 5 个底层 run() 内部参数。

    原因: 用户明确要求"调用它们的 run()", 即让 5 个底层 run() 自己用默认值,
    不在顶层编排层覆写。如果未来需要调参, 直接调 new_target.run / target_all.run /
    from_*_to_*.run / dipan._run 的关键字参数 (这些 run 都已经把每个步骤的参数做成可调关键字)。
    """
    return argparse.ArgumentParser(
        description=(
            "task5/the_final v1: 端到端分拣入库流水线 (4 阶段)\n"
            "  Phase 1: new_target.run (识别高仓颜色)\n"
            "  Phase 2: target_all.run (识别球数: 黄/蓝 分桶)\n"
            "  Phase 3: 底盘 + 高/低塔循环 (matching → 高塔, opposite → 低塔)\n"
            "  默认耗时 ~30-60s (含底盘后退)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    client = ArmClient.connect()
    if not client.ping():
        raise RuntimeError("机械臂 runtime 未在线, 请检查 arm_feed 守护进程")
    runner = ArmRunner(client)
    result = run(client, runner)
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    sys.exit(main())