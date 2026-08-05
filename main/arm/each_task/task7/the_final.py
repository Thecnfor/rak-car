"""task7 / the_final —— **任务七完整编排** (liaobiao1+liaobiao2 对应匹配循环 → 投递)。

按用户 2026-08-05 现场修正后的循环逻辑 (v2):

  ┌─── 循环 (max N 次, 默认 10) ───┐
  │  Step 1: duiying.run()         target OCR + liebiao 比对, 拿到 matches
  │    ↓                           (matches: [{source, list_no, name, goods, target_id, target_label}])
  │  Step 2: 检查 matches            找到第一个 source=='liaobiao1' OR source=='liaobiao2'
  │    ↓ 是                         ↓ 否 (matches 空)
  │    ├ source=='liaobiao1':       ↓
  │    │  Step 3: runner.suck()     底盘前进 60cm (move_for +600mm)
  │    │  Step 4: get_position1()        ↓
  │    │  Step 5: position1()       再次 duiying → 回到 Step 1
  │    ├ source=='liaobiao2':       (下一轮继续走对应循环)
  │    │  Step 3: runner.suck()
  │    │  Step 4: get_position2()
  │    │  Step 5: position2()
  │    └ 命中后 break (任务完成)
  └─────────────────────────────────┘

  ⚠️ **v2 修正 (2026-08-05)**:
    - v1 只看 source=='liaobiao1', 用户现场修正: **liaobiao1 和 liaobiao2 都要对应**
    - match.source == 'liaobiao1' → runner.suck() + get_position1 + position1 → break
    - match.source == 'liaobiao2' → runner.suck() + get_position2 + position2 → break
    - **未命中分支也回到 Step 1** (重新调 duiying), 跟 v1 一致 (v1 已实现, 仅描述不准确)
    - v1 的 target_id==1 限制**取消**: 任何 source 匹配都触发对应 position (用户 2026-08-05 简化)

  ⚠️ 终止条件 (任一触发即停):
    1. 找到匹配 (source=='liaobiao1' OR source=='liaobiao2') → 执行完对应 get_position+position → break
    2. 达到 max_iterations 次 (默认 10) → 仍未匹配 → abort (防硬件无限循环)
    3. 任一步骤抛异常 → 立即 abort (跟 task6/the_final.py 同款硬件安全策略)

  ⚠️ matches 解析逻辑 (来自 duiying.run() 返回):
    - match.source == "liaobiao1": ✅ 触发 position1 分支
    - match.source == "liaobiao2": ✅ 触发 position2 分支
    - matches 中可能有多个匹配, 取第一个 (按 duiying 输出顺序, liaobiao1 优先)
    - matches == []: 未命中 → 底盘前进 + 重试

  ⚠️ **错误处理策略 (硬件安全优先)**:
    - duiying.run() 抛异常 → abort (返回 failed_step="iter{N}-duiying")
    - 底盘 move_for 失败 (status != succeeded) → abort
    - runner.suck() / get_position / position 内部异常 → 透传抛, abort
    - **不回滚**前面已成功的步骤, 让用户人工复位 + 查日志后继续

  ⚠️ **本文件编排 (orchestrator) 角色 — 与 task6/the_final.py / task7/duiying.py 同款**:
    task7 自包含条款字面只禁止 "不 import task7 包内任何模块" (见 position1.py:47-50),
    本文件**编排包内多个模块**, 故跨模块 import 破例允许。

  ⚠️ **不动手的事**:
    - 不修改 duiying / get_position* / position* / dipan 等已有文件
    - 不调用 suck 之外的真空管理 (不放气, 因为 position1/2 内部 Step 2.5/Step 6 已放气)
    - 不引入 pytest 单测 (跟 task7 其他脚本同款)

跑法:
    python main/arm/each_task/task7/the_final.py                  # 默认 max=10 迭代, 前进 60cm
    python -m main.arm.each_task.task7.the_final
    python main/arm/each_task/task7/the_final.py --max-iterations 5
    python main/arm/each_task/task7/the_final.py --forward 400    # 每次前进 40cm
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

# task7 编排器, import 包内其他模块 (编排破例, 见 docstring)
from main.arm.each_task.task7 import (  # noqa: E402
    duiying as duiying_mod,
    get_position1 as get_position1_mod,
    get_position2 as get_position2_mod,
    position1 as position1_mod,
    position2 as position2_mod,
)


# ---------- 序列常量 ----------

LOG_PREFIX: str = "[task7/the_final]"

DEFAULT_MAX_ITERATIONS: int = 10
"""最大迭代次数 (防硬件无限循环)。用户 2026-08-04 未指定, 默认 10 次足够覆盖
30cm × 10 = 6m 范围。"""

DEFAULT_FORWARD_MM: float = 600.0
"""未匹配时底盘前进距离 (mm)。用户 2026-08-04 指定 60cm = 600mm。
CLI 接收正值, 内部直接用 → move_for x_m = +dist_mm/1000 (前进)。
负值会被 ``abs()`` 强制取正再前进, 避免误后退错过摆位。"""

DEFAULT_CHASSIS_TIMEOUT_S: float = 10.0
"""底盘 move_for HTTP 同步超时兜底 (秒)。
600mm / 0.10 m/s ≈ 6s; 脚本按 max(5.0, |dist|/vel + 2.0) 自适应放大,
用户 ``--timeout`` 是这个自适应值的下限。"""

DEFAULT_CHASSIS_VELOCITY_MS: float = 0.10
"""底盘最大线速度 (m/s), 与 task7/position1.py / dipan.py 默认一致。"""


# ---------- 底盘 move_for 内联 (跟 task6/the_final.py / task7/position1.py 同款) ----------

def _chassis_move_for(client: ArmClient, dist_mm: float,
                      max_velocity_ms: float, timeout: float,
                      log_prefix: str) -> dict:
    """底盘相对位姿位移 (move_for)。sync=True 阻塞等闭环完成。

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


# ---------- 主流程 (循环编排) ----------

def run(client: ArmClient, runner: ArmRunner,
        max_iterations: int = DEFAULT_MAX_ITERATIONS,
        forward_mm: float = DEFAULT_FORWARD_MM,
        max_velocity_ms: float = DEFAULT_CHASSIS_VELOCITY_MS,
        timeout: float = DEFAULT_CHASSIS_TIMEOUT_S) -> dict:
    """任务七编排主循环: duiying 匹配 → 命中 (liaobiao1+target=1) 走投顾, 否则前进重试。

    ⚠️ 终止条件:
      - 命中 (matches 含 source=='liaobiao1' AND target_id==1) → 执行完 position1.run() → break
      - 达 max_iterations → abort (返回 ok=False, failed_step="max_iterations_reached")
      - 任一步骤抛异常 → 立即 abort, 不回滚

    Args:
        client: ArmClient
        runner: ArmRunner
        max_iterations: 最大迭代次数 (默认 10, 防硬件无限循环)
        forward_mm: 未匹配时底盘前进距离 (mm, 正值, 默认 600 = 60cm)
        max_velocity_ms: 底盘限速 (m/s, 默认 0.10)
        timeout: 底盘 move_for HTTP 同步超时下限 (秒, 默认 10)

    Returns:
        成功时 (匹配 + 执行完 position1):
        {
            "ok":         True,
            "matched_match":  dict,    # 命中的 match 详情 (source=list_no=1, name, goods, target_id=1)
            "iterations":  int,        # 实际迭代次数
            "results":     list,       # 每轮迭代结果 (dict)
            "duration_s":  float,
        }
        失败时:
        {
            "ok":           False,
            "failed_step":  str,        # "iter{N}-duiying" / "iter{N}-forward" / "max_iterations_reached"
            "error":        str | None,
            "iterations":  int,
            "results":      list,
            "duration_s":  float,
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (max_iterations={max_iterations}, "
          f"forward={forward_mm:.0f}mm) ==========")
    print(f"  循环逻辑: duiying → 匹配 [liaobiao1+target=1] → suck+get_position1+position1 (break)")
    print(f"            未匹配 → 底盘前进 {forward_mm:.0f}mm → 再次 duiying (max {max_iterations} 次)")
    print()

    iteration_results: list[dict] = []
    matched_match: dict | None = None

    for i in range(1, max_iterations + 1):
        print(f"\n{'=' * 60}")
        print(f"─── 迭代 {i}/{max_iterations}: duiying (target OCR + liebiao 比对) ───")
        print(f"{'=' * 60}")

        # ===== Step 1: duiying =====
        try:
            duiying_result = duiying_mod.run(client, runner)
        except Exception as exc:                                # noqa: BLE001
            print(f"\n  ❌ duiying.run() 异常: {exc!r}")
            print(f"\n========== {LOG_PREFIX} 中止 (迭代 {i} duiying 失败) ==========")
            return {
                "ok": False,
                "failed_step": f"iter{i}-duiying",
                "error": repr(exc),
                "iterations": len(iteration_results),
                "results": iteration_results,
                "duration_s": time.time() - t0,
            }

        matches: list[dict] = duiying_result.get("matches") or []
        print(f"  duiying.matches = {matches}")

        # ===== 检查匹配 (source=='liaobiao1' OR source=='liaobiao2', 任意 target_id) =====
        # v2 修正: 不只看 liaobiao1, 也要对照 liaobiao2。
        # target_id 不再限制 (v1 限制 target_id==1, v2 取消)。
        # 取第一个匹配 (duiying 输出顺序, liaobiao1 优先 if 都有)。
        target_match: dict | None = None
        for m in matches:
            if m.get("source") in ("liaobiao1", "liaobiao2"):
                target_match = m
                break

        if target_match is not None:
            # ===== 命中: 根据 source 分支执行 =====
            source = target_match.get("source")
            print(f"\n  ✅ 命中: source={source}")
            print(f"     人名={target_match['name']!r}, 蔬菜={target_match['goods']!r}")
            print(f"     target 位置 {target_match['target_id']} ({target_match['target_label']})")

            # Step 3: suck (无论 source 是哪个, 都先启动吸气)
            print(f"\n  ── Step 3: runner.suck() (启动吸气) ──")
            runner.suck()

            # Step 4 + 5: 按 source 分支
            if source == "liaobiao1":
                print(f"\n  ── Step 4: get_position1.run() (5 步纯臂: y up → x → arm → hand → y down) ──")
                get_position1_mod.run(client, runner)
                print(f"\n  ── Step 5: position1.run() (底盘后退 13cm + 7 步臂含 Step 2.5 放气 + 底盘前进 13cm) ──")
                position1_mod.run(client, runner)
            elif source == "liaobiao2":
                print(f"\n  ── Step 4: get_position2.run() (5 步纯臂, 跟 position1 同款但 x 略不同) ──")
                get_position2_mod.run(client, runner)
                print(f"\n  ── Step 5: position2.run() (底盘前进 + 7 步臂含 Step 6 放气 + 底盘后退) ──")
                position2_mod.run(client, runner)
            else:
                # 理论上不会到这里 (上面 for 循环只 accept liaobiao1/2), 但防御性 raise
                raise RuntimeError(f"未知的 match.source: {source!r}")

            matched_match = target_match
            iteration_results.append({
                "iter": i,
                "matched": True,
                "match": target_match,
            })
            dt = time.time() - t0
            print(f"\n========== {LOG_PREFIX} 任务完成 ({dt:.2f}s, 迭代 {i} 命中, source={source}) ==========")
            print(f"  ✅ 投递成功: 货物落到 {source} 位置, 真空已断 "
                  f"(position{'1' if source == 'liaobiao1' else '2'} Step 放气)")
            print(f"  ⚠️ 不放气 (下次编排需自己负责 suck/drop_object 周期)")
            return {
                "ok": True,
                "matched_match": matched_match,
                "iterations": i,
                "results": iteration_results,
                "duration_s": dt,
            }

        # ===== 未命中: 底盘前进 + 重试 (回到 Step 1 下一轮继续走对应循环) =====
        print(f"\n  ⚠️ 未命中 (matches 不含 liaobiao1/2) → 底盘前进 {forward_mm:.0f}mm, 下一轮重新 duiying")
        iteration_results.append({
            "iter": i,
            "matched": False,
            "matches": matches,
        })

        # 底盘前进 (强制转正, 即便用户传 --forward -50 也变成 +600 前进)
        forward_signed_mm = abs(forward_mm)
        forward_timeout = max(timeout, 5.0, abs(forward_signed_mm) / 1000.0 / max(max_velocity_ms, 0.01) + 2.0)
        try:
            _chassis_move_for(
                client, forward_signed_mm,
                max_velocity_ms, forward_timeout,
                log_prefix=f"  {LOG_PREFIX} iter{i}-forward",
            )
        except Exception as exc:                                # noqa: BLE001
            print(f"\n  ❌ 底盘前进异常: {exc!r}")
            print(f"\n========== {LOG_PREFIX} 中止 (迭代 {i} 底盘前进失败) ==========")
            return {
                "ok": False,
                "failed_step": f"iter{i}-forward",
                "error": repr(exc),
                "iterations": len(iteration_results),
                "results": iteration_results,
                "duration_s": time.time() - t0,
            }

    # ===== 达 max_iterations 仍未命中 =====
    dt = time.time() - t0
    print(f"\n========== {LOG_PREFIX} 中止 ({dt:.2f}s, 达 max_iterations={max_iterations} 仍未命中) ==========")
    print(f"  ⚠️ 硬件累计前进 {max_iterations} × {forward_signed_mm:.0f}mm = "
          f"{max_iterations * forward_signed_mm / 1000:.1f}m 仍未找到匹配项")
    print(f"  请人工检查:")
    print(f"    1. .liebiao.json 是否正确 (跑过 task6/target1.py?)")
    print(f"    2. duiying.target OCR 是否识别到 6 个名字")
    print(f"    3. 底盘当前物理位置是否在赛道内")
    return {
        "ok": False,
        "failed_step": "max_iterations_reached",
        "error": None,
        "iterations": max_iterations,
        "results": iteration_results,
        "duration_s": dt,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 4 个 (max-iterations / forward / vel / timeout), 其它常量都顶在文件里。"""
    p = argparse.ArgumentParser(
        description=(
            "task7 the_final: 对应匹配循环 (duiying → 匹配 [liaobiao1+target=1] → "
            "suck+get_position1+position1 break, 否则前进 60cm 重试, max 10 次)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--max-iterations", type=int, default=DEFAULT_MAX_ITERATIONS,
                   help="最大迭代次数 (防硬件无限循环, 默认 10)")
    p.add_argument("--forward", type=float, default=DEFAULT_FORWARD_MM,
                   help="未匹配时底盘前进距离 (mm, 默认 600 = 60cm, 强制正值)")
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
    result = run(client, runner,
                max_iterations=args.max_iterations,
                forward_mm=args.forward,
                max_velocity_ms=args.max_velocity,
                timeout=args.timeout)
    return 0 if result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
