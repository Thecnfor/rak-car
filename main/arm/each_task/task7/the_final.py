"""task7 / the_final —— **任务七完整编排** (liaobiao1+liaobiao2 对应匹配循环 → 双位置投递)。

按用户 2026-08-05 现场修正后的循环逻辑 (v3):

  ┌─── 循环 (max N 次, 默认 2) ───┐
  │  Step 1: duiying.run()         target OCR + liebiao 比对, 拿到 matches
  │    ↓                           (matches: [{source, list_no, name, goods, target_id, target_label}])
  │  Step 2: 检查 matches            matches 数: 0 / 1 / 2
  │    ↓ 0 个                       ↓ 1+ 个
  │    底盘前进 60cm                遍历每个 match, 每个独立执行:
  │    再次 duiying (回 Step 1)      ├ match.source=='liaobiao1' → runner.suck() + get_position1()
  │                                   └ match.source=='liaobiao2' → runner.suck() + get_position2()
  │                                   然后按 match.target_id (1-6) 调 position1/2/3/4/5/6.py
  │                                全部执行完 → break (任务完成)
  └─────────────────────────────────┘

  ⚠️ **v4 修正 (2026-08-05)**:
    - v3 命中后 break (跑一次就结束), 用户现场确认:
      **命中后也不 break, 跟无匹配一样都前进 60cm, 然后下一轮重新走对应循环**。
    - 两路 (有/无 match) 统一底盘前进 + continue, 跑到 max_iterations 才停。
    - 修了 v3 的 dead code bug: 之前 `return {...ok: True...}` 让底盘前进 (line 365-384) 永远跑不到。
    - v3 的 matches 遍历 + dispatch (source→get_position, target_id→position) 完整保留。

  ⚠️ **v3 修正 (2026-08-05)**:
    - v2 只处理"第一个匹配"且只 dispatch liaobiao1→position1, liaobiao2→position2
    - v3 修正:
      1. **遍历所有 matches** (可能有 2 个: 一个 liaobiao1 + 一个 liaobiao2)
      2. 每个 match 独立走: ``runner.suck() → get_position<source> → position<target_id>``
      3. ``source`` (liaobiao1/2) 决定 get_position (1 或 2)
      4. ``target_id`` (1-6) 决定 position (1-6), 不是只 1/2
      5. 0 个 match → 底盘前进 60cm → 回到 Step 1 (跟 v2 一致)
      6. v2 的 "只第一个匹配 + target_id==1 限制" 全部取消

  ⚠️ **dispatch 表** (target_id 1-6 → position 模块):
    | target_id | module         |
    |-----------|----------------|
    | 1         | position1_mod  |  (底盘后退 + 7 步臂含 Step 2.5 放气 + 底盘前进, v4)
    | 2         | position2_mod  |  (底盘前进 + 7 步臂含 Step 6 放气 + 底盘后退, v4)
    | 3         | position3_mod  |  (底盘前进 + 7 步臂含 Step 2.5 放气 + 底盘后退, v4)
    | 4         | position4_mod  |  (底盘后退 + 10 步臂含 Step 2.06 放气 + 底盘前进, v3)
    | 5         | position5_mod  |  (无底盘, 10 步臂含 Step 7 放气, v3)
    | 6         | position6_mod  |  (底盘前进 + 10 步臂含 Step 2.06 放气 + 底盘后退, v3)

  ⚠️ 终止条件 (任一触发即停):
    1. 达到 max_iterations 次 (默认 2) → 正常收尾 (v4: 不是 abort, 是 ok=True 退出)
    2. 任一步骤抛异常 → 立即 abort (跟 task6/the_final.py 同款硬件安全策略)
    ⚠️ v4: 命中不再 break (用户 2026-08-05 现场要求: 命中后前进 60cm 继续下一轮)
    ⚠️ 默认 2 轮足够 (找 1 个目标 + 60cm 后再看 1 次); 跑 N 轮 --max-iterations=N

  ⚠️ **每个 match 执行三步** (顺序固定):
    1. ``runner.suck()`` — 启动吸气 (建立真空)
    2. ``get_position<source>.run()`` — 5 步纯臂摆位:
       - source=='liaobiao1' → ``get_position1_mod.run()``
       - source=='liaobiao2' → ``get_position2_mod.run()``
    3. ``position<target_id>.run()`` — 投递 (含 Step 放气):
       - target_id 1-6 → ``position{1,2,3,4,5,6}_mod.run()``

  ⚠️ matches 解析 (来自 duiying.run() 返回):
    - 0 个 match → 未命中 → 底盘前进 + 重试
    - 1 个 match (任一 source) → 执行 1 次
    - 2 个 match (source=liaobiao1 + source=liaobiao2 各 1) → 执行 2 次 (按 matches 列表顺序)
    - match.source ∈ {'liaobiao1', 'liaobiao2'}, match.target_id ∈ {1, 2, 3, 4, 5, 6}
    - 其它 source/target_id 视为异常 (防御性 raise)

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
    - 不调用 suck 之外的真空管理 (不放气, 因为 position<N> 内部 Step 放气)
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
    position3 as position3_mod,
    position4 as position4_mod,
    position5 as position5_mod,
    position6 as position6_mod,
)


# ---------- 序列常量 ----------

LOG_PREFIX: str = "[task7/the_final]"

DEFAULT_MAX_ITERATIONS: int = 2
"""最大迭代次数 (防硬件无限循环)。用户 2026-08-05 现场改为 2 次:
典型场景: 路上 1-2 个目标, duiying #1 找第一个 + duiying #2 看 60cm 后还有没有。
若还有, 下一轮跑 (但默认只跑 2 轮, 想多跑 --max-iterations=N)。"""

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

# Dispatch 表: match.target_id (1-6) → 对应 position 模块
# v3 加: 不再只 dispatch 到 position1/2, 而是根据 target_id 调 position1-6。
_POSITION_MODULES: dict[int, object] = {
    1: position1_mod,
    2: position2_mod,
    3: position3_mod,
    4: position4_mod,
    5: position5_mod,
    6: position6_mod,
}
"""match.target_id (1-6) → position{N}.run() 模块引用。

⚠️ 每个 position 模块内含底盘 + 7-10 步臂 + Step 放气 (v3/v4 状态)。
⚠️ key 必须是 1-6 范围; 其它值在 valid_matches 过滤时被丢弃, run() 不会再到这里。"""


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

        # ===== 过滤合法 matches (source ∈ liaobiao1/2 AND target_id ∈ 1-6) =====
        valid_matches = [
            m for m in matches
            if m.get("source") in ("liaobiao1", "liaobiao2")
            and m.get("target_id") in (1, 2, 3, 4, 5, 6)
        ]

        if not valid_matches:
            # ===== 0 个 match: 标记 + 准备前进 =====
            print(f"\n  ⚠️ 0 个有效 match → 本轮执行跳过, 前进 {forward_mm:.0f}mm 后下一轮重新 duiying")
            iteration_results.append({
                "iter": i,
                "matched": False,
                "matches": matches,
                "valid_count": 0,
            })
        else:
            # ===== 1+ 个 valid match: 遍历执行每个 =====
            print(f"\n  ✅ 命中: {len(valid_matches)} 个有效 match, 依次执行")
            executed_matches: list[dict] = []
            for match_idx, match in enumerate(valid_matches, 1):
                source = match.get("source")
                target_id = match.get("target_id")
                print(f"\n  ── Match {match_idx}/{len(valid_matches)}: source={source}, target_id={target_id} ───")
                print(f"     人名={match['name']!r}, 蔬菜={match['goods']!r}, "
                      f"target 标签={match.get('target_label')!r}")

                # Step 3a: suck (启动吸气, 每个 match 前都需要)
                print(f"  [Step 3a] runner.suck() (启动吸气)")
                runner.suck()

                # Step 3b: get_position based on source
                if source == "liaobiao1":
                    print(f"  [Step 3b] get_position1.run() (5 步纯臂摆位, liaobiao1 入口)")
                    get_position1_mod.run(client, runner)
                elif source == "liaobiao2":
                    print(f"  [Step 3b] get_position2.run() (5 步纯臂摆位, liaobiao2 入口)")
                    get_position2_mod.run(client, runner)
                else:
                    raise RuntimeError(
                        f"match.source={source!r} 不是 'liaobiao1'/'liaobiao2' (已被过滤掉, "
                        f"理论上到这里不可能)"
                    )

                # Step 3c: position based on target_id (1-6)
                pos_mod = _POSITION_MODULES.get(target_id)
                if pos_mod is None:
                    raise RuntimeError(
                        f"match.target_id={target_id!r} 不在 1-6 范围 (已被过滤掉, 理论上到这里不可能)"
                    )
                print(f"  [Step 3c] position{target_id}_mod.run() "
                      f"(投递到位置 {target_id}, position 内部 Step 放气)")
                pos_mod.run(client, runner)

                executed_matches.append(match)
                iteration_results.append({
                    "iter": i,
                    "matched": True,
                    "match": match,
                })
                print(f"  ✅ Match {match_idx}/{len(valid_matches)} 完成 (source={source}, "
                      f"target_id={target_id})")

            # v4 改动: 命中后**不 break**, 跟"无匹配"一样都前进 60cm 然后继续下一轮
            print(f"\n  ➡️ 本轮 {len(executed_matches)} 个 match 全部执行完, "
                  f"准备前进 {forward_mm:.0f}mm 进入下一轮 duiying")

        # ===== 统一前进 (无论有/无 match, 都前进 60cm + 下一轮继续) =====
        # v4 改动: 之前命中分支提前 return 导致这段 dead code, 现在合并到统一处理
        # v5 改动 (2026-08-06): **最后一轮不前进** (i == max_iterations 时跳过)
        # 原因: 最后一轮跑完 duiying 后没有"下一轮"需要进入, 前进 60cm 是白白浪费,
        # 且会让车停在 60cm 偏移的位置 (后续编排不知道这是 the_final 末尾)。
        if i < max_iterations:
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
            # 底盘前进成功 → 继续下一轮 duiying (回到 for 开头)
        else:
            print(f"\n  ⏹️ 最后一轮 (iter {i}/{max_iterations}), 跳过前进 60cm 直接收尾")

    # ===== 达 max_iterations 收尾 =====
    # v4 改动: 之前是 "max_iterations 仍未命中 → abort", 现在是 "max_iterations 收尾 (不论命中与否)"
    dt = time.time() - t0
    total_matches = sum(1 for r in iteration_results if r.get("matched"))
    total_empty = sum(1 for r in iteration_results if not r.get("matched"))
    print(f"\n========== {LOG_PREFIX} 收尾 ({dt:.2f}s, "
          f"跑完 {max_iterations} 轮, 命中 {total_matches} 轮 / 空 {total_empty} 轮) ==========")
    # v5 改动: 最后一轮 (iter max_iterations) 不前进, 累计 = (max-1) × forward
    actual_forwards = max(0, max_iterations - 1)
    print(f"  ⚠️ 硬件累计前进 {actual_forwards} × {forward_signed_mm:.0f}mm = "
          f"{actual_forwards * forward_signed_mm / 1000:.1f}m "
          f"(最后一轮 iter {max_iterations} 不前进, 避免白白走 60cm)")
    print(f"  ⚠️ 本脚本无 break-on-match, 跑到 max_iterations 才停。如想提前停, "
          f"用 max-iterations=N 减小上限。")
    print(f"  请人工检查:")
    print(f"    1. .liebiao.json 是否正确 (跑过 task6/target1.py?)")
    print(f"    2. duiying.target OCR 是否识别到 6 个名字")
    print(f"    3. 底盘当前物理位置是否在赛道内")
    return {
        "ok": True,                          # v4: 跑完 max_iterations 是正常退出, 不是 abort
        "failed_step": None,
        "error": None,
        "iterations": max_iterations,
        "total_matches": total_matches,
        "total_empty": total_empty,
        "results": iteration_results,
        "duration_s": dt,
    }


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: 4 个 (max-iterations / forward / vel / timeout), 其它常量都顶在文件里。"""
    p = argparse.ArgumentParser(
        description=(
            "task7 the_final v4: 对应匹配循环 (duiying → 匹配 [liaobiao1/2+target=1-6] → "
            "suck+get_position+position → 底盘前进 60cm → 下一轮继续, max 10 次, 无 break-on-match)"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--max-iterations", type=int, default=DEFAULT_MAX_ITERATIONS,
                   help="最大迭代次数 (防硬件无限循环, 默认 2)")
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
