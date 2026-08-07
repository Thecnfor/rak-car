#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务七: 外卖投放 —— **完整编排器** (self-contained)。

本文件**完整实现** task7 编排逻辑 (从 ``main.arm.each_task.task7.the_final`` 迁出),
不调外部 the_final.main, **可独立运行**。

# ⚙️ 整体逻辑 (按 2026-08-06 现场跑通顺序)

```
┌─── Step -1: 底盘前进 315mm (2026-08-06 用户新增预备) ───┐
│  _chassis_move_for(client, +315mm, vel=0.10, timeout≈auto)
│  ⚠️ 失败硬 abort (跟 pingcang 软警告不同: 这是物理位姿调整, 错了就跑不到正确起点)
└────────────────────────────────────────────────────────┘
                ↓
┌─── Step 0: pingcang 平仓 (储存仓 → +90°, 默认开) ───┐
│  pingcang_mod._run(client, angle=+90°, speed=100, timeout=10s)
│  ⚠️ 默认 strict=False: 失败只警告不阻塞主循环; --strict-pingcang 切 abort
└────────────────────────────────────────────────────┘
                ↓
┌─── 循环 (max N 次, 默认 2) ───┐
│                                │
│  iter i:                       │
│    ── Step 1: duiying.run()    │
│       target OCR (ERNIE VL) 6 个名字
│       + liebiao1/liaobiao2 比对
│       → matches: [{source, list_no, name, goods, target_id, target_label}]
│                                │
│    ── Step 2: 命中判读         │
│       0 个 match → 前进 600mm → 下一轮
│       1+ 个 match → 遍历每个 match 执行 3 步:
│                                │
│       每个 match 3 步:         │
│         Step 3a: runner.suck() (启动吸气)
│         Step 3b: get_position<source>.run()
│                  (4 步纯臂: composite + y_down + suck + y_up)
│                  source==liaobiao1 → the_final_get_position1 (x=0)
│                  source==liaobiao2 → the_final_get_position2 (x=-58)
│         Step 3c: position<target_id>.run()  (投递, 含底盘 + Step 放气)
│                  target_id 1-6 → the_final_position1-6
│                                │
│       全部执行完 → 前进 600mm → 下一轮
│                                │
│    最后一轮 (iter == max):     │
│       跳过前进 600mm 直接收尾  │
└────────────────────────────────┘
                ↓
        打印收尾 summary
```

# 🎯 关键常量

| 常量 | 值 | 说明 |
|---|---|---|
| `DEFAULT_MAX_ITERATIONS` | 2 | 用户 2026-08-05 现场定, 路上 1-2 个目标 |
| `DEFAULT_FORWARD_MM` | 600 | 60cm, 未匹配时底盘前进距离 |
| `DEFAULT_CHASSIS_VELOCITY_MS` | 0.10 | 底盘限速 |
| `DEFAULT_CHASSIS_TIMEOUT_S` | 10.0 | move_for HTTP 超时下限 (秒) |
| `DEFAULT_PINGCANG_ANGLE_DEG` | +90° | 储存仓角度 (复用 pingcang.py 默认) |
| `DEFAULT_PINGCANG_SPEED` | 100 | 舵机速度 |
| `DEFAULT_PINGCANG_TIMEOUT_S` | 10.0 | 平仓超时 |
| `DEFAULT_PINGCANG_ENABLED` | True | 默认自动平仓; `--skip-pingcang` 跳 |
| `DEFAULT_PINGCANG_STRICT` | False | 2026-08-06 v6: pingcang 失败软警告不阻塞 |

# 🛡️ 错误处理策略 (硬件安全优先)

- `pingcang` 失败: 默认软警告 (strict=False), `--strict-pingcang` 切 abort
- `duiying` 抛异常 → abort (`failed_step="iter{N}-duiying"`)
- 底盘 `move_for` 失败 (`status != succeeded`) → abort (`failed_step="iter{N}-forward"`)
- `runner.suck()` / `get_position<N>` / `position<N>` 内部异常 → 透传抛, abort
- **不回滚**前面已成功的步骤, 让用户人工复位 + 查日志后继续

# 📝 Dispatch 表 (match.target_id 1-6 → position 模块)

| target_id | module | 行为 |
|---|---|---|
| 1 | `the_final_position1` | 编排器: 后退 → 投臂 → 前进 |
| 2 | `the_final_position2` | 纯臂 3 步 (composite + drop + x 归零) |
| 3 | `the_final_position3` | 编排器: 前进 → 投臂 → 后退 |
| 4 | `the_final_position4` | 编排器: 后退 → 投臂 → 前进 |
| 5 | `the_final_position5` | 纯臂 7 步 |
| 6 | `the_final_position6` | 编排器: 前进 → 投臂 → 后退 |

# 跑法

```bash
# ✅ 推荐 (走 -m 包导入):
cd /home/jetson/workspace/rak-car
python -m main.task.task7_deliver
python -m main.task.task7_deliver --max-iterations 3
python -m main.task.task7_deliver --skip-pingcang --max-iterations 1
python -m main.task.task7_deliver --strict-pingcang    # pingcang 失败 abort

# ✅ 也支持 (本文件带 sys.path 注入, 可直接 python 跑):
python main/task/task7_deliver.py
python main/task/task7_deliver.py --max-iterations 1

# ✅ orchestrator 自动调 (TASK_RUNNERS[7]):
from main.task import TASK_RUNNERS
TASK_RUNNERS[7](client=None)
```

# ⚠️ 跟 main/arm/each_task/task7/the_final.py 关系

本文件逻辑**完全等价** the_final.run() (除去 import 路径不同),
不再调 the_final.main 是为了**业务层独立可读** — 一个文件能跑完整个 task7,
现场调试时不需要跳到 each_task 包内。

如果 the_final.py 后续再演进 (新 match 处理 / 新 dispatch), 需同步本文件。
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Any, Dict, List, Optional

# ========== sys.path 注入 (可独立 python 跑) ==========
_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

# ========== 业务模块 import ==========
from main.api_client import RuntimeApiClient  # noqa: E402
from main.arm import ArmClient, ArmRunner  # noqa: E402
from main.arm.each_task.task7 import (  # noqa: E402
    duiying as duiying_mod,                                      # Step 1: target OCR + liebiao 比对
    pingcang as pingcang_mod,                                    # Step 0: 储存仓 → +90° 平仓
    the_final_get_position1 as get_position1_mod,                # Step 3b (source==liaobiao1): 4 步纯臂抓取 x=0
    the_final_get_position2 as get_position2_mod,                # Step 3b (source==liaobiao2): 4 步纯臂抓取 x=-58
    the_final_position1 as position1_mod,                        # Step 3c (target_id=1): 编排器 后退+投臂+前进
    the_final_position2 as position2_mod,                        # Step 3c (target_id=2): 纯臂 3 步
    the_final_position3 as position3_mod,                        # Step 3c (target_id=3): 编排器 前进+投臂+后退
    the_final_position4 as position4_mod,                        # Step 3c (target_id=4): 编排器 后退+投臂+前进
    the_final_position5 as position5_mod,                        # Step 3c (target_id=5): 纯臂 7 步
    the_final_position6 as position6_mod,                        # Step 3c (target_id=6): 编排器 前进+投臂+后退
)


# ========== 序列常量 ==========

LOG_PREFIX: str = "[task7_deliver]"

# ---- 底盘 / 循环 ----

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
"""底盘最大线速度 (m/s), 与 task7 现场标定一致。"""

DEFAULT_PRE_FORWARD_MM: float = 315.0
"""任务起点预备动作: 底盘前进距离 (mm)。
2026-08-06 用户要求在 run() 最前面加 315mm 前进, 跟 pingcang 平仓同款"预备"角色,
其他逻辑不变。
CLI ``--skip-pre-forward`` 跳过。"""

DEFAULT_PRE_FORWARD_ENABLED: bool = True
"""是否在 run() 开头自动前进 315mm。True (默认) = 每次跑都先前进;
CLI ``--skip-pre-forward`` 改 False (调试主循环时不真动底盘)。"""

# ---- pingcang (平仓) ----

DEFAULT_PINGCANG_ANGLE_DEG: float = pingcang_mod.DEFAULT_ANGLE_DEG
"""平仓角度 (raw 协议值, 度)。默认 +90°, 合法区间 [-128, 127]。
直接复用 pingcang.py 的 DEFAULT_ANGLE_DEG 常量, 改 pingcang 同步生效。"""

DEFAULT_PINGCANG_SPEED: int = pingcang_mod.DEFAULT_SPEED
"""平仓舵机速度 (1-100), 默认 100 = 全速。复用 pingcang.py 常量。"""

DEFAULT_PINGCANG_TIMEOUT_S: float = pingcang_mod.DEFAULT_TIMEOUT_S
"""平仓 HTTP 同步超时 (秒), 默认 10s。复用 pingcang.py 常量。"""

DEFAULT_PINGCANG_ENABLED: bool = True
"""是否在 run() 开头自动平仓。True (默认) = 每次跑都先平仓;
CLI ``--skip-pingcang`` 改 False (调试循环时不真动舵机)。"""

DEFAULT_PINGCANG_STRICT: bool = False
"""平仓失败是否 abort。False (默认, 2026-08-06 v6 改) = pingcang 失败**只警告不阻塞**,
让主循环照常跑; True = 维持原 abort 策略 (硬件安全优先)。

⚠️ **业务正确性提醒**: pingcang 失败 = 储存仓角度未复位, 后续投球可能撞车/投偏。
这是**业务折衷**: 不阻塞 vs 物理安全。"""

# ---- Dispatch 表 ----
# match.target_id (1-6) → position{N}_mod.run() 投递脚本。
# position1/3/4/6 是编排器 (底盘 + 委托 the_final_position2/5 + 底盘),
# position2/5 是纯臂脚本 (7/3 步臂流程, 含放气)。
_POSITION_MODULES: Dict[int, Any] = {
    1: position1_mod,
    2: position2_mod,
    3: position3_mod,
    4: position4_mod,
    5: position5_mod,
    6: position6_mod,
}


# ========== 底盘 move_for 内联 (跟 the_final.py / position1.py 同款) ==========

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


# ========== 主流程 (循环编排) ==========

def run(client: ArmClient, runner: ArmRunner,
        max_iterations: int = DEFAULT_MAX_ITERATIONS,
        forward_mm: float = DEFAULT_FORWARD_MM,
        max_velocity_ms: float = DEFAULT_CHASSIS_VELOCITY_MS,
        timeout: float = DEFAULT_CHASSIS_TIMEOUT_S,
        do_pre_forward: bool = DEFAULT_PRE_FORWARD_ENABLED,
        pre_forward_mm: float = DEFAULT_PRE_FORWARD_MM,
        do_pingcang: bool = DEFAULT_PINGCANG_ENABLED,
        strict_pingcang: bool = DEFAULT_PINGCANG_STRICT,
        pingcang_angle_deg: float = DEFAULT_PINGCANG_ANGLE_DEG,
        pingcang_speed: int = DEFAULT_PINGCANG_SPEED,
        pingcang_timeout: float = DEFAULT_PINGCANG_TIMEOUT_S) -> dict:
    """任务七主入口: Step -1 pre-forward (前进 315mm) → Step 0 pingcang → 循环 Step 1-3。

    终止条件 (任一触发即停):
      1. 达到 ``max_iterations`` 次 (默认 2) → 正常收尾 (ok=True 退出, v4/v6 行为)
      2. 任一步骤抛异常 → 立即 abort (不回滚)
      3. ``strict_pingcang=True`` 时 pingcang 失败 → abort (旧硬件安全策略)

    Args:
        client: ArmClient
        runner: ArmRunner
        max_iterations: 最大迭代次数 (默认 2)
        forward_mm: 未匹配时底盘前进距离 (mm, 默认 600)
        max_velocity_ms: 底盘限速 (m/s, 默认 0.10)
        timeout: 底盘 move_for HTTP 同步超时下限 (秒, 默认 10)
        do_pre_forward: True (默认) 任务开头先跑一次底盘前进 ``pre_forward_mm`` mm,
                    False 跳过 (调试主循环时不真动底盘)。
        pre_forward_mm: 预备前进距离 (mm, 默认 315)。
        do_pingcang: True 自动跑 pingcang, False 跳过
        strict_pingcang: False (默认) pingcang 失败软警告, True 维持 abort
        pingcang_angle_deg: 储存仓角度 (度, 默认 +90°)
        pingcang_speed: 舵机速度 1-100, 默认 100
        pingcang_timeout: HTTP 同步超时 (秒, 默认 10)

    Returns:
        Dict:
        {
            "ok": True / False,
            "failed_step": str | None,         # "pre_forward" / "pingcang" / "iter{N}-duiying" / "iter{N}-forward" / None
            "error": str | None,
            "iterations": int,
            "total_matches": int,
            "total_empty": int,
            "results": list[dict],
            "duration_s": float,
            "pingcang_result": dict | None,
        }
    """
    t0 = time.time()
    print(f"\n========== {LOG_PREFIX} run (max_iterations={max_iterations}, "
          f"forward={forward_mm:.0f}mm) ==========")
    print(f"  循环逻辑: duiying → 匹配 [liaobiao1+target=1] → suck+get_position1+position1 (break)")
    print(f"            未匹配 → 底盘前进 {forward_mm:.0f}mm → 再次 duiying (max {max_iterations} 次)")
    print(f"  起点预备: 前进={pre_forward_mm:.0f}mm ({'开' if do_pre_forward else '关'})")
    print(f"            平仓={'开' if do_pingcang else '关'} "
          f"(angle={pingcang_angle_deg:+.0f}°, speed={pingcang_speed}, "
          f"timeout={pingcang_timeout:.0f}s, "
          f"strict={'开 (失败 abort)' if strict_pingcang else '关 (失败软警告, 继续主循环)'})")
    print()

    iteration_results: List[Dict] = []
    matched_match: Optional[Dict] = None
    pingcang_result: Optional[Dict] = None

    # ===== Step -1: 底盘前进 315mm (任务起点预备, 2026-08-06 用户新增) =====
    # ⚠️ 失败硬 abort (不软警告): 这是物理位姿调整, 错了就跑不到正确起点,
    # 跟 pingcang "可软警告" 不同 (pingcang 失败只是角度未复位, 后续业务可能勉强跑)。
    if do_pre_forward:
        pre_forward_timeout = max(timeout, 5.0, abs(pre_forward_mm) / 1000.0 / max(max_velocity_ms, 0.01) + 2.0)
        try:
            _chassis_move_for(
                client, pre_forward_mm,
                max_velocity_ms, pre_forward_timeout,
                log_prefix=f"  {LOG_PREFIX} pre-forward",
            )
        except Exception as exc:                                # noqa: BLE001
            print(f"\n  ❌ 底盘预前进 {pre_forward_mm:.0f}mm 异常: {exc!r}")
            print(f"\n========== {LOG_PREFIX} 中止 (预前进失败) ==========")
            return {
                "ok": False,
                "failed_step": "pre_forward",
                "error": repr(exc),
                "iterations": 0,
                "results": [],
                "duration_s": time.time() - t0,
            }
        print(f"  ➡️ 预前进完成, 进入平仓")

    # ===== Step 0: 平仓 (任务起点先把储存仓摆到 +90°) =====
    if do_pingcang:
        try:
            pingcang_result = pingcang_mod._run(
                client,
                angle_deg=pingcang_angle_deg,
                speed=pingcang_speed,
                timeout=pingcang_timeout,
            )
        except Exception as exc:                                # noqa: BLE001
            print(f"\n  ⚠️ pingcang 平仓异常: {exc!r}")
            if strict_pingcang:
                # 旧策略 (strict=True): 硬件安全优先, 立即 abort
                print(f"\n========== {LOG_PREFIX} 中止 (pingcang 失败, strict 模式) ==========")
                return {
                    "ok": False,
                    "failed_step": "pingcang",
                    "error": repr(exc),
                    "iterations": 0,
                    "results": [],
                    "duration_s": time.time() - t0,
                }
            # 新策略 (strict=False, 默认): 软警告继续
            print(f"  ➡️ strict_pingcang=False, 警告继续, 进主循环 "
                  f"(储存仓角度未复位, 后续投球需谨慎)")
            pingcang_result = {"ok": False, "error": repr(exc), "strict": False}
            print()
        else:
            print(f"  ➡️ 平仓完成, 进入主循环")

    # ===== 主循环 =====
    for i in range(1, max_iterations + 1):
        print(f"\n{'=' * 60}")
        print(f"─── 迭代 {i}/{max_iterations}: duiying (target OCR + liebiao 比对) ───")
        print(f"{'=' * 60}")

        # ===== Step 1: duiying (target OCR + liebiao 比对) =====
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

        matches: List[Dict] = duiying_result.get("matches") or []
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
            # ===== 1+ 个 valid match: 遍历执行每个 match 的 3 步 =====
            print(f"\n  ✅ 命中: {len(valid_matches)} 个有效 match, 依次执行")
            executed_matches: List[Dict] = []
            for match_idx, match in enumerate(valid_matches, 1):
                source = match.get("source")
                target_id = match.get("target_id")
                print(f"\n  ── Match {match_idx}/{len(valid_matches)}: source={source}, target_id={target_id} ───")
                print(f"     人名={match['name']!r}, 蔬菜={match['goods']!r}, "
                      f"target 标签={match.get('target_label')!r}")

                # ---- Step 3a: suck (启动吸气, 每个 match 前都需要) ----
                print(f"  [Step 3a] runner.suck() (启动吸气)")
                runner.suck()

                # ---- Step 3b: get_position based on source ----
                if source == "liaobiao1":
                    print(f"  [Step 3b] get_position1.run() (4 步纯臂抓取, liaobiao1 入口, x=0)")
                    get_position1_mod.run(client, runner)
                elif source == "liaobiao2":
                    print(f"  [Step 3b] get_position2.run() (4 步纯臂抓取, liaobiao2 入口, x=-58)")
                    get_position2_mod.run(client, runner)
                else:
                    raise RuntimeError(
                        f"match.source={source!r} 不是 'liaobiao1'/'liaobiao2' (已被过滤掉, "
                        f"理论上到这里不可能)"
                    )

                # ---- Step 3c: position based on target_id (1-6) ----
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

            # ===== 本轮命中完, 准备前进 =====
            print(f"\n  ➡️ 本轮 {len(executed_matches)} 个 match 全部执行完, "
                  f"准备前进 {forward_mm:.0f}mm 进入下一轮 duiying")

        # ===== 统一前进 (最后一轮跳过) =====
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
        else:
            print(f"\n  ⏹️ 最后一轮 (iter {i}/{max_iterations}), 跳过前进 60cm 直接收尾")

    # ===== 收尾 =====
    dt = time.time() - t0
    total_matches = sum(1 for r in iteration_results if r.get("matched"))
    total_empty = sum(1 for r in iteration_results if not r.get("matched"))
    print(f"\n========== {LOG_PREFIX} 收尾 ({dt:.2f}s, "
          f"跑完 {max_iterations} 轮, 命中 {total_matches} 轮 / 空 {total_empty} 轮) ==========")
    actual_forwards = max(0, max_iterations - 1)
    print(f"  ⚠️ 硬件累计前进 {actual_forwards} × {forward_mm:.0f}mm = "
          f"{actual_forwards * forward_mm / 1000:.1f}m "
          f"(最后一轮 iter {max_iterations} 不前进, 避免白白走 60cm)")
    print(f"  请人工检查:")
    print(f"    1. .liebiao.json 是否正确 (跑过 task6/target1.py?)")
    print(f"    2. duiying.target OCR 是否识别到 6 个名字")
    print(f"    3. 底盘当前物理位置是否在赛道内")
    return {
        "ok": True,
        "failed_step": None,
        "error": None,
        "iterations": max_iterations,
        "total_matches": total_matches,
        "total_empty": total_empty,
        "results": iteration_results,
        "duration_s": dt,
        "pingcang_result": pingcang_result,
    }


# ========== TASK_RUNNERS 兼容 (orchestrator 调用入口) ==========

def runner_entry(client: Optional[RuntimeApiClient] = None,
                 argv: Optional[List[str]] = None) -> Dict[str, Any]:
    """``TASK_RUNNERS[7]`` 入口 (跟 task1/2/3/4/5/6 签名一致)。

    Args:
        client: 形式参数, 实际被忽略 (内部自建 ArmClient, 跟 task5 同款约定)。
        argv: 透传给 ``build_parser().parse_args`` 的 CLI 参数列表。
            orchestrator 调用时显式传 ``[]``, 不读 ``sys.argv`` (避免把
            ``run.py --task 7`` 误当成本任务的参数)。

    Returns:
        Dict: {"ok": bool, "task": "task7_deliver", "rc": int, "detail": str}
    """
    if argv is None:
        argv = []
    args = build_parser().parse_args(argv)
    rc = main_with_args(args)
    return {
        "ok": (rc == 0),
        "task": "task7_deliver",
        "rc": rc,
        "detail": "see logs (pingcang + duiying + matches + position*)",
    }


# ========== CLI ==========

def build_parser() -> argparse.ArgumentParser:
    """CLI 参数: max-iterations / forward / vel / timeout / pingcang 相关 6 个。"""
    p = argparse.ArgumentParser(
        prog="task7_deliver",
        description=(
            "task7_deliver v6: 自包含完整编排器 (pingcang → duiying → matches dispatch → 前进)"
            "\n逻辑等价 main.arm.each_task.task7.the_final.run(), 独立可读 + 可独立运行。"
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
    p.add_argument("--skip-pre-forward", dest="skip_pre_forward", action="store_true",
                   default=False,
                   help="跳过任务开头的预备前进 315mm (默认 False = 自动跑)")
    p.add_argument("--pre-forward-mm", type=float, default=DEFAULT_PRE_FORWARD_MM,
                   help=f"预备前进距离 (mm, 默认 {DEFAULT_PRE_FORWARD_MM:.0f}, "
                        f"正数 = 前进, 内部 abs() 强制)")
    p.add_argument("--skip-pingcang", dest="skip_pingcang", action="store_true",
                   default=False,
                   help="跳过任务开头的平仓动作 (默认 False = 自动跑 pingcang 把储存仓摆到 +90°)")
    p.add_argument("--strict-pingcang", dest="strict_pingcang", action="store_true",
                   default=DEFAULT_PINGCANG_STRICT,
                   help="平仓失败是否 abort (默认 False = 软警告继续主循环; "
                        "True = 维持旧 abort 策略, 硬件安全优先)")
    p.add_argument("--pingcang-angle", type=float,
                   default=DEFAULT_PINGCANG_ANGLE_DEG,
                   help=f"平仓角度 (度, 合法区间 [-128, 127], 默认 {DEFAULT_PINGCANG_ANGLE_DEG:+.0f})")
    p.add_argument("--pingcang-speed", type=int,
                   default=DEFAULT_PINGCANG_SPEED,
                   help=f"平仓舵机速度 (1-100, 默认 {DEFAULT_PINGCANG_SPEED})")
    p.add_argument("--pingcang-timeout", type=float,
                   default=DEFAULT_PINGCANG_TIMEOUT_S,
                   help=f"平仓 HTTP 同步超时 (秒, 默认 {DEFAULT_PINGCANG_TIMEOUT_S:.0f})")
    return p


def main_with_args(args: argparse.Namespace) -> int:
    """已 parse 完 args 的主入口。"""
    client = ArmClient.connect()
    runner = ArmRunner(client)
    result = run(client, runner,
                 max_iterations=args.max_iterations,
                 forward_mm=args.forward,
                 max_velocity_ms=args.max_velocity,
                 timeout=args.timeout,
                 do_pre_forward=not args.skip_pre_forward,
                 pre_forward_mm=args.pre_forward_mm,
                 do_pingcang=not args.skip_pingcang,
                 strict_pingcang=args.strict_pingcang,
                 pingcang_angle_deg=args.pingcang_angle,
                 pingcang_speed=args.pingcang_speed,
                 pingcang_timeout=args.pingcang_timeout)
    return 0 if result["ok"] else 1


def main(argv: Optional[List[str]] = None) -> int:
    """CLI 入口: 让本文件可直接 ``python main/task/task7_deliver.py`` 跑。

    orchestrator 通过 ``TASK_RUNNERS[7](client=None)`` 调用时, 也走这条路径
    (经 runner_entry 包装, 内部仍调 main)。
    """
    args = build_parser().parse_args(argv)
    return main_with_args(args)


if __name__ == "__main__":
    sys.exit(main())
