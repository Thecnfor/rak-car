#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/shoot_target.py - 智能车射击任务 (2026-08-03 完全重写)

**任务目标**:
- 用户前进路上有 4 个目标板,板中心距 8cm
- 起点 cam 视野可能只看到 2~3 只目标,从能看到的开始 L→R 依次为 #1 #2 #3 #4
- 用户指定要射的目标编号 (--targets "1 3 4"),可对 yaw 微调对准
- 若没击倒 → 前后调整位置 或 调整 yaw → 继续射,直至击倒 或 射击次数超过 5 次
- 若成功击倒 → 向前移动 8cm 到下一个目标的射击点
- 击倒完最后一个目标后停止移动
- 记录击倒的是第几个目标,不要将其他目标重新记为这个目标

**板上号语义 (关键约定)**:
- 板上 #k = 物理位置 (起点 L→R 第 k 只板)
- 板上号 = 板上物理位置,**与 cam xc 无关**
- 击倒板上 #k → hit_set 添加 #k
- in_view_idx = shot_seq - len(hit_set)
  - hit_set 让 cam 视野 L→R 索引自动"压缩" (已击倒的板不再占用 cam 视野位置)
  - 不依赖 xc 容差,不会误排除场上合法的板上 #k

**为什么不用 banned_xcs (xc 容差)**:
- 直行 8cm 后,板上 #k 的 cam xc 会变化 (板变近 → xc 漂向 0.5)
- 旧的 banned_xcs 容差 (0.15) 会在新 xc 撞上旧 xc 时**误排除场上仍合法的板上 #k**
- 用 hit_set (板上号) 计数 → 不依赖 xc → 不会误排除

**用法**:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    PYTHONIOENCODING=utf-8 python -m main.tasks.task333.shoot_target --targets '"1 3 4"'
    PYTHONIOENCODING=utf-8 python -m main.tasks.task333.shoot_target    # 询问
    PYTHONIOENCODING=utf-8 python -m main.tasks.task333.shoot_target --targets all
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from main.api_client import RuntimeApiClient
from main.tasks.task333.shoot_4_targets import (
    bbox_xc,
    bbox_yc,
    bbox_width,
    get_animals,
    get_animals_retry,
)


# ============================================================================
# 常量 (集中,只在一处定义)
# ============================================================================

# 几何
N_TOTAL_BOARDS = 4                 # 板上总数 (用户硬约束)
BOARD_SPACING_M = 0.16             # **2026-08-03 修正**: 板中心距 16cm (用户硬约束)
                                   # 实测: 板宽 8cm + 间距 8cm → 中心距 16cm
                                   # 从 #1 前进到 #3 需 2 × 16 = 32cm (用户实测)
TARGET_BOARD_WIDTH_M = 0.08        # 板宽度 8cm (用户硬约束)

# 算法阈值
MAX_SHOTS_PER_BOARD = 5            # 每只板上最多 5 发 (用户硬约束)
MIN_YOLO_SCORE = 0.30              # YOLO 置信度阈值
HIT_XC_TOL = 0.10                  # 命中判定: shot_anchor_xc ± 此值内有 detection → 板上还在
                                   # **2026-08-03 第四次调整**: 配合 CONFIRM_XC_TOL=0.30 多帧检查
CONFIRM_XC_TOL = 0.30              # 多帧确认 HIT 时, 找同板上用的松容差
                                   # **2026-08-03 第四次调整**: 0.20 → 0.30
                                   # 实测 cam 板上 xc 在射击后可能偏移 ~0.20-0.25 (车抖动 + yaw drift)
                                   # 0.30 兼容这种偏移, 同时不会跨过 0.25 间距抓到相邻板上太多次
LOCK_XC_TOL = 0.10                 # 5 发循环锁定 first_xc 容差 (身份门控)
                                   # **2026-08-03 第二次修复**: 旧 0.20 太松,
                                   # cam 视野 [+0.47, +0.69, +0.92] 时会锁定到相邻板上 (#2)
                                   # 新 0.10 不会跨过 0.25 间距抓到相邻板上
AIM_TARGET_XC = 0.5                # 期望 cam xc (实测枪口基本在车头方向,不需 gun_xc 偏移)
YAW_SIGN = 1.0                     # +yaw 让目标 xc 增大 (实测 2026-08-01)
YAW_PER_STEP_CAP_DEG = 3.0         # 单次 yaw 上限 (用户硬约束:小步微调)
YAW_RESET_PER_STEP_DEG = 3.0       # yaw 重置单步上限 (≤ YAW_PER_STEP_CAP_DEG)
YAW_HFOV_DEG = 70.0                # cam 水平视场角 (估算)
YAW_K = 0.6                        # yaw 增益 (实测需要保守一点)
POSITION_ADJUST_M = 0.04           # 位置微调 4cm
HIT_GRACE_S = 0.3                  # 射击后等多久再检测命中 (实测电磁阀响应 < 200ms)

# 重试
DETECT_RETRIES = 3
DETECT_DELAY_S = 0.1
DETECT_LOST_RETRIES = 4
DETECT_LOST_DELAY_S = 0.2

# 推进
DRIVE_STEP_TIMEOUT_PER_M = 20
INITIAL_WAIT_TIMEOUT_S = 15.0       # 起点等 cam 视野出现目标的最大时间
NO_PROGRESS_MAX_STEPS = 6          # 视野空连续步数上限 (防死循环)
# **2026-08-03 第八次修复**: 30cm 兜底 — shot_seq=N tracker 找不到时,
# 累计 drive 16cm 步数, 超过 MAX_SEARCH_M 直接放弃 (跳过) shot_seq=N
# 防止死循环跑出场 (用户实测: 1.3.4 测试时 #4 找不到 → 跑出视野)
MAX_SEARCH_M = 0.30                # 30cm 兜底 (用户硬约束)


# ============================================================================
# helpers
# ============================================================================

def car_call(client, name, *args, timeout=10.0, **kwargs):
    """执行 car action (sync mode), 失败抛 RuntimeError。

    走 RuntimeApiClient.execute_car_action, 内层 *args 让外层 list 整体作为
    位置参数传到 runtime, runtime 内部 _dispatch_car 再 *args 解包一次。
    """
    job = client.execute_car_action(name, *args, timeout=timeout,
                                    sync=True, **kwargs)
    if job.get("status") != "succeeded":
        raise RuntimeError(f"car.{name} failed: {job.get('error')}")
    return job.get("result")


def drive_forward(client, distance_m, label=""):
    """沿车头方向直行 distance_m 米 (正=前, 负=后)。"""
    if abs(distance_m) < 0.005:
        return
    direction = "前" if distance_m > 0 else "后"
    print(f"  [drive] {direction} {abs(distance_m) * 100:.1f}cm ({label})",
          flush=True)
    try:
        car_call(client, "move_for", [distance_m, 0.0, 0.0],
                 timeout=max(5, abs(distance_m) * DRIVE_STEP_TIMEOUT_PER_M))
    except Exception as e:
        print(f"  [drive err] {e}", file=sys.stderr)


def adjust_yaw(client, deg, label=""):
    """转 yaw deg 度 (限制在 ±YAW_PER_STEP_CAP_DEG 内)。

    Returns:
        实际发出的 yaw 度 (deg) — 0 表示本次未发。
    """
    if abs(deg) < 0.1:
        return 0.0
    deg_clamped = max(-YAW_PER_STEP_CAP_DEG,
                      min(YAW_PER_STEP_CAP_DEG, deg))
    print(f"  [yaw] {deg:+.2f}° → 发 {deg_clamped:+.2f}° ({label})",
          flush=True)
    try:
        car_call(client, "move_for", [0.0, 0.0, math.radians(deg_clamped)],
                 timeout=10)
        return deg_clamped
    except Exception as e:
        print(f"  [yaw err] {e}", file=sys.stderr)
        return 0.0


def reset_yaw(client, yaw_used):
    """**2026-08-03 新增**: 反向 yaw_used 度,分步 ≤ YAW_RESET_PER_STEP_DEG。

    目的: 5 发循环累积 yaw drift (实测可达 ±10°) 会让车前进时偏航。
    在每只板上射击完后,反向所有累积 yaw, 让车回到原始朝向, 再前进 8cm。

    Args:
        yaw_used: shoot_one_board 累积的 yaw (度)
    """
    if abs(yaw_used) < 0.5:
        return
    remaining = -yaw_used   # 反向
    n_steps = max(1, int(abs(remaining) / YAW_RESET_PER_STEP_DEG) + 1)
    step = remaining / n_steps
    print(f"  [yaw-reset] 反向 {yaw_used:+.2f}° → 分 {n_steps} 步, "
          f"每步 {step:+.2f}°", flush=True)
    for i in range(n_steps):
        try:
            car_call(client, "move_for", [0.0, 0.0, math.radians(step)],
                     timeout=10)
            time.sleep(0.1)
        except Exception as e:
            print(f"  [yaw-reset err step {i+1}/{n_steps}] {e}",
                  file=sys.stderr)
            return


def find_target_by_xc(animals, anchor_xc, xc_tol=LOCK_XC_TOL):
    """在 anchor_xc ± xc_tol 内找最近的 detection (身份门控)。

    Returns:
        (target, cur_xc) 或 (None, None) 如果找不到。
    """
    if not animals:
        return None, None
    nearby = [a for a in animals if abs(bbox_xc(a) - anchor_xc) < xc_tol]
    if not nearby:
        return None, None
    target = min(nearby, key=lambda a: abs(bbox_xc(a) - anchor_xc))
    return target, bbox_xc(target)


def shoot_one_attempt(client, first_xc, shot_i):
    """单发: re-detect + 调 yaw + shoot + 命中判定。

    **2026-08-03 改进**:
    - 命中判定改用「板上数量减少」+ 「tight xc 容差」双判
    - LOCK 失败 (找不到 first_xc ± 容差内的 detection) → 视为板上已消失 → HIT

    Args:
        first_xc: 板上目标冻结的 cam xc (用于身份门控,锁定 5 发循环的"板上是谁")
        shot_i: 第几发 (1-based)

    Returns:
        (hit: bool, yaw_used_deg: float, lost_identity: bool)
    """
    # re-detect with retry (给 yolo 多帧机会避免漏检)
    animals_before = get_animals_retry(client, MIN_YOLO_SCORE,
                                       label=f"shot{shot_i} pre-detect",
                                       retries=DETECT_RETRIES,
                                       delay=DETECT_DELAY_S)
    if not animals_before:
        print(f"    [s{shot_i}] cam 视野空, 放弃这只", flush=True)
        return False, 0.0, True

    n_before = len(animals_before)
    sorted_animals = sorted(animals_before, key=bbox_xc)
    all_xcs = sorted([bbox_xc(a) for a in sorted_animals])
    print(f"    [s{shot_i}] cam 视野 xc: "
          f"{[f'{x:+.2f}' for x in all_xcs]} "
          f"(anchor=first_xc={first_xc:+.2f}, n={n_before})",
          flush=True)

    # 身份门控: 在 first_xc ± LOCK_XC_TOL 内找目标
    target, cur_xc = find_target_by_xc(sorted_animals, first_xc)
    if target is None:
        # 多帧重试 (给 yolo 机会重检漏检板)
        retried_ok = False
        for retry_i in range(DETECT_LOST_RETRIES):
            time.sleep(DETECT_LOST_DELAY_S)
            animals_retry = get_animals(client, MIN_YOLO_SCORE)
            target, cur_xc = find_target_by_xc(animals_retry, first_xc)
            if target is not None:
                sorted_animals = sorted(animals_retry, key=bbox_xc)
                all_xcs = sorted([bbox_xc(a) for a in sorted_animals])
                n_before = len(animals_retry)
                print(f"    [s{shot_i}] 重试 {retry_i+1}/{DETECT_LOST_RETRIES} "
                      f"成功, 找到目标 xc={cur_xc:+.3f}", flush=True)
                retried_ok = True
                break
        if not retried_ok:
            # **2026-08-03 第四次修复**: LOCK 失败 = 找不到 first_xc ± LOCK_XC_TOL
            # 内的 detection。两种可能:
            #   a) 板上移出 LOCK 容差 (车抖动 / yaw drift 引起) → 板上还在场上, 只是变了位置
            #   b) 板上已被击倒 → 不该再射
            # **2026-08-03 第四次**: 不再用「LOCK 失败 = HIT」逻辑 (假阳性太多)
            # 改用: 在 CONFIRM_XC_TOL=0.30 内找板上, 找到就拍 (可能拍到相邻板上, 由多帧判定)
            # 找不到 → lost_id (彻底放弃本发)
            target_wide, cur_xc_wide = find_target_by_xc(
                sorted_animals, first_xc, xc_tol=CONFIRM_XC_TOL)
            if target_wide is not None:
                target = target_wide
                cur_xc = cur_xc_wide
                print(f"    [s{shot_i}] LOCK 失败 (±{LOCK_XC_TOL}) 但松搜索 "
                      f"(±{CONFIRM_XC_TOL}) 找到 xc={cur_xc:+.3f}, 拍它",
                      flush=True)
            elif n_before == 0:
                print(f"    [s{shot_i}] ✗ cam 视野完全空, 放弃这只", flush=True)
                return False, 0.0, True
            else:
                print(f"    [s{shot_i}] ✗ LOCK 失败 (±{LOCK_XC_TOL}) 且松搜索 "
                      f"(±{CONFIRM_XC_TOL}) 也无 detection, 放弃本发",
                      flush=True)
                return False, 0.0, True   # lost_id (不再当 HIT)

    shot_anchor_xc = cur_xc            # 冻结本发 anchor
    shot_anchor_score = target.get("score", 0.0)
    shot_anchor_yc = bbox_yc(target)
    err = cur_xc - AIM_TARGET_XC

    # yaw 微调 (基于 err 算期望 yaw, K 增益保守)
    yaw_needed = -err * YAW_HFOV_DEG * YAW_K * YAW_SIGN
    yaw_used = adjust_yaw(client, yaw_needed, label=f"s{shot_i}")

    # shoot
    print(f"    [s{shot_i}] cur_xc={cur_xc:+.3f} err={err:+.3f} "
          f"yaw_used={yaw_used:+.2f}° → 射击 (score={shot_anchor_score:.3f},"
          f" yc={shot_anchor_yc:.3f})", flush=True)
    try:
        car_call(client, "shooting", timeout=5)
    except Exception as e:
        print(f"    [s{shot_i} shoot err] {e}", file=sys.stderr)
    time.sleep(HIT_GRACE_S)

    # **2026-08-03 第三次修复**: 多帧确认 HIT (保守策略)
    # 实测射击后车抖动 / YOLO 噪声会让 cam 板上数临时 -1 (单帧 count decrease
    # 不可靠)。多帧确认 + score drop + yc drop 综合判断, 避免假阳性。
    print(f"    [s{shot_i}] 射后多帧确认 (shot_anchor={shot_anchor_xc:+.3f},"
          f" score={shot_anchor_score:.3f}, yc={shot_anchor_yc:.3f})...",
          flush=True)
    hit_confirmed, reason = confirm_hit_multi_frame(
        client, shot_anchor_xc, shot_anchor_score, shot_anchor_yc)
    if hit_confirmed:
        print(f"    ✓ 命中! ({reason})", flush=True)
        return True, yaw_used, False

    print(f"    [s{shot_i} miss] ({reason})", flush=True)
    return False, yaw_used, False


def confirm_hit_multi_frame(client, shot_anchor_xc, shot_anchor_score,
                             shot_anchor_yc):
    """**2026-08-03 第三次修复**: 多帧确认 HIT (保守策略)。

    单帧「count decrease」不可靠 — 实测射击后车抖动 / YOLO 噪声会让 cam 板上数
    临时 -1, 看起来像 HIT 但实际未中。

    保守 HIT 判定 (3 帧, 间隔 ~0.15s):
    - 3/3 帧空 → HIT (板上完全消失, 不可能是噪声)
    - 2/3 帧空 → HIT (板上大概率被打掉, YOLO 偶尔假阳)
    - 1/3 帧空 → 看 score drop / yc drop 平均
        - score drop > 0.20 → HIT
        - yc drop > 0.05 → HIT (板上倒下后 yc 下降)
    - 0/3 帧空 → MISS (板上还在)

    Args:
        shot_anchor_xc: 射前板上冻结的 cam xc
        shot_anchor_score: 射前板上 YOLO score
        shot_anchor_yc: 射前板上 cam yc

    Returns:
        (hit: bool, reason: str)
    """
    N_FRAMES = 3
    FRAME_INTERVAL_S = 0.15

    frames = []
    for fi in range(N_FRAMES):
        animals = get_animals_retry(client, MIN_YOLO_SCORE,
                                    label=f"confirm-hit f{fi+1}",
                                    retries=1, delay=0.05)
        frames.append(animals)
        time.sleep(FRAME_INTERVAL_S)

    # 打印每帧状态
    for fi, animals in enumerate(frames):
        xcs = [f"{bbox_xc(a):+.2f}" for a in animals] if animals else ["空"]
        print(f"      f{fi+1}: cam=[{', '.join(xcs)}]", flush=True)

    # 判 1: cam 视野完全空
    all_empty = all(not f for f in frames)
    if all_empty:
        return True, "cam 视野完全空 (3/3 帧)"

    # 判 2-4: shot_anchor 附近的 detection 状态
    detect_count = 0
    score_drops = []
    yc_drops = []
    for animals in frames:
        # **2026-08-03 第四次修复**: 用 CLOSEST detection (不是 nearby[0]),
        # 避免当多个 detection 在容差内时选错板上
        candidates = [a for a in animals
                      if abs(bbox_xc(a) - shot_anchor_xc) < CONFIRM_XC_TOL]
        if candidates:
            closest = min(candidates,
                          key=lambda a: abs(bbox_xc(a) - shot_anchor_xc))
            detect_count += 1
            score = closest.get("score", 0.0)
            yc = bbox_yc(closest)
            score_drops.append(shot_anchor_score - score)
            yc_drops.append(shot_anchor_yc - yc)
        else:
            score_drops.append(shot_anchor_score)  # 全 drop (板不在)
            yc_drops.append(shot_anchor_yc)

    miss_count = N_FRAMES - detect_count
    avg_score_drop = sum(score_drops) / N_FRAMES
    avg_yc_drop = sum(yc_drops) / N_FRAMES

    # **2026-08-03 第四次修复**: HIT 阈值更保守
    # 判 2: 3/3 帧完全无 detection → HIT (板上真消失)
    # 旧: 2/3 帧 miss → HIT (假阳性太多, 车抖动会让单帧漏)
    if miss_count >= 3:
        return True, (f"板上 {miss_count}/{N_FRAMES} 帧完全消失 "
                      f"(avg score drop {avg_score_drop:.2f})")

    # 判 3: score 显著下降 → HIT (旧阈值 0.20 太松, 提高到 0.40)
    if avg_score_drop > 0.40:
        return True, f"score 平均下降 {avg_score_drop:.2f} (板上置信度大幅下降)"

    # 判 4: yc 显著下降 → HIT (旧阈值 0.05 太松, 提高到 0.10)
    if avg_yc_drop > 0.10:
        return True, f"yc 平均下降 {avg_yc_drop:.2f} (板上倒下 yc 变小)"

    return False, (f"板上 {detect_count}/{N_FRAMES} 帧仍在 "
                   f"(avg score drop {avg_score_drop:.2f}, "
                   f"yc drop {avg_yc_drop:.2f})")


def shoot_one_board(client, first_xc, board_num):
    """单板上 5 发循环 (最多)。

    流程: 第 1 发先 yaw 微调, miss → 第 2 发前/后 4cm, miss → 第 3 发 yaw,
    miss → 第 4 发 反方向前/后 4cm, miss → 第 5 发 yaw。

    **2026-08-03 改进**: 累积 yaw_used_total, 板上射击完后调用 reset_yaw
    反向累积 yaw, 防止前进时偏航。

    Returns:
        (hit: bool, attempts: int)
    """
    print(f"\n  [shoot #{board_num}] first_xc={first_xc:+.3f}, "
          f"最多 {MAX_SHOTS_PER_BOARD} 发", flush=True)

    hit = False
    attempts = 0
    yaw_used_total = 0.0  # **2026-08-03**: 累积 yaw, 板上结束后反向

    for shot_i in range(1, MAX_SHOTS_PER_BOARD + 1):
        attempts = shot_i
        this_hit, yaw_used, lost_id = shoot_one_attempt(
            client, first_xc, shot_i)
        yaw_used_total += yaw_used   # **2026-08-03**: 累积
        if lost_id:
            break
        if this_hit:
            hit = True
            print(f"  ✓ 板上 #{board_num} 命中 ({attempts} 发)", flush=True)
            break

        # miss → 调整位置 (前后 4cm) 准备下一发
        if shot_i < MAX_SHOTS_PER_BOARD:
            # 交替: 奇数次向前 4cm, 偶数次向后 4cm
            if shot_i % 2 == 1:
                drive_forward(client, +POSITION_ADJUST_M,
                              label=f"miss→fwd 准备 s{shot_i+1}")
            else:
                drive_forward(client, -POSITION_ADJUST_M,
                              label=f"miss→back 准备 s{shot_i+1}")

    if not hit:
        print(f"  ✗ 板上 #{board_num} 放弃 ({attempts} 发用完仍未命中)",
              flush=True)

    # **2026-08-03 新增**: 反向累积 yaw, 让车回到原始朝向
    # 下次前进 8cm 不会偏航
    reset_yaw(client, yaw_used_total)
    return hit, attempts


# ============================================================================
# 主流程
# ============================================================================

class BoardTracker:
    """**2026-08-03 第五次修复**: 板上身份追踪。

    问题: 旧算法用 `in_view_idx = shot_seq - len(hit_set)` 假设 cam 视野 L→R
    严格对应板上 #k 顺序。但 cam 视野板数 < N_TOTAL_BOARDS 时, 这个假设失效,
    导致脚本把 #2 误认成 #4 (cam 视野只 1 只时, best-effort 取最左)。

    解决: 用 proximity matching 跟踪每只板上的 xc 漂移, 按物理身份 board_num 锁定。

    工作流:
    1. initialize(sorted_animals) — 起点按 L→R 给 cam 视野的 detection 赋 board_num
       (1, 2, 3, ...). 起点未见的板上 (e.g. #4) 留为 None, 后续第一次检测到时绑定。
    2. update(animals) — 每次 cam 视野更新, 按 proximity 匹配 detection 到已知板上。
       - 对每个 standing 板上, 找最近 unmatched detection (距离 < MATCH_TOL)
       - 匹配上: 更新 last_xc/yc/score
       - 没匹配上: frames_missing++, 连续 3 帧缺失 → 自动 mark hit
    3. get_xc(board_num) — 查 board_num 的当前 xc, 不用 cam_view index 推测
    4. mark_hit(board_num) / is_hit(board_num) — 手动标记 / 查询
    """

    MATCH_TOL = 0.45     # **2026-08-03 第七次调整**: 0.30 → 0.45
                            # 实测 16cm 直行 + yaw drift 后板上 xc 漂移可达 ~0.40
                            # (板上 #4 last_xc=+0.94, 实际已到 +0.54, 距离 0.40)
                            # 0.45 容差兼容这种漂移, 由 Hungarian (而非 greedy) 缓解误匹配
    MAX_MISSING = 3      # 连续 N 帧缺失 → 自动 mark hit

    def __init__(self, n_total=N_TOTAL_BOARDS):
        self.n_total = n_total
        # board_num (1-based) -> info dict or None (not yet seen)
        self.boards = {i: None for i in range(1, n_total + 1)}

    def initialize(self, sorted_animals):
        """按 L→R 给起点 cam 视野 detection 赋 board_num (1, 2, 3, ...)。

        Args:
            sorted_animals: 起点 cam 视野 sorted by xc L→R
        """
        for i, a in enumerate(sorted_animals[:self.n_total]):
            board_num = i + 1
            self.boards[board_num] = {
                'last_xc': bbox_xc(a),
                'last_yc': bbox_yc(a),
                'last_score': a.get("score", 0.0),
                'frames_missing': 0,
                'status': 'standing',
                'first_seen_xc': bbox_xc(a),   # 起点 xc, 用于调试
            }

    def update(self, animals):
        """**2026-08-03 第六次修复**: 用 greedy global matching 更新跟踪。

        旧版按 board_num 顺序匹配, 容易被 stale last_xc 卡住 (e.g. #3 在 +0.938
        但实际在 +0.78, 匹配失败)。

        新版:
        1. 生成所有 (board, detection, distance) 三元组, 按 distance 排序
        2. greedy: 最小 distance 先匹配, 双方都不能再用
        3. 未匹配的 standing 板上: frames_missing++, 连续 3 帧 → auto hit
        4. 未匹配的 detection: 绑定给第一个未赋值的 board_num (起点未见的板)

        Returns:
            matches: dict {board_num -> matched detection 或 None}
        """
        from itertools import permutations
        matches = {}

        standing_boards = [(bn, info) for bn, info in self.boards.items()
                          if info is not None and info['status'] == 'standing']

        if not standing_boards or not animals:
            for bn, info in standing_boards:
                info['frames_missing'] += 1
                if info['frames_missing'] >= self.MAX_MISSING:
                    info['status'] = 'hit'
                matches[bn] = None
            return matches

        # **2026-08-03 第七次修复**: Hungarian (brute force for small N) 而非 greedy
        # Greedy 会被 stale last_xc 卡住 (e.g. #2 和 #3 都要 +0.84, greedy 给 #2,
        # #3 unmatched)。Hungarian 找全局最优分配 (总距离最小), 缓解误匹配。
        n_b = len(standing_boards)
        n_a = len(animals)

        best_assignment = {}
        best_cost = float('inf')

        if n_b <= n_a:
            # 每个 board 分一个 animal (n_a 选 n_b 个排列)
            for combo in permutations(range(n_a), n_b):
                cost = 0
                valid = True
                assignment = {}
                for i, (bn, info) in enumerate(standing_boards):
                    a = animals[combo[i]]
                    d = abs(bbox_xc(a) - info['last_xc'])
                    if d > self.MATCH_TOL:
                        valid = False
                        break
                    cost += d
                    assignment[bn] = a
                if valid and cost < best_cost:
                    best_cost = cost
                    best_assignment = assignment
        else:
            # n_b > n_a: 每个 animal 分一个 board (n_b 选 n_a 个排列)
            for combo in permutations(range(n_b), n_a):
                cost = 0
                valid = True
                assignment = {}
                for i in range(n_a):
                    bn, info = standing_boards[combo[i]]
                    a = animals[i]
                    d = abs(bbox_xc(a) - info['last_xc'])
                    if d > self.MATCH_TOL:
                        valid = False
                        break
                    cost += d
                    assignment[bn] = a
                if valid and cost < best_cost:
                    best_cost = cost
                    best_assignment = assignment

        # 应用最优匹配
        matched_boards = set()
        matched_animals = set()
        for bn, a in best_assignment.items():
            info = self.boards[bn]
            info['last_xc'] = bbox_xc(a)
            info['last_yc'] = bbox_yc(a)
            info['last_score'] = a.get("score", 0.0)
            info['frames_missing'] = 0
            matched_boards.add(bn)
            matched_animals.add(id(a))
            matches[bn] = a

        # 未匹配的 standing 板上: frames_missing++
        for bn, info in standing_boards:
            if bn not in matched_boards:
                info['frames_missing'] += 1
                if info['frames_missing'] >= self.MAX_MISSING:
                    info['status'] = 'hit'
                matches[bn] = None

        # **首次出现绑定**: 未赋值的 board (e.g. #4) → 绑给剩余未匹配 detection
        # 假设 cam 视野 L→R 顺序 = 板上 #k 顺序 (initialize 时的假设一致)
        unmatched_animals = [a for a in animals if id(a) not in matched_animals]
        unmatched_animals_sorted = sorted(unmatched_animals, key=bbox_xc)
        unassigned = sorted([bn for bn, info in self.boards.items()
                             if info is None])
        for i, board_num in enumerate(unassigned):
            if i >= len(unmatched_animals_sorted):
                break
            a = unmatched_animals_sorted[i]
            self.boards[board_num] = {
                'last_xc': bbox_xc(a),
                'last_yc': bbox_yc(a),
                'last_score': a.get("score", 0.0),
                'frames_missing': 0,
                'status': 'standing',
                'first_seen_xc': bbox_xc(a),
            }
            matches[board_num] = a

        return matches

    def get_xc(self, board_num):
        """查 board_num 的当前 xc。返回 None 表示不存在 / 已倒 / 未跟踪。"""
        if board_num not in self.boards:
            return None
        info = self.boards[board_num]
        if info is None or info['status'] != 'standing':
            return None
        return info['last_xc']

    def is_hit(self, board_num):
        return (board_num in self.boards
                and self.boards[board_num] is not None
                and self.boards[board_num]['status'] == 'hit')

    def mark_hit(self, board_num):
        """手动标记 board_num 为 hit。"""
        if board_num in self.boards and self.boards[board_num] is not None:
            self.boards[board_num]['status'] = 'hit'

    def first_seen_xc(self, board_num):
        """起点 xc (调试用)。"""
        if board_num in self.boards and self.boards[board_num] is not None:
            return self.boards[board_num].get('first_seen_xc')
        return None


def parse_targets_arg(s):
    """解析 --targets 字符串, 返回 1-based 板上号集合。

    特殊值:
        "all"  → 全部 1..N_TOTAL_BOARDS
        None   → None (调用方决定交互询问)
    """
    if s is None:
        return None
    s = s.strip().lower()
    if s == "all":
        return set(range(1, N_TOTAL_BOARDS + 1))
    nums = set()
    for tok in s.split():
        try:
            n = int(tok)
        except ValueError:
            raise ValueError(f"--targets 解析失败: '{tok}' 不是数字")
        if n < 1 or n > N_TOTAL_BOARDS:
            raise ValueError(f"--targets 编号 #{n} 超出范围 "
                             f"(1..{N_TOTAL_BOARDS})")
        nums.add(n)
    if not nums:
        raise ValueError("--targets 没指定任何编号")
    return nums


def ask_targets_interactive(n_total):
    """交互式询问用户要射哪些板上号。"""
    print(f"\n[select] 1..{n_total} 选要射的目标 "
          f"(空格分隔 1-based, 如 '1 3 4', 直接回车 = 全部):",
          flush=True)
    while True:
        try:
            user_input = input("  > ").strip()
        except EOFError:
            user_input = ""
        if not user_input:
            return set(range(1, n_total + 1))
        try:
            nums = [int(x) for x in user_input.split()]
            if any(n < 1 or n > n_total for n in nums):
                print(f"  编号超出范围 (1-{n_total}), 重输", flush=True)
                continue
            return set(nums)
        except ValueError:
            print(f"  输入解析失败, 重输", flush=True)


def wait_for_initial_detect(client, min_score):
    """等起点 cam 视野出现 ≥1 只目标, 返回 sorted animals (L→R)。"""
    print(f"\n[ready] 等待 cam 视野出现 ≥1 只目标 (最多 "
          f"{INITIAL_WAIT_TIMEOUT_S:.0f}s)...", flush=True)
    t0 = time.time()
    animals = []
    while time.time() - t0 < INITIAL_WAIT_TIMEOUT_S:
        animals = get_animals(client, min_score)
        if animals:
            break
        time.sleep(0.3)
    if not animals:
        return None
    return sorted(animals, key=bbox_xc)


def main():
    ap = argparse.ArgumentParser(
        description="智能车射击任务: 沿 4 板直行, 射击指定目标",
    )
    ap.add_argument("--targets", type=str, default=None,
                    help="要射击的目标编号 (空格分隔 1-based), 如 '1 3 4'。"
                         "不传 = 询问;'all' = 全部")
    ap.add_argument("--min-score", type=float, default=MIN_YOLO_SCORE,
                    help=f"YOLO 置信度阈值 (默认 {MIN_YOLO_SCORE})")
    args = ap.parse_args()

    # ---- 1. 启动 + 等待起点 cam 视野 ----
    client = RuntimeApiClient()
    client.wait_until_ready()

    sorted_animals = wait_for_initial_detect(client, args.min_score)
    if sorted_animals is None:
        print(f"[err] cam 视野 {INITIAL_WAIT_TIMEOUT_S:.0f}s 都没目标, 退出",
              file=sys.stderr)
        return 1

    print(f"[detect] 起点 cam 视野 {len(sorted_animals)} 只 (L→R):",
          flush=True)
    for i, a in enumerate(sorted_animals):
        xc = bbox_xc(a)
        yc = bbox_yc(a)
        wn = bbox_width(a)
        score = a.get("score", 0.0)
        print(f"  cam_view[{i+1}]: xc={xc:+.3f} yc={yc:.3f} "
              f"wn={wn:.3f} score={score:.3f}", flush=True)
    n_visible = len(sorted_animals)
    if n_visible > N_TOTAL_BOARDS:
        print(f"[warn] cam 视野 {n_visible} 只 > 板上总数 {N_TOTAL_BOARDS}, "
              f"取前 {N_TOTAL_BOARDS} 只", flush=True)

    # ---- 2. 询问 / 解析 --targets ----
    if args.targets is None:
        targets_to_shoot = ask_targets_interactive(N_TOTAL_BOARDS)
        print(f"  → 选 #{sorted(targets_to_shoot)}", flush=True)
    else:
        try:
            targets_to_shoot = parse_targets_arg(args.targets)
        except ValueError as e:
            print(f"[err] --targets 解析失败: {e}", file=sys.stderr)
            return 1
        print(f"[select] 选 #{sorted(targets_to_shoot)} "
              f"(--targets '{args.targets}')", flush=True)

    # ---- 3. 主循环 ----
    # **2026-08-03 第五次修复**: 用 BoardTracker 跟踪板上身份
    # 旧算法用 `in_view_idx = shot_seq - len(hit_set)` 假设 cam 视野 L→R 严格
    # 对应板上 #k 顺序, 但 cam 视野板数 < N_TOTAL 时失效 (把 #2 误认成 #4)。
    # 改为: 起点按 L→R 给 detection 赋 board_num, 后续按 proximity 匹配。
    results = []
    hit_set = set()
    shot_seq = 1
    no_progress_steps = 0  # 视野空连续步数 (防死循环)
    # **2026-08-03 第八次修复**: shot_seq 找不到时的累计 drive 距离
    # 当累计 drive > MAX_SEARCH_M (30cm) → 跳过 shot_seq, 进入下一个
    search_dist_m = 0.0

    # 初始化 BoardTracker (按 L→R 顺序给起点 detection 赋 board_num)
    tracker = BoardTracker(n_total=N_TOTAL_BOARDS)
    tracker.initialize(sorted_animals)
    print(f"\n[tracker] 起点板上身份初始化:")
    for bn in range(1, N_TOTAL_BOARDS + 1):
        xc = tracker.first_seen_xc(bn)
        if xc is not None:
            print(f"  板上 #{bn}: 起点 xc={xc:+.3f}", flush=True)
        else:
            print(f"  板上 #{bn}: 起点未检测到, 等待后续匹配", flush=True)

    while shot_seq <= N_TOTAL_BOARDS:
        # re-detect
        animals = get_animals_retry(client, args.min_score,
                                    label=f"#{shot_seq} detect",
                                    retries=DETECT_RETRIES, delay=DETECT_DELAY_S)
        if not animals:
            print(f"\n========== 板上 #{shot_seq} ==========", flush=True)
            print(f"  [warn] cam 视野空, 直行 16cm 找板上 #{shot_seq}",
                  flush=True)
            drive_forward(client, BOARD_SPACING_M, label=f"视野空")
            no_progress_steps += 1
            if no_progress_steps > NO_PROGRESS_MAX_STEPS:
                print(f"[err] 连续 {no_progress_steps} 步视野空, 退出",
                      file=sys.stderr)
                break
            continue

        sorted_animals = sorted(animals, key=bbox_xc)
        all_xcs = [bbox_xc(a) for a in sorted_animals]
        no_progress_steps = 0

        # **2026-08-03 第六次修复**: 更新 BoardTracker 并获取 matches
        # matches: dict {board_num -> matched detection 或 None}
        matches = tracker.update(animals)

        # 检查 shot_seq 是否被自动 mark hit (3 帧没匹配)
        if tracker.is_hit(shot_seq):
            hit_set.add(shot_seq)
            print(f"\n========== 板上 #{shot_seq} ==========", flush=True)
            print(f"  [auto-hit] tracker 检测到板上 #{shot_seq} 连续缺失, "
                  f"auto mark hit → hit_set={sorted(hit_set)}",
                  flush=True)
            shot_seq += 1
            if shot_seq > N_TOTAL_BOARDS:
                print(f"\n[done] 全部 {N_TOTAL_BOARDS} 板上处理完, "
                      f"停止移动", flush=True)
                break
            drive_forward(client, BOARD_SPACING_M,
                          label=f"auto-hit #{shot_seq-1} → #{shot_seq}")
            continue

        # **2026-08-03 第六次修复**: 用 matches[shot_seq] 查板上**当前**位置
        # 不用 stale tracker.get_xc() — 用 greedy matching 后的实际位置
        target_detection = matches.get(shot_seq)
        if target_detection is not None:
            cur_xc = bbox_xc(target_detection)   # 本帧匹配到的位置
            cur_yc = bbox_yc(target_detection)
            cur_score = target_detection.get("score", 0.0)
            print(f"\n========== 板上 #{shot_seq} ==========", flush=True)
            print(f"  cam 视野 {len(sorted_animals)} 只: "
                  f"{[f'{x:+.2f}' for x in all_xcs]} "
                  f"(板上 #{shot_seq} matched at xc={cur_xc:+.3f}, "
                  f"hit_set={sorted(hit_set)})",
                  flush=True)
        else:
            # 板上 #shot_seq 本帧没匹配上, 但 tracker 还在追踪 (last_xc 不为 None)
            cur_xc = tracker.get_xc(shot_seq)
            if cur_xc is None:
                # tracker 也没追踪到 (从未绑定, e.g. #4 起点没见且后续没匹配)
                print(f"\n========== 板上 #{shot_seq} ==========", flush=True)
                print(f"  [warn] 板上 #{shot_seq} tracker 还没绑定 "
                      f"(起点未检到 + 后续没匹配), cam 视野: "
                      f"{[f'{x:+.2f}' for x in all_xcs]}",
                      flush=True)
                # **2026-08-03 第八次修复**: 30cm 兜底
                if search_dist_m >= MAX_SEARCH_M:
                    print(f"  [abandon] 累计 drive {search_dist_m*100:.0f}cm "
                          f">= {MAX_SEARCH_M*100:.0f}cm, 板上 #{shot_seq} "
                          f"找不到, 放弃本只, 进入 #{shot_seq+1}",
                          flush=True)
                    results.append({"board": shot_seq, "hit": False,
                                    "attempts": 0, "skipped": False,
                                    "abandoned": True, "first_xc": None})
                    shot_seq += 1
                    search_dist_m = 0.0
                    if shot_seq > N_TOTAL_BOARDS:
                        print(f"\n[done] 全部 {N_TOTAL_BOARDS} 板上处理完, "
                              f"停止移动", flush=True)
                        break
                    drive_forward(client, BOARD_SPACING_M,
                                  label=f"abandon #{shot_seq-1} → #{shot_seq}")
                    continue
                drive_forward(client, BOARD_SPACING_M,
                              label=f"等板上 #{shot_seq} 进视野")
                search_dist_m += BOARD_SPACING_M
                continue
            # tracker 找到了 → 重置搜索距离
            search_dist_m = 0.0
            # tracker 有 last_xc (上帧位置), 但本帧没匹配
            # 用 last_xc 拍 (cam 可能丢了一帧)
            print(f"\n========== 板上 #{shot_seq} ==========", flush=True)
            print(f"  cam 视野 {len(sorted_animals)} 只: "
                  f"{[f'{x:+.2f}' for x in all_xcs]} "
                  f"(板上 #{shot_seq} last_xc={cur_xc:+.3f}, "
                  f"本帧未匹配, hit_set={sorted(hit_set)})",
                  flush=True)

        if shot_seq not in targets_to_shoot:
            # 跳过 (板上 #shot_seq 仍在场上合法, hit_set 不增加)
            print(f"  [skip] 板上 #{shot_seq} xc={cur_xc:+.3f}", flush=True)
            results.append({"board": shot_seq, "hit": False, "attempts": 0,
                            "skipped": True, "first_xc": cur_xc})
            shot_seq += 1
            if shot_seq > N_TOTAL_BOARDS:
                print(f"\n[done] 全部 {N_TOTAL_BOARDS} 板上处理完, "
                      f"停止移动", flush=True)
                break
            drive_forward(client, BOARD_SPACING_M,
                          label=f"skip #{shot_seq-1} → #{shot_seq}")
            continue

        # 射击 (5 发循环)
        hit, attempts = shoot_one_board(client, cur_xc, shot_seq)
        results.append({"board": shot_seq, "hit": hit, "attempts": attempts,
                        "skipped": False, "first_xc": cur_xc})
        if hit:
            hit_set.add(shot_seq)
            tracker.mark_hit(shot_seq)
            print(f"  [hit] 板上 #{shot_seq} → hit_set={sorted(hit_set)}",
                  flush=True)
        else:
            # 放弃: 板上 #shot_seq 仍在场上合法, hit_set 不增加
            print(f"  [abandon] 板上 #{shot_seq} 未命中, "
                  f"仍在场上合法", flush=True)

        shot_seq += 1

        if shot_seq > N_TOTAL_BOARDS:
            print(f"\n[done] 全部 {N_TOTAL_BOARDS} 板上处理完, "
                  f"停止移动", flush=True)
            break

        # 击倒/放弃后直行 16cm 到下一只板射击点
        drive_forward(client, BOARD_SPACING_M,
                      label=f"#{shot_seq-1} → #{shot_seq}")

    # ---- 4. 汇总 ----
    print(f"\n{'='*60}", flush=True)
    print(f"========== 射击任务汇总 ==========", flush=True)
    print(f"{'='*60}", flush=True)
    hit_count = sum(1 for r in results if r["hit"])
    skip_count = sum(1 for r in results if r.get("skipped"))
    fail_count = sum(1 for r in results
                     if not r["hit"] and not r.get("skipped"))
    print(f"  命中 {hit_count} / 目标 {len(targets_to_shoot)} "
          f"(跳过 {skip_count} / 失败 {fail_count} / "
          f"板上 {N_TOTAL_BOARDS})", flush=True)
    for r in results:
        if r.get("skipped"):
            print(f"  #{r['board']}  [skip] xc={r.get('first_xc', 0):+.3f}",
                  flush=True)
        elif r["hit"]:
            print(f"  #{r['board']}  ✓ 命中 ({r['attempts']} 发) "
                  f"xc={r.get('first_xc', 0):+.3f}", flush=True)
        else:
            print(f"  #{r['board']}  ✗ 放弃 ({r['attempts']} 发) "
                  f"xc={r.get('first_xc', 0):+.3f}", flush=True)
    print(f"{'='*60}", flush=True)

    return 0 if hit_count == len(targets_to_shoot) else 1


if __name__ == "__main__":
    sys.exit(main())
