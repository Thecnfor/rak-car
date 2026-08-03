#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/shoot_4_targets.py — 直走 + 瞄准 + 射击 + 归位

完整流程(2026-08-02 用户硬约束:完全禁用 yaw/lateral,只允许前后移动):
1. 检测视野内 4 个目标,按 xc 左→右排好,冻结顺序
2. 对每只目标:
   2.1 re-detect + 射击(不再 yaw 微调)
   2.2 miss → 前后微调 5cm(根据 err 符号 forward/back)→ 再射,最多 5 发
   2.3 命中 OR 放弃都归位(回 odom 起点)
3. 直至 4 只都打完

约束(2026-08-02 用户硬约束):
- ❌ yaw(`move_for([0, 0, ±yaw])`)
- ❌ lateral(`move_for([0, ±lateral, 0])`)
- ✅ forward/back(`move_for([±d, 0, 0])`)
- 完成第一只目标射击后,通过 forward 8cm(或多个 8cm)推进到下一只目标
- 每只目标最多 5 发
- 严格 L→R 顺序
- 目标 8×8cm 板子,板间距 8cm → 板中心距 16cm
- 枪在 cam 右边 25cm
- 身份门控 HIT_XC_TOL = 0.40

用法:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.shoot_4_targets
    python -m main.tasks.task333.shoot_4_targets --shoot-distance 0.5 --target-distance 1.05
"""
from __future__ import annotations

import argparse
import math
import sys
import time

from main.api_client import RuntimeApiClient
from main.tasks.task333.go_straight_2m import (
    car_call,
    read_odom,
)


# ============================================================================
# 常量(集中,只在一处定义)
# ============================================================================

# chassis 坐标系(实测 2026-07-31 + 2026-08-01 翻号验证)
# **实测最终版(2026-08-01 第二轮日志)**:
# #3 phase 2 miss→big yaw:err=-0.096 → 发 -1.88° → cur_xc:+0.404 → +0.351(更左)
# #4 phase 2 miss→big yaw:err=+0.300 → 发 +6.01° → cur_xc:+0.822(更右)
# → **+yaw 让 cam 视野中目标 xc 增大**(目标视觉右移,车头右转)
# → **-yaw 让目标 xc 减小**(目标视觉左移)
# → 想要 cur_xc 增往 cam 中心 → 发 +yaw → yaw_deg = -dx × gain
# 在公式 yaw_deg = -dx × gain × YAW_SIGN 下,YAW_SIGN=+1 才能让
# dx<0(目标在左)→ yaw=-dx>0 → 发正 yaw → cur_xc 增 → 收敛。
YAW_SIGN = 1.0
FORWARD_AXIS_LOCAL = (1.0, 0.0, 0.0)   # 车体局部 +x = 前进(odom +x)

# 几何
GUN_OFFSET_CM = 25.0               # 摄像头右边 25cm(用户硬约束)
CFOV_HORIZONTAL_DEG = 70.0         # cam2 水平视场角估算
TARGET_BOARD_WIDTH_M = 0.08        # 8cm(用户硬约束)
TARGET_BOARD_GAP_M = 0.08          # 8cm(用户硬约束)
TARGET_CENTER_SPACING_M = 0.16     # 板中心距 16cm
# **2026-08-01 新增**(用户原话"目标与目标之间是固定的8cm"):
BOARD_SPACING_M = 0.08            # 板间距 8cm(直行这个距离到下一只板)
BANNED_XC_TOL = 0.15              # BFS 排除已击倒板子的 cam xc 容差
# **2026-08-01 BFS 改造**:身份门控容差(initial_xc ± HIT_XC_TOL 内
# 找不到 detection → 身份丢失)
# **放宽到 0.40**(实测 0.25 太严,aim phase 1 +5° yaw 后目标 cam xc
# 可能漂 0.20-0.30,正好在 0.25 边缘找不到 → 身份丢失 → 不射子弹)
HIT_XC_TOL = 0.40

# 算法阈值
ALIGN_DX_CONVERGE = 0.025          # |dx| < 此值才能射
YAW_PER_STEP_CAP_DEG = 15.0        # 单次 yaw 上限(用户硬约束)
D_SHOOT_M = 1.0                    # 停车距离:枪口到板子 1.0m(用户实测)
D_TARGET_M = 1.0                    # 起点到板子的初始距离(用户实测)
D_TARGET_TOL_M = 0.10               # D_target 容差 ±10cm
MAX_SHOTS_PER_TARGET = 5           # 用户硬约束
MIN_YOLO_SCORE = 0.50              # YOLO 置信度阈值
HIT_GRACE_S = 0.3                  # 射击后等多久再检测命中(实测电磁阀响应 < 200ms)


# ============================================================================
# helpers(cam detection + bbox 读取)
# ============================================================================

def get_animals(client, min_score):
    """读 cam2 实时检测结果,过滤 animal + score。"""
    try:
        ts = (client.get_task_state() or {}).get("task_state") or {}
        return [
            d for d in (ts.get("detections") or [])
            if d.get("label") == "animal"
            and float(d.get("score") or 0.0) >= min_score
        ]
    except Exception:
        return []


def get_animals_retry(client, min_score, retries=2, delay=0.1, label=""):
    """YOLO 单帧漏检重试。cam refresh ~50ms,默认 2 次 × 0.1s 上限 ~200ms。

    主路径嫌 retry 慢的话,可单独传 `retries=1, delay=0`。
    """
    best = []
    for i in range(retries):
        a = get_animals(client, min_score)
        if len(a) >= len(best):
            best = a
        if len(a) > 0:
            return a
        if i < retries - 1:
            time.sleep(delay)
    if label:
        print(f"    [retry] {label}: {retries} 次都空,放弃", flush=True)
    return best


def bbox_xc(det):
    return float((det.get("bbox_norm") or {}).get("x_center", 0.0))


def bbox_yc(det):
    return float((det.get("bbox_norm") or {}).get("y_center", 0.5))


def bbox_width(det):
    return float((det.get("bbox_norm") or {}).get("width", 0.0))


def bbox_height(det):
    return float((det.get("bbox_norm") or {}).get("height", 0.0))


# ============================================================================
# 几何推导
# ============================================================================

def xc_to_alpha_rad(xc, hfov_deg=CFOV_HORIZONTAL_DEG):
    """从 cam 视角 xc → 目标相对 cam 光轴的角度(弧度)。

    α > 0 → 目标在 cam 图像右侧(对应 chassis -y 方向,因为 cam 图像右 = 世界右)
    """
    return (xc - 0.5) * math.radians(hfov_deg)


def geometric_gun_xc(distance_m, gun_offset_cm=GUN_OFFSET_CM,
                     hfov_deg=CFOV_HORIZONTAL_DEG):
    """cam 中心 + 枪偏的几何:为了子弹命中目标,cam 中心应该指向哪个 xc。

    推导(实测修正 2026-08-01):
      - 子弹从枪口(在 cam 右边 25cm)沿枪口 ray 方向射出
      - 枪口 ray 相对车头方向(也是 cam 中心朝向)右偏 atan(0.25/D)
      - 要子弹打中目标(target 在 cam 正前方 D),车头必须**指向目标的左 25cm**
        (即 cam 中心 xc 偏左 = -atan(0.25/D)/HFOV_rad)
      - 之前代码用 +atan(0.25/D) 是错的:让 cam 中心指向 gun_xc=0.7 反而导致
        子弹打在 cam 视野中心(车头方向)= xc=0.5 位置,实测子弹打到板右边 10cm

      公式:gun_xc = 0.5 - atan(gun_offset/D) / HFOV_rad
    """
    if distance_m <= 0.05:
        distance_m = 0.05
    gun_offset_m = gun_offset_cm / 100.0
    angle_rad = math.atan2(gun_offset_m, distance_m)
    hfov_rad = math.radians(hfov_deg)
    # 2026-08-01 实测:子弹打板左边(面向板子看)= cam 视野中央的左边
    # → 板子在 cam 视野 gun_xc 位置,子弹从 cam 视野中央(车头方向)射出
    # → 子弹实际打 cam 视野 0.5 位置,板子在 0.701 → 子弹打板的左边
    # → gun_xc = 0.5 + atan(0.25/D)/HFOV(保持 +号)
    xc = 0.5 + angle_rad / hfov_rad
    return max(-0.5, min(1.5, xc))


def estimate_distance_from_bbox(wn, W=TARGET_BOARD_WIDTH_M,
                                hfov_deg=CFOV_HORIZONTAL_DEG,
                                k_min=0.4, k_max=1.2):
    """从 bbox 宽度估目标距离(针孔模型)。

    Z = W / (2 · wn · tan(H/2))
    clamp [k_min, k_max] 防止 wn 极小时爆炸。
    """
    if wn < 0.01:
        return 0.8   # fallback
    hfov_rad = math.radians(hfov_deg)
    Z = W / (2.0 * wn * math.tan(hfov_rad / 2))
    return max(k_min, min(k_max, Z))


# ============================================================================
# 目标识别
# ============================================================================

def find_next_target_by_bfs(animals, banned_xcs, current_target_xc=None):
    """**BFS 算法**找当前 cam 视野里的「下一只板」(2026-08-01 用户新需求)。

    **核心思路**:击倒 #1 后,cam 视野里 #2/#3/#4 自动降级成新的 #1/#2/#3。
    用 banned_xcs 排除已击倒的板子,在剩下的 animals 里找**最左**那只 = 下一只板。

    严格 L→R 射击顺序 → 下一只板一定在视野里最靠左(因为上一只已倒),
    即使 yolo 漏检导致视野数 ≠ 4,「最左」也是相对位置稳定的。

    Args:
        animals: cam 视野里的所有 detection
        banned_xcs: 已击倒板子的 cam xc 列表(每次 aim_and_shoot
                    命中/放弃后,会 shot_anchor_xc 加进来)
        current_target_xc: 当前正在打的板的 cam xc(可选,用于更严格的
                           「下一只」选择 — 选 current_target_xc 右侧最近的)

    Returns:
        (target, cur_xc) 或 (None, None) 如果找不到任何未 banned 的板子
    """
    if not animals:
        return None, None
    # Step 1: 用 banned_xcs 排除已击倒的板子
    candidates = []
    for a in sorted(animals, key=bbox_xc):
        xc = bbox_xc(a)
        banned = any(abs(xc - bx) < BANNED_XC_TOL for bx in banned_xcs)
        if not banned:
            candidates.append(a)
    if not candidates:
        return None, None
    # Step 2: 优先选 current_target_xc 右侧 8cm × N 处的板子(如果给了)
    if current_target_xc is not None:
        # 严格 L→R → 下一只板应在 current_target_xc 右侧最近处
        right_candidates = [a for a in candidates
                            if bbox_xc(a) > current_target_xc - BANNED_XC_TOL]
        if right_candidates:
            target = min(right_candidates, key=lambda a: bbox_xc(a))
            return target, bbox_xc(target)
    # fallback:返回 banned 之外最左的板子
    target = candidates[0]
    return target, bbox_xc(target)


# ============================================================================
# 瞄准 + 射击
# ============================================================================

def aim_and_shoot(client, target_idx, initial_xc, args,
                  D_est=None, initial_score=None,
                  n_at_initial_detect=None):
    """**前后微调 + 射击**,miss 时调距离(根据 err 符号 forward/back 5cm),最多 5 发。

    **2026-08-02 用户硬约束**:完全禁用 yaw/lateral,只允许前后移动。
    - aim phase 1(原开环 yaw)→ 改为 no-op(只 re-detect + log)
    - aim phase 2 miss 微调:
      - big yaw(err>0.05)→ forward/back 5cm(err>0→forward,err<0→backward)
      - mid yaw(err>0.015)→ forward/back 5cm
      - forward 5cm(|<0.015)→ 保留

    Args:
        client: RuntimeApiClient
        target_idx: L→R 编号(0-based)
        initial_xc: 初始 xc(冻结)
        args: 全局 args
        D_est: 初始 detect 时 bbox 反推的距离 m
        initial_score: 初始 detect 的 score

    Returns:
        (hit: bool, attempts: int, final_err: float)
    """
    # **2026-08-01 修复**:gun_xc 用实测 D_est 算,不用 args.shoot_distance=1.0m
    # 实测板子距离 ~0.5m,1.0m 会让 gun_xc 算错(0.701 vs 0.880)
    D_for_gun = D_est if D_est is not None else args.shoot_distance
    gun_xc = geometric_gun_xc(D_for_gun,
                              args.gun_offset_cm, args.cfov_deg)
    # **2026-08-01 用户实测(test_gun_geometry.py)**:
    # 把 #1 转到 cam 中央(xc=0.476)射 1 发,板子击中击倒!
    # → 枪口基本在 cam 中央(车头方向),gun_xc 几何补偿不需要
    # → aim 用 cam 中央 0.5 而非 gun_xc
    AIM_TARGET_XC = 0.5
    # **实测**:用户报枪能响但 5 发不中 — 怀疑 D_shoot 估错。
    # 改用"yc 实测距离"覆盖 args.shoot_distance(因为 bbox 反推有误差)。
    animals_init = get_animals_retry(client, args.min_score,
                                     label="aim 初始 re-detect",
                                     retries=2, delay=0.1)
    if animals_init:
        # 用 initial_xc 当 anchor(cam 视野里可能有 1-2 只,按 L→R 编号找)
        target_init = min(animals_init,
                          key=lambda a: abs(bbox_xc(a) - initial_xc))
        yc_now = bbox_yc(target_init)
        # yc 实测距离(用上次 detect 的 wn 估的 D_est,但 yc 在视野底边 → 距离近)
        # 简单经验:yc=0.30 → D ≈ 1.0m;yc=0.40 → D ≈ 0.5m
        # 这里用 init["D_est"] (从 bbox 反推)
        D_now = args.shoot_distance   # 先用默认
        # 实际可用 yc 反推距离(更准但需要先标定)
        print(f"\n[aim #{target_idx+1}] 目标对准 cam 中央 0.5 "
              f"(实测枪口基本在车头方向,不需 gun_xc 偏移;"
              f"D={D_for_gun:.2f}m, yc={yc_now:.3f})",
              flush=True)
        # **实时 gun_xc 校正**:每发前 re-detect 目标,用其 wn 反推 D,再算 gun_xc
        # 这样距离变化时 gun_xc 跟着调
    else:
        yc_now = 0.5
        D_now = args.shoot_distance
        print(f"\n[aim #{target_idx+1}] 目标对准 cam 中央 0.5 "
              f"(实测枪口基本在车头方向,不需 gun_xc 偏移)",
              flush=True)

    # Phase 1:[2026-08-02 NO-OP] 开环 yaw 已禁用,只做 re-detect + log
    # **2026-08-02 用户硬约束:禁 yaw** — phase 1 不再调整车身朝向,
    # 仅做一次 re-detect 确认目标身份,不调车。
    print(f"  [phase 1] **[NO-OP 2026-08-02:禁 yaw]** "
          f"只 re-detect + log,不调车...", flush=True)
    animals = get_animals_retry(client, args.min_score,
                                label="aim 一次算",
                                retries=2, delay=0.1)
    if not animals:
        print(f"    视野无目标,跳过 phase 1", flush=True)
    else:
        # **2026-08-01 BFS 改造**:aim phase 1 用 initial_xc 容差找本只板,
        # 不用 L→R 索引(索引可能退化)
        nearby_for_aim = [a for a in animals
                          if abs(bbox_xc(a) - initial_xc) < HIT_XC_TOL]
        if not nearby_for_aim:
            all_xcs = sorted([bbox_xc(a) for a in animals])
            print(f"    身份丢失 #{target_idx+1} (cam 视野 xc={all_xcs})",
                  flush=True)
        else:
            target = min(nearby_for_aim,
                         key=lambda a: abs(bbox_xc(a) - initial_xc))
            cur_xc = bbox_xc(target)
            dx = cur_xc - AIM_TARGET_XC
            # **2026-08-02 NO-OP**:不再发 yaw,仅记录期望 yaw 量供日志
            K = 0.75
            yaw_deg_hint = -dx * args.cfov_deg * K * YAW_SIGN
            AIM_PHASE1_YAW_CAP = 5.0
            yaw_deg_hint = max(-AIM_PHASE1_YAW_CAP,
                               min(AIM_PHASE1_YAW_CAP, yaw_deg_hint))
            print(f"    [NO-OP] 若未禁 yaw,本应发 yaw={yaw_deg_hint:+.2f}° "
                  f"(dx={dx:+.3f}, cap={AIM_PHASE1_YAW_CAP}°)— "
                  f"因 2026-08-02 硬约束,不发",
                  flush=True)

    # Phase 2:最多 5 发射击(用 initial_xc 当 anchor,L→R 编号对应)
    print(f"  [phase 2] 最多 {args.max_shots} 发...", flush=True)
    hit = False
    attempts = 0
    final_err = 0.0
    # anchor 冻结为 initial_xc(2026-08-01 修复 anchor 漂移问题)
    # cam 视野里多目标时不要让 anchor 跟 cur_xc 漂移

    # **2026-08-01 优化**:无进展计数 — 如果目标还在 + err 漂移 < 0.02
    # 连续 2 发没改善 → 提前 break,不再继续浪费发射
    no_progress_count = 0
    last_err = None

    # **2026-08-01 修复**:#1 命中后视野里剩 2 只(原本 4 只),L→R 索引 ti=0
    # 选 #4(被误认为 #1 还在)。把"初始 phase 1 时的 cam 视野数"传进
    # is_target_destroyed,只有 cam 视野数 < 初始值才算目标已不存在
    n_at_phase1_end = len(animals) if animals else 0

    for shot_i in range(1, args.max_shots + 1):
        attempts = shot_i
        # 射前 re-detect
        # **2026-08-01 BUG 修复**:cam 漏检 1 只时多等几帧,
        # 避免「身份丢失」不射子弹(实测 cam refresh ~50ms,默认 2×0.1s 略紧)
        animals = get_animals_retry(client, args.min_score,
                                    label=f"shoot 射前 a{shot_i}",
                                    retries=3, delay=0.1)
        if not animals:
            print(f"    [a{shot_i}] 视野无目标,放弃这只", flush=True)
            break
        # **debug**:打印 cam 视野里所有目标 xc,确认 anchor 找到的"目标"
        # 是不是真的对应 #1
        all_xcs = sorted([bbox_xc(a) for a in animals])
        print(f"    [a{shot_i}] cam 视野所有 xc: "
              f"{[f'{x:+.2f}' for x in all_xcs]} "
              f"(anchor=initial_xc={initial_xc:+.2f})", flush=True)
        # **2026-08-01 关键修复**:射前必须用 initial_xc 容差做**身份门控**,
        # 不再用 L→R 索引(target_idx)选目标。理由:实测 #1 aim phase 1
        # 发 +9.34° 后 cam 视野里只剩 [+0.58,+0.83](原 #3/#4),
        # L→R 索引 0 = +0.577(原 #3 位置),但 initial_xc=0.190 ± 0.25
        # 容差 [-0.06,+0.44] 内**无 detection**,说明 #1 已不在原位 →
        # 不能选 +0.577 当 #1 射,会打到 #3(无辜的目标)
        nearby_pre = [a for a in animals
                      if abs(bbox_xc(a) - initial_xc) < HIT_XC_TOL]
        if not nearby_pre:
            # **2026-08-02 改进**:容差内无 detection 时,**先重试 re-detect
            # 几帧**(给 yolo 机会重新检测到漏检板),而不是立即放弃。
            # 用户选 C 方案:不追求 cam 中央对齐,接受命中率低,
            # 但不要因为 yolo 漏检 1 帧就放弃整只。
            print(f"    [a{shot_i}] 容差内无 detection → 重试 re-detect 4 帧...",
                  flush=True)
            for retry_i in range(4):
                time.sleep(0.2)
                animals_retry = get_animals(client, args.min_score)
                nearby_pre = [a for a in animals_retry
                              if abs(bbox_xc(a) - initial_xc) < HIT_XC_TOL]
                if nearby_pre:
                    animals = animals_retry
                    all_xcs = sorted([bbox_xc(a) for a in animals])
                    print(f"    [a{shot_i}] 重试 {retry_i+1}/4 成功,"
                          f"找到 {len(nearby_pre)} 个候选", flush=True)
                    break
            if not nearby_pre:
                # **身份丢失**:4 帧 0.8s 还没找到 → 真的看不到 → 放弃
                print(f"    [a{shot_i}] ✗ 身份丢失!initial_xc={initial_xc:+.3f} "
                      f"± {HIT_XC_TOL} 容差内 4 帧 re-detect 仍无 detection "
                      f"(视野 xc={all_xcs})→ 放弃这只",
                      flush=True)
                break
        # 选容差内最近 detection 作为本发目标
        target = min(nearby_pre,
                     key=lambda a: abs(bbox_xc(a) - initial_xc))
        cur_xc = bbox_xc(target)
        # **2026-08-01 双 anchor 方案**:冻结 shot_anchor_xc = cur_xc,
        # 用它做本发的射后消失判定(同相机姿态,坐标可比)
        shot_anchor_xc = cur_xc
        shot_anchor_score = target.get("score", 1.0)
        print(f"    [a{shot_i}] 身份确认:cur_xc={cur_xc:+.3f} "
              f"距 initial_xc={initial_xc:+.3f}={cur_xc-initial_xc:+.3f} "
              f"score={shot_anchor_score:.3f} "
              f"(shot_anchor 冻结={shot_anchor_xc:+.3f})",
              flush=True)

        # **2026-08-01**:对 cam 中央 0.5(实测枪口在车头方向)
        err = cur_xc - AIM_TARGET_XC
        print(f"    [a{shot_i}] cur_xc={cur_xc:+.3f} err={err:+.3f}",
              flush=True)
        # **2026-08-01 修复**:不要再"射前 err<0.01 → 命中"假判命中!
        # 之前的逻辑没真的开枪,只是 cam 视野对齐就 break,4/4 都是假命中
        # 移除这个判定,每发都真射击
        print(f"    >>> 射击...", flush=True)
        try:
            car_call(client, "shooting", timeout=5)
        except Exception as e:
            print(f"    [shoot err] {e}", file=sys.stderr)
        time.sleep(HIT_GRACE_S)

        # 命中判定:re-detect,扫描整帧看 shot_anchor_xc ± HIT_XC_TOL 内
        # 还有没有 detection
        animals_after = get_animals(client, args.min_score)
        all_xcs_after = sorted([bbox_xc(a) for a in animals_after]) \
            if animals_after else []
        print(f"    [a{shot_i}] 射后 cam 视野 xc={all_xcs_after} "
              f"(shot_anchor={shot_anchor_xc:+.3f})", flush=True)
        # **2026-08-01 双 anchor 命中判定**:
        # - shot_anchor_xc ± HIT_XC_TOL 内有 detection → miss
        # - 容差内无 detection → 命中(目标已倒/出视野)
        # **不**再用 L→R 索引 target_after(会因为索引退化选错板子)
        # **不**再用 score 下降单独报命中(容差内有 detection 时
        # score 下降可能只是 yolo 抖动/遮挡,不是真击中)
        target_destroyed = False
        msg = ""
        if not animals_after:
            target_destroyed = True
            msg = "cam 视野空(目标完全消失)"
        else:
            nearby_after = [a for a in animals_after
                            if abs(bbox_xc(a) - shot_anchor_xc) < HIT_XC_TOL]
            if not nearby_after:
                # 容差内无 detection → 命中
                target_destroyed = True
                msg = (f"shot_anchor_xc={shot_anchor_xc:+.3f} "
                       f"± {HIT_XC_TOL} 内无 detection"
                       f"(射后视野 xc={all_xcs_after})")
            # else:容差内还有 detection → 目标还在原位,不算命中
        if target_destroyed:
            print(f"    ✓ 命中!{msg}", flush=True)
            hit = True
            final_err = err
            break
        # **debug**:目标还在(shot_anchor 容差内有 detection)
        cur_score = nearby_after[0].get("score", 1.0)
        print(f"    [re-detect] 目标还在(score={cur_score:.3f},"
              f"shot_anchor xc={bbox_xc(nearby_after[0]):+.3f})",
              flush=True)

        # **2026-08-01 优化**:无进展检测 — err 漂移 < 0.02 + score 变化 < 0.05
        # 连续 2 发没改善 → 提前放弃(目标可能根本打不到,比如 yaw 响应延迟)
        # 用 shot_anchor_score(本发冻结)代替 initial_score,避免长期衰减
        # 被误判为「无进展」
        if last_err is not None and abs(err - last_err) < 0.02 \
                and abs(cur_score - shot_anchor_score) < 0.05:
            no_progress_count += 1
            if no_progress_count >= 2:
                print(f"    [abort] 连续 {no_progress_count} 发无进展 "
                      f"(err 漂移 < 0.02, score 变化 < 0.05) → 放弃这只",
                      flush=True)
                break
        else:
            no_progress_count = 0
        last_err = err

        # miss → **[2026-08-02 用户硬约束:禁 yaw]** forward/back 5cm 微调
        # 原开环 yaw 大步/中步 → 改为 forward/back 5cm(根据 err 符号判断方向)
        # **2026-08-01 修复**:用容差内的 detection 算 cur_xc_now,
        # 不用 target_after(可能选错板子)
        if nearby_after:
            cur_xc_now = bbox_xc(nearby_after[0])
        else:
            # 容差内已没 detection(理论上这种情况前面 target_destroyed
            # 已 break,但保险起见)
            cur_xc_now = initial_xc   # fallback,不调
            print(f"    [miss→fallback] 容差内无 detection,"
                  f"用 initial_xc 兜底", flush=True)
        # **2026-08-01**:对 cam 中央 0.5(实测枪口在车头方向)
        err = cur_xc_now - AIM_TARGET_XC
        final_err = err

        # **2026-08-02 用户硬约束 v2**:**完全不调车**(只接受 cam xc 在 ±0.3 内都射)
        # 用户选 C 方案:保持禁 yaw,不追求 cam 中央对齐。
        # 物理事实:cam 视角固定 + 禁 yaw + 车头初始没对准 → cam xc 不会
        # 收敛到 0.5,前进/后退只会改 wn 和 score,不会改 cam xc。
        # 所以 miss 阶段调距离毫无意义 → 改为 NO-OP,直接进入下一发。
        # 接受命中率可能很低,但每次都真射。
        print(f"    [miss→noop] err={err:+.3f}, "
              f"完全不调车(用户硬约束 v2:±0.3 容差内都射)",
              flush=True)
        time.sleep(0.2)

    # **2026-08-01 优化**:如果目标还在 + score 几乎不变(差 < 0.05)
    # + err 也在 ±0.02 内连续 2 发没进展 → 提前 break,不再继续
    # 节省命中不了时浪费的后续发射
    # (逻辑放在 phase 2 循环结束后 — 跟下面 while 同级)

    return hit, attempts, final_err


# ============================================================================
# 归位
# ============================================================================

def return_to_origin(client, original_x, original_y, args, label=""):
    """命中 OR 放弃都执行:车回到 odom 起点。

    **2026-08-02 用户硬约束:禁 yaw/lateral** — 只允许沿当前车头
    方向前进/后退,不能 correct_yaw 纠偏。
    """
    cur = read_odom(client)
    if cur is None:
        print(f"  [{label}] 读不到 odom,跳过归位", flush=True)
        return
    cur_x, cur_y, cur_yaw = cur

    dx = original_x - cur_x
    dy = original_y - cur_y
    dist = math.sqrt(dx**2 + dy**2)
    print(f"\n[return #{label}] cur=({cur_x:+.3f}, {cur_y:+.3f}) "
          f"start=({original_x:+.3f}, {original_y:+.3f}) "
          f"dx={dx*100:+.1f}cm dy={dy*100:+.1f}cm "
          f"dist={dist*100:.1f}cm",
          flush=True)

    if dist < 0.02:
        print(f"  [{label}] 距离 < 2cm,已近原点,跳过", flush=True)
        return

    # 沿 cam 视野前方方向(odom +x)走 dist(粗归位)
    # **实测修正 2026-08-01**:4 axis 实测 → +x=前,+y=左前,-y=右后
    # 归位 = 后退(odom -x)
    try:
        car_call(client, "move_for", [-dist, 0.0, 0.0],
                 timeout=max(5, dist * 20))
        print(f"  [{label}] 沿 cam 后方(-x)归位 {dist*100:.1f}cm",
              flush=True)
    except Exception as e:
        print(f"  [{label}] 归位 err: {e}", file=sys.stderr)

    # 2026-08-02 用户硬约束:禁 yaw — 仅 log 当前 yaw,不调车
    cur = read_odom(client)
    if cur is not None:
        cur_x, cur_y, cur_yaw = cur
        print(f"  [{label}] [NO-OP yaw 恢复] 禁 yaw, "
              f"当前 yaw={math.degrees(cur_yaw):+.1f}° 不调",
              flush=True)


# ============================================================================
# 主流程
# ============================================================================

def main():
    ap = argparse.ArgumentParser(
        description="直走到 4 个目标正前方 → 瞄准 → 射击 → 归位"
    )
    # 几何
    ap.add_argument("--gun-offset-cm", type=float, default=GUN_OFFSET_CM,
                    help=f"枪在 cam 右边 cm (默认 {GUN_OFFSET_CM})")
    ap.add_argument("--shoot-distance", type=float, default=D_SHOOT_M,
                    help=f"停车距离(目标正前方 m,默认 {D_SHOOT_M})")
    ap.add_argument("--target-distance", type=float, default=D_TARGET_M,
                    help=f"起点到板子列的初始距离(估算,m,"
                         f"默认 {D_TARGET_M},实际 ±{D_TARGET_TOL_M*100:.0f}cm)")
    ap.add_argument("--cfov-deg", type=float, default=CFOV_HORIZONTAL_DEG,
                    help=f"cam2 水平视场角(度,默认 {CFOV_HORIZONTAL_DEG})")
    # 算法
    ap.add_argument("--min-score", type=float, default=MIN_YOLO_SCORE,
                    help=f"YOLO 置信度阈值(默认 {MIN_YOLO_SCORE})")
    ap.add_argument("--max-shots", type=int, default=MAX_SHOTS_PER_TARGET,
                    help=f"每只目标最多射击次数(默认 {MAX_SHOTS_PER_TARGET})")
    ap.add_argument("--yaw-cap-deg", type=float, default=YAW_PER_STEP_CAP_DEG,
                    help=f"单次 yaw 上限(度,默认 {YAW_PER_STEP_CAP_DEG})")
    ap.add_argument("--drift-budget", type=float, default=90.0,
                    help="纠偏预算上限(度,默认 90)")
    # 流程控制
    ap.add_argument("--no-reset", action="store_true",
                    help="不重置 odom(默认 ON:跑前 reset_position)")
    ap.add_argument("--with-correction", action="store_true",
                    help="开 correct_yaw 横向纠偏(默认 OFF:实测 chassis "
                         "yaw 响应有问题,纠偏修了跟没修一样)")
    ap.add_argument("--skip-index", type=int, nargs="*", default=[],
                    help="跳过这些 L→R 编号(0-based)")
    ap.add_argument("--targets", type=str, default=None,
                    help="要射击的目标编号,空格分隔(1-based),如 '1 3 4'。"
                         "不传就跑前询问。传 'all' = 全部")
    args = ap.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()

    # 跑前 reset odom(避免历史漂移污染起点)
    if not args.no_reset:
        try:
            car_call(client, "reset_position", timeout=5)
            print("[reset] odom 已清零", flush=True)
            time.sleep(0.5)
        except Exception as e:
            print(f"[reset err] {e}", file=sys.stderr)

    # 起点 odom
    start = read_odom(client)
    if start is None:
        print("[err] 读不到 odom,退出", file=sys.stderr)
        return 1
    sx, sy, syaw = start
    print(f"[start] x={sx:+.3f}m y={sy:+.3f}m yaw={math.degrees(syaw):+.2f}°",
          flush=True)

    # ---- Step 1:检测目标 + Kalman 跟踪初始化 ----
    # **2026-08-01 BFS + Kalman 改造**:不再硬要求 4 只板
    # 等待 ≥1 只 detection,然后用 BoardTrack 跟踪 + 预测填补漏检
    print(f"\n[detect] 等待 ≥1 只目标...", flush=True)
    t0 = time.time()
    animals = []
    while time.time() - t0 < 15:
        animals = get_animals(client, args.min_score)
        if len(animals) >= 1:
            break
        time.sleep(0.3)
    if not animals:
        print(f"[err] cam 视野 15s 都没目标,退出", file=sys.stderr)
        return 1

    # **初始化** :按 L→R 排序,记录每只板的初始 xc / D_est
    # **2026-08-02 改进**:Kalman 跟踪(BFS 之外)已删除 — 漏检板由
    # get_animals_retry 多帧等实测出现更鲁棒;predicted track 偏差大
    # 注入 BFS 会让 aim 身份门控误判丢失。
    animals_sorted = sorted(animals, key=bbox_xc)
    print(f"[detect] 检测到 {len(animals_sorted)} 只(xc 左→右):",
          flush=True)
    initial_xcs = []
    for i, a in enumerate(animals_sorted):
        xc = bbox_xc(a)
        yc = bbox_yc(a)
        wn = bbox_width(a)
        score = a.get("score", 0.0)
        D_est = estimate_distance_from_bbox(wn, hfov_deg=args.cfov_deg)
        print(f"  #{i+1}: xc={xc:+.2f} yc={yc:.2f} wn={wn:.3f} "
              f"score={score:.2f} → D_est={D_est:.2f}m",
              flush=True)
        initial_xcs.append({"xc": xc, "yc": yc, "wn": wn, "D_est": D_est,
                             "score": score})

    # 用户给的 D_target 跟实测做平均
    if initial_xcs:
        D_target = initial_xcs[0]["D_est"]
        print(f"[plan] D_target={D_target:.2f}m "
              f"(实测 bbox 反推;用户给 {args.target_distance}m 忽略)",
              flush=True)
    else:
        D_target = args.target_distance
        print(f"[plan] D_target={D_target:.2f}m (用户给,无实测)",
              flush=True)

    # ---- 询问要射哪几个目标(用户硬约束 2026-08-01)----
    n_targets = len(initial_xcs)
    # args.targets 可能值:
    #   None          → 跑前询问
    #   "all"         → 全部
    #   "1 3 4"       → 射 #1#3#4
    if args.targets is None:
        # 交互式询问
        print(f"\n[select] 检测到 {n_targets} 只目标,你要射哪几个?",
              flush=True)
        print(f"  输入 1-based 编号,空格分隔(如 '1 3 4')。"
              f"直接回车 = 全部。输入 q 退出。", flush=True)
        while True:
            try:
                user_input = input("  请输入> ").strip()
            except EOFError:
                user_input = ""
            if user_input.lower() in ("q", "quit", "exit"):
                print("  [abort] 用户取消,退出", flush=True)
                return 0
            if not user_input:
                targets_to_shoot = set(range(n_targets))
                print(f"  → 全部 {n_targets} 只", flush=True)
                break
            try:
                nums = [int(x) for x in user_input.split()]
                # 1-based → 0-based
                indices = [n - 1 for n in nums]
                # 校验
                if any(i < 0 or i >= n_targets for i in indices):
                    print(f"  编号超出范围(1-{n_targets}),重输", flush=True)
                    continue
                targets_to_shoot = set(indices)
                nums_1based = sorted([i + 1 for i in indices])
                print(f"  → 选 #{' #'.join(str(n) for n in nums_1based)}",
                      flush=True)
                break
            except ValueError:
                print(f"  输入解析失败,重输", flush=True)
    elif args.targets.lower() == "all":
        targets_to_shoot = set(range(n_targets))
        print(f"[select] 全部 {n_targets} 只(--targets all)", flush=True)
    else:
        try:
            nums = [int(x) for x in args.targets.split()]
            indices = [n - 1 for n in nums]
            if any(i < 0 or i >= n_targets for i in indices):
                print(f"[err] --targets 编号超出范围(1-{n_targets})",
                      file=sys.stderr)
                return 1
            targets_to_shoot = set(indices)
            nums_1based = sorted([i + 1 for i in indices])
            print(f"[select] 选 #{' #'.join(str(n) for n in nums_1based)} "
                  f"(--targets '{args.targets}')", flush=True)
        except ValueError:
            print(f"[err] --targets 解析失败:{args.targets}",
                  file=sys.stderr)
            return 1

    # ---- 射击循环(L→R)----
    results = []

    # **2026-08-01 新流程**:BFS + 8cm 位移推算
    # 准备:用户选要射的目标(按 0-based L→R 索引排序)
    sorted_targets = sorted(targets_to_shoot)
    banned_xcs = []   # 已击倒板子的 cam xc(BFS 排除用)
    current_target_xc = None   # 当前 cam 视野里正在打的板子的 xc
    prev_target_idx = None   # 上一只板的 L→R 索引(用于计算 step_count)

    for i, ti in enumerate(sorted_targets):
        if ti in args.skip_index:
            print(f"\n========== 跳过 #{ti+1} (--skip-index) ==========",
                  flush=True)
            continue
        init = initial_xcs[ti]
        is_last = (i == len(sorted_targets) - 1)

        print(f"\n========== 目标 #{ti+1}/{len(initial_xcs)} ==========",
              flush=True)
        print(f"  初始 xc={init['xc']:+.3f} yc={init['yc']:.3f} "
              f"D_est={init['D_est']:.2f}m "
              f"(banned_xcs={banned_xcs})",
              flush=True)

        # Phase A:**BFS**(2026-08-01)
        # 每帧检测后,用 BFS 在 banned 之外找最左未 banned 的板子
        animals = get_animals_retry(client, args.min_score,
                                    label=f"target #{ti+1} detect",
                                    retries=2, delay=0.1)
        if not animals:
            print(f"  [detect #{ti+1}] ✗ 视野空,跳过这只", flush=True)
            results.append({"idx": ti, "hit": False, "attempts": 0,
                            "err": 0.0, "init_xc": init["xc"]})
            continue
        meas_xcs = [bbox_xc(a) for a in animals]
        real_xcs = sorted([f"{m:+.2f}" for m in meas_xcs])
        print(f"  [detect #{ti+1}] cam 视野 xc: {real_xcs}",
              flush=True)
        bfs_animals = list(animals)

        target, cur_xc = find_next_target_by_bfs(
            bfs_animals, banned_xcs, current_target_xc=current_target_xc)
        if target is None:
            print(f"  [BFS #{ti+1}] ✗ 找不到未 banned 的板子 "
                  f"(实测 xc={real_xcs})",
                  flush=True)
            results.append({"idx": ti, "hit": False, "attempts": 0,
                            "err": 0.0, "init_xc": init["xc"]})
            continue
        all_xcs_visible = sorted([bbox_xc(a) for a in bfs_animals])
        print(f"  [BFS #{ti+1}] 选中 cur_xc={cur_xc:+.3f} "
              f"(视野 xc={[f'{x:+.2f}' for x in all_xcs_visible]})",
              flush=True)
        current_target_xc = cur_xc

        # Phase B:**[2026-08-02 NO-OP]** 单步 turn 已禁用
        # **2026-08-02 用户硬约束:禁 yaw/lateral** — 删掉 turn_to_align 调用,
        # 仅保留日志说明。8cm 位移推算假设目标已接近 cam 中央,不再做 yaw 修正。
        print(f"\n[turn #{ti+1}] **[NO-OP 2026-08-02:禁 yaw]** "
              f"原 turn_to_align 已删除,不调车...",
              flush=True)

        # Phase C:瞄准 + 射击(最多 5 发)
        hit, attempts, final_err = aim_and_shoot(
            client, ti, init["xc"], args,
            D_est=init["D_est"],
            initial_score=init["score"],
            n_at_initial_detect=n_targets
        )

        results.append({"idx": ti, "hit": hit, "attempts": attempts,
                        "err": final_err, "init_xc": init["xc"]})

        # **2026-08-01 新增**:无论命中/放弃,都把当前 shot 锚点加入 banned
        # 这样下一次 BFS 不会把同一只板再选出来(避免重复射击)
        banned_xcs.append(cur_xc)
        print(f"  [banned] #{ti+1} xc={cur_xc:+.3f} 加入 banned "
              f"(total={len(banned_xcs)})", flush=True)

        # Phase D:**8cm 位移推算**到下一只板(用户原话 2026-08-01)
        if not is_last:
            next_ti = sorted_targets[i + 1]
            step_count = next_ti - ti   # 跳过几只板
            drive_m = BOARD_SPACING_M * step_count
            print(f"\n[drive #{ti+1}→#{next_ti+1}] 直行 {drive_m*100:.1f}cm "
                  f"= {BOARD_SPACING_M*100:.0f}cm × {step_count} 步 "
                  f"(板间距固定)",
                  flush=True)
            try:
                car_call(client, "move_for", [drive_m, 0.0, 0.0],
                         timeout=max(5, drive_m * 20))
                print(f"  [drive] 完成 {drive_m*100:.1f}cm "
                      f"沿车头方向直行", flush=True)
            except Exception as e:
                print(f"  [drive err] {e}", file=sys.stderr)
            time.sleep(0.5)
        else:
            # 最后一只 → 归位(用户硬约束:最后归位)
            return_to_origin(client, sx, sy, args,
                             label=f"#{ti+1} (最后一只)")
            continue   # 跳过下面的 time.sleep(已在 return_to_origin 内)

        time.sleep(0.3)

    # ---- 汇总 ----
    print(f"\n{'='*60}", flush=True)
    print(f"========== 射击任务汇总 ==========", flush=True)
    print(f"{'='*60}", flush=True)
    hit_count = sum(1 for r in results if r["hit"])
    print(f"  命中 {hit_count}/{len(results)}", flush=True)
    for r in results:
        status = "✓ 命中" if r["hit"] else f"✗ 放弃({r['attempts']} 发)"
        print(f"  #{r['idx']+1}  {status}  "
              f"init_xc={r['init_xc']:+.2f} err={r['err']:+.3f}",
              flush=True)
    print(f"{'='*60}", flush=True)

    return 0 if hit_count == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
