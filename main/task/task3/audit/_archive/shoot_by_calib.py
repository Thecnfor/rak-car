#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/shoot_by_calib.py - 用校准表指导自动射击(2026-08-03 v3)

**核心思路**(2026-08-03 用户策略 v3):
1. 起点 cam 视野 4 只板 = 板上 L→R #1..#4(用户摆车保证)
2. `targets_to_shoot` 是按板上 L→R 顺序索引(如 [1, 3, 4] = 击倒第 1, 3, 4 只,跳过第 2)
3. **每只板 (选/跳过/打) 后直行 8cm 到下一只板上板正前方**
   - 因为 4 只板时 #3 #4 离车远,5 发循环调 yaw 不够;直行让下一只板靠近车
4. 选板永远用「车上视野 L→R 第 1 只非 banned 板」
5. 跳过时也加 banned (因为板上 #2 还在场上,不排除会被下一轮选到)
6. 5 发循环锁 first_xc ± 0.20,所有发都打同一只板
7. 击倒/未命中后调 yaw 微调(不动车位置)
8. 击倒最后一个指定板上板后停止移动

**校准数据**:由 manual_calibrate.py 写入 calib JSON,本脚本按
(yaw_bucket, n_view, 板上板上 L→R 顺序索引)三元组查表。

**用法**:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.shoot_by_calib --targets "1 3 4"

**前提**:
- 车手动摆到能看全 4 只板的起点(板上 4 只板都在 cam 视野里)
- 机械臂位置固定:`arm_seq_v9 --y1 -0.150 --x -0.200 --arm-angle 90 --hand-angle -90`
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys
import time

from main.api_client import RuntimeApiClient

# 复用 shoot_4_targets 的核心工具
from main.tasks.task333.shoot_4_targets import (
    bbox_xc,
    bbox_yc,
    bbox_width,
    MIN_YOLO_SCORE,
    HIT_XC_TOL,
    BANNED_XC_TOL,
    YAW_SIGN,
    get_animals,
    get_animals_retry,
)


def car_call(client, name, *args, timeout=10.0, **kwargs):
    job = client.execute_car_action(name, *args, timeout=timeout,
                                    sync=True, **kwargs)
    if job.get("status") != "succeeded":
        raise RuntimeError(f"car.{name} failed: {job.get('error')}")
    return job.get("result")


def read_odom(client):
    try:
        odo = (client.get_runtime() or {}).get("runtime", {}).get("odometry") or [0, 0, 0]
        return float(odo[0]), float(odo[1]), float(odo[2])
    except Exception:
        return None


CALIB_JSON_PATH = r"C:\Users\花花世界\Desktop\天道酬勤\rak-car\manual_calibrate_result.json"
CALIB_TABLE = {}   # {(yaw_bucket, n_view, tid): {...}}


def _yaw_bucket(yaw_deg, bucket_width=5.0):
    """把 yaw 度数量化到 ±180° 区间的桶。"""
    y = yaw_deg % 360
    if y > 180:
        y -= 360
    elif y < -180:
        y += 360
    return int(y / bucket_width) * bucket_width


def load_calib_table(json_path=CALIB_JSON_PATH):
    """从 manual_calibrate_result.json 加载校准表。"""
    if not os.path.exists(json_path):
        print(f"[calib] 警告 校准文件 {json_path} 不存在,"
              f"请先跑 `python -m main.tasks.task333.manual_calibrate` 校准",
              file=sys.stderr)
        return {}
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    samples_by_key = {}
    all_shots = data.get("all_shots", [])
    for s in all_shots:
        if not (s.get("result") and s["result"].get("hit")):
            continue
        tid = s["result"]["target_id"]
        hit_xc = s["result"].get("hit_xc")
        if hit_xc is None:
            continue
        n_view = len(s.get("cam_view", []))
        yaw = s.get("odom", {}).get("yaw_deg", 0.0)
        yaw_b = _yaw_bucket(yaw)
        key = (yaw_b, n_view, tid)
        samples_by_key.setdefault(key, []).append((
            yaw,
            s.get("odom", {}).get("x", 0.0),
            s.get("odom", {}).get("y", 0.0),
        ))

    calib = {}
    for name, info in data.get("calib_table", {}).items():
        m = re.match(r"yaw(-?\d+)_view(\d+)_target(\d+)", name)
        if m:
            yaw_b = int(m.group(1))
            n_view = int(m.group(2))
            tid = int(m.group(3))
        else:
            m2 = re.match(r"view(\d+)_target(\d+)", name)
            if not m2:
                continue
            yaw_b = None
            n_view = int(m2.group(1))
            tid = int(m2.group(2))
        key = (yaw_b, n_view, tid)
        odom_x_samples = info.get("odom_x_samples") or []
        odom_y_samples = info.get("odom_y_samples") or []
        yaw_samples = info.get("yaw_samples") or []
        if not odom_x_samples:
            for (yb, nv, t), samples in samples_by_key.items():
                if nv == n_view and t == tid:
                    odom_x_samples = [s[1] for s in samples]
                    odom_y_samples = [s[2] for s in samples]
                    yaw_samples = [s[0] for s in samples]
                    break
        calib[key] = {
            "xc_mean": info.get("xc_mean"),
            "xc_min": info.get("xc_min"),
            "xc_max": info.get("xc_max"),
            "n_hits": info.get("n_hits", 0),
            "reliable": info.get("reliable", False),
            "yaw_bucket_width_deg": info.get("yaw_bucket_width_deg", 5.0),
            "yaw_samples": yaw_samples,
            "odom_x_samples": odom_x_samples,
            "odom_y_samples": odom_y_samples,
        }
    return calib


def find_nearest_yaw_bucket(current_yaw_deg, calib_table, bucket_width=5.0):
    """找当前 yaw 最近的桶。"""
    if not calib_table:
        return None, float("inf")
    bucket_yaws = {}
    for (yb, _nv, _tid), info in calib_table.items():
        if yb is not None:
            bucket_yaws[yb] = yb
        else:
            ys = info.get("yaw_samples") or []
            if ys:
                mean_y = sum(ys) / len(ys)
                qy = int(mean_y / bucket_width) * bucket_width
                bucket_yaws[qy] = mean_y
    if not bucket_yaws:
        return None, float("inf")
    best = None
    best_d = float("inf")
    for b, mean in bucket_yaws.items():
        raw_d = current_yaw_deg - mean
        while raw_d > 180:
            raw_d -= 360
        while raw_d < -180:
            raw_d += 360
        if abs(raw_d) < best_d:
            best_d = abs(raw_d)
            best = b
    return best, best_d


def pick_target_by_tid(animals, tid, banned_xcs):
    """按板上 L→R 索引选目标(排除 banned 的 cam xc,容差 BANNED_XC_TOL)。"""
    candidates = [a for a in sorted(animals, key=bbox_xc)
                  if not any(abs(bbox_xc(a) - bx) < BANNED_XC_TOL
                             for bx in banned_xcs)]
    if tid - 1 >= len(candidates):
        return None, None
    target = candidates[tid - 1]
    return target, bbox_xc(target)


def pick_target_by_shot_seq(animals, shot_seq, hit_count):
    """**2026-08-03 大改**:选板上 #shot_seq = 车上视野 L→R 第 (shot_seq - hit_count) 只。

    不依赖 xc 容差, 用板上 L→R 索引直接定位。
    hit_count = 已击倒数 (跳过不计), 因此:
      - 起点视野 2 只 = #1 #2, hit_count=0:
        - 选 #1 → in_view_idx=1, 选 L→R 第 1 只 = 板上 #1 ✓
        - 选 #3 → in_view_idx=3 > 2, 报 None (用户应直行后再选)
      - 击倒 #1 后直行 8cm, hit_count=1:
        - 车上视野 2 只 = #2 #3
        - 选 #3 → in_view_idx=2, 选 L→R 第 2 只 = 板上 #3 ✓

    关键: 不传 banned_xcs, 不依赖 BANNED_XC_TOL, 不会误排除场上合法的板上 #k。
    """
    in_view_idx = shot_seq - hit_count
    candidates = sorted(animals, key=bbox_xc)
    if in_view_idx < 1 or in_view_idx > len(candidates):
        return None, None
    target = candidates[in_view_idx - 1]
    return target, bbox_xc(target)


def pick_target_by_calib(animals, tid, banned_xcs, calib_xc):
    """用 calib_xc 找最近的非 banned 板(不靠 L→R 编号)。"""
    candidates = [a for a in sorted(animals, key=bbox_xc)
                  if not any(abs(bbox_xc(a) - bx) < BANNED_XC_TOL
                             for bx in banned_xcs)]
    if not candidates:
        return None, None
    target = min(candidates, key=lambda a: abs(bbox_xc(a) - calib_xc))
    return target, bbox_xc(target)


def pick_target_by_xc(animals, target_xc, banned_xcs=None, xc_tol=0.20):
    """锁定 first_xc: 选 cam 视野里 xc 在 target_xc ± xc_tol 内最近的板。

    用于 5 发循环里锁定第一发选中的板,避免 yolo 漏检导致每发重选不同板。
    容差内无板 → 视野里该位置的板消失了(被打了/推走),返回 None。

    banned_xcs 参数保留向后兼容, 默认 None 不排除任何板。
    """
    if banned_xcs is None:
        candidates = [a for a in animals
                      if abs(bbox_xc(a) - target_xc) < xc_tol]
    else:
        candidates = [a for a in animals
                      if abs(bbox_xc(a) - target_xc) < xc_tol
                      and not any(abs(bbox_xc(a) - bx) < BANNED_XC_TOL
                                  for bx in banned_xcs)]
    if not candidates:
        return None, None
    target = min(candidates, key=lambda a: abs(bbox_xc(a) - target_xc))
    return target, bbox_xc(target)


def get_calib_xc(nearest_bucket, n_view, tid, calib_table):
    """查校准表,返回 (calib_xc, src_desc) 或 (None, msg) 表示无校准。"""
    if nearest_bucket is None:
        return None, "无 yaw 桶"
    calib_key = (nearest_bucket, n_view, tid)
    if calib_key in calib_table:
        calib_xc = calib_table[calib_key]["xc_mean"]
        return calib_xc, f"yaw={nearest_bucket:+.0f} + n_view={n_view}"
    for nv_fallback in [4, 3, 2, 1]:
        if (nearest_bucket, nv_fallback, tid) in calib_table:
            calib_xc = calib_table[
                (nearest_bucket, nv_fallback, tid)]["xc_mean"]
            return calib_xc, (f"yaw={nearest_bucket:+.0f} + "
                              f"n_view={nv_fallback} fallback")
    return None, "无校准"


def main():
    ap = argparse.ArgumentParser(description="用校准表指导射击")
    ap.add_argument("--targets", type=str, default=None,
                    help="按板上 L→R 顺序索引(空格分隔,1-based),"
                         "如 '1 3 4' = 击倒第 1, 3, 4 只(跳过第 2)。"
                         "不传则跑前询问")
    ap.add_argument("--cfov-deg", type=float, default=70.0,
                    help="cam 水平视场角(度,默认 70)")
    ap.add_argument("--min-score", type=float, default=MIN_YOLO_SCORE,
                    help=f"YOLO 置信度阈值(默认 {MIN_YOLO_SCORE},"
                         f"调低可检测更远的板,但 false positive 增多)")
    args = ap.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()

    calib_loaded = load_calib_table(CALIB_JSON_PATH)
    if calib_loaded:
        CALIB_TABLE.clear()
        CALIB_TABLE.update(calib_loaded)

    start = read_odom(client)
    if start is None:
        sx, sy, start_yaw_deg = 0.0, 0.0, 0.0
    else:
        sx, sy = start[0], start[1]
        start_yaw_rad = start[2] if (len(start) > 2 and start[2] is not None) else 0.0
        start_yaw_deg = math.degrees(start_yaw_rad)

    print(f"[start] x={sx:+.3f}m y={sy:+.3f}m "
          f"yaw={start_yaw_deg:+.2f}°", flush=True)

    nearest_bucket, yaw_dist = find_nearest_yaw_bucket(
        start_yaw_deg, CALIB_TABLE)
    print(f"\n[calib] 校准表:", flush=True)
    by_yaw_view = {}
    for (yb, nv, tid), info in sorted(CALIB_TABLE.items()):
        by_yaw_view.setdefault((yb, nv), []).append((tid, info))
    for (yb, nv) in sorted(by_yaw_view.keys()):
        marker = "  ★ 当前 yaw 桶" if yb == nearest_bucket else ""
        yb_str = f"{yb:+.0f}°" if yb is not None else "无"
        print(f"  --- yaw={yb_str} 桶,cam 视野 {nv} 只 ---{marker}",
              flush=True)
        for tid, info in sorted(by_yaw_view[(yb, nv)]):
            reliable = "✓" if info.get("reliable") else "⚠"
            print(f"    板 #{tid}: xc 均值={info['xc_mean']:+.3f} "
                  f"(n_hits={info['n_hits']}) [{reliable}]",
                  flush=True)

    if yaw_dist > 15.0:
        bucket_text = (f"{nearest_bucket:+.0f}°"
                       if nearest_bucket is not None else "无")
        print(f"\n[warn] 当前 yaw={start_yaw_deg:+.2f}° 跟最近校准桶"
              f" {bucket_text} 距离 {yaw_dist:.1f}° > 15°!",
              flush=True)
        print(f"  [WARN] 自动归位已禁用。请手动摆车到校准起点"
              f"(odom x≈+0.05, y≈+0.03, yaw≈-9°)。",
              flush=True)
    elif nearest_bucket is not None:
        print(f"\n[info] 用 yaw={nearest_bucket:+.0f}° 桶的校准 "
              f"(距离 {yaw_dist:.1f}°)", flush=True)
    else:
        print(f"\n[info] 校准表为空或无 yaw 桶 — "
              f"需要先跑 manual_calibrate 校准", flush=True)

    # 询问要射哪些
    if args.targets is None:
        print(f"\n[select] 输入按板上 L→R 顺序索引(空格分隔,如 '1 3 4'):")
        try:
            user_input = input("> ").strip()
        except EOFError:
            user_input = ""
        if not user_input:
            print("  没输入,默认 1..4")
            targets_to_shoot = [1, 2, 3, 4]
        else:
            targets_to_shoot = [int(x) for x in user_input.split()]
    elif args.targets.lower() == "all":
        targets_to_shoot = [1, 2, 3, 4]
    else:
        targets_to_shoot = [int(x) for x in args.targets.split()]

    targets_to_shoot = sorted(set(targets_to_shoot))
    n_total = max(targets_to_shoot)
    print(f"\n[select] 按板上 L→R 顺序索引 = {targets_to_shoot}",
          flush=True)
    print(f"[plan] 总板上板数 = {n_total}, 共要前进 {n_total - 1} 次 8cm",
          flush=True)

    # 检测起点 cam 视野
    print(f"\n[detect] 起点 cam 视野检测...", flush=True)
    animals = get_animals(client, args.min_score)
    if not animals:
        for _ in range(10):
            time.sleep(0.3)
            animals = get_animals(client, args.min_score)
            if animals:
                break
    # **2026-08-03 修**:起点视野 < 4 只时, 自动降 score retry
    # (用户说有 4 只板上, 但 yolo 默认 score=0.30 漏检 #3 #4)
    if animals and len(animals) < n_total and args.min_score > 0.10:
        print(f"  [warn] 起点 cam 视野 {len(animals)} 只 < 期望 {n_total} 只,"
              f" 自动降 score retry 检测更远的板", flush=True)
        for lower_score in [0.20, 0.15, 0.10]:
            if args.min_score <= lower_score:
                continue
            new_score = lower_score
            for _ in range(5):
                time.sleep(0.3)
                new_animals = get_animals(client, new_score)
                if new_animals and len(new_animals) > len(animals):
                    print(f"  [info] 降 score 到 {new_score} 抓到"
                          f" {len(new_animals)} 只板", flush=True)
                    animals = new_animals
                    args.min_score = new_score  # 全局用更低的 score
                    break
            if len(animals) >= n_total:
                break
        if len(animals) < n_total:
            print(f"  ⚠ 起点 cam 视野 {len(animals)} 只 < 期望 {n_total} 只,"
                  f"即使降 score 也检测不全。需要车后退让 cam 看全 4 只,"
                  f"或调整 yolo 模型。", flush=True)
    if not animals:
        print("[err] cam 视野空,退出", file=sys.stderr)
        return 1

    print(f"[detect] 起点 cam 视野 {len(animals)} 只:", flush=True)
    for a in sorted(animals, key=bbox_xc):
        print(f"  xc={bbox_xc(a):+.3f} yc={bbox_yc(a):+.3f} "
              f"wn={bbox_width(a):.3f}", flush=True)
    # **2026-08-03 约定**:车上 cam 视野 N 只 = 板上 L→R #1..#N
    n_view_start = len(animals)
    print(f"  [约定] cam 视野 {n_view_start} 只 = 板上 L→R #1.."
          f"#{n_view_start}", flush=True)
    if any(t > n_view_start for t in targets_to_shoot):
        out_of_view = [t for t in targets_to_shoot if t > n_view_start]
        print(f"  ⚠ 注意:选了 cam 视野外的板号 {out_of_view} "
              f"(cam 视野只到 #{n_view_start})。"
              f"这些板只能靠「直行到下一只板正前方」后重新 detect 命中。",
              flush=True)

    results = []
    hit_count = 0     # 已击倒板上数 (跳过不计数), 用作选板上 #k 的索引信号

    def _drive_one_step(reason):
        """前进 8cm 到下一只板上板正前方, 重新 detect。
        4 只板时 #3 #4 离车远, 5 发循环调 yaw 不够;
        每只板后直行 8cm, 让下一只板靠近车 (cam 视野中央, 射程合适)。
        """
        try:
            car_call(client, "move_for", [0.08, 0.0, 0.0], timeout=5)
            print(f"  ✓ 直行 8cm ({reason})", flush=True)
        except Exception as e:
            print(f"  [drive err] {e}", file=sys.stderr)
        time.sleep(0.3)
        new_animals = get_animals_retry(client, args.min_score,
                                        label=f"after_drive",
                                        retries=3, delay=0.1)
        if not new_animals:
            for _ in range(5):
                time.sleep(0.3)
                new_animals = get_animals(client, args.min_score)
                if new_animals:
                    break
        return new_animals or []

    print(f"\n[plan] 选板 = 车上视野 L→R 第 1 只非 banned 板 = "
          f"板上当前最近板", flush=True)
    print(f"[plan] 每只板 (选/跳过/打) 后直行 8cm, "
          f"让下一只板靠近车", flush=True)

    # **2026-08-03 v3 主循环**:按板上 L→R 顺序遍历 n_total 只板上板
    # 每只板上板:
    #   1. 选车上视野 L→R 第 1 只非 banned 板
    #   2. 如果是 targets_to_shoot 里的序号 → 5 发循环打
    #   3. 否则 → 跳过(不射), 也加 banned(防止下一轮选到)
    #   4. 击倒/跳过/放弃都前进 8cm 到下一只板上板(除非是最后一只)
    animals_now = animals
    animals_start = animals  # 起点视野快照, 用于自计算校准
    for shot_seq in range(1, n_total + 1):
        is_in_targets = shot_seq in targets_to_shoot
        is_last = (shot_seq == n_total)

        print(f"\n========== 板上第 {shot_seq}/{n_total} 只 "
              f"{'(要射)' if is_in_targets else '(跳过)'} ==========",
              flush=True)

        # 选板上 #shot_seq = 车上视野 L→R 第 (shot_seq - hit_count) 只
        # (跳过时不增加 hit_count, 因为板上 #k 仍在场上合法)
        target, cur_xc = pick_target_by_shot_seq(
            animals_now, shot_seq, hit_count)
        if target is None:
            # **2026-08-03 大改**:车上视野里板上 #shot_seq 不在 (板上 #k 在更远位置)
            # 调距离让板上 #shot_seq 进入视野 (5 发循环里也有同样逻辑)
            print(f"  [warn] 车上视野 {len(animals_now)} 只,"
                  f" 选不到板上第 {shot_seq} 只"
                  f" (车上视野 L→R 索引 {(shot_seq - hit_count)} 越界),"
                  f" 直行 8cm 让板上 #shot_seq 进入视野",
                  flush=True)
            if not is_last:
                animals_now = _drive_one_step("选不到板上 #shot_seq")
            continue

        if not is_in_targets:
            # **2026-08-03 大改**:跳过时不增加 hit_count (板上 #k 仍在场上合法)
            # 跳过不影响 hit_count, 因为 hit_count 只在击倒时 +1
            print(f"  [skip] 板上第 {shot_seq} 只 xc={cur_xc:+.3f}"
                  f" (用户选不射)", flush=True)
            if not is_last:
                animals_now = _drive_one_step(f"跳过 #{shot_seq}")
            continue

        # 要射 → 5 发循环
        print(f"  [shoot] 板上第 {shot_seq} 只 → 5 发循环 xc={cur_xc:+.3f}",
              flush=True)

        # 校准 xc 查表 (按板上 L→R 顺序 shot_seq)
        n_view = len(animals_now)
        calib_xc, calib_src = get_calib_xc(nearest_bucket, n_view,
                                           shot_seq, CALIB_TABLE)
        if calib_xc is None:
            # **2026-08-03 自计算校准**:无校准时, 用 cam 视野里看到的
            # 板上 #shot_seq 实际 xc (前提: 车上视野 L→R 第 (shot_seq - banned_count) 只
            # = 板上 #shot_seq, 但车上视野 L→R 索引 ≠ 板上 L→R 索引因为 banned)
            # 简化: 用起点视野 (animals_start) 里板上 #shot_seq 的 xc
            # (前提: 起点 cam 视野能看到板上 #shot_seq)
            # 注: animals_start 是 main 里"起点 cam 视野"
            if shot_seq <= len(animals_start):
                sorted_start = sorted(animals_start, key=bbox_xc)
                # 起点视野 L→R 第 k 只 = 板上 #k (无 banned)
                # 如果起点视野里有 banned 板 (从之前的 shot_seq), 要扣掉
                # 但起点视野是 main 开始时 detect 的, banned 是空的
                calib_xc = bbox_xc(sorted_start[shot_seq - 1])
                calib_src = (f"自计算: 起点视野 L→R 第 {shot_seq} 只"
                             f" xc={calib_xc:+.3f}")
                print(f"  ⚠ 板上第 {shot_seq} 只无校准,"
                      f"自计算用 {calib_src}", flush=True)
            else:
                calib_xc = 0.5
                calib_src = "无校准,起点视野也看不到,用 cam 中央"
                print(f"  ⚠ 板上第 {shot_seq} 只无校准,起点视野也看不到,"
                      f"用 cam 中央 0.5", flush=True)
        err = cur_xc - calib_xc
        print(f"  [detect] 选中 xc={cur_xc:+.3f} (校准 {calib_xc:+.3f} "
              f"{calib_src}, err={err:+.3f})", flush=True)
        if abs(err) > HIT_XC_TOL:
            print(f"  ⚠ cam xc 偏离校准值 {err:.3f} > "
                  f"{HIT_XC_TOL},需要 yaw 微调", flush=True)

        # **2026-08-03 用户实测**:#3 #4 离车远 + yolo 漏检, 5 发不够
        # 改成 15 发 (3 轮 5 发), 每轮用完调距离 8cm 重试
        # 直到命中或 15 发用完
        MAX_SHOTS = 15
        SHOTS_PER_ROUND = 5
        MAX_TOTAL_YAW = 30.0
        yaw_total = [0.0]
        hit_this_target = False
        attempts = 0

        def _yaw_budget_ok(delta_deg):
            return abs(yaw_total[0] + delta_deg) <= MAX_TOTAL_YAW

        def _yaw_consume(delta_deg):
            yaw_total[0] += delta_deg

        # 5 发循环锁 first_xc
        first_xc = cur_xc
        LOCK_XC_TOL = 0.20

        # **2026-08-03 改**:5 发循环内允许调距离 (±2cm),
        # 让板上 #3 #4 离车更近 (射程合适)
        # 不 abandon: 持续微调直至命中或 5 发用完

        for shot_i in range(1, MAX_SHOTS + 1):
            attempts = shot_i
            if shot_i == 1 and shot_seq == 1:
                animals_shot = animals_now
            else:
                animals_shot = get_animals_retry(client, args.min_score,
                                                label=f"a{shot_i} re-detect",
                                                retries=2, delay=0.1)
            if not animals_shot:
                # 视野空 → 调距离 ±4cm 让板上 #3 #4 进入视野
                # (车上视野里板上 #1 #2 已 banned, 但 #1 #2 仍可能被 yolo 检测到)
                d_adj = +0.04 if shot_i % 2 == 1 else -0.04
                direction = "forward" if d_adj > 0 else "backward"
                print(f"  [a{shot_i}] 视野空,调距离 {direction} 4cm"
                      f" 让板上 #3 #4 进入视野", flush=True)
                try:
                    car_call(client, "move_for", [d_adj, 0.0, 0.0], timeout=3)
                except Exception as e:
                    print(f"  [a{shot_i} dist err] {e}", file=sys.stderr)
                time.sleep(0.3)
                continue

            # 5 发循环锁 first_xc, 内无板时 fall back
            target, cur_xc = pick_target_by_xc(
                animals_shot, first_xc, xc_tol=LOCK_XC_TOL)
            if target is None:
                # fall back 到板上 #shot_seq = 车上视野 L→R 第 (shot_seq - hit_count) 只
                target_fb, cur_xc_fb = pick_target_by_shot_seq(
                    animals_shot, shot_seq, hit_count)
                if target_fb is not None:
                    print(f"  [a{shot_i}] first_xc={first_xc:+.3f} ±"
                          f" {LOCK_XC_TOL} 内无板,fall back 到板上 #{shot_seq}"
                          f" (车上视野 L→R 第 {shot_seq - hit_count} 只)"
                          f" xc={cur_xc_fb:+.3f}",
                          flush=True)
                    target = target_fb
                    cur_xc = cur_xc_fb
                else:
                    # 车上视野里没有板上 #shot_seq → 调距离 ±4cm 让它进入视野
                    d_adj = +0.04 if shot_i % 2 == 1 else -0.04
                    direction = "forward" if d_adj > 0 else "backward"
                    print(f"  [a{shot_i}] 车上视野里没有板上 #{shot_seq},"
                          f"调距离 {direction} 4cm 让它进入视野",
                          flush=True)
                    try:
                        car_call(client, "move_for",
                                 [d_adj, 0.0, 0.0], timeout=3)
                    except Exception as e:
                        print(f"  [a{shot_i} dist err] {e}",
                              file=sys.stderr)
                    time.sleep(0.3)
                    continue
            err = cur_xc - calib_xc
            print(f"  [a{shot_i}] 选中 xc={cur_xc:+.3f} "
                  f"(校准 {calib_xc:+.3f}, err={err:+.3f})", flush=True)

            try:
                car_call(client, "shooting", timeout=5)
                print(f"  [a{shot_i}] 已射", flush=True)
            except Exception as e:
                print(f"  [a{shot_i} shoot err] {e}", file=sys.stderr)
            time.sleep(0.3)

            # 命中判定
            animals_after = get_animals_retry(client, args.min_score,
                                              label=f"a{shot_i} after",
                                              retries=4, delay=0.2)
            if not animals_after:
                animals_after2 = get_animals_retry(client, args.min_score,
                                                   label=f"a{shot_i} after2",
                                                   retries=3, delay=0.2)
                if not animals_after2:
                    print(f"  [a{shot_i}] 命中(连续 7 次视野空)",
                          flush=True)
                    hit_this_target = True
                    break
                else:
                    animals_after = animals_after2
            nearby = [a for a in animals_after
                      if abs(bbox_xc(a) - cur_xc) < 0.20]
            if not nearby:
                animals_after2 = get_animals_retry(client, args.min_score,
                                                   label=f"a{shot_i} near2",
                                                   retries=3, delay=0.2)
                nearby2 = ([a for a in animals_after2
                            if abs(bbox_xc(a) - cur_xc) < 0.20]
                           if animals_after2 else [])
                if not nearby2:
                    print(f"  [a{shot_i}] 命中(连续 2 次 xc {cur_xc:+.3f}"
                          f" ± 0.20 空)", flush=True)
                    hit_this_target = True
                    break
                else:
                    animals_after = animals_after2

            # 未命中 → 只调 yaw
            YAW_STEP_CAP_DEG = 3.0
            K_YAW = 20.0
            yaw_adj = -err * K_YAW * YAW_SIGN
            yaw_adj = max(-YAW_STEP_CAP_DEG, min(YAW_STEP_CAP_DEG, yaw_adj))
            if abs(yaw_adj) >= 0.3 and _yaw_budget_ok(yaw_adj):
                try:
                    car_call(client, "move_for",
                             [0.0, 0.0, math.radians(yaw_adj)], timeout=3)
                    _yaw_consume(yaw_adj)
                    print(f"  [a{shot_i} miss→yaw] err={err:+.3f} → "
                          f"yaw {yaw_adj:+.2f}° (累计 {yaw_total[0]:+.2f}°)",
                          flush=True)
                except Exception as e:
                    print(f"  [a{shot_i} yaw err] {e}", file=sys.stderr)
            else:
                d_adj = +0.02 if err > 0 else -0.02
                direction = "forward" if d_adj > 0 else "backward"
                print(f"  [a{shot_i} miss→dist] err={err:+.3f} → "
                      f"{direction} 2cm (calib {calib_xc:+.3f})",
                      flush=True)
                try:
                    car_call(client, "move_for", [d_adj, 0.0, 0.0], timeout=3)
                except Exception as e:
                    print(f"  [a{shot_i} dist err] {e}", file=sys.stderr)
            time.sleep(0.2)

            # **2026-08-03 改**:每 5 发循环用完 → 调大距离 8cm 重试,
            # 让板上 #3 #4 进入视野. 总共 15 发 (3 轮 5 发).
            # 第 2 轮 forward 8cm, 第 3 轮 backward 8cm
            if shot_i % SHOTS_PER_ROUND == 0 and shot_i < MAX_SHOTS \
                    and not hit_this_target:
                if shot_i // SHOTS_PER_ROUND == 1:
                    big_d = +0.08
                    direction = "forward"
                else:
                    big_d = -0.08
                    direction = "backward"
                print(f"  [round-{shot_i // SHOTS_PER_ROUND}]"
                      f" 5 发用完未命中,{direction} 8cm"
                      f" 让板上 #3 #4 进入视野", flush=True)
                try:
                    car_call(client, "move_for",
                             [big_d, 0.0, 0.0], timeout=5)
                except Exception as e:
                    print(f"  [round dist err] {e}", file=sys.stderr)
                # 重置 yaw budget (距离调了, yaw 也得重调)
                if abs(yaw_total[0]) > 0.5:
                    try:
                        car_call(client, "move_for",
                                 [0.0, 0.0, math.radians(-yaw_total[0])],
                                 timeout=5)
                    except Exception:
                        pass
                    yaw_total[0] = 0.0
                time.sleep(0.3)

        # 5 发循环 yaw 归位
        if abs(yaw_total[0]) > 0.5:
            print(f"  [yaw-reset] yaw 累计 {yaw_total[0]:+.2f}°,反向归位...",
                  flush=True)
            try:
                car_call(client, "move_for",
                         [0.0, 0.0, math.radians(-yaw_total[0])],
                         timeout=5)
                print(f"  [yaw-reset] 归位 -({yaw_total[0]:+.2f}°) 完成",
                      flush=True)
            except Exception as e:
                print(f"  [yaw-reset err] {e}", file=sys.stderr)
            time.sleep(0.2)

        # 加入 hit_count (击倒才计数, 跳过不算)
        if hit_this_target:
            results.append({"tid": shot_seq, "hit": True, "attempts": attempts})
            hit_count += 1
            print(f"  [hit] 板上第 {shot_seq} 只命中 "
                  f"(hit_count={hit_count})", flush=True)
        else:
            print(f"  [abandon] {MAX_SHOTS} 发用完仍未命中板上第 {shot_seq} 只",
                  flush=True)
            results.append({"tid": shot_seq, "hit": False, "attempts": attempts})
            # abandon 不增加 hit_count (板上 #k 仍未被打掉, 仍在场上合法)

        # **2026-08-03 新策略**:不论打中/放弃,直行 8cm 到下一只板上板
        if not is_last:
            animals_now = _drive_one_step(
                f"板上 #{shot_seq} 击倒/放弃后")
        else:
            print(f"\n  [done] 板上第 {shot_seq} 只 = 最后一只,不再前进",
                  flush=True)

    # 归位(沿 -x 后退到起点)
    cur = read_odom(client)
    if cur is not None:
        dx = sx - cur[0]
        print(f"\n[return] 沿 -x 归位 {abs(dx)*100:.1f}cm", flush=True)
        if abs(dx) > 0.02:
            try:
                car_call(client, "move_for", [dx, 0.0, 0.0],
                         timeout=max(5, abs(dx) * 20))
            except Exception as e:
                print(f"  [return err] {e}", file=sys.stderr)

    # 汇总
    print(f"\n{'='*60}", flush=True)
    print(f"========== 射击汇总 ==========", flush=True)
    print(f"{'='*60}", flush=True)
    n_hit = sum(1 for r in results if r["hit"])
    print(f"  命中 {n_hit}/{len(results)}")
    for r in results:
        status = "✓" if r["hit"] else "✗"
        print(f"  板上第 {r['tid']} 只 {status} attempts={r['attempts']}",
              flush=True)
    return 0 if n_hit == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
