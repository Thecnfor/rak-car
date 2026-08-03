#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/manual_calibrate.py - 手动射击校准脚本

**目的**:在机械臂固定位置(arm_seq_v9 --y1 -0.150 --x -0.200 --arm-angle 90 --hand-angle -90)
下,手动校准「cam 视野哪个 xc 位置能命中哪块目标」。

**工作流**(用户手动):
1. 摆车到任意位置
2. 程序循环:
   - 显示当前 cam 视野 xc 列表 + 车 odom
   - 用户输入命令:
     - `s`  → 射 1 发(记录射前 cam 状态 + 车 odom)
     - `hit N`  → 报告第 N 块板被打中(1-based)
     - `miss`   → 报告本次射击未中
     - `q`      → 退出并打印校准表
3. 程序输出 JSON 校准表 + 每个板的命中区间

**输出**:
- 控制台打印:每块板的 (xc_min, xc_max) 命中区间
- 文件 `manual_calibrate_result.json`:完整校准数据

用法:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.manual_calibrate
"""
from __future__ import annotations

import json
import math
import sys
import time
from datetime import datetime

from main.api_client import RuntimeApiClient

# 复用 shoot_4_targets 的核心工具
from main.tasks.task333.shoot_4_targets import (
    bbox_xc,
    bbox_yc,
    bbox_width,
    MIN_YOLO_SCORE,
    get_animals,
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


def print_cam_state(client):
    """打印 cam 视野 + odom。返回 (animals, odo) 供 caller 复用,避免重复 ZMQ。"""
    animals = get_animals(client, MIN_YOLO_SCORE)
    odo = read_odom(client)
    odo_str = (f"odom=(x={odo[0]:+.3f},y={odo[1]:+.3f},"
               f"yaw={math.degrees(odo[2]):+.2f}°)") if odo else "odom=None"
    print(f"  [{odo_str}]", flush=True)
    if animals:
        sorted_animals = sorted(animals, key=bbox_xc)
        for i, a in enumerate(sorted_animals):
            xc = bbox_xc(a)
            yc = bbox_yc(a)
            wn = bbox_width(a)
            score = a.get("score", 0.0)
            print(f"    #{i+1}: xc={xc:+.3f} yc={yc:.3f} "
                  f"wn={wn:.3f} score={score:.3f}", flush=True)
    else:
        print("    cam 视野空", flush=True)
    return animals, odo


def main():
    client = RuntimeApiClient()
    client.wait_until_ready()
    print("[ready] 等待手动操作", flush=True)

    # 校准数据
    shots = []   # 每次射击的完整记录
    n_shots = 0

    print("\n========== 手动校准 ==========", flush=True)
    print("命令:", flush=True)
    print("  s        - 射 1 发(自动记录 cam 状态)", flush=True)
    print("  hit N    - 报告刚才那发命中了第 N 块板(1-based)", flush=True)
    print("  miss     - 报告刚才那发没中", flush=True)
    print("  q        - 退出 + 打印校准表", flush=True)
    print("=============================", flush=True)

    last_shot_time = None   # 上一次射的时间(等待用户报告 hit/miss)

    while True:
        try:
            cmd = input("\n> ").strip().lower()
        except EOFError:
            cmd = "q"

        if cmd == "q" or cmd == "quit" or cmd == "exit":
            break

        if cmd == "s" or cmd == "shoot":
            # 记录射前 cam 状态
            animals, odo = print_cam_state(client)
            shot_state = {
                "shot_id": n_shots + 1,
                "timestamp": datetime.now().isoformat(),
                "odom": {
                    "x": odo[0] if odo else 0,
                    "y": odo[1] if odo else 0,
                    "yaw_deg": math.degrees(odo[2]) if odo else 0,
                },
                "cam_view": [
                    {
                        "xc": bbox_xc(a),
                        "yc": bbox_yc(a),
                        "wn": bbox_width(a),
                        "score": a.get("score", 0.0),
                    }
                    for a in sorted(animals, key=bbox_xc)
                ],
                "result": None,   # 等待用户填
            }
            # 射 1 发
            try:
                car_call(client, "shooting", timeout=5)
                print("  ✓ 已射 1 发(等待你报告 hit/miss)", flush=True)
                n_shots += 1
                shot_state["shot_id"] = n_shots
                shots.append(shot_state)
                last_shot_time = time.time()
            except Exception as e:
                print(f"  [shoot err] {e}", file=sys.stderr)
            continue

        if cmd == "miss":
            if not shots:
                print("  还没有射击记录", flush=True)
                continue
            shots[-1]["result"] = {"hit": False, "target_id": None}
            print(f"  记录:shot #{shots[-1]['shot_id']} miss", flush=True)
            continue

        if cmd.startswith("hit"):
            # hit N [xc=X.XXX]
            parts = cmd.split()
            if len(parts) < 2 or len(parts) > 3:
                print("  用法:hit N [xc=X.XXX]"
                      "(N 是 1-based 板编号,xc 可选精确 cam 位置)",
                      flush=True)
                continue
            try:
                n = int(parts[1])
            except ValueError:
                print(f"  解析失败:{parts[1]}", flush=True)
                continue
            # 可选:xc=X.XXX(精确报告 cam 视野里被命中板的 xc)
            hit_xc = None
            if len(parts) == 3:
                xc_str = parts[2]
                if xc_str.startswith("xc="):
                    try:
                        hit_xc = float(xc_str[3:])
                    except ValueError:
                        print(f"  xc 解析失败:{xc_str}", flush=True)
                        continue
            if not shots:
                print("  还没有射击记录", flush=True)
                continue
            # **2026-08-02 改进**:如果用户没报 xc,自动从 cam_view 选
            # 「x 最接近 cam 中央」的 detection xc 作为该板命中位置
            # (这是「我射中的那只板的 cam xc」)
            if hit_xc is None:
                cam_view = shots[-1]["cam_view"]
                if cam_view:
                    # 选 cam_view 里 xc 接近 0.5 的(detection 中线)
                    hit_xc = min(cam_view,
                                 key=lambda d: abs(d["xc"] - 0.5))["xc"]
                    print(f"  (自动从 cam_view 选 xc={hit_xc:+.3f})",
                          flush=True)
                else:
                    # **2026-08-02 修复**:cam_view 空时不报 nan,
                    # 标记 hit_xc=None,后续 calib 统计会跳过
                    hit_xc = None
                    print(f"  ⚠ cam 视野空,无法从 cam_view 自动选 xc。"
                          f"建议用 `hit N xc=X.XXX` 手动指定",
                          flush=True)
            shots[-1]["result"] = {"hit": True, "target_id": n,
                                   "hit_xc": hit_xc}
            print(f"  记录:shot #{shots[-1]['shot_id']} hit #{n} "
                  f"@ xc={hit_xc:+.3f}", flush=True)
            continue

        if cmd == "state" or cmd == "view":
            # 仅打印 cam 视野,不射
            _, _ = print_cam_state(client)
            continue

        if cmd == "recon" or cmd == "recon calib":
            # 重新跑一遍「校准报告」 — 用于退出前先看汇总
            print("\n[临时汇总]", flush=True)
            n_with_hit = sum(1 for s in shots
                             if s.get("result") and s["result"]["hit"])
            print(f"  总射击:{len(shots)},已报告命中:{n_with_hit}",
                  flush=True)
            for s in shots:
                if s.get("result") and s["result"]["hit"]:
                    tid = s["result"]["target_id"]
                    hxc = s["result"].get("hit_xc", float("nan"))
                    n_view = len(s["cam_view"])
                    print(f"  shot #{s['shot_id']} hit #{tid} "
                          f"@ xc={hxc:+.3f} (cam 视野 {n_view} 只)",
                          flush=True)
            continue

        # 未知命令
        print(f"  未知命令:{cmd}(可用 s/hit N/miss/q)", flush=True)

    # 退出:汇总 + 输出
    print(f"\n{'='*60}", flush=True)
    print(f"========== 校准汇总 ==========", flush=True)
    print(f"{'='*60}", flush=True)
    print(f"  总射击: {n_shots} 发", flush=True)

    hit_shots = [s for s in shots if s["result"] and s["result"]["hit"]]
    miss_shots = [s for s in shots if s["result"] and not s["result"]["hit"]]
    pending = [s for s in shots if not s["result"]]
    print(f"  命中: {len(hit_shots)} 发", flush=True)
    print(f"  未中: {len(miss_shots)} 发", flush=True)
    if pending:
        print(f"  ⚠ 未报告结果: {len(pending)} 发(标记为 miss)", flush=True)
        for s in pending:
            s["result"] = {"hit": False, "target_id": None}

    # 按 (yaw_bucket, cam_view_count, target_id) 分组,统计每块板的命中 cam xc
    # **2026-08-02 用户实测**:校准和实际任务的初始 odom(尤其是 yaw)不同,
    # 导致校准数据失效。校准表按「yaw 容差 5° 分桶」分组,自动射击脚本
    # 据此挑最接近当前 yaw 的桶。
    print(f"\n[calib] 每块板的命中区间(按 yaw 桶 + cam 视野板数分组):",
          flush=True)
    YAW_BUCKET_DEG = 5.0   # 每个桶宽度

    def yaw_bucket(yaw_deg):
        # 把 yaw 度数量化到 ±180° 区间,然后量化到桶
        y = yaw_deg % 360
        if y > 180:
            y -= 360
        elif y < -180:
            y += 360
        bucket = int(y / YAW_BUCKET_DEG) * YAW_BUCKET_DEG
        return bucket

    # {(yaw_bucket, n_view, tid): [(xc, odom), ...]}
    target_hits = {}
    skipped_no_xc = 0
    for s in shots:
        if s["result"] and s["result"]["hit"]:
            tid = s["result"]["target_id"]
            hit_xc = s["result"].get("hit_xc")
            if hit_xc is None:
                skipped_no_xc += 1
                continue
            n_view = len(s["cam_view"])
            yaw = s.get("odom", {}).get("yaw_deg", 0.0)
            bucket = yaw_bucket(yaw)
            key = (bucket, n_view, tid)
            if key not in target_hits:
                target_hits[key] = []
            target_hits[key].append({
                "xc": hit_xc,
                "yaw_deg": yaw,
                "odom_x": s.get("odom", {}).get("x", 0.0),
                "odom_y": s.get("odom", {}).get("y", 0.0),
            })
    if skipped_no_xc > 0:
        print(f"  ⚠ 跳过 {skipped_no_xc} 条无 xc 的命中记录"
              f"(cam 视野空时用户未手动报 xc)", flush=True)

    calib_table = {}
    RELIABLE_THRESHOLD = 3
    for (bucket, n_view, tid), samples in sorted(target_hits.items()):
        xcs = [s["xc"] for s in samples]
        if xcs:
            xc_min = min(xcs)
            xc_max = max(xcs)
            xc_mean = sum(xcs) / len(xcs)
            reliable = "✓" if len(xcs) >= RELIABLE_THRESHOLD else "⚠样本不足"
            print(f"  yaw={bucket:+.0f}° 桶,cam 视野 {n_view} 只,板 #{tid}: "
                  f"{len(xcs)} 次命中 [{reliable}],"
                  f"xc ∈ [{xc_min:+.3f}, {xc_max:+.3f}],"
                  f"均值={xc_mean:+.3f}", flush=True)
            calib_table[f"yaw{bucket:.0f}_view{n_view}_target{tid}"] = {
                "yaw_bucket_deg": bucket,
                "yaw_bucket_width_deg": YAW_BUCKET_DEG,
                "cam_view_count": n_view,
                "n_hits": len(xcs),
                "reliable": len(xcs) >= RELIABLE_THRESHOLD,
                "xc_min": xc_min,
                "xc_max": xc_max,
                "xc_mean": xc_mean,
                "xc_samples": xcs,
                "yaw_samples": [s["yaw_deg"] for s in samples],
                "odom_x_samples": [s["odom_x"] for s in samples],
                "odom_y_samples": [s["odom_y"] for s in samples],
            }

    # 保存 JSON
    out_path = r"C:\Users\花花世界\Desktop\天道酬勤\rak-car\manual_calibrate_result.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump({
            "arm_seq_v9_position": {
                "y_m": -0.150, "x_m": -0.200,
                "arm_angle_deg": 90, "hand_angle_deg": -90,
            },
            "n_shots": n_shots,
            "calib_table": calib_table,
            "all_shots": shots,
        }, f, indent=2, ensure_ascii=False)
    print(f"\n[calib] 完整数据已存到 {out_path}", flush=True)

    # 按 xc 排序的「哪些 xc 区间能命中哪些板」总表(按 yaw 桶 + cam 视野板数分块)
    print(f"\n[calib] xc → 板号映射(按 yaw 桶 + cam 视野板数分块):",
          flush=True)
    by_yaw_view = {}
    for name, info in calib_table.items():
        yaw_b = info["yaw_bucket_deg"]
        n_view = info["cam_view_count"]
        key = (yaw_b, n_view)
        by_yaw_view.setdefault(key, []).append((name, info))
    for (yaw_b, n_view) in sorted(by_yaw_view.keys()):
        print(f"\n  --- yaw={yaw_b:+.0f}° 桶,cam 视野 {n_view} 只 ---",
              flush=True)
        sorted_targets = sorted(by_yaw_view[(yaw_b, n_view)],
                                key=lambda kv: kv[1].get("xc_mean", 0))
        for name, info in sorted_targets:
            if info.get("n_hits", 0) > 0:
                tid = name.split("target")[1]
                print(f"    xc ∈ [{info['xc_min']:+.3f}, "
                      f"{info['xc_max']:+.3f}] → 板 #{tid} "
                      f"({info['n_hits']} 次命中)",
                      flush=True)

    print(f"\n[done] 校准完成", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())