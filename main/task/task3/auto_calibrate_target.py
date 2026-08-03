#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/auto_calibrate_target.py - 半自动补校准脚本(方案 A)

**目的**:针对 `manual_calibrate_result.json` 里 n_hits 不足 3 的组,
脚本按「目标板号顺序」自动射击,每只板射 1 发,**不调车不调 yaw**:

1. 起点:用户手动摆车到 odom ≈ (x=+0.05, y=+0.03, yaw=-9°)
2. 检测 cam 视野,确认能看到目标板 → 射 1 发
3. 用户报 `hit N` / `miss` / `skip`
4. 直行 8cm 到下一只板位置 → 重复

**为什么不调车 / 不调 yaw**:
- 用户硬约束:**完全禁 yaw / lateral**(2026-08-02)
- 板间距固定 8cm → 直行 8cm = 走到下一只板正前方
- cam 视角固定 → 直行不改变 cam xc(只改 wn),不需要 yaw 微调
- 期望 odom 推算容易跟实际漂移,导致脚本一直 drive_to 失败 → 一直跑(2026-08-03 bug 修复)

**为什么不完全自动化命中判定**:
- 板子倒下目测 vs yolo 假阴性(板还立着但 cam 没看到)需要人工核验

**预设要补的 14 发**(2026-08-03 用户补校准):
   起点 odom ≈ (x=+0.05, y=+0.03, yaw=-9°)
   任务:把 -5° 桶 和 -15° 桶 缺的板全部补到 n_hits ≥ 3

**用法**:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    PYTHONIOENCODING=utf-8 python -m main.tasks.task333.auto_calibrate_target
    # 不真射(只摆车 + 报 hit):
    PYTHONIOENCODING=utf-8 python -m main.tasks.task333.auto_calibrate_target --dry-run
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from datetime import datetime

from main.api_client import RuntimeApiClient
from main.tasks.task333.shoot_4_targets import (
    bbox_xc,
    bbox_yc,
    bbox_width,
    MIN_YOLO_SCORE,
    get_animals,
    get_animals_retry,
)
from main.tasks.task333.manual_calibrate import car_call, read_odom


CALIB_JSON_PATH = r"C:\Users\花花世界\Desktop\天道酬勤\rak-car\manual_calibrate_result.json"
YAW_BUCKET_DEG = 5.0
BOARD_SPACING_M = 0.08
HIT_XC_TOL = 0.40
BOARD_WIDTH_CM = 8.0
# 校准初始 odom(用户实测基准)
START_ODOM_X = 0.05
START_ODOM_Y = 0.03
START_ODOM_YAW_DEG = -9.0


def _yaw_bucket(yaw_deg):
    y = yaw_deg % 360
    if y > 180:
        y -= 360
    elif y < -180:
        y += 360
    return int(y / YAW_BUCKET_DEG) * YAW_BUCKET_DEG


def load_calib_table(json_path=CALIB_JSON_PATH):
    """读现有校准表,返回 {(yaw_bucket, n_view, tid): info}。"""
    if not os.path.exists(json_path):
        print(f"[err] {json_path} 不存在,请先跑 manual_calibrate",
              file=sys.stderr)
        return {}
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    out = {}
    for name, info in data.get("calib_table", {}).items():
        try:
            parts = name.split("_")
            yaw_b = int(parts[0].replace("yaw", ""))
            n_view = int(parts[1].replace("view", ""))
            tid = int(parts[2].replace("target", ""))
            out[(yaw_b, n_view, tid)] = info
        except (IndexError, ValueError):
            continue
    return out


def read_state(client):
    """读 cam 视野 + odom。"""
    animals = get_animals(client, MIN_YOLO_SCORE)
    odo = read_odom(client)
    return animals, odo


def print_state(client, label=""):
    animals, odo = read_state(client)
    if odo:
        yaw_deg = math.degrees(odo[2])
        print(f"[{label}] odom=(x={odo[0]:+.3f}, y={odo[1]:+.3f}, "
              f"yaw={yaw_deg:+.2f}° 桶={_yaw_bucket(yaw_deg):+.0f}°)",
              flush=True)
    else:
        print(f"[{label}] odom=None", flush=True)
    if animals:
        for i, a in enumerate(sorted(animals, key=bbox_xc)):
            xc = bbox_xc(a)
            yc = bbox_yc(a)
            wn = bbox_width(a)
            score = a.get("score", 0.0)
            print(f"  #{i+1}: xc={xc:+.3f} yc={yc:.3f} wn={wn:.3f} "
                  f"score={score:.3f}", flush=True)
    else:
        print("  cam 视野空", flush=True)
    return animals, odo


def record_shot(shots, odo, animals, hit_tid=None, hit_xc=None):
    """把这一发写入 shots 列表。"""
    shot = {
        "shot_id": len(shots) + 1,
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
        "result": {
            "hit": hit_tid is not None,
            "target_id": hit_tid,
            "hit_xc": hit_xc,
        } if hit_tid is not None else {"hit": False, "target_id": None},
    }
    shots.append(shot)


def save_calib_json(shots, calib_json_path=CALIB_JSON_PATH):
    """把补的发追加到现有 calib JSON,重新生成 calib_table。

    **简化版**:只更新 all_shots 列表 + 重新统计 calib_table。
    原 calib_table 字段全部保留,但 n_hits / xc_min/max/mean / yaw_samples /
    odom_samples 重新计算。
    """
    if os.path.exists(calib_json_path):
        with open(calib_json_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    else:
        data = {
            "arm_seq_v9_position": {
                "y_m": -0.15, "x_m": -0.2,
                "arm_angle_deg": 90, "hand_angle_deg": -90,
            },
            "n_shots": 0,
            "calib_table": {},
            "all_shots": [],
        }

    data["all_shots"].extend(shots)
    data["n_shots"] = len(data["all_shots"])

    RELIABLE_THRESHOLD = 3
    target_hits = {}
    skipped_no_xc = 0
    for s in data["all_shots"]:
        if s.get("result") and s["result"].get("hit"):
            tid = s["result"]["target_id"]
            hit_xc = s["result"].get("hit_xc")
            if hit_xc is None:
                skipped_no_xc += 1
                continue
            n_view = len(s.get("cam_view", []))
            yaw = s.get("odom", {}).get("yaw_deg", 0.0)
            bucket = _yaw_bucket(yaw)
            key = (bucket, n_view, tid)
            target_hits.setdefault(key, []).append({
                "xc": hit_xc,
                "yaw_deg": yaw,
                "odom_x": s.get("odom", {}).get("x", 0.0),
                "odom_y": s.get("odom", {}).get("y", 0.0),
            })

    calib_table = {}
    for (bucket, n_view, tid), samples in sorted(target_hits.items()):
        xcs = [s["xc"] for s in samples]
        if not xcs:
            continue
        xc_min = min(xcs)
        xc_max = max(xcs)
        xc_mean = sum(xcs) / len(xcs)
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

    data["calib_table"] = calib_table
    with open(calib_json_path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"[calib] 已写入 {calib_json_path} ({len(shots)} 发新,"
          f"总 {len(data['all_shots'])} 发)", flush=True)
    if skipped_no_xc > 0:
        print(f"  ⚠ 跳过 {skipped_no_xc} 条 cam 视野空且未手动报 xc 的命中",
              flush=True)


def drive_forward(client, distance_m, timeout=None):
    """沿车头方向直行 distance_m 米(可正可负)。"""
    if abs(distance_m) < 0.01:
        print(f"  [drive] 距离 < 1cm,跳过", flush=True)
        return True
    if timeout is None:
        timeout = max(5, abs(distance_m) * 20)
    direction = "前" if distance_m > 0 else "后"
    print(f"  [drive] 沿车头 {direction} {abs(distance_m):.3f}m "
          f"(timeout={timeout:.0f}s)", flush=True)
    try:
        car_call(client, "move_for", [distance_m, 0.0, 0.0], timeout=timeout)
        time.sleep(0.3)
        return True
    except Exception as e:
        print(f"  [drive err] {e}", file=sys.stderr)
        return False


def shoot_and_ask(animals, dry_run=False):
    """射 1 发(除非 dry_run),让用户报 hit/miss/skip。

    Returns:
        (hit_tid: int | None, hit_xc: float | None, skip: bool)
    """
    if dry_run:
        print(f"  [dry-run] 跳过射击", flush=True)
        return None, None, False

    print(f"  >>> 射击 (cam 视野 {len(animals)} 只)...", flush=True)
    try:
        car_call(None and None or None, "shooting", timeout=5) if False else None
    except Exception as e:
        print(f"  [shoot err] {e}", file=sys.stderr)
    # 实际射击调用(独立 try/except)
    try:
        from main.tasks.task333.manual_calibrate import car_call as _car_call
        _car_call(None, "shooting", timeout=5) if False else None
    except Exception:
        pass
    # 直接调车
    try:
        from main.tasks.task333.manual_calibrate import car_call as __cc
        import inspect
        # 拿到 caller 的 client(用 inspect 抓栈)
        frame = inspect.currentframe().f_back
        client = frame.f_locals.get("client")
        if client is None:
            raise RuntimeError("can't find client in caller frame")
        __cc(client, "shooting", timeout=5)
        print(f"  ✓ 已射 1 发", flush=True)
    except Exception as e:
        print(f"  [shoot err] {e}", file=sys.stderr)
        return None, None, False
    time.sleep(0.5)

    while True:
        try:
            cmd = input("\n  报结果 (hit N [xc=X.XXX] / miss / skip): ").strip().lower()
        except EOFError:
            cmd = "miss"
        parts = cmd.split()
        if not parts:
            continue
        if parts[0] == "miss":
            return None, None, False
        if parts[0] == "skip":
            return None, None, True
        if parts[0] == "hit" and len(parts) >= 2:
            try:
                hit_tid = int(parts[1])
            except ValueError:
                print(f"  解析失败:{parts[1]}", flush=True)
                continue
            hit_xc = None
            if len(parts) >= 3 and parts[2].startswith("xc="):
                try:
                    hit_xc = float(parts[2][3:])
                except ValueError:
                    pass
            # 自动 fallback:从 cam_view 选 L→R 第 hit_tid 只的 xc
            if hit_xc is None:
                cam_view = sorted(animals, key=bbox_xc)
                if cam_view:
                    idx = min(hit_tid - 1, len(cam_view) - 1)
                    hit_xc = bbox_xc(cam_view[idx])
                    print(f"  (auto) 用 cam_view L→R 第 {hit_tid} 只 xc={hit_xc:+.3f}",
                          flush=True)
            return hit_tid, hit_xc, False
        print(f"  未知命令:{cmd}", flush=True)


def main():
    ap = argparse.ArgumentParser(description="半自动补校准脚本(方案 A:按板号顺序,不调车不调 yaw)")
    ap.add_argument("--dry-run", action="store_true",
                    help="只摆车 + 报 hit,不真射子弹")
    args = ap.parse_args()

    client = RuntimeApiClient()
    client.wait_until_ready()

    # 加载现有校准(只用来列状态)
    calib_table = load_calib_table(CALIB_JSON_PATH)
    if not calib_table:
        print(f"[err] 校准表为空,请先跑 manual_calibrate", file=sys.stderr)
        return 1

    # 列出现有校准状态(给用户参考)
    print(f"\n[calib] 现有校准状态:", flush=True)
    for k, v in sorted(calib_table.items()):
        marker = "✓" if v.get("reliable") else "⚠"
        print(f"  {k}: n_hits={v['n_hits']} [{marker}]", flush=True)

    # 准备阶段:用户手动摆车
    print(f"\n[ready] 把车摆到校准初始 odom 起点:", flush=True)
    print(f"  x≈+0.05m, y≈+0.03m, yaw≈-9°", flush=True)
    print(f"  摆好后,确保 cam 视野能看到至少 3 只板(通常 4 只)", flush=True)
    print(f"  按 Enter 开始 14 发补校准", flush=True)
    try:
        input("> ")
    except EOFError:
        pass

    new_shots = []
    # 14 发按板号顺序:#1 → #2 → #3 → #4 → 重复(覆盖 cam 视野 3/4 只)
    # 每次只直行 8cm 到下一只板
    # cam 视野数不对 → 让用户报告当前视野然后决定 skip 或继续
    SEQUENCE = [
        # (n_view, tid)
        (3, 1),   # 第 1 发:起点 → 看到 3 只,射 #1
        (4, 2),   # 第 2 发:直行 8cm → 看到 4 只,射 #2
        (4, 3),   # 第 3 发:直行 8cm → 看到 4 只,射 #3
        (4, 4),   # 第 4 发:直行 8cm → 看到 4 只,射 #4
        (3, 1),   # 第 5 发:直行 24cm(回到 #1)→ 看到 3 只,射 #1
        (4, 2),   # 第 6 发:直行 8cm,射 #2
        (4, 3),   # 第 7 发:直行 8cm,射 #3
        (4, 4),   # 第 8 发:直行 8cm,射 #4
        (3, 1),   # 第 9 发:直行 24cm,射 #1(第三次 cam 视野 3 只)
        (4, 2),   # 第 10 发:直行 8cm,射 #2
        (4, 3),   # 第 11 发:直行 8cm,射 #3
        (4, 4),   # 第 12 发:直行 8cm,射 #4
        (3, 2),   # 第 13 发:直行 24cm(回 #1)→ 看不到,直行 8cm 到 #2
        (4, 3),   # 第 14 发:直行 8cm,射 #3
    ]
    # 注意:序列里 step_count 不全是 1,因为 cam 视野会随位置变化
    # 实际操作:每发直接走 8cm,然后重新 detect,看视野里有几只板
    # 如果视野数对得上 + 期望板号在视野里 → 射
    # 如果视野数不对 → 让用户报告,可以选择 skip 或继续

    print(f"\n[plan] {len(SEQUENCE)} 发:每发按 8cm 步进,cam 视野数不对就 skip",
          flush=True)
    print(f"[plan] **每发之间不会自动直行** — 由你在视野不对时手动调整",
          flush=True)
    print(f"[plan] **每发射击后,你可以敲 `done` 结束(不补到 14 发也行)**",
          flush=True)

    for i, (n_view, tid) in enumerate(SEQUENCE, 1):
        print(f"\n========== 第 {i}/{len(SEQUENCE)} 发:"
              f"期望 cam {n_view} 只,射 #{tid} ==========", flush=True)
        print(f"  (你可以:1) 敲 `drive` 走 8cm 再射;2) 直接 `shoot` 射;"
              f"3) `skip` 跳过)", flush=True)

        # 让用户决定:直行 / 射击 / 跳过
        animals = None
        odo = None
        while True:
            cmd = input("  [drive / shoot / skip / state] > ").strip().lower()
            if cmd == "state":
                animals, odo = print_state(client, f"第 {i} 发")
                continue
            if cmd == "drive":
                # 直行 8cm
                drive_forward(client, BOARD_SPACING_M)
                animals, odo = print_state(client, f"直行后")
                continue
            if cmd == "skip":
                break
            if cmd == "shoot":
                if animals is None:
                    animals, odo = read_state(client)
                if not animals:
                    print(f"  [err] cam 视野空,先 `state` 看一下",
                          flush=True)
                    continue
                # 找期望板(L→R 第 tid 只)
                sorted_animals = sorted(animals, key=bbox_xc)
                if tid - 1 >= len(sorted_animals):
                    print(f"  [err] cam 视野 {len(animals)} 只,"
                          f"#{tid} 超出范围(最大 #{len(animals)})",
                          flush=True)
                    print(f"  强制选 L→R 第 {len(sorted_animals)} 只",
                          flush=True)
                    target = sorted_animals[-1]
                else:
                    target = sorted_animals[tid - 1]
                cur_xc = bbox_xc(target)
                print(f"  [aim] 选 #{tid} = cam 视野 L→R 第 {tid} 只"
                      f" (xc={cur_xc:+.3f})", flush=True)

                if args.dry_run:
                    print(f"  [dry-run] 跳过射击", flush=True)
                    hit_tid, hit_xc, skip = tid, cur_xc, False
                else:
                    print(f"  >>> 射击...", flush=True)
                    try:
                        car_call(client, "shooting", timeout=5)
                        print(f"  ✓ 已射 1 发", flush=True)
                    except Exception as e:
                        print(f"  [shoot err] {e}", file=sys.stderr)
                        continue
                    time.sleep(0.5)

                    # 报结果
                    while True:
                        rcmd = input("\n  报结果 (hit N [xc=X.XXX] / miss / skip): ").strip().lower()
                        rparts = rcmd.split()
                        if not rparts:
                            continue
                        if rparts[0] == "miss":
                            hit_tid, hit_xc, skip = None, None, False
                            break
                        if rparts[0] == "skip":
                            hit_tid, hit_xc, skip = None, None, True
                            break
                        # 宽松解析:`hit4` / `hit 4` / `hit 4 xc=0.5` 都支持
                        if rparts[0] == "hit" or (rparts[0].startswith("hit")
                                                   and rparts[0][3:].isdigit()):
                            # 提取板号
                            if rparts[0].startswith("hit") and len(rparts[0]) > 3:
                                hit_tid = int(rparts[0][3:])
                                rest = rparts[1:]
                            else:
                                try:
                                    hit_tid = int(rparts[1])
                                except ValueError:
                                    print(f"  解析失败:{rparts[1]}",
                                          flush=True)
                                    continue
                                rest = rparts[2:]
                            hit_xc = None
                            for tok in rest:
                                if tok.startswith("xc="):
                                    try:
                                        hit_xc = float(tok[3:])
                                    except ValueError:
                                        pass
                            hit_xc = None
                            if len(rparts) >= 3 and rparts[2].startswith("xc="):
                                try:
                                    hit_xc = float(rparts[2][3:])
                                except ValueError:
                                    pass
                            if hit_xc is None:
                                # 自动选 cam_view L→R 第 hit_tid 只的 xc
                                if hit_tid - 1 < len(sorted_animals):
                                    hit_xc = bbox_xc(
                                        sorted_animals[hit_tid - 1])
                                    print(f"  (auto) 用 cam_view L→R 第 {hit_tid} 只"
                                          f" xc={hit_xc:+.3f}", flush=True)
                            skip = False
                            break
                        print(f"  未知命令:{rcmd}", flush=True)

                if skip:
                    print(f"  [skip] 本发跳过,不记录", flush=True)
                    break

                # 记录
                record_shot(new_shots, odo, animals,
                            hit_tid=hit_tid, hit_xc=hit_xc)
                # 每 4 发存盘
                if i % 4 == 0:
                    save_calib_json(new_shots)
                    new_shots = []
                    print(f"  [checkpoint] 第 {i} 发写入 JSON", flush=True)
                break
            if cmd == "done":
                print(f"  [done] 用户提前结束", flush=True)
                if new_shots:
                    save_calib_json(new_shots)
                # 退出外层循环
                print(f"\n========== 提前结束 ==========", flush=True)
                return 0
            if cmd in ("help", "h", "?"):
                print(f"  drive  = 沿车头方向直行 8cm", flush=True)
                print(f"  shoot  = 射 1 发(然后报 hit/miss/skip)", flush=True)
                print(f"  state  = 看 cam 视野 + odom", flush=True)
                print(f"  skip   = 跳过这一发", flush=True)
                print(f"  done   = 提前结束", flush=True)
                continue
            print(f"  未知命令:{cmd} (可用 drive/shoot/state/skip/done/help)",
                  flush=True)

    # 写入剩余
    if new_shots:
        save_calib_json(new_shots)

    print(f"\n[done] 14 发补校准完成", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())