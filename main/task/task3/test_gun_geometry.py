#!/usr/bin/python3
# -*- coding: utf-8 -*-
r"""main/tasks/task333/test_gun_geometry.py - 验证枪口几何位置

最简测试:把 #1 转到 cam 中央 → 射 1 发 → 看子弹落点

如果打中 #1(消失或 score 大降)→ 枪口基本在 cam 中央(车头方向)
如果打到 #1 左边 → 枪口在 cam 视野左,需要 +offset
如果打到 #1 右边 → 枪口在 cam 视野右,需要 -offset
如果完全没打到 → 距离/角度问题,跟枪位置无关

用法:
    cd "C:\Users\花花世界\Desktop\天道酬勤\rak-car"
    python -m main.tasks.task333.test_gun_geometry
"""
from __future__ import annotations

import math
import sys
import time

from main.api_client import RuntimeApiClient


# 复用 shoot_4_targets 的核心工具函数
sys.path.insert(0, r"C:\Users\花花世界\Desktop\天道酬勤\rak-car")
from main.tasks.task333.shoot_4_targets import (
    car_call, read_odom, get_animals, get_animals_retry,
    bbox_xc, bbox_yc, bbox_width, MIN_YOLO_SCORE,
    YAW_SIGN
)


def main():
    client = RuntimeApiClient()
    client.wait_until_ready()

    # reset odom
    try:
        car_call(client, "reset_position", timeout=5)
        print("[reset] odom 已清零", flush=True)
        time.sleep(0.5)
    except Exception as e:
        print(f"[reset err] {e}", file=sys.stderr)

    # 等待检测到目标
    print("\n[detect] 等待 cam 视野出现目标...")
    t0 = time.time()
    while time.time() - t0 < 15:
        animals = get_animals(client, MIN_YOLO_SCORE)
        if animals:
            break
        time.sleep(0.3)
    if not animals:
        print("[err] 没检测到目标,退出", file=sys.stderr)
        return 1

    animals = sorted(animals, key=bbox_xc)
    target = animals[0]   # 取最左一只做测试
    initial_xc = bbox_xc(target)
    initial_score = target.get("score", 0.0)
    print(f"\n[test] 选 #1 xc={initial_xc:+.3f} yc={bbox_yc(target):.3f} "
          f"score={initial_score:.3f}", flush=True)

    # Phase 1:用 yaw 把 #1 转到 cam 中央(xc=0.5)
    print(f"\n[turn] 把 #1 转到 cam 中央...", flush=True)
    for i in range(10):
        animals = get_animals_retry(client, MIN_YOLO_SCORE,
                                    label=f"turn {i+1}", retries=2, delay=0.2)
        if not animals:
            print(f"  步 {i+1}: 无目标,停", flush=True)
            break
        target = min(animals, key=lambda a: abs(bbox_xc(a) - initial_xc))
        cur_xc = bbox_xc(target)
        dx = cur_xc - 0.5
        if abs(dx) < 0.025:
            print(f"  ✓ 已到 cam 中央 cur_xc={cur_xc:+.3f}", flush=True)
            break
        # yaw:YAW_SIGN=+1(实测),dx>0 → 发负 yaw 让目标视觉左移 → xc 减小
        yaw_adj = -dx * 20.0 * YAW_SIGN
        yaw_adj = max(-10.0, min(10.0, yaw_adj))
        try:
            car_call(client, "move_for", [0.0, 0.0, math.radians(yaw_adj)],
                     timeout=5)
            print(f"  步 {i+1}: dx={dx:+.3f} → yaw {yaw_adj:+.2f}°",
                  flush=True)
        except Exception as e:
            print(f"  yaw err: {e}", file=sys.stderr)
            break
        time.sleep(0.4)

    # 重新读目标位置
    animals = get_animals_retry(client, MIN_YOLO_SCORE,
                                label="射前 re-detect", retries=3, delay=0.3)
    if not animals:
        print("[err] 找不到目标,退出", file=sys.stderr)
        return 1
    target = min(animals, key=lambda a: abs(bbox_xc(a) - initial_xc))
    pre_xc = bbox_xc(target)
    pre_score = target.get("score", 0.0)
    print(f"\n[射前] #1 xc={pre_xc:+.3f} score={pre_score:.3f}", flush=True)

    # Phase 2:射 1 发
    print(f"\n[shoot] >>> 射击 1 发...", flush=True)
    try:
        car_call(client, "shooting", timeout=5)
    except Exception as e:
        print(f"[shoot err] {e}", file=sys.stderr)

    time.sleep(1.0)   # 等久一点,让板子倒的视觉变化稳定

    # Phase 3:re-detect,看板子变化
    animals = get_animals_retry(client, MIN_YOLO_SCORE,
                                label="射后 re-detect", retries=3, delay=0.3)
    print(f"\n[射后] cam 视野 {len(animals)} 只:", flush=True)
    for a in sorted(animals, key=bbox_xc):
        xc = bbox_xc(a)
        yc = bbox_yc(a)
        score = a.get("score", 0.0)
        marker = " ← 测的 #1" if abs(xc - pre_xc) < 0.15 else ""
        print(f"  xc={xc:+.3f} yc={yc:.3f} score={score:.3f}{marker}",
              flush=True)

    # 找原始 #1(按 pre_xc 最近)
    if animals:
        target_after = min(animals, key=lambda a: abs(bbox_xc(a) - pre_xc))
        post_xc = bbox_xc(target_after)
        post_score = target_after.get("score", 0.0)
        print(f"\n[对比] 测的 #1:")
        print(f"  射前: xc={pre_xc:+.3f} score={pre_score:.3f}")
        print(f"  射后: xc={post_xc:+.3f} score={post_score:.3f}")
        print(f"  xc 变化: {post_xc - pre_xc:+.3f}")
        print(f"  score 变化: {post_score - pre_score:+.3f}")
        if post_score < pre_score - 0.3:
            print(f"\n[诊断] ✓ score 大降,可能打中(板子被击中/倒下)")
        if abs(post_xc - pre_xc) > 0.1:
            print(f"[诊断] xc 偏移 {post_xc - pre_xc:+.3f} "
                  f"→ 子弹落点不在 cam 中央位置!")
            print(f"  如果 xc 偏负 → 子弹落到板子左边(枪口在 cam 视野左)")
            print(f"  如果 xc 偏正 → 子弹落到板子右边(枪口在 cam 视野右)")
        else:
            print(f"[诊断] xc 偏移很小,可能打中中心或没打到")
    else:
        print(f"\n[诊断] ✓ cam 视野空了 — 板子完全倒下(强烈击中)")

    return 0


if __name__ == "__main__":
    sys.exit(main())