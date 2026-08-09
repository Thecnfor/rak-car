#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task3 识别/射击机械臂姿态常量 (arm_seq_v9 CLI 参数, orchestrator 与 task3 共用).

参数顺序与 main/task/task3/arm_seq_v9.py 的 CLI 一致:
    (y1 中间位, y2 最终位, x, arm_angle, hand_angle)
- RECOGNITION_ARM: 识别段 (cam2 看卡片) —— y2=-40mm, x=-270mm, 大臂+90°, 手爪-70°
- SHOOTING_ARM:    射击段 (cam2 看目标卡) —— y2=-150mm, x=-200mm, 大臂+90°, 手爪-90°

orchestrator 在 task1→识别区 / task2→射击区的巡线途中用这些姿态后台摆臂,
task3_pipeline / task3_shoot 在任务点再确认一次 (已在位则跳过)。
"""
from __future__ import annotations


RECOGNITION_ARM = ("-0.100", "-0.040", "-0.270", "90", "-70")
SHOOTING_ARM = ("-0.100", "-0.150", "-0.200", "90", "-90")


def arm_at_pose(client, pose, tol_m: float = 0.020, tol_deg: float = 12.0) -> bool:
    """检查机械臂是否已在目标姿态 (用于跳过重复摆臂)。

    pose 顺序同 RECOGNITION_ARM: (y1, y2, x, arm_angle, hand_angle), 只比较最终位 y2。
    读不到状态 / 值缺失 → 返回 False (调用方会重新摆臂, 保底)。
    """
    try:
        state = (client.get_arm_state() or {}).get("arm_state") or {}
        y_m = state.get("y_m")
        x_m = state.get("x_m")
        arm_angle = state.get("arm_angle")
        hand_angle = state.get("hand_angle")
    except Exception:
        return False
    if any(v is None for v in (y_m, x_m, arm_angle, hand_angle)):
        return False
    try:
        return (abs(float(y_m) - float(pose[1])) <= tol_m
                and abs(float(x_m) - float(pose[2])) <= tol_m
                and abs(float(arm_angle) - float(pose[3])) <= tol_deg
                and abs(float(hand_angle) - float(pose[4])) <= tol_deg)
    except (TypeError, ValueError, IndexError):
        return False


__all__ = ["RECOGNITION_ARM", "SHOOTING_ARM", "arm_at_pose"]
