#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务一: 自动移苗 (播种) — 右侧育苗筒 S1/S2/S3 -> 左侧种植区 T1/T2/T3.

业务流程 (按 S1 -> S2 -> S3 顺序循环处理每个源位置):
  1. 机械臂复位到 S1 检测姿态 (X=-100, arm=-90°, Y=-100, 手爪=0°)
  2. cam2 视觉扫描识别幼苗标签 (颜色/类别)
  3. 通过 target_slot_map 查表得到目标种植槽编号 T1/T2/T3
  4. Y 轴下降到抓取高度 -> 开真空吸苗 -> 等待真空稳定 -> 抬升到运输高度
  5. 底盘纵向移动: 对齐源位置 S_idx 与目标槽 T_slot
  6. 机械臂旋转到 +90° + X 轴伸出到种植槽上方 (X=-270)
  7. Y 轴下降到放置高度 -> 关真空放苗
  8. Y 轴抬升 -> 先收 X 到 -100 再旋转 arm 回 -90° (composite_run 内置防碰撞顺序)
  9. 底盘纵向移动到下一个源位置 (或任务结束归位到 S1)

坐标约定 (与 task_config.yml 对齐):
  x_mm:       0 = 机械臂最右端, 数值减小 = 向左伸出
  y_mm:       0 = 最下端限位, 负值 = 向上抬升
  arm_angle:  task1 用 ±90° 范围 (reset_x 撞右墙为原点, 与 task2 的 [-150°,+90°] 不同):
                 +90° = 左侧最大角度 (对准 T2 种植槽方向)
                 -90° = 右侧检测姿态 (对准 S1 育苗筒方向)
  hand_angle: -90° = 手爪竖直向上, 0° = 向下; 抓取时取 0° (正下对准育苗筒)

架构说明 (2026-08 重构):
  本任务使用 main.arm.ArmRunner (含 y 保护区 / 角度硬限 / 丢步核对 / 并发执行)
  + CompositeMixin (composite_pick / composite_release / composite_run) 编排动作,
  不再依赖 main/task/_helpers.py (该文件已删除, 详见 main/task/README.md §架构历史).
"""
from __future__ import annotations

import logging
import time
from typing import Any, Dict, List, Optional

from main.api_client import RuntimeApiClient
from main.arm import ArmClient, ArmRunner
from main.task._config import load_task_config
from main.task._constants import SLOT_POSITIONS_M, SOURCE_POSITIONS_M

logger = logging.getLogger("task.task1_seeding")


# ── 视觉读取（cam2 缓存） ─────────────────────────────────────────

def _scan_labels(
    client: RuntimeApiClient,
    valid_labels: List[str],
    retries: int = 3,
    backoff_s: float = 0.5,
) -> Optional[str]:
    """通过 cam2 实时视觉接口扫描幼苗标签（HTTP 缓存, 不直接打 ZMQ）.

    技术说明:
      此接口读取 task_feed 守护线程（默认 10Hz）写入的内存缓存,
      不直接调用 ZMQ 推理后端, 不持有 car_lock, 因此不会因推理后端卡死
      而导致 runtime 崩溃. 该设计绕过了旧接口 /v1/vision/task (POST)
      可能导致长生命周期 ZMQ REQ 套接字死锁的已知 bug.

    返回数据格式 task_state:
      {
        active, mode,
        detections: [{cls_id, det_id, label, score, bbox_norm{...}}],
        count, updated_at
      }

    筛选策略: 按 bbox_norm.width 降序排列（越靠近镜头的育苗筒检测框越宽）,
    返回第一个属于 valid_labels 白名单的识别结果. 所有重试耗尽仍失败则返回 None.
    """
    for attempt in range(retries):
        try:
            resp = client.get("/v1/realtime/vision/task", timeout=2)
        except Exception as exc:
            logger.warning("[scan_labels] 第 %d 次获取失败: %s", attempt + 1, exc)
            time.sleep(backoff_s)
            continue
        if not isinstance(resp, dict) or not resp.get("ok"):
            time.sleep(backoff_s)
            continue
        task_state = resp.get("task_state") or {}
        if not task_state.get("active"):
            time.sleep(backoff_s)
            continue
        dets = sorted(
            task_state.get("detections") or [],
            key=lambda d: -(float((d.get("bbox_norm") or {}).get("width", 0.0))),
        )
        for d in dets:
            label = (d or {}).get("label", "")
            if label in valid_labels:
                return label
        time.sleep(backoff_s)
    return None


# ── 底盘移动 ─────────────────────────────────────────────────────

def _chassis_move_for(
    arm_client: ArmClient,
    dx_m: float,
    timeout: float,
) -> dict:
    """底盘纵向 move_for 阻塞调用（sync=True 等结果）.

    走 ArmClient._call_car (HTTP car target). sync=True 让底层 SDK 跑完才返回.
    """
    return arm_client._call_car(
        "move_for", dx_m, timeout=timeout, sync=True,
    )


# ── 单棵幼苗抓取 → 运输 → 放置 ─────────────────────────────────

def _transport_to_slot(
    runner: ArmRunner,
    arm_client: ArmClient,
    cfg: Dict[str, Any],
    slot: int,
    source_idx: int,
) -> None:
    """底盘纵向对齐 + 机械臂联合运动到目标种植槽.

    执行顺序（composite_run 内置并发 + y 保护区）:
      1. composite_run: arm → place.arm_angle_deg + X → place.x_mm + Y → carry_y 并发
      2. 底盘 move_for: 从 SOURCE_POSITIONS_M[source_idx] 对齐到 SLOT_POSITIONS_M[slot]
    """
    timeout = cfg["chassis_move_timeout_s"]
    place = cfg["arm_place_pose_T2"]

    dx_m = SLOT_POSITIONS_M[slot] - SOURCE_POSITIONS_M[source_idx]

    # 1) 复合动作: 大臂转 + X 伸出并发（composite_run 自动校验 y 保护区）
    runner.client.composite_run(
        arm=float(place["arm_angle_deg"]),
        x_mm=float(place["x_mm"]),
        y_mm=float(cfg["arm_carry_pose"]["y_mm"]),
    )

    # 2) 底盘纵向移动到目标槽
    if abs(dx_m) > 1e-3:
        logger.info("  底盘纵向移动 %.3f m (S%d → T%d)", dx_m, source_idx, slot)
        _chassis_move_for(arm_client, dx_m, timeout=timeout)


def _safe_return_to_s1(
    runner: ArmRunner,
    cfg: Dict[str, Any],
) -> None:
    """防碰撞顺序归位: composite_run 自动按 (X 收 → 大臂转) 顺序提交.

    约束说明（防止 X=-270 时大臂横扫撞到育苗筒）:
      arm 在 +90° 状态下执行大角度旋转 (-90°) 时, 如果 X 还停在 -270 会
      横扫底盘边缘. 因此顺序必须: 先收 X 到内侧, 再旋转大臂.
      composite_run 把这两个动作并发, 但内部驱动层会自动按此顺序提交.
    """
    ret = cfg["arm_return_S1_pose"]
    runner.client.composite_run(
        arm=float(ret["arm_angle_deg"]),
        x_mm=float(ret["x_mm"]),
        y_mm=float(ret["y_mm"]),
    )


# ── 主入口 ────────────────────────────────────────────────────────

def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """任务一主入口: 自动移苗 (S1/S2/S3 -> T1/T2/T3).

    Args:
        client: 可选的 RuntimeApiClient 实例, 未传入时自动创建新连接

    Returns:
        Dict: {
            "ok": bool,           # 任务是否成功完成
            "completed": List[str],  # 已成功处理的幼苗标签列表
            "error": str          # 失败时的错误信息 (仅 ok=False 时存在)
        }
    """
    cfg = load_task_config("auto_seeding")
    if cfg.get("placeholder"):
        raise NotImplementedError("任务 auto_seeding 配置尚未完成")

    # 初始化 runtime 连接
    if client is None:
        client = RuntimeApiClient()
    client.wait_until_ready(timeout=30.0)

    # 初始化机械臂客户端与执行器 (ArmRunner 集成 SafetyMixin / Composite / 丢步核对)
    arm_client = ArmClient.connect()
    if not arm_client.ping():
        raise RuntimeError("机械臂 runtime 未在线, 请检查 arm_feed 守护进程")
    runner = ArmRunner(arm_client)

    completed: List[str] = []
    init_y_mm = cfg.get("init_y_mm", -100)

    try:
        # ===== 初始化步骤 1. X 编码器校准 (撞右墙硬限位定原点) =====
        logger.info("init: X 编码器撞墙校准 (reset_x → right)")
        arm_client.reset_x(direction="right", timeout=30.0)

        # ===== 初始化步骤 2. Y 轴抬升到安全初始高度 =====
        logger.info("init: 抬升 Y 到 %s mm", init_y_mm)
        runner.move_y(init_y_mm)

        # ===== 初始化步骤 3. 走到 S1 检测姿态 =====
        pick = cfg["arm_pick_pose"]
        logger.info(
            "init: S1 检测姿态 arm=%s° hand=%s° X=%s mm Y=%s mm",
            pick["arm_angle_deg"], pick["hand_angle_deg"], pick["x_mm"], init_y_mm,
        )
        arm_client.set_hand_angle(float(pick["hand_angle_deg"]), speed=80, timeout=10.0)
        runner.set_arm_angle(float(pick["arm_angle_deg"]), speed=80)
        runner.move_x(float(pick["x_mm"]))

        # ===== 主循环: 按 S1 → S2 → S3 顺序处理每个源位置 =====
        source_position_order = cfg["source_position_order"]
        valid_labels = list(cfg["target_slot_map"].keys())

        for i, source_idx in enumerate(source_position_order):
            logger.info("=== 处理 S%d (iteration %d/%d) ===",
                        source_idx, i + 1, len(source_position_order))

            # ===== 步骤 0. 复位到 S1 检测姿态 (from any previous pose) =====
            _safe_return_to_s1(runner, cfg)
            arm_client.set_hand_angle(float(pick["hand_angle_deg"]), speed=80, timeout=10.0)
            runner.move_y(init_y_mm)

            # ===== 步骤 1. 视觉识别幼苗标签 =====
            label = _scan_labels(client, valid_labels)
            if label is None:
                raise RuntimeError(
                    f"cam2 在 S{source_idx} 位置未检测到任何有效标签 {valid_labels}"
                )
            slot = cfg["target_slot_map"][label]
            logger.info("S%d 检测到 %s → T%d", source_idx, label, slot)

            # ===== 步骤 2. 抓取幼苗 (composite_pick 一步完成 arm+X+Y+hand 并发) =====
            runner.client.composite_pick(
                arm_angle=float(pick["arm_angle_deg"]),
                x_mm=float(pick["x_mm"]),
                y_mm=float(pick["y_mm"]),
                hand=float(pick["hand_angle_deg"]),
                speed=80,
            )
            runner.grasp(on=True)
            time.sleep(cfg["vacuum_settle_s"])
            runner.move_y(float(cfg["arm_carry_pose"]["y_mm"]))

            # ===== 步骤 3. 底盘 + 机械臂联合运动到目标种植槽 =====
            _transport_to_slot(runner, arm_client, cfg, slot, source_idx)

            # ===== 步骤 4. 释放幼苗 (composite_release 一步完成) =====
            place = cfg["arm_place_pose_T2"]
            runner.client.composite_release(
                drop_x_mm=float(place["x_mm"]),
                drop_y_mm=float(place["y_mm"]),
                hand=float(place["hand_angle_deg"]),
                speed=80,
            )
            runner.grasp(on=False)

            # ===== 步骤 5. 防碰撞顺序归位 + 底盘纵向移动到下一个源 =====
            carry_y = float(cfg["arm_carry_pose"]["y_mm"])
            runner.move_y(carry_y)
            _safe_return_to_s1(runner, cfg)

            # 底盘位移编排: 源/槽位置常量见 main.task._constants
            current_chassis = SLOT_POSITIONS_M[slot]
            if i + 1 < len(source_position_order):
                next_source_idx = source_position_order[i + 1]
                next_chassis = SOURCE_POSITIONS_M[next_source_idx]
                next_offset_m = next_chassis - current_chassis
                if abs(next_offset_m) > 1e-3:
                    logger.info(
                        "底盘: T%d (%.2f) → S%d (%.2f), dx=%.3f m",
                        slot, current_chassis, next_source_idx, next_chassis, next_offset_m,
                    )
                    _chassis_move_for(
                        arm_client, dx_m=next_offset_m,
                        timeout=cfg["chassis_move_timeout_s"],
                    )
            else:
                # 任务结束: 若落在 T1/T3, 需归位到 S1
                if abs(current_chassis) > 1e-3:
                    logger.info("底盘: 任务结束, 从 %.2f 归位到 S1", current_chassis)
                    _chassis_move_for(
                        arm_client, dx_m=-current_chassis,
                        timeout=cfg["chassis_move_timeout_s"],
                    )

            completed.append(label)

    except Exception as exc:
        logger.exception("task1_seeding 失败: %s", exc)
        return {"ok": False, "completed": completed, "error": str(exc)}

    return {"ok": True, "completed": completed}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
    result = run()
    print("任务一 自动移苗 执行结果:", result)