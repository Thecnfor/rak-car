#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务一: 自动移苗 (播种) — 右侧育苗筒 -> 左侧种植区.

业务流程 (2026-08-02 重写, 用智能定位追踪 + 新 grasp/drop 协议):
  1. 初始化: reset_x 撞墙校准 + 抬升 Y 到 init_y_mm (-180) + 走到 S 姿态
     (arm=-90°, x=0, y=init_y_mm, hand=0°) — S 姿态 = track_velocity_pick 起始位
  2. 三轮循环 (按 source_position_order 走底盘列):
     a) 底盘纵向移到 SOURCE_POSITIONS_M[i]
     b) 视觉扫本列的 cylinder label (1/2/3);
        runner.track_velocity_pick 智能定位抓取 (arm 控 cx + x 十字控 cy →
        对准吸嘴 setpoint → y 降 0 → 吸气 → 抬回)
     c) composite_run 到 place 姿态 (arm=+90°, x=-270, y=servo_start, hand=-10°)
     d) 底盘纵向移到 SLOT_POSITIONS_M[target_slot_map[label]]  (label→底盘位置, 写死映射)
     e) 视觉扫本列 marker (cylinder_set);
        runner.track_velocity_pick(mode="drop") 智能定位释放 (对齐 marker 吸嘴
        setpoint → y 降 0 → drop_object)
     f) 归位: composite_run 回 S 姿态 (防碰撞顺序)

底盘位置约定 (与 task_config.yml / _constants.py 对齐):
  SOURCE_POSITIONS_M / SLOT_POSITIONS_M {1:0.0, 2:0.15, 3:0.30}.
  每个 label (cylinder_1/2/3) → 一个固定的底盘槽位 (cfg.target_slot_map).

坐标约定:
  x_mm:       0 = 机械臂最右端, 数值减小 = 向左伸出
  y_mm:       0 = 最下端限位, 负值 = 向上抬升
  arm_angle:  task1 用 ±90° 范围:
                 +90° = 左侧最大角度 (对准 T 种植槽方向, 大臂在左)
                 -90° = 右侧检测姿态 (对准 S 育苗筒方向, 大臂在右)
  hand_angle: -90° = 手爪竖直向上, 0° = 向下; 抓取时取 0°, 释放 hand=-10°

机械结构实测映射 (2026-08-02 实机标定, y=-180 时):
  画面 cx ← arm_angle (大臂更负 → cx 更右; 吸嘴中心 cx=0.161 对应 arm≈-97)
  画面 cy ← x 十字位置 (x 更左 → cy 更上)
  y 十字/手抓 → 锁死 (y 下移目标出视野, hand 固定 0° 朝下)

吸嘴 setpoint (origin.nozzle_offset_map, 2026-08-02 标定):
  目标在吸嘴正下方时其 bbox 中心坐标; 按 label 分组查表
  (cylinder_1/2/3 → (0.161,-0.519), ball_* 各自分档, 未知回落全局默认).

架构说明 (2026-08 重构):
  本任务使用 main.arm.ArmRunner + ArmVisionClient.find_target_arm_cross
  (velocity 模式实时追踪, 免 arm_queue) + 吸嘴 per-label setpoint + 新抓取协议
  (y 降 0 → 吸气; y 降 0 → drop_object 释放).
"""
from __future__ import annotations

import logging
import time
from typing import Any, Dict, List, Optional

from main.api_client import RuntimeApiClient
from main.arm import ArmClient, ArmRunner
from main.arm.vision import SelectionStrategy, TargetSelector
from main.task._config import load_task_config
from main.task._constants import SLOT_POSITIONS_M, SOURCE_POSITIONS_M

logger = logging.getLogger("task.task1_seeding")


# ── 视觉读取（cam2 task_feed 缓存） ─────────────────────────────────────────

# 每列允许看到的源头 label: 三个圆柱 (1=大/2=中/3=小)
SOURCE_LABELS: tuple = ("cylinder_1", "cylinder_2", "cylinder_3")


def _scan_cylinder_label(
    client: RuntimeApiClient,
    valid_labels: List[str],
    retries: int = 3,
    backoff_s: float = 0.5,
) -> Optional[str]:
    """通过 cam2 实时视觉接口扫描本列的 cylinder 标签 (源头识别).

    技术说明:
      此接口读取 task_feed 守护线程（默认 10Hz）写入的内存缓存,
      不直接调用 ZMQ 推理后端, 不持有 car_lock. 绕过旧接口
      /v1/vision/task (POST) 长生命周期 ZMQ REQ 套接字死锁的已知 bug.

    每列 cam2 视野里只看到 1 个 cylinder (用户约定 2026-08-02),
    因此返回首个属于 valid_labels 白名单的识别结果。
    """
    for attempt in range(retries):
        try:
            resp = client.get("/v1/realtime/vision/task", timeout=2)
        except Exception as exc:
            logger.warning("[scan_cylinder] 第 %d 次获取失败: %s", attempt + 1, exc)
            time.sleep(backoff_s)
            continue
        if not isinstance(resp, dict) or not resp.get("ok"):
            time.sleep(backoff_s)
            continue
        task_state = resp.get("task_state") or {}
        if not task_state.get("active"):
            time.sleep(backoff_s)
            continue
        for d in task_state.get("detections") or []:
            label = (d or {}).get("label", "")
            if label in valid_labels:
                return label
        time.sleep(backoff_s)
    return None


def _scan_marker_present(
    client: RuntimeApiClient,
    marker_label: str,
    retries: int = 3,
    backoff_s: float = 0.5,
) -> bool:
    """通过 cam2 实时视觉接口检查本列 marker 是否可见 (目的地识别).

    每列 cam2 视野里只看到 1 个 marker (用户约定 2026-08-02), 见到一个即放行.
    """
    for attempt in range(retries):
        try:
            resp = client.get("/v1/realtime/vision/task", timeout=2)
        except Exception as exc:
            logger.warning("[scan_marker] 第 %d 次获取失败: %s", attempt + 1, exc)
            time.sleep(backoff_s)
            continue
        if not isinstance(resp, dict) or not resp.get("ok"):
            time.sleep(backoff_s)
            continue
        task_state = resp.get("task_state") or {}
        if not task_state.get("active"):
            time.sleep(backoff_s)
            continue
        for d in task_state.get("detections") or []:
            if (d or {}).get("label", "") == marker_label:
                return True
        time.sleep(backoff_s)
    return False


# ── 底盘纵向移动 ─────────────────────────────────────────────────────────

def _chassis_move_for(
    arm_client: ArmClient,
    dx_m: float,
    timeout: float,
) -> dict:
    """底盘纵向 move_for 阻塞调用（sync=True 等结果）.

    走 ArmClient._call_car (HTTP car target). sync=True 让底层 SDK 跑完才返回.
    """
    # 注意: _call_car(name, timeout=20, *args, ...) — dx_m 必须走 args 关键字,
    # 直接位置传会被当成 timeout (duplicate value bug).
    # 直接用 http.execute_car_action — 绕开 _call_car(name, timeout, *args, ...) 的
    # timeout 位置参数陷阱 (dx_m 位置传会被当 timeout → duplicate value bug).
    # move_for 第一个参数是 position_offset=[x偏移(m), y偏移(m), 角偏移(rad)].
    return arm_client.http.execute_car_action(
        "move_for", [dx_m, 0.0, 0.0], timeout=timeout, sync=True,
    )


# ── 单轮: 抓 + 放 ─────────────────────────────────────────────────────

def _pick_at_source(
    runner: ArmRunner,
    arm_client: ArmClient,
    client: RuntimeApiClient,
    cfg: Dict[str, Any],
    column_idx: int,
) -> str:
    """第 i 列: 移到源列, 扫描, 视觉伺服对准, 抓.

    Returns: 抓到的 cylinder label (1/2/3).
    """
    logger.info("[S%d] 视觉扫描源头 cylinder label", column_idx)
    label = _scan_cylinder_label(client, list(SOURCE_LABELS))
    if label is None:
        raise RuntimeError(
            f"S{column_idx} 位置未检测到任何 cylinder ({list(SOURCE_LABELS)})"
        )
    logger.info("  -> 抓到 %s, 智能定位抓取 (arm 控 cx + x 十字控 cy)", label)

    # 2026-08-02: 智能定位抓取 (track_velocity_pick) — find_target_arm_cross 实时追踪.
    #   吸嘴中心 setpoint 走 per-label 查表 (origin.nozzle_offset_for(label));
    #   追踪在 y=init_y_mm (-180, 准备高度, 看得清楚) 开始 — 本机械结构实测:
    #     画面 cx ← arm_angle (吸嘴中心对应 arm≈-97)
    #     画面 cy ← x 十字位置
    #   大臂转 + x 十字把目标对准吸嘴中心 → y 降 0 → 吸气 → 抬回.
    init_y_mm = float(cfg.get("init_y_mm", -180.0))
    result = runner.track_velocity_pick(
        label,
        x_start=0.0, y_start=init_y_mm,
        arm_start=-90.0, hand_start=0.0,
        timeout=cfg.get("pick_track_timeout_s", 25.0),
        hz=20.0,
        gain_arm=0.4, gain_x=0.08,
        deadzone=0.02, max_vel=0.15,
        settle_hits=3,
        hold_s=cfg.get("vacuum_settle_s", 0.5),
        lift_back=True,
    )
    if not result.get("ok"):
        raise RuntimeError(
            f"S{column_idx} 智能定位抓取 {label} 失败: {result.get('reason')} "
            f"(trace_hits={result.get('trace_hits')}, end_arm={result.get('end_arm')})"
        )
    return label


def _place_at_slot(
    runner: ArmRunner,
    arm_client: ArmClient,
    client: RuntimeApiClient,
    cfg: Dict[str, Any],
    column_idx: int,
) -> None:
    """第 i 列: 移到槽列, 扫描 marker, 智能定位对准 (mode=drop), y 降 0 释放."""
    place = cfg["arm_place_pose_T2"]
    marker_label = cfg.get("marker_label", "cylinder_set")
    carry_y = float(cfg["arm_carry_pose"]["y_mm"])
    init_y_mm = cfg.get("init_y_mm", -180)
    # 智能定位起始 y: 需高于 carry_y (留 50mm+ 给追踪移动), 否则推到
    # [-200,0] 软限外被 _check_safe 拦. 现 max(-180, -100) = -100.
    servo_start_y = max(init_y_mm, carry_y + 50.0)

    # 1) composite_run 到 place 姿态 (arm=+90, x=-270, y=servo_start, hand=-10)
    logger.info("[T%d] composite_run 到 place 姿态 (arm=+90°, x=-270, y=%s)",
                column_idx, servo_start_y)
    runner.client.composite_run(
        arm=float(place["arm_angle_deg"]),
        x_mm=float(place["x_mm"]),
        y_mm=servo_start_y,
        hand=float(place["hand_angle_deg"]),
        speed=80, timeout=20.0,
    )

    # 2) 检查 marker 是否可见
    logger.info("  视觉扫描本列 marker label=%s", marker_label)
    if not _scan_marker_present(client, marker_label):
        raise RuntimeError(
            f"T{column_idx} 位置未检测到 marker {marker_label}"
        )

    # 3) marker 对准: 智能定位 (2026-08-02) — 复用 track_velocity_pick 的
    #    arm 控 cx + x 十字控 cy 对齐逻辑, mode="drop" 对齐后 y 降 0 释放.
    #    setpoint 走 per-label 查表 (cylinder_set 未标定回落全局默认).
    if cfg.get("place_align", True):
        result = runner.track_velocity_pick(
            marker_label,
            x_start=float(place["x_mm"]), y_start=servo_start_y,
            arm_start=float(place["arm_angle_deg"]),
            hand_start=float(place["hand_angle_deg"]),
            grasp_y_mm=0.0,
            mode="drop",
            timeout=cfg.get("place_track_timeout_s", 20.0),
            hz=20.0,
            gain_arm=0.4, gain_x=0.08,
            deadzone=0.02, max_vel=0.15,
            settle_hits=3,
            hold_s=0.0,
            lift_back=False,  # 已在 place 姿态, 不抬回 (后续走 _return_to_source_pose)
        )
        if not result.get("ok"):
            raise RuntimeError(
                f"T{column_idx} 智能定位释放 {marker_label} 失败: {result.get('reason')} "
                f"(trace_hits={result.get('trace_hits')}, end_arm={result.get('end_arm')})"
            )
        return  # 智能定位已含 y 降 0 + drop + (无 lift_back), 直接结束

    # 4) y 降 0 → drop_object
    logger.info("  移动 y→0 释放")
    runner.client.move_y(0.0, timeout=15.0)
    runner.drop_object()

    # 5) 抬回 carry 高度
    runner.client.move_y(carry_y, timeout=15.0)


def _return_to_source_pose(
    runner: ArmRunner,
    cfg: Dict[str, Any],
) -> None:
    """防碰撞顺序归位: composite_run 自动按 (X 收 → 大臂转) 顺序提交."""
    ret = cfg["arm_return_S1_pose"]
    runner.client.composite_run(
        arm=float(ret["arm_angle_deg"]),
        x_mm=float(ret["x_mm"]),
        y_mm=float(ret["y_mm"]),
        hand=float(ret["hand_angle_deg"]),
        speed=80, timeout=30.0,
    )


# ── 主入口 ────────────────────────────────────────────────────────────────

def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """任务一主入口: 自动移苗 (S1/S2/S3 -> T1/T2/T3, 智能定位追踪抓取/释放).

    Args:
        client: 可选的 RuntimeApiClient 实例, 未传入时自动创建新连接

    Returns:
        Dict: {
            "ok": bool,
            "completed": List[str],  # 已成功处理的 cylinder 标签列表
            "error": str             # 失败时的错误信息 (仅 ok=False 时存在)
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
    # S 姿态 = track_velocity_pick 起始位 (y=-180, 看得清楚)
    init_y_mm = cfg.get("init_y_mm", -180)

    try:
        # ===== 初始化步骤 1. X 编码器校准 (撞右墙硬限位定原点; 必须独占) =====
        logger.info("init: X 编码器撞墙校准 (reset_x → right)")
        arm_client.reset_x(direction="right", timeout=30.0)

        # ===== 初始化步骤 2. 一次性走完 S 姿态 4 轴 (composite_run 并发) =====
        # composite_run 内部按 (X 收 → 大臂转) 顺序防碰, 4 个目标并发执行.
        # 省掉原来的 4 个串行调用 (~7s → ~2s), 比赛里每次进 task1 都赚回来.
        pick = cfg["arm_pick_pose"]
        logger.info(
            "init: S 姿态 (composite_run) arm=%s° hand=%s° X=%s mm Y=%s mm",
            pick["arm_angle_deg"], pick["hand_angle_deg"], pick["x_mm"], init_y_mm,
        )
        runner.client.composite_run(
            arm=float(pick["arm_angle_deg"]),
            x_mm=float(pick["x_mm"]),
            y_mm=init_y_mm,
            hand=float(pick["hand_angle_deg"]),
            speed=80, timeout=20.0,
        )

        # ===== 主循环: 按 source_position_order 走底盘列 =====
        source_position_order = cfg["source_position_order"]
        target_slot_map = cfg["target_slot_map"]   # cylinder_N -> slot N (底盘位置)
        chassis_move_timeout = cfg["chassis_move_timeout_s"]

        last_chassis_col: Optional[int] = None

        for i, column_idx in enumerate(source_position_order):
            logger.info("=== 处理底盘列 %d (S%d) ===", i + 1, column_idx)

            # 底盘纵向移到本列
            if last_chassis_col is not None:
                dx_m = SOURCE_POSITIONS_M[column_idx] - SOURCE_POSITIONS_M[last_chassis_col]
                if abs(dx_m) > 1e-3:
                    logger.info("  底盘纵向移动 %.3f m → S%d", dx_m, column_idx)
                    _chassis_move_for(arm_client, dx_m, timeout=chassis_move_timeout)

            # 抓: 视觉识别 label
            label = _pick_at_source(runner, arm_client, client, cfg, column_idx)
            completed.append(label)

            # 底盘移到该 label 对应的槽位 (label → slot, 写死映射)
            slot_idx = int(target_slot_map[label])
            slot_dx_m = SLOT_POSITIONS_M[slot_idx] - SOURCE_POSITIONS_M[column_idx]
            if abs(slot_dx_m) > 1e-3:
                logger.info("  底盘纵向移动 %.3f m → T%d (label=%s)",
                            slot_dx_m, slot_idx, label)
                _chassis_move_for(arm_client, slot_dx_m, timeout=chassis_move_timeout)

            # 放
            _place_at_slot(runner, arm_client, client, cfg, slot_idx)

            # 抬回 + 防碰撞归位 S 姿态
            carry_y = float(cfg["arm_carry_pose"]["y_mm"])
            runner.move_y(carry_y)
            _return_to_source_pose(runner, cfg)

            last_chassis_col = column_idx

        # ===== 任务结束: 若最后一轮落在 T 列, 归位到 S1 =====
        if last_chassis_col is not None and last_chassis_col != source_position_order[0]:
            dx_m = SOURCE_POSITIONS_M[source_position_order[0]] - SOURCE_POSITIONS_M[last_chassis_col]
            if abs(dx_m) > 1e-3:
                logger.info("任务结束, 底盘归位到 S%d, dx=%.3f m",
                            source_position_order[0], dx_m)
                _chassis_move_for(arm_client, dx_m, timeout=chassis_move_timeout)

    except Exception as exc:
        logger.exception("task1_seeding 失败: %s", exc)
        return {"ok": False, "completed": completed, "error": str(exc)}

    return {"ok": True, "completed": completed}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
    result = run()
    print("任务一 自动移苗 执行结果:", result)