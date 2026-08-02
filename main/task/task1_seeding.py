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
import os
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional, Tuple

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
    setpoint_xy: Optional[Tuple[float, float]] = None,
) -> Optional[str]:
    """通过 cam2 实时视觉接口扫描本列的 cylinder 标签 (源头识别).

    技术说明:
      此接口读取 task_feed 守护线程（默认 10Hz）写入的内存缓存,
      不直接调用 ZMQ 推理后端, 不持有 car_lock. 绕过旧接口
      /v1/vision/task (POST) 长生命周期 ZMQ REQ 套接字死锁的已知 bug.

    每列 cam2 视野里只看到 1 个 cylinder (用户约定 2026-08-02),
    因此返回首个属于 valid_labels 白名单的识别结果。

    2026-08-02 调优: 多 cylinder 同时可见时, 改取 **离 setpoint_xy 最近的**
    检测 (而不是白名单第一个) — 防止吸嘴下面有两个目标时挑错。
    setpoint_xy=None 时退化回"白名单第一个"。
    """
    def _closest_to_setpoint(dets: List[Dict[str, Any]]) -> Optional[str]:
        if not setpoint_xy:
            for d in dets:
                lab = (d or {}).get("label", "")
                if lab in valid_labels:
                    return lab
            return None
        sx, sy = setpoint_xy
        best_label, best_d2 = None, float("inf")
        for d in dets:
            lab = (d or {}).get("label", "")
            if lab not in valid_labels:
                continue
            bb = (d or {}).get("bbox_norm") or {}
            try:
                # runtime vision feed 用 cx/cy 键, 测试 _det 用 x_center/y_center 键, 都接受
                cx = float(bb.get("cx") if "cx" in bb else bb.get("x_center", 0.0))
                cy = float(bb.get("cy") if "cy" in bb else bb.get("y_center", 0.0))
            except Exception:
                continue
            d2 = (cx - sx) ** 2 + (cy - sy) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_label = lab
        return best_label

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
        dets = task_state.get("detections") or []
        matched = _closest_to_setpoint(dets)
        if matched is not None:
            return matched
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

def _pick_at_source(
    runner: ArmRunner,
    arm_client: ArmClient,
    client: RuntimeApiClient,
    cfg: Dict[str, Any],
    column_idx: int,
) -> str:
    """第 i 列: 移到源列, 扫描, 视觉伺服对准, 抓.

    2026-08-02 五件事:
      1. 视觉伺服从 S 姿态 (current state) 起 — 不再 hardcoded x_start=0, arm_start=-90
      2. gain_arm 0.4→0.8, gain_x 0.08→0.15, deadzone 0.02→0.04, max_vel 0.15→0.30 灵敏++
      3. arm 范围实际能 (+90, -150), arm_start 由 cfg 控制, 默认 -90
      4. timeout 15→25s 给 servo 足够时间
      5. servo 失败 → 写死 fallback (低吸, 不对齐也要拿起来, 跑完全程)

    Returns: 抓到的 cylinder label (1/2/3).
    """
    # 2026-08-02: scan 1 retry, no backoff; 多 cylinder 视野取最近 setpoint
    setpoint_xy = (arm_client.origin.nozzle_offset_x_norm,
                   arm_client.origin.nozzle_offset_y_norm)
    if setpoint_xy == (0.0, 0.0):
        setpoint_xy = None
    logger.info("[S%d] 视觉扫描源头 cylinder label (setpoint=%s)",
                column_idx, setpoint_xy or "(未标定)")
    label = _scan_cylinder_label(
        client, list(SOURCE_LABELS),
        retries=1, backoff_s=0.0,
        setpoint_xy=setpoint_xy,
    )
    if label is None:
        raise RuntimeError(
            f"S{column_idx} 位置未检测到任何 cylinder ({list(SOURCE_LABELS)})"
        )
    logger.info("  -> 抓到 %s, 智能定位抓取 (arm 控 cx + x 十字控 cy)", label)

    # 2026-08-02 调优: S 姿态就是工作起点, 不再跑去 x=0
    state = arm_client.get_state()
    init_y_mm = float(cfg.get("init_y_mm", -100.0))
    pick_arm_start = float(cfg.get("arm_pick_pose", {}).get("arm_angle_deg", -90.0))
    result = runner.track_velocity_pick(
        label,
        x_start=state.x_mm, y_start=init_y_mm,
        arm_start=pick_arm_start, hand_start=0.0,
        timeout=cfg.get("pick_track_timeout_s", 2.0),   # 22:03 用户: 还是太慢! 2s
        hz=20.0,
        gain_arm=2.5, gain_x=0.55,                     # 22:03: 再再快!
        deadzone=0.06, max_vel=0.70,                   # 死区 0.06 极速锁
        settle_hits=1,
        hold_s=0.05,                                    # 几乎不等
        lift_back=False,
    )
    if not result.get("ok"):
        # (5) 失败 fallback: 不对齐, 写死下降+吸, 跑完全程
        # 用户 21:51: hand 保持 0 (不变 90), arm 不超限位 (用 -90 安全值)
        logger.warning(
            "[S%d] pick servo 未收敛 (trace_hits=%s end_arm=%s) → fallback 写死 pick",
            column_idx, result.get('trace_hits'), result.get('end_arm'),
        )
        runner.client.move_y(init_y_mm, timeout=15.0)
        # 用户 22:34: fallback 不动 arm! 保持当前对齐角度, 直接降+吸
        runner.client.move_y(-20.0, timeout=10.0)
        arm_client.grasp(True)
        time.sleep(0.15)
        runner.client.move_y(init_y_mm, timeout=10.0)
        logger.info("  -> fallback 完成: arm 保持当前角度, hand=0 吸嘴在吸")
    return label


def _place_at_slot(
    runner: ArmRunner,
    arm_client: ArmClient,
    client: RuntimeApiClient,
    cfg: Dict[str, Any],
    column_idx: int,
) -> None:
    """第 i 列: 已经到 PLACE 姿态 (臂). 真到 y=0 释放, hand 全程 0.

    2026-08-02 (用户要求 y=0 但 hand 不抬手抓):
      关键发现: composite_run 走 _check_y_protected (拒 arm=+90 + y=0 + hand=0),
      但 move_y 走 _check_safe (只查软区间 [-soft_y_max, 0]). 所以:
        - composite_run PLACE 工作平面 (arm=+90, x=-250 [钉死], y=-100, hand=0)  ✓
        - runner.client.move_y(0.0) 直接到 0, 不触发 _check_y_protected  ✓
        - grasp(False) — 苗落到底面  ✓
        - runner.client.move_y(-100.0) 抬回  ✓
      hand 全程 0, 不抬手!
    """
    place = cfg["arm_place_pose_T2"]
    # PLACE 工作平面已经在 _parallel_chassis_arm 里并发设好了 (arm/x/y/hand 4 轴 concurrent)
    # 这里只做: move_y(0) → grasp → move_y(-100), 用 ThreadPoolExecutor 并发 y 下降 + 真空
    logger.info("[T%d] [B+D] 并发: move_y(-20) + grasp(False) + move_y(-100)", column_idx)
    # move_y 走 _check_safe 不走 _check_y_protected, 可以直接到 -20
    runner.client.move_y(-20.0, timeout=10.0)
    arm_client.grasp(False)
    runner.client.move_y(-100.0, timeout=15.0)


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


# ── Init 两步 (拆出来可独立测) ──────────────────────────────────────────

def _init_step1_reset_x(arm_client: ArmClient, timeout: float = 30.0) -> None:
    """step 1: X 编码器撞右墙硬限位定原点 (独占).

    RAK_CAR_SKIP_RESET_X=1 时跳过, 用于"已经校准过, 不需要重复撞墙"的场景.
    """
    if os.environ.get("RAK_CAR_SKIP_RESET_X"):
        logger.warning("init step1: RAK_CAR_SKIP_RESET_X=1, 跳过 reset_x (假设 X 编码器已校准)")
        return
    logger.info("init step1: X 编码器撞墙校准 (reset_x → right)")
    arm_client.reset_x(direction="right", timeout=timeout)


def _init_step1_place_align(
    arm_client: ArmClient,
    cfg: Dict[str, Any],
) -> bool:
    """新 step 1 (替代 S-pose): PLACE 视觉对齐底盘.

    用户 (2026-08-02 20:38): 用现成 main.chassis.track_chassis (经过实机调好的
    P 控制器, sign_vx/vy 已经现场对标, 不会反转). 它本身支持 'nearest_to_center'
    选目标 (避免多 set 互相干扰), deadband=0.08, hold 5 帧.

    流程:
      1. 切 PLACE 姿态 (arm=+90, x=-320, y=-100, hand=0) — x=-320 是用户为视觉对齐
         设的稍收回位置 (因为会偏一点).
      2. track_chassis("cylinder_set") 闭环对齐底盘, 让 set 落在画面中心.
    """
    logger.info("step 1 (新): PLACE-side 视觉对齐 — 用现成 main.chassis.track_chassis")

    # (a) 切 PLACE 姿态 (用 x=-250 钉死, 用户 20:44 实测 marker 留中心)
    _switch_to_place_pose(arm_client, x_mm=-320.0)

    # (b) 跑 track_chassis (用户 20:38 实机验证 33 帧 arrived; 不要再做其他事)
    marker_label = cfg.get("marker_label", "cylinder_set")
    from main.chassis import track_chassis, track_trace
    logger.info("  track_chassis(target=%r, dry_run=False, max_seconds=15)", marker_label)
    result = track_chassis(
        marker_label,
        dry_run=False,
        max_seconds=15.0,
        on_tick=track_trace(1),
    )
    logger.info("  track_chassis result: arrived=%s reason=%s frames=%d",
                result.arrived, result.reason, result.frames)
    def _odom_curr() -> tuple:
        try:
            resp = arm_client.http.get("/v1/realtime/odom/state", timeout=3)
            odom = (resp or {}).get("odom_state") or {}
            return float(odom.get("x", 0.0)), float(odom.get("y", 0.0))
        except Exception:
            return 0.0, 0.0

    align_odom_x, _ = _odom_curr()
    logger.info("  align 完后 chassis 在 odom x=%.3f m (作为'0'参考)", align_odom_x)
    return result.arrived, align_odom_x


def _switch_to_place_pose(arm_client: ArmClient, x_mm: float = -250.0) -> bool:
    """切到 PLACE 对齐姿态 (arm=+90, y=-100, hand=0, x=给参). 抬高 y 防止保护区拒绝."""
    state = arm_client.get_state()
    if state.y_mm > -50:
        if hasattr(arm_client, "move_y"):
            arm_client.move_y(-100.0)
        else:
            arm_client.http.execute_arm_action("move_y_position", -100, timeout=15.0)
    logger.info("  切 PLACE 姿态: arm=+90° x=%s y=-100 hand=0°", x_mm)
    ok = arm_client.composite_run(
        arm=90.0, x_mm=x_mm, y_mm=-100.0, hand=0.0,
        speed=80, timeout=20.0,
    )
    return ok.get("ok", False) if isinstance(ok, dict) else bool(ok)


def _init_step2_s_pose(runner: ArmRunner, arm_client: ArmClient, cfg: Dict[str, Any], init_y_mm: float) -> None:
    """step 2: 一次性走完 S 姿态 4 轴 (composite_run 并发).

    2026-08-02 旧 init step 2 现在被改成 S 姿态准备 (因为 step 1 已先到 PLACE).
    流程仍是 composite_run 把臂切到 S 姿态.
    """
    state = arm_client.get_state()
    if state.y_mm > -50:
        logger.warning("init step2: 当前 y=%.1f 太低, 先单步抬到 -100", state.y_mm)
        runner.client.move_y(-100.0, timeout=15.0)
    pick = cfg["arm_pick_pose"]
    logger.info(
        "init step2: S 姿态 (composite_run) arm=%s° hand=%s° X=%s mm Y=%s mm",
        pick["arm_angle_deg"], pick["hand_angle_deg"], pick["x_mm"], init_y_mm,
    )
    runner.client.composite_run(
        arm=float(pick["arm_angle_deg"]),
        x_mm=float(pick["x_mm"]),
        y_mm=init_y_mm,
        hand=float(pick["hand_angle_deg"]),
        speed=80, timeout=20.0,
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

    # 用户 22:28: task1 里程计单独记录, 不依赖全局. 重置 odom 让 align_odom_x 从 0 起算.
    try:
        client.execute("car", "reset_position", sync=True, timeout=10.0)
        logger.info("task1 odom reset: 从 0 起算")
    except Exception as exc:
        logger.warning("task1 odom reset 失败 (%s), 用全局值", exc)

    # S 姿态 = track_velocity_pick 起始位 (y=-180, 看得清楚)
    init_y_mm = cfg.get("init_y_mm", -180)

    try:
        # ===== 初始化步骤 1 (用户 20:53): PLACE 视觉对齐 — 一次性 step 1 =====
        # 用户: "开始place对齐（x=-320，只在任务第一次触发时定位0在哪）"
        # 1. 切 PLACE 姿态 (arm=+90, x=-320, y=-100, hand=0) — x=-320 是用户标定的
        #    视觉对齐位置 (比实际 PLACE 落点 x=-250 略收回)
        # 2. track_chassis("cylinder_set") — 用户 20:38 实机验证 33 帧 arrived
        # 3. 记录 align 完后 chassis 的 odom x 作为 "0 参考" — main loop 后面用
        #    此偏移算 S/T 实际 odom 目标 (避免从 PLACE 退到 S 时的 2.8m "反向跑")
        align_arrived = False
        align_odom_x = 0.0
        if cfg.get("chassis_align", {}).get("enabled", False):
            align_arrived, align_odom_x = _init_step1_place_align(arm_client, cfg)
        else:
            logger.info("step 1: PLACE visual align 已禁用 (chassis_align.enabled=False)")
            align_odom_x, _ = _odom_curr_x_y()

        # ===== 初始化步骤 2. (旧) X 编码器校准, 默认跳过 =====
        _init_step1_reset_x(arm_client)

        # ===== 初始化步骤 3. (旧) S 姿态 — 现在作为 pick 前切换的辅助 =====
        #   注意: 主循环 _pick_at_source 前也会调用 _init_step2_s_pose 来切换到 S 姿态.
        #   这里先跑一次是为了刷新视觉伺服起点 =====
        # (暂时跳过, 主循环已经处理)

        # ===== 主循环: 按 source_position_order 走底盘列 =====
        source_position_order = cfg["source_position_order"]
        target_slot_map = cfg["target_slot_map"]   # cylinder_N -> slot N (底盘位置)
        chassis_move_timeout = cfg["chassis_move_timeout_s"]

        last_chassis_col: Optional[int] = None

        # 2026-08-02 (3) (5): 真底盘位置记账 (米), 加并发 chassis+arm 调度.
        # 2026-08-02 (用户报 chassis 漂移不是直线):
        #   move_for([dx, 0, 0]) 是**开环**增量, 累计漂移 (上一轮跑完 x=1.40 y=0.31 theta=0.39,
        #   实际应该 x≈0.30 y≈0 theta=0).
        #   改用 move_to_position([target_x, curr_y, 0]) **闭环** (PID + odom feedback),
        #   自动纠 theta/y 漂移. 既然已知绝对目标, 不再需要 last_chassis_pos_m 记账.

        # 给 place 用的 PLACE 工作平面参数 (cfg 一次性读完)
        place_pose = cfg["arm_place_pose_T2"]
        place_arm   = float(place_pose["arm_angle_deg"])   # 90
        place_x_mm  = float(place_pose["x_mm"])            # -270 (用户撤回 -250 决定)
        place_hand  = float(place_pose["hand_angle_deg"])  # 0 (保持)
        s_arm       = float(cfg["arm_pick_pose"]["arm_angle_deg"])  # -90
        s_x_mm      = float(cfg["arm_pick_pose"]["x_mm"])           # -100

        def _odom_curr_x_y() -> Tuple[float, float]:
            """读 odom_state (轮编码器反馈) 拿 chassis 当前 x, y (m)."""
            try:
                resp = arm_client.http.get("/v1/realtime/odom/state", timeout=3)
                odom = (resp or {}).get("odom_state") or {}
                return float(odom.get("x", 0.0)), float(odom.get("y", 0.0))
            except Exception:
                return 0.0, 0.0

        def _chassis_goto(target_x_m: float) -> None:
            """闭环 chassis 到绝对 x (m). 保留当前 theta, **不校正**, 避免"看起来在转".

            走 move_to_position (SDK 闭环 PID + odom 反馈). target=[x, y, current_theta]
            保留当前 theta (不传 0). 用 requests.post 直接打 /v1/execute, 避免
            api_client.execute_car_action 的 sync 模式双打包问题.

            用户 (2026-08-02 21:00): "step2 应该是进入 S 姿态, 不是底盘动". 改: 如果
            |target - curr| < 5cm, 跳过 move_to_position (避免 theta 校正旋转).
            """
            curr_x, curr_y = _odom_curr_x_y()
            if abs(target_x_m - curr_x) < 0.05:
                logger.info("  底盘已在 %.3f m (距离 target %.3f < 5cm), 跳过移动",
                            curr_x, target_x_m)
                return
            # 用户 22:28: 保证和车道平行 (theta→0), 不然对不齐
            logger.info("  闭环底盘移动 odom=%.3f → target=%.3f m (y 锁 %.3f, theta→0)",
                        curr_x, target_x_m, curr_y)
            import requests as _req
            api_base = getattr(arm_client.http, "settings", None)
            api_base = api_base.api_base if api_base else "http://192.168.5.230:5050"
            try:
                resp = _req.post(
                    f"{api_base}/v1/execute",
                    json={
                        "target": "car",
                        "name": "move_to_position",
                        "args": [[target_x_m, curr_y, 0.0]],   # theta→0 保持平行
                        "kwargs": {},
                        "sync": True,
                        "timeout": chassis_move_timeout,
                    },
                    timeout=chassis_move_timeout + 5,
                )
                resp.raise_for_status()
                job = resp.json().get("job", {})
                logger.info("  move_to_position: %s", job.get("status"))
            except Exception as exc:
                logger.warning("  move_to_position 失败 (%s), 退路: 用 move_for 增量", exc)
                # 退路: 开环增量 (不如 PID 准, 但至少会动)
                dx_m = target_x_m - curr_x
                arm_client.http.execute_car_action(
                    "move_for", [dx_m, 0.0, 0.0],
                    timeout=chassis_move_timeout, sync=True,
                )

        def _parallel_chassis_arm(target_x_m: Optional[float],
                                 arm_kwargs: dict) -> None:
            """chassis 平移到绝对 x + arm composite_run 并发 (ThreadPoolExecutor)."""
            tasks = []
            with ThreadPoolExecutor(max_workers=2) as ex:
                if target_x_m is not None:
                    tasks.append(ex.submit(_chassis_goto, target_x_m))
                if arm_kwargs:
                    logger.info("  发起 arm composite_run: %s", arm_kwargs)
                    tasks.append(ex.submit(arm_client.composite_run,
                                            speed=100, timeout=15.0, **arm_kwargs))
                for t in tasks:
                    t.result()

        for i, column_idx in enumerate(source_position_order):
            curr_x, _ = _odom_curr_x_y()
            logger.info("=== 处理底盘列 %d (S%d, odom=%.3f m) ===",
                        i + 1, column_idx, curr_x)

            # (1) 底盘闭环移到本列源 (用 align_odom_x 偏移, 世界坐标 0,0.15,0.30 → odom)
            # 用户 (2026-08-02 21:02): "step2 不调用 chassis-goto, 只用动机械臂"
            # step 1 align 后 chassis 已经在 S1=T1 列. 第 0 列 (i=0) 跳过底盘移动, 直接 S 姿态.
            if i > 0:
                target_s_world = SOURCE_POSITIONS_M[column_idx]
                target_s = align_odom_x + target_s_world
                logger.info("  底盘 → S%d (world=%.3f → odom target %.3f m, align=%.3f)",
                            column_idx, target_s_world, target_s, align_odom_x)
                _chassis_goto(target_s)
            else:
                logger.info("  step 1 align 后 chassis 已在 S1 列 (odom %.3f), 跳过底盘移动", curr_x)
                # 用户 22:28: 第一列也要保证 theta≈0 (和车道平行)
                try:
                    odom = arm_client.http.get("/v1/realtime/odom/state", timeout=3).get("odom_state") or {}
                    th = float(odom.get("theta", 0.0))
                    if abs(th) > 0.02:
                        logger.info("  theta=%.3f 偏了, 修正→0", th)
                        arm_client.http.execute_car_action(
                            "move_to_position", [curr_x, float(odom.get("y", 0.0)), 0.0],
                            timeout=10.0, sync=True)
                except Exception:
                    pass

            # (1.5) 切 S 姿态 — x 用 move_x(v_max=100), arm/y/hand 用 composite_run 并发
            logger.info("  切 S 姿态: arm=-90° x=-80 y=-100 hand=0°")
            with ThreadPoolExecutor(max_workers=2) as ex:
                f_x = ex.submit(arm_client.move_x, -80.0, 100.0, 10.0, 15.0)  # v_max=100mm/s!
                f_ah = ex.submit(arm_client.composite_run,
                                 arm=-90.0, x_mm=None, y_mm=-100.0, hand=0.0,
                                 speed=100, timeout=15.0)
                f_x.result(); f_ah.result()

            # (2) 抓
            label = _pick_at_source(runner, arm_client, client, cfg, column_idx)
            completed.append(label)

            # (3) pick→place 连贯: 抬y + 底盘移T + 切PLACE姿态 三路并发 (用户 22:08: 不要等!)
            slot_idx = int(target_slot_map[label])
            target_t_world = SLOT_POSITIONS_M[slot_idx]
            target_t = align_odom_x + target_t_world
            place_x_override = float(cfg.get("place_x_overrides", {}).get(label, place_x_mm))
            logger.info("  → T%d (label=%s, x=%s) 并发: y↑ + 底盘 + PLACE姿态(x快)",
                        slot_idx, label, place_x_override)
            with ThreadPoolExecutor(max_workers=4) as ex:
                f_y = ex.submit(runner.client.move_y, -100.0, 10.0)
                f_chassis = ex.submit(_chassis_goto, target_t)
                f_x = ex.submit(arm_client.move_x, place_x_override, 100.0, 10.0, 15.0)
                f_ah = ex.submit(arm_client.composite_run,
                                  arm=place_arm, x_mm=None,
                                  y_mm=None, hand=place_hand,
                                  speed=100, timeout=15.0)
                f_y.result(); f_chassis.result(); f_x.result(); f_ah.result()

            # (5) 放: y→-20 → grasp → y→-100 (用户 22:08: 先抬y再动底盘, 不推倒种子!)
            logger.info("[T%d] place: y→-20 + grasp + y→-100", slot_idx)
            runner.client.move_y(-20.0, timeout=10.0)
            arm_client.grasp(False)
            runner.client.move_y(-100.0, timeout=10.0)   # 必须先抬y!

            # (6) 下一列: 底盘移动 + 切PLACE对齐姿态 并发 (y已抬好, 不会推倒)
            if i + 1 < len(source_position_order):
                next_col_idx = source_position_order[i + 1]
                next_source_world = SOURCE_POSITIONS_M[next_col_idx]
                next_source = align_odom_x + next_source_world
                logger.info("  底盘→S%d + 切PLACE对齐(x快)", next_col_idx)
                with ThreadPoolExecutor(max_workers=3) as ex:
                    f_chassis = ex.submit(_chassis_goto, next_source)
                    f_x = ex.submit(arm_client.move_x, -320.0, 100.0, 10.0, 15.0)
                    f_ah = ex.submit(arm_client.composite_run,
                                      arm=90.0, x_mm=None, y_mm=None, hand=0.0,
                                      speed=100, timeout=15.0)
                    f_chassis.result(); f_x.result(); f_ah.result()
                try:
                    from main.chassis import track_chassis, track_trace
                    marker_label = cfg.get("marker_label", "cylinder_set")
                    r = track_chassis(marker_label, dry_run=False, max_seconds=8.0, on_tick=track_trace(5))
                    logger.info("  对齐: arrived=%s frames=%d", r.arrived, r.frames)
                except Exception as exc:
                    logger.warning("  对齐失败 (%s), pass", exc)
            else:
                logger.info("  最后一列完成, 底盘留在 %.3f m", align_odom_x + SOURCE_POSITIONS_M[source_position_order[-1]])

    except Exception as exc:
        logger.exception("task1_seeding 失败: %s", exc)
        return {"ok": False, "completed": completed, "error": str(exc)}

    # 用户 22:03: reset 静默运行, 不等它执行完
    logger.info("task1 完成, 底盘→30 + reset 机械臂 (fire-and-forget)...")
    try:
        final_x = align_odom_x + SOURCE_POSITIONS_M[source_position_order[-1]]
        _chassis_goto(final_x)
        # reset 异步, 不阻塞返回
        import threading
        def _bg_reset():
            try:
                runner.client.move_y(-100.0, timeout=15.0)
                arm_client.move_x(0.0, 100.0, 10.0, 15.0)  # v_max=100mm/s
                runner.client.composite_run(arm=0.0, x_mm=None, y_mm=None, hand=-90.0, speed=100, timeout=15.0)
                arm_client.http.execute_arm_action("reset_position", timeout=30.0, sync=True)
            except Exception:
                pass
        threading.Thread(target=_bg_reset, daemon=True).start()
        logger.info("  底盘到 %.3f m, reset 后台运行中", final_x)
    except Exception as exc:
        logger.warning("  收尾失败 (%s), 跳过", exc)

    return {"ok": True, "completed": completed}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
    result = run()
    print("任务一 自动移苗 执行结果:", result)