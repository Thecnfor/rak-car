#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务二: 水塔取水 (向两个水塔投放冰块方块).

场地布局 (进入任务区并到达第一座水塔位置后):
  - 左侧 (X 负方向): 6 个水方块, 分 3 组, 每组 2 个, 组与组间隔 30cm
  - 右侧 (X 正方向): 2 座水塔, 间距 60cm
  - cam2 视觉识别水塔上的等级标 (water_l1/l2/l3 → 需 1/2/3 块)

进入第一座水塔时的初始姿态:
  - Y = -150 mm   (安全运输高度)
  - X = 0 mm
  - 大臂 arm = +90°  (正前方中位)
  - 手爪 hand = -90°  (竖直向上)

=== 安全约束 (由 main.arm.SafetyMixin 统一保证) ===
  - 大臂角度硬限: [-150°, +90°]
  - 手爪角度硬限: [-90°, 0°]
  - y 保护区: y > -30 mm 时拒绝 set_arm_angle/set_hand_angle/move_x
               (除非在 init 姿态 - 大臂 90°/0°/手爪 -90°/x∈[-300,-150])
  - 丢步核对: move_y/move_x 完成后实际值与目标值差距 > 阈值时打印警告

业务层不需要再写自己的 Rule A/B/C —— 这些约束已由 SafetyMixin 在每个
动作入口处自动校验。

单块方块完整流程 (从检测姿态 X=-160, Y=-30, arm=-95°, hand=-45° 开始):
  1. composite_run: X 收回 + Y 抬升到 -120 (并发)
  2. composite_run: 大臂转 +95° + X 伸出到方块抓取坐标 (并发)
  3. 手爪转 0° (Rule C: set_arm_angle 已内置 get_state 物理到位确认)
  4. cam2 视觉伺服 (pick_vision.enabled): track_velocity_pick 识别水立方 →
     定位到吸嘴正下方 → move_y 到吸附高度 -75 → 吸附 → 抬回
     (盲抓回退: move_y 到 -75 + grasp + move_y 到运输高度)
  5. move_y 到运输高度 -150
  6. composite_run: X 收回 + 大臂转 -95° + 手爪 -90° (并发)
  7. Y 梯度下降 (第 1/2/3 块深度不同) + composite_release + grasp off

架构说明 (2026-08 重构):
  本任务使用 main.arm.ArmRunner + CompositeMixin 编排动作,
  不再依赖 main/task/_helpers.py (该文件已删除). 自定义的 Rule A/B/C
  安全门已删除 —— 统一由 SafetyMixin 在 move_y/move_x/set_arm_angle/
  set_hand_angle 入口自动校验 y 保护区与角度硬限.
"""
from __future__ import annotations

import logging
import time
from typing import Any, Dict, List, Optional

from main.api_client import RuntimeApiClient
from main.arm import ArmClient, ArmRunner
from main.task._config import load_task_config

logger = logging.getLogger("task.task2_water_tower")

# 水塔等级标签 → 所需方块数
WATER_TOWER_LABELS = {"water_l1", "water_l2", "water_l3"}


# ── 辅助函数 ─────────────────────────────────────────────────

def _chassis_move_for(
    arm_client: ArmClient,
    dx_m: float,
    timeout: float,
) -> dict:
    """底盘纵向 move_for 阻塞调用 (sync=True 等结果).

    走 ChassisClient.move_for —— move_for 是底盘动作, 不应走
    ArmClient._call_car. 后者签名是 (name, timeout=20.0, *args, sync=False, **kwargs),
    第二个位置参数是 timeout, 写成 _call_car("move_for", dx_m, timeout=...)
    会把 dx_m 误绑给 timeout, 报 "multiple values for argument 'timeout'".
    """
    from main.chassis import ChassisClient
    chassis = ChassisClient.connect()
    try:
        return chassis.move_for(dx_m=dx_m, timeout=timeout)
    finally:
        chassis.close()


def _read_arm_state(arm_client: ArmClient) -> Dict[str, Any]:
    """读取实时 arm 状态 (走 /v1/execute get_arm_state).

    返回单位转换:
      SDK 原始返回: x/y 为米, arm_angle/hand_angle 为度
      本函数统一转换为业务单位: x_mm/y_mm 为毫米
    """
    try:
        raw = arm_client.get_state()
    except Exception as exc:
        logger.warning("[_read_arm_state] 获取失败: %s", exc)
        return {}
    out: Dict[str, Any] = {}
    if getattr(raw, "x_mm", None) is not None:
        out["x_mm"] = float(raw.x_mm)
    if getattr(raw, "y_mm", None) is not None:
        out["y_mm"] = float(raw.y_mm)
    if getattr(raw, "arm_angle", None) is not None:
        out["arm_angle"] = float(raw.arm_angle)
    if getattr(raw, "hand_angle", None) is not None:
        out["hand_angle"] = float(raw.hand_angle)
    return out


def _wait_arm_angle_reached(
    arm_client: ArmClient,
    target_deg: float,
    tolerance: float = 3.0,
    timeout: float = 10.0,
) -> None:
    """轮询 arm_state 直到大臂物理到达目标角度.

    业务层校验: SafetyMixin.set_arm_angle 已自动校验角度硬限, 但大臂运动
    异步完成时, 下游动作需要在物理到位后再进行. 本函数通过 get_state()
    轮询实际角度, 与目标值差距 < tolerance 时返回.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        cur = _read_arm_state(arm_client).get("arm_angle")
        if cur is not None and abs(cur - target_deg) <= tolerance:
            logger.info("大臂物理到位: %.1f° (目标 %.0f° ± %.0f°)", cur, target_deg, tolerance)
            return
        time.sleep(0.15)
    raise RuntimeError("大臂角度在 {:.0f}s 内未到达 {:.0f}°".format(timeout, target_deg))


def _detect_tower_count(client: RuntimeApiClient) -> Optional[int]:
    """cam2 识别水塔等级标签, 返回需要的方块数.

    仅轮询 1 秒. 识别到等级标 → 返回块数 (water_l1→1, water_l2→2, water_l3→3);
    没识别到 → 返回 None (调用方决定底盘前移重试或兜底取 1 块, 不崩溃).
    """
    count_map = {"water_l1": 1, "water_l2": 2, "water_l3": 3}
    deadline = time.time() + 1.0
    while time.time() < deadline:
        try:
            resp = client.get("/v1/realtime/vision/task", timeout=2)
        except Exception:
            time.sleep(0.2)
            continue
        if not isinstance(resp, dict) or not resp.get("ok"):
            time.sleep(0.2)
            continue
        task_state = resp.get("task_state") or {}
        if not task_state.get("active"):
            time.sleep(0.2)
            continue
        for d in task_state.get("detections") or []:
            label = (d or {}).get("label", "")
            if label in WATER_TOWER_LABELS:
                n = count_map[label]
                logger.info("水塔识别 %s → 需要 %d 块", label, n)
                return n
        time.sleep(0.2)
    logger.warning("cam2 未识别到水塔等级标")
    return None


def _align_to_tower(
    arm_client: ArmClient,
    track_cfg: Dict[str, Any],
) -> Any:
    """cam2 视觉闭环: 把底盘纵向对齐到水塔等级标前后到位 (track_chassis).

    track_chassis 阻塞式跑闭环, 把 water 等级标 bbox 中心拉到 setpoint_cxcy.
    vx_only=True (规定, 2026-08-03) → 只控 vx(前后), vy 恒 0, 横向不做闭环.
    退出时零速命令是异步的 → 显式停稳 (task1_seeding.py:337-344 的范式).
    """
    from main.chassis import track_chassis  # 懒加载, 与 task1 一致

    result = track_chassis(
        track_cfg.get("target", "water"),
        setpoint_cxcy=tuple(track_cfg.get("setpoint_cxcy", (0.0, 0.0))),
        # 只控前后 (任务二规定), vy 恒 0, 到达只看 cx
        vx_only=bool(track_cfg.get("vx_only", False)),
        # 轴符号从配置来 (水塔场景方向实测与 cylinder 默认相反, 2026-08-03)
        sign_vx=int(track_cfg.get("sign_vx", -1)),
        sign_vy=int(track_cfg.get("sign_vy", 1)),
        kp=track_cfg.get("kp", 0.50),
        v_max=track_cfg.get("v_max", 0.25),
        v_slew=track_cfg.get("v_slew", 0.02),  # 每帧速度变化限幅 (2026-08-03 实测太急 → 调缓)
        deadband=track_cfg.get("deadband", 0.08),
        hold_frames=track_cfg.get("hold_frames", 3),
        max_lost_frames=track_cfg.get("max_lost_frames", 30),
        max_seconds=track_cfg.get("max_seconds", 6.0),
    )
    logger.info("track_chassis result: arrived=%s reason=%s frames=%d",
                result.arrived, result.reason, result.frames)
    _stop_chassis(arm_client)
    return result


def _stop_chassis(arm_client: ArmClient) -> None:
    """对齐结束后显式把底盘停稳 (track_chassis 零速异步, 防止漂移)."""
    try:
        arm_client.http.post(
            "/v1/realtime/chassis-velocity",
            {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            timeout=2.0,
        )
    except Exception:
        pass
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        try:
            resp = arm_client.http.get("/v1/realtime/wheels/speeds", timeout=1)
            speeds = (resp or {}).get("speeds") or []
            if all(abs(float(s)) < 0.01 for s in speeds):
                break
        except Exception:
            break
        time.sleep(0.1)
    time.sleep(0.2)


# ── 核心动作子流程 ────────────────────────────────────────────────

def _pick_cube(
    arm_client: ArmClient,
    runner: ArmRunner,
    cfg: Dict[str, Any],
    cube_x_mm: float,
) -> None:
    """抓取单个水方块 (不含投放).

    手爪转 0° 后走 cam2 视觉伺服 (2026-08-03, pick_vision.enabled=true):
      1. composite_run: 大臂 → +95° + X → cube_x_mm + Y → servo_y_mm (并发)
      2. 手爪 → 0° (末端朝下, cam2 能看到吸嘴正下方的水立方)
      3. 轮询 get_state 确认大臂物理到位 (Rule C 业务层校验)
      4. runner.track_velocity_pick: cam2 识别水立方 → 视觉定位到吸嘴正下方
         (velocity 模式, 免 arm_queue) → move_y 到吸附高度 → 吸附 → 抬回
      5. 补 move_y 到运输高度 (与盲抓路径一致的结束位, 供后续 _deliver_cube)

    盲抓回退 (pick_vision.enabled=false):
      4'. move_y 到吸取高度 + grasp + 等待真空稳定 + move_y 抬升到运输高度
    """
    pick = cfg["pick_pose"]
    vision = cfg.get("pick_vision") or {}

    # 2026-08-04: 抓取 init 姿态 X 由 cube_x_mm 决定 (per-cube, 用 first/second_cube_x_mm).
    # pick_pose 不再单独定义 x_mm. 视觉伺服从方块物理 X 起步 (cam2 装在 X 滑台上,
    # 臂在水方块正上方时 bbox 中心始终 = setpoint_cxcy, 与 X 起点无关).
    init_x_mm = float(cube_x_mm)

    # 1) 复合动作: arm 转 + X 伸出 + Y 抬升并发 (走 init 姿态)
    runner.client.composite_run(
        arm=float(pick["arm_angle_deg"]),
        x_mm=init_x_mm,
        y_mm=float(vision.get("servo_y_mm", pick["y_transition_mm"])),
    )

    # 2) 手爪转 0°
    arm_client.set_hand_angle(float(pick["hand_angle_deg"]), speed=80, timeout=10.0)

    # 3) 业务层校验大臂物理到位
    _wait_arm_angle_reached(arm_client, pick["arm_angle_deg"])

    if not vision.get("enabled"):
        # ---- 盲抓回退: 固定姿态下降 + 吸附 + 抬升 ----
        runner.move_y(float(pick["y_descend_mm"]))
        runner.grasp(on=True)
        time.sleep(cfg["vacuum_settle_s"])
        runner.move_y(float(pick["y_lift_mm"]))
        return

    # ---- cam2 视觉伺服抓水立方 ----
    # 2026-08-04: setpoint_cxcy 从 pick_vision 透传, 覆盖 arm_origin.yaml 的默认标定.
    # 标定方法: 手动把臂摆到 pick_pose init 姿态 (x=-150, y=-150, arm=90, hand=-10),
    # 把水方块放在吸嘴正下方, curl 5 帧取 bbox_norm 平均.
    sp = vision.get("setpoint_cxcy")
    sp_x = float(sp[0]) if (sp and len(sp) >= 1) else None
    sp_y = float(sp[1]) if (sp and len(sp) >= 2) else None
    result = runner.track_velocity_pick(
        vision.get("label", "water"),
        x_start=init_x_mm,
        y_start=float(vision.get("servo_y_mm", pick["y_transition_mm"])),
        arm_start=float(pick["arm_angle_deg"]),
        hand_start=float(pick["hand_angle_deg"]),
        grasp_y_mm=float(vision.get("grasp_y_mm", pick["y_descend_mm"])),
        timeout=float(vision.get("timeout", 15.0)),
        hz=float(vision.get("hz", 20.0)),
        gain_arm=float(vision.get("gain_arm", 0.4)),
        gain_x=float(vision.get("gain_x", 0.08)),
        deadzone=float(vision.get("deadzone", 0.03)),
        max_vel=float(vision.get("max_vel", 0.20)),
        sign_arm=float(vision.get("sign_arm", 1.0)),
        sign_x=float(vision.get("sign_x", -1.0)),
        arm_min=vision.get("arm_min"),
        arm_max=vision.get("arm_max"),
        setpoint_x_norm=sp_x,
        setpoint_y_norm=sp_y,
        descend_hand_deg=vision.get("descend_hand_deg"),  # 2026-08-04: 下降到位后手爪转 0°
        settle_hits=int(vision.get("settle_hits", 3)),
        hold_s=float(vision.get("hold_s", 0.3)),
        lift_back=True,
        # (1)(2) 已摆好 S 姿态 (手爪 -10°), 跳过 runner 内部重复 composite_run
        skip_pose_align=True,
    )
    if not result.get("ok"):
        raise RuntimeError(
            f"cam2 视觉抓水立方失败 (reason={result.get('reason')}, "
            f"trace_hits={result.get('trace_hits')}, end_arm={result.get('end_arm')})"
        )
    # track_velocity_pick 已抬回 servo_y_mm; 补一次 move_y 到运输高度 (与盲抓路径一致)
    lift_y = float(pick.get("y_lift_mm", -150.0))
    servo_y = float(vision.get("servo_y_mm", pick["y_transition_mm"]))
    if abs(lift_y - servo_y) > 1.0:
        runner.move_y(lift_y)


def _deliver_cube(
    arm_client: ArmClient,
    runner: ArmRunner,
    cfg: Dict[str, Any],
    cube_index: int = 0,
) -> None:
    """将吸住的水方块投放到水塔内.

    执行顺序 (2026-08-04 改, 防撞塔):
      0. (前置: run() 在 chassis 回塔前已 runner.move_x(detection_pose.x_mm)=-200)
      1. 阶段1: composite_run 转大臂到 -95° + 手爪 -90° + 降 Y (X 留在 -200 不动)
                → 此时水方块在车体 back-left, 远离 +X 侧水塔, 旋转零碰撞
      2. 阶段2: move_x 到投递位 -105 (大臂已锁 -95°, 水方块沿车体左侧平移, 不甩)
      3. grasp off 释放方块
    """
    carry = cfg["carry_pose"]
    deliver_ys = cfg.get("deliver_y_by_index", [-50.0, -65.0, -80.0])
    deliver_y = deliver_ys[min(cube_index, len(deliver_ys) - 1)]

    safe_x_mm = float(cfg["detection_pose"]["x_mm"])   # -200 (init/检测位, 远离水塔)
    deliver_x_mm = float(carry["x_mm"])                 # -105 (投递位)

    # 阶段 1: 大臂转 + 手爪转 + Y 降并发, X 留在 safe 位置 (-200)
    runner.client.composite_run(
        arm=float(carry["arm_angle_deg"]),
        hand=float(carry["hand_angle_deg"]),
        x_mm=safe_x_mm,
        y_mm=float(deliver_y),
    )

    # 阶段 2: 大臂已 -95° (水方块在 back-left), 平移 X 到投递位
    if abs(deliver_x_mm - safe_x_mm) > 1.0:
        runner.move_x(deliver_x_mm)

    # 关真空释放方块
    runner.grasp(on=False)


# ── 主入口 ────────────────────────────────────────────────────────

def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """任务二主入口: 水塔取水 (2 座水塔 × N 块水方块投放).

    主流程:
      初始化: X 收至 detection_pose.x_mm → 大臂转检测角 + 手爪检测角 (检测姿态)
      对每座水塔循环:
        1) Y 下降到 detection_pose.y_mm → cam2 识别水塔等级 (需几块)
        2) track_chassis 视觉闭环把底盘横向对齐到水塔等级标居中
        3) 按块循环: 底盘到方块组 → 抓块 → 底盘回水塔 → 梯度投放
        4) 结束后底盘前进 tower_spacing_m 到下一座水塔

    Args:
        client: 可选 RuntimeApiClient, None 时内部新建

    Returns:
        Dict: {"ok": bool, "completed": [处理过的水塔列表], "error": str}
    """
    cfg = load_task_config("water_tower_task")
    if cfg.get("placeholder"):
        raise NotImplementedError("water_tower_task 配置尚未完成")

    if client is None:
        client = RuntimeApiClient()
    client.wait_until_ready(timeout=30.0)

    # 初始化机械臂客户端与执行器
    arm_client = ArmClient.connect()
    if not arm_client.ping():
        raise RuntimeError("机械臂 runtime 未在线, 请检查 arm_feed 守护进程")
    runner = ArmRunner(arm_client)

    completed: List[str] = []
    detection = cfg["detection_pose"]
    track_cfg = cfg.get("track_align", {})
    timeout = cfg["chassis_move_timeout_s"]
    group_forward_m = cfg["group_forward_m"]
    tower_spacing_m = cfg.get("tower_spacing_m", 0.65)
    detect_retry_step_m = cfg.get("detect_retry_step_m", 0.10)
    detect_retry_max = cfg.get("detect_retry_max", 1)
    x_target_mm = float(detection["x_mm"])

    try:
        # ===== 初始化: 摆好检测姿态 =====
        logger.info("初始化: X 收至 %.0f mm (100mm/s)", x_target_mm)
        # 2026-08-04: 初始化 X 速度 100mm/s (其他 move_x 走默认 40)
        runner.move_x(x_target_mm, v_max_mms=100)

        logger.info("初始化: 大臂=%s°, 手爪=%s°",
                    detection["arm_angle_deg"], detection["hand_angle_deg"])
        runner.set_arm_angle(float(detection["arm_angle_deg"]), speed=80)
        # 2026-08-04: 用 _wait_arm_angle_reached 等物理到位, 替代 time.sleep(2.0);
        # task1 结束后大臂在 +90°, 要转到 -95° 是 185° / 80°/s = 2.3s, sleep 2s 不到位.
        _wait_arm_angle_reached(arm_client, float(detection["arm_angle_deg"]),
                                tolerance=5.0, timeout=10.0)
        arm_client.set_hand_angle(float(detection["hand_angle_deg"]), speed=80, timeout=10.0)
        time.sleep(0.5)

        for tower_idx, tower_label in enumerate(cfg["source_position_order"]):
            logger.info("=== 处理水塔 %s (第 %d 座) ===", tower_label, tower_idx + 1)

            # 第 2 座塔: 底盘前进 tower_spacing_m → 重新摆检测姿态
            if tower_idx > 0:
                logger.info("底盘: 从第 %d 座到水塔 %s (前进 %.2f m)",
                            tower_idx, tower_label, tower_spacing_m)
                runner.move_x(x_target_mm)
                _chassis_move_for(arm_client, tower_spacing_m, timeout=timeout)
                runner.move_x(x_target_mm)
                # 2026-08-04: 跨塔前必须把大臂拉回 detection.arm_angle_deg.
                # 上一塔最后一块抓失败时大臂会留在 +95° (pick_pose), 不拉回去
                # cam2 朝前方看 → 看不到水塔等级标 → 识别失败.
                logger.info("恢复大臂=%s° 检测姿态 (水塔 %s)",
                            detection["arm_angle_deg"], tower_label)
                runner.set_arm_angle(float(detection["arm_angle_deg"]), speed=80)
                _wait_arm_angle_reached(arm_client, float(detection["arm_angle_deg"]),
                                        tolerance=5.0, timeout=10.0)
                logger.info("恢复手爪 %s° 检测姿态 (水塔 %s)",
                            detection["hand_angle_deg"], tower_label)
                arm_client.set_hand_angle(float(detection["hand_angle_deg"]), speed=80, timeout=10.0)
                time.sleep(0.5)

            # 下降 Y 到检测高度
            logger.info("Y 下降到 %.0fmm 执行检测", detection["y_mm"])
            try:
                runner.move_y(float(detection["y_mm"]))
                time.sleep(0.3)
            except Exception:
                logger.warning("Y 下降失败, 跳过水塔 %s", tower_label)
                continue

            # 识别需几块; 未识别到等级标 → 底盘前移 detect_retry_step_m 再检测
            # (2026-08-03 用户规定: cam2 没看到第一个水塔等级标 → 前进 0.1m 再 cam2)
            needed = _detect_tower_count(client)
            retry = 0
            while needed is None and retry < detect_retry_max:
                retry += 1
                logger.info("cam2 未识别到水塔 %s 等级标, 底盘前移 %.2f m 再检测 (第 %d/%d 次)",
                            tower_label, detect_retry_step_m, retry, detect_retry_max)
                # 前移用 main.chassis.ChassisClient.move_for (2026-08-03 用户规定)
                from main.chassis import ChassisClient
                chassis = ChassisClient.connect()
                try:
                    chassis.move_for(detect_retry_step_m, timeout=timeout)
                finally:
                    chassis.close()
                needed = _detect_tower_count(client)
            if needed is None:
                logger.warning("水塔 %s 重试后仍未识别到等级标, 兜底取 1 块", tower_label)
                needed = 1
            logger.info("水塔 %s 需投放 %d 块水方块", tower_label, needed)

            # cam2 视觉闭环: 把底盘横向对齐到水塔等级标居中
            _align_to_tower(arm_client, track_cfg)

            # 对齐后的底盘位置视为水塔原点
            chassis_at_tower_m = 0.0  # 底盘相对水塔原点的偏移 (m): >0 前进, <0 后退
            picked = 0
            first_x = cfg["first_cube_x_mm"]
            second_x = cfg["second_cube_x_mm"]
            # 第一座水塔: 方块组在水塔前方 → 向前拿; 后续水塔: 方块组在后方 → 向后拿
            direction = 1.0 if tower_idx == 0 else -1.0

            while picked < needed:
                try:
                    group = picked // 2  # 每 2 块一组
                    target_offset = direction * group * group_forward_m

                    # 底盘移动到对应方块组位置
                    d = target_offset - chassis_at_tower_m
                    if abs(d) > 1e-3:
                        runner.move_x(x_target_mm)
                        logger.info("底盘移动 %.2f m → 第 %d 组方块", d, group + 1)
                        _chassis_move_for(arm_client, d, timeout=timeout)
                        chassis_at_tower_m = target_offset

                    # 抓块: 组内第 1 块用 first_x, 第 2 块用 second_x
                    pick_x = first_x if (picked % 2 == 0) else second_x
                    logger.info("抓取第 %d 块, X=%s mm (第 %d 组)",
                                picked + 1, pick_x, group + 1)
                    _pick_cube(arm_client, runner, cfg, pick_x)

                    # 抓完块, 底盘回水塔正前方
                    if abs(chassis_at_tower_m) > 1e-3:
                        logger.info("底盘后退 %.2f m → 回水塔位置", -chassis_at_tower_m)
                        _chassis_move_for(arm_client, -chassis_at_tower_m, timeout=timeout)
                        chassis_at_tower_m = 0.0

                    # 投放
                    _deliver_cube(arm_client, runner, cfg, cube_index=picked)
                    runner.move_x(x_target_mm)
                except Exception:
                    logger.exception("第 %d 块失败, 继续下一块", picked + 1)
                picked += 1

            completed.append("tower_{}".format(tower_label))

    except Exception as exc:
        logger.exception("water_tower_task 失败: %s", exc)
        return {"ok": False, "completed": completed, "error": str(exc)}

    return {"ok": True, "completed": completed}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
    result = run()
    print("任务二 水塔取水 执行结果:", result)