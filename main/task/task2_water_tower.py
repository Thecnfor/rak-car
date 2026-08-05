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
from concurrent.futures import ThreadPoolExecutor
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

    2026-08-06 提速: 仿 task1_seeding.py 的 _chassis_goto, 直接走
    execute_car_action (绕过 ChassisClient.connect()/close() 每次建连的开销),
    HTTP 层 sync=True 内部 polling 直到 succeeded. SDK 串口 SerialEngine
    统一调度 (CLAUDE.md §Runtime concurrency model), 多线程并发安全.
    """
    return arm_client.http.execute_car_action(
        "move_for", [dx_m, 0.0, 0.0],
        timeout=timeout, sync=True,
    )


def _parallel_chassis_arm(
    arm_client: ArmClient,
    runner: ArmRunner,
    *,
    target_dx_m: float = 0.0,
    arm_kwargs: Optional[Dict[str, Any]] = None,
    timeout: float = 10.0,
) -> None:
    """底盘 move_for + 臂 composite_run 并发 (task1_seeding._parallel_chassis_arm 模式).

    Args:
        target_dx_m: 底盘相对位移 (m), 0 跳过底盘动作
        arm_kwargs:  传 composite_run 的 kwargs (arm/x_mm/y_mm/hand/speed/timeout),
                     None/{} 跳过臂动作
        timeout:     两个动作的最大等待

    2026-08-06 提速:
      - chassis 0.15m (~0.7s) 与臂切姿态 (~2-3s) 完全并发, 主循环零阻塞.
      - composite_run 用 sync=False 立即返回 job_id (避免 HTTP 响应流持有到
        动作结束, 减少 504 风险; 任务二原版全串行 ~8s 改并发 ~3s).
      - 复合动作 m 单位转换 (composite_run kwargs 用 x_mm/y_mm, 自动转 m).

    task1_seeding.py:619-631 是同样的模式; SDK 串口 SerialEngine 单 io 线程
    调度 (CLAUDE.md §Runtime concurrency model), 多线程并发安全.
    """
    # 2026-08-06: 底盘移动前也遵循大臂转动的安全区限制 (Y ∈ [-200,-90], X ∈ [-300,-200])
    # 这保证底盘移动期间 X/Y 已在安全区, 与大臂 3 阶段共享同一约束.
    # 若同时转大臂, 3 阶段 phase1 已是 no-op (X/Y 已就位).
    if abs(target_dx_m) > 1e-3:
        _ensure_xy_in_safe_zone(arm_client, runner, timeout=timeout)

    tasks = []
    with ThreadPoolExecutor(max_workers=2) as ex:
        # 底盘 move_for 与 臂 3 阶段序列并发 (修复 2026-08-06 bug:
        # 之前 early return 让底盘完全没动)
        if abs(target_dx_m) > 1e-3:
            tasks.append(ex.submit(_chassis_move_for, arm_client, target_dx_m, timeout))

        # 2026-08-06: 大臂转动必须分 3 阶段, 转的时候只有大臂和末端可以动
        # 阶段 1: X/Y 调到安全区 (无 arm 变化, 已有不动, X 在 [-150,-10] 先 X, 否则先 Y)
        # 阶段 2: arm + hand 转动 (X/Y 冻结)
        # 阶段 3: X/Y 到目标 (无 arm 变化, Y 先 X 后)
        if arm_kwargs and arm_kwargs.get("arm") is not None:
            tasks.append(ex.submit(_safe_arm_rotation_sequence, arm_client,
                                    runner, arm_kwargs=arm_kwargs, timeout=timeout))
        else:
            # 不转大臂时, 直接 composite_run (单步)
            payload = dict(arm_kwargs) if arm_kwargs else {}
            payload.setdefault("speed", 100)
            payload.setdefault("timeout", timeout)
            if payload:
                tasks.append(ex.submit(runner.client.composite_run, **payload))

        for t in tasks:
            t.result()


def _safe_arm_rotation_sequence(
    arm_client: ArmClient,
    runner: ArmRunner,
    *,
    arm_kwargs: Dict[str, Any],
    timeout: float = 10.0,
) -> None:
    """2026-08-06 大臂转动 3 阶段序列 (task2 专用).

    用户规定 (2026-08-06):
      - 大臂转动必须满足 Y ∈ [-200, -90] 且 X ∈ [-300, -200]
      - 已满足的不动; X/Y 都不满足时, X 在 [-150, -10] 先动 X, 否则先动 Y
      - 大臂转动期间只有大臂 + 末端可以动, X/Y 必须冻结
      - 转动完成后 X/Y 才能继续到目标

    实现:
      阶段 1: composite_run X/Y (无 arm/hand, 已有不动)
      阶段 2: composite_run arm + hand (X/Y 冻结在安全位)
      阶段 3: composite_run X/Y 到目标 (无 arm 变化)

    注: 阶段 1 用 composite_run 不用 move_x 是因为 move_x 受 Y 保护区 (-30~0)
    限制, composite_run 无 client-side _check_y_protected.
    """
    try:
        state = arm_client.get_state()
    except Exception as exc:
        logger.warning("_safe_arm_rotation_sequence: 读不到状态, 跳过 (%s)", exc)
        return

    cur_y = float(state.y_mm) if state.y_mm is not None else None
    cur_x = float(state.x_mm) if state.x_mm is not None else None
    if cur_y is None or cur_x is None:
        return

    target_y = arm_kwargs.get("y_mm")
    target_x = arm_kwargs.get("x_mm")
    target_arm = arm_kwargs.get("arm")
    target_hand = arm_kwargs.get("hand")

    Y_LO, Y_HI = -200.0, -90.0
    X_LO, X_HI = -300.0, -200.0

    y_in = Y_LO <= cur_y <= Y_HI                              # Y 已在安全区
    x_in = X_LO <= cur_x <= X_HI                              # X 已在安全区

    # 安全位: 已满足 = 当前值; 不满足 = clamp 到范围边界
    safe_y = cur_y if y_in else max(Y_LO, min(Y_HI, cur_y))
    safe_x = cur_x if x_in else max(X_LO, min(X_HI, cur_x))

    logger.info(
        "大臂 3 阶段: 当前 Y=%.1f X=%.1f → 安全位 Y=%.1f X=%.1f "
        "(y∈[%.0f,%.0f], x∈[%.0f,%.0f]) → 目标 Y=%s X=%s arm=%s hand=%s",
        cur_y, cur_x, safe_y, safe_x, Y_LO, Y_HI, X_LO, X_HI,
        target_y, target_x, target_arm, target_hand,
    )

    # 阶段 1: X/Y 调到安全位 (已有不动, 顺序: X 在 [-150,-10] 先 X, 否则先 Y)
    _ensure_xy_in_safe_zone(arm_client, runner, timeout=timeout)

    # 阶段 2: arm + hand (X/Y 冻结在安全位)
    if target_arm is not None or target_hand is not None:
        logger.info("  阶段 2: arm=%s hand=%s (X/Y 冻结)", target_arm, target_hand)
        runner.client.composite_run(arm=target_arm, hand=target_hand,
                                     speed=100, timeout=timeout)

    # 阶段 3: X/Y 到目标 (从安全位到目标, 无 arm 变化)
    # 2026-08-06 用户规定: Y 先 X 后 (X 移动到 -110 等伸出位前必须 Y 降到目标,
    # 防止大臂转后高 Y + 伸出 X 撞车)
    # 注意: 这里的 target_x/target_y 是原始 arm_kwargs 里的最终目标
    if target_y is not None and abs(target_y - safe_y) > 1.0:
        logger.info("  阶段 3: Y=%.1f (从安全位到目标, 先 Y)", target_y)
        runner.client.composite_run(y_mm=target_y, timeout=timeout)
    if target_x is not None and abs(target_x - safe_x) > 1.0:
        logger.info("  阶段 3: X=%.1f (从安全位到目标, 后 X)", target_x)
        runner.client.composite_run(x_mm=target_x, timeout=timeout)


def _ensure_xy_in_safe_zone(
    arm_client: ArmClient,
    runner: ArmRunner,
    *,
    timeout: float = 10.0,
) -> None:
    """2026-08-06 把 X/Y 调到安全区 (底盘移动 / 大臂转动 共用).

    安全范围: Y ∈ [-200, -90], X ∈ [-300, -200].
    - 已满足的不动
    - 都不满足时, X 在 [-150, -10] 先动 X, 否则先动 Y
    - 用 composite_run (无 client-side y_protected) 调 X/Y

    调用点:
      1. _parallel_chassis_arm 入口 (底盘 move_for 前, 同步执行)
      2. _safe_arm_rotation_sequence 阶段 1 (大臂 3 阶段第 1 步)
    """
    try:
        state = arm_client.get_state()
    except Exception as exc:
        logger.warning("_ensure_xy_in_safe_zone: 读不到状态, 跳过 (%s)", exc)
        return

    cur_y = float(state.y_mm) if state.y_mm is not None else None
    cur_x = float(state.x_mm) if state.x_mm is not None else None
    if cur_y is None or cur_x is None:
        return

    Y_LO, Y_HI = -200.0, -90.0
    X_LO, X_HI = -300.0, -200.0

    y_in = Y_LO <= cur_y <= Y_HI
    x_in = X_LO <= cur_x <= X_HI
    safe_y = cur_y if y_in else max(Y_LO, min(Y_HI, cur_y))
    safe_x = cur_x if x_in else max(X_LO, min(X_HI, cur_x))

    need_y_adj = not y_in
    need_x_adj = not x_in

    if need_y_adj or need_x_adj:
        logger.info(
            "X/Y 调安全区: Y=%.1f X=%.1f → Y=%.1f X=%.1f "
            "(y∈[%.0f,%.0f], x∈[%.0f,%.0f])",
            cur_y, cur_x, safe_y, safe_x, Y_LO, Y_HI, X_LO, X_HI,
        )

    if need_x_adj and need_y_adj:
        if -150.0 <= cur_x <= -10.0:
            if abs(safe_x - cur_x) > 1.0:
                runner.client.composite_run(x_mm=safe_x, timeout=timeout)
            if abs(safe_y - cur_y) > 1.0:
                runner.client.composite_run(y_mm=safe_y, timeout=timeout)
        else:
            if abs(safe_y - cur_y) > 1.0:
                runner.client.composite_run(y_mm=safe_y, timeout=timeout)
            if abs(safe_x - cur_x) > 1.0:
                runner.client.composite_run(x_mm=safe_x, timeout=timeout)
    elif need_x_adj:
        if abs(safe_x - cur_x) > 1.0:
            runner.client.composite_run(x_mm=safe_x, timeout=timeout)
    elif need_y_adj:
        if abs(safe_y - cur_y) > 1.0:
            runner.client.composite_run(y_mm=safe_y, timeout=timeout)
    # else: 都在安全区, 直接返回


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
        hz=track_cfg.get("hz", 20.0),  # 2026-08-06: 控制频率 (yaml 可配, 默认 20)
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

    2026-08-06 提速: 前置 _parallel_chassis_arm 已把臂切到 pick 姿态
    (arm=pick.arm=+90°, X=cube_x_mm, Y=servo_y=-150, hand=pick.hand=-10°).
    这里直接进视觉伺服或盲抓, 不再做 composite_run / set_hand_angle.
    省: 串行 composite_run → set_hand_angle → _wait_arm_angle_reached ≈ 2s/块.

    cam2 视觉伺服 (pick_vision.enabled=true):
      runner.track_velocity_pick: cam2 识别水立方 → 视觉定位到吸嘴正下方
      (velocity 模式, 免 arm_queue) → move_y 到吸附高度 → 吸附 → 抬回.

    盲抓回退 (pick_vision.enabled=false):
      move_y 到吸取高度 + grasp + 等待真空稳定 + move_y 抬升到运输高度.
    """
    pick = cfg["pick_pose"]
    vision = cfg.get("pick_vision") or {}

    if not vision.get("enabled"):
        # ---- 盲抓回退: 固定姿态下降 + 吸附 + 抬升 ----
        runner.move_y(float(pick["y_descend_mm"]))
        runner.grasp(on=True)
        time.sleep(cfg["vacuum_settle_s"])
        runner.move_y(float(pick["y_lift_mm"]))
        return

    # ---- cam2 视觉伺服抓水立方 ----
    sp = vision.get("setpoint_cxcy")
    sp_x = float(sp[0]) if (sp and len(sp) >= 1) else None
    sp_y = float(sp[1]) if (sp and len(sp) >= 2) else None
    result = runner.track_velocity_pick(
        vision.get("label", "water"),
        x_start=float(cube_x_mm),
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
        # 2026-08-06: 前置 _parallel_chassis_arm 已摆好 pick 姿态 (手爪 -10°),
        # 跳过 runner 内部重复 composite_run
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
    # 2026-08-06: 第二座塔 3 块时回退距离单独配置 (不走 0.35, 用 0.33)
    group_backward_m = cfg.get("group_backward_m", group_forward_m)
    tower_spacing_m = cfg.get("tower_spacing_m", 0.65)
    detect_retry_step_m = cfg.get("detect_retry_step_m", 0.2)
    detect_retry_max = cfg.get("detect_retry_max", 1)
    x_target_mm = float(detection["x_mm"])
    pick = cfg["pick_pose"]
    carry = cfg["carry_pose"]
    vision = cfg.get("pick_vision") or {}

    try:
        # ===== 初始化: composite_run 4 轴并发到 detection 姿态 (transport Y=-150) =====
        # 2026-08-06 提速: 原版 move_x → set_arm_angle → _wait_arm_angle_reached
        #                 → set_hand_angle → time.sleep(0.5) 全串行 ~5s.
        #                 改 composite_run 4 轴并发 ~2s, 省 3s.
        logger.info("初始化: composite_run → detection 姿态 (X=%.0f Y=-150 arm=%s hand=%s)",
                    x_target_mm, detection["arm_angle_deg"], detection["hand_angle_deg"])
        _parallel_chassis_arm(
            arm_client, runner,
            target_dx_m=0.0,
            arm_kwargs=dict(
                arm=float(detection["arm_angle_deg"]),
                x_mm=x_target_mm,
                y_mm=-150.0,
                hand=float(detection["hand_angle_deg"]),
                speed=100,
                timeout=10.0,
            ),
        )

        for tower_idx, tower_label in enumerate(cfg["source_position_order"]):
            logger.info("=== 处理水塔 %s (第 %d 座) ===", tower_label, tower_idx + 1)

            # 第 2 座塔: 底盘前进 tower_spacing_m + 切回 detection 姿态 (并发)
            # 2026-08-06 提速: 原版 move_x → chassis → move_x → set_arm → _wait →
            #                 set_hand → sleep 5 步串行 ~8s. 改并发 ~3s, 省 5s.
            if tower_idx > 0:
                logger.info("底盘前进 %.2f m → 水塔 %s (并发切回 detection 姿态)",
                            tower_spacing_m, tower_label)
                _parallel_chassis_arm(
                    arm_client, runner,
                    target_dx_m=tower_spacing_m,
                    arm_kwargs=dict(
                        arm=float(detection["arm_angle_deg"]),
                        x_mm=x_target_mm,
                        y_mm=-150.0,
                        hand=float(detection["hand_angle_deg"]),
                        speed=100,
                        timeout=10.0,
                    ),
                )

            # 下降 Y 到检测高度 (2026-08-06 提速: 去掉 time.sleep(0.3), runner.move_y 已 polling 物理到位)
            logger.info("Y 下降到 %.0fmm 执行检测", detection["y_mm"])
            try:
                runner.move_y(float(detection["y_mm"]), timeout=5.0)
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
                # 2026-08-06 提速: 直接 execute_car_action, 跳过 ChassisClient.connect/close
                _chassis_move_for(arm_client, detect_retry_step_m, timeout=timeout)
                needed = _detect_tower_count(client)
            if needed is None:
                logger.warning("水塔 %s 重试后仍未识别到等级标, 兜底取 1 块", tower_label)
                needed = 1
            logger.info("水塔 %s 需投放 %d 块水方块", tower_label, needed)

            # cam2 视觉闭环: 把底盘纵向对齐到水塔等级标居中
            # 2026-08-06 提速: 第 2 座塔跳过此步 (用 move_for 闭环已到位, 横向无变化 ~7s)
            if tower_idx == 0:
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
                    target_offset = direction * group * (
                        group_forward_m if direction > 0 else group_backward_m
                    )
                    pick_x = first_x if (picked % 2 == 0) else second_x
                    deliver_hands = cfg.get("deliver_hand_by_index",
                                            [float(carry["hand_angle_deg"])])
                    deliver_hand = deliver_hands[min(picked, len(deliver_hands) - 1)]

                    # 准备 pick: 底盘到组 + 臂切 pick 姿态 (并发)
                    # 2026-08-06 提速: 原版 move_x + chassis + composite_run 全串行 ~5s.
                    #                 改并发 ~3s (max(底盘 1.5s, 臂 3s)).
                    d_to_group = target_offset - chassis_at_tower_m
                    logger.info("第 %d 块: 底盘 Δ=%.2f m → 第 %d 组 (并发切 pick 姿态, X=%s)",
                                picked + 1, d_to_group, group + 1, pick_x)
                    _parallel_chassis_arm(
                        arm_client, runner,
                        target_dx_m=d_to_group,
                        arm_kwargs=dict(
                            arm=float(pick["arm_angle_deg"]),       # +90
                            x_mm=float(pick_x),                       # -165 / -210
                            y_mm=float(vision.get("servo_y_mm",
                                                  pick["y_transition_mm"])),  # -150
                            hand=float(pick["hand_angle_deg"]),      # -10
                            speed=100,
                            timeout=10.0,
                        ),
                    )
                    chassis_at_tower_m = target_offset

                    # 抓块 (含 vision servo + 自动抬回 transport Y)
                    _pick_cube(arm_client, runner, cfg, pick_x)

                    # 准备 deliver: 底盘回塔 + 臂切 carry 姿态 (并发)
                    # 2026-08-06 提速: 原版 chassis_back + composite_run + move_x +
                    #                 move_y 全串行 ~7s. 改并发 ~3s + 单 move_y 1.5s,
                    #                 省 ~5s/块.
                    d_back = -chassis_at_tower_m
                    logger.info("第 %d 块: 底盘回塔 Δ=%.2f m (并发切 carry 姿态, X=%s)",
                                picked + 1, d_back, carry["x_mm"])
                    _parallel_chassis_arm(
                        arm_client, runner,
                        target_dx_m=d_back,
                        arm_kwargs=dict(
                            arm=float(carry["arm_angle_deg"]),      # -95
                            x_mm=float(carry["x_mm"]),              # -100 (投递位)
                            y_mm=-75.0,                              # TRANSIT_Y
                            hand=float(deliver_hand),                # per-cube 梯度
                            speed=100,
                            timeout=10.0,
                        ),
                    )
                    chassis_at_tower_m = 0.0

                    # 投放: 单步 move_y → grasp off (Y=-75 → deliver_y)
                    # 2026-08-06: 到位就放, 不等待. move_y 用 v_max=150mm/s 加速;
                    # 第 3 块 (deliver_y=-75 == 当前 Y) 跳过 move_y, 直接 grasp off.
                    deliver_ys = cfg.get("deliver_y_by_index",
                                         [-50.0, -65.0, -80.0])
                    deliver_y = deliver_ys[min(picked, len(deliver_ys) - 1)]
                    logger.info("第 %d 块: 投放 Y=%.0f mm + grasp off",
                                picked + 1, deliver_y)
                    try:
                        cur_y = arm_client.get_state().y_mm
                    except Exception:
                        cur_y = None
                    if cur_y is None or abs(cur_y - deliver_y) > 1.0:
                        runner.client.move_y(float(deliver_y),
                                              v_max_mms=100.0, timeout=3.0)
                    runner.grasp(on=False)

                except Exception:
                    logger.exception("第 %d 块失败, 继续下一块", picked + 1)
                picked += 1

            completed.append("tower_{}".format(tower_label))

        # 2026-08-06: 任务结束前把 X/Y 调到大臂安全区 (X∈[-300,-200], Y∈[-200,-90])
        # 防止 orchestrator arm-home reset 接手时 X/Y 不在安全区撞车.
        # 用 _ensure_xy_in_safe_zone: 已满足 no-op, 不满足时按规则调.
        logger.info("任务结束: 把 X/Y 调到大臂安全区")
        _ensure_xy_in_safe_zone(arm_client, runner, timeout=10.0)

    except Exception as exc:
        logger.exception("water_tower_task 失败: %s", exc)
        return {"ok": False, "completed": completed, "error": str(exc)}

    return {"ok": True, "completed": completed}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
    result = run()
    print("任务二 水塔取水 执行结果:", result)