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
import math
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional

from main.api_client import RuntimeApiClient
from main.arm import ArmClient, ArmRunner
from main.task._config import load_task_config

logger = logging.getLogger("task.task2_water_tower")

# 水塔等级标签 → 所需方块数
WATER_TOWER_LABELS = {"water_l1", "water_l2", "water_l3"}


# ──────────────────────────────────────────────────────────────────────
# 任务二 姿态参数 (集中管理, 仿 task4 模式 main.arm.each_task.task4.constants)
#
# 默认值在顶部常量段定义, run() 函数把它们作为默认参数透传, 现场快速调整改这里即可.
# ──────────────────────────────────────────────────────────────────────

# init 姿态 (进入任务区 / 底盘前进到位后, 抬升臂的初始 Y/X/角度)
INIT_POSE_Y_MM: float = -150.0
INIT_POSE_X_MM: float = 0.0
INIT_POSE_ARM_DEG: float = 90.0
INIT_POSE_HAND_DEG: float = -90.0

# detection 姿态 (底盘到位后, 视觉识别水塔等级标前的姿态)
# 2026-08-06: 大臂改为 -92°, Y 改为 -10 (抬高 10mm), track_align setpoint 重测 → (0.148, 0.234)
DETECT_POSE_X_MM: float = -200.0
DETECT_POSE_ARM_DEG: float = -92.0
DETECT_POSE_HAND_DEG: float = -60.0
DETECT_POSE_Y_MM: float = -10.0

# 抓取姿态 (S 形态, 视觉伺服起点) — pick_pose 段
PICK_POSE_Y_TRANSITION_MM: float = -150.0
PICK_POSE_Y_DESCEND_MM: float = -50.0
PICK_POSE_Y_LIFT_MM: float = -150.0
PICK_POSE_ARM_DEG: float = 90.0
PICK_POSE_HAND_DEG: float = -10.0

# 方块 X 坐标 (per-cube)
FIRST_CUBE_X_MM: float = -165.0
SECOND_CUBE_X_MM: float = -210.0

# carry 姿态 (从方块到水塔的运输/投放位姿)
# 2026-08-06: arm 改为 -92° (对标 detection_pose.arm_angle_deg)
CARRY_POSE_X_MM: float = -115.0
CARRY_POSE_ARM_DEG: float = -92.0
CARRY_POSE_HAND_DEG: float = -90.0

# 投放梯度 (同一水塔内 1/2/3 块的 Y 深度 + 手爪角度)
# 2026-08-06: Y 改为 [0, -45, -70] (浅→深: 0 → -45 → -70), hand 统一 -80° (不再梯度)
DELIVER_Y_BY_INDEX: List[float] = [0.0, -45.0, -70.0]
DELIVER_HAND_BY_INDEX: List[float] = [-80.0, -80.0, -80.0]

# 中转 Y (carry 切姿态时 Y 降到 -75, 不直接走投放深度, 防撞)
TRANSIT_Y_MM: float = -75.0

# 场地几何
TOWER_SPACING_M: float = 0.43       # 8-06: 0.66 → 0.68 → 0.70 → 0.58 → 0.50 → 0.43 (实测两塔间距)
GROUP_FORWARD_M: float = 0.35        # 8-06: 0.33 → 0.35 → 0.37 (tower 1 前向组间距)
GROUP_BACKWARD_M: float = 0.33       # 8-06: tower 2 后向用 0.33 (不走 GROUP_FORWARD)

# 视觉伺服 (cam2 闭环抓水立方)
PICK_VISION_LABEL: str = "water"
PICK_VISION_SETPOINT_CXCY: List[float] = [0.063, -0.202]      # 8-06 重测 20 帧均值 (cx=0.063, cy=-0.202, 抖动 < 2px)
PICK_VISION_TIMEOUT_S: float = 3.5
# 2026-08-09: 每块视觉伺服单独超时 (None=用 yaml 默认 timeout).
#   第2块(1): 6s — 用户规定 (默认 5s 不够稳, 第 2 块单独加到 6s)
#   第3块(2): 7s — 高处堆叠抖动大收敛慢 (原 PICK_BLOCK3_TIMEOUT_S)
PICK_BLOCK2_TIMEOUT_S: Optional[float] = 6.0
PICK_BLOCK3_TIMEOUT_S: Optional[float] = 7.0
# 2026-08-09 用户新规则: 视觉对齐超时**不许失败** — 降死区 + 加时重新对齐一次.
#   默认死区 0.05 → 重试放宽到 0.07; 超时 +4s (块默认 5/6/7s → 重试 9/10/11s).
PICK_RETRY_DEADZONE: float = 0.07
PICK_RETRY_EXTRA_S: float = 4.0
PICK_VISION_HZ: float = 25.0
PICK_VISION_GAIN_ARM: float = 1.6
PICK_VISION_GAIN_X: float = 0.35
PICK_VISION_DEADZONE: float = 0.15
PICK_VISION_MAX_VEL: float = 0.05
PICK_VISION_SETTLE_HITS: int = 4
PICK_VISION_HOLD_S: float = 0.0

# track_align (底盘对齐水塔等级标)
TRACK_ALIGN_TARGET: str = "water"
TRACK_ALIGN_SETPOINT_CXCY: List[float] = [0.148, 0.234]        # 8-06 重测 (大臂 -92° 后重测 cx/cy)
TRACK_ALIGN_VX_ONLY: bool = True
TRACK_ALIGN_SIGN_VX: int = +1
TRACK_ALIGN_SIGN_VY: int = +1
TRACK_ALIGN_KP: float = 0.22
TRACK_ALIGN_V_MAX: float = 0.11
TRACK_ALIGN_V_SLEW: float = 0.011
TRACK_ALIGN_HZ: float = 25.0
TRACK_ALIGN_DEADBAND: float = 0.06
TRACK_ALIGN_HOLD_FRAMES: int = 4
TRACK_ALIGN_MAX_LOST_FRAMES: int = 30
TRACK_ALIGN_MAX_SECONDS: float = 6.0

# 业务参数
DETECT_RETRY_STEP_M: float = 0.10
DETECT_RETRY_MAX: int = 2
VACUUM_SETTLE_S: float = 0.0       # 8-06: 0.5 → 0.2 → 0.0 (grasp 后立即下一步)
V_MAX_ARM_X_MMS: float = 80.0
CHASSIS_MOVE_TIMEOUT_S: float = 30.0


# ── 辅助函数 ─────────────────────────────────────────────────

def _chassis_move_for(
    arm_client: ArmClient,
    dx_m: float,
    timeout: float,
    *,
    use_lane_align: bool = False,
    speed_mps: float = 0.10,
) -> dict:
    """底盘纵向 move_for — 仿 task1_seeding 范式: sync=False + wait_job.

    2026-08-06 提速: 仿 task1_seeding.py 的 _chassis_goto, 直接走
    execute_car_action (绕过 ChassisClient.connect()/close() 每次建连的开销).
    不走 sync=True (server-side polling 阻塞 HTTP), 改 sync=False 立即返回
    job_id, client 端 wait_job 轮询, 省 server polling ~30-50ms.
    SDK 串口 SerialEngine 统一调度 (CLAUDE.md §Runtime concurrency model),
    多线程并发安全.

    2026-08-09: 前进/后退统一走 move_for (SDK 4 轮等速, odom 自洽). 不再默认走
    move_along_lane — 2026-08-06 曾因 odom theta 漂移改走车道线, 但 move_for 正用反用
    各一次可抵消漂移 (见 CLAUDE.md "底盘平移一律 move_for" 规则).
    use_lane_align=True 仍可强制 main.chassis.move_along_lane (特殊场景兜底).

    Args:
        arm_client: ArmClient (用来访问 runtime HTTP / chassis API).
        dx_m: 位移 (m), 正=前进, 负=后退. lane_align 模式下取绝对值当 distance_m.
        timeout: 单次最大等待 (s).
        use_lane_align: True=走 move_along_lane (沿车道线, 不偏), False=走 move_for (SDK 4 轮等速).
        speed_mps: 底盘移动速度上限 (m/s), 默认 0.10 (2026-08-09: 0.15/0.2 → 0.10).
                  move_along_lane 用它当 vx; move_for 用它当 max_velocities[0/1] (前后都限).
    """
    if use_lane_align and abs(dx_m) > 1e-3:
        # 沿中心车道线走 (vy=0 + ω 锁对齐, 弯道不偏)
        from main.chassis import move_along_lane
        vx = float(speed_mps) if dx_m > 0 else -float(speed_mps)
        move_along_lane(
            vx=vx,
            distance_m=abs(float(dx_m)),
        )
        return {"ok": True, "mode": "lane_align", "dx_m": float(dx_m)}

    # 主路径: SDK move_for (字节流 4 轮等速, 正=前进 负=后退)
    # 2026-08-09: 显式传 max_velocities 控速 (默认 0.2 太快), 前后都限 speed_mps.
    job = arm_client.http.execute_car_action(
        "move_for", [dx_m, 0.0, 0.0],
        timeout=timeout, sync=False,
        max_velocities=[float(speed_mps), float(speed_mps), math.pi / 3.0],
    )
    jid = job.get("id") if isinstance(job, dict) else None
    if jid:
        arm_client.http.wait_job(jid, timeout=timeout + 5.0)
    return job


def _parallel_chassis_arm(
    arm_client: ArmClient,
    runner: ArmRunner,
    *,
    target_dx_m: float = 0.0,
    arm_kwargs: Optional[Dict[str, Any]] = None,
    timeout: float = 10.0,
    use_lane_align: Optional[bool] = None,
) -> None:
    """底盘 move_for + 臂 composite_run 并发 (task1_seeding._parallel_chassis_arm 模式).

    Args:
        target_dx_m: 底盘相对位移 (m), 0 跳过底盘动作
        arm_kwargs:  传 composite_run 的 kwargs (arm/x_mm/y_mm/hand/speed/timeout),
                     None/{} 跳过臂动作
        timeout:     两个动作的最大等待
        use_lane_align: 覆盖底盘前进走法. None=默认 全部 move_for (4 轮等速, 2026-08-09 起
                     前进不再走 move_along_lane); False=强制 move_for; True=强制 move_along_lane.

    2026-08-06 提速:
      - chassis 0.15m (~0.7s) 与臂切姿态 (~2-3s) 完全并发, 主循环零阻塞.
      - composite_run 用 sync=False 立即返回 job_id (避免 HTTP 响应流持有到
        动作结束, 减少 504 风险; 任务二原版全串行 ~8s 改并发 ~3s).
      - 复合动作 m 单位转换 (composite_run kwargs 用 x_mm/y_mm, 自动转 m).

    task1_seeding.py:619-631 是同样的模式; SDK 串口 SerialEngine 单 io 线程
    调度 (CLAUDE.md §Runtime concurrency model), 多线程并发安全.
    """
    tasks = []
    with ThreadPoolExecutor(max_workers=2) as ex:
        # 2026-08-09: 底盘 move_for 与臂动作**从第一帧就并发** — 不再先串行调安全区.
        # 转大臂时 _safe_arm_rotation_sequence 阶段 1 自己会调 X/Y 安全位, 与底盘移动
        # 同时进行 (用户要求: 进入任务点就动 X, 底盘前进时臂同步; 机械臂内部顺序仍按
        # 3 阶段). 省每块 ~2-4s 串行等待.
        # ⚠️ 代价: 底盘移动期间 X/Y 正收回安全位 (非先收回再移动), 真机需确认无碰撞.
        # 只有不转大臂 (arm_kwargs 无 arm) 时才先保证 X/Y 安全区 (无 3 阶段兜底).
        if (arm_kwargs and arm_kwargs.get("arm") is None
                and abs(target_dx_m) > 1e-3):
            _ensure_xy_in_safe_zone(arm_client, runner, timeout=timeout)
        # 前进/后退统一走 SDK move_for (4 轮等速, odom 自洽, 见 CLAUDE.md
        # "底盘平移一律 move_for" 规则). 不再默认走 move_along_lane 车道线.
        # use_lane_align=True 仍可强制 move_along_lane (特殊场景兜底).
        if abs(target_dx_m) > 1e-3:
            use_lane = False if use_lane_align is None else use_lane_align
            tasks.append(ex.submit(
                _chassis_move_for, arm_client, target_dx_m, timeout,
                use_lane_align=use_lane,
            ))

        # 2026-08-06 提速: 大臂转动不再拆 3 阶段, 走 _safe_arm_rotation_sequence
        # (task1 范式: 1 次 composite_run 4 轴并发 + sync=False + wait_job).
        # X/Y 物理时间 ~50ms << 大臂 ~2s, X/Y 早就到位冻结, 安全等价于旧 3 阶段.
        if arm_kwargs and arm_kwargs.get("arm") is not None:
            tasks.append(ex.submit(_safe_arm_rotation_sequence, arm_client,
                                    runner, arm_kwargs=arm_kwargs, timeout=timeout))
        elif arm_kwargs:
            # 不转大臂时, 直接单步 composite_run (走 _safe_composite_run 范式)
            ak = dict(arm_kwargs)
            tasks.append(ex.submit(_safe_composite_run, arm_client,
                                    arm=ak.get("arm"),
                                    x_mm=ak.get("x_mm"),
                                    y_mm=ak.get("y_mm"),
                                    hand=ak.get("hand"),
                                    speed=ak.get("speed", 100),
                                    timeout=ak.get("timeout", timeout)))

        for t in tasks:
            t.result()


def _safe_arm_rotation_sequence(
    arm_client: ArmClient,
    runner: ArmRunner,
    *,
    arm_kwargs: Dict[str, Any],
    timeout: float = 10.0,
) -> None:
    """大臂转动 (3 阶段顺序, 不并发) — 恢复用户原始规定.

    2026-08-06 用户规定: 大臂转动必须满足 Y ∈ [-200, -90] 且 X ∈ [-300, -200].
    大臂转动期间只有大臂 + 末端可以动, X/Y **必须冻结**, 转动完成后 X/Y 才能到目标.

    实现 3 阶段 (顺序, 不并发):
      阶段 1: composite_run X/Y (无 arm/hand, 把 X/Y 调到安全位, 已有不动)
      阶段 2: composite_run arm + hand (X/Y 冻结在安全位)
      阶段 3: composite_run X/Y 到目标 (无 arm 变化, Y 先 X 后)

    2026-08-06 实车 Y 冲顶修复: 之前改成 4 轴并发 (1 次 composite_run), 大臂转动期间
    Y PID 闭环 + 大臂物理干扰 → Y overshoot, 实际 Y 数值不稳. 改回 3 阶段顺序.

    runner 参数保留以兼容 _parallel_chassis_arm 调用, 但本函数不再调 runner.
    """
    target_y = arm_kwargs.get("y_mm")
    target_x = arm_kwargs.get("x_mm")
    target_arm = arm_kwargs.get("arm")
    target_hand = arm_kwargs.get("hand")

    try:
        state = arm_client.get_state()
    except Exception as exc:
        logger.warning("_safe_arm_rotation_sequence: 读不到状态, 跳过 (%s)", exc)
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

    logger.info(
        "大臂 3 阶段: 当前 Y=%.1f X=%.1f → 安全位 Y=%.1f X=%.1f → 目标 Y=%s X=%s arm=%s hand=%s",
        cur_y, cur_x, safe_y, safe_x,
        target_y, target_x, target_arm, target_hand,
    )

    # 阶段 1: X/Y 调到安全位 (已有不动)
    if abs(safe_x - cur_x) > 1.0 or abs(safe_y - cur_y) > 1.0:
        logger.info("  阶段 1: X/Y 调安全位 Y=%.1f X=%.1f", safe_y, safe_x)
        _safe_composite_run(
            arm_client,
            arm=None, hand=None,
            y_mm=safe_y if abs(safe_y - cur_y) > 1.0 else None,
            x_mm=safe_x if abs(safe_x - cur_x) > 1.0 else None,
            speed=100, timeout=timeout,
        )

    # 阶段 2: arm + hand (X/Y 冻结在安全位)
    if target_arm is not None or target_hand is not None:
        logger.info("  阶段 2: arm=%s hand=%s (X/Y 冻结在安全位)", target_arm, target_hand)
        _safe_composite_run(
            arm_client,
            x_mm=None, y_mm=None,
            arm=target_arm, hand=target_hand,
            speed=100, timeout=timeout,
        )

    # 阶段 3: X/Y 并发到目标 (用户 2026-08-06 规定: X 伸出 + Y 下降同时进行)
    # 一次 composite_run 4 轴并发, SDK 内部 ThreadPoolExecutor 真的并发
    need_x = target_x is not None and abs(target_x - safe_x) > 1.0
    need_y = target_y is not None and abs(target_y - safe_y) > 1.0
    if need_x or need_y:
        logger.info("  阶段 3: X/Y 并发 X=%s Y=%s", target_x, target_y)
        _safe_composite_run(
            arm_client, arm=None, hand=None,
            x_mm=target_x if need_x else None,
            y_mm=target_y if need_y else None,
            speed=100, timeout=timeout,
        )


def _safe_composite_run(
    arm_client: ArmClient,
    *,
    arm: Optional[float] = None,
    x_mm: Optional[float] = None,
    y_mm: Optional[float] = None,
    hand: Optional[float] = None,
    speed: int = 100,
    timeout: float = 10.0,
) -> None:
    """composite_run 仿 task1 范式: sync=False + wait_job, 走 http.execute 直发.

    不走 _call_arm (sync=True server-side polling), 省 ~30-50ms 调度延迟.
    SDK 串口 SerialEngine 调度, 物理执行时间按动作而定.
    """
    if arm is None and x_mm is None and y_mm is None and hand is None:
        return
    kwargs = dict(speed=speed, timeout=timeout)
    if arm is not None:
        kwargs["arm"] = float(arm)
    if x_mm is not None:
        kwargs["x"] = float(x_mm) / 1000.0
    if y_mm is not None:
        kwargs["y"] = float(y_mm) / 1000.0
    if hand is not None:
        kwargs["hand"] = float(hand)
    job = arm_client.http.execute(
        "arm", "composite_run", kwargs=kwargs, sync=False,
    )
    jid = job.get("id") if isinstance(job, dict) else None
    if jid:
        arm_client.http.wait_job(jid, timeout=timeout + 5.0)


def _deliver_prepare(
    arm_client: ArmClient,
    runner: ArmRunner,
    *,
    target_dx_m: float,
    carry_x_mm: float,
    carry_arm_deg: float,
    carry_hand_deg: float,
    deliver_y_mm: float,
    timeout: float = 10.0,
) -> None:
    """carry 切姿态 (用户 2026-08-08 新规定, 顺序: 收X → 大臂转 → Y降投放深度 → X伸出).

    抓取后到投放位 (与旧版 "阶段3 X/Y 并发 + 先到 -75 transit" 不同):
      1) X 收到 -260 (更收回, 远离水塔, 给大臂转动留空间)
      2) _safe_arm_rotation_sequence: 只转 arm + hand (X/Y 冻结在安全位, 不做阶段3 X/Y)
      3) Y 直接降到投放深度 deliver_y_mm (不再去 -75 transit; 必须在大臂之后)
      4) X 伸出到 carry_x_mm (必须在 Y 之后)

    底盘 move_for 回塔 与 臂步骤并发 (跟旧版一致).
    """
    tasks = []
    with ThreadPoolExecutor(max_workers=2) as ex:
        # 底盘回塔: 统一 SDK move_for (2026-08-09, 前进/后退都走 move_for).
        # d_back 正值=Tower 2 抓完前进回塔, 负值=Tower 1 抓完后退回塔, _chassis_move_for
        # 自动按 dx_m 正负决定 vx 方向.
        if abs(target_dx_m) > 1e-3:
            tasks.append(ex.submit(
                _chassis_move_for, arm_client, target_dx_m, timeout,
                use_lane_align=False,
            ))

        # 臂步骤顺序 (一个 worker 内串行, 跟底盘并发; 2026-08-08 改: 不做 X/Y 并发)
        def _arm_prep():
            # 1) X 收到 -260 (更收回)
            _safe_composite_run(
                arm_client, arm=None,
                x_mm=-260.0, y_mm=None,
                hand=None, speed=100, timeout=5.0,
            )
            # 2) 大臂 + 手爪转 (X/Y 冻结在安全位, 不做阶段3 X/Y)
            _safe_arm_rotation_sequence(
                arm_client, runner,
                arm_kwargs=dict(
                    arm=carry_arm_deg,
                    hand=carry_hand_deg,
                    speed=100,
                    timeout=timeout,
                ),
            )
            # 3) Y 直接降到投放深度 (大臂之后, X 之前)
            _safe_composite_run(
                arm_client, arm=None,
                y_mm=deliver_y_mm, x_mm=None,
                hand=None, speed=100, timeout=timeout,
            )
            # 4) X 伸出到投放位 (Y 之后)
            _safe_composite_run(
                arm_client, arm=None,
                x_mm=carry_x_mm, y_mm=None,
                hand=None, speed=100, timeout=timeout,
            )

        tasks.append(ex.submit(_arm_prep))
        for t in tasks:
            t.result()


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
      1. _parallel_chassis_arm 入口 — 仅当 arm_kwargs 无 arm (不转大臂, 无 3 阶段兜底)
         且底盘移动时才同步执行 (2026-08-09 改: 转大臂的并发走 _safe_arm_rotation_sequence 阶段 1)
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
    2026-08-06: 删除 sleep, 用紧凑 busy-poll (cam2 feed 默认 30Hz, 任务里等的就是它下一帧).
    """
    count_map = {"water_l1": 1, "water_l2": 2, "water_l3": 3}
    deadline = time.time() + 1.0
    while time.time() < deadline:
        try:
            resp = client.get("/v1/realtime/vision/task", timeout=2)
        except Exception:
            continue
        if not isinstance(resp, dict) or not resp.get("ok"):
            continue
        task_state = resp.get("task_state") or {}
        if not task_state.get("active"):
            continue
        for d in task_state.get("detections") or []:
            label = (d or {}).get("label", "")
            if label in WATER_TOWER_LABELS:
                n = count_map[label]
                logger.info("水塔识别 %s → 需要 %d 块", label, n)
                return n
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
    """对齐结束后显式把底盘停稳 (track_chassis 零速异步, 防止漂移).

    2026-08-06: 删除末尾 sleep (用户要求删除所有 sleep).
    """
    try:
        arm_client.http.post(
            "/v1/realtime/chassis-velocity",
            {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            timeout=2.0,
        )
    except Exception:
        pass
    # 等底盘轮速归零: 真实编码器反馈双采样 (2026-08-09 修 — 旧 GET
    # wheels/speeds 端点只存在 POST, 405 → 等待从未生效)。
    arm_client.http.wait_wheels_stopped(settle_s=0.15, timeout_s=1.0)


# ── 核心动作子流程 ────────────────────────────────────────────────

def _pick_cube_servo_local(
    arm_client: ArmClient,
    vision: Dict[str, Any],
    pick: Dict[str, Any],
    sp_x: Optional[float],
    sp_y: Optional[float],
    timeout_override: Optional[float] = None,
) -> Dict[str, Any]:
    """本地视觉伺服 (2026-08-09 闭环下沉): runtime 进程内闭环, main 只发一次目标.

    走 /v1/execute run_arm_servo —— runtime 内每帧读 task_feed 缓存 + 直调 arm
    (x_speed / set_arm_angle), 无网络往返. 对齐收敛 → y 降 grasp_y + hand 转
    descend_hand (并发) → 吸气 → 抬回 servo_y. 未收敛抛 RuntimeError.

    timeout_override (2026-08-09): 覆盖 servo_timeout (每座塔第 3 块单独加时用).
    None → 用 vision 配置的 timeout.
    """
    servo_kw = dict(
        label=vision.get("label", "water"),
        hz=float(vision.get("hz", 20.0)),
        gain_arm=float(vision.get("gain_arm", 0.4)),
        gain_x=float(vision.get("gain_x", 0.08)),
        deadzone=float(vision.get("deadzone", 0.03)),
        max_vel=float(vision.get("max_vel", 0.20)),
        arm_start=float(pick["arm_angle_deg"]),
        sign_arm=float(vision.get("sign_arm", 1.0)),
        sign_x=float(vision.get("sign_x", -1.0)),
        setpoint_x_norm=sp_x if sp_x is not None else 0.0,
        setpoint_y_norm=sp_y if sp_y is not None else 0.0,
        arm_min=float(vision["arm_min"]) if vision.get("arm_min") is not None else -150.0,
        arm_max=float(vision["arm_max"]) if vision.get("arm_max") is not None else 90.0,
        servo_timeout=float(timeout_override if timeout_override is not None
                            else vision.get("timeout", 15.0)),
        settle_hits=int(vision.get("settle_hits", 3)),
    )
    logger.info(
        "cam2 本地视觉伺服: run_arm_servo(setpoint=(%.3f,%.3f) hz=%s gain_arm=%s gain_x=%s "
        "deadzone=%s max_vel=%s arm=[%s,%s] settle=%s servo_timeout=%s)",
        servo_kw["setpoint_x_norm"], servo_kw["setpoint_y_norm"],
        servo_kw["hz"], servo_kw["gain_arm"], servo_kw["gain_x"],
        servo_kw["deadzone"], servo_kw["max_vel"],
        servo_kw["arm_min"], servo_kw["arm_max"],
        servo_kw["settle_hits"], servo_kw["servo_timeout"],
    )
    job = arm_client.http.execute(
        "car", "run_arm_servo", kwargs=servo_kw, sync=True,
        timeout=float(servo_kw["servo_timeout"]) + 15.0,
    )
    result = (job or {}).get("result") if isinstance(job, dict) else None
    result = result if isinstance(result, dict) else {}
    if isinstance(job, dict) and job.get("status") not in (None, "succeeded"):
        raise RuntimeError(
            f"run_arm_servo 任务失败: status={job.get('status')} error={job.get('error')}"
        )
    logger.info("cam2 本地视觉伺服结果: reason=%s settled=%s trace_hits=%s end_arm=%s",
                result.get("reason"), result.get("settled"),
                result.get("trace_hits"), result.get("end_arm"))
    # 2026-08-09 用户新规则: 超时不许失败 — 降死区 PICK_RETRY_DEADZONE(0.07) +
    # 加时 PICK_RETRY_EXTRA_S(4s) 重新对齐一次. 仅 timeout 重试 (stopped=急停不重试).
    if (not result.get("settled")
            and result.get("reason") == "timeout"):
        logger.info("视觉对齐超时, 降死区重试: deadzone %.3f→%.3f, servo_timeout +%.0fs",
                    servo_kw["deadzone"], PICK_RETRY_DEADZONE, PICK_RETRY_EXTRA_S)
        servo_kw["deadzone"] = PICK_RETRY_DEADZONE
        servo_kw["servo_timeout"] = float(servo_kw["servo_timeout"]) + PICK_RETRY_EXTRA_S
        job = arm_client.http.execute(
            "car", "run_arm_servo", kwargs=servo_kw, sync=True,
            timeout=float(servo_kw["servo_timeout"]) + 15.0,
        )
        result = (job or {}).get("result") if isinstance(job, dict) else None
        result = result if isinstance(result, dict) else {}
        if isinstance(job, dict) and job.get("status") not in (None, "succeeded"):
            raise RuntimeError(
                f"run_arm_servo 重试任务失败: status={job.get('status')} error={job.get('error')}"
            )
        logger.info("cam2 本地视觉伺服重试结果: reason=%s settled=%s trace_hits=%s end_arm=%s",
                    result.get("reason"), result.get("settled"),
                    result.get("trace_hits"), result.get("end_arm"))
    if not result.get("settled"):
        raise RuntimeError(
            f"cam2 本地视觉抓水立方失败 (reason={result.get('reason')}, "
            f"trace_hits={result.get('trace_hits')}, end_arm={result.get('end_arm')})"
        )

    # 对齐完成 → y 降 grasp_y + hand 转 descend_hand (并发) → 吸气 → 抬回 servo_y
    try:
        hand_param = vision.get("descend_hand_deg")
        if hand_param is not None:
            hand_param = float(hand_param)
        target_m = float(vision.get("grasp_y_mm", pick["y_descend_mm"])) / 1000.0
        job_down = arm_client.http.execute(
            "arm", "composite_run",
            kwargs=dict(arm=None, x=None, y=target_m,
                        hand=hand_param, speed=100, timeout=5.0),
            sync=False,
        )
        jid = job_down.get("id") if isinstance(job_down, dict) else None
        if jid:
            arm_client.http.wait_job(jid, timeout=5.0)
        arm_client.http.execute("arm", "grasp", kwargs=dict(value=True), sync=False)
        # 抬回 servo_y (fire-and-forget, 下游 move_y 并发)
        servo_y = float(vision.get("servo_y_mm", pick["y_transition_mm"]))
        arm_client.http.execute(
            "arm", "composite_run",
            kwargs=dict(arm=None, x=None, y=servo_y / 1000.0, hand=None,
                        speed=100, timeout=5.0),
            sync=False,
        )
    except Exception as exc:
        try:
            safe_y = float(vision.get("servo_y_mm", pick["y_transition_mm"])) / 1000.0
            arm_client.http.execute(
                "arm", "move_y_position",
                kwargs=dict(target=safe_y, timeout=5.0), sync=False,
            )
        except Exception:
            pass
        raise RuntimeError(f"cam2 本地视觉抓水立方 grasp 段失败: {exc}") from exc
    return result


def _pick_cube(
    arm_client: ArmClient,
    runner: ArmRunner,
    cfg: Dict[str, Any],
    cube_x_mm: float,
    vision_timeout_override: Optional[float] = None,
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
        # 2026-08-06: vacuum_settle_s 直接 0 (yaml 改), 删除 sleep, fire-and-forget 抬升
        runner.move_y(float(pick["y_descend_mm"]))
        runner.grasp(on=True)
        # 删除 sleep (vacuum_settle_s=0); 抬升 fire-and-forget
        runner.move_y(float(pick["y_lift_mm"]))
        return

    # ---- cam2 视觉伺服抓水立方 ----
    sp = vision.get("setpoint_cxcy")
    sp_x = float(sp[0]) if (sp and len(sp) >= 1) else None
    sp_y = float(sp[1]) if (sp and len(sp) >= 2) else None
    logger.info(
        "cam2 视觉对齐开始: cube_x_mm=%.0f, setpoint_cxcy=(%.3f, %.3f), "
        "settle_hits=%d, timeout=%.1fs, deadzone=%.3f",
        float(cube_x_mm),
        sp_x if sp_x is not None else 0.0,
        sp_y if sp_y is not None else 0.0,
        int(vision.get("settle_hits", 3)),
        float(vision.get("timeout", 15.0)),
        float(vision.get("deadzone", 0.03)),
    )
    import time as _time
    _t0 = _time.time()
    if vision.get("local_servo"):
        # 2026-08-09 闭环下沉: runtime 进程内视觉伺服 (main 只发一次目标, 无每帧网络)
        result = _pick_cube_servo_local(arm_client, vision, pick, sp_x, sp_y,
                                        timeout_override=vision_timeout_override)
    else:
        result = runner.track_velocity_pick(
            vision.get("label", "water"),
            x_start=float(cube_x_mm),
            y_start=float(vision.get("servo_y_mm", pick["y_transition_mm"])),
            arm_start=float(pick["arm_angle_deg"]),
            hand_start=float(pick["hand_angle_deg"]),
            grasp_y_mm=float(vision.get("grasp_y_mm", pick["y_descend_mm"])),
            timeout=float(vision_timeout_override if vision_timeout_override is not None
                          else vision.get("timeout", 15.0)),
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
    logger.info(
        "cam2 视觉对齐完成: cube_x_mm=%.0f, 用时=%.2fs, ok=%s, "
        "trace_hits=%d, settled=%s, end_arm=%.1f°",
        float(cube_x_mm),
        _time.time() - _t0,
        result.get("ok"),
        int(result.get("trace_hits", 0)),
        result.get("settled"),
        float(result.get("end_arm") or 0.0),
    )
    # 2026-08-06: 打印视觉对齐实际效果 (trace_hits / settled / end_arm),
    # 确认 setpoint 在 X=cube_x_mm 位置是否真的收敛.
    logger.info(
        "cam2 视觉对齐: cube_x_mm=%.0f, trace_hits=%d, settled=%s, "
        "end_arm=%.1f°, end_hand=%s, steps=%s",
        float(cube_x_mm),
        int(result.get("trace_hits", 0)),
        result.get("settled"),
        float(result.get("end_arm") or 0.0),
        result.get("end_hand"),
        result.get("steps"),
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
        # ===== 初始化: 进入任务点就动 X — 底盘回退 entry_back_off_m 与臂切 detection 姿态并发 =====
        # 2026-08-09: orchestrator waypoint 的 back_off(0.2m) 挪进任务内, 底盘后退的同时
        # 臂收 X/转大臂/切 detection 姿态 (进入任务点就开始先移动 X, 跟底盘移动一起进行).
        # 臂内部仍是 3 阶段顺序 (安全位→转臂→到位), 只是跟底盘并发.
        entry_back_off_m = float(cfg.get("entry_back_off_m", 0.0))
        # 2026-08-09: orchestrator 已在 task3 识别结束后巡线途中预摆 detection 姿态
        # (arm_poses.TASK2_DETECTION_ARM) → 已在位则跳过臂动作, 只做底盘回退, 省重复摆臂.
        from main.task.task3.arm_poses import TASK2_DETECTION_ARM, arm_at_pose
        try:
            detection_in_place = bool(arm_at_pose(arm_client, TASK2_DETECTION_ARM))
        except Exception:
            detection_in_place = False
        init_arm_kwargs = None if detection_in_place else dict(
            arm=float(detection["arm_angle_deg"]),
            x_mm=x_target_mm,
            y_mm=-150.0,
            hand=float(detection["hand_angle_deg"]),
            speed=100,
            timeout=10.0,
        )
        if detection_in_place:
            logger.info("初始化: 臂已在 detection 姿态 (预摆完成), 只做底盘回退 %.2fm",
                        entry_back_off_m)
        else:
            logger.info("初始化: 底盘回退 %.2fm + 切 detection 姿态 (X=%.0f Y=-150 arm=%s hand=%s) 并发",
                        entry_back_off_m, x_target_mm,
                        detection["arm_angle_deg"], detection["hand_angle_deg"])
        _parallel_chassis_arm(
            arm_client, runner,
            target_dx_m=(-entry_back_off_m) if entry_back_off_m > 1e-3 else 0.0,
            arm_kwargs=init_arm_kwargs,
        )

        for tower_idx, tower_label in enumerate(cfg["source_position_order"]):
            logger.info("=== 处理水塔 %s (第 %d 座) ===", tower_label, tower_idx + 1)

            # 第 2 座塔: 底盘前进 tower_spacing_m + 切回 detection 姿态 (并发)
            # 2026-08-06 提速: 原版 move_x → chassis → move_x → set_arm → _wait →
            #                 set_hand → sleep 5 步串行 ~8s. 改并发 ~3s, 省 5s.
            # 2026-08-08: 前进第二座塔改 SDK move_for 直推 (不用 move_along_lane 车道线),
            #             距离 0.55m (tower_spacing_m 已在 yaml 改).
            if tower_idx > 0:
                logger.info("底盘前进 %.2f m → 水塔 %s (move_for 直推, 并发切回 detection 姿态)",
                            tower_spacing_m, tower_label)
                _parallel_chassis_arm(
                    arm_client, runner,
                    target_dx_m=tower_spacing_m,
                    use_lane_align=False,
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

            # cam2 视觉闭环: 把底盘对齐到水塔等级标居中
            # 2026-08-06 实测: 第 2 座塔也必须对齐 (move_for 0.7m 期间 odom theta
            # 漂移, 横向位置变化, cam2 看到的等级标不在第一座标定的 setpoint).
            # 每座水塔都做一次 track_chassis (~7s).
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
                    # 2026-08-09: 每块视觉伺服单独超时 — 第1块用 yaml 默认 5s;
                    #   第2块 6s (PICK_BLOCK2_TIMEOUT_S), 第3块 7s (PICK_BLOCK3_TIMEOUT_S).
                    _block_timeout = None
                    if picked == 1:
                        _block_timeout = PICK_BLOCK2_TIMEOUT_S
                    elif picked == 2:
                        _block_timeout = PICK_BLOCK3_TIMEOUT_S
                    _pick_cube(arm_client, runner, cfg, pick_x,
                               vision_timeout_override=_block_timeout)

                    # 准备 deliver: 底盘回塔 + 臂切 carry 姿态 (用户 2026-08-08 新规定)
                    #   顺序: X 收 -260 → 大臂转 → Y 直接降到投放深度 → X 伸到投放位
                    #   (不再去 -75 transit, 不再 X/Y 并发)
                    deliver_ys = cfg.get("deliver_y_by_index",
                                         [-50.0, -65.0, -80.0])
                    deliver_y = deliver_ys[min(picked, len(deliver_ys) - 1)]
                    d_back = -chassis_at_tower_m
                    # 2026-08-09: 投放 X 分水塔 (carry_x_by_tower_mm[tower_idx], 缺省 carry.x_mm)
                    carry_xs = cfg.get("carry_x_by_tower_mm") or []
                    if carry_xs and tower_idx < len(carry_xs):
                        carry_x_mm = float(carry_xs[tower_idx])
                    else:
                        carry_x_mm = float(carry["x_mm"])
                    logger.info("第 %d 块: 底盘回塔 Δ=%.2f m → carry (X收-260 → 大臂转%s° → Y降%.0f → X伸%.0f [塔%d])",
                                picked + 1, d_back, carry["arm_angle_deg"], deliver_y,
                                carry_x_mm, tower_idx + 1)
                    _deliver_prepare(
                        arm_client, runner,
                        target_dx_m=d_back,
                        carry_x_mm=carry_x_mm,
                        carry_arm_deg=float(carry["arm_angle_deg"]),
                        carry_hand_deg=float(deliver_hand),
                        deliver_y_mm=deliver_y,
                        timeout=10.0,
                    )
                    chassis_at_tower_m = 0.0

                    # 投放: grasp off (X/Y 已由 _deliver_prepare 就位; 2026-08-08 不再单步 move_y)
                    # grasp off fire-and-forget: 立即返回, 不等 grasp 物理完成,
                    # 下一轮 pick 的 _parallel_chassis_arm 立即启动 X 移动 (与 grasp 物理并发).
                    logger.info("第 %d 块: 投放 Y=%.0f mm + grasp off",
                                picked + 1, deliver_y)
                    arm_client.http.execute(
                        "arm", "grasp", kwargs=dict(value=False), sync=False,
                    )

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