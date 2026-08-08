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


# ══════════════════════════════════════════════════════════════════════════════
# 快速调参区 — 所有可调姿态 / 伺服 / 运动参数集中在此
# ══════════════════════════════════════════════════════════════════════════════

# ── 吸嘴 setpoint (目标在吸嘴正下方时其 bbox 中心归一化坐标) ────────────────
# 注意: hand=-15° 后吸嘴倾斜, 需重新标定 (2026-08-06 TODO)
TASK1_NOZZLE_OFFSET_MAP: Dict[str, Tuple[float, float]] = {
    "cylinder_1": (0.050, -0.425),
    "cylinder_2": (0.140, -0.420),
    "cylinder_3": (0.120, -0.410),
}

# ── 视觉伺服参数 (track_velocity_pick) ─────────────────────────────────────
PICK_SERVO_GAIN_ARM = 2.5
PICK_SERVO_GAIN_X = 0.55
PICK_SERVO_DEADZONE = 0.06
PICK_SERVO_MAX_VEL = 0.70
PICK_SERVO_SETTLE_HITS = 1
PICK_SERVO_HOLD_S = 0.05
PICK_SERVO_LIFT_BACK = True
PICK_SERVO_SKIP_POSE_ALIGN = True
PICK_SERVO_HZ = 20.0
# pick_track_timeout_s 优先从 cfg 读, 此处为缺省值
PICK_SERVO_TIMEOUT_S_DEFAULT = 2.0

# ── 抓取起始 hand 角度 ─────────────────────────────────────────────────────
PICK_START_HAND_DEG = -15.0   # 2026-08-06: S 姿态 hand 固定 -15°

# ── S 姿态 (track_velocity_pick 起始位 / 循环切 S 用) ──────────────────────
# arm_angle_deg 优先从 cfg["arm_pick_pose"] 读; x/y/hand 在此写死
S_POSE_Y_MM = -100.0    # 安全抬升高度 (mm)
S_POSE_X_MM = -80.0     # 主循环切 S 用 x (mm)
S_POSE_HAND_DEG = 0.0   # 主循环切 S 用 hand (deg)

# ── PLACE 姿态 (释放工作平面) ──────────────────────────────────────────────
PLACE_ARM_DEG = 90.0
PLACE_HAND_DEG = 0.0
PLACE_Y_MM = -100.0          # 工作平面安全高度 (mm)
PLACE_X_MM_FALLBACK = -235.0 # 唯一依据，禁止从 cfg / overrides 覆盖
PLACE_ALIGN_X_MM = -300.0    # 视觉对齐时稍收回 (mm)

# ── 释放 y 轨迹 (单位 mm; 负 = 向下) ──────────────────────────────────────
PLACE_DESCEND_MM = -20.0   # 吸住后下降到 -20mm
PLACE_LIFT_MM = -40.0      # 释放后抬离到 -40mm, 防拖拽
# composite_run HTTP 层用 m, 由调用处 /1000.0 转换

# ── 底盘安全约束 ──────────────────────────────────────────────────────────
# y 高于此值 (mm) 时才允许并发移动底盘 + 机械臂 (防撞)
CHASSIS_CONCURRENT_Y_THRESHOLD_MM = -30.0

# ── composite_run 公共参数 ─────────────────────────────────────────────────
COMPOSITE_SPEED_DEFAULT = 100
COMPOSITE_TIMEOUT_S_DEFAULT = 5.0


# ── 视觉读取（cam2 task_feed 缓存） ─────────────────────────────────────────

# 每列允许看到的源头 label: 三个圆柱 (1=大/2=中/3=小)
SOURCE_LABELS: tuple = ("cylinder_1", "cylinder_2", "cylinder_3")

# 吸嘴 setpoint (2026-08-02 标定, hand=0°): 目标在吸嘴正下方时其 bbox 中心坐标; 按 label 分组查表
# (cylinder_1/2/3 → (0.161,-0.519), ball_* 各自分档, 未知回落全局默认).
# 2026-08-06: S 姿态 hand=-15° 后吸嘴倾斜, setpoint 需重新标定, 此处先占位.
TASK1_NOZZLE_OFFSET_MAP: Dict[str, Tuple[float, float]] = {
    "cylinder_1": (0.050, -0.425),  # TODO: hand=-15° 后重定位
    "cylinder_2": (0.140, -0.420),  # TODO: hand=-15° 后重定位
    "cylinder_3": (0.120, -0.410),  # TODO: hand=-15° 后重定位
}
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
    seen: Dict[str, Any],
) -> str:
    """第 i 列: 移到源列, 扫描, 视觉伺服对准, 抓.

    2026-08-02 五件事:
      1. 视觉伺服从 S 姿态 (current state) 起 — 不再 hardcoded x_start=0, arm_start=-90
      2. gain_arm 0.4→0.8, gain_x 0.08→0.15, deadzone 0.02→0.04, max_vel 0.15→0.30 灵敏++
      3. arm 范围实际能 (+90, -150), arm_start 由 cfg 控制, 默认 -90
      4. timeout 15→25s 给 servo 足够时间
      5. servo 失败 → 写死 fallback (低吸, 不对齐也要拿起来, 跑完全程)

    Args:
        seen: run() 传入的本次运行共享 dict, 记录首次识别 label 供 1↔3 纠错。
              2026-08-03: 从函数属性 _pick_at_source._seen_first 挪进来 ——
              函数属性会跨测试/跨运行泄漏状态 (测试互相污染的根因)。

    Returns: 抓到的 cylinder label (1/2/3).
    """
    # 2026-08-06: S 姿态 hand=-15°, 到姿后等 1s 让振动/视觉稳定再开始定位
    time.sleep(1.0)
    # 2026-08-02: scan 1 retry, no backoff; 多 cylinder 视野取最近 setpoint
    # scan 阶段 label 未知, 先取任意 cylinder 的 setpoint 当默认（task1 只扫 cylinder）
    setpoint_xy = next(iter(TASK1_NOZZLE_OFFSET_MAP.values()))
    logger.info("[S%d] 视觉扫描源头 cylinder label (setpoint=%s)",
                column_idx, setpoint_xy)
    label = _scan_cylinder_label(
        client, list(SOURCE_LABELS),
        retries=1, backoff_s=0.0,
        setpoint_xy=setpoint_xy,
    )
    if label is None:
        raise RuntimeError(
            f"S{column_idx} 位置未检测到任何 cylinder ({list(SOURCE_LABELS)})"
        )

    # 用户 22:40: 全场只有 1 个 cylinder, 1 和 3 容易认错.
    # 第一次识别到啥就是啥; 之后如果又识别到同一个, 自动 swap 1↔3.
    # (seen 是 run() 内的本次运行 holder, 不再用函数属性)
    first = seen.get("first")
    if first is not None:
        if label == first and first in ("cylinder_1", "cylinder_3"):
            corrected = "cylinder_3" if first == "cylinder_1" else "cylinder_1"
            logger.info("  label 纠错: %s → %s (全场只有一个, 和第一次重复)", label, corrected)
            label = corrected
    else:
        seen["first"] = label
        logger.info("  首次识别: %s (后续 1↔3 自动纠错)", label)

    logger.info("  -> 抓到 %s, 智能定位抓取 (arm 控 cx + x 十字控 cy)", label)

    # 2026-08-02 调优: S 姿态就是工作起点, 不再跑去 x=0
    state = arm_client.get_state()
    init_y_mm = float(cfg.get("init_y_mm", -100.0))
    pick_arm_start = float(cfg.get("arm_pick_pose", {}).get("arm_angle_deg", -90.0))
    result = runner.track_velocity_pick(
        label,
        x_start=state.x_mm, y_start=init_y_mm,
        arm_start=pick_arm_start, hand_start=PICK_START_HAND_DEG,
        setpoint_x_norm=TASK1_NOZZLE_OFFSET_MAP[label][0],
        setpoint_y_norm=TASK1_NOZZLE_OFFSET_MAP[label][1],
        timeout=cfg.get("pick_track_timeout_s", PICK_SERVO_TIMEOUT_S_DEFAULT),
        hz=PICK_SERVO_HZ,
        gain_arm=PICK_SERVO_GAIN_ARM, gain_x=PICK_SERVO_GAIN_X,
        deadzone=PICK_SERVO_DEADZONE, max_vel=PICK_SERVO_MAX_VEL,
        settle_hits=PICK_SERVO_SETTLE_HITS,
        hold_s=PICK_SERVO_HOLD_S,
        lift_back=PICK_SERVO_LIFT_BACK,
        skip_pose_align=PICK_SERVO_SKIP_POSE_ALIGN,
    )
    if not result.get("ok"):
        # 用户 00:19: 不要 fallback! 太慢! 直接 raise, 主循环跳过该列
        raise RuntimeError(
            f"S{column_idx} pick 未收敛 (trace_hits={result.get('trace_hits')}, "
            f"end_arm={result.get('end_arm')})"
        )
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
    # 这里只做: move_y(-20) → grasp(False) → move_y(-40), 用 ThreadPoolExecutor 并发 y 下降 + 真空
    # 用户 (2026-08-03): "place 之后 y 要上升到 -40! 不然会把圆柱体拖走"
    logger.info("[T%d] [B+D] 顺序: move_y(%d) + grasp(False) + move_y(%d) 抬离", column_idx, PLACE_DESCEND_MM, PLACE_LIFT_MM)
    # move_y 走 _check_safe 不走 _check_y_protected, 可以直接到 -20
    runner.client.move_y(PLACE_DESCEND_MM, timeout=3.0)
    arm_client.grasp(False)
    runner.client.move_y(PLACE_LIFT_MM, timeout=3.0)


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

    # (a) 切 PLACE 姿态 (用 x=PLACE_ALIGN_X_MM 钉死, 用户 20:44 实测 marker 留中心)
    _switch_to_place_pose(arm_client, x_mm=PLACE_ALIGN_X_MM)

    # (b) 跑 track_chassis (用户 20:38 实机验证 33 帧 arrived)
    # 2026-08-03: max_seconds 从 cfg 读 (yaml 之前写 12.0 但代码硬编码 2.0, 配置撒谎),
    # 缺省回落 2.0; 想调直接改 task_config.yml chassis_align.max_seconds。
    # 2026-08-06: 失败重试 1 次 (用户决策)。align 是 task1 整个网格的原点参考,
    # 单次 timeout/no_target 就静默放行会让后面所有 S/T 列错位; 重试仍失败则
    # ERROR 大声告警但照样放行 (比赛完赛优先)。
    marker_label = cfg.get("marker_label", "cylinder_set")
    align_max_s = float(cfg.get("chassis_align", {}).get("max_seconds", 2.0))
    from main.chassis import track_chassis
    result = None
    for attempt in (1, 2):
        logger.info("  track_chassis #%d (target=%r, dry_run=False, max_seconds=%.1f)",
                    attempt, marker_label, align_max_s)
        # 2026-08-09: 控制律已下沉 runtime, on_tick 逐帧回调不可用 (会打 warning),
        # 逐帧轨迹改为 runtime result 的 frames/elapsed_s 摘要。
        result = track_chassis(
            marker_label,
            dry_run=False,
            max_seconds=align_max_s,
        )
        logger.info("  track_chassis #%d result: arrived=%s reason=%s frames=%d",
                    attempt, result.arrived, result.reason, result.frames)
        if result.arrived:
            break
        if attempt == 1:
            logger.warning("  对齐未收敛 (reason=%s), 重试第 2 次...", result.reason)
    if not result.arrived:
        logger.error("  底盘对齐两次均失败 (reason=%s)! task1 放行, 但后续 S/T 列"
                     "网格原点不可信, 请检查 task_feed / 检测 label / realtime 速度通道!",
                     result.reason)
    if not getattr(result, "stop_ok", True):
        logger.warning("  align 闭环零速下发失败 (stop_ok=False)! 显式停车兜底, "
                       "但注意底盘可能仍在按最后非零指令滑行 (串口/下位机异常)")
    # track_chassis 用 realtime/chassis-velocity, 结束后必须显式停车 + 确认停稳!
    # 用户 00:26: pick 第二个时车会往前跑 — 因为零速指令是异步的, 车还没停就开始 pick
    try:
        arm_client.http.post("/v1/realtime/chassis-velocity",
                             {"vx": 0.0, "vy": 0.0, "wz": 0.0}, timeout=2.0)
    except Exception:
        pass
    # 等车真正停稳: 真实编码器反馈双采样判定轮子不动才继续 (最多等 1s).
    # 2026-08-09: 旧实现 GET /v1/realtime/wheels/speeds 端点只存在 POST (405) →
    # 等待立即 break, "车会往前跑" 的保护从来没生效过; 改为编码器双采样
    # (与 target4 rearm 判停同构)。停稳失败只 warn, 不阻塞任务 (完赛优先)。
    if not arm_client.http.wait_wheels_stopped(settle_s=0.2, timeout_s=1.0):
        logger.warning("  align 后 1s 内轮子未停稳 (编码器仍位移), 继续但注意漂移")
    time.sleep(0.2)  # 额外喘气, 防 504
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


def _switch_to_place_pose(arm_client: ArmClient, x_mm: float = PLACE_ALIGN_X_MM) -> bool:
    """切到 PLACE 对齐姿态 (arm=+90, y=-100, hand=0, x=给参). 抬高 y 防止保护区拒绝."""
    state = arm_client.get_state()
    if state.y_mm > -50:
        if hasattr(arm_client, "move_y"):
            arm_client.move_y(PLACE_Y_MM)
        else:
            arm_client.http.execute_arm_action("move_y_position", PLACE_Y_MM, timeout=3.0)
    logger.info("  切 PLACE 姿态: arm=+90° x=%s y=%s hand=0°", x_mm, PLACE_Y_MM)
    ok = arm_client.composite_run(
        arm=PLACE_ARM_DEG, x_mm=x_mm, y_mm=PLACE_Y_MM, hand=PLACE_HAND_DEG,
        speed=COMPOSITE_SPEED_DEFAULT, timeout=20.0,
    )
    return ok.get("ok", False) if isinstance(ok, dict) else bool(ok)


def _init_step2_s_pose(runner: ArmRunner, arm_client: ArmClient, cfg: Dict[str, Any], init_y_mm: float) -> None:
    """step 2: 一次性走完 S 姿态 4 轴 (composite_run 并发).

    2026-08-02 旧 init step 2 现在被改成 S 姿态准备 (因为 step 1 已先到 PLACE).
    流程仍是 composite_run 把臂切到 S 姿态.
    """
    state = arm_client.get_state()
    if state.y_mm > -50:
        logger.warning("init step2: 当前 y=%.1f 太低, 先单步抬到 %s", state.y_mm, S_POSE_Y_MM)
        runner.client.move_y(S_POSE_Y_MM, timeout=3.0)
    pick = cfg["arm_pick_pose"]
    logger.info(
        "init step2: S 姿态 (composite_run) arm=%s° hand=%s° X=%s mm Y=%s mm",
        pick["arm_angle_deg"], pick["hand_angle_deg"], pick["x_mm"], S_POSE_Y_MM,
    )
    runner.client.composite_run(
        arm=float(pick["arm_angle_deg"]),
        x_mm=float(pick["x_mm"]),
        y_mm=init_y_mm,
        hand=PICK_START_HAND_DEG,  # 2026-08-06: S 姿态 hand 固定 -15°
        speed=COMPOSITE_SPEED_DEFAULT, timeout=20.0,
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
            "chassis_aligned": bool, # 2026-08-06: PLACE 对齐是否 arrived (含重试结果)
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

    def _odom_curr_x_y_theta() -> Tuple[float, float, float]:
        """读 odom_state (轮编码器反馈) 拿 chassis 当前 x, y (m), theta (rad).

        2026-08-03: theta 必须一起读 —— run.py 全流程到场时巡线累积了航向
        (实车实测一圈可到 0.39 rad)。移动目标 theta 硬写 0 会让 move_to_position
        先把车头转正到「任务启动时航向」, 那不是到场时的 S/T 列方向 → 底盘一动车
        就转头 + 斜着跑 (S2 乱跑根因)。theta=None (odom feed idle) 回落 0.0。
        """
        try:
            resp = arm_client.http.get("/v1/realtime/odom/state", timeout=3)
            odom = (resp or {}).get("odom_state") or {}
            theta = odom.get("theta")
            return (float(odom.get("x", 0.0)), float(odom.get("y", 0.0)),
                    float(theta) if theta is not None else 0.0)
        except Exception:
            return 0.0, 0.0, 0.0

    completed: List[str] = []

    # 用户 22:54: 不用 reset_x / reset_position, 直接到 S 姿态开始
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
            align_odom_x, _, _ = _odom_curr_x_y_theta()

        # ===== 初始化: 直接到 S 姿态开始 (用户 22:56: 不要 reset_x!) =====
        #   注意: 主循环 _pick_at_source 前也会调用 _init_step2_s_pose 来切换到 S 姿态.
        #   这里先跑一次是为了刷新视觉伺服起点 =====
        # (暂时跳过, 主循环已经处理)

        # ===== 主循环: 按 source_position_order 走底盘列 =====
        source_position_order = cfg["source_position_order"]
        target_slot_map = cfg["target_slot_map"]   # cylinder_N -> slot N (底盘位置)
        chassis_move_timeout = cfg["chassis_move_timeout_s"]

        last_chassis_col: Optional[int] = None

        # 1↔3 纠错的本次运行状态 (每轮 run() 独立, 不跨运行/测试泄漏)
        seen_state: Dict[str, Any] = {}

        # 2026-08-02 (3) (5): 真底盘位置记账 (米), 加并发 chassis+arm 调度.
        # 2026-08-02 (用户报 chassis 漂移不是直线):
        #   move_for([dx, 0, 0]) 是**开环**增量, 累计漂移 (上一轮跑完 x=1.40 y=0.31 theta=0.39,
        #   实际应该 x≈0.30 y≈0 theta=0).
        #   改用 move_to_position([target_x, curr_y, 0]) **闭环** (PID + odom feedback),
        #   自动纠 theta/y 漂移. 既然已知绝对目标, 不再需要 last_chassis_pos_m 记账.

        # 给 place 用的 PLACE 工作平面参数 (cfg 一次性读完)
        place_pose = cfg["arm_place_pose_T2"]
        place_arm   = float(place_pose["arm_angle_deg"])   # 90
        place_x_mm  = PLACE_X_MM_FALLBACK
        place_hand  = float(place_pose["hand_angle_deg"])  # 0 (保持)
        s_arm       = float(cfg["arm_pick_pose"]["arm_angle_deg"])  # -90
        s_x_mm      = float(cfg["arm_pick_pose"]["x_mm"])           # -100

        # 底盘纵向记账: 沿车头方向的物理相对位移 (m), S1 列为 0.
        # 2026-08-03: 旧版用「目标 odom x = align_odom_x + k*0.15」, 把 0.15 当成
        # odom x 轴增量。但全流程到场时 odom theta 漂到 ~0.97 rad (mecanum 横滑误
        # 积分; theta 对现实是垃圾, odom x/y/theta 三者却自洽), 轨道方向在 odom 系
        # 里是 theta 方向, 物理 15cm 只投影 0.15*cos(theta) ≈ 0.085 到 odom x →
        # 目标系统性偏远 1/cos(theta) ≈ 1.75 倍; 且 place 列移动后 curr_x 已含上次
        # 推进, 下一列再按网格算会重复记账 (实测 S2 列多走了 6.4cm, 全靠臂伺服补)。
        # 改成自记账: move_for([dx,0,0]) 沿车头闭环, dx = 目标相对位移 - 记账值,
        # 网格用真实物理间距, 不依赖 odom 绝对值/theta (theta≈0 的 standalone 等价)。
        pos_along = [0.0]

        def _chassis_goto(target_along_m: float) -> None:
            """闭环 chassis: 沿车头移动到相对位移 target_along_m (m, S1 列 = 0).

            move_for([dx,0,0]) 内部 = 偏移按当前 theta 旋转成绝对目标再走
            move_to_position 闭环; 世界→车速度转换用同一个 theta 逆变换, odom theta
            的漂移误差**互相抵消** → 轮速纯前进 → 物理上沿车头 (≈ 行方向) 直走,
            不转头不斜走 (2026-08-03 实车验证)。
            """
            dx = target_along_m - pos_along[0]
            if abs(dx) < 0.05:
                logger.info("  底盘已在相对 %.3f m (|dx|=%.3f < 5cm), 跳过移动",
                            target_along_m, abs(dx))
                return
            curr_x, curr_y, curr_theta = _odom_curr_x_y_theta()
            logger.info("  底盘闭环: move_for(dx=%+.3f, 0, 0) → 相对 %.3f m "
                        "(odom x=%.3f y=%.3f theta=%.3f)",
                        dx, target_along_m, curr_x, curr_y, curr_theta)
            arm_client.http.execute_car_action(
                "move_for", [dx, 0.0, 0.0],
                timeout=chassis_move_timeout, sync=True,
            )
            pos_along[0] = target_along_m

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
            curr_x, _, curr_theta = _odom_curr_x_y_theta()
            logger.info("=== 处理底盘列 %d (S%d, odom x=%.3f m, theta=%.3f rad) ===",
                        i + 1, column_idx, curr_x, curr_theta)

            # (1) 底盘闭环移到本列源 (物理相对位移: S1=0, S2=0.15, S3=0.30)
            # 用户 (2026-08-02 21:02): "step2 不调用 chassis-goto, 只用动机械臂"
            # step 1 align 后 chassis 已经在 S1=T1 列. 第 0 列 (i=0) 跳过底盘移动, 直接 S 姿态.
            if i > 0:
                target_s = SOURCE_POSITIONS_M[column_idx]
                logger.info("  底盘 → S%d (相对 %.3f m)", column_idx, target_s)
                _chassis_goto(target_s)
            else:
                logger.info("  step 1 align 后 chassis 已在 S1 列 (odom %.3f), 跳过底盘移动", curr_x)

            # (1.5) 切 S 姿态 — 全轴并发, timeout 5s (物理到位 ~3-4s, 之前 3s 不够)
            # 用 sync=False + 手动 poll 避免 504 (track_chassis 后 runtime HTTP 会卡)
            logger.info("  切 S 姿态: arm=%s° x=%s y=%s hand=%s°",
                        s_arm, S_POSE_X_MM, S_POSE_Y_MM, S_POSE_HAND_DEG)
            job = arm_client.http.execute(
                "arm", "composite_run",
                kwargs={"arm": s_arm, "x": S_POSE_X_MM / 1000.0,
                        "y": S_POSE_Y_MM / 1000.0, "hand": S_POSE_HAND_DEG,
                        "speed": COMPOSITE_SPEED_DEFAULT, "timeout": COMPOSITE_TIMEOUT_S_DEFAULT},
                sync=False,
            )
            job_id = job.get("id")
            if job_id:
                arm_client.http.wait_job(job_id, timeout=COMPOSITE_TIMEOUT_S_DEFAULT + 10)

            # (2) 抓 — 优化#5: 超时直接跳过该列, 不走 fallback
            try:
                label = _pick_at_source(runner, arm_client, client, cfg, column_idx,
                                        seen_state)
            except Exception as exc:
                picked_so_far = set(completed)
                remaining = [l for l in SOURCE_LABELS if l not in picked_so_far]
                if remaining:
                    label = remaining[0]
                    logger.warning("  S%d pick 失败 (%s), 兜底用剩余 label=%s", column_idx, exc, label)
                else:
                    logger.warning("  S%d pick 失败 (%s), 无剩余 label, 跳过", column_idx, exc)
                    continue
            completed.append(label)

            # (3) 优化#2: pick→PLACE 零串行! 一个 ThreadPool 全并发:
            #     y抬 + 底盘移T + arm切PLACE + x到place位
            # 用户 00:07: 唯一条件 — y<-30 才可以并发移动底盘和机械臂!
            slot_idx = int(target_slot_map[label])
            target_t = SLOT_POSITIONS_M[slot_idx]   # 物理相对位移; 与 S 列同网格 (S_i↔T_i 同列)
            place_x_override = place_x_mm
            # 用户 01:05: y<-30 绝对不能删! 防撞!
            st = arm_client.get_state()
            if st.y_mm > CHASSIS_CONCURRENT_Y_THRESHOLD_MM:
                arm_client.composite_run(arm=None, x_mm=None, y_mm=S_POSE_Y_MM, hand=None,
                                         speed=COMPOSITE_SPEED_DEFAULT, timeout=COMPOSITE_TIMEOUT_S_DEFAULT)
            logger.info("  → T%d (label=%s, x=%s) 全并发", slot_idx, label, place_x_override)
            # 2026-08-03 优化: 并发改成 sync=False, 否则两个 sync HTTP 同时打 /v1/execute
            # 会让 runtime 队列拥塞 504。
            with ThreadPoolExecutor(max_workers=2) as ex:
                f_chassis = ex.submit(_chassis_goto, target_t)
                f_arm = ex.submit(arm_client.http.execute,
                                  "arm", "composite_run",
                                  kwargs=dict(arm=place_arm, x=place_x_override / 1000.0,
                                              y=PLACE_Y_MM / 1000.0, hand=PLACE_HAND_DEG,
                                              speed=COMPOSITE_SPEED_DEFAULT, timeout=5.0),
                                  sync=False)
                f_chassis.result()
                arm_job = f_arm.result()
                ajid = arm_job.get("id") if isinstance(arm_job, dict) else None
                if ajid:
                    arm_client.http.wait_job(ajid, timeout=COMPOSITE_TIMEOUT_S_DEFAULT + 10)

            # (5) 放: y→-20 + grasp(False) 释放 + y→-40 抬离!
            # 用户 (2026-08-03): "place 之后 y 要上升到 -40! 不然会把圆柱体拖走"
            # 关键协议: 释放后必须立即抬到 y<=-40 才能离开当前列, 否则吸嘴会拖动落地的物体。
            logger.info("[T%d] place: y→-20 + grasp(False) + y→-40 抬离", slot_idx)
            # 5a) 下降到 -20 (必须等到位才能释放, 否则 vacuum 开着物体没到位)
            job1 = arm_client.http.execute(
                "arm", "composite_run",
                kwargs=dict(arm=None, x=None, y=PLACE_DESCEND_MM / 1000.0, hand=None,
                            speed=COMPOSITE_SPEED_DEFAULT, timeout=5.0),
                sync=False,
            )
            jid1 = job1.get("id") if isinstance(job1, dict) else None
            # 2026-08-03: timeout 3→5s. 物理 2-3s 边界, 之前 3s 偶发超时 → grasp 没发
            # → 主循环走兜底盘 504 timeout.
            if jid1:
                arm_client.http.wait_job(jid1, timeout=COMPOSITE_TIMEOUT_S_DEFAULT + 10)
            # 5b) grasp(False) 释放 — 100ms 即完成
            arm_client.grasp(False)
            # 5c) 立即抬到 -40 (离开保护区更远, 跨列移动时不拖物体)
            job2 = arm_client.http.execute(
                "arm", "composite_run",
                kwargs=dict(arm=None, x=None, y=PLACE_LIFT_MM / 1000.0, hand=None,
                            speed=COMPOSITE_SPEED_DEFAULT, timeout=5.0),
                sync=False,
            )
            jid2 = job2.get("id") if isinstance(job2, dict) else None
            if jid2:
                arm_client.http.wait_job(jid2, timeout=COMPOSITE_TIMEOUT_S_DEFAULT + 10)

            # (6) 优化#3: y抬回 + 底盘移下一列 + 切PLACE对齐 全并发!
            # 用户 00:07: y<-30 才可以并发!
            if i + 1 < len(source_position_order):
                next_col_idx = source_position_order[i + 1]
                logger.info("  列 %d 完成, 底盘相对位移 %.3f m (下一列 S%d)",
                            column_idx, pos_along[0], next_col_idx)

    except Exception as exc:
        logger.exception("task1_seeding 失败: %s", exc)
        # 2026-08-07: 用户要求"不管结束在哪, 都在 S3 停", 异常路径也要尝试移到 S3
        # 作为终点, 否则任务炸了车会卡在赛道中间挡道。
        try:
            arm_client.http.execute_car_action(
                "move_for",
                [SOURCE_POSITIONS_M[3] - pos_along[0], 0.0, 0.0],
                timeout=chassis_move_timeout, sync=True,
            )
            pos_along[0] = SOURCE_POSITIONS_M[3]
            logger.info("  异常路径也已把底盘移到 S3 (%.3f m)", pos_along[0])
        except Exception as move_exc:
            logger.warning("  异常路径移到 S3 失败 (原异常优先): %s", move_exc)
        return {"ok": False, "completed": completed, "error": str(exc)}

    # task 业务结束, 机械臂归位交给 orchestrator._schedule_arm_home_reset
    # (2026-08-03 重构, 不在 task 里做 reset, 边重置边巡航由编排层统一处理).
    # 2026-08-07: 用户要求"不管结束在哪, 都在 S3 停作为终点"。
    # 沿车头闭环 move_for([dx,0,0]), 不转头不斜走 (2026-08-03 实车验证)。
    try:
        s3_target = SOURCE_POSITIONS_M[3]
        s3_dx = s3_target - pos_along[0]
        if abs(s3_dx) >= 0.05:
            logger.info("task1 结束, 底盘移到 S3 (%.3f m, dx=%+.3f m)", s3_target, s3_dx)
            arm_client.http.execute_car_action(
                "move_for",
                [s3_dx, 0.0, 0.0],
                timeout=chassis_move_timeout, sync=True,
            )
        else:
            logger.info("task1 结束, 底盘已在 S3 (%.3f m, |dx|=%.3f < 5cm) 跳过移动",
                        s3_target, abs(s3_dx))
        pos_along[0] = s3_target
    except Exception as move_exc:
        logger.warning("task1 末尾移到 S3 失败 (任务已成功): %s", move_exc)

    return {"ok": True, "completed": completed, "chassis_aligned": align_arrived}


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
    result = run()
    print("任务一 自动移苗 执行结果:", result)