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
import math
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
PICK_SERVO_GAIN_ARM = 0.5     # 2026-08-10: 0.7 → 0.5 (用户: 大臂增量再调缓, 每帧 +0.7×dx → +0.5×dx)
PICK_SERVO_GAIN_X = 0.30
PICK_SERVO_DEADZONE = 0.05    # 2026-08-10: 0.06 → 0.05 (第一次死区收紧)
PICK_SERVO_MAX_VEL = 0.3      # 2026-08-10: 0.5 → 0.3 (x十字速度上限收紧)
PICK_SERVO_SETTLE_HITS = 3
PICK_SERVO_HOLD_S = 0.05
PICK_SERVO_LIFT_BACK = True
PICK_SERVO_SKIP_POSE_ALIGN = True
PICK_SERVO_HZ = 20.0
# pick_track_timeout_s 优先从 cfg 读, 此处为缺省值
PICK_SERVO_TIMEOUT_S_DEFAULT = 4.0
# ── 对齐失败重试 (2026-08-09 用户): 失败但看得到目标 → 加时 3s + 死区调大 ──
PICK_SERVO_RETRY_TIMEOUT_EXTRA_S = 3.0   # 重试超时在原超时上加 3s
PICK_SERVO_RETRY_DEADZONE = 0.10         # 重试死区调大 (第一次 0.05 → 0.10), 更容易锁上

# ── 抓取起始 hand 角度 ─────────────────────────────────────────────────────
PICK_START_HAND_DEG = -15.0   # 2026-08-06: S 姿态 hand 固定 -15°

# ── S 姿态 (track_velocity_pick 起始位 / 循环切 S 用) ──────────────────────
# arm_angle_deg 优先从 cfg["arm_pick_pose"] 读; x/y/hand 在此写死
S_POSE_Y_MM = -100.0    # 安全抬升高度 (mm)
S_POSE_X_MM = -70.0     # 主循环切 S 用 x (mm)
S_POSE_HAND_DEG = 0.0   # 主循环切 S 用 hand (deg)

# ── PLACE 姿态 (释放工作平面) ──────────────────────────────────────────────
PLACE_ARM_DEG = 90.0
PLACE_HAND_DEG = 0.0
PLACE_Y_MM = -100.0          # 工作平面安全高度 (mm)
PLACE_X_MM_FALLBACK = -235.0 # 唯一依据，禁止从 cfg / overrides 覆盖
# 2026-08-10: PLACE 对齐初始姿态 = 现 place 姿态 (原 -300 是"视觉对齐稍收回",
# 用户改为 place_x=-235 起步, 见 task_config.yml place_align 段).
PLACE_ALIGN_X_MM = -235.0
PLACE_ALIGN_LABEL = "cylinder_set"
PLACE_ALIGN_SETPOINT_CXCY = (0.072, -0.331)  # 2026-08-10 用户标定 (0.042,-0.359) → (0.072,-0.331)
PLACE_ALIGN_TIMEOUT_S = 5.0
PLACE_ALIGN_SERVO_ARM_MIN = 30.0  # 起始 +90, 允许向下微调到 +30
PLACE_ALIGN_SERVO_ARM_MAX = 150.0  # 2026-08-10: 90 → 150 (clamp 放宽, 与 yaml arm_max 对齐)
PLACE_ALIGN_SERVO_SIGN_ARM = 1.0  # ⚠️ 沿用抓苗标定; 若"越对越偏"取反 (task2 同规则)
PLACE_ALIGN_SERVO_SIGN_X = -1.0   # ⚠️ 同上

# ── 释放 y 轨迹 (单位 mm; 负 = 向下) ──────────────────────────────────────
PLACE_DESCEND_MM = -20.0   # 吸住后下降到 -20mm
PLACE_LIFT_MM = -40.0      # 释放后抬离到 -40mm, 防拖拽
# composite_run HTTP 层用 m, 由调用处 /1000.0 转换

# ── 底盘安全约束 ──────────────────────────────────────────────────────────
# y 高于此值 (mm) 时才允许并发移动底盘 + 机械臂 (防撞)
CHASSIS_CONCURRENT_Y_THRESHOLD_MM = -30.0

# ── 底盘网格移动速度 (move_for max_velocities) ───────────────────────────
# 2026-08-10: SDK 默认 0.2 → 0.1 m/s (xy 平移降速, 角速度留 π/3 默认)
CHASSIS_MOVE_MAX_VEL_MPS = 0.1

# ── composite_run 公共参数 ─────────────────────────────────────────────────
COMPOSITE_SPEED_DEFAULT = 100
COMPOSITE_TIMEOUT_S_DEFAULT = 5.0

# ── step0: 任务点触发后 lane follow 并发臂切 PLACE (2026-08-09 用户) ──────
TRIGGER_SETTLE_LANE_M = 0.1    # lane follow 前移距离 (m), 速度 0.1m/s
TRIGGER_SETTLE_LANE_VX = 0.1   # lane follow 速度 (m/s)


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

def _pick_cylinder_servo_local(
    arm_client: ArmClient,
    runner: ArmRunner,
    cfg: Dict[str, Any],
    label: str,
    setpoint_xy: Tuple[float, float],
    init_y_mm: float,
    pick_arm_start: float,
) -> Dict[str, Any]:
    """本地视觉伺服抓苗 (2026-08-09 闭环下沉): runtime 进程内 run_arm_servo.

    mirror task1 的 track_velocity_pick: 对齐收敛 → y 降 0 (grasp_y) → grasp →
    抬回 init_y_mm (fire-and-forget). main 只发一次目标参数, 无每帧网络往返.
    控制律 (arm 控 cx + x 十字控 cy) 与 find_target_arm_cross 同构, 方向符号沿用
    task1 标定: sign_arm=+1, sign_x=-1.

    Returns: run_arm_servo 结果 dict (含 ok/settled/trace_hits/end_arm/end_x).
    未收敛抛 RuntimeError (主循环跳过该列, 与旧路径一致).
    """
    vision = cfg.get("pick_vision") or {}
    servo_timeout = float(vision.get(
        "timeout", cfg.get("pick_track_timeout_s", PICK_SERVO_TIMEOUT_S_DEFAULT)))
    servo_kw = dict(
        label=label,
        hz=PICK_SERVO_HZ,
        gain_arm=PICK_SERVO_GAIN_ARM,
        gain_x=PICK_SERVO_GAIN_X,
        deadzone=PICK_SERVO_DEADZONE,
        max_vel=PICK_SERVO_MAX_VEL,
        arm_start=float(pick_arm_start),
        sign_arm=1.0,
        sign_x=-1.0,
        setpoint_x_norm=float(setpoint_xy[0]),
        setpoint_y_norm=float(setpoint_xy[1]),
        arm_min=-150.0,
        arm_max=90.0,
        servo_timeout=servo_timeout,
        settle_hits=int(PICK_SERVO_SETTLE_HITS),
    )
    logger.info(
        "本地视觉伺服: run_arm_servo(label=%s setpoint=(%.3f,%.3f) hz=%s "
        "gain_arm=%s gain_x=%s deadzone=%s max_vel=%s arm_start=%s settle=%s servo_timeout=%s)",
        label, servo_kw["setpoint_x_norm"], servo_kw["setpoint_y_norm"],
        servo_kw["hz"], servo_kw["gain_arm"], servo_kw["gain_x"],
        servo_kw["deadzone"], servo_kw["max_vel"], servo_kw["arm_start"],
        servo_kw["settle_hits"], servo_kw["servo_timeout"],
    )
    def _servo_once(kw: Dict[str, Any]) -> Dict[str, Any]:
        """跑一轮 run_arm_servo 并解析 result (含 status 失败 raise)。"""
        job = arm_client.http.execute(
            "car", "run_arm_servo", kwargs=kw, sync=True,
            timeout=float(kw["servo_timeout"]) + 15.0,
        )
        result = (job or {}).get("result") if isinstance(job, dict) else None
        result = result if isinstance(result, dict) else {}
        if isinstance(job, dict) and job.get("status") not in (None, "succeeded"):
            raise RuntimeError(
                f"run_arm_servo 任务失败: status={job.get('status')} error={job.get('error')}"
            )
        return result

    result = _servo_once(servo_kw)
    logger.info("本地视觉伺服结果: reason=%s settled=%s trace_hits=%s end_arm=%s end_x=%s",
                result.get("reason"), result.get("settled"),
                result.get("trace_hits"), result.get("end_arm"), result.get("end_x"))
    # 2026-08-09 用户: 对齐失败但这一轮看到过目标 (trace_hits>0) →
    # 加时 3s + 死区调大重试一次。目标还在视野里只是振荡/太慢没进死区,
    # 再给一次机会大概率能锁上; trace_hits=0 全程没看到目标, 重试也白搭。
    if (not result.get("settled")
            and int(result.get("trace_hits", 0) or 0) > 0):
        retry_kw = dict(servo_kw)
        retry_kw["servo_timeout"] = (float(servo_kw["servo_timeout"])
                                     + PICK_SERVO_RETRY_TIMEOUT_EXTRA_S)
        retry_kw["deadzone"] = PICK_SERVO_RETRY_DEADZONE
        # 从上次结束角度续跑, 不回到 arm_start (-90) 起点, 免得把已对齐的角度丢回去
        if result.get("end_arm") is not None:
            retry_kw["arm_start"] = float(result["end_arm"])
        logger.warning(
            "S 视觉抓苗未收敛但看得到目标 (reason=%s trace_hits=%s) → 重试: "
            "超时 %ss + 死区 %.3f",
            result.get("reason"), result.get("trace_hits"),
            retry_kw["servo_timeout"], retry_kw["deadzone"],
        )
        result = _servo_once(retry_kw)
        logger.info("重试视觉伺服结果: reason=%s settled=%s trace_hits=%s end_arm=%s end_x=%s",
                    result.get("reason"), result.get("settled"),
                    result.get("trace_hits"), result.get("end_arm"), result.get("end_x"))
    if not result.get("settled"):
        raise RuntimeError(
            f"S 视觉抓苗未收敛 (reason={result.get('reason')}, "
            f"trace_hits={result.get('trace_hits')}, end_arm={result.get('end_arm')})"
        )
    # 对齐完成 → y 降 0 → grasp → 抬回 init_y (mirror track_velocity_pick grasp 段)
    try:
        job_down = arm_client.http.execute(
            "arm", "composite_run",
            kwargs=dict(arm=None, x=None, y=0.0, hand=None, speed=100, timeout=5.0),
            sync=False,
        )
        jid = job_down.get("id") if isinstance(job_down, dict) else None
        if jid:
            arm_client.http.wait_job(jid, timeout=5.0)
        arm_client.http.execute("arm", "grasp", kwargs=dict(value=True), sync=False)
        # 抬回 init_y fire-and-forget (下游 move 并发)
        arm_client.http.execute(
            "arm", "composite_run",
            kwargs=dict(arm=None, x=None, y=float(init_y_mm) / 1000.0, hand=None,
                        speed=100, timeout=5.0),
            sync=False,
        )
    except Exception as exc:
        try:
            arm_client.http.execute(
                "arm", "move_y_position",
                kwargs=dict(target=float(init_y_mm) / 1000.0, timeout=5.0), sync=False,
            )
        except Exception:
            pass
        raise RuntimeError(f"本地视觉抓苗 grasp 段失败: {exc}") from exc
    return result


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
    # 2026-08-09: local_servo 开关 — true=本机闭环 run_arm_servo, false=旧网络每帧
    if (cfg.get("pick_vision") or {}).get("local_servo"):
        result = _pick_cylinder_servo_local(
            arm_client, runner, cfg, label,
            setpoint_xy=TASK1_NOZZLE_OFFSET_MAP[label],
            init_y_mm=init_y_mm, pick_arm_start=pick_arm_start,
        )
    else:
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


def _init_step0_trigger_lane_arm(
    arm_client: ArmClient,
    cfg: Dict[str, Any],
) -> bool:
    """2026-08-09: 任务点触发后 step0 — lane follow 前移 与 臂切 PLACE 并发.

    编排层 waypoint 触发 (右侧 IR<阈值 AND 里程≥阈值) 后、step1 视觉对齐前:
      1. 底盘 move_along_lane(vx, distance_m) 沿车道线前进 (拉近 marker, 不偏航)
      2. 机械臂并发 composite_run 切 PLACE 对齐姿态 (arm=+90, x=-300, y=-100,
         hand=0) — "进入任务点就动臂" (task2 同款并发范式, task1_seeding 原本
         step1 也要切 PLACE, 提前并发省串行等待).

    参数走 task_config.yml trigger_settle 段 (enabled / lane_follow_m /
    lane_speed_mps). 关闭或 lane_m<=0 → 原样返回.

    安全: 当前 y > -50 (臂偏低) 先串行抬到 PLACE_Y_MM 再并发, 防底盘前进撞臂
    (与 _switch_to_place_pose 同规则). lane follow 需要 lane_feed 存活
    (orchestrator 触发后只 pause 外环 runner, feed 仍跑, 与 settle_forward 同理).

    失败不阻塞任务: lane follow 或臂切失败只记 warning, 继续 step1.
    Returns: lane follow 且 臂切 是否都完成.
    """
    settle = cfg.get("trigger_settle") or {}
    if not settle.get("enabled", False):
        return True
    lane_m = float(settle.get("lane_follow_m", TRIGGER_SETTLE_LANE_M))
    lane_vx = float(settle.get("lane_speed_mps", TRIGGER_SETTLE_LANE_VX))
    if lane_m <= 0:
        return True

    # 安全: y 偏低先抬 (串行 ~0.5s), 否则底盘前进时臂还低着会撞
    try:
        st = arm_client.get_state()
        if st.y_mm > -50:
            logger.info("step0: 当前 y=%.1f 偏低, 先抬到 %s 再并发", st.y_mm, PLACE_Y_MM)
            arm_client.move_y(PLACE_Y_MM, timeout=5.0)
    except Exception as exc:
        logger.warning("step0: 检查/抬升 y 失败 (%s), 继续", exc)

    from concurrent.futures import ThreadPoolExecutor
    from main.chassis import move_along_lane

    logger.info("step0: lane follow %.2fm @ %.2fm/s 并发臂切 PLACE "
                "(arm=%s° x=%s y=%s hand=%s°)",
                lane_m, lane_vx, PLACE_ARM_DEG, PLACE_ALIGN_X_MM,
                PLACE_Y_MM, PLACE_HAND_DEG)
    lane_done = True
    arm_ok = False
    with ThreadPoolExecutor(max_workers=2) as ex:
        f_lane = ex.submit(move_along_lane, vx=lane_vx, distance_m=lane_m)
        f_arm = ex.submit(
            arm_client.composite_run,
            arm=PLACE_ARM_DEG, x_mm=PLACE_ALIGN_X_MM, y_mm=PLACE_Y_MM,
            hand=PLACE_HAND_DEG, speed=COMPOSITE_SPEED_DEFAULT, timeout=20.0,
        )
        try:
            f_lane.result()
        except Exception as exc:
            lane_done = False
            logger.warning("step0: lane follow 失败 (%s), 继续 step1", exc)
        try:
            arm_ok = bool(f_arm.result())
        except Exception as exc:
            logger.warning("step0: 臂切 PLACE 失败 (%s), 继续 step1", exc)
    if not lane_done:
        logger.warning("step0: lane follow 未完成, step1 对齐从当前位置开始")
    return lane_done and arm_ok


def _init_step1_place_align_arm(
    arm_client: ArmClient,
    cfg: Dict[str, Any],
) -> Dict[str, Any]:
    """step 1 (2026-08-10 替代原底盘对齐): 放苗侧 cylinder_set 机械臂视觉对齐.

    原 track_chassis("cylinder_set") 底盘对齐已禁用 → 改机械臂对齐:
      1. 切初始姿态 arm=+90, x=-235, y=-100, hand=0 (= 现 place 姿态)
      2. run_arm_servo("cylinder_set", setpoint=(cx,cy)) 视觉伺服收敛
         (大臂控 cx + x 十字控 cy, 与抓苗同机制, runtime 进程内闭环)
      3. 记住收敛终点 (end_arm, end_x) → 本次运行放苗直接开到这个姿态,
         单次运行内恒定, 跨运行重新对齐重记.
    未收敛 → 回落写死 place 姿态 (-235/+90), 完赛优先.

    Returns: {"ok": bool, "arm_deg": float|None, "x_mm": float|None,
              "reason": str|None, "end_arm": float|None, "end_x": float|None}
    """
    pa = cfg.get("place_align") or {}
    label = str(pa.get("label", PLACE_ALIGN_LABEL))
    sp = tuple(float(v) for v in pa.get("setpoint_cxcy", PLACE_ALIGN_SETPOINT_CXCY))
    init_x_mm = float(pa.get("init_x_mm", PLACE_ALIGN_X_MM))
    init_arm_deg = float(pa.get("init_arm_deg", PLACE_ARM_DEG))
    init_hand_deg = float(pa.get("init_hand_deg", PLACE_HAND_DEG))

    # (a) 切对齐初始姿态 (现 place 姿态)
    try:
        _switch_to_place_pose(arm_client, x_mm=init_x_mm, arm_deg=init_arm_deg,
                              hand_deg=init_hand_deg)
    except Exception as exc:
        logger.error("step 1: 切 place_align 初始姿态失败 (%s), 回落写死 place 姿态", exc)
        return {"ok": False, "arm_deg": None, "x_mm": None,
                "reason": f"init_pose_fail: {exc}", "end_arm": None, "end_x": None}

    # (b) 视觉伺服收敛 (run_arm_servo, 参数走 yaml place_align 段, 缺省回落 PICK_SERVO_*)
    servo_kw = dict(
        label=label,
        hz=float(pa.get("hz", PICK_SERVO_HZ)),
        gain_arm=float(pa.get("gain_arm", PICK_SERVO_GAIN_ARM)),
        gain_x=float(pa.get("gain_x", PICK_SERVO_GAIN_X)),
        deadzone=float(pa.get("deadzone", PICK_SERVO_DEADZONE)),
        max_vel=float(pa.get("max_vel", PICK_SERVO_MAX_VEL)),
        arm_start=init_arm_deg,
        sign_arm=float(pa.get("sign_arm", PLACE_ALIGN_SERVO_SIGN_ARM)),
        sign_x=float(pa.get("sign_x", PLACE_ALIGN_SERVO_SIGN_X)),
        setpoint_x_norm=sp[0],
        setpoint_y_norm=sp[1],
        arm_min=float(pa.get("arm_min", PLACE_ALIGN_SERVO_ARM_MIN)),
        arm_max=float(pa.get("arm_max", PLACE_ALIGN_SERVO_ARM_MAX)),
        servo_timeout=float(pa.get("timeout", PLACE_ALIGN_TIMEOUT_S)),
        settle_hits=int(pa.get("settle_hits", PICK_SERVO_SETTLE_HITS)),
    )
    logger.info("step 1 (新): PLACE 机械臂对齐 cylinder_set — run_arm_servo(label=%s "
                "setpoint=(%.3f,%.3f) arm_start=%.0f x_init=%.0f)",
                label, sp[0], sp[1], init_arm_deg, init_x_mm)

    def _servo_once(kw: Dict[str, Any]) -> Dict[str, Any]:
        job = arm_client.http.execute(
            "car", "run_arm_servo", kwargs=kw, sync=True,
            timeout=float(kw["servo_timeout"]) + 15.0,
        )
        result = (job or {}).get("result") if isinstance(job, dict) else None
        result = result if isinstance(result, dict) else {}
        if isinstance(job, dict) and job.get("status") not in (None, "succeeded"):
            raise RuntimeError(
                f"run_arm_servo 任务失败: status={job.get('status')} error={job.get('error')}"
            )
        return result

    result = _servo_once(servo_kw)
    logger.info("place 对齐结果: reason=%s settled=%s trace_hits=%s end_arm=%s end_x=%s",
                result.get("reason"), result.get("settled"),
                result.get("trace_hits"), result.get("end_arm"), result.get("end_x"))
    # 对齐失败但这一轮看到过目标 (trace_hits>0) → 加时 + 死区调大重试一次
    if (not result.get("settled")
            and int(result.get("trace_hits", 0) or 0) > 0):
        retry_kw = dict(servo_kw)
        retry_kw["servo_timeout"] = (float(servo_kw["servo_timeout"])
                                     + PICK_SERVO_RETRY_TIMEOUT_EXTRA_S)
        retry_kw["deadzone"] = PICK_SERVO_RETRY_DEADZONE
        if result.get("end_arm") is not None:
            retry_kw["arm_start"] = float(result["end_arm"])
        logger.warning("place 对齐未收敛但看得到目标 → 重试: 超时 %ss + 死区 %.3f",
                       retry_kw["servo_timeout"], retry_kw["deadzone"])
        result = _servo_once(retry_kw)
        logger.info("place 对齐重试结果: reason=%s settled=%s end_arm=%s end_x=%s",
                    result.get("reason"), result.get("settled"),
                    result.get("end_arm"), result.get("end_x"))

    arm_deg = float(result["end_arm"]) if result.get("end_arm") is not None else None
    x_mm = float(result["end_x"]) * 1000.0 if result.get("end_x") is not None else None
    # ok 需 arm 和 x 都在 (end_x 缺失 = runtime 未升级, 也算失败, 防放苗段 float(None))
    ok = bool(result.get("settled")) and arm_deg is not None and x_mm is not None
    if ok:
        logger.info("step 1: 记住放苗姿态 arm=%.1f° x=%.1fmm (本次运行通用)", arm_deg, x_mm)
    else:
        logger.error("step 1: place 对齐未收敛 (reason=%s), 回落写死 place 姿态 (-235/+90)",
                     result.get("reason"))
    return {"ok": ok, "arm_deg": arm_deg, "x_mm": x_mm,
            "reason": result.get("reason"),
            "end_arm": result.get("end_arm"), "end_x": result.get("end_x")}


def _switch_to_place_pose(arm_client: ArmClient, x_mm: float = PLACE_ALIGN_X_MM,
                          arm_deg: float = PLACE_ARM_DEG,
                          hand_deg: float = PLACE_HAND_DEG) -> bool:
    """切到 PLACE 对齐姿态 (arm=给参默认+90, y=-100, hand=给参默认0, x=给参). 抬高 y 防止保护区拒绝."""
    state = arm_client.get_state()
    if state.y_mm > -50:
        if hasattr(arm_client, "move_y"):
            arm_client.move_y(PLACE_Y_MM)
        else:
            arm_client.http.execute_arm_action("move_y_position", PLACE_Y_MM, timeout=3.0)
    logger.info("  切 PLACE 姿态: arm=%s° x=%s y=%s hand=%s°", arm_deg, x_mm, PLACE_Y_MM, hand_deg)
    ok = arm_client.composite_run(
        arm=arm_deg, x_mm=x_mm, y_mm=PLACE_Y_MM, hand=hand_deg,
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
        # ===== 初始化步骤 0 (2026-08-09 用户): 任务点触发后 lane follow 0.15m
        #     并发臂切 PLACE 初始姿态 (arm=+90, x=-235, y=-100, hand=0) =====
        _init_step0_trigger_lane_arm(arm_client, cfg)
        # ===== 初始化步骤 1 (2026-08-10): 放苗侧 cylinder_set 机械臂对齐 =====
        # 原 track_chassis 底盘对齐已禁用 → 改机械臂视觉对齐 (place_align 段):
        #   1. 切初始姿态 arm=+90, x=-235, y=-100, hand=0 (= 现 place 姿态)
        #   2. run_arm_servo("cylinder_set") 收敛 → 记住 (arm, x)
        #   3. 本次运行放苗直接开到这个记忆姿态 (不再用写死 -235/+90)
        # 未收敛 → 回落写死 place 姿态 (-235/+90), 完赛优先.
        align_arrived = False
        align_odom_x = 0.0
        place_mem: Dict[str, Any] = {"ok": False, "arm_deg": None, "x_mm": None}
        if cfg.get("place_align", {}).get("enabled", False):
            place_mem = _init_step1_place_align_arm(arm_client, cfg)
            align_arrived = place_mem.get("ok", False)
        else:
            logger.info("step 1: PLACE 机械臂对齐已禁用 (place_align.enabled=False)")
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
        place_arm   = float(place_pose["arm_angle_deg"])   # 90 (place_align 失败兜底)
        place_x_mm  = PLACE_X_MM_FALLBACK                   # -235 (同上兜底)
        place_hand  = float(place_pose["hand_angle_deg"])  # 0 (保持)
        # 2026-08-10: place_align 记住的对齐终点优先 — 放苗直接开到记忆姿态
        if place_mem.get("ok"):
            place_arm = float(place_mem["arm_deg"])
            place_x_mm = float(place_mem["x_mm"])
            logger.info("  放苗姿态用记忆值: arm=%.1f° x=%.1fmm (本次运行通用)",
                        place_arm, place_x_mm)
        else:
            logger.info("  放苗姿态用兜底写死: arm=%.1f° x=%.1fmm", place_arm, place_x_mm)
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
                max_velocities=[CHASSIS_MOVE_MAX_VEL_MPS, CHASSIS_MOVE_MAX_VEL_MPS, math.pi / 3.0],
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

            # (1.5) 切 S 姿态 — 全轴并发, timeout 20s (物理到位 ~3-4s + 大臂 +90→-90 摆动
            # 实测 ~5-6s, 之前 5s 不够会内部 TimeoutError, 虽不阻塞但结果不可信)
            # 用 sync=False + 手动 poll 避免 504 (track_chassis 后 runtime HTTP 会卡)
            logger.info("  切 S 姿态: arm=%s° x=%s y=%s hand=%s°",
                        s_arm, S_POSE_X_MM, S_POSE_Y_MM, S_POSE_HAND_DEG)
            job = arm_client.http.execute(
                "arm", "composite_run",
                kwargs={"arm": s_arm, "x": S_POSE_X_MM / 1000.0,
                        "y": S_POSE_Y_MM / 1000.0, "hand": S_POSE_HAND_DEG,
                        "speed": COMPOSITE_SPEED_DEFAULT, "timeout": 20.0},
                sync=False,
            )
            job_id = job.get("id")
            if job_id:
                arm_client.http.wait_job(job_id, timeout=30.0)
            # x 到位校验 (2026-08-10 task4 同款补丁, 2026-08-12 移植到 task1):
            # composite_run 并发时 move_x_position 假收敛 — 大臂旋转 + 串口争用下 X 只走一半
            # (PLACE x=-235 → S x=-70 实测停在 -152, SDK 却报 x:true). 用 arm_feed
            # 编码器 (唯一可信源) 校验, 未到位单轴补走 (arm=None 只动 X, 大臂已停不抢串口).
            # 读数失败 (feed 未启 / 测试 mock 返回非数值) → 直接跳过补走, 不阻塞任务.
            for _ in range(3):
                try:
                    actual_x = arm_client._read_x_mm_realtime()
                    if actual_x is None or abs(float(actual_x) - S_POSE_X_MM) < 15.0:
                        break
                except (TypeError, ValueError):
                    break
                logger.warning("  切 S: x 未到位 (实际=%.0fmm 目标=%.0fmm), 单轴补走",
                               actual_x, S_POSE_X_MM)
                arm_client.composite_run(
                    x_mm=S_POSE_X_MM, speed=COMPOSITE_SPEED_DEFAULT, timeout=15.0)

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
                max_velocities=[CHASSIS_MOVE_MAX_VEL_MPS, CHASSIS_MOVE_MAX_VEL_MPS, math.pi / 3.0],
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
                max_velocities=[CHASSIS_MOVE_MAX_VEL_MPS, CHASSIS_MOVE_MAX_VEL_MPS, math.pi / 3.0],
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