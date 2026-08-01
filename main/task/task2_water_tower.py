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
  4. move_y 到吸取高度 -75
  5. grasp + move_y 到运输高度 -105
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

# 梯度放置: 同一水塔内第 1/2/3 块的投放 Y 深度依次加深 (避免堆叠碰撞)
DELIVER_Y_BY_INDEX = [-60.0, -75.0, -95.0]

# 水塔等级标签 → 所需方块数
WATER_TOWER_LABELS = {"water_l1", "water_l2", "water_l3"}


# ── 辅助函数 ─────────────────────────────────────────────────

def _chassis_move_for(
    arm_client: ArmClient,
    dx_m: float,
    timeout: float,
) -> dict:
    """底盘纵向 move_for 阻塞调用 (sync=True 等结果)."""
    return arm_client._call_car("move_for", dx_m, timeout=timeout, sync=True)


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


def _detect_tower_count(client: RuntimeApiClient) -> int:
    """cam2 识别水塔等级标签, 返回需要的方块数.

    仅轮询 1 秒, 超时/失败 → 默认返回 1 块, 不崩溃.
    返回值映射: water_l1 → 1 块, water_l2 → 2 块, water_l3 → 3 块.
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
    logger.warning("cam2 识别超时, 默认取 1 块")
    return 1


# ── 核心动作子流程 ────────────────────────────────────────────────

def _pick_cube(
    arm_client: ArmClient,
    runner: ArmRunner,
    cfg: Dict[str, Any],
    cube_x_mm: float,
) -> None:
    """抓取单个水方块 (不含投放).

    执行顺序 (composite_run 内置并发 + SafetyMixin 自动 y 保护区校验):
      1. composite_run: 大臂 → +95° + X → cube_x_mm + Y → -120 (并发)
      2. 手爪 → 0°
      3. 轮询 get_state 确认大臂物理到位 (Rule C 业务层校验)
      4. move_y 到吸取高度
      5. grasp + 等待真空稳定 + move_y 抬升到运输高度
    """
    pick = cfg["pick_pose"]

    # 1) 复合动作: arm 转 + X 伸出 + Y 抬升并发
    runner.client.composite_run(
        arm=float(pick["arm_angle_deg"]),
        x_mm=float(cube_x_mm),
        y_mm=float(pick["y_descend_mm"]),
    )

    # 2) 手爪转 0°
    arm_client.set_hand_angle(float(pick["hand_angle_deg"]), speed=80, timeout=10.0)

    # 3) 业务层校验大臂物理到位
    _wait_arm_angle_reached(arm_client, pick["arm_angle_deg"])

    # 4) 下降到吸取高度
    runner.move_y(float(pick["y_descend_mm"]))

    # 5) 开真空 + 等待稳定 + 抬升
    runner.grasp(on=True)
    time.sleep(cfg["vacuum_settle_s"])
    runner.move_y(float(pick["y_lift_mm"]))


def _deliver_cube(
    arm_client: ArmClient,
    runner: ArmRunner,
    cfg: Dict[str, Any],
    cube_index: int = 0,
) -> None:
    """将吸住的水方块投放到水塔内.

    执行顺序 (梯度投放避免堆叠):
      1. composite_run: 大臂 → -95° + 手爪 → -90° + X → -120 + Y → carry (并发)
      2. move_y 梯度下降到该方块对应深度
      3. grasp off 释放方块
    """
    carry = cfg["carry_pose"]
    deliver_y = DELIVER_Y_BY_INDEX[min(cube_index, len(DELIVER_Y_BY_INDEX) - 1)]

    # 1) 复合动作: 大臂转 + 手爪转 + X 伸出并发 (SafetyMixin 自动校验 y 保护区)
    runner.client.composite_run(
        arm=float(carry["arm_angle_deg"]),
        hand=float(carry["hand_angle_deg"]),
        x_mm=float(carry["x_mm"]),
        y_mm=float(deliver_y),
    )

    # 2) 关真空释放方块
    runner.grasp(on=False)


# ── 主入口 ────────────────────────────────────────────────────────

def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """任务二主入口: 水塔取水 (2 座水塔 × N 块水方块投放).

    主流程:
      初始化: X 收至 -220 → 大臂转 -95° + 手爪 -45° (检测姿态)
      对每座水塔循环:
        1) Y 下降到 -30 → cam2 识别水塔等级 (需几块)
        2) 按块循环: 底盘到方块组 → 抓块 → 底盘回水塔 → 梯度投放
        3) 结束后底盘前进 60cm 到下一座水塔

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
    timeout = cfg["chassis_move_timeout_s"]
    group_forward_m = cfg["group_forward_m"]
    x_target_mm = -220.0

    try:
        # ===== 初始化步骤 1: X 收至 -220mm =====
        logger.info("初始化: X 收至 %.0f mm", x_target_mm)
        runner.move_x(x_target_mm)

        # ===== 初始化步骤 2: 大臂 + 手爪到检测姿态 =====
        logger.info("初始化: 大臂=%s°, 手爪=-45°", detection["arm_angle_deg"])
        runner.set_arm_angle(float(detection["arm_angle_deg"]), speed=80)
        time.sleep(2.0)
        arm_client.set_hand_angle(-45.0, speed=80, timeout=10.0)
        time.sleep(1.0)

        for tower_idx, tower_label in enumerate(cfg["source_position_order"]):
            logger.info("=== 处理水塔 %s (第 %d 座) ===", tower_label, tower_idx + 1)

            # 非第一座水塔: 底盘前进到下一座 (间距 60cm)
            if tower_idx > 0:
                tower_spacing_m = cfg.get("tower_spacing_m", 0.60)
                logger.info("底盘: 从第 %d 座到水塔 %s (前进 %.2f m)",
                            tower_idx, tower_label, tower_spacing_m)
                runner.move_x(x_target_mm)
                _chassis_move_for(arm_client, tower_spacing_m, timeout=timeout)
                runner.move_x(x_target_mm)
                logger.info("恢复手爪 -45° 检测姿态 (水塔 %s)", tower_label)
                arm_client.set_hand_angle(-45.0, speed=80, timeout=10.0)
                time.sleep(0.5)

            # 下降 Y 到检测高度
            logger.info("Y 下降到 -30mm 执行检测")
            try:
                runner.move_y(-30)
                time.sleep(0.3)
            except Exception:
                logger.warning("Y 下降失败, 跳过水塔 %s", tower_label)
                continue

            # 识别需几块
            needed = _detect_tower_count(client)
            logger.info("水塔 %s 需投放 %d 块水方块", tower_label, needed)

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