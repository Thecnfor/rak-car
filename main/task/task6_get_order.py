#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""任务六: 智能接单 (推杆扫订单 + OCR 读单 + 抓取对应蔬菜).

推杆动作完整序列 (sweep 期间 Y 保持 = 0 触底):
  1. X 收至 -200 mm (PID 闭环定位)
  2. 大臂 arm → -95° + 等待 2s 稳定
  3. 手爪 hand → -90° + 等待 1s 稳定
  4. Y 下降 → 0 mm (触底限位, 推杆前保持此高度)
  5. hand → -55° (推杆准备姿态)
  6. X 推杆扫动: -200 → -120 @ 100 mm/s (PID 闭环带动推牌杆)
  7. X 回退 → -150 mm (调整位置)
  8. 先抬 Y → -80 (防碰撞) 再转大臂 arm → -85° (读单姿态)
  9. hand → -55° (确认)
  10. Y → -100 mm (安全抬升)

=== 分阶段划分 ===
  阶段 1-2: 推杆动作 + X/Y/arm 复位 (读单前姿态准备)
  阶段 3  : LLM × 2 轮读单 (调用 test_order_read.run)
  阶段 4  : 抓取对应蔬菜 → LLM 视觉识别 + 真空抓取 × 2 棵
  阶段 5  : 回到运送待命姿态 (准备给任务七, 目前 stub)

动作辅助函数来源: 统一走 main.arm.ArmRunner (含 y 保护区 / 角度硬限 /
丢步核对 / composite_* 并发执行). 自定义的安全门不再存在 —— SafetyMixin
在每个动作入口处统一校验.

架构说明 (2026-08 重构):
  本任务使用 main.arm.ArmRunner + CompositeMixin 编排动作,
  不再依赖 main/task/_helpers.py (该文件已删除, 详见 main/task/README.md).
"""
from __future__ import annotations

import base64
import logging
import requests
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(_PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(_PROJECT_ROOT))

import yaml

from main.api_client import RuntimeApiClient
from main.arm import ArmClient, ArmRunner
from main.misc.test_order_read import run as order_read_run

# task6 配置独立保留在 test/task6_config.yml (避免侵入 task_config.yml 其它段)
_TASK6_CONFIG = Path(_PROJECT_ROOT) / "test" / "task6_config.yml"

logger = logging.getLogger("task.get_order")


# ── 配置加载 ──────────────────────────────────────────────────

def _load_task6_config() -> Dict[str, Any]:
    """加载任务六独立配置 (只读 test/task6_config.yml, 不侵入 task_config.yml)."""
    if not _TASK6_CONFIG.exists():
        raise FileNotFoundError(f"任务六配置文件不存在: {_TASK6_CONFIG}")
    with _TASK6_CONFIG.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    cfg = data.get("get_order") if isinstance(data, dict) else {}
    if not cfg or cfg.get("placeholder"):
        raise KeyError("task6_config.yml 中未找到 get_order 配置段或为占位")
    return cfg


# ── 蔬菜视觉抓取 (track_velocity_pick 视觉对齐) ────────────────────────────
# 2026-08-07: 抓取从"固定坐标盲抓"换成"目标检测 + 视觉对齐".
# track_velocity_pick 读 cam2 task_feed 的 YOLO 检测 (h_* label) → 选最近目标 →
# arm 控 cx + X 十字控 cy 对齐到 setpoint → y 降到 grasp_y → 真空吸 → 抬回.

# 订单蔬菜名 → YOLO 检测 label (与 main/arm/labels.py 的 h_* 对应)
PICK_VEGGIE_LABEL_MAP: Dict[str, str] = {
    "番茄": "h_fan_qie", "土豆": "h_tu_dou", "蘑菇": "h_mo_gu",
    "青椒": "h_qing_jiao", "油菜": "h_you_cai", "芹菜": "h_qin_cai",
    "豆角": "h_dou_jiao", "西兰花": "h_xi_lan_hua", "金针菇": "h_jin_zhen_gu",
}

# 视觉伺服起始姿态 (2026-08-08: -190/-180/-85/-25 → -150/-190/-90/-10; Y 改 -180)
PICK_START_X_MM = -160.0   # 2026-08-08: -150 → -160 (检测姿态 X)
PICK_START_Y_MM = -180.0   # 2026-08-08: -190 → -180 (与精对齐 setpoint 测量姿态一致)
PICK_START_ARM_DEG = -90.0
PICK_START_HAND_DEG = -10.0

# 精对齐 setpoint (2026-08-08 实测): 把目标框拉到的画面点 = 固定
# (0.168, -0.525) (y=-180, 末端=10), 八个位置 (左右列 × 4 行) 都一样.
# 每行 cal 的 (cx,cy) 只用于**匹配行号**, 不再当 setpoint.
PICK_SETPOINT_CXCY: Tuple[float, float] = (0.168, -0.525)

# 下降吸附高度 / 抓取偏移
PICK_GRASP_Y_MM = -25.0    # 2026-08-08: -20→-50→-30→-25 (目标在中心则末端朝下 + 降-25 吸)

# 放置位 (2026-08-08): 第1个订单蔬菜 → 左, 第2个 → 右; y 2026-08-08: -60 → -70
PICK_PLACE_1: Dict[str, float] = dict(x_mm=0.0, y_mm=-70.0, arm=86.0, hand=10.0)
PICK_PLACE_2: Dict[str, float] = dict(x_mm=-58.0, y_mm=-70.0, arm=86.0, hand=10.0)

# 抓菜前任务点前进 (move_along_lane, 2026-08-08 全改用任务点前进)
PICK_APPROACH_M = 0.14     # 2026-08-08: 0.22→0.13→0.12→0.14 (定位到右侧一列)
PICK_APPROACH_VX = 0.20

# 一列一列检测 (2026-08-08): 先右列后左列, 不再两排一起检测
PICK_COLUMN_ORDER = ["right", "left"]
PICK_COLUMN_MOVE_M = 0.10  # 切到左列的底盘移动 (2026-08-08: 0.2→0.08→0.05→0.10)
PICK_COLUMN_MOVE_VX = 0.20

# 大臂基础增益 (精对齐 = 0.5×, 见 PICK_FINE_GAIN_*)
PICK_GAIN_ARM = 8.0      # 2026-08-08: 4.0→6.0→8.0
PICK_DEADZONE = 0.02
PICK_MAX_VEL = 0.10      # 2026-08-08: 0.15 → 0.10 (X 更慢, 防冲)
PICK_HOLD_Y = True            # 2026-08-07: 只三轴联动, 不动 y 十字
PICK_DESCEND_HAND_DEG = 10.0  # 2026-08-08: 0→10 (精对齐/抓取末端都保持 10°)
PICK_ARM_MIN = -150.0         # 2026-08-08: 大臂下限放宽 (默认 -90 挡住偏左目标)
PICK_ARM_MAX = 150.0          # 2026-08-08: 大臂上限也放宽到 +150
PICK_X_ERROR_SOURCE = "dy"    # 2026-08-08: X 跟随垂直误差 (X 右移→目标框下移, 即 X 影响 cy)

# 精对齐 (2026-08-08, 粗对齐已删): X 到匹配行后, 末端先显式转到 10° (精对齐姿势),
# 再精对齐 — 末端保持 10° (gain_hand=0, hand_start=10), 只动 X + 大臂,
# setpoint 固定 (0.168,-0.525) (八个位置一致)
PICK_FINE_HAND_DEG = 10.0
# ⚠️ 必须把 hand_min/hand_max 传进 find_target_4dof: 其默认 hand_max=0 会把
#    hand_start=10 的 clamp 压回 0 (末端变 0° 而非 10°) — 2026-08-08 实机发现
PICK_FINE_HAND_MIN = -90.0
PICK_FINE_HAND_MAX = 30.0
PICK_FINE_GAIN_X = 0.05
PICK_FINE_GAIN_ARM = PICK_GAIN_ARM * 0.25   # 2026-08-08: 8.0*0.5=4.0 → 8.0*0.25=2.0 (大臂再慢, 原 0.5 倍)
PICK_FINE_TIMEOUT_S = 6.0   # 2026-08-08: 4.0 → 6.0

# 蔬菜布局 (2026-08-08 用户实测标定): 右列 4 行 (从上到下 1-4).
# 每行: {x_mm, cx_ref, cy_ref} — 检测姿态 (大臂-90 Y-180) 下该行蔬菜框的 (cx,cy),
# 只用来**匹配行号** (最近邻) → X 移到该行 x_mm → 精对齐.
# 对齐 setpoint 是固定 PICK_SETPOINT_CXCY (0.168,-0.525), 不用每行 (cx,cy).
PICK_RIGHT_CAL: Dict[int, Dict[str, float]] = {
    1: dict(x_mm=-48.0,  cx=0.148, cy=-0.317),
    2: dict(x_mm=-95.0,  cx=0.119, cy=-0.287),
    3: dict(x_mm=-146.0, cx=0.113, cy=-0.302),
    4: dict(x_mm=-190.0, cx=0.158, cy=-0.252),
}
PICK_LEFT_CAL: Dict[int, Dict[str, float]] = {
    1: dict(x_mm=-48.0,  cx=0.148, cy=-0.317),   # 番茄 (最上)
    2: dict(x_mm=-95.0,  cx=0.119, cy=-0.287),   # 豆角
    3: dict(x_mm=-146.0, cx=0.113, cy=-0.302),   # 油菜
    4: dict(x_mm=-190.0, cx=0.158, cy=-0.252),   # 金针菇 (最下)
}
# 2026-08-08: 左列 X 与右列同 (-48/-95/-146/-190 从上到下, 用户确认);
#   左列 (cx,cy) 实测与右列相同 — 检测姿态下蔬菜框落点与货架列无关 (左右列靠底盘横移区分);
#   行区分主要靠 cy (垂直), cx 每次停位略有漂移.
# 列检测共同姿态 (2026-08-08)
PICK_COL_ARM_DEG = -90.0
PICK_COL_HAND_DEG = -18.0  # 2026-08-08: 0 → -18 (列检测姿态末端)
PICK_COL_Y_MM = -180.0   # 2026-08-08: -190 → -180 (精对齐 setpoint 测量姿态)

# 列检测静止时长 (2026-08-08): 到检测点停稳后静止检测秒数
PICK_DETECT_SECONDS_RIGHT = 1.0   # 右列停 1s
PICK_DETECT_SECONDS_LEFT = 2.0    # 左列停 2s (未标定, 多看几帧)

# 目标检测不到时 X 左右扫 (2026-08-08)
PICK_SCAN_X_MM = 20.0      # 2026-08-08: 10 → 20

# 列聚类 (2026-08-08): 同列检测框 cx 很近, 列间 cx 差距大 → 按间距聚类分列.
# 不再用 cx 符号单框阈值 (车未停稳时右列框会抖过 0 被判到左列).
COLUMN_GAP_CX = 0.25       # 同列相邻框 cx 最大间距; 超出则另起一列


# ── Y=0 触底时的动作 (绕开业务层 y 保护区) ────────────────────────
#
# ⚠️ main/arm/api/safety.py:76 的 y 保护区 fail-closed: y > -30mm 时
#    set_hand_angle / set_arm_angle / move_x 一律 raise ValueError,
#    只豁免 hand=-90 / arm=0 / arm=+90 三个 init 姿态。
#
#    但任务六的推杆动作**必须在 Y=0 触底时**摆手爪 + 扫 X (推牌杆要贴地
#    才能拨动订单机推杆, 见 test/task6_config.yml push_bar_pose.y_mm=0),
#    走 wrapper 必被拒。
#
#    因此这两个动作走 `_call_arm` 底层直调, 跳过 Python 层校验, 合法性
#    交由车端判断 —— 与 task5/low_tower.py:118-126 (手爪 0° DOWN) 和
#    task5/get_blue.py:172 同款处理。
#
#    ⚠️ 代价: 绕过安全门后, 若姿态没摆对, 手爪在触底高度横扫会撞到车体。
#       首次上场前务必先低速手动确认 arm=-95° 时手爪的扫掠半径。

def _set_hand_angle_at_bottom(
    arm_client: ArmClient,
    angle: float,
    speed: int = 80,
    timeout: float = 10.0,
) -> dict:
    """Y 触底时设置手爪角度 (底层直调, 绕开 y 保护区)."""
    return arm_client._call_arm(
        "set_hand_angle", timeout=timeout, sync=True,
        angle=float(angle), speed=speed,
    )


def _move_x_at_bottom(
    arm_client: ArmClient,
    x_mm: float,
    v_max_mms: float = 40.0,
    out_time: float = 15.0,
    timeout: float = 30.0,
) -> dict:
    """Y 触底时移动 X (底层直调, 绕开 y 保护区).

    注意: 绕开 wrapper 也就绕开了 `move_x` 的丢步核对 (_check_step_loss),
    需要校验实际到位时请自行读 realtime x_mm 对比。
    """
    return arm_client._call_arm(
        "move_x_position", timeout=timeout, sync=True,
        target=float(x_mm) / 1000.0, out_time=out_time,
        v_max_mms=float(v_max_mms),
    )


# ── 单棵蔬菜视觉抓取 + 投放 (track_velocity_pick_4dof) ───────────

def _target_at_center(
    runner: ArmRunner,
    label: str,
    tol: float,
) -> bool:
    """读 task_feed 检测, 检查目标 label 是否已在画面中心附近 (|cx|,|cy| < tol).

    2026-08-07: 目标已在中心则跳过视觉对齐 (省时间), 直接降+吸.
    """
    try:
        state = (runner.client.http.get_task_state() or {}).get("task_state") or {}
        dets = state.get("detections") or []
        for d in dets:
            if d.get("label") != label:
                continue
            bb = d.get("bbox_norm") or {}
            cx = float(bb.get("x_center", 1.0))
            cy = float(bb.get("y_center", 1.0))
            if abs(cx) < tol and abs(cy) < tol:
                return True
    except Exception:
        pass
    return False


def _descend_and_grasp(runner: ArmRunner) -> None:
    """末端保持 10° (hand=10) + y→grasp_y + 真空吸 (2026-08-08, 不再 X 向0偏移)."""
    runner.client.composite_run(
        y_mm=PICK_GRASP_Y_MM, hand=PICK_DESCEND_HAND_DEG, speed=100,
    )
    runner.grasp(on=True)
    runner.move_y(PICK_START_Y_MM)  # 抬回起始高度


def _det_cx(d: dict) -> float:
    bb = (d or {}).get("bbox_norm") or {}
    try:
        return float(bb.get("x_center", 0.0))
    except Exception:
        return 0.0


def _det_cy(d: dict) -> float:
    bb = (d or {}).get("bbox_norm") or {}
    try:
        return float(bb.get("y_center", 0.0))
    except Exception:
        return 0.0


def _cluster_by_cx(dets: List[dict]) -> List[List[dict]]:
    """按 bbox cx 把检测聚成列 (同列框很近, 列间 cx 差距大). 返回按质心 cx 升序的簇."""
    s = sorted(dets, key=_det_cx)
    clusters: List[List[dict]] = []
    for d in s:
        if clusters and _det_cx(d) - _det_cx(clusters[-1][-1]) < COLUMN_GAP_CX:
            clusters[-1].append(d)
        else:
            clusters.append([d])
    return clusters


def _wait_chassis_stopped(runner: ArmRunner, timeout: float = 3.0,
                          eps_m: float = 0.002, dwell_s: float = 0.2) -> bool:
    """等底盘停稳再检测 (odom 连续 0.15s 位移 ≤ eps 判定停稳).

    2026-08-08: move_along_lane 按里程到达即返回, 车未完全停稳时 task_feed 的
    检测框在抖 (右列框 cx 会抖过 0 被判到左列). 停稳后再等 dwell 刷新一帧检测.
    """
    try:
        from main.chassis import get_odometry
        t0 = time.time()
        prev = get_odometry()[0]
        while time.time() - t0 < timeout:
            time.sleep(0.15)
            cur = get_odometry()[0]
            if abs(cur - prev) <= eps_m:
                time.sleep(dwell_s)
                logger.info("  底盘停稳 (0.15s 位移≤%.0fmm), 开始检测", eps_m * 1000)
                return True
            prev = cur
        logger.warning("  底盘停稳超时 (%.0fs), 直接检测", timeout)
        return False
    except Exception as exc:
        logger.info("  停稳检测跳过 (读里程计失败: %s)", exc)
        return False


def _dump_detections(runner: ArmRunner, label: str) -> None:
    """诊断: 打印当前 task_feed 里该 label 的所有检测 (cx/cy/score).

    判断抓菜失败是"识别问题" (检测不到目标/无该 label) 还是"对齐问题"
    (检测到但伺服没收敛). 2026-08-08 用户要求加日志定位.
    """
    try:
        state = (runner.client.http.get_task_state() or {}).get("task_state") or {}
        dets = state.get("detections") or []
        mine = [d for d in dets if d.get("label") == label]
        logger.info("    [诊断] label=%s 检测到 %d 个:", label, len(mine))
        for d in mine[:5]:
            bb = d.get("bbox_norm") or {}
            logger.info("      cx=%+.3f cy=%+.3f score=%.2f",
                        float(bb.get("x_center", 0.0)),
                        float(bb.get("y_center", 0.0)),
                        float(d.get("score", 0.0)))
        if not mine:
            logger.warning("    [诊断] 无 %s 检测 — 疑似识别问题 (目标不在视野/模型检不出)",
                           label)
    except Exception as exc:
        logger.info("    [诊断] 读检测失败: %s", exc)


def _has_detection(runner: ArmRunner, label: str) -> bool:
    """task_feed 里是否存在该 label 的检测."""
    try:
        state = (runner.client.http.get_task_state() or {}).get("task_state") or {}
        dets = state.get("detections") or []
        return any(d.get("label") == label for d in dets)
    except Exception:
        return False


def _scan_for_target(runner: ArmRunner, label: str) -> bool:
    """目标检测不到 → X 左右扫 ±10mm 再检测 (2026-08-08). 返回是否找到."""
    try:
        cur_x = float(runner.client.get_state().x_mm)
    except Exception as exc:
        logger.info("    [诊断] X 扫描: 读 x 失败 (%s), 跳过扫描", exc)
        return _has_detection(runner, label)
    for dx in [PICK_SCAN_X_MM, -PICK_SCAN_X_MM]:
        if _has_detection(runner, label):
            return True
        runner.client.composite_run(x_mm=cur_x + dx, speed=100)
        time.sleep(0.4)  # 等 X 到位 + 检测刷新
        if _has_detection(runner, label):
            logger.info("  [%s] X 扫描找到目标 (offset=%.0fmm)", label, dx)
            return True
    logger.info("  [%s] X 扫描 ±%.0fmm 后仍无目标", label, PICK_SCAN_X_MM)
    return _has_detection(runner, label)


def _classify_frame(
    runner: ArmRunner,
    label: str,
    cal: Dict[int, Dict[str, float]],
    column: str,
) -> Dict[str, Any]:
    """读一帧 task_feed: 按 cx 聚列 → 给所有蔬菜标 列+行 → 返回目标判定.

    2026-08-08: 列判定用聚类 (同列框很近, 列间 cx 差距大), 不再用 cx 符号
    单框阈值 (车未停稳时右列框会抖到负). 簇列身份 = 最右簇=右 / 最左簇=左.

    Returns:
        dict: {
          "target": Optional[Tuple[int, float]],  # 目标 (行, x_mm) 若在当前列
          "col": Optional[str],                   # 目标所在列 (left/right), None=未检出
          "rows": List[dict],                     # 全帧蔬菜: {label, col, row, cx, cy}
        }
    """
    try:
        state = (runner.client.http.get_task_state() or {}).get("task_state") or {}
        dets = state.get("detections") or []
    except Exception:
        return {"target": None, "col": None, "rows": []}
    veg = [d for d in dets if d.get("label") in PICK_VEGGIE_LABEL_MAP.values()]
    clusters = _cluster_by_cx(veg)
    rows: List[dict] = []
    target_col = None
    target_match = None
    for i, cl in enumerate(clusters):
        if len(clusters) == 1:
            side = column
        elif i == len(clusters) - 1:
            side = "right"
        elif i == 0:
            side = "left"
        else:
            side = "?"
        for d in cl:
            lab = d.get("label")
            cx, cy = _det_cx(d), _det_cy(d)
            row = None
            if side == column and cal:
                best_r, best_d2 = None, None
                for r, ref in cal.items():
                    d2 = (cx - ref["cx"]) ** 2 + (cy - ref["cy"]) ** 2
                    if best_d2 is None or d2 < best_d2:
                        best_d2, best_r = d2, r
                row = best_r
            if lab == label:
                target_col = side
                if row is not None:
                    target_match = (row, cal[row]["x_mm"])
            rows.append({"label": lab, "col": side, "row": row, "cx": cx, "cy": cy})
    return {"target": target_match, "col": target_col, "rows": rows}


def _detect_column_stationary(
    runner: ArmRunner,
    label: str,
    cal: Dict[int, Dict[str, float]],
    column: str,
    seconds: float = 1.0,
    interval_s: float = 0.15,
) -> Optional[Tuple[int, float]]:
    """到检测点停稳后静止 ~1s 多帧检测 (不前进), 严格日志输出左右列+行顺序.

    2026-08-08: 用户要求到达检测点先停 1s, 这一秒内多次识别 (不前进), 日志里
    严格输出 左右列 和 顺序(第几行). 行结果多数票取 (防单帧抖动). 返回 (行, x_mm).
    """
    _wait_chassis_stopped(runner)
    votes: Dict[int, int] = {}
    n = 0
    t0 = time.time()
    while time.time() - t0 < seconds:
        n += 1
        det = _classify_frame(runner, label, cal, column)
        # 严格日志: 每帧全部蔬菜的 列 + 行顺序 + (cx,cy) (方便读标定值)
        parts = []
        for r in det["rows"]:
            col_txt = {"left": "左", "right": "右"}.get(r["col"], str(r["col"] or "?"))
            row_txt = f"#{r['row']}" if r["row"] is not None else "-"
            parts.append(f"{r['label']}={col_txt}列{row_txt}({r['cx']:+.2f},{r['cy']:+.2f})")
        logger.info("  [检测#%d] %s", n, "  ".join(parts))
        if det["col"] == column and det["target"] is not None:
            r, _x = det["target"]
            votes[r] = votes.get(r, 0) + 1
        time.sleep(interval_s)
    if votes:
        best = max(votes, key=votes.get)
        logger.info("  [检测汇总] %s → %s列 第%d行 %d/%d 票 (X=%.0fmm)",
                    label, column, best, votes[best], n, cal[best]["x_mm"])
        return (best, cal[best]["x_mm"])
    reason = "未标定" if not cal else "未检出/列不匹配"
    logger.warning("  [检测汇总] %s → %s列 %s (%d 帧)", label, column, reason, n)
    return None


def _align_4dof_phase(
    runner: ArmRunner,
    label: str,
    *,
    gain_x: float,
    gain_arm: float,
    gain_hand: float,
    max_vel: float,
    timeout: float,
    arm_start: float,
    hand_start: float,
    hand_min: float = -90.0,
    hand_max: float = 0.0,
    setpoint_cxcy: Optional[Tuple[float, float]] = None,
) -> Tuple[bool, int]:
    """跑一次 find_target_4dof (velocity 对齐). 返回 (settled, hits).

    gain_x>0 → X 十字也动; gain_hand>0 → 末端也动 (task6 只用精对齐, gain_hand=0).
    hand_min/hand_max (2026-08-08): 必须显式传 — find_target_4dof 默认 hand_max=0,
    会把 hand_start=10 的 clamp 压回 0 (末端 0° 而非 10°).
    setpoint_cxcy (2026-08-08): 目标框拉到的画面点 = 固定
    PICK_SETPOINT_CXCY=(0.168,-0.525) (八个位置一致), 不再默认画面中心.
    """
    sx, sy = setpoint_cxcy if setpoint_cxcy is not None else PICK_SETPOINT_CXCY
    try:
        runner._set_arm_feed(stop=True)
        result = runner.client._make_vision_with_move().find_target_4dof(
            label, timeout=timeout, hz=20.0,
            gain_x=gain_x, gain_y=0.05, gain_arm=gain_arm, gain_hand=gain_hand,
            deadzone=PICK_DEADZONE, max_vel=max_vel,
            arm_start=arm_start, hand_start=hand_start,
            arm_min=PICK_ARM_MIN, arm_max=PICK_ARM_MAX,
            hand_min=hand_min, hand_max=hand_max,
            hold_y=PICK_HOLD_Y,
            x_error_source=PICK_X_ERROR_SOURCE,
            setpoint_x_norm=sx,
            setpoint_y_norm=sy,
        )
    finally:
        runner._set_arm_feed(stop=False)

    def _conv(t) -> bool:
        return not t.miss and abs(t.dx) < PICK_DEADZONE and abs(t.dy) < PICK_DEADZONE
    settled = False
    tail = list(result.trace[-30:])
    for start in range(len(tail) - 3 + 1):
        if all(_conv(t) for t in tail[start:start + 3]):
            settled = True
            break
    return settled, result.hits


def _pick_one_veggie_visual(
    runner: ArmRunner,
    goods_name: str,
    label: str,
    place_pose: Dict[str, float],
    column: Optional[str] = None,
) -> bool:
    """单棵蔬菜: 列姿态 → 匹配行(cx,cy) → X到该行 → 末端转10° → 精对齐 → 降+吸 → 放.

    2026-08-08: 只精对齐 (粗对齐已删). X 到匹配行后, 末端先显式转到 10°, 精对齐
    只动 X+大臂 (末端保持 10°), setpoint 固定 (0.168,-0.525) 八个位置一致, timeout 6s;
    每行 (cx,cy) 只用于匹配行号, 一列一列, 左右列各自标定.

    Returns:
        True=已抓取投放; False=检测不到/匹配不到行 (跳过).
    """
    # 0) 切到列检测姿态 (X-160 末端-18 Y-180, 2026-08-08)
    runner.client.composite_run(
        x_mm=PICK_START_X_MM, y_mm=PICK_COL_Y_MM,
        arm=PICK_COL_ARM_DEG, hand=PICK_COL_HAND_DEG, speed=100,
    )
    logger.info("  切到列检测姿态: X→%.0f Y→%.0f arm→%.0f hand→%.0f",
                PICK_START_X_MM, PICK_COL_Y_MM, PICK_COL_ARM_DEG, PICK_COL_HAND_DEG)

    # 1) 到检测点停稳后静止检测 (不前进), 严格日志输出左右列+行顺序 (2026-08-08)
    #    左右两列都停稳检测 (左列未标定也检测并输出日志; 左列停 2s 右列 1s)
    cal = PICK_RIGHT_CAL if column == "right" else PICK_LEFT_CAL
    detect_s = PICK_DETECT_SECONDS_LEFT if column == "left" else PICK_DETECT_SECONDS_RIGHT
    match = _detect_column_stationary(runner, label, cal, column, seconds=detect_s)
    if not cal:
        logger.info("  [%s] %s列未标定, 跳过 (已停 %.0fs 检测)", goods_name, column, detect_s)
        return False
    if match is None:
        logger.info("  [%s] 未检出/列不匹配 (%s列), 跳过", goods_name, column)
        return False
    row, x_target = match
    logger.info("  [%s] 匹配到 %s列 第%d行 → X=%.0f", goods_name, column, row, x_target)

    # 2) 先 X 移到匹配行的 X (精对齐前, 2026-08-08)
    runner.client.composite_run(x_mm=x_target, speed=100)
    logger.info("  [%s] X→%.0f (第%d行)", goods_name, x_target, row)

    # 3) 精对齐 (粗对齐已删 2026-08-08): X 到匹配行后, 末端先显式转到 10° (精对齐姿势),
    #    再精对齐 — 末端保持 10° (gain_hand=0, hand_start=10), 只动 X + 大臂,
    #    setpoint 固定 (0.168,-0.525) (八个位置一致)
    runner.client.composite_run(hand=PICK_FINE_HAND_DEG, speed=100)
    logger.info("  [%s] 精对齐前末端→%.0f°", goods_name, PICK_FINE_HAND_DEG)
    settled, hits = _align_4dof_phase(
        runner, label,
        gain_x=PICK_FINE_GAIN_X, gain_arm=PICK_FINE_GAIN_ARM,
        gain_hand=0.0,
        max_vel=PICK_MAX_VEL, timeout=PICK_FINE_TIMEOUT_S,
        arm_start=PICK_COL_ARM_DEG, hand_start=PICK_FINE_HAND_DEG,
        hand_min=PICK_FINE_HAND_MIN, hand_max=PICK_FINE_HAND_MAX,
        setpoint_cxcy=PICK_SETPOINT_CXCY,
    )
    logger.info("  [%s] 精对齐(X+大臂, 末端保持10): settled=%s hits=%d",
                goods_name, settled, hits)

    # 4) 末端朝下 y-30 吸
    _descend_and_grasp(runner)

    # 5) 放 (用户 2026-08-08 顺序修正): 抓完已抬到 -180, 先在抬升高度转 X/大臂/末端
    #    到放置位, 再降 y→place_y(-70), 关真空释放, 再抬 y→-180 (先抬升, 再动其它三轴, 防拖拽)
    runner.client.composite_run(
        x_mm=place_pose["x_mm"], arm=place_pose["arm"],
        hand=place_pose["hand"], speed=100,
    )
    runner.move_y(place_pose["y_mm"])
    runner.grasp(on=False)
    runner.move_y(PICK_START_Y_MM)   # 抬回 -180 (先抬升, 再动其它轴)
    logger.info("  [%s] 已投放 (X→%.0f Y→%.0f arm→%.0f hand→%.0f)",
                goods_name, place_pose["x_mm"], place_pose["y_mm"],
                place_pose["arm"], place_pose["hand"])
    return True


# ── 推杆姿态 + 扫牌 + 读单姿态 ──────────────────────────────────────

def _enter_read_pose(
    arm_client: ArmClient,
    runner: ArmRunner,
    cfg: Dict[str, Any],
) -> None:
    """从当前姿态 → 读单姿态 (X=-150, Y=-150, arm=-80, hand=-60), 4 轴并行.

    2026-08-07 顺序重排: 第一次读单在推杆**前** (先读后推). 任务启动时臂在
    init (y=-150, x=0, arm=+90, hand=-90), 4 轴复合一步切到读单姿态 (同推杆
    姿势的 4 轴并行范式, 无 y 保护区). 同时底盘直行 approach_m 到任务点.
    """
    repos = cfg["reposition_pose"]
    read_x = float(repos.get("final_x_mm", -200.0))
    read_y = float(repos.get("y_mm", -180.0))
    read_arm = float(repos.get("arm_angle_deg", -85.0))
    read_hand = float(repos.get("hand_angle_deg", -45.0))
    # 前进 0.3m + 4轴摆读单姿态 并发 (2026-08-07: 前进从推杆移到读单前)
    _approach_straight_parallel(
        runner, cfg,
        dict(x_mm=read_x, y_mm=read_y, arm=read_arm, hand=read_hand, speed=100),
    )
    logger.info("  读单姿态 (4轴并行): X→%.0f Y→%.0f arm→%.0f hand→%.0f",
                read_x, read_y, read_arm, read_hand)


# 摆读单姿态/摆姿势同时底盘直行到任务点 (move_along_lane, 与 4 轴复合并发)
STAGE1_APPROACH_M = 0.2     # 直行距离 (m) 2026-08-08: 0.5→0.3→0.2→0.15→0.2
STAGE1_APPROACH_VX = 0.20   # 直行速度 (m/s)

# 第二次读单位 (推牌完 Y 保持 0 触底直接去, 不专门抬升; 大臂/手爪转读单角; 2026-08-08)
SECOND_READ_X_MM = -200.0     # 第二次读单 X
SECOND_READ_ARM_DEG = -80.0   # 第二次读单大臂 (订单机大臂, 2026-08-08: -87→-80)
SECOND_READ_HAND_DEG = -75.0  # 第二次读单末端 (2026-08-08: -60→-75)

def _approach_straight_parallel(
    runner: ArmRunner,
    cfg: Dict[str, Any],
    composite_kwargs: Dict[str, Any],
) -> None:
    """底盘直行 approach_m + 臂 composite_run 并发 (move_along_lane 后台线程).

    2026-08-07: 前进从推杆移到读第一次订单前. 直行(0.3m@0.2=1.5s) 与 4轴复合
    (~2-3s) 并发; 复合完等直行到位 (车停稳再对齐/读单).
    """
    approach_m = float(cfg.get("approach_straight_m", STAGE1_APPROACH_M))
    approach_vx = float(cfg.get("approach_straight_vx", STAGE1_APPROACH_VX))
    ex = None
    drive_fut = None
    if approach_m > 1e-3:
        from concurrent.futures import ThreadPoolExecutor
        from main.chassis import move_along_lane
        ex = ThreadPoolExecutor(max_workers=1)
        drive_fut = ex.submit(move_along_lane, vx=approach_vx, distance_m=approach_m)
        logger.info("  底盘直行 %.2fm (vx=%.2f) 与摆姿态并发", approach_m, approach_vx)
    try:
        runner.client.composite_run(**composite_kwargs)
        if drive_fut is not None:
            wait_t = abs(approach_m) / max(abs(approach_vx), 0.05) * 2.0 + 3.0
            drive_fut.result(timeout=wait_t)
            logger.info("  底盘直行 %.2fm 完成", approach_m)
    finally:
        if ex is not None:
            ex.shutdown(wait=True)

def _enter_push_bar_pose(
    arm_client: ArmClient,
    runner: ArmRunner,
    cfg: Dict[str, Any],
) -> None:
    """推杆姿势 → 扫牌 → 直接到第二次读单位 (2026-08-08 重排).

    初始姿势 (任务启动时): y=-150, x=0, arm=+90, hand=-90.
      (0.15m 前进已移到读第一次订单前, 推杆这里不再直行)
      1. 进入姿势 (task1 4 轴并行联动, speed=100): X→-220, arm→-92, Y→0, hand→-55
      2. 推牌: X -220→-120 @ 150mm/s (Y=0 触底)
      3. 推牌完直接去第二次读单位: X→-200 + arm→-80 + hand→-75, **Y 保持 0 触底** (不抬升)

    用 composite_run 走 4 轴/2 轴并行 (该接口无 y 保护区, 用户 23:31 定案).
    """
    pose = cfg["push_bar_pose"]
    sweep_end = cfg.get("sweep_x_end_mm", -120.0)
    sweep_speed = cfg.get("sweep_speed_mms", 150.0)

    # 1) 进入推杆姿势: 4 轴并行 (SDK 内部真并发, 无 y 保护区)
    runner.client.composite_run(
        x_mm=float(pose["x_mm"]), arm=float(pose["arm_angle_deg"]),
        y_mm=float(pose["y_mm"]), hand=float(pose["hand_angle_deg"]),
        speed=100,
    )
    logger.info("  进入推杆姿势: X→%.0f arm→%.0f Y→%.0f hand→%.0f (4 轴并行)",
                pose["x_mm"], pose["arm_angle_deg"], pose["y_mm"], pose["hand_angle_deg"])
    # 诊断: 读实际大臂角度 (确认是否真到 -95, 2026-08-08)
    try:
        logger.info("    [诊断] 推杆姿势实际大臂 = %s°",
                    runner.client.get_state().arm_angle)
    except Exception as exc:
        logger.info("    [诊断] 读大臂角度失败: %s", exc)

    # 2) 推牌: X -220 → -120 @ 150mm/s (Y=0 触底, 走底层直调)
    _move_x_at_bottom(arm_client, float(sweep_end), v_max_mms=float(sweep_speed))
    logger.info("  推牌 X→%.0f @ %.0fmm/s 完成", sweep_end, sweep_speed)

    # 3) 推牌完直接去第二次读单位 (Y 保持 0 触底不抬升; 大臂/手爪转读单角; 2026-08-08)
    runner.client.composite_run(
        x_mm=SECOND_READ_X_MM, arm=SECOND_READ_ARM_DEG,
        hand=SECOND_READ_HAND_DEG, speed=100,
    )
    logger.info("  直接到第二次读单位: X→%.0f arm→%.0f hand→%.0f (Y=0 触底不抬升)",
                SECOND_READ_X_MM, SECOND_READ_ARM_DEG, SECOND_READ_HAND_DEG)
    logger.info("推杆 + 扫牌完成, 已到第二次读单位")


# ============================================================
# 阶段 3-5 存根 (目前 stub, 部分逻辑已在 run() 中内联实现)
# 阶段 1-2 (推杆姿态 + 扫牌) 由 _enter_push_bar_pose 统一处理
# ============================================================

def _detect_and_ocr(arm_client: ArmClient, runner: ArmRunner, cfg: Dict[str, Any]) -> dict:
    """阶段 3 存根: cam2 检测前方订单牌 + OCR 读取 + 解析. 当前未实现."""
    raise NotImplementedError("阶段 3 detect_and_ocr - 待实现")


# ── LLM-as-detector + track_chassis 控制律: 底盘对齐订单牌 ────────
#
# 2026-08-07: 订单牌/订单机没有 YOLO 训练数据, 检测 backend 检不出. 改用
# ERNIE Vision 当"检测器" (看帧报订单牌中心 cx/cy), 但**复用 main.chassis 的
# track_chassis 整套控制律** (kp/slew/deadband/hold_frames/丢帧/停稳), 只把
# 检测源从 task_feed 换成 LLM sense_fn (track_chassis 新增 sense_fn 参数).
#
# 限制: LLM 慢 (~0.5-1Hz) + 精度 ±10%, 所以 hold_frames/lost_frames 按帧调小.
# 轴符号沿用 track_chassis 水塔标定 (cx↔vx 前后, cy↔vy 横向); 实车反了改下面.

LLM_ALIGN_SIGN_VX: int = +1   # 2026-08-07: -1 → +1 (实车方向写反了)
LLM_ALIGN_SIGN_VY: int = +1
LLM_ALIGN_KP: float = 0.30
LLM_ALIGN_V_MAX: float = 0.10
LLM_ALIGN_DEADBAND: float = 0.05
LLM_ALIGN_HOLD_FRAMES: int = 2
LLM_ALIGN_MAX_LOST: int = 5
LLM_ALIGN_MAX_SECONDS: float = 8.0
LLM_ALIGN_VX_ONLY: bool = True   # 2026-08-07: 只移动底盘前后, 不移动左右

def _llm_align_card(
    client: RuntimeApiClient,
    target_cx: float = 0.5,
    target_cy: float = 0.5,
    max_seconds: float = LLM_ALIGN_MAX_SECONDS,
    kp: float = LLM_ALIGN_KP,
    v_max: float = LLM_ALIGN_V_MAX,
) -> Tuple[bool, List[Tuple[float, float]]]:
    """LLM-as-detector + track_chassis 控制律: 把订单牌拉到画面目标点.

    sense_fn 抓 cam2 帧 → ERNIE Vision 报 (card_cx, card_cy, found) → 组装
    TrackFrame 喂给 track_chassis (vx=sign_vx*kp*cx_err, vy=sign_vy*kp*cy_err,
    deadband/hold_frames 收敛, max_lost 丢帧停, 结束零速). 轴符号/增益实车标定.

    Args:
        client: RuntimeApiClient (track_chassis 内部自建 ChassisClient)
        target_cx, target_cy: 目标位置 (默认 0.5 = 画面中心)
        max_seconds: 超时秒数
        kp: 比例增益
        v_max: 最大速度 m/s

    Returns:
        (aligned: bool, samples: List[(cx, cy)]) — 是否对齐 + 所有采样
    """
    from main.chassis import track_chassis
    from main.chassis.loops.visual_track import TrackFrame
    from main.misc.test_order_read import (
        _call_llm, _load_cfg, _load_token, fetch_frame,
    )
    from main.settings import load_settings

    settings = load_settings()
    cfg = _load_cfg()
    token = _load_token(cfg)
    ernie = cfg.get("ernie", {})

    POS_PROMPT = (
        "你是订单牌定位程序。看这张图找到订单牌(白底蓝色字的卡片)。\n"
        "返回 STRICT JSON, 不要 Markdown 标记:\n"
        '{"card_cx": 0.5, "card_cy": 0.5, "found": true}\n'
        "- card_cx: 订单牌中心水平位置 (0=最左, 0.5=正中, 1=最右)\n"
        "- card_cy: 订单牌中心垂直位置 (0=最上, 0.5=正中, 1=最下)\n"
        "- found: 是否看到订单牌 (true/false)\n"
        "如果没看到订单牌, card_cx=card_cy=0.5, found=false"
    )

    session = requests.Session()
    samples: List[Tuple[float, float]] = []

    def _sense() -> TrackFrame:
        frame = fetch_frame(session, settings.streamer_url)
        if not frame:
            logger.info("    [诊断] LLM 对齐: 取帧失败 (丢帧)")
            return TrackFrame()
        img = base64.b64encode(frame).decode()
        d = _call_llm(token, img, POS_PROMPT, ernie)
        if "error" in d or not d.get("found"):
            logger.info("    [诊断] LLM 对齐: 没看到订单牌 / LLM 失败 → 丢目标帧")
            return TrackFrame()   # 没看到牌 / LLM 失败 = 丢目标帧
        cx = float(d.get("card_cx", 0.5))
        cy = float(d.get("card_cy", 0.5))
        samples.append((cx, cy))
        logger.info("    [诊断] LLM 对齐 #%d: card=(%.2f, %.2f) Δ=(%+.2f, %+.2f)",
                    len(samples), cx, cy, target_cx - cx, target_cy - cy)
        return TrackFrame(
            target_found=True, label="order_card",
            cx=cx, cy=cy,
            cx_err=target_cx - cx, cy_err=target_cy - cy,
            age_ms=None,
        )

    # 记录对齐前后里程计 x (算前后移动多少)
    from main.chassis import get_odometry
    try:
        x0 = get_odometry()[0]
    except Exception:
        x0 = float("nan")

    result = track_chassis(
        "order_card",
        setpoint_cxcy=(target_cx, target_cy),
        sense_fn=_sense,
        # 控制律 (LLM 慢, 帧计数按帧调小; 轴符号实车反了改 LLM_ALIGN_SIGN_*)
        # 2026-08-07: 只前后 (vx_only=True), 不动左右
        kp=kp, v_max=v_max,
        sign_vx=LLM_ALIGN_SIGN_VX, sign_vy=LLM_ALIGN_SIGN_VY,
        vx_only=LLM_ALIGN_VX_ONLY,
        deadband=LLM_ALIGN_DEADBAND, hold_frames=LLM_ALIGN_HOLD_FRAMES,
        max_lost_frames=LLM_ALIGN_MAX_LOST, recover_after_lost=True,
        hz=1.0, max_seconds=max_seconds,
    )
    try:
        x1 = get_odometry()[0]
        dx_m = x1 - x0
    except Exception:
        x1 = float("nan")
        dx_m = float("nan")
    logger.info(
        "  LLM-as-servo (track_chassis): %s reason=%s frames=%d elapsed=%.1fs "
        "识别%d次 前后%+.3fm (x %.3f→%.3f)",
        "OK" if result.arrived else "FAIL", result.reason,
        result.frames, result.elapsed_s, len(samples), dx_m, x0, x1,
    )
    return result.arrived, samples


def _pick_goods(arm_client: ArmClient, runner: ArmRunner, cfg: Dict[str, Any]) -> dict:
    """阶段 4 存根: 物理抓取 5cm 订单方块. 当前未实现 (run 中另有实现)."""
    raise NotImplementedError("阶段 4 pick_goods - 待实现")


def _lift_and_carry(arm_client: ArmClient, runner: ArmRunner, cfg: Dict[str, Any]) -> dict:
    """阶段 5 存根: 抬升 + 运输待命姿态 (为任务七做准备). 当前未实现."""
    raise NotImplementedError("阶段 5 lift_and_carry - 待实现")


# ============================================================
# run() 主入口
# ============================================================

def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """任务六主入口: 推杆扫牌 + LLM 双轮读单 + 蔬菜识别匹配 + 抓取投放.

    Args:
        client: 可选 RuntimeApiClient, None 时内部新建

    Returns:
        Dict: {
            "ok": bool,                          # 任务是否成功
            "completed": List[str],             # 已完成的子步骤列表
            "order_list": {"round1":[], "round2":[]},  # 两轮 LLM 读单结果
            "error": str                        # 失败时的错误信息 (仅 ok=False)
        }
    """
    cfg = _load_task6_config()

    if client is None:
        client = RuntimeApiClient()
    client.wait_until_ready(timeout=30.0)

    # 初始化机械臂客户端与执行器
    arm_client = ArmClient.connect()
    if not arm_client.ping():
        raise RuntimeError("机械臂 runtime 未在线, 请检查 arm_feed 守护进程")
    runner = ArmRunner(arm_client)

    completed: List[str] = []
    order_list: List[Dict[str, Any]] = []

    try:
        # ===== 阶段 1: 摆读单姿态 + 底盘对齐 + 第一次读单 (先读后推) =====
        logger.info("=== 阶段 1: 摆读单姿态 + 底盘对齐 + 第一次读单 (先读后推) ===")
        _enter_read_pose(arm_client, runner, cfg)
        completed.append("read_pose")

        # 底盘对齐订单牌 (LLM-as-detector + track_chassis 控制律)
        try:
            align_ok, align_samples = _llm_align_card(
                client, target_cx=0.5, target_cy=0.5,
                max_seconds=LLM_ALIGN_MAX_SECONDS, kp=LLM_ALIGN_KP, v_max=LLM_ALIGN_V_MAX,
            )
            logger.info(
                "  LLM-as-servo 结果: %s, samples=%d, last=(%.2f, %.2f)",
                "OK" if align_ok else "FAIL", len(align_samples),
                *(align_samples[-1] if align_samples else (0.5, 0.5)),
            )
        except Exception as exc:
            logger.warning("LLM-as-servo 异常 (不影响后续读单): %s", exc)
            align_ok, align_samples = False, []
        completed.append("llm_align")

        # 第一次读单 (对齐位)
        logger.info("=== 阶段 1c: LLM 读单 第一轮 (对齐位) ===")
        round1 = order_read_run()
        if round1.get("ok") and round1.get("orders"):
            logger.info("  [第一轮] 读取到 %d 条订单:", len(round1["orders"]))
            for o in round1["orders"]:
                logger.info("    客户: %s | 商品: %s", o["name"], o["goods"])
        else:
            logger.warning("  [第一轮] 读取失败: %s", round1.get("error", "未识别到订单"))
        completed.append("order_read_1")

        # ===== 阶段 2: 回推杆姿势 + 推牌 (approach 0.3m 并发) =====
        logger.info("=== 阶段 2: 回推杆姿势 + 推牌 ===")
        _enter_push_bar_pose(arm_client, runner, cfg)
        completed.extend(["push_bar_pose", "sweep", "reposition"])

        # ===== 阶段 3: 第二次读单 (推牌完已到第二次读单位, 手爪多角度重试) =====
        # 2026-08-08: _enter_push_bar_pose 推牌完直接到 (X→-200 hand→-70 Y=0 触底),
        # 这里不再重复调整姿态, 直接多角度重试读.
        logger.info("=== 阶段 3: LLM 读单 第二轮 (已在第二次读单位) ===")
        round2: Dict[str, Any] = {"ok": False, "orders": [], "error": "no attempts"}
        for attempt, hand_angle in enumerate([SECOND_READ_HAND_DEG, -55.0, -90.0]):
            logger.info("=== 阶段 3: 读单 第二轮 (手爪=%.0f°, 第 %d 次尝试) ===", hand_angle, attempt + 1)
            if attempt > 0:
                # Y=0 触底, 走 wrapper 必被 y 保护区拒 → 底层直调
                _set_hand_angle_at_bottom(arm_client, hand_angle)
                time.sleep(0.5)
                logger.info("  手爪 → %.0f° (重试)", hand_angle)
            round2 = order_read_run()
            if round2.get("ok") and round2.get("orders"):
                logger.info("  [第二轮] 读取到 %d 条订单 (第 %d 次尝试, hand=%.0f°):",
                            len(round2["orders"]), attempt + 1, hand_angle)
                for o in round2["orders"]:
                    logger.info("    客户: %s | 商品: %s", o["name"], o["goods"])
                break
            logger.warning("  [第二轮] 第 %d 次尝试失败 (hand=%.0f°): %s",
                           attempt + 1, hand_angle, round2.get("error", "未识别到订单"))
        completed.append("order_read_2")

        # 合并两轮读单结果
        order_list = {
            "round1": round1.get("orders", []),
            "round2": round2.get("orders", []),
        }

        # ===== 阶段 4: 再前进 + 按菜单顺序视觉抓取订单蔬菜 =====
        logger.info("=== 阶段 4: 再前进 + 按菜单顺序视觉抓取订单蔬菜 ===")

        # 4a) 先抬臂到列检测姿态 (第二次读单 Y=0 触底, 必须抬出再动底盘防拖地)
        runner.client.composite_run(
            x_mm=PICK_START_X_MM, y_mm=PICK_COL_Y_MM,
            arm=PICK_COL_ARM_DEG, hand=PICK_COL_HAND_DEG, speed=100,
        )
        logger.info("  抬臂到列检测姿态: X→%.0f Y→%.0f arm→%.0f hand→%.0f",
                    PICK_START_X_MM, PICK_COL_Y_MM, PICK_COL_ARM_DEG, PICK_COL_HAND_DEG)

        # 4b) 底盘前进 13cm 靠近蔬菜货架 (定位到右侧一列, 2026-08-08)
        from main.chassis import move_along_lane
        move_along_lane(vx=PICK_APPROACH_VX, distance_m=PICK_APPROACH_M)
        logger.info("  底盘前进 %.0fcm 完成 (任务点前进)", PICK_APPROACH_M * 100)

        # 4c) 订单蔬菜 → YOLO label (按菜单顺序, 去重)
        ordered_labels: List[Tuple[str, str]] = []
        seen_labels: set = set()
        for rnd in [round1, round2]:
            for o in (rnd.get("orders") or []):
                g = o.get("goods", "")
                lab = PICK_VEGGIE_LABEL_MAP.get(g)
                if g and lab and lab not in seen_labels:
                    seen_labels.add(lab)
                    ordered_labels.append((g, lab))
        logger.info("  订单蔬菜 (按菜单顺序): %s",
                    [(g, lab) for g, lab in ordered_labels])

        # 4d) 一列一列检测抓取 (先右列后左列, 2026-08-08)
        # 放置位跟蔬菜本身 (第几个订单) 绑定, 不是跟抓取成功顺序.
        # 已抓取的蔬菜不再重复尝试 (同一棵在右列抓过后左列还会出现) (2026-08-08)
        picked_count = 0
        picked_labels: set = set()
        for col in PICK_COLUMN_ORDER:
            # 切到左列 (右列结束后底盘前移)
            if col == "left" and PICK_COLUMN_MOVE_M > 1e-3:
                move_along_lane(vx=PICK_COLUMN_MOVE_VX, distance_m=PICK_COLUMN_MOVE_M)
                logger.info("  切换到左列: 前进 %.2fm", PICK_COLUMN_MOVE_M)
            for idx, (goods_name, label) in enumerate(ordered_labels):
                if label in picked_labels:
                    logger.info("  [%s] 已在 %s列 抓到, 跳过重复抓取", goods_name, col)
                    continue
                place = PICK_PLACE_1 if idx == 0 else PICK_PLACE_2
                logger.info("  → 抓取 '%s' (label=%s, %s列, 订单%d)",
                            goods_name, label, col, idx + 1)
                if _pick_one_veggie_visual(runner, goods_name, label, place, column=col):
                    picked_count += 1
                    picked_labels.add(label)
                    completed.append(f"picked_{picked_count}")
                else:
                    logger.info("  [%s] %s列 未抓到 (原因见上方检测日志)", goods_name, col)

        if picked_count == 0:
            logger.warning("  最终未能抓取任何蔬菜 (全部未收敛)")

        completed.append("pick_goods")

    except Exception as exc:
        logger.exception("get_order 任务失败: %s", exc)
        return {
            "ok": False,
            "completed": completed,
            "order_list": order_list,
            "error": str(exc),
        }

    return {
        "ok": True,
        "completed": completed,
        "order_list": order_list,
    }


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(name)s] %(message)s")
    result = run()
    print("任务六 智能接单 执行结果:", result)