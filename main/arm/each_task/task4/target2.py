"""task4 / target2 —— 侧摄目标识别球类 + 返回坐标

调 task_feed 守护线程 (GET /v1/realtime/vision/task, 10Hz 刷新) 实时识别
球类 (蓝/黄), 返回每球的归一化坐标 + 颜色 + 置信度。

输出字段 (每球一个 dict):
  - color       : "blue" / "yellow" / "unknown"  (label 映射, 见 _label_to_color)
  - cx_norm     : bbox 中心 x 归一化 [-0.5, 0.5], 0=画面正中, -0.5=最左
  - cy_norm     : bbox 中心 y 归一化 [-0.5, 0.5], 0=画面正中, +0.5=最下
  - w_norm      : bbox 宽归一化 [0, 1]
  - h_norm      : bbox 高归一化 [0, 1]
  - score       : 检测置信度 [0, 1]
  - det_id      : PaddleDet 跟踪 ID (帧间一致; task_feed 没启用跟踪时为 None)
  - cls_id      : 模型类别 id (int; 0=blue / 1=yellow 是常见约定, 但以 label 为准)
  - label       : 模型原始 label 字符串 (小写)

过滤阈值 (复用 constants.py, ARM_API.md §ARM业务约定):
  - TARGET_SCORE_MIN   : 0.85  score 低于此值丢弃 (实测 0.927-0.941, 留 0.08 余量)
  - TARGET_ASPECT_TOL  : 0.8   |w/h - 1| 高于此值丢弃 (球要圆)
  - TARGET_AREA_MIN    : 0.15  bbox 面积归一化下界 (太小的不要)
  - TARGET_AREA_MAX    : 0.50  bbox 面积归一化上界 (太大的不要)

⚠️ 2026-08-01: 删除 BALL_VERIFIED_* 位姿期望验证 (cx/cy/w/h/area/aspect 7 项)。
   用户实测 target1 位姿偏移时 BALL_VERIFIED_* 误伤过滤 (round 4 唯一检出的
   球被 reject)。fetch_balls 现在只过 score + aspect + area 三层基线过滤,
   cx/cy 不再卡范围。

⚠️ 本文件**自包含**: 只依赖 main.api_client / constants, 不 import task4 包内其它模块。
   原因: task5 目录里的辅助文件曾被外部动作清空过, 自包含保证 `python target2.py` 直接跑。

跑法:
    python main/arm/each_task/task4/target2.py --once             # 单次扫, 打印 + 返回
    python main/arm/each_task/task4/target2.py --once --save      # 单次 + 写 TASK4_TARGET_CACHE
    python main/arm/each_task/task4/target2.py --loop --hz 5     # 5Hz 轮询
    python main/arm/each_task/task4/target2.py --once --color blue  # 只看蓝色球
    python main/arm/each_task/task4/target2.py --once --debug     # 打印 raw detections 诊断
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Any, Optional

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

try:
    # 包内跑: python -m main.arm.each_task.task4.target2
    from .constants import (  # noqa: E402
        LOG_PREFIX_TASK4,
        COLOR_BLUE, COLOR_YELLOW, COLOR_UNKNOWN,
        TARGET_SCORE_MIN, TARGET_ASPECT_TOL,
        TARGET_AREA_MIN, TARGET_AREA_MAX,
        TASK4_TARGET_CACHE,
    )
except ImportError:  # pragma: no cover — 直接 python target2.py 时无包上下文
    from main.arm.each_task.task4.constants import (  # type: ignore
        LOG_PREFIX_TASK4,
        COLOR_BLUE, COLOR_YELLOW, COLOR_UNKNOWN,
        TARGET_SCORE_MIN, TARGET_ASPECT_TOL,
        TARGET_AREA_MIN, TARGET_AREA_MAX,
        TASK4_TARGET_CACHE,
    )

LOG_PREFIX: str = LOG_PREFIX_TASK4 + "/target2"

# ---- cls_id → color fallback (部分模型不返回 label 字符串, 用 cls_id 兜底) ----
# 约定: 0=blue, 1=yellow (跟 task4 训练数据对齐; 以 label 为准, cls_id 仅做 fallback)
CLS_ID_TO_COLOR: dict[int, str] = {
    0: COLOR_BLUE,
    1: COLOR_YELLOW,
}


# ---------- 核心 helper ----------

def _label_to_color(label: Optional[str], cls_id: Optional[int]) -> str:
    """PaddleDet label → 业务 color。

    优先级: label 字符串 > cls_id (int) > unknown。
    label 大小写归一; 含 'blue' / 'yellow' 关键词即映射; 否则 unknown。
    """
    if isinstance(label, str) and label:
        lo = label.strip().lower()
        if "blue" in lo:
            return COLOR_BLUE
        if "yellow" in lo:
            return COLOR_YELLOW
        return COLOR_UNKNOWN
    if isinstance(cls_id, int) and cls_id in CLS_ID_TO_COLOR:
        return CLS_ID_TO_COLOR[cls_id]
    return COLOR_UNKNOWN


def _norm_xy(bbox_norm: dict) -> tuple[float, float, float, float]:
    """bbox_norm dict → (cx_norm, cy_norm, w_norm, h_norm)。

    支持三种 bbox 格式 (按优先级匹配):
      - {"x_center": ..., "y_center": ..., "width": ..., "height": ...}
        (task4 实测 2026-07-25 — cls_id 16=ball_blue / 17=ball_yellow,
         cx/cy 是相对图像中心的归一化, 不是 [0,1] 左上原点)
      - {"cx": ..., "cy": ..., "w": ..., "h": ...}     (部分 PaddleDet 输出)
      - {"x1": ..., "y1": ..., "x2": ..., "y2": ...}   (xyxy 归一化, 左上原点)
    缺字段 → ValueError。
    """
    if not isinstance(bbox_norm, dict):
        raise ValueError(f"bbox_norm 不是 dict: {bbox_norm!r}")
    try:
        if all(k in bbox_norm for k in ("x_center", "y_center", "width", "height")):
            cx = float(bbox_norm["x_center"])
            cy = float(bbox_norm["y_center"])
            w = float(bbox_norm["width"])
            h = float(bbox_norm["height"])
        elif all(k in bbox_norm for k in ("cx", "cy", "w", "h")):
            cx = float(bbox_norm["cx"])
            cy = float(bbox_norm["cy"])
            w = float(bbox_norm["w"])
            h = float(bbox_norm["h"])
        elif all(k in bbox_norm for k in ("x1", "y1", "x2", "y2")):
            x1 = float(bbox_norm["x1"])
            y1 = float(bbox_norm["y1"])
            x2 = float(bbox_norm["x2"])
            y2 = float(bbox_norm["y2"])
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w = x2 - x1
            h = y2 - y1
        else:
            raise ValueError(f"bbox_norm 字段缺失 (需 x_center/y_center/width/height "
                              f"或 cx/cy/w/h 或 x1/y1/x2/y2): {bbox_norm!r}")
    except (TypeError, ValueError) as exc:
        raise ValueError(f"bbox_norm 解析失败: {exc} (raw={bbox_norm!r})") from exc
    return cx, cy, w, h


def _is_ball_like(
    bbox_norm: dict,
    score: float,
    aspect_tol: float,
    area_min: float,
    area_max: float,
) -> bool:
    """球形过滤: score / 宽高比 (≈1) / 面积 都过关才返回 True。

    异常时返回 False (静默丢弃, 不抛)。
    """
    try:
        cx, cy, w, h = _norm_xy(bbox_norm)
    except (ValueError, TypeError):
        return False
    if w <= 0 or h <= 0:
        return False
    # 宽高比 (球 ≈ 1)
    if abs(w / h - 1.0) > aspect_tol:
        return False
    area = w * h
    if not (area_min <= area <= area_max):
        return False
    return True


# ---------- 核心 API ----------

def fetch_balls(
    http_client,
    *,
    score_min: Optional[float] = None,
    color_filter: Optional[str] = None,
    aspect_tol: Optional[float] = None,
    area_min: Optional[float] = None,
    area_max: Optional[float] = None,
) -> list[dict]:
    """调 task_feed 拿当前帧的球类识别结果 + 按阈值过滤 + 颜色映射。

    Args:
        http_client: RuntimeApiClient 实例 (有 .get_task_state() 方法)。
        score_min: score 阈值 (None → 用 constants.TARGET_SCORE_MIN)
        color_filter: "blue" / "yellow" / None (None=不按颜色过滤)
        aspect_tol / area_min / area_max: 几何阈值 (None → 用 constants)

    Returns:
        list[dict]: 每球一个 dict, 字段见模块 docstring。
            task_feed 未运行 / 无结果时返回 []。
    """
    score_min = TARGET_SCORE_MIN if score_min is None else float(score_min)
    aspect_tol = TARGET_ASPECT_TOL if aspect_tol is None else float(aspect_tol)
    area_min = TARGET_AREA_MIN if area_min is None else float(area_min)
    area_max = TARGET_AREA_MAX if area_max is None else float(area_max)

    try:
        resp = http_client.get_task_state()
    except Exception as e:
        print(f"  [{LOG_PREFIX}] get_task_state 异常: "
              f"{type(e).__name__}: {str(e)[:80]}", file=sys.stderr)
        return []

    task_state = (resp or {}).get("task_state") or {}
    if not task_state.get("active"):
        # task_feed 未启 / 刚启动
        return []
    detections = task_state.get("detections") or []
    if not isinstance(detections, list):
        return []

    out: list[dict] = []
    for det in detections:
        if not isinstance(det, dict):
            continue
        try:
            score = float(det.get("score", 0.0))
        except (TypeError, ValueError):
            continue
        if score < score_min:
            continue
        bbox_norm = det.get("bbox_norm") or {}
        if not _is_ball_like(bbox_norm, score, aspect_tol, area_min, area_max):
            continue
        try:
            cx, cy, w, h = _norm_xy(bbox_norm)
        except (ValueError, TypeError):
            continue
        label = det.get("label")
        cls_id = det.get("cls_id")
        color = _label_to_color(label, cls_id)
        if color_filter and color != color_filter:
            continue
        ball = {
            "color": color,
            "cx_norm": cx,
            "cy_norm": cy,
            "w_norm": w,
            "h_norm": h,
            "score": score,
            "det_id": det.get("det_id"),
            "cls_id": cls_id,
            "label": label,
        }
        out.append(ball)

    return out


def save_latest(balls: list[dict], path: str = TASK4_TARGET_CACHE) -> str:
    """把当前帧识别结果写盘 (供 b2 / b3 后续步骤读取)。

    写盘字段: {"timestamp", "count", "balls": [...]}。
    失败返回 None (warn 但不抛)。
    """
    payload = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "count": len(balls),
        "balls": balls,
    }
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        return path
    except Exception as e:
        print(f"  [{LOG_PREFIX}] save_latest 失败: "
              f"{type(e).__name__}: {str(e)[:80]}", file=sys.stderr)
        return ""


# ---------- 打印 ----------

def _fmt_ball(b: dict, idx: int) -> str:
    return (f"  [{idx}] color={b['color']:7s}  "
            f"cx={b['cx_norm']:+.3f}  cy={b['cy_norm']:+.3f}  "
            f"w×h={b['w_norm']:.3f}×{b['h_norm']:.3f}  "
            f"score={b['score']:.3f}  "
            f"det_id={b['det_id']}")


def _print_balls(balls: list[dict], raw: Optional[dict] = None) -> None:
    print(f"  [{LOG_PREFIX}] 识别到 {len(balls)} 个球")
    for i, b in enumerate(balls):
        print(_fmt_ball(b, i))
    if raw is not None:
        print(f"  [{LOG_PREFIX}] raw task_state.active={raw.get('active')}  "
              f"raw_count={len(raw.get('detections') or [])}  "
              f"updated_at={raw.get('updated_at')}")


# ---------- 主入口 ----------

def step_target2_once(
    http_client,
    *,
    score_min: Optional[float] = None,
    color_filter: Optional[str] = None,
    save: bool = False,
    show_raw: bool = False,
    debug: bool = False,
) -> dict:
    """单次识别 + (可选) 写盘。

    Args:
        debug: True 时打印 raw detections 全部字段 (前 3 条) + active/count/updated_at;
               用于现场诊断 task_feed 输出结构跟 fetch_balls 假设不一致的问题。

    Returns:
        {"ok": bool, "balls": list[dict], "raw_task_state": dict|None, "saved_path": str|None}
    """
    print(f"========== {LOG_PREFIX} step_target2_once ==========")
    raw_task_state = None
    try:
        raw_task_state = (http_client.get_task_state() or {}).get("task_state")
    except Exception as e:
        print(f"  [{LOG_PREFIX}] get_task_state 异常: "
              f"{type(e).__name__}: {str(e)[:80]}")
    if debug:
        # 把 raw 全部内容打出来: 诊断 fetch_balls 假设跟实际格式不匹配时用
        print(f"  [{LOG_PREFIX}] [DEBUG] raw task_state keys = "
              f"{list(raw_task_state.keys()) if isinstance(raw_task_state, dict) else 'N/A'}")
        if isinstance(raw_task_state, dict):
            print(f"  [{LOG_PREFIX}] [DEBUG] active={raw_task_state.get('active')}  "
                  f"mode={raw_task_state.get('mode')}  "
                  f"updated_at={raw_task_state.get('updated_at')}")
            detections = raw_task_state.get("detections") or []
            print(f"  [{LOG_PREFIX}] [DEBUG] detections count = {len(detections)}")
            for i, det in enumerate(detections[:3]):
                print(f"  [{LOG_PREFIX}] [DEBUG] det[{i}] = {det!r}")
    balls = fetch_balls(
        http_client,
        score_min=score_min,
        color_filter=color_filter,
    )
    saved_path = save_latest(balls) if save else None
    _print_balls(balls, raw=raw_task_state if show_raw else None)
    if saved_path:
        print(f"  [{LOG_PREFIX}] 已写盘: {saved_path}")
    print(f"========== {LOG_PREFIX} 完成 ==========\n")
    return {
        "ok": True,
        "balls": balls,
        "raw_task_state": raw_task_state,
        "saved_path": saved_path,
    }


def step_target2_loop(
    http_client,
    *,
    hz: float = 5.0,
    score_min: Optional[float] = None,
    color_filter: Optional[str] = None,
    save_each: bool = False,
    duration_s: Optional[float] = None,
) -> dict:
    """轮询识别 + 持续打印。Ctrl-C 中止。

    Args:
        hz: 轮询频率 (Hz); 默认 5Hz。
        duration_s: 跑多少秒后自动退出 (None=无限, Ctrl-C 中止)。
        save_each: 每帧写盘 (TASK4_TARGET_CACHE)。
    """
    interval = 1.0 / max(hz, 0.1)
    print(f"========== {LOG_PREFIX} step_target2_loop (hz={hz}) ==========")
    if duration_s:
        print(f"  [{LOG_PREFIX}] 限时 {duration_s}s, 自动退出")
    else:
        print(f"  [{LOG_PREFIX}] 无限循环, Ctrl-C 中止")
    t_start = time.monotonic()
    n_rounds = 0
    try:
        while True:
            balls = fetch_balls(
                http_client,
                score_min=score_min,
                color_filter=color_filter,
            )
            ts = time.strftime("%H:%M:%S")
            print(f"  [{LOG_PREFIX}] {ts} 识别到 {len(balls)} 个球")
            for i, b in enumerate(balls):
                print(_fmt_ball(b, i))
            if save_each:
                save_latest(balls)
            n_rounds += 1
            if duration_s and time.monotonic() - t_start >= duration_s:
                break
            time.sleep(max(0.0, interval - 0.001))
    except KeyboardInterrupt:
        print(f"\n  [{LOG_PREFIX}] Ctrl-C, 退出")
    elapsed = time.monotonic() - t_start
    print(f"========== {LOG_PREFIX} 完成 ({n_rounds} 轮, {elapsed:.1f}s) ==========\n")
    return {"ok": True, "rounds": n_rounds, "elapsed_s": elapsed}


# ---------- CLI ----------

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="task4 target2: 侧摄识别球类 (蓝/黄) + 返回归一化坐标",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--once", action="store_true", default=True,
                      help="单次识别 (默认)")
    mode.add_argument("--loop", action="store_true",
                      help="持续轮询 (Ctrl-C 中止)")
    p.add_argument("--hz", type=float, default=5.0, help="轮询频率 (Hz)")
    p.add_argument("--duration", type=float, default=None,
                   help="限时秒数 (loop 模式)")
    p.add_argument("--color", choices=[COLOR_BLUE, COLOR_YELLOW], default=None,
                   help="按颜色过滤 (默认不过滤)")
    p.add_argument("--score-min", type=float, default=None,
                   help=f"score 阈值 (默认 {TARGET_SCORE_MIN})")
    p.add_argument("--save", action="store_true",
                   help="写盘到 TASK4_TARGET_CACHE (供 b2/b3 复用)")
    p.add_argument("--show-raw", action="store_true",
                   help="打印 task_state.active/raw_count/updated_at")
    p.add_argument("--debug", action="store_true",
                   help="打印 raw detections 前 3 条完整字段 (诊断用)")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    from main.api_client import RuntimeApiClient  # noqa: E402 — 延迟 import, 减少启动开销
    http = RuntimeApiClient()
    if args.loop:
        step_target2_loop(
            http,
            hz=args.hz,
            score_min=args.score_min,
            color_filter=args.color,
            save_each=args.save,
            duration_s=args.duration,
        )
    else:
        step_target2_once(
            http,
            score_min=args.score_min,
            color_filter=args.color,
            save=args.save,
            show_raw=args.show_raw,
            debug=args.debug,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())