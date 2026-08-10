"""纯 Python 底盘路径平滑：不访问 odom、串口或 runtime。"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class Pose2D:
    x_m: float
    y_m: float
    heading_rad: float = 0.0


@dataclass(frozen=True)
class PathWaypoint:
    x_m: float
    y_m: float
    heading_rad: Optional[float] = None
    speed_mps: Optional[float] = None
    stop: bool = False


@dataclass(frozen=True)
class PathSample:
    pose: Pose2D
    arc_length_m: float
    curvature_inv_m: float
    speed_mps: float
    waypoint_index: int


@dataclass(frozen=True)
class SmoothPath:
    samples: Tuple[PathSample, ...]
    waypoints: Tuple[PathWaypoint, ...]

    @property
    def length_m(self) -> float:
        return self.samples[-1].arc_length_m if self.samples else 0.0


def _finite(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def _heading(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.atan2(b[1] - a[1], b[0] - a[0])


def _lerp(a: Pose2D, b: Pose2D, t: float) -> Pose2D:
    return Pose2D(a.x_m + (b.x_m - a.x_m) * t,
                  a.y_m + (b.y_m - a.y_m) * t,
                  a.heading_rad + (b.heading_rad - a.heading_rad) * t)


def plan_smooth_path(start: Pose2D, waypoints: Sequence[PathWaypoint],
                     goal: Pose2D, *, spacing_m: float = 0.05,
                     max_speed_mps: float = 0.2,
                     max_curvature_inv_m: Optional[float] = None) -> SmoothPath:
    """生成保守的分段平滑路径；输入 waypoint 全部作为硬采样点保留。"""
    spacing_m = _finite(spacing_m, "spacing_m")
    max_speed_mps = _finite(max_speed_mps, "max_speed_mps")
    if spacing_m <= 0 or max_speed_mps < 0:
        raise ValueError("spacing_m must be > 0 and max_speed_mps must be >= 0")
    points = [PathWaypoint(start.x_m, start.y_m, start.heading_rad)]
    points.extend(waypoints)
    points.append(PathWaypoint(goal.x_m, goal.y_m, goal.heading_rad))
    for index, point in enumerate(points):
        _finite(point.x_m, f"waypoints[{index}].x_m")
        _finite(point.y_m, f"waypoints[{index}].y_m")
        if point.heading_rad is not None:
            _finite(point.heading_rad, f"waypoints[{index}].heading_rad")
        if point.speed_mps is not None and (_finite(point.speed_mps, "speed_mps") < 0):
            raise ValueError("speed_mps must be >= 0")
    samples = []
    arc = 0.0
    for index, (left, right) in enumerate(zip(points, points[1:])):
        a = (left.x_m, left.y_m)
        b = (right.x_m, right.y_m)
        distance = math.hypot(b[0] - a[0], b[1] - a[1])
        if distance == 0.0:
            raise ValueError(f"duplicate waypoint at index {index + 1}")
        h0 = left.heading_rad if left.heading_rad is not None else _heading(a, b)
        h1 = right.heading_rad if right.heading_rad is not None else _heading(a, b)
        count = max(1, int(math.ceil(distance / spacing_m)))
        for step in range(count + 1):
            if index and step == 0:
                continue
            t = step / count
            pose = _lerp(Pose2D(left.x_m, left.y_m, h0),
                         Pose2D(right.x_m, right.y_m, h1), t)
            curvature = 0.0
            speed = min(max_speed_mps, left.speed_mps or max_speed_mps,
                         right.speed_mps or max_speed_mps)
            if max_curvature_inv_m is not None:
                curvature = max(-abs(max_curvature_inv_m), min(abs(max_curvature_inv_m), curvature))
            samples.append(PathSample(pose, arc + distance * t, curvature, speed, index))
        arc += distance
    if points and (not samples or samples[-1].pose != goal):
        samples.append(PathSample(goal, arc, 0.0, min(max_speed_mps, goal.heading_rad * 0 + max_speed_mps), len(points) - 2))
    return SmoothPath(tuple(samples), tuple(waypoints))


def load_waypoints_geometry(config_path: Optional[str] = None, *,
                            spacing_m: float = 0.05,
                            max_speed_mps: float = 0.2,
                            max_curvature_inv_m: Optional[float] = None
                            ) -> Tuple[list, dict]:
    """从 task_config.yml（或自定义路径）的 waypoints 段解析几何路径配置。

    只消费带几何坐标（``x_m``/``y_m``）的条目；纯任务触发条目（只有
    ``ir_threshold_m``/``dis_at_least_m``/``task_id`` 等, 没有坐标）会被跳过——
    保证旧的 mission-only YAML 向后兼容。每个几何条目可带:
      heading_deg  期望航向（度；缺省 = 沿该段方向自动推导）
      speed_mps    该点目标速度（缺省 = max_speed_mps）
      stop         该点停车（速度压 0）

    校验：有限坐标 / 有限 heading / 重复点 / spacing>0 / speed>=0。
    返回 ``(waypoints, params)``:
      waypoints — 校验过的 PathWaypoint 序列
      params    — 解析后的几何参数 dict（spacing_m / max_speed_mps /
                  max_curvature_inv_m），可直接传给 ``plan_smooth_path``。
    """
    import os
    from pathlib import Path

    params = {
        "spacing_m": _finite(spacing_m, "spacing_m"),
        "max_speed_mps": _finite(max_speed_mps, "max_speed_mps"),
        "max_curvature_inv_m": (abs(_finite(max_curvature_inv_m, "max_curvature_inv_m"))
                                if max_curvature_inv_m is not None else None),
    }
    if params["spacing_m"] <= 0 or params["max_speed_mps"] < 0:
        raise ValueError("spacing_m must be > 0 and max_speed_mps must be >= 0")

    # 定位配置文件：显式路径优先，否则仓库根目录默认 task_config.yml
    if config_path:
        path = Path(config_path).resolve()
    else:
        repo_root = Path(__file__).resolve().parents[3]
        path = repo_root / "task_config.yml"
    if not path.is_file():
        raise FileNotFoundError(f"任务配置文件不存在: {path}")

    try:
        import yaml
    except ModuleNotFoundError as exc:  # pragma: no cover
        raise RuntimeError("缺少 PyYAML 依赖,请先执行: python3 -m pip install pyyaml") from exc
    with path.open("r", encoding="utf-8") as f:
        all_cfg = yaml.safe_load(f)
    if not isinstance(all_cfg, dict):
        raise ValueError(f"{path} 顶层必须是 mapping")
    raw = all_cfg.get("waypoints")
    if not isinstance(raw, list):
        raise ValueError(f"{path} 里没有 waypoints 段 (或不是 list)")

    waypoints = []
    for index, entry in enumerate(raw):
        if not isinstance(entry, dict):
            continue
        x = entry.get("x_m")
        y = entry.get("y_m")
        if x is None or y is None:
            continue  # 任务触发条目, 无几何坐标 —— 跳过
        x = _finite(x, f"waypoints[{index}].x_m")
        y = _finite(y, f"waypoints[{index}].y_m")
        heading_rad = None
        if entry.get("heading_deg") is not None:
            heading_rad = math.radians(_finite(
                entry["heading_deg"], f"waypoints[{index}].heading_deg"))
        speed = entry.get("speed_mps")
        if speed is not None:
            speed = _finite(speed, f"waypoints[{index}].speed_mps")
            if speed < 0:
                raise ValueError("speed_mps must be >= 0")
        waypoints.append(PathWaypoint(
            x_m=x, y_m=y, heading_rad=heading_rad, speed_mps=speed,
            stop=bool(entry.get("stop", False)),
        ))
        if len(waypoints) > 1 and (waypoints[-2].x_m == x and waypoints[-2].y_m == y):
            raise ValueError(f"duplicate waypoint at index {index}")

    return waypoints, params
