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
