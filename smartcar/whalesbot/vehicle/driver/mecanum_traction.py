"""麦克纳姆轮四轮牵引补偿的纯数学实现。"""

from __future__ import annotations

from typing import Iterable

import numpy as np


_TRACTION_NULL_VECTOR = np.array([1.0, -1.0, 1.0, -1.0])


def apply_traction_bias(
    base_wheels: Iterable[float],
    wheel_to_vehicle_matrix: np.ndarray,
    vx: float,
    vy: float,
    wz: float,
    *,
    min_translation: float = 0.02,
    min_axis_speed: float = 0.01,
    min_wheel_ratio: float = 0.15,
    bias_ratio: float = 0.25,
    max_bias: float = 0.08,
) -> np.ndarray:
    """在不改变车体速度的前提下，给组合平移增加四轮牵引。

    麦轮逆解的四维轮速有一个零空间方向 ``[1,-1,1,-1]``。沿该方向
    调整不会改变正解出的 ``vx/vy/wz``，但可以把组合平移时接近零的轮子
    推离零点。只有 x/y 均有足够速度且最小轮速确实偏低时才启用。
    """
    wheels = np.asarray(list(base_wheels), dtype=float)
    if wheels.shape != (4,) or not np.isfinite(wheels).all():
        return wheels

    if (
        abs(float(vx)) < min_axis_speed
        or abs(float(vy)) < min_axis_speed
        or abs(float(wz)) > min_translation
    ):
        return wheels

    motion_scale = max(abs(float(vx)), abs(float(vy)), min_translation)
    base_min = float(np.min(np.abs(wheels)))
    if base_min >= max(min_axis_speed, min_wheel_ratio * motion_scale):
        return wheels

    null_vector = _TRACTION_NULL_VECTOR
    forward = np.asarray(wheel_to_vehicle_matrix, dtype=float)
    if forward.shape != (4, 3) or not np.isfinite(forward).all():
        return wheels
    if not np.allclose(null_vector @ forward, 0.0, atol=1e-9):
        return wheels

    limit = min(float(max_bias), float(bias_ratio) * motion_scale)
    if limit <= 0.0:
        return wheels

    candidates = [-limit, 0.0, limit]
    for i in range(4):
        candidates.append(-wheels[i] / null_vector[i])
    for i in range(4):
        for j in range(i + 1, 4):
            for sign in (-1.0, 1.0):
                denominator = null_vector[i] - sign * null_vector[j]
                if abs(denominator) > 1e-12:
                    candidates.append(
                        (sign * wheels[j] - wheels[i]) / denominator
                    )

    valid = [alpha for alpha in candidates if -limit <= alpha <= limit]
    if not valid:
        return wheels

    def score(alpha):
        candidate = wheels + alpha * null_vector
        return (
            float(np.min(np.abs(candidate))),
            -float(np.sum((candidate - wheels) ** 2)),
        )

    alpha = max(valid, key=score)
    adjusted = wheels + alpha * null_vector
    if float(np.min(np.abs(adjusted))) <= base_min + 1e-9:
        return wheels
    return adjusted
