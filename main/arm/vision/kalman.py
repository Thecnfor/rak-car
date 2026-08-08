"""main/arm/vision/kalman.py — arm 视觉伺服的 bbox 轨迹 Kalman 平滑 (2026-08-09).

与 runtime/services/chassis_align.py::_KalmanTracker 同构, 但独立封装——
main/ 不 import runtime (CLAUDE.md 硬约束), 只能各自一份。改动需两边同步。

用途: velocity 模式视觉伺服 (find_target_velocity / find_target_4dof) 里,
task_feed 检测的 bbox 中心有帧间抖动 + 推理延迟; 在 step 算 dx/dy 之前
平滑 x_center/y_center, 控制输入更稳。

filterpy 是纯 numpy 库, Python 3.8 兼容; 未安装时调用方自动禁用
(kalman=False 降级回原始检测)。常速模型 (CV): 状态 [cx, cy, vcx, vcy],
Q 小 (目标近似静止/缓动), R 控制平滑强度。只处理有检测帧; 丢帧 (pick=None)
由原逻辑处理, 本类不参与。
"""
from __future__ import annotations

import logging
from typing import Tuple

logger = logging.getLogger(__name__)


class ArmKalmanTracker:
    """arm 视觉伺服的 bbox 中心常速 Kalman 平滑器。

    用法:
        tr = ArmKalmanTracker(dt=0.05)
        # 有检测帧:
        cx_s, cy_s = tr.update(pick.bbox_norm.x_center, pick.bbox_norm.y_center)
        dx = cx_s - setpoint_x
    """

    def __init__(self, dt: float = 0.05, q: float = 1e-3, r: float = 1e-2):
        try:
            import numpy as np
            from filterpy.kalman import KalmanFilter  # 懒加载: 未装则上层降级
        except ImportError:
            raise ImportError(
                "filterpy 未安装, kalman 不可用 (pip install filterpy)"
            )
        self._np = np
        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.F = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
            dtype=float,
        )
        self.kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=float)
        self.kf.P *= 1.0
        self.kf.Q = np.eye(4) * q
        self.kf.R = np.eye(2) * r
        self._initialized = False

    def update(self, cx: float, cy: float) -> Tuple[float, float]:
        """喂一帧检测中心, 返回平滑后的 (cx, cy)。首帧直接初始化（不过滤）。"""
        np = self._np
        z = np.array([[cx], [cy]], dtype=float)
        if not self._initialized:
            self.kf.x = np.array([[cx], [cy], [0.0], [0.0]], dtype=float)
            self._initialized = True
            return cx, cy
        self.kf.predict()
        self.kf.update(z)
        return float(self.kf.x[0, 0]), float(self.kf.x[1, 0])
