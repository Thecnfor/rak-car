"""main/chassis/controllers/base.py
所有外环控律都遵守这套接口。

额外提供 ``WheelSmoother``：在控制律算出来的 4 轮目标速度上做：
1. 单轮 |v| 饱和
2. 单帧 slew rate 限幅（防突加速/突减速）
3. 异常跳变（如控制律切换、断电重启）的复位接口

为什么需要：麦轮逆解 ``vx ± vy ± r*omega`` 在大弧度差急转弯时，单轮目标
可能从 ``0.30`` 直接跳到 ``1.0+`` m/s；50Hz 外环下相当于 ~35 m/s² 阶跃，
下位机电源被瞬时拉爆 → 掉电压。``WheelSmoother`` 是下发前的最后一道闸。
"""
from abc import ABC, abstractmethod
from typing import List, Optional

from ..state import LaneState


def mecanum_inverse(vx: float, vy: float, omega: float, r: float) -> List[float]:
    """按 vehicle_to_wheel_matrix = [[1,-1,-1,1],[t,t,-t,-t],[r,r,r,r]] 推。
    这里只取机械混合部分（忽略 roller_angle），
    omega -> r*omega 中 r = half_track*tan(roller) + half_wheel_base。
    """
    r_eff = max(r, 1e-6)
    return [
        vx - vy + r_eff * omega,
        -vx + vy + r_eff * omega,
        -vx - vy + r_eff * omega,
        vx + vy + r_eff * omega,
    ]


class WheelSmoother:
    """4 轮目标线速度软化器：饱和 + slew rate 限幅。

    设计目标：把"控制律想要的轮速"平滑成"下位机收得到的轮速"，避免
    弯道入/出瞬间单轮目标跨度过大，把 MC602 电源拉掉。

    参数（按 50Hz 外环默认推算）：
    - ``max_abs``：单轮 |v| 上限。``0.55 m/s`` ≈ ``v_max(0.30) + |vy_max| + |r*omega_max|``
      加上 30% 余量；远超此值的单轮目标几乎一定是控制律 bug。
    - ``max_accel``：单帧最大加速量。``0.4 m/s/frame`` × 50Hz = 20 m/s²，对
      麦轮小车是 1~2 g 的可承受加速；再大就开始撞电流峰值。
    - ``max_decel``：单帧最大减速量，刻意比 ``max_accel`` 大一点（约 1.5x），
      让急停/丢线恢复时反应更快；电源对减速的反电势更友好。
    """

    def __init__(
        self,
        max_abs: float = 0.55,
        max_accel: float = 0.4,
        max_decel: float = 0.6,
    ) -> None:
        self.max_abs = float(max_abs)
        self.max_accel = float(max_accel)
        self.max_decel = float(max_decel)
        # 上一帧下发值；首帧没有"上一帧"，按 0 起步（外环起来前车也是停的）
        self._last: List[float] = [0.0, 0.0, 0.0, 0.0]
        self._has_last = False

    def reset(self, speeds: Optional[List[float]] = None) -> None:
        """重置内部"上一帧"。

        场景：
        - 外环 run() 进入时：传 ``None`` → 按 0 起步
        - emergency_stop 后：传 ``[0,0,0,0]`` 显式归零
        - 控制律切换：传旧控制律最后一帧，避免被新控制律的瞬间目标"撞到"
        """
        if speeds is None:
            self._last = [0.0, 0.0, 0.0, 0.0]
        else:
            self._last = [float(v) for v in speeds[:4]]
        self._has_last = True

    @property
    def last(self) -> List[float]:
        return list(self._last)

    def step(self, target: List[float]) -> List[float]:
        """把控制律原始目标软化成可下发目标。

        算法：
            v_clamped = clamp(target, ±max_abs)
            dv = v_clamped - last
            dv = clamp(dv, -max_decel, +max_accel)
            new = last + dv
        """
        if not self._has_last:
            # 首帧：直接拿目标作为"上一帧"，避免被空 _last 拽回 0
            self._last = [float(v) for v in target[:4]]
            self._has_last = True
            return list(self._last)

        out: List[float] = []
        for prev, raw in zip(self._last, target):
            tgt = max(-self.max_abs, min(self.max_abs, float(raw)))
            dv = tgt - prev
            if dv > self.max_accel:
                dv = self.max_accel
            elif dv < -self.max_decel:
                dv = -self.max_decel
            out.append(prev + dv)
        self._last = out
        return out


class OuterLoop(ABC):
    """外环控制律接口。

    step(state, dt) -> [v1, v2, v3, v4] 四轮线速度（m/s）。
    state.error_y / error_angle 始终是底盘前摄的视觉误差。
    """

    @abstractmethod
    def step(self, state: LaneState, dt: float) -> List[float]:
        ...

    @staticmethod
    def _safe_zero() -> List[float]:
        return [0.0, 0.0, 0.0, 0.0]
