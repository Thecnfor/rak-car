"""main/chassis/examples/04_curvature_adaptive.py
弧度偏差自适应巡线：
- error_angle 偏离或大幅变动 → 自动降速 + 加强转向
- error_angle 与 error_y 同时小到阈值并稳定 hold_ms 后，恢复标称 v_max

下发层 ``WheelSmoother`` 兜底：单轮 |v| 饱和 + 单帧 slew rate 限幅，
防止弯道瞬间单轮目标跨度过大把下位机电源拉爆。

打印内部 curvature 估计便于实车调参：
    kappa_ema    弧度偏差（EMA 平滑）
    dkappa_ema   弧度偏差变化率（EMA 平滑）
    vx           当前下发前向速度
"""
from main.chassis import (
    ChassisClient,
    CurvatureAdaptiveOuterLoop,
    DoubleLoopRunner,
    WheelSmoother,
)


def main(max_seconds: float = 20.0) -> None:
    api = ChassisClient.connect()
    api.start_lane_feed(hz=20.0)

    outer = CurvatureAdaptiveOuterLoop(
        v_max=0.30,
        v_min=0.08,
        kappa_full=0.6,
        dkappa_full=1.5,
        # 默认已在 CurvatureAdaptiveOuterLoop 里下调过（kp_theta=1.2, omega_gain=0.35,
        # omega_cap=1.8），弯道不再"打满舵"。这里显式写出来便于实车调参。
        kp_y=0.5,
        kp_theta=1.2,
        omega_gain=0.35,
        k_curvature=0.25,
        omega_cap=1.8,
        ey_release=0.02,
        ea_release=0.05,
        hold_ms=250.0,
    )

    # 最后一道闸：单轮 |v| 饱和到 0.55 m/s，单帧最大加速 0.4 m/s / 减速 0.6 m/s
    smoother = WheelSmoother(max_abs=0.55, max_accel=0.4, max_decel=0.6)

    runner = DoubleLoopRunner(
        api=api,
        outer=outer,
        hz=50.0,
        smoother=smoother,
        on_tick=lambda state, speeds: print(
            f"ey={state.error_y:+.4f} ea={state.error_angle:+.4f} "
            f"kappa={outer._kappa_ema:.3f} dkappa={outer._dkappa_ema:.3f} "
            f"streak={outer._straight_streak_ms:.0f}ms "
            f"v=[{speeds[0]:+.2f},{speeds[1]:+.2f},{speeds[2]:+.2f},{speeds[3]:+.2f}]"
        ),
    )
    runner.run(max_seconds=max_seconds)
    api.stop_lane_feed()


if __name__ == "__main__":
    main()