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
        # 默认已在 CurvatureAdaptiveOuterLoop 里下调过（kp_theta=1.2, omega_gain=0.35），
        # 弯道不再"打满舵"。这里显式写出来便于实车调参。
        kp_y=0.5,
        kp_theta=1.2,
        omega_gain=0.35,
        k_curvature=0.25,
        # omega_cap 比类默认 1.8 保守：axis_mix≈1 时 omega 是全额（不再被同时存在的 vy 稀释），
        # 单轮电源撞峰风险上调一档；现场若仍见电源报警再降至 1.2。
        omega_cap=1.5,
        ey_release=0.02,
        ea_release=0.05,
        hold_ms=250.0,
        # axis_mix sigmoid 分水岭：kappa=1.0 是「弧度偏差≈0.6 rad 或 弧度变化≈1.5 rad/s」级别的
        # 中等强度弯道；width=0.5 给 ±0.5 单位左右的平滑过渡带。现场调:
        # - 直线上小抖都触发到 ω → 把 kappa_axis_width 调到 0.7+
        # - 进弯后才打到 ω 全额 → 把 kappa_axis_center 调到 0.7 左右
        kappa_axis_center=1.0,
        kappa_axis_width=0.5,
        # vy_floor（修复 2026-07-16 弯内侧压边界）：弯道段保留 15% 横向修正能力，
        # 让 error_y 在长弯里仍能缓慢收敛，不会压内边界。
        # 现场调：弯内侧仍贴线 → 调到 0.20~0.25；旋转被稀释（车在弯里晃）→ 调到 0.10
        vy_floor=0.15,
    )

    # 最后一道闸：单轮 |v| 饱和到 0.55 m/s，单帧最大加速 0.4 m/s / 减速 0.6 m/s
    smoother = WheelSmoother(max_abs=0.55, max_accel=0.4, max_decel=0.6)

    def _on_tick(state: "LaneState", speeds):  # type: ignore[name-defined]
        dbg = outer.debug_snapshot()
        print(
            f"ey={state.error_y:+.4f} ea={state.error_angle:+.4f} "
            f"kappa={dbg['kappa_ema']:.3f} dkappa={dbg['dkappa_ema']:.3f} "
            f"axis_mix={dbg['axis_mix']:.3f} vy_keep={dbg['vy_keep']:.3f} "
            f"streak={dbg['straight_streak_ms']:.0f}ms "
            f"v=[{speeds[0]:+.2f},{speeds[1]:+.2f},{speeds[2]:+.2f},{speeds[3]:+.2f}]"
        )

    runner = DoubleLoopRunner(
        api=api,
        outer=outer,
        hz=50.0,
        smoother=smoother,
        on_tick=_on_tick,
    )
    runner.run(max_seconds=max_seconds)
    api.stop_lane_feed()


if __name__ == "__main__":
    main()