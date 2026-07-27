"""main/chassis/examples/05_subscribe_lane_state.py
弧度偏差自适应巡线的"WS 直读"变体：

与 04_curvature_adaptive.py 的差别
-----------------------------------
- 感知层：直接走 ``RuntimeWsClient.realtime_lane_state()``（realtime 通道，
  不进 job_queue、不打 ZMQ、不抢 car_lock），而不是 HTTP 轮询。
  通道路径与 04 完全一致（都读同一份 ``lane_feed`` 守护线程缓存），
  只是传输层从 HTTP 换成 WS，单轮 RTT 从 ~10ms 降到 ~5ms。
- 控制律：``CurvatureAdaptiveOuterLoop`` + ``WheelSmoother`` 手写内层循环，
  而不是 ``DoubleLoopRunner``——便于 ``dry_run`` 开关与 ``on_tick`` 打印格式
  自定义（按 1/50s 一行打印 curvature / axis_mix / vy_keep / ey_int / streak）。
- ``set_wheel_speeds`` 走 ``ChassisClient.set_wheel_speeds``（已内置
  WS 优先 + HTTP 回落）。

适用场景
--------
- 想在调试时打开 dry_run 纯打印观察 ``kappa`` / ``dkappa`` / ``axis_mix``
  / ``ey_int`` 分布，不让车真的跑；
- 想拿 WS 通道 50Hz 的 RTT 优势（相比 04 的 HTTP 轮询，每轮省 ~5ms）；
- 想在 chassis 控制律与 WS realtime op 之间做端到端 sanity 检查。

用法
----
    python3 -m main.chassis.examples.05_subscribe_lane_state            # 默认 50Hz / 跑 85s
    # 想 dry_run 纯打印不下发，直接改 __main__ 末尾：
    #     subscribe_lane_state(hz=50.0, max_seconds=85.0, dry_run=True)
    # 文件名以数字开头，子模块无法 `from main.chassis.examples import ...` 导入；
    # 只走 `python3 -m main.chassis.examples.05_subscribe_lane_state` 或脚本顶层 `if __name__ == "__main__"`。

依赖
----
全部走 ``main.chassis`` 公共 API：

- ``ChassisClient`` — 统一 HTTP + WS 连接入口
- ``CurvatureAdaptiveOuterLoop`` — 弧度偏差自适应外环
- ``WheelSmoother`` — 下发前单轮 slew-rate 限幅
- ``LaneState`` — lane 误差缓存视图（dataclass）
"""
from typing import Optional

from main.chassis import (
    ChassisClient,
    CurvatureAdaptiveOuterLoop,
    WheelSmoother,
)
from main.chassis.state import LaneState


def subscribe_lane_state(
    hz: float = 50.0,
    max_seconds: Optional[float] = 50.0,
    dry_run: bool = False,
    v_max: float = 0.30,
    v_min: float = 0.08,
    kappa_full: float = 0.6,
    dkappa_full: float = 1.5,
    kp_y: float = 0.65,
    kp_theta: float = 1.2,
    omega_gain: float = 0.5,
    k_curvature: float = 0.4,
    omega_cap: float = 1.8,
    ey_release: float = 0.02,
    ea_release: float = 0.05,
    hold_ms: float = 250.0,
    # 轴向互斥权重（v6+）：直线段 axis_mix≈0 时 vy 接管（朝向不变、质心斜向），
    # 弯道段 axis_mix≈1 时 ω 接管（后轮做轴、前轮差速旋转）。
    kappa_axis_center: float = 1.0,
    kappa_axis_width: float = 0.5,
    # 弯道段 vy 保底比例（修复 2026-07-16 弯内侧压边界）：
    # axis_mix≈1 时保留 vy_raw * vy_floor 的横向修正能力，让 error_y 在长弯里仍能收敛。
    # 经验区间：0.10~0.25。0=完全互斥（不推荐）；>0.4≈Stanley 全时修正。
    vy_floor: float = 0.15,
    # 横向 I 项（修复 2026-07-16 直行稳态误差）：
    # 纯 P 控制遇到恒定 bias 永远需要稳态 error_y 维持修正力，看起来就是过偏才矫正。
    # leaky 积分项消除稳态偏差。ki_y=0 退化到纯 P。
    # 经验：ki_y=0.6（与 kp_y=0.65 同量纲）, ey_int_cap=0.10 m（抗饱和）, ey_int_decay=0.5（半衰期≈1.4s）
    ki_y: float = 0.6,
    ey_int_cap: float = 0.10,
    ey_int_decay: float = 0.5,
    r_eff: float = 0.30,
    wheel_max_abs: float = 0.55,
    wheel_max_accel: float = 0.4,
    wheel_max_decel: float = 0.6,
) -> None:
    """按 50Hz 跑 curvature-adaptive 外环；详细参数说明见模块 docstring。"""
    import time

    api = ChassisClient.connect()
    if not api.ws_ready:
        print("[subscribe_lane_state] WS 连接失败，回退到 HTTP 轮询路径（lane 读取将改走 ChassisClient.get_lane_state）")

    outer = CurvatureAdaptiveOuterLoop(
        v_max=v_max,
        v_min=v_min,
        kappa_full=kappa_full,
        dkappa_full=dkappa_full,
        kp_y=kp_y,
        kp_theta=kp_theta,
        omega_gain=omega_gain,
        k_curvature=k_curvature,
        omega_cap=omega_cap,
        ey_release=ey_release,
        ea_release=ea_release,
        hold_ms=hold_ms,
        kappa_axis_center=kappa_axis_center,
        kappa_axis_width=kappa_axis_width,
        vy_floor=vy_floor,
        ki_y=ki_y,
        ey_int_cap=ey_int_cap,
        ey_int_decay=ey_int_decay,
        r_eff=r_eff,
    )

    smoother = WheelSmoother(
        max_abs=wheel_max_abs,
        max_accel=wheel_max_accel,
        max_decel=wheel_max_decel,
    )

    period = 1.0 / max(hz, 1e-3)
    deadline = None if max_seconds is None else time.monotonic() + max_seconds

    next_t = time.monotonic()
    print(
        f"[subscribe_lane_state] start hz={hz:.1f} via {'WS' if api.ws_ready else 'HTTP'}  "
        f"v_max={v_max:.2f} v_min={v_min:.2f} "
        f"kappa_full={kappa_full:.2f} hold_ms={hold_ms:.0f}"
    )
    try:
        while deadline is None or time.monotonic() < deadline:
            now = time.monotonic()
            sleep = next_t - now
            if sleep > 0:
                time.sleep(sleep)
            next_t += period
            dt = period

            try:
                if api.ws_ready:
                    payload = api.ws.realtime_lane_state() or {}
                else:
                    payload = api.get_lane_state() or {}
                state = LaneState.from_lane_state_payload(payload)
            except Exception as exc:
                print(f"[subscribe_lane_state] lane_state read error: {exc!r}")
                state = LaneState()

            target_speeds = outer.step(state, dt)
            target_speeds = smoother.step(target_speeds)

            if not dry_run:
                try:
                    api.set_wheel_speeds(target_speeds)
                except Exception as exc:
                    print(f"[subscribe_lane_state] set_wheel_speeds error: {exc!r}")

            v1, v2, v3, v4 = target_speeds
            dbg = outer.debug_snapshot()
            print(
                f"ey={state.error_y!s:>10}  ea={state.error_angle!s:>10}  "
                f"kappa={dbg['kappa_ema']:.3f}  dkappa={dbg['dkappa_ema']:.3f}  "
                f"axis_mix={dbg['axis_mix']:.3f}  vy_keep={dbg['vy_keep']:.3f}  "
                f"ey_int={dbg['ey_int']:+.4f}  "
                f"streak={dbg['straight_streak_ms']:>5.0f}ms  "
                f"v1={v1:>8.4f}  v2={v2:>8.4f}  v3={v3:>8.4f}  v4={v4:>8.4f}"
            )
    except KeyboardInterrupt:
        print("[subscribe_lane_state] KeyboardInterrupt -> exit")
    finally:
        if not dry_run:
            try:
                api.stop_wheel_speeds()
            except Exception:
                pass
        try:
            api.ws.close()
        except Exception:
            pass


if __name__ == "__main__":
    subscribe_lane_state(hz=50.0, max_seconds=85.0, dry_run=False)