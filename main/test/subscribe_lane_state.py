import time
from typing import Optional

from main.ws_client import RuntimeWsClient
from main.chassis import CurvatureAdaptiveOuterLoop, WheelSmoother
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
    r_eff: float = 0.30,
    wheel_max_abs: float = 0.55,
    wheel_max_accel: float = 0.4,
    wheel_max_decel: float = 0.6,
) -> None:
    ws = RuntimeWsClient()
    try:
        ws.connect()
    except Exception as exc:
        print(f"[subscribe_lane_state] WebSocket 连接失败: {exc!r}")
        return

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
        f"[subscribe_lane_state] start hz={hz:.1f} via WebSocket  "
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
                payload = ws.realtime_lane_state() or {}
                state = LaneState.from_lane_state_payload(payload)
            except Exception as exc:
                print(f"[subscribe_lane_state] ws realtime_lane_state error: {exc!r}")
                state = LaneState()

            target_speeds = outer.step(state, dt)

            target_speeds = smoother.step(target_speeds)

            if not dry_run:
                ws.realtime_wheel_speeds(target_speeds)

            v1, v2, v3, v4 = target_speeds
            dbg = outer.debug_snapshot()
            print(
                f"ey={state.error_y!s:>10}  ea={state.error_angle!s:>10}  "
                f"kappa={dbg['kappa_ema']:.3f}  dkappa={dbg['dkappa_ema']:.3f}  "
                f"streak={dbg['straight_streak_ms']:>5.0f}ms  "
                f"v1={v1:>8.4f}  v2={v2:>8.4f}  v3={v3:>8.4f}  v4={v4:>8.4f}"
            )
    except KeyboardInterrupt:
        print("[subscribe_lane_state] KeyboardInterrupt -> exit")
    finally:

        if not dry_run:
            try:
                ws.realtime_wheel_speeds([0.0, 0.0, 0.0, 0.0])
            except Exception:
                pass
        ws.close()


if __name__ == "__main__":
    subscribe_lane_state(hz=50.0, max_seconds=70.0, dry_run=False)