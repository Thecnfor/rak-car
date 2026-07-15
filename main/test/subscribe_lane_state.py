"""main/test/subscribe_lane_state.py

外环巡线入口：通过 WebSocket realtime op 与车端 lane_feed 守护线程协作，
调用 ``main/chassis`` 工程代码里的 ``CurvatureAdaptiveOuterLoop`` 做弧度自适应控律。

数据流（双环不抢锁）：
  视觉 lane 误差  ─►  WS realtime/lane_state  ─►  LaneState dataclass
                                                   │
                                                   ▼
                              CurvatureAdaptiveOuterLoop.step(state, dt)
                                                   │
                                                   ▼
                          mecanum_inverse → [v1..v4]
                                                   │
                                                   ▼
                          WS realtime/wheel_speeds 直达车端 car_lock 路径
                          （不进 job_queue，与 lane_feed 共存）

行为：
  - 弧度偏差（|error_angle|）或弧度变化率（|d(error_angle)/dt|）越大，vx 越接近 v_min，
    转向增益越大；
  - 当 ``|error_y| < ey_release`` 且 ``|error_angle| < ea_release`` 连续 ``hold_ms`` 后，
    才把 vx 真正放回 v_max（防抖）。
  - ``dry_run=True`` 时只读视觉结果并打印，不下发任何轮速（车不运动）。
"""
import time
from typing import Optional

from main.ws_client import RuntimeWsClient
from main.chassis import CurvatureAdaptiveOuterLoop, WheelSmoother
from main.chassis.state import LaneState


def subscribe_lane_state(
    hz: float = 50.0,
    max_seconds: Optional[float] = 50.0,
    dry_run: bool = False,
    # --- 弧度自适应控制律参数（直接透传给 CurvatureAdaptiveOuterLoop） ---
    v_max: float = 0.30,
    v_min: float = 0.08,
    kappa_full: float = 0.6,
    dkappa_full: float = 1.5,
    # 横向 P 项降反馈（原 0.8 → 0.5，约 -37%），横向矫正在弯道/直线都同步变柔
    kp_y: float = 0.65,
    kp_theta: float = 1.2,
    omega_gain: float = 0.5,
    k_curvature: float = 0.4,
    omega_cap: float = 1.8,
    ey_release: float = 0.02,
    ea_release: float = 0.05,
    hold_ms: float = 250.0,
    r_eff: float = 0.30,
    # --- WheelSmoother 参数（最后一道闸，防止弯道瞬时单轮目标跳变拉爆电源） ---
    wheel_max_abs: float = 0.55,
    wheel_max_accel: float = 0.4,
    wheel_max_decel: float = 0.6,
) -> None:
    """外环：读 lane 误差 → 弧度自适应控律 → 麦轮逆解 → 下发 4 轮线速度。

    弧度偏差大幅变动 → 降速 + 加强转向；
    弧度偏差与行向偏差同时小到阈值并稳定 hold_ms 后才恢复 v_max。
    """
    # === 初始化：建立 WebSocket 长连接 ===
    ws = RuntimeWsClient()
    try:
        ws.connect()
    except Exception as exc:
        print(f"[subscribe_lane_state] WebSocket 连接失败: {exc!r}")
        return

    # === 初始化：弧度自适应控律（自带 EMA 平滑，不再需要外部 smoother） ===
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

    # === 初始化：轮速软化器（最后一道闸，防弯道瞬时跳变把下位机电源拉爆） ===
    smoother = WheelSmoother(
        max_abs=wheel_max_abs,
        max_accel=wheel_max_accel,
        max_decel=wheel_max_decel,
    )

    # === 初始化：定频调度参数 ===
    period = 1.0 / max(hz, 1e-3)
    deadline = None if max_seconds is None else time.monotonic() + max_seconds

    # === 主循环 ===
    next_t = time.monotonic()
    print(
        f"[subscribe_lane_state] start hz={hz:.1f} via WebSocket  "
        f"v_max={v_max:.2f} v_min={v_min:.2f} "
        f"kappa_full={kappa_full:.2f} hold_ms={hold_ms:.0f}"
    )
    try:
        while deadline is None or time.monotonic() < deadline:
            # --- 定频等待：落后超过 dt/2 就放弃补偿，避免 catch up ---
            now = time.monotonic()
            sleep = next_t - now
            if sleep > 0:
                time.sleep(sleep)
            next_t += period
            dt = period  # 外环节拍（与实际 wall-clock 漂移可忽略）

            # ========== 外环步骤 1：感知 ==========
            # 通过 WS realtime op（不进 job_queue，不打 ZMQ，不抢 car_lock）
            # 直接从 lane_feed 守护线程的缓存读取 error_y/error_angle
            try:
                payload = ws.realtime_lane_state() or {}
                state = LaneState.from_lane_state_payload(payload)
            except Exception as exc:
                print(f"[subscribe_lane_state] ws realtime_lane_state error: {exc!r}")
                state = LaneState()

            # ========== 外环步骤 2：弧度自适应控律 + 运动学 ==========
            target_speeds = outer.step(state, dt)

            # ========== 外环步骤 2.5：轮速软化（饱和 + slew rate 限幅） ==========
            # 控制律原始目标在急转弯瞬间会让单轮跳变（r*omega 项 + vy 翻转），
            # 直接下发给下位机会把电源拉爆。这里做单轮 |v| 饱和 + 单帧
            # 加速/减速上限，保证任何一帧的 4 轮速度都落在安全包络内。
            target_speeds = smoother.step(target_speeds)

            # ========== 内环：执行 ==========
            # 通过 WS realtime op（不走 job_queue，实时路径）将 4 轮目标线速度下发车端
            # 车端内环 PID 调速器接收目标值 → 闭环
            if not dry_run:
                ws.realtime_wheel_speeds(target_speeds)

            # ========== 调试打印 ==========
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
        # === 安全兜底：退出前零速停车 + 断开 WS ===
        if not dry_run:
            try:
                ws.realtime_wheel_speeds([0.0, 0.0, 0.0, 0.0])
            except Exception:
                pass
        ws.close()


if __name__ == "__main__":
    subscribe_lane_state(hz=50.0, max_seconds=70.0, dry_run=False)