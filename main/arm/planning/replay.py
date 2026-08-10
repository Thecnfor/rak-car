"""main/arm/planning/replay.py —— 连续轨迹回放（真机 / fake 通用）。

把 `JointTrajectory` 按**真实流逝时间**实时采样，喂给 `/v1/realtime/arm-velocity`：
  - x_vel / y_vel：轨迹有限差分得到的滑台速度 (m/s)，连续驱动，不排队；
  - arm_angle / hand_angle：每个采样点的目标角度（舵机异步滑到）；
  - **结束 / 异常都在 finally 发 0 速度 + 末姿态**，防滑台失速。

实时采样（区别于"按节拍 sleep"）：
  用 `time.monotonic() - t0` 作轨迹时间轴，速度 = 相邻采样位移 / 真实流逝时间。
  这样即使每次 HTTP+串口 post 较慢，滑台也在正确的时间点拿到正确速度，
  不会因节拍堆积而漂移。

位置反馈（防漂移，真机建议开启）：
  速度模式无位置闭环，滑台 stiction / 速度标定偏差会累积误差。`kp_position>0`
  时每 ~5 帧读一次 `/v1/realtime/arm/state` 实际滑台位置，加比例修正项
  `v += kp * (x_planned - x_actual)`，把轨迹拉回路径。
"""
from __future__ import annotations

import time
from typing import Callable, Optional

from main.arm.planning.joint_trajectory import JointTrajectory


def replay_trajectory(traj: JointTrajectory, client, *,
                      hz: float = 50.0,
                      kp_position: float = 0.0,
                      on_sample: Optional[Callable] = None) -> float:
    """按真实时间连续回放轨迹，返回耗时 (s)。

    client: 有 `.post(path, payload)`（且可选 `.get`）的任意 client。
    kp_position: >0 开启滑台位置比例修正（真机建议 0.05~0.2，fake 可 0）。
    on_sample(t, pose, vx_mm_s, vy_mm_s): 每帧回调。
    """
    post_holder = getattr(client, "http", client)   # ArmClient.http 或裸 client
    post = post_holder.post
    get = getattr(post_holder, "get", None)
    tick = 1.0 / max(float(hz), 1.0)
    T = traj.total_time
    t0 = time.monotonic()
    prev = traj.sample(0.0)
    prev_t = 0.0
    n = 0
    try:
        while True:
            t = time.monotonic() - t0
            if t >= T:
                break
            pose = traj.sample(t)
            # 速度 = 相邻采样位移 / 真实流逝时间（mm/s）
            dt_real = t - prev_t
            if dt_real > 1e-6:
                vx = (pose.x_mm - prev.x_mm) / dt_real
                vy = (pose.y_mm - prev.y_mm) / dt_real
            else:
                vx = vy = 0.0
            prev, prev_t = pose, t
            # 位置比例修正（真机滑台反馈，防漂移）
            if kp_position > 0.0 and get is not None and n % 5 == 0:
                try:
                    st = get("/v1/realtime/arm/state") or {}
                    ax = (st.get("arm_state") or {}).get("x_mm")
                    ay = (st.get("arm_state") or {}).get("y_mm")
                    if ax is not None:
                        vx += kp_position * (pose.x_mm - float(ax))
                    if ay is not None:
                        vy += kp_position * (pose.y_mm - float(ay))
                except Exception:
                    pass
            post("/v1/realtime/arm-velocity", {
                "x_vel": vx / 1000.0,             # m/s
                "y_vel": vy / 1000.0,
                "arm_angle": pose.arm_deg,
                "hand_angle": pose.hand_deg,
            })
            if on_sample is not None:
                on_sample(t, pose, vx, vy)
            n += 1
            time.sleep(tick)
    finally:
        end = traj.sample(T)
        post("/v1/realtime/arm-velocity", {
            "x_vel": 0.0, "y_vel": 0.0,
            "arm_angle": end.arm_deg, "hand_angle": end.hand_deg,
        })
    return time.monotonic() - t0
