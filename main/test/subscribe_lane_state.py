import time
from main.ws_client import RuntimeWsClient
from main.chassis.controllers.base import mecanum_inverse


class PIDSmoother:

    def __init__(self, alpha_y: float = 0.3, alpha_angle: float = 0.3):
        self.alpha_y = alpha_y
        self.alpha_angle = alpha_angle
        self._y = 0.0
        self._angle = 0.0
        self._initialized = False

    def step(self, error_y: float, error_angle: float) -> tuple[float, float]:
        if not self._initialized:
            self._y = error_y
            self._angle = error_angle
            self._initialized = True
        else:
            self._y = self.alpha_y * error_y + (1 - self.alpha_y) * self._y
            self._angle = (
                self.alpha_angle * error_angle + (1 - self.alpha_angle) * self._angle
            )
        return self._y, self._angle


def subscribe_lane_state(
    hz: float = 20.0,
    max_seconds: float | None = None,
    vx: float = 0.3,
    kp_y: float = 0.4,
    kp_theta: float = 2.0,
    r_eff: float = 0.30,
    alpha_y: float = 0.3,
    alpha_angle: float = 0.5,
    dry_run: bool = False,
) -> None:
    """外环：读 lane 误差 → PID 平滑 → P 控(vx,vy,ω) → 麦轮逆解 → 下发 4 轮线速度。

    外环（客户端 50Hz）：
      视觉 lane 误差 → PID 平滑 → P 控制律 → 麦轮逆运动学 → 4 轮目标线速度
    内环（车端已有 PID）：
      realtime_wheel_speeds WS 直达车端 car_lock 路径 → 车端 PID 调速器 → 电机
    双环不抢锁：外环只走 WS realtime op，不进 job_queue，与车端 lane_feed 共存。

    dry_run=True 时只读视觉结果并打印，不下发任何轮速（小车不运动）。
    """
    # === 初始化：建立 WebSocket 长连接 ===
    ws = RuntimeWsClient()
    try:
        ws.connect()
    except Exception as exc:
        print(f"[subscribe_lane_state] WebSocket 连接失败: {exc!r}")
        return

    # === 初始化：一阶低通滤波器（平滑原始误差，防止抖动导致轮速突变） ===
    smoother = PIDSmoother(alpha_y=alpha_y, alpha_angle=alpha_angle)

    # === 初始化：定频调度参数 ===
    period = 1.0 / max(hz, 1e-3)          # 每帧间隔（秒）
    deadline = None if max_seconds is None else time.monotonic() + max_seconds  # 结束时间

    # === 主循环：50Hz 外环 ===
    next_t = time.monotonic()
    print(f"[subscribe_lane_state] start hz={hz:.1f} via WebSocket")
    try:
        while deadline is None or time.monotonic() < deadline:
            # --- 定频等待：落后超过 dt/2 就放弃补偿，避免 catch up ---
            now = time.monotonic()
            sleep = next_t - now
            if sleep > 0:
                time.sleep(sleep)
            next_t += period

            # ========== 外环步骤 1：感知 ==========
            # 通过 WS realtime op（不进 job_queue，不打 ZMQ，不抢 car_lock）
            # 直接从 lane_feed 守护线程的缓存读取 error_y/error_angle
            try:
                lane_state = ws.realtime_lane_state()
                error_y = lane_state.get("error_y")
                error_angle = lane_state.get("error_angle")
            except Exception as exc:
                print(f"[subscribe_lane_state] ws realtime_lane_state error: {exc!r}")
                error_y = error_angle = None

            # ========== 外环步骤 2：平滑 ==========
            # 一阶低通滤波，alpha 越小平滑越强（响应越慢）
            if error_y is not None and error_angle is not None:
                smooth_y, smooth_angle = smoother.step(error_y, error_angle)
            else:
                smooth_y = smooth_angle = None

            # ========== 外环步骤 3：控制律 + 运动学 ==========
            # P 控制器：   vy = -kp_y * error_y      （横向修正）
            #              ω  = -kp_theta * error_angle（转向修正）
            # 麦轮逆解：   [v1,v2,v3,v4] = M * [vx, vy, ω]^T
            #              M = [[1, -1, -r], [-1, 1, -r], [-1, -1, -r], [1, 1, -r]]
            vy = 0.0
            omega = 0.0
            vx_eff = 0.0
            if smooth_y is not None and smooth_angle is not None:
                vx_eff = vx * max(0.35, 1.0 - abs(smooth_angle))
                vy = -kp_y * smooth_y
                omega = kp_theta * smooth_angle
                target_speeds = mecanum_inverse(vx_eff, vy, omega, r_eff)
            else:
                # 无有效误差 → 零速，不贸然前进
                target_speeds = [0.0, 0.0, 0.0, 0.0]

            # ========== 内环：执行 ==========
            # 通过 WS realtime op（不走 job_queue，实时路径）将 4 轮目标线速度下发车端
            # 车端内环 PID 调速器接收目标值 → 闭环
            if not dry_run:
                ws.realtime_wheel_speeds(target_speeds)

            # ========== 调试打印 ==========
            v1, v2, v3, v4 = target_speeds
            print(
                f"error_y={error_y!s:>10}  smooth_y={smooth_y!s:>10}  "
                f"error_angle={error_angle!s:>10}  smooth_angle={smooth_angle!s:>10}  "
                f"vx={vx_eff:.3f}  vy={vy!s:>8}  omega={omega!s:>8}  "
                f"v1={v1:>8.4f}  v2={v2:>8.4f}  v3={v3:>8.4f}  v4={v4:>8.4f}"
            )
    except KeyboardInterrupt:
        print("[subscribe_lane_state] KeyboardInterrupt -> exit")
    finally:
        # === 安全兜底：退出前零速停车 + 断开 WS ===
        if not dry_run:
            ws.realtime_wheel_speeds([0.0, 0.0, 0.0, 0.0])
        ws.close()


if __name__ == "__main__":
    subscribe_lane_state(hz=20.0, max_seconds=50.0, dry_run=False)
