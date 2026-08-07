"""main/chassis/controllers/move_along_lane.py
沿中心车道线方向**只前进/后退**（视觉对齐）的底盘方法。

控制律 = ``StraightOuterLoop(vx_cruise=vx, strafe_v=0.0)``：
  - **vy 通道被 ``strafe_v=0`` 锁死** → 物理上只有 vx 平移（vx>0 前进 / vx<0 后退），
    不会左右横移 / 侧滑。
  - **ω 通道照常**用 error_angle PI + error_y cross-track 让车头始终对齐车道
    中心线方向（车偏右 → 左转拉回，漂移归零 = 真平行）。
  - 其余节律 / 软化 / 兜底走 ``subscribe_lane_state``（LANE_FOLLOW profile）。

**两种退出模式**：
  - 时间模式（默认）：跑满 ``max_seconds`` 停。
  - 距离模式（``distance_m`` 设了）：沿车道累计行驶 ``distance_m`` 米后停
    （用 ``LaneState.distance``，即 SDK odometry 的路径长累加器，前进/后退都累计；
    车道跑偏时由 runner 的 watchdog / 丢线兜底急停）。

实现说明：``move_along_lane`` 是装配方法（不是控制律类），放在 controllers 目录
便于底盘组按"新写一个可调用方法"的路径落地；为避免 controllers ↔ 包 __init__
的循环 import，``subscribe_lane_state`` 用延迟 import（调用时才解析，届时包已
加载完毕——与 ``config/lane_follow.py`` 里 build_outer() 的延迟 import 同款）。
距离模式要能中途 stop runner，所以自建 DoubleLoopRunner（同 run_lane_follow 的写法），
``subscribe_lane_state`` 的公共参数语义保持一致。

**直接跑真车**（本文件自带 __main__ 入口）：

    # 需要 runtime 可达：export RAK_CAR_SERVER_ORIGIN=http://<Jetson IP>:5050
    python3 main/chassis/controllers/move_along_lane.py --vx 0.20 --seconds 5.0
    python3 main/chassis/controllers/move_along_lane.py --vx -0.15 --seconds 3.0   # 后退
    python3 main/chassis/controllers/move_along_lane.py --vx 0.20 --distance 2.0    # 前进 2 米
    python3 main/chassis/controllers/move_along_lane.py --dry-run                   # 只看方向不下发
    python3 main/chassis/controllers/move_along_lane.py --straight sign_theta=-1   # 方向反了
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Callable, Optional

# 路径: main/chassis/controllers/ → repo_root。直接 `python3 .../move_along_lane.py`
# 跑真车时 `main` 才找得到；作为包模块被 import 时这行无害（同 test 文件的 bootstrap）。
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from main.chassis.state import LaneState
from main.chassis.config import LANE_FOLLOW, LaneFollowProfile
from main.chassis.controllers.straight import StraightOuterLoop
from main.chassis.api import ChassisClient
from main.chassis.loops.closed_loop import DoubleLoopRunner
from main.chassis.loops.telemetry import lane_trace


def _make_distance_stop(distance_m: float, user_on_tick: Optional[Callable], holder: dict) -> Callable[[LaneState, list[float]], None]:
    """距离模式用的 on_tick：累计路径长到 ``distance_m`` 就调 ``holder["runner"].stop()``。

    ``LaneState.distance`` = SDK odometry 路径长累加器（单调不减，前进/后退都累计）。
    runtime auto-init 重置 odometry 时 distance 会回跳 → 检测到 d < start 就重新记账。
    先透传 ``user_on_tick``（用户自己的每帧回调），再判距离。
    """
    start = [None]

    def _tick(state: LaneState, wheels: list[float]) -> None:
        if user_on_tick is not None:
            try:
                user_on_tick(state, wheels)
            except Exception:
                pass
        d = state.distance
        if d is None:
            return
        if start[0] is None:
            start[0] = d
            return
        if d < start[0]:  # runtime 重置了 odometry → 重新记账
            start[0] = d
            return
        if d - start[0] >= abs(distance_m):
            runner = holder.get("runner")
            if runner is not None:
                runner.stop()

    return _tick


def move_along_lane(
    *,
    vx: float = 0.20,
    distance_m: Optional[float] = None,
    max_seconds: Optional[float] = None,
    profile: LaneFollowProfile = LANE_FOLLOW,
    hz: Optional[float] = None,
    dry_run: bool = False,
    with_trace: bool = False,
    on_tick: Optional[Callable[[LaneState, list[float]], None]] = None,
    stop_when: Optional[Callable[[LaneState, list[float]], bool]] = None,
    calibrator: Optional["ErrorCalibrator"] = None,
    straight: Optional[dict] = None,
) -> None:
    """沿中心车道线方向**只前进/后退**（视觉对齐）的一键方法。

    用法::

        from main.chassis.controllers import move_along_lane
        move_along_lane(vx=0.20, max_seconds=5.0)    # 沿车道中心线前进 5 秒
        move_along_lane(vx=-0.15, max_seconds=3.0)   # 沿车道中心线后退 3 秒
        move_along_lane(vx=0.20, distance_m=2.0)     # 沿车道中心线前进 2 米

    参数：
        vx          - 带符号前向速度 (m/s)：正=前进，负=后退。默认 0.20。
        distance_m  - 目标行驶距离 (m)。设了则以距离为准：累计路径长到该值停
                      （lane_state.distance，前进/后退都累计）。None = 纯时间模式。
        max_seconds - 运行时长上限 (s)。纯时间模式默认 5.0；距离模式下默认按
                      ``distance_m / |vx|`` 自动算 ~3 倍作为兜底，防止到不了距离干等。
        profile     - 节律 / 软化 / 兜底 profile，默认 LANE_FOLLOW。
        hz          - 循环频率，默认用 profile.hz。
        dry_run     - True 时只跑控制律不下发轮速。
        with_trace  - True 时每帧打印 lane 误差 + 轮速。默认 False（任务原语，
                      避免 50Hz 刷屏；排障时再开）。
        on_tick     - 覆盖 with_trace 的自定义回调。
        calibrator  - 误差标定层（lane 模型裸输出 → 物理量）。
        straight    - 透传 ``StraightOuterLoop`` 调参（kp_theta / sign_theta /
                      k_ey_omega / omega_max …），现场方向反了改这里。
                      ``vx_cruise`` 恒用 vx，``strafe_v`` 恒 0，不可覆盖。
    """
    straight_kwargs = dict(straight or {})
    straight_kwargs["vx_cruise"] = float(vx)
    straight_kwargs["strafe_v"] = 0.0  # 锁死 vy：物理上只有前进/后退
    outer = StraightOuterLoop(**straight_kwargs)

    if distance_m is None:
        # 纯时间模式：委托 subscribe_lane_state（原行为，max_seconds 默认 5.0）
        from main.chassis import subscribe_lane_state
        return subscribe_lane_state(
            outer=outer,
            max_seconds=5.0 if max_seconds is None else max_seconds,
            profile=profile,
            hz=hz,
            dry_run=dry_run,
            with_trace=with_trace,
            on_tick=on_tick,
            calibrator=calibrator,
        )

    # 距离模式：自建 runner，on_tick 累计路径长到 distance_m → runner.stop()
    api = ChassisClient.connect()
    effective_hz = profile.hz if hz is None else hz
    try:
        api.start_lane_feed(hz=effective_hz)
    except Exception:
        pass
    if on_tick is None and with_trace:
        on_tick = lane_trace(outer)
    holder: dict = {"runner": None}

    def combined_on_tick(state: LaneState, wheels: list[float]) -> None:
        if on_tick is not None:
            try:
                on_tick(state, wheels)
            except Exception:
                pass
        if stop_when is not None:
            try:
                if stop_when(state, wheels):
                    runner = holder.get("runner")
                    if runner is not None:
                        runner.stop()
            except Exception:
                pass

    distance_stop = _make_distance_stop(distance_m, None, holder)

    def distance_and_target_stop(state: LaneState, wheels: list[float]) -> None:
        distance_stop(state, wheels)
        combined_on_tick(state, wheels)

    runner = DoubleLoopRunner(
        api=api,
        outer=outer,
        hz=effective_hz,
        watchdog_ms=profile.watchdog_ms,
        lost_line_ms=profile.lost_line_ms,
        dry_run=dry_run,
        smoother=profile.build_smoother(),
        on_tick=distance_and_target_stop,
        calibrator=calibrator,
    )
    holder["runner"] = runner
    if max_seconds is None:
        # 兜底：按距离/速度算 ~3 倍名义时间，防 lane 视觉一直给"还没到"却干等
        max_seconds = abs(distance_m) / max(abs(vx), 0.05) * 3.0 + 2.0
    try:
        runner.run(max_seconds=float(max_seconds))
    finally:
        try:
            api.stop_lane_feed()
        except Exception:
            pass


def main(argv: list[str] | None = None) -> None:
    """真车实测 CLI：连 runtime → 开 lane_feed → 沿车道中心线前进/后退 → 停。

    前提：runtime 服务可达（``RAK_CAR_SERVER_ORIGIN=http://<Jetson IP>:5050``），
    lane_feed 在跑。默认带 3-2-1 倒计时，--dry-run 或 --no-countdown 可跳过。
    """
    parser = argparse.ArgumentParser(
        prog="main.chassis.controllers.move_along_lane",
        description="真车实测：沿中心车道线只前进/后退（vy 锁死 + ω 视觉对齐）。",
    )
    parser.add_argument("--vx", type=float, default=0.20,
                        help="带符号前向速度 (m/s)：正=前进，负=后退。默认 0.20。")
    parser.add_argument("--distance", type=float, default=None,
                        help="目标行驶距离 (m)。设了则以距离为准；默认 None = 纯时间模式。")
    parser.add_argument("--seconds", type=float, default=None,
                        help="运行时长上限 (s)。纯时间模式默认 5.0；--distance 模式下作为兜底"
                             "（默认按 distance/|vx| 自动算）。")
    parser.add_argument("--dry-run", action="store_true",
                        help="只跑控制律不下发轮速（看方向对不对）。")
    parser.add_argument("--no-countdown", action="store_true",
                        help="跳过 3-2-1 倒计时（配合 --dry-run 免等待）。")
    parser.add_argument("--straight", action="append", default=[], metavar="key=value",
                        help="透传 StraightOuterLoop 调参，如 sign_theta=-1 / kp_theta=2.0 / omega_max=0.3。")
    args = parser.parse_args(argv)

    straight: dict = {}
    for raw in args.straight:
        if "=" not in raw:
            raise SystemExit(f"--straight 必须是 key=value，实际: {raw!r}")
        k, v = raw.split("=", 1)
        try:
            straight[k.strip()] = float(v)
        except ValueError:
            raise SystemExit(f"--straight {k!r} 解析失败: {v!r}（应为数字）")

    if args.distance is None and args.seconds is None:
        args.seconds = 5.0  # 纯时间模式默认 5 秒
    mode = f"距离 {args.distance:+.2f} m" if args.distance is not None else f"时间 {args.seconds:.1f} s"
    print(f"[move_along_lane] vx={args.vx:+.2f} m/s · {mode} · "
          f"{'dry-run（不下发）' if args.dry_run else '真车下发'}"
          + (f" · straight={straight}" if straight else ""))
    if not args.dry_run and not args.no_countdown:
        for i in (3, 2, 1):
            print(f"  倒计时 {i}…")
            time.sleep(1)
        print("  GO！")
    move_along_lane(vx=args.vx, distance_m=args.distance, max_seconds=args.seconds,
                    dry_run=args.dry_run, straight=straight)
    print("[move_along_lane] 结束")


__all__ = ["move_along_lane"]


if __name__ == "__main__":
    main()
