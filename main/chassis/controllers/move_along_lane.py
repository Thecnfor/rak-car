"""main/chassis/controllers/move_along_lane.py
沿中心车道线方向**只前进/后退**（视觉对齐）的底盘方法。

2026-08-11 下沉：循线闭环从网络外环（DoubleLoopRunner + ChassisClient 每帧
WS 收 lane_state + HTTP 下发轮速）改为 **runtime 进程内**闭环 —— 直接调
``MyCar.lane_dis_offset`` / ``MyCar.lane_time``（官方极简法，与
``baidu_smartcar_2026/task/lane.py::auto_lane_tracing`` 同款：
读 get_lane_results + lane_pid + set_velocity，不经过网络）。本函数只
POST 一次 ``/v1/execute`` 同步等结果，不再每帧网络往返。

**两种退出模式**（由 runtime 底层闭环负责）：
  - 时间模式（默认）：跑满 ``max_seconds`` 停（``lane_time``）。
  - 距离模式（``distance_m`` 设了）：沿车道累计行驶 ``distance_m`` 米后停
    （``lane_dis_offset``，SDK odometry 里程计）。

**直接跑真车**（本文件自带 __main__ 入口）：

    # 需要 runtime 可达：export RAK_CAR_SERVER_ORIGIN=http://<Jetson IP>:5050
    python3 main/chassis/controllers/move_along_lane.py --vx 0.20 --seconds 5.0
    python3 main/chassis/controllers/move_along_lane.py --vx -0.15 --seconds 3.0   # 后退
    python3 main/chassis/controllers/move_along_lane.py --vx 0.20 --distance 2.0    # 前进 2 米
    python3 main/chassis/controllers/move_along_lane.py --dry-run                   # 只看方向不下发
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

# 路径: main/chassis/controllers/ → repo_root。直接 `python3 .../move_along_lane.py`
# 跑真车时 `main` 才找得到；作为包模块被 import 时这行无害（同 test 文件的 bootstrap）。
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from main.chassis.config import LANE_FOLLOW, LaneFollowProfile
from main.chassis.api import ChassisClient


def move_along_lane(
    *,
    vx: float = 0.20,
    distance_m: Optional[float] = None,
    max_seconds: Optional[float] = None,
    profile: LaneFollowProfile = LANE_FOLLOW,
    hz: Optional[float] = None,
    dry_run: bool = False,
    with_trace: bool = False,
    on_tick=None,
    stop_when=None,
    calibrator=None,
    straight: Optional[dict] = None,
) -> None:
    """沿中心车道线方向**只前进/后退**（视觉对齐）的一键方法。

    2026-08-11 下沉：循线闭环改在 **runtime 进程内** 跑（官方极简法
    ``MyCar.lane_dis_offset`` / ``MyCar.lane_time``，每帧零网络往返）。
    本函数只 POST 一次 ``/v1/execute`` 同步等结果。

    用法::

        from main.chassis.controllers import move_along_lane
        move_along_lane(vx=0.20, max_seconds=5.0)    # 沿车道中心线前进 5 秒
        move_along_lane(vx=-0.15, max_seconds=3.0)   # 沿车道中心线后退 3 秒
        move_along_lane(vx=0.20, distance_m=2.0)     # 沿车道中心线前进 2 米

    参数：
        vx          - 带符号前向速度 (m/s)：正=前进，负=后退。默认 0.20。
        distance_m  - 目标行驶距离 (m)。设了则以距离为准（runtime 里程计累计到该值停）。
                      None = 纯时间模式。
        max_seconds - 运行时长上限 (s)。纯时间模式默认 5.0；距离模式下忽略（距离优先）。
        profile / hz / with_trace / on_tick / stop_when / calibrator / straight
                    - 网络外环（DoubleLoopRunner）时代的调参 / 每帧钩子。
                      下沉后闭环在 runtime 进程内，这些不再参与，仅保留签名兼容。
    """
    api = ChassisClient.connect()
    if dry_run:
        return
    if distance_m is not None:
        # 距离模式 → runtime 进程内 lane_dis_offset(speed, dis_hold)，走到距离停。
        secs = abs(float(distance_m)) / max(abs(float(vx)), 0.05) * 3.0 + 2.0
        job = api.http.execute_car_action(
            "lane_dis_offset", float(vx), abs(float(distance_m)),
            timeout=secs + 5.0, sync=True,
        )
    else:
        # 时间模式 → runtime 进程内 lane_time(speed, time_dur)，到时间停。
        secs = 5.0 if max_seconds is None else float(max_seconds)
        job = api.http.execute_car_action(
            "lane_time", float(vx), secs,
            timeout=secs + 5.0, sync=True,
        )
    status = (job or {}).get("status")
    if status != "succeeded":
        err = (job or {}).get("error") or (job or {}).get("result")
        raise RuntimeError(f"move_along_lane 失败 (status={status}): {err}")


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
