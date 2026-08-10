"""main/chassis/cli/lane_only.py
仅巡线入口：直线（StraightOuterLoop）+ 弯道（CurveDetector 识别 → StaircaseTurn 阶梯转弯），
只调 controllers 里现成的控制律，无任务逻辑。

用法：
    python3 -m main.chassis.cli.lane_only                       

标定说明（两处里程计用法）：
  1. max-distance（停止里程计）：
     - 读到 odom_feed 的 distance（沿路累计里程，m），d >= 阈值即停。
     - 修改：CLI 传 `--max-distance 50`；改默认值改下方 parser 的 default=30.0；
       关掉：`--max-distance 0`。
  2. 转弯的里程计（theta 增量闭环）：
     - 转弯读 odom_feed 的 theta，只用增量（θ_start → θ_start+turn_deg），
       不依赖绝对 theta，所以不受实车 odom theta 整体漂移影响。
     - 转多少度：--maneuver 的 TURN（或 _DEFAULT_MANEUVERS），正=左/负=右。
     - 转到多准停：OdomTurnPID(tol_deg=...) 默认 2°（odom_turn.py），越大转得越糙越快。
     - 转反了：TURN 取负。
     - 力度：OdomTurnPID 的 kp/ki/kd/omega_max（odom_turn.py），弯道盲转由
       CurveDetector→StaircaseTurn 用同一 theta 增量（45→90→120 阶梯）。
"""
from __future__ import annotations

import argparse
import math
import threading
import time

from ..api import ChassisClient
from ..controllers.base import WheelSmoother
from ..controllers.odom_turn import CurveDetector, OdomTurnPID, StaircaseTurn
from ..controllers.straight import StraightOuterLoop
from ..loops.closed_loop import DoubleLoopRunner

# 默认脚本动作：(distance_m, turn_deg, straight_m) —— 到点停转弯逻辑 → 转 → 直行 → 恢复
_DEFAULT_MANEUVERS = ((2.0, -45.0, 1.0), (20.0, -135.0, 0.7))


def _parse_maneuvers(items: list[str]) -> list[tuple[float, float, float]]:
    """把 `--maneuver "dist:turn:straight"` 解成 (distance, turn_deg, straight_m) 列表。"""
    out: list[tuple[float, float, float]] = []
    for raw in items:
        parts = [p.strip() for p in raw.split(":")]
        if len(parts) != 3:
            raise SystemExit(f"--maneuver 格式必须 dist:turn:straight，实际 {raw!r}")
        try:
            out.append((float(parts[0]), float(parts[1]), float(parts[2])))
        except ValueError:
            raise SystemExit(f"--maneuver 数字解析失败: {raw!r}")
    return out


def _turn_by_odom(api: ChassisClient, turn_deg: float, dt: float = 0.02,
                  timeout: float = 10.0) -> None:
    """里程计 θ 增量闭环转 turn_deg（正=左转 / 负=右转），OdomTurnPID 控制。

    修改指引：
      - 转多少度：turn_deg —— 脚本动作里是 --maneuver 的中间位 / _DEFAULT_MANEUVERS。
      - 转到多准停：OdomTurnPID(tol_deg=...) 默认 2°，越大转得越糙越快。
      - 转反了：turn_deg 取负。
      - 角速度力度：OdomTurnPID 的 kp/ki/kd/omega_max（见 odom_turn.py）。
      - theta 来源：api.get_odometry_state().theta（rad），依赖 odom_feed 开着。
    """
    odom = api.get_odometry_state()
    if odom.theta is None:
        return
    turn = OdomTurnPID(turn_deg=turn_deg)
    turn.start(odom.theta)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        odom = api.get_odometry_state()
        if odom.theta is None:
            time.sleep(dt)
            continue
        omega, done = turn.step(odom.theta, dt)
        if done:
            api.set_wheel_speeds([0.0] * 4)
            return
        api.set_wheel_speeds(turn.wheels(omega))
        time.sleep(dt)
    api.set_wheel_speeds([0.0] * 4)


def _do_maneuver(api: ChassisClient, runner: DoubleLoopRunner, turn: StaircaseTurn,
                 detector: CurveDetector, turn_deg: float, straight_m: float) -> None:
    """distance 到点：停掉转弯逻辑 → 转 turn_deg → 直行 straight_m → 恢复转弯逻辑。"""
    runner.pause()
    runner.turn = None
    runner.detector = None
    try:
        # 若暂停恰好在转弯中途：清掉阶梯转弯/识别状态，恢复后从直道重新开始
        if turn.phase == "turning":
            turn.phase = "idle"
        detector.reset()
        _turn_by_odom(api, turn_deg)
        api.move_for(dx_m=straight_m)
    finally:
        runner.turn = turn
        runner.detector = detector
        runner.resume()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="main.chassis.cli.lane_only",
        description="仅巡线：StraightOuterLoop 直线 + CurveDetector/StaircaseTurn 转弯。",
    )
    parser.add_argument("--max-distance", type=float, default=30.0,
                        help="里程计 distance 达到此值 (m) 即停止；0/负=不启用一直跑")
    parser.add_argument("--hz", type=float, default=50.0, help="外环频率，默认 50")
    parser.add_argument("--dry-run", action="store_true", help="只跑控制律不下发轮速")
    parser.add_argument("--turn-tol-deg", type=float, default=20.0,
                        help="弯道识别阈值 (deg)：|error_angle| 持续超过才判弯；急弯降阈值")
    parser.add_argument("--turn-sustain", type=int, default=5,
                        help="弯道识别连续帧数（50Hz 下 5 帧=0.1s 去抖）")
    parser.add_argument("--sign-cal", type=int, default=1,
                        help="error_angle→转向 符号标定，实车转反了改 -1")
    parser.add_argument("--maneuver", action="append", default=None,
                        metavar="DIST:TURN:STRAIGHT",
                        help="脚本动作：到 DIST (m) 停转弯逻辑 → 转 TURN 度(正=左/负=右) → "
                             "直行 STRAIGHT (m) → 恢复。可重复传多个。"
                             "默认 2:-45:1.0 和 20:-135:0.7")
    parser.add_argument("--no-maneuver", action="store_true", help="关掉所有脚本动作")
    args = parser.parse_args(argv)

    maneuvers: list[tuple[float, float, float]] = (
        [] if args.no_maneuver
        else list(_DEFAULT_MANEUVERS) if args.maneuver is None
        else _parse_maneuvers(args.maneuver)
    )

    api = ChassisClient.connect()
    try:
        try:
            api.start_lane_feed(hz=args.hz)
        except Exception:
            pass
        detector = CurveDetector(tol_deg=args.turn_tol_deg,
                                 sustain=args.turn_sustain,
                                 sign_cal=args.sign_cal)
        turn = StaircaseTurn()
        runner = DoubleLoopRunner(
            api=api,
            outer=StraightOuterLoop(),
            smoother=WheelSmoother(),
            hz=args.hz,
            watchdog_ms=500.0,
            lost_line_ms=None,  # 笔直居中路段误差齐 0，不按丢线急停
            dry_run=args.dry_run,
            detector=detector,
            turn=turn,
        )
        # run() 阻塞 → 放后台线程，主线程按里程计 distance 提前收。
        # max_seconds=∞：无时间超时，停止只由里程计 / Ctrl+C 触发。
        t = threading.Thread(target=runner.run,
                             kwargs={"max_seconds": math.inf},
                             daemon=True)
        t.start()
        maneuver_idx = 0
        try:
            while t.is_alive():
                odom = api.get_odometry_state()
                d = odom.distance if odom.distance is not None else 0.0
                # 脚本动作列表（按 distance 递增执行）：到点停转弯逻辑 → 转+直行 → 恢复。
                # dry-run 下跳过（动作会真实下发轮速 + move_for，不属于"只跑控制律"）。
                if maneuver_idx < len(maneuvers):
                    m_dist, m_turn, m_straight = maneuvers[maneuver_idx]
                    if d >= m_dist:
                        maneuver_idx += 1
                        if args.dry_run:
                            print(f"dry-run：distance {d:.2f}m 已到 {m_dist}m，脚本动作跳过")
                        else:
                            print(f"distance {d:.2f}m >= {m_dist}m，"
                                  "停转弯逻辑，执行脚本动作")
                            _do_maneuver(api, runner, turn, detector,
                                         m_turn, m_straight)
                            print("脚本动作完成，恢复转弯逻辑")
                # max-distance 停止判定：odom distance（沿路累计里程 m）≥ 阈值即停。
                # 修改：CLI `--max-distance 50`；改默认改 parser default=30.0；关掉传 0。
                if args.max_distance and args.max_distance > 0 and d >= args.max_distance:
                    print(f"里程计 distance {d:.2f}m >= {args.max_distance}m，提前停止")
                    runner.stop()
                    break
                time.sleep(0.2)
            t.join()
        except KeyboardInterrupt:
            # 手动中断 → 零速收尾（daemon 线程随进程死掉不会补发零速）
            runner.stop()
    finally:
        try:
            api.stop_lane_feed()
        except Exception:
            pass


if __name__ == "__main__":
    main()
