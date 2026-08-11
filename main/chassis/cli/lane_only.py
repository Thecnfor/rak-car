"""main/chassis/cli/lane_only.py
仅巡线入口：直线（StraightOuterLoop）+ 弯道（CurveDetector 识别 → StaircaseTurn 阶梯转弯），
只调 controllers 里现成的控制律，无任务逻辑。

所有转弯配置从 lane_only.yml 读（--config 可换路径），代码不动：
  - maneuvers 段：到点停转弯逻辑 → 里程计 θ 盲转 → 直行 → 视觉对齐拉直
  - turn 段：弯道视觉转弯（CurveDetector 识别 + StaircaseTurn 阶梯 45→90→120）
  - crossroad_turn：第几个弯用加固转弯（里程碑窗口出口+冷却）

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
     - 转多少度：maneuvers 段的 TURN（或 _DEFAULT_MANEUVERS），正=左/负=右。
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
from pathlib import Path
from typing import Optional, Tuple

from ..api import ChassisClient
from ..controllers.base import WheelSmoother
from ..controllers.odom_turn import CurveDetector, OdomTurnPID, StaircaseTurn
from ..controllers.straight import StraightOuterLoop
from ..loops.closed_loop import DoubleLoopRunner

# 默认脚本动作（lane_only.yml 无 maneuvers 段时回退）：
# (distance_m, turn_deg, straight_m, align_m)
# —— 到点停转弯逻辑 → 转 → 直行 → 视觉对齐拉直 → 恢复
_DEFAULT_MANEUVERS = ((1.35, -45.0, 0.0, 0.4), (20.0, -120.0, 0.0, 0.4))

# 本文件 main/chassis/cli/lane_only.py → 仓库根目录
_ROOT = Path(__file__).resolve().parents[3]


def _load_yml(path: Path) -> Optional[dict]:
    """读 lane_only.yml → dict；文件缺失/解析失败/非 dict → None（回退代码默认）。"""
    if not path.is_file():
        print(f"⚠ lane_only.yml 不存在: {path}（回退代码默认配置）")
        return None
    try:
        import yaml
        cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception as exc:
        print(f"⚠ lane_only.yml 解析失败: {exc}（回退代码默认配置）")
        return None
    if not isinstance(cfg, dict):
        print(f"⚠ lane_only.yml 顶层不是 dict: {type(cfg).__name__}（回退代码默认配置）")
        return None
    return cfg


def _load_crossroad_turn(cfg: Optional[dict]) -> int:
    """从 lane_only.yml 读 crossroad_turn：第几个弯（1 起）用加固转弯；0=全部原版单帧判定。"""
    if cfg is None:
        return 0
    v = cfg.get("crossroad_turn")
    return int(v) if v is not None else 0


def _load_maneuvers(cfg: Optional[dict]) -> Optional[list]:
    """从 lane_only.yml 的 maneuvers 段 → [(distance_m, turn_deg, straight_m, align_m), ...]。

    返回 None = 段未配置（调用方回退 _DEFAULT_MANEUVERS）；
    返回 []  = 显式空列表（关掉脚本动作）；
    否则 list of tuples。straight_m / align_m 缺省 0。
    """
    if cfg is None or "maneuvers" not in cfg:
        return None
    items = cfg["maneuvers"]
    if not items:
        return []
    out: list[tuple[float, float, float, float]] = []
    for it in items:
        if not isinstance(it, dict) or "distance_m" not in it or "turn_deg" not in it:
            continue
        try:
            out.append((float(it["distance_m"]), float(it["turn_deg"]),
                        float(it.get("straight_m", 0.0)),
                        float(it.get("align_m", 0.0))))
        except (TypeError, ValueError):
            continue
    return out


def _build_turn(cfg: Optional[dict]) -> Tuple[CurveDetector, StaircaseTurn]:
    """从 lane_only.yml 的 turn: 段造 CurveDetector + StaircaseTurn（缺字段用类默认）。

    弯道视觉转弯参数全部从 yml 读：detector 控制识别，staircase 控制阶梯转弯。
    缺 turn 段 / 读失败 → 两个类各自默认（等价旧 `CurveDetector()` + `StaircaseTurn()`）。
    """
    det_kw: dict = {}
    stair_kw: dict = {}
    if cfg is not None:
        turn = cfg.get("turn")
        if isinstance(turn, dict):
            det_kw = turn.get("detector") or {}
            stair_kw = turn.get("staircase") or {}
    return CurveDetector(**det_kw), StaircaseTurn(**stair_kw)


def _parse_maneuvers(items: list[str]) -> list[tuple[float, float, float, float]]:
    """把 `--maneuver "dist:turn:straight[:align]"` 解成 (distance, turn_deg, straight_m, align_m) 列表。"""
    out: list[tuple[float, float, float, float]] = []
    for raw in items:
        parts = [p.strip() for p in raw.split(":")]
        if len(parts) not in (3, 4):
            raise SystemExit(f"--maneuver 格式必须 dist:turn:straight[:align]，实际 {raw!r}")
        try:
            align = float(parts[3]) if len(parts) == 4 else 0.0
            out.append((float(parts[0]), float(parts[1]), float(parts[2]), align))
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


def _align_straight(api: ChassisClient, distance_m: float, vx: float = 0.20,
                    hz: float = 50.0, timeout: float = 20.0) -> None:
    """视觉对齐拉直：vy 锁死 + ω 视觉对齐，沿车道中心线前进 distance_m 米。

    控制律 = StraightOuterLoop(vx_cruise=vx, strafe_v=0)：vy 锁死不横移，
    ω 用 error_angle 拉回 → 车头对齐车道并保持平行（真"拉直"）。
    这阶段 detector 还没恢复，不会把转完后的偏头误判成弯。

    复用主 api（不动 lane_feed / 不 close），距离用 lane_state.distance
    （沿路累计，前进/后退都累计）。跑到 distance_m、3s 无前进进度（丢线/卡住）
    或 timeout 即停。
    """
    outer = StraightOuterLoop(vx_cruise=vx, strafe_v=0.0)
    smoother = WheelSmoother()
    smoother.reset([0.0, 0.0, 0.0, 0.0])
    start_d = None
    last_progress = time.monotonic()
    dt = 1.0 / hz
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = api.read_lane()
        wheels = smoother.step(outer.step(state, dt))
        api.set_wheel_speeds(wheels)
        d = state.distance
        if d is not None:
            if start_d is None:
                start_d = d
            elif d < start_d:
                start_d = d  # runtime 重置 odometry → 重新记账
            elif d - start_d >= distance_m:
                break
            if d - start_d > 0.001:
                last_progress = time.monotonic()
        if time.monotonic() - last_progress > 3.0:
            break
        time.sleep(dt)
    api.set_wheel_speeds([0.0, 0.0, 0.0, 0.0])


def _do_maneuver(api: ChassisClient, runner: DoubleLoopRunner,
                 turn_deg: float, straight_m: float, align_m: float) -> None:
    """distance 到点：停掉转弯逻辑 → 转 turn_deg → 直行 straight_m → 视觉对齐拉直 align_m → 恢复。"""
    runner.pause()
    saved_turn, saved_detector = runner.turn, runner.detector
    runner.turn = None
    runner.detector = None
    try:
        # 若暂停恰好在转弯中途：清掉阶梯转弯/识别状态，恢复后从直道重新开始。
        # 用当前活跃对（可能是加固的 crossroad 对），别用模块级默认对。
        if saved_turn is not None and saved_turn.phase == "turning":
            saved_turn.phase = "idle"
        if saved_detector is not None:
            saved_detector.reset()
        _turn_by_odom(api, turn_deg)
        if straight_m and straight_m > 0:
            api.move_for(dx_m=straight_m)
        if align_m and align_m > 0:
            _align_straight(api, align_m)
    finally:
        runner.turn = saved_turn
        runner.detector = saved_detector
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
    parser.add_argument("--maneuver", action="append", default=None,
                        metavar="DIST:TURN:STRAIGHT[:ALIGN]",
                        help="脚本动作：到 DIST (m) 停转弯逻辑 → 转 TURN 度(正=左/负=右) → "
                             "直行 STRAIGHT (m) → 视觉对齐拉直 ALIGN (m，可选) → 恢复。"
                             "可重复传多个。"
                             "默认 1.35:-45:0:0.4 和 20:-120:0:0.4")
    parser.add_argument("--no-maneuver", action="store_true", help="关掉所有脚本动作")
    parser.add_argument("--config", type=str, default=str(_ROOT / "lane_only.yml"),
                        help="lane_only.yml 路径（maneuvers / turn / crossroad_turn 三段）；"
                             "默认仓库根目录 lane_only.yml")
    args = parser.parse_args(argv)

    # 所有转弯配置从 lane_only.yml 读：脚本动作 maneuvers、弯道视觉转弯 turn、
    # 加固转弯 crossroad_turn。CLI 只留 --maneuver 临时覆盖 / --no-maneuver 关掉。
    cfg = _load_yml(Path(args.config))
    yml_maneuvers = _load_maneuvers(cfg)
    maneuvers: list[tuple[float, float, float, float]] = (
        [] if args.no_maneuver
        else _parse_maneuvers(args.maneuver) if args.maneuver is not None
        else yml_maneuvers if yml_maneuvers is not None
        else list(_DEFAULT_MANEUVERS)
    )
    detector, turn = _build_turn(cfg)
    crossroad_turn = _load_crossroad_turn(cfg)

    api = ChassisClient.connect()
    try:
        # 每次运行里程计清零，避免上次运行残留 distance/theta 污染本次判定。
        # 必须用 execute(sync=True) 等到 job 真正执行完 —— call() 会把 sync=True
        # 吞进 action kwargs 走异步，主循环可能先读到上次残留的 distance。
        try:
            job = api.http.execute("car", "reset_position", sync=True, timeout=10.0)
            if job is None or job.get("status") != "succeeded":
                print(f"⚠ reset_position 未成功: {job}（里程计可能残留，继续）")
            else:
                print("里程计已清零 (reset_position)")
        except Exception as exc:
            print(f"reset_position 失败（忽略，继续）: {exc}")
        try:
            api.start_lane_feed(hz=args.hz)
        except Exception:
            pass
        if crossroad_turn:
            print(f"crossroad_turn={crossroad_turn}：第 {crossroad_turn} 个弯用加固转弯")
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
            crossroad_turn=crossroad_turn,
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
                    m_dist, m_turn, m_straight, m_align = maneuvers[maneuver_idx]
                    if d >= m_dist:
                        maneuver_idx += 1
                        if args.dry_run:
                            print(f"dry-run：distance {d:.2f}m 已到 {m_dist}m，脚本动作跳过")
                        else:
                            print(f"distance {d:.2f}m >= {m_dist}m，"
                                  "停转弯逻辑，执行脚本动作")
                            _do_maneuver(api, runner, m_turn, m_straight, m_align)
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
