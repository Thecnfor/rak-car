"""main/chassis/cli/run_lane_follow.py
巡线外环的命令行入口。

用法：
    python3 -m main.chassis.cli.run_lane_follow
    python3 -m main.chassis.cli.run_lane_follow --dry-run --max-seconds 5
    python3 -m main.chassis.cli.run_lane_follow --profile slow --tune hz=30 --tune wheel_max_abs=0.5
    python3 -m main.chassis.cli.run_lane_follow --no-trace --controller stanley
"""
from __future__ import annotations

import argparse
import inspect
from dataclasses import fields

from ..api import ChassisClient
from ..config.lane_follow import (
    LANE_FOLLOW,
    LANE_FOLLOW_SLOW,
    ControllerType,
    LaneFollowProfile,
)
from ..controllers.calibration import ErrorCalibrator
from ..loops.closed_loop import DoubleLoopRunner
from ..loops.telemetry import lane_trace


# 内置 profile 列表（#3）：CLI 与 subscribe_lane_state 共用同一份装配逻辑
_PROFILE_CHOICES = {
    "default": LANE_FOLLOW,
    "slow": LANE_FOLLOW_SLOW,
}


def _parse_kv_pairs(items: list[str]) -> dict:
    """把 `--tune key=value` 解成 dict。"""
    out: dict = {}
    for raw in items:
        if "=" not in raw:
            raise SystemExit(f"--tune 参数必须是 key=value，实际: {raw!r}")
        k, v = raw.split("=", 1)
        k = k.strip()
        try:
            out[k] = float(v)
        except ValueError:
            raise SystemExit(f"--tune 字段 {k!r} 解析失败: {v!r}（应为数字）")
    return out


def _build_profile(args: argparse.Namespace) -> LaneFollowProfile:
    """根据 CLI 参数选 profile，再 apply --tune overrides。

    --controller 切换控制律（#6），--tune 仍然按字段名直接覆盖 profile 字段
    —— 比 inspect.signature 拆 outer/smoother 更简单，且不会因 outer 字段重命名
    而失效。
    """
    base = _PROFILE_CHOICES[args.profile]

    # 控制律切换：单独字段，不走 --tune（避免 enum 字符串解析歧义）
    if args.controller:
        base = base.tuned(controller_type=ControllerType(args.controller))

    # --tune 走 dataclasses.replace（frozen dataclass），只认 profile 自有字段
    tune = _parse_kv_pairs(args.tune)
    if tune:
        _valid = {f.name for f in fields(LaneFollowProfile)}
        _bad = set(tune) - _valid
        if _bad:
            raise SystemExit(
                f"--tune 字段不存在: {sorted(_bad)}（合法: {sorted(_valid)}）\n"
                "控制器增益（v_max/kp_y…）不在 profile，走构造 CurvatureAdaptiveOuterLoop"
            )
        base = base.tuned(**tune)
    return base


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="main.chassis.cli.run_lane_follow",
        description="跑一行底盘巡线外环，参数走 LaneFollowProfile。",
    )
    parser.add_argument(
        "--profile",
        choices=sorted(_PROFILE_CHOICES.keys()),
        default="default",
        help="内置 profile：default（实车）/ slow（dry-run 看数）",
    )
    parser.add_argument(
        "--controller",
        choices=[c.value for c in ControllerType],
        default=None,
        help="覆盖 profile.controller_type（straight / curvature_adaptive / stanley / p / orthogonal）",
    )
    parser.add_argument("--dry-run", action="store_true", help="只跑控制律不下发轮速")
    parser.add_argument("--no-trace", action="store_true", help="关掉每帧打印")
    parser.add_argument("--hz", type=float, default=None, help="循环频率，默认用 profile.hz")
    parser.add_argument("--max-seconds", type=float, default=None, help="最大运行时间，默认用 profile.max_seconds")
    parser.add_argument("--watchdog-ms", type=float, default=None, help="数据过期急停阈值 ms")
    parser.add_argument("--lost-line-ms", type=float, default=None, help="丢线检测阈值 ms")
    parser.add_argument(
        "--tune",
        action="append",
        default=[],
        metavar="key=value",
        help="覆盖任意 profile 字段（直接走 dataclasses.replace）",
    )
    parser.add_argument(
        "--vx-target",
        type=float,
        default=None,
        help="正交模式的前向速度（m/s）。默认 None=保留 outer 默认值：\n"
             "  * orthogonal 默认 0.0（原地水平稳定，只横移+旋转修正 d_e/d_a）\n"
             "  * 传正值切到正交巡航（例如 --vx-target 0.25），\n"
             "    相当于把 vx 通道也独立打开，三个自由度各管各的",
    )
    parser.add_argument(
        "--tui",
        action="store_true",
        help="打开 TUI 常驻面板（rich.Live，全屏实时刷新 + 单键快捷键）。\n"
             "与 --no-trace 互斥（--tui 时 lane_trace 不启动）。\n"
             "快捷键: r/z 清零积分, p 切换 dry-run, s 急停零速, q 退出",
    )
    parser.add_argument(
        "--error-scale-y", type=float, default=None,
        help="error_y 标定倍率（默认 None=不标定）。lane 模型裸输出单位未知，\n"
             "先跑 diag_lane_error 实测分布再定；若 d_e 恒为 0.000x 而实车明显偏离，\n"
             "典型倍率是 1e3~1e4。"
    )
    parser.add_argument(
        "--error-scale-angle", type=float, default=None,
        help="error_angle 标定倍率（默认 None=不标定）。通常模型输出已是角度量纲，\n"
             "多半不需要动；若 d_a 也偏小再调。"
    )
    parser.add_argument(
        "--error-offset-y", type=float, default=0.0,
        help="error_y 零点偏移（视觉零漂，米/模型尺度）。默认 0；\n"
             "若 profile.error_offset_y 已标好，这里留 0 即用 profile 值。"
    )
    parser.add_argument(
        "--error-ema-alpha", type=float, default=None,
        help="误差 EMA 平滑系数 (0,1]，None=不平滑。越小越信任历史、越稳；\n"
             "太大等于没滤。建议 0.2~0.5。"
    )
    parser.add_argument(
        "--straight-follow", action="store_true",
        help="切到直道逻辑：vx 定速巡航 + vy 横移回正正交合成（十字正交分解，\n"
             "odom theta 保持 0）。弯道里程计 90° 转弯后续接回。\n"
             "默认关 = 走原 50Hz 速度环。"
    )
    parser.add_argument(
        "--vx-cruise", type=float, default=0.25,
        help="直道巡航前向速度 (m/s)，默认 0.25。"
    )
    parser.add_argument(
        "--deadband-y", type=float, default=0.3,
        help="y 回正死区 (m，标定后)。|error_y| 超过才启动 vy 横移，默认 0.3。"
    )
    parser.add_argument(
        "--kp-y", type=float, default=0.2,
        help="vy 横移通道比例增益 (m/s 每米误差)。横移回正与 vx 巡航正交合成。"
    )
    parser.add_argument(
        "--kd-y", type=float, default=0.2,
        help="vy 横移通道阻尼增益。误差快速归零（要冲过头）时压掉部分 vy，\n"
             "抑制回正过头 → 回正后重新偏移的震荡。0 关闭。默认 0.2。"
    )
    parser.add_argument(
        "--sign-y", type=float, default=1.0,
        help="y 回正方向，+1 = error_y>0(车在线右) 左移回中；实车反了改 -1。"
    )
    parser.add_argument(
        "--strafe-v", type=float, default=0.05,
        help="|vy| 横移速度上限 (m/s)，默认 0.05。"
    )
    parser.add_argument(
        "--kp-theta", type=float, default=1.5,
        help="ω 视觉航向通道比例增益 (rad/s 每弧度航向误差)，默认 1.5。"
    )
    parser.add_argument(
        "--ki-theta", type=float, default=0.15,
        help="ω 航向通道积分增益，消稳态航向偏差（直道右偏就是稳态偏差），默认 0.15。"
    )
    parser.add_argument(
        "--omega-max", type=float, default=0.25,
        help="|ω| 旋转速度上限 (rad/s)，默认 0.25（直道巡航比 orthogonal 小）。"
    )
    parser.add_argument(
        "--sign-theta", type=float, default=1.0,
        help="ω 方向，+1 = error_angle>0(车头偏右) 逆时针左转回正；实车反了改 -1。"
    )
    parser.add_argument(
        "--ea-target", type=float, default=None,
        help="可选额外 ω 收敛偏置 (rad)。默认 None=用控制器默认 0.0——配合\n"
             "--k-ey-omega cross-track 让车头收敛到与实际车道中心线平行（真平行由\n"
             "横向漂移反推，不靠这个固定偏置）。仅当你仍想要固定右偏时才设正偏置。"
    )
    parser.add_argument(
        "--k-ey-omega", type=float, default=None,
        help="error_y → ω 的 cross-track 增益 (rad/s 每单位 error_y)。\n"
             "error_angle 读 0 的零区内角度通道是瞎的，靠横向偏移反推真实平行方向：\n"
             "车头不平行 → 横向漂移 → error_y 变化 → 纠正到漂移归零 = 真平行。\n"
             "默认 None=用 StraightOuterLoop 的默认 0.5。需按 error_y 标定尺度调。"
    )
    parser.add_argument(
        "--turn", action="store_true",
        help="接弯道阶梯转弯：CurveDetector(|error_angle|>20° 持续 5 帧)识别弯道 →\n"
             "StaircaseTurn θ 闭环 45→90→120° 连续转，lane 回正后交还直道巡航。\n"
             "转弯底层在 controllers/odom_turn.py。"
    )
    args = parser.parse_args(argv)

    profile = _build_profile(args)
    effective_hz = profile.hz if args.hz is None else args.hz
    effective_max_seconds = profile.max_seconds if args.max_seconds is None else args.max_seconds
    effective_watchdog = profile.watchdog_ms if args.watchdog_ms is None else args.watchdog_ms
    effective_lost_line = profile.lost_line_ms if args.lost_line_ms is None else args.lost_line_ms

    api = ChassisClient.connect()

    try:
        api.start_lane_feed(hz=effective_hz)
    except Exception:
        pass

    outer = profile.build_outer()
    # 直道逻辑：StraightOuterLoop 自己就是控制律（vx 巡航 + vy 横移正交回正），
    # 复走 DoubleLoopRunner 的软化 / 标定 / 暂停流程。
    if args.straight_follow:
        from ..controllers.straight import StraightOuterLoop

        outer = StraightOuterLoop(
            vx_cruise=args.vx_cruise,
            deadband_y=args.deadband_y,
            kp_y=args.kp_y,
            kd_y=args.kd_y,
            sign_y=args.sign_y,
            strafe_v=args.strafe_v,
            kp_theta=args.kp_theta,
            ki_theta=args.ki_theta,
            omega_max=args.omega_max,
            sign_theta=args.sign_theta,
        )
    # --ea-target / --k-ey-omega：覆盖 ω 通道收敛目标 / cross-track 增益。
    # 两个路径（--controller straight 走 build_outer / --straight-follow 手写）统一在这里覆盖。
    if args.ea_target is not None and hasattr(outer, "ea_target"):
        outer.ea_target = float(args.ea_target)
    if args.k_ey_omega is not None and hasattr(outer, "k_ey_omega"):
        outer.k_ey_omega = float(args.k_ey_omega)
    # --vx-target 只作用在有 vx_target 字段的控制器（OrthogonalOuterLoop 等），
    # 用来从"原地水平稳定（vx=0）"切到"正交巡航"。其他 outer 没有这个字段
    # 也没关系，跳过即可。
    if args.vx_target is not None and hasattr(outer, "vx_target"):
        outer.vx_target = float(args.vx_target)
        if hasattr(outer, "locked_vx"):
            outer.locked_vx = (outer.vx_target == 0.0)
    smoother = profile.build_smoother()

    # 误差标定层：只有显式传了标定参数才启用，否则 None → 完全 no-op。
    cal_kwargs: dict = {}
    if args.error_scale_y is not None:
        cal_kwargs["scale_y"] = args.error_scale_y
    if args.error_scale_angle is not None:
        cal_kwargs["scale_angle"] = args.error_scale_angle
    if args.error_offset_y != 0.0 or profile.error_offset_y != 0.0:
        # 优先 CLI 显式传的 offset；没传（==0 哨兵）且 profile 标过 → 用 profile 值
        cal_kwargs["offset_y"] = args.error_offset_y if args.error_offset_y != 0.0 else profile.error_offset_y
    if args.error_ema_alpha is not None:
        cal_kwargs["ema_alpha"] = args.error_ema_alpha
    calibrator = ErrorCalibrator(**cal_kwargs) if cal_kwargs else None

    # 弯道阶梯转弯：detector 识别 → turn 接管（纯旋转）→ 回正交还 outer
    turn = detector = None
    if args.turn:
        from ..controllers.odom_turn import CurveDetector, StaircaseTurn

        detector = CurveDetector()
        turn = StaircaseTurn()

    use_tui = bool(args.tui) and not args.straight_follow
    if use_tui:
        # rich 是可选依赖，只在 --tui 时才 import；普通巡线不需要装 rich。
        from ..loops.tui import lane_tui
        # --tui 时自动抑制 --no-trace，不做滚动打印
        on_tick_factory = None  # 先占位，with 块里拿 runner 后再绑定
        runner_on_tick = None
    else:
        runner_on_tick = None if args.no_trace else lane_trace(outer)

    _runner_kwargs = dict(
        api=api,
        outer=outer,
        hz=effective_hz,
        watchdog_ms=effective_watchdog,
        lost_line_ms=effective_lost_line,
        dry_run=args.dry_run,
        smoother=smoother,
        on_tick=None if use_tui else runner_on_tick,
        calibrator=calibrator,
        turn=turn,
        detector=detector,
    )
    runner = DoubleLoopRunner(**_runner_kwargs)
    try:
        if use_tui:
            with lane_tui(outer, title=f"底盘正交寻路 · {profile.controller_type.value}") as make_cb:
                on_tick = make_cb(runner)
                runner.on_tick = on_tick
                runner.run(max_seconds=effective_max_seconds)
        else:
            runner.run(max_seconds=effective_max_seconds)
    finally:
        try:
            api.stop_lane_feed()
        except Exception:
            pass


if __name__ == "__main__":
    main()