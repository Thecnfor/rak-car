"""main/chassis/cli/diag_lane_error.py
离线诊断：lane 模型裸输出的误差尺度到底是多少？

背景：lane 模型把 ``result[0]/result[1]`` 原样当 error_y/error_angle 喂给控制律，
但控制律假设 error_y 是"米"。如果模型输出的量级/单位不是米（归一化 / 像素 /
1e-3 小值），则 vy 通道基本是死的，车只能靠 ω 修 —— 表现为"出弯后修正慢"。

本工具只做测量不下发：连 runtime → 读 lane_state + odom，统计 error_y /
error_angle 的原始分布，并在结尾给出标定倍率建议。

用法：
    python3 -m main.chassis.cli.diag_lane_error --seconds 30
    python3 -m main.chassis.cli.diag_lane_error --hz 20 --seconds 10

现场操作建议：
    1. 让车静止在中线上 → 看 error_y 的"零点漂移"（offset_y）
    2. 让车明显偏左/偏右一定距离 → 看 error_y 的幅值，反推 scale_y：
           scale_y ≈ 实际偏移（米）/ 模型输出幅值
    3. 在弯道上跑 → 看 error_angle 的动态范围是否够用（d_a 通道）
"""
from __future__ import annotations

import argparse
import statistics
import time
from typing import List, Optional

from ..api import ChassisClient


# ── IK 一致性对比表（静态信息，现场对照用）────────────────────
def _sdk_ik(vx: float, vy: float, wz: float) -> List[float]:
    """复刻 SDK vehicle_to_wheel_matrix 的逆解（tan_roller≈1.052·45°）。"""
    import math
    half_track = 0.30 / 2.0
    half_wb = 0.28 / 2.0
    tan_r = math.tan(math.pi / 4.0 * 1.052)
    r_c = half_track * tan_r + half_wb
    return [
        vx + vy * tan_r + wz * r_c,
        -vx + vy * tan_r + wz * r_c,
        -vx - vy * tan_r + wz * r_c,
        vx - vy * tan_r + wz * r_c,
    ]


def _chassis_ik(vx: float, vy: float, wz: float) -> List[float]:
    """main/chassis/controllers/base.py 的手写逆解（r_eff=0.30）。

    2026-08-01 已修正对齐 SDK 矩阵的 vy 轮序（此前元素 0/3 符号反、
    纯横移反解出的横向速度为 0）。
    """
    r = 0.30
    return [
        vx + vy + r * wz,
        -vx + vy + r * wz,
        -vx - vy + r * wz,
        vx - vy + r * wz,
    ]


def print_ik_table() -> None:
    """打印 SDK 逆解 vs chassis 手写逆解，现场核对横移轮序。"""
    cases = [
        ("纯前进 (0.3, 0, 0)", (0.3, 0.0, 0.0)),
        ("纯横移右 (0, .1, 0)", (0.0, 0.1, 0.0)),
        ("纯左转 (0, 0, .5)", (0.0, 0.0, 0.5)),
        ("前进+横移 (.2,.1,0)", (0.2, 0.1, 0.0)),
    ]
    print("── IK 一致性核对（SDK set_velocity 路径 vs chassis set_wheel_speeds 路径）──")
    for name, (vx, vy, wz) in cases:
        s = _sdk_ik(vx, vy, wz)
        c = _chassis_ik(vx, vy, wz)
        # 判据：逐位符号一致 + 量级差 < 1cm/s（chassis 忽略 roller_angle，r 差 ~1%）
        ok = all((a > 0) == (b > 0) and abs(a - b) < 0.01 for a, b in zip(s, c))
        match = "✓" if ok else "✗ 不一致！"
        print(f"{name:22s}  SDK   = [{', '.join('%+.4f' % v for v in s)}]")
        print(f"{'':22s}  chassis= [{', '.join('%+.4f' % v for v in c)}]  {match}")
    print("  纯前进一致是必然（vy=ω=0）。若纯横移标 ✗，说明手写 IK 与 SDK")
    print("  的 vy 轮序又不一致了 —— 需要实车确认哪套物理正确。\n")


# ── 统计 ───────────────────────────────────────────────────────
def _fmt(v: Optional[float]) -> str:
    return "  -  " if v is None else f"{v:+.7f}"


def _stats(samples: List[float], label: str) -> None:
    n = len(samples)
    if n == 0:
        print(f"  {label}: 无样本")
        return
    lo = min(samples)
    hi = max(samples)
    mean = statistics.fmean(samples)
    sd = statistics.pstdev(samples)
    s = sorted(samples)
    p50 = s[n // 2]
    p99 = s[min(n - 1, int(n * 0.99))]
    p01 = s[max(0, int(n * 0.01))]
    print(f"  {label}: n={n}  min={lo:+.7f}  max={hi:+.7f}  "
          f"span={hi - lo:.7f}  mean={mean:+.7f}  σ={sd:.7f}")
    print(f"           p01={p01:+.7f}  p50={p50:+.7f}  p99={p99:+.7f}")
    # 关键指标：|error_y| 有多大比例落在"基本为 0"区间（<1e-3）
    near_zero = sum(1 for v in samples if abs(v) < 1e-3)
    print(f"           |值|<1e-3 占比 = {near_zero / n * 100:.1f}%  "
          f"(接近 100% 说明 d_e 通道几乎不动 → 需要放大 scale_y)")


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(
        prog="main.chassis.cli.diag_lane_error",
        description="测量 lane 模型裸误差输出的分布，反推标定倍率。只读不下发。",
    )
    parser.add_argument("--seconds", type=float, default=20.0, help="采集时长（秒）")
    parser.add_argument("--hz", type=float, default=20.0, help="采样频率（Hz）")
    parser.add_argument("--no-ik", action="store_true", help="关掉开头的 IK 对比表")
    args = parser.parse_args(argv)

    if not args.no_ik:
        print_ik_table()
    print(f"开始采集 error_y/error_angle（{args.seconds:.0f}s @ {args.hz:.0f}Hz，"
          f"只读不下发，Ctrl+C 提前结束）...")

    api = ChassisClient.connect()
    try:
        api.start_lane_feed(hz=50.0)  # 保证 feed 在跑（幂等）
    except Exception:
        pass

    ey: List[float] = []
    ea: List[float] = []
    oy: List[float] = []

    period = 1.0 / max(args.hz, 1.0)
    deadline = time.monotonic() + max(0.0, args.seconds)
    next_tick = time.monotonic()
    try:
        while time.monotonic() < deadline:
            st = api.read_lane()
            if st.error_y is not None:
                ey.append(st.error_y)
                ea.append(st.error_angle if st.error_angle is not None else 0.0)
            try:
                _, y, _ = api.get_odometry()
                oy.append(y)
            except Exception:
                pass
            next_tick += period
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\n[手动结束]")
    finally:
        try:
            api.close()
        except Exception:
            pass

    print("\n── 测量结果 ──")
    _stats(ey, "error_y (d_e, 原始模型输出)")
    _stats(ea, "error_angle (d_a)")
    if len(oy) >= 2:
        dx = max(oy) - min(oy)
        print(f"  里程计 y 行程（可用来反推物理横移量）: {dx:.4f} m")
    if not ey:
        print("\n警告：没有采到任何 error_y 样本 —— 检查 lane_feed 是否在跑"
              "（runtime /v1/health）或镜头是否出画面。")
        return

    span = max(ey) - min(ey)
    if span < 1e-3:
        print(f"\n诊断：error_y 动态范围 {span:.7f} << 1e-3，d_e 通道基本是死的。")
        print("  下一步：把车开到明显偏左/偏右的位置重跑本工具，确认模型到底")
        print("  输出多大的值。若实车偏 0.1m 而输出仍 ~1e-4，则")
        print(f"    scale_y 建议起点 ≈ {0.1 / max(span, 1e-5):.0f}（把 0.1m 物理偏移放大到全幅）")
        print("  并配合 --error-scale-y 重跑巡线对比。")
    else:
        print(f"\n诊断：error_y 动态范围 {span:.7f}，模型输出可见但量纲未知。")
        print("  拿尺量实车一次明显偏移 S(米)，同时记录该状态下模型输出幅值 A，")
        print("  则 scale_y ≈ S / A。之后用 --error-scale-y 重跑巡线对比。")


if __name__ == "__main__":
    main()
