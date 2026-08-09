"""task4 / target4 —— 底盘视觉对齐 (最左球) + 卡死重武装。

从 target4.py 拆出 (2026-08-10 拆分): 单一职责 = "把画面最左球拉到画面中心"。
- ``_track_leftmost_ball``  底盘视觉伺服, 5 段式成功判据 (硬停/软成/软重试/宽成/硬失败)。
- ``_chassis_rearm_if_stuck`` 底盘命令路径假死时重武装 (reset-stop + 0 速 + 直发轮速 IK)。
"""
from __future__ import annotations

from .constants import (  # noqa: E402
    BALL_LABELS,
    DEFAULT_TRACK_SOFT_DEADBAND, DEFAULT_TRACK_RETRY_SECONDS, DEFAULT_TRACK_WIDE_DEADBAND,
    LOG_PREFIX_TARGET4 as LOG_PREFIX,
)
from main.chassis import track_chassis  # noqa: E402
from main.chassis.loops.visual_track import TrackChassisResult  # noqa: E402


def _track_leftmost_ball(
    *,
    max_seconds: float,
    dry_run: bool,
    soft_deadband: float = DEFAULT_TRACK_SOFT_DEADBAND,
    retry_seconds: float = DEFAULT_TRACK_RETRY_SECONDS,
    wide_deadband: float = DEFAULT_TRACK_WIDE_DEADBAND,
):
    """底盘视觉伺服: 把画面最左 (cx 最小) 的球拉到画面中心。

    走 main.chassis.track_chassis (现场标定的 sign/kp/v_max/slew),
    内部 finally 自动零速。返回 TrackChassisResult
    (arrived / reason / final_frame.label=cx 最小的球 label)。

    2026-08-06 第 7 次迭代: 现场反馈 "没有失败, 然后失败了也不结束".
    之前软成功只覆盖 timeout + final_frame 落入 [soft, 2*soft] 区间
    重试 — 拉得不够, 现场说"应该差不多对齐"时 final_frame 偏 0.30+
    (远处偏不到位的球) 仍被判失败 → 退出。
    改进: 五段式成功判据
      1. 硬停: 3 帧连续 cx_err/cy_err < 0.05 → arrived=True
      2. 软成: timeout, final_frame |cx_err| < soft_deadband (0.15) → 视为 arrived
      3. 软重试: timeout, final_frame |cx_err| ∈ [soft_deadband, 2*soft_deadband]
                  → 额外 1s 重试, 用 2x kp 收口
      4. 宽成: timeout, final_frame |cx_err| ∈ [2*soft_deadband, wide_deadband (0.45)]
                  → 视为 near_arrived_wide, 走 pick, 错误计数不算
      5. 硬失败: 偏 wide_deadband 之外 (cx_err > 0.45 ≈ 画面 1/4 宽) → 失败

    设计意图: "现场肉眼看着差不多对齐" = cx_err < wide_deadband → 走 pick 一次.
    pick 失败再计数. 避免单次 visual 偏一点就硬退.
    """
    # 用户反馈底盘抖动, 回调稳: kp=0.10, v_max=0.08, v_slew=0.02, hold=3。
    res = track_chassis(
        target=BALL_LABELS,
        select_mode="leftmost",
        setpoint_cxcy=(0.0, 0.0),
        kp=0.20,
        v_max=0.12,
        deadband=0.05,
        hold_frames=3,
        v_slew=0.04,
        decouple_xy=False,
        max_seconds=max_seconds,
        dry_run=dry_run,
    )

    # 2026-08-06: 已 arrived / 软成功 / 重试全部覆盖后, 真正的失败
    # (no_target / watchdog / stopped 等) 仍返回原 res. step_target4 自己按 reason 决定.
    if res.arrived or res.reason != "timeout":
        return res

    # 软成功判定: final_frame 在软死区内 → 视为 arrived
    ff = res.final_frame
    if ff is not None and ff.target_found:
        cx_err = ff.cx_err if ff.cx_err is not None else 0.0
        cy_err = ff.cy_err if ff.cy_err is not None else 0.0
        if abs(cx_err) < soft_deadband and abs(cy_err) < soft_deadband:
            print(f"  [{LOG_PREFIX}] �� track 软成功: timeout 但 final_frame |cx_err|="
                  f"{abs(cx_err):.3f} |cy_err|={abs(cy_err):.3f} "
                  f"均在软死区 {soft_deadband:.2f} 内, 视为 arrived")
            # 强行构造 arrived=True 返回: TrackChassisResult 是 dataclass, 替换
            res = TrackChassisResult(
                arrived=True,
                reason="near_arrived_soft",
                final_frame=ff,
                frames=res.frames,
                elapsed_s=res.elapsed_s,
                stop_ok=getattr(res, "stop_ok", True),
                motion_ok=getattr(res, "motion_ok", True),
                enc_delta=getattr(res, "enc_delta", None),
            )
            return res

        # 软重试: final_frame 偏 [soft_deadband, 2*soft_deadband] 区间
        # 再跑一次 retry_seconds 短时 track, 用更大 kp 让它"加把劲"收口
        if retry_seconds > 0 and abs(cx_err) < 2 * soft_deadband and abs(cy_err) < 2 * soft_deadband:
            print(f"  [{LOG_PREFIX}] �� track 软重试: 偏 [soft, 2*soft] 区间, "
                  f"再给 {retry_seconds:.1f}s 用更大 kp 收口")
            retry_res = track_chassis(
                target=BALL_LABELS,
                select_mode="leftmost",
                setpoint_cxcy=(0.0, 0.0),
                kp=0.20,
                v_max=0.12,
                deadband=0.05,
                hold_frames=3,
                v_slew=0.04,
                decouple_xy=False,
                max_seconds=retry_seconds,
                dry_run=dry_run,
            )
            if retry_res.arrived:
                print(f"  [{LOG_PREFIX}] �� track 软重试成功 arrived=True "
                      f"reason={retry_res.reason}")
                return retry_res
            # 软重试失败: 也用 final_frame 软死区判一次
            rff = retry_res.final_frame
            if rff is not None and rff.target_found:
                rcx = rff.cx_err if rff.cx_err is not None else 0.0
                rcy = rff.cy_err if rff.cy_err is not None else 0.0
                if abs(rcx) < soft_deadband and abs(rcy) < soft_deadband:
                    print(f"  [{LOG_PREFIX}] �� track 软重试后 final_frame 落入软死区: "
                          f"|cx_err|={abs(rcx):.3f} |cy_err|={abs(rcy):.3f}")
                    return TrackChassisResult(
                        arrived=True,
                        reason="near_arrived_soft_retry",
                        final_frame=rff,
                        frames=res.frames + retry_res.frames,
                        elapsed_s=res.elapsed_s + retry_res.elapsed_s,
                        stop_ok=getattr(retry_res, "stop_ok",
                                        getattr(res, "stop_ok", True)),
                        motion_ok=getattr(retry_res, "motion_ok",
                                          getattr(res, "motion_ok", True)),
                        enc_delta=getattr(
                            retry_res, "enc_delta",
                            getattr(res, "enc_delta", None)),
                    )
            # 软重试失败: 也检查宽死区
            rff = retry_res.final_frame
            if rff is not None and rff.target_found:
                rcx = rff.cx_err if rff.cx_err is not None else 0.0
                rcy = rff.cy_err if rff.cy_err is not None else 0.0
                if abs(rcx) < wide_deadband and abs(rcy) < wide_deadband:
                    print(f"  [{LOG_PREFIX}] �� track 软重试后落入宽死区 "
                          f"({wide_deadband:.2f}): |cx_err|={abs(rcx):.3f} "
                          f"|cy_err|={abs(rcy):.3f}, 视为 near_arrived_wide")
                    return TrackChassisResult(
                        arrived=True,
                        reason="near_arrived_wide",
                        final_frame=rff,
                        frames=res.frames + retry_res.frames,
                        elapsed_s=res.elapsed_s + retry_res.elapsed_s,
                        stop_ok=getattr(retry_res, "stop_ok",
                                        getattr(res, "stop_ok", True)),
                        motion_ok=getattr(retry_res, "motion_ok",
                                          getattr(res, "motion_ok", True)),
                        enc_delta=getattr(
                            retry_res, "enc_delta",
                            getattr(res, "enc_delta", None)),
                    )
            # 真正失败: 仍返回原 res (arrived=False, reason=timeout)
            print(f"  [{LOG_PREFIX}] ❌ track 软重试也失败: "
                  f"arrived={retry_res.arrived} reason={retry_res.reason}")

        # 宽成: 第一阶段 timeout 但 final_frame 在 [2*soft, wide_deadband] 区间
        # (软重试未触发), 视为"差不多对齐" → 走 pick
        elif abs(cx_err) < wide_deadband and abs(cy_err) < wide_deadband:
            print(f"  [{LOG_PREFIX}] �� track 宽成: 偏 [2*soft, wide] 区间, "
                  f"视为 near_arrived_wide: |cx_err|={abs(cx_err):.3f} "
                  f"|cy_err|={abs(cy_err):.3f} (< 宽死区 {wide_deadband:.2f})")
            return TrackChassisResult(
                arrived=True,
                reason="near_arrived_wide",
                final_frame=ff,
                frames=res.frames,
                elapsed_s=res.elapsed_s,
                stop_ok=getattr(res, "stop_ok", True),
                motion_ok=getattr(res, "motion_ok", True),
                enc_delta=getattr(res, "enc_delta", None),
            )

    return res


# 2026-08-06: track_chassis 在 no_target 场景也常常是"底盘响应但视觉丢了"
# 或"底盘真没动但视野里球仍在" —— 两种情况都是 set_chassis_velocity 没真正
# 生效 (CLAUDE.md 提的 OPEN chassis realtime-velocity no-motion bug).
# 解决: no_target 时主动读一次 odom_encoder 比对命令速度, 若 0.05s 内轮速
# 变化 < 阈值, 判定"底盘没动", 发一次强制 reset-stop + 直发轮速 IK 重启通信.
# 这只针对 no_target (视野里球还在但 lost_frames++), 不动 timeout / arrived.
# 见 _chassis_rearm_if_stuck() 详情.

def _chassis_rearm_if_stuck(http_client, *, settle_s: float = 0.5) -> bool:
    """底盘 stuck 检测 + 重新武装。

    流程:
      1. 读当前 wheel_encoders (fast-path, 单次 < 2ms)
      2. sleep settle_s 一段时间
      3. 再读 wheel_encoders
      4. 如果 4 轮编码器总变化 < 1.0 (≈ 0.5mm 累计, 极保守阈值)
         → 判定"底盘没动", 顺序 call:
            a) POST /v1/control/reset-stop (清 _stop_flag, 急停残留)
            b) POST /v1/realtime/chassis-velocity (vx=0, vy=0, wz=0)
            c) POST /v1/realtime/wheels/speeds (IK 反算 4 轮速, 通过 SerialEngine
               协调心跳, 跟 set_chassis_velocity 不同链路) ½s 内
            d) 再次发 vx=0 vy=0 wz=0
            返回 True (重武装成功)
         否则返回 False (底盘真的在动, 不需要 re-arm).

    这是粗暴的兜底: 不重建 chassis 引用 (那是 runtime 层), 也不重启守护线程
    (那是 force=True 路径). 只清 stop_flag + 重新下发 baseline 速度, 重置
    SerialEngine 的 IK 命令缓存.
    """
    try:
        e1 = http_client.get(f"{http_client.api_prefix}/realtime/wheels/encoders")
    except Exception:
        return False
    if not isinstance(e1, dict):
        return False
    enc1 = e1.get("encoders") or []
    if not isinstance(enc1, list) or len(enc1) < 4:
        return False
    try:
        enc1 = [float(x) for x in enc1]
    except (TypeError, ValueError):
        return False

    import time as _t
    _t.sleep(settle_s)

    try:
        e2 = http_client.get(f"{http_client.api_prefix}/realtime/wheels/encoders")
    except Exception:
        return False
    if not isinstance(e2, dict):
        return False
    enc2 = e2.get("encoders") or []
    if not isinstance(enc2, list) or len(enc2) < 4:
        return False
    try:
        enc2 = [float(x) for x in enc2]
    except (TypeError, ValueError):
        return False

    total_delta = sum(abs(enc2[i] - enc1[i]) for i in range(4))
    if total_delta >= 1.0:
        # 底盘在动, 不需要 re-arm
        return False

    # 底盘 stuck: 重武装
    try:
        http_client.post(f"{http_client.api_prefix}/control/reset-stop", payload={})
    except Exception:
        pass
    attempts = [
        ("realtime/chassis-velocity", {"vx": 0.0, "vy": 0.0, "wz": 0.0}),
        ("realtime/wheels/speeds", {"speeds": [0.0, 0.0, 0.0, 0.0]}),
    ]
    for path, payload in attempts:
        try:
            http_client.post(f"{http_client.api_prefix}/{path}", payload=payload, timeout=1.0)
        except Exception:
            pass
    try:
        http_client.post(
            f"{http_client.api_prefix}/realtime/chassis-velocity",
            {"vx": 0.0, "vy": 0.0, "wz": 0.0},
            timeout=1.0,
        )
    except Exception:
        pass
    return True
