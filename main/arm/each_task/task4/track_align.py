"""task4 / target4 —— 底盘视觉对齐 (最左球) + 卡死重武装。

从 target4.py 拆出 (2026-08-10 拆分): 单一职责 = "把画面最左球拉到画面中心"。
- ``_track_leftmost_ball``  底盘视觉伺服: 跑 max_seconds, 超时且给了 extend_seconds
                              再跑 extend_seconds, 返回最终结果 (上层不因未对齐阻塞)。
- ``_chassis_rearm_if_stuck`` 底盘命令路径假死时重武装 (reset-stop + 0 速 + 直发轮速 IK)。

2026-08-11 用户确认: 底盘对齐 4s → 超时加时 3s (总 7s) → 仍超时记"超时失败"但
**继续下一步** (不阻塞、不放弃球)。原五段式 (软成功/软重试/宽成) 废除。
"""
from __future__ import annotations

from .constants import (  # noqa: E402
    BALL_LABELS,
    LOG_PREFIX_TARGET4 as LOG_PREFIX,
)
from main.chassis import track_chassis  # noqa: E402
from main.chassis.loops.visual_track import TrackChassisResult  # noqa: E402


def _track_failed_result(reason: str = "error") -> TrackChassisResult:
    """HTTP/网络异常时的兜底结果 (arrived=False, 上层继续, 不阻塞)."""
    return TrackChassisResult(
        arrived=False, reason=reason, final_frame=None,
        frames=0, elapsed_s=0.0, stop_ok=False, motion_ok=False, enc_delta=None,
    )


def _track_leftmost_ball(
    *,
    max_seconds: float,
    dry_run: bool,
    extend_seconds: float = 0.0,
    kp: float = 0.05,
    v_max: float = 0.04,
    deadband: float = 0.08,
    hold_frames: int = 3,
    v_slew: float = 0.01,
    decouple_xy: bool = True,
):
    """底盘视觉伺服: 把画面最左 (cx 最小) 的球拉到画面中心。

    走 main.chassis.track_chassis (现场标定的 sign/kp/v_max/slew),
    内部 finally 自动零速。返回 TrackChassisResult
    (arrived / reason / final_frame.label=cx 最小的球 label)。

    2026-08-11 用户: 4s 超时 → 加时 extend_seconds (总上限 4+3=7s)。
    无论最终 arrived 与否, 上层都继续 (失败不阻塞) —— 机械臂视觉伺服接管对齐。
    ⚠️ track_chassis 是阻塞 HTTP, runtime 无响应会抛 ReadTimeout/ConnectionReset;
        必须捕获, 否则会打崩整个 task4 (2026-08-11 实车复现)。
    """
    try:
        res = track_chassis(
            target=BALL_LABELS,
            select_mode="leftmost",
            setpoint_cxcy=(0.0, 0.0),
            kp=kp,
            v_max=v_max,
            deadband=deadband,
            hold_frames=hold_frames,
            v_slew=v_slew,
            decouple_xy=decouple_xy,
            max_seconds=max_seconds,
            dry_run=dry_run,
        )
    except Exception as e:
        print(f"  [{LOG_PREFIX}] ⚠️ track_chassis 异常 "
              f"({type(e).__name__}: {str(e)[:100]}), 按失败继续 (臂伺服接管)")
        return _track_failed_result()
    if (not res.arrived) and res.reason == "timeout" and extend_seconds > 0:
        print(f"  [{LOG_PREFIX}] track 超时 ({max_seconds:.0f}s), "
              f"加时 {extend_seconds:.0f}s (总上限 {max_seconds + extend_seconds:.0f}s)")
        try:
            retry = track_chassis(
                target=BALL_LABELS,
                select_mode="leftmost",
                setpoint_cxcy=(0.0, 0.0),
                kp=kp,
                v_max=v_max,
                deadband=deadband,
                hold_frames=hold_frames,
                v_slew=v_slew,
                decouple_xy=decouple_xy,
                max_seconds=extend_seconds,
                dry_run=dry_run,
            )
        except Exception as e:
            print(f"  [{LOG_PREFIX}] ⚠️ track 加时异常 "
                  f"({type(e).__name__}: {str(e)[:100]}), 保留首次结果继续")
            return res
        return retry
    return res


# 2026-08-06: track_chassis 在 no_target 场景也常常是"底盘响应但视觉丢了"
# 或"底盘真没动但视野里球仍在" —— 两种情况都是 set_chassis_velocity 没真正
# 生效 (CLAUDE.md 提的 OPEN chassis realtime-velocity no-motion bug).
# 解决: no_target 时主动读一次 odom_encoder 比对命令速度, 若 0.05s 内轮速
# 变化 < 阈值, 判定"底盘没动", 发一次强制 reset-stop + 直发轮速 IK 重启通信.
# 这只针对 no_target (视野里球还在但 lost_frames++), 不动 timeout / arrived.
# 见 _chassis_rearm_if_stuck() 详情.
# 2026-08-11 新版流程下 0.1m move_for 前进若底盘假死也会用到此兜底, 保留.

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
