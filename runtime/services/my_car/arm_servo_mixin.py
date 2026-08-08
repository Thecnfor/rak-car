#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""MyCar 侧 arm 视觉伺服闭环 —— 进程内直调，无每帧网络往返。

2026-08-09 控制闭环下沉：task2 抓取视觉伺服原本在 main/ 侧每帧
WS 收 task_feed + HTTP POST /v1/realtime/arm-velocity，远程跑延迟高。
改 runtime 进程内闭环：读 streamer task_feed 缓存 + 直调 arm.x_speed /
set_arm_angle。main 只通过 /v1/execute 发一次目标参数，等结果回来。

控制律 mirror `main/arm/vision/velocity.py::find_target_arm_cross`
(大臂控 cx + x 十字控 cy，2026-08-02 实机标定):
  dx = cx - setpoint_x  → d_arm = sign_arm * dx * gain_arm (大臂增量)
  dy = cy - setpoint_y  → x_vel = sign_x  * dy * gain_x   (x 十字速度)
  y 十字锁 0，hand 由调用方在 servo 前摆好 (不动)。
方向符号 (task2 实机)：sign_arm=+1, sign_x=+1 (见 task_config pick_vision)。

速度/角度软限位 mirror `car_runtime_service.set_arm_velocity`
(X∈[-0.30,0]m, 大臂 clamp 由 arm_min/arm_max 控)，到硬界停速防撞墙。
"""
from __future__ import annotations

import logging
import time
from typing import Optional

logger = logging.getLogger("my_car.arm_servo")

# mirror car_runtime_service.set_arm_velocity 的十字硬界 (m)
X_MIN_M: float = -0.30
X_MAX_M: float = 0.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class ArmServoMixin:
    """进程内 arm 视觉伺服闭环 (arm 控 cx + x 十字控 cy)。"""

    def run_arm_servo(
        self,
        *,
        label: str = "water",
        hz: float = 20.0,
        gain_arm: float = 0.4,
        gain_x: float = 0.08,
        deadzone: float = 0.02,
        max_vel: float = 0.15,
        arm_start: float = -90.0,
        sign_arm: float = 1.0,
        sign_x: float = -1.0,
        setpoint_x_norm: float = 0.0,
        setpoint_y_norm: float = 0.0,
        arm_min: float = -150.0,
        arm_max: float = 90.0,
        timeout: float = 30.0,
        settle_hits: int = 3,
    ) -> dict:
        """跑一轮 arm 视觉伺服对齐，settled 或超时/停止后返回。

        参数即 main/arm/vision 同名字段（task_config pick_vision 原样传）：
        label/gain_arm/gain_x/deadzone/max_vel/sign_arm/sign_x/
        setpoint_x_norm/setpoint_y_norm/arm_min/arm_max/hz/timeout/settle_hits。
        arm_start 是增量微调起点 (task2 = pick_pose.arm_angle_deg = +90)。

        Returns:
            {"ok": True, "reason": "settled|timeout|stopped",
             "trace_hits": int, "settled": bool, "end_arm": float|None}
        """
        streamer = getattr(self, "streamer", None)
        if streamer is None:
            logger.warning("run_arm_servo: streamer 未注入，跳过")
            return {"ok": False, "reason": "streamer 未注入", "trace_hits": 0,
                    "settled": False, "end_arm": None}

        # 伺服前停 arm_feed，释放串口给 x_speed/set_arm_angle (同 main 侧约定).
        # 必须 force=True: force=False 是 NOOP, arm_feed 20Hz goto_position 轮询
        # 会 starve 串口队列, 伺服命令饿死 → job 超时 (2026-08-09 真机复现).
        try:
            self.stop_arm_feed(force=True)
        except Exception as exc:
            logger.warning("run_arm_servo: stop_arm_feed 异常 %s", exc)

        arm = getattr(self, "arm", None)
        if arm is None:
            logger.warning("run_arm_servo: arm 未初始化")
            try:
                self.start_arm_feed()
            except Exception:
                pass
            return {"ok": False, "reason": "arm 未初始化", "trace_hits": 0,
                    "settled": False, "end_arm": None}

        arm_target = float(arm_start)
        hits = 0
        in_band = 0
        settled = False
        reason = "timeout"
        period = 1.0 / max(hz, 1.0)
        deadline = time.time() + max(0.0, float(timeout))

        def _read_pick():
            """读 streamer task_feed 缓存，选离画面中心(吸嘴)最近的目标。"""
            try:
                state = streamer.get_task_state()
            except Exception:
                return None
            dets = []
            if isinstance(state, dict):
                dets = state.get("detections") or []
            best: Optional[tuple] = None
            best_d = 1e9
            for d in dets:
                if not isinstance(d, dict):
                    continue
                if (d.get("label") or "") != label:
                    continue
                bb = d.get("bbox_norm") or {}
                cx = bb.get("cx", bb.get("x_center"))
                cy = bb.get("cy", bb.get("y_center"))
                if cx is None or cy is None:
                    continue
                try:
                    cx, cy = float(cx), float(cy)
                except (TypeError, ValueError):
                    continue
                dist = abs(cx) + abs(cy)  # lock_first: 离吸嘴最近
                if dist < best_d:
                    best_d, best = dist, (cx, cy)
            return best

        try:
            while time.time() < deadline:
                t0 = time.time()
                # 协作停止：cancel / emergency-stop 每帧查 arm._must_stop
                try:
                    if arm._must_stop():
                        reason = "stopped"
                        break
                except Exception:
                    pass

                pick = _read_pick()
                if pick is None:
                    # 检测丢失 → x 停, 角度保持
                    try:
                        arm.x_speed(0.0)
                    except Exception:
                        pass
                    in_band = 0
                else:
                    cx, cy = pick
                    dx = cx - float(setpoint_x_norm)
                    dy = cy - float(setpoint_y_norm)
                    x_vel = 0.0 if abs(dy) < deadzone else _clamp(
                        sign_x * dy * gain_x, -max_vel, max_vel)
                    d_arm = 0.0 if abs(dx) < deadzone else sign_arm * dx * gain_arm
                    arm_target = _clamp(arm_target + d_arm, arm_min, arm_max)
                    hits += 1
                    # 软限位 mirror set_arm_velocity: 到硬界停速 (防撞墙)
                    try:
                        x_pos = arm.x_get_position()
                        if x_pos is not None:
                            if x_vel > 0 and float(x_pos) >= X_MAX_M:
                                x_vel = 0.0
                            elif x_vel < 0 and float(x_pos) <= X_MIN_M:
                                x_vel = 0.0
                    except Exception:
                        pass
                    try:
                        arm.x_speed(x_vel)
                        arm.set_arm_angle(arm_target, speed=80)
                    except Exception as exc:
                        logger.warning("run_arm_servo 下发异常: %s", exc)
                    # settle: 连续 settle_hits 帧 dx/dy 都在死区内
                    if abs(dx) < deadzone and abs(dy) < deadzone:
                        in_band += 1
                        if in_band >= settle_hits:
                            settled = True
                            reason = "settled"
                            break
                    else:
                        in_band = 0

                elapsed = time.time() - t0
                if elapsed < period:
                    time.sleep(period - elapsed)
        finally:
            # 结束必停 x (检测丢失/超时/停止统一收尾)
            try:
                arm.x_speed(0.0)
            except Exception:
                pass
            try:
                self.start_arm_feed()
            except Exception:
                pass

        return {
            "ok": True,
            "reason": reason,
            "trace_hits": hits,
            "settled": settled,
            "end_arm": arm_target,
        }
