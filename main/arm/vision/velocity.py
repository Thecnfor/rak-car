"""main/arm/vision/velocity.py — velocity 模式追踪 (07/08 示例抽出的正式 API).

背景 (2026-08-02 实机验证):
  位置闭环 (goto_position) ~500ms/步 + arm_queue 积压 = "离散/乱跑" 根因
  (TEST_PREFLIGHT §13.1)。velocity 模式 POST /v1/realtime/arm-velocity 直发
  (免 queue, _realtime_gate 免 car_lock), 是高频视觉伺服的推荐传输。

两种追踪:
  find_target_velocity : 只动 xy 十字 (示例 07, 实机命中 63%)
  find_target_4dof     : xy + 大臂(yaw) + 手抓(pitch) 四轴增量联调 (示例 08,
                         方向修正后实机命中 88%)

方向约定 (2026-08-01/02 真机实测, 已固化, 可用 sign_* 参数覆盖):
  x_vel  = -dx * gain_x     bbox 偏右(dx>0) → 十字向左追
  y_vel  = +dy * gain_y     bbox 偏下(dy>0) → 十字向下追   ← 提交版 -dy 是反的
  d_arm  = +dx * gain_arm   bbox 偏右 → 大臂向右转
  d_hand = +dy * gain_hand  bbox 偏下 → 手爪向下转

安全:
  - y_speed 内置磁感安全门 + 末段/顶段减速 (arm_base.y_speed)
  - x 无软限位: 检测丢失即停 + max_vel 限速 + 结束必然 x_vel=0
  - 本模块只做追踪循环; 起始位 composite_run / arm_feed 让位 / 复位由
    ArmRunner.track_velocity / track_4dof 编排。
"""
from __future__ import annotations

import dataclasses
import logging
import threading
import time
from typing import Callable, List, Optional

from .parsers import _parse_cache

logger = logging.getLogger(__name__)

REALTIME_URL = "/v1/realtime/arm-velocity"


@dataclasses.dataclass(frozen=True)
class VelocityTrace:
    """单帧 velocity 追踪记录 (逐帧可回放)."""
    t_s: float
    dx: float
    dy: float
    x_vel: float
    y_vel: float
    arm: Optional[float] = None
    hand: Optional[float] = None
    score: float = 0.0
    miss: bool = False


@dataclasses.dataclass(frozen=True)
class VelocityResult:
    """velocity 追踪汇总. trace 逐帧可回放; summary() 一行打印概况."""
    label: str
    frames: int
    hits: int
    misses: int
    elapsed_s: float
    end_arm: Optional[float]
    end_hand: Optional[float]
    max_abs_vel_mms: float
    avg_abs_vel_mms: float
    trace: tuple

    def summary(self) -> str:
        return (f"velocity[{self.label}] frames={self.frames} "
                f"hit={self.hits} miss={self.misses} "
                f"elapsed={self.elapsed_s:.1f}s |v|max={self.max_abs_vel_mms:.0f}mm/s "
                f"|v|avg={self.avg_abs_vel_mms:.0f}mm/s "
                f"arm_end={self.end_arm} hand_end={self.end_hand}")


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _try_post(post_fn: Callable, **kw) -> None:
    try:
        post_fn(**kw)
    except Exception as exc:
        logger.warning("velocity post 异常: %s", exc)


class VelocityLoop:
    """velocity 追踪 mixin. 需 self.http (RuntimeApiClient) + 懒建 self.ws."""

    def _ensure_ws(self, ws):
        if ws is None:
            try:
                from main.ws_client import RuntimeWsClient
            except ImportError:  # pragma: no cover
                from ws_client import RuntimeWsClient  # type: ignore
            ws = RuntimeWsClient()
        return ws

    def _default_post_fn(self) -> Callable:
        """POST /v1/realtime/arm-velocity (免 queue 直发); 单测可注入 post_fn 覆盖.

        内部 step 用领域词 arm/hand, 端点 payload 要求 arm_angle/hand_angle —
        这里做映射, 注入的 post_fn 则原样收到 arm/hand (内部契约)。
        """
        import requests

        def post(**kw) -> dict:
            payload: dict = {}
            for k, v in kw.items():
                if v is None:
                    continue
                if k == "arm":
                    payload["arm_angle"] = float(v)
                elif k == "hand":
                    payload["hand_angle"] = float(v)
                else:
                    payload[k] = float(v)
            r = requests.post(self.http.build_url(REALTIME_URL),
                              json=payload, timeout=2.0)
            r.raise_for_status()
            return r.json()
        return post

    def _run_velocity(self, label: str, *, timeout: float, hz: float,
                      ws, step_fn, selector=None,
                      settle_frames: Optional[int] = None,
                      settle_tol: float = 0.04) -> List[VelocityTrace]:
        """共享 velocity 核心: WS 订阅 → 每帧 step_fn(t, pick) → 超时/订阅退出.

        step_fn(t, pick) 负责算速度 + 发命令 + 返回 VelocityTrace;
        pick 为 None 表示该帧检测丢失 (step_fn 内部处理停)。
        selector (可选): 多目标选择器, 传入后用 selector.matches + apply_strategy;
        否则默认 label 过滤 + HIGHEST_SCORE。

        settle_frames (可选, 2026-08-04): 连续 N 帧 |dx|,|dy| < settle_tol → 提前
        收敛退出 (不用等满 timeout)。None = 关闭 (保持旧行为, 跑满 timeout)。
        """
        t0 = time.time()
        trace: List[VelocityTrace] = []
        done = threading.Event()
        settle_count = [0]  # 连续收敛帧计数 (闭包内可变)

        def _on_push(raw: dict) -> None:
            if done.is_set():
                return
            if time.time() - t0 > timeout:
                done.set()
                return
            try:
                dets = _parse_cache(raw)
            except Exception:
                return
            pick = None
            if selector is not None:
                # 用 selector.matches + apply_strategy (支持 lock_first)
                cands = [d for d in dets if selector.matches(d)]
                if selector.track_id is not None:
                    # 已锁定: 只取该 track_id
                    pick = next((d for d in cands if d.track_id == selector.track_id), None)
                elif cands:
                    # 未锁定: 用 CLOSEST_TO_CENTER 选离吸嘴 (0,0) 最近的目标, 锁定 track_id
                    chosen = min(cands, key=lambda d:
                                 abs(d.bbox_norm.x_center) + abs(d.bbox_norm.y_center))
                    if chosen is not None and chosen.track_id is not None:
                        selector.track_id = chosen.track_id  # 锁住
                    pick = chosen
            else:
                # 默认 label 过滤 + HIGHEST_SCORE
                best = -1.0
                for d in dets:
                    if d.label == label and (d.score or 0.0) > best:
                        best = d.score or 0.0
                        pick = d
            tr = step_fn(time.time() - t0, pick)
            trace.append(tr)
            # 2026-08-04: 收敛提前退出 (settle_frames 开启时)
            if settle_frames is not None:
                if (not tr.miss) and abs(tr.dx) < settle_tol and abs(tr.dy) < settle_tol:
                    settle_count[0] += 1
                    if settle_count[0] >= settle_frames:
                        done.set()
                else:
                    settle_count[0] = 0

        stop_sub = ws.subscribe_task_detection(_on_push, hz=hz)
        try:
            while not done.is_set() and time.time() - t0 < timeout:
                time.sleep(0.05)
        finally:
            done.set()
            try:
                stop_sub()
            except Exception:
                pass
        return trace

    def find_target_velocity(self, label: str, *,
                             timeout: float = 30.0, hz: float = 20.0,
                             gain: float = 0.05, deadzone: float = 0.02,
                             max_vel: float = 0.15,
                             sign_x: float = -1.0, sign_y: float = 1.0,
                             setpoint_x_norm: float = 0.0,
                             setpoint_y_norm: float = 0.0,
                             post_fn: Optional[Callable] = None,
                             ws=None) -> VelocityResult:
        """velocity XY 追踪 (示例 07): 只动十字, 检测丢失即停.

        方向: x_vel=-dx·gain, y_vel=+dy·gain (真机实测已固化)。
        setpoint_x/y_norm (2026-08-02): 吸嘴中心偏移 (目标在吸嘴正下方时其 bbox 中心
        坐标)。默认 (0,0)=画面中心; 传标定值即把目标对准吸嘴正下方而非画面中心。
        """
        post_fn = post_fn or self._default_post_fn()
        ws = self._ensure_ws(ws)

        def step(t: float, pick) -> VelocityTrace:
            if pick is None:
                _try_post(post_fn, x_vel=0.0, y_vel=0.0)
                return VelocityTrace(t, 0.0, 0.0, 0.0, 0.0, score=0.0, miss=True)
            dx = pick.bbox_norm.x_center - setpoint_x_norm
            dy = pick.bbox_norm.y_center - setpoint_y_norm
            x_vel = 0.0 if abs(dx) < deadzone else _clamp(sign_x * dx * gain, -max_vel, max_vel)
            y_vel = 0.0 if abs(dy) < deadzone else _clamp(sign_y * dy * gain, -max_vel, max_vel)
            _try_post(post_fn, x_vel=x_vel, y_vel=y_vel)
            return VelocityTrace(t, dx, dy, x_vel, y_vel, score=pick.score)

        t0 = time.time()
        try:
            trace = self._run_velocity(label, timeout=timeout, hz=hz, ws=ws, step_fn=step)
        finally:
            _try_post(post_fn, x_vel=0.0, y_vel=0.0)  # 结束必然急停
        return _summarize(label, trace, time.time() - t0, None, None)

    def find_target_4dof(self, label: str, *,
                         timeout: float = 30.0, hz: float = 20.0,
                         gain_x: float = 0.05, gain_y: float = 0.05,
                         gain_arm: float = 2.0, gain_hand: float = 2.0,
                         deadzone: float = 0.02, max_vel: float = 0.15,
                         arm_start: float = 0.0, hand_start: float = -90.0,
                         arm_min: float = -90.0, arm_max: float = 90.0,
                         hand_min: float = -90.0, hand_max: float = 0.0,
                         sign_x: float = -1.0, sign_y: float = 1.0,
                         sign_arm: float = 1.0, sign_hand: float = 1.0,
                         setpoint_x_norm: float = 0.0,
                         setpoint_y_norm: float = 0.0,
                         hold_y: bool = True,
                         post_fn: Optional[Callable] = None,
                         ws=None) -> VelocityResult:
        """4-DOF 追踪 (示例 08, 方向修正后): x 十字 + 大臂 + 手抓 增量联调.

        大臂/手抓是增量式角度目标 (从 arm_start/hand_start 起, 每帧 clamp 后累加),
        全部打包进 /v1/realtime/arm-velocity 一发。检测丢失 → xy 停, 角度不动。
        setpoint_x/y_norm (2026-08-02): 吸嘴中心偏移 (目标在吸嘴正下方时其 bbox 中心
        坐标)。默认 (0,0)=画面中心。
        hold_y (2026-08-02, 默认 True): 用户协议 — 对齐阶段锁死 y 十字 (y_vel=0),
        垂直误差只靠 hand 增量转补偿。原因: y=-180 对齐时一旦 y 十字下移, 目标立刻
        被推出视野 (相机随 y 移动)。水平误差走 x 十字 + arm 转双通道。
        """
        post_fn = post_fn or self._default_post_fn()
        ws = self._ensure_ws(ws)
        arm_target = arm_start
        hand_target = hand_start

        def step(t: float, pick) -> VelocityTrace:
            nonlocal arm_target, hand_target
            if pick is None:
                _try_post(post_fn, x_vel=0.0, y_vel=0.0)
                return VelocityTrace(t, 0.0, 0.0, 0.0, 0.0,
                                     arm=arm_target, hand=hand_target,
                                     score=0.0, miss=True)
            dx = pick.bbox_norm.x_center - setpoint_x_norm
            dy = pick.bbox_norm.y_center - setpoint_y_norm
            x_vel = 0.0 if abs(dx) < deadzone else _clamp(sign_x * dx * gain_x, -max_vel, max_vel)
            y_vel = 0.0 if hold_y else (
                0.0 if abs(dy) < deadzone else _clamp(sign_y * dy * gain_y, -max_vel, max_vel))
            d_arm = 0.0 if abs(dx) < deadzone else sign_arm * dx * gain_arm
            d_hand = 0.0 if abs(dy) < deadzone else sign_hand * dy * gain_hand
            arm_target = _clamp(arm_target + d_arm, arm_min, arm_max)
            hand_target = _clamp(hand_target + d_hand, hand_min, hand_max)
            _try_post(post_fn, x_vel=x_vel, y_vel=y_vel,
                      arm=arm_target, hand=hand_target)
            return VelocityTrace(t, dx, dy, x_vel, y_vel,
                                 arm=arm_target, hand=hand_target, score=pick.score)

        t0 = time.time()
        try:
            trace = self._run_velocity(label, timeout=timeout, hz=hz, ws=ws, step_fn=step)
        finally:
            _try_post(post_fn, x_vel=0.0, y_vel=0.0)  # 结束必然急停
        return _summarize(label, trace, time.time() - t0, arm_target, hand_target)


    def find_target_arm_cross(self, label: str, *,
                              timeout: float = 30.0, hz: float = 20.0,
                              gain_arm: float = 0.4, gain_x: float = 0.08,
                              deadzone: float = 0.02, max_vel: float = 0.15,
                              arm_start: float = -90.0,
                              arm_min: float = -150.0, arm_max: float = 90.0,
                              setpoint_x_norm: float = 0.0,
                              setpoint_y_norm: float = 0.0,
                              sign_arm: float = 1.0, sign_x: float = -1.0,
                              selector=None,
                              settle_frames: Optional[int] = None,
                              settle_tol: float = 0.04,
                              post_fn: Optional[Callable] = None,
                              ws=None) -> VelocityResult:
        """机械臂专用追踪 (2026-08-02 实机标定): 大臂控 cx + x 十字控 cy.

        本机械结构 (y=-180 标定姿态实测):
          画面水平 cx  ← arm_angle (大臂更负 → cx 更右/更大)
          画面垂直 cy  ← x 十字位置 (x 更左 → cy 更上/更小)
          y 十字/手抓  → 锁死 (y 下移目标出视野, hand 固定 0° 朝下)

        因此误差映射与通用 4-DOF 不同:
          dx = cx - setpoint_x  → arm 增量 (d_arm = dx·gain_arm)
          dy = cy - setpoint_y  → x 十字速度 (x_vel = sign_x·dy·gain_x, sign_x=-1)
          y_vel 恒 0, hand 不动.

        方向符号 (实机标定 2026-08-02):
          dx>0 (目标偏右) → arm 要减小(更负) → sign_arm=+1 时 d_arm 负
          dy>0 (目标偏下) → x 要往左 → x_vel 负 → sign_x=-1

        Returns: VelocityResult (arm 终值 = end_arm).
        """
        post_fn = post_fn or self._default_post_fn()
        ws = self._ensure_ws(ws)
        arm_target = arm_start

        def step(t: float, pick) -> VelocityTrace:
            nonlocal arm_target
            if pick is None:
                _try_post(post_fn, x_vel=0.0, y_vel=0.0)
                return VelocityTrace(t, 0.0, 0.0, 0.0, 0.0,
                                     arm=arm_target, score=0.0, miss=True)
            dx = pick.bbox_norm.x_center - setpoint_x_norm
            dy = pick.bbox_norm.y_center - setpoint_y_norm
            x_vel = 0.0 if abs(dy) < deadzone else _clamp(
                sign_x * dy * gain_x, -max_vel, max_vel)
            d_arm = 0.0 if abs(dx) < deadzone else sign_arm * dx * gain_arm
            arm_target = _clamp(arm_target + d_arm, arm_min, arm_max)
            _try_post(post_fn, x_vel=x_vel, y_vel=0.0,
                      arm=arm_target)
            return VelocityTrace(t, dx, dy, x_vel, 0.0,
                                 arm=arm_target, score=pick.score)

        t0 = time.time()
        try:
            trace = self._run_velocity(label, timeout=timeout, hz=hz, ws=ws,
                                      step_fn=step, selector=selector,
                                      settle_frames=settle_frames,
                                      settle_tol=settle_tol)
        finally:
            _try_post(post_fn, x_vel=0.0, y_vel=0.0)
        return _summarize(label, trace, time.time() - t0, arm_target, None)


def _summarize(label: str, trace: List[VelocityTrace], elapsed_s: float,
               end_arm: Optional[float], end_hand: Optional[float]) -> VelocityResult:
    hits = [t for t in trace if not t.miss]
    n_hit = len(hits)
    if n_hit:
        max_v = max(max(abs(t.x_vel), abs(t.y_vel)) for t in hits)
        avg_v = sum(abs(t.x_vel) + abs(t.y_vel) for t in hits) / n_hit
    else:
        max_v = avg_v = 0.0
    return VelocityResult(
        label=label, frames=len(trace), hits=n_hit, misses=len(trace) - n_hit,
        elapsed_s=elapsed_s, end_arm=end_arm, end_hand=end_hand,
        max_abs_vel_mms=max_v * 1000.0, avg_abs_vel_mms=avg_v * 1000.0,
        trace=tuple(trace),
    )
