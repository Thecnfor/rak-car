"""main/arm/vision/realtime.py — WS 推送路径: find_target_realtime + find_target_track."""
from __future__ import annotations

import dataclasses
import logging
import threading
import time
from typing import Callable, Optional

from .parsers import _parse_cache
from .selector import SelectionStrategy
from .types import Detection, ServoResult

logger = logging.getLogger(__name__)


def _pid_step(err: float, int_err: float, last_err: float, dt: float,
              kp: float, ki: float, kd: float) -> tuple:
    """单轴 PID 一步. 返回 (output, new_int_err, new_last_err), output 限幅 ±1.0."""
    int_err = max(-1.0, min(1.0, int_err + err * dt))
    deriv = (err - last_err) / dt if dt > 0 else 0.0
    out = kp * err + ki * int_err + kd * deriv
    out = max(-1.0, min(1.0, out))
    return out, int_err, err


class RealtimeLoop:
    """WS 推送路径 mixin. 需 self.http + 注入 self.ws (默认懒建).

    find_target_realtime / find_target_track 默认 ki=0 + settle_stable_frames=1
    → 行为 100% 等价于原版 (单 P, 单帧收敛). 传 ki/target_real_height_m 即升级 PID+depth.
    """

    def _ensure_ws(self, ws):
        if ws is None:
            try:
                from main.ws_client import RuntimeWsClient
            except ImportError:  # pragma: no cover
                from ws_client import RuntimeWsClient  # type: ignore
            ws = RuntimeWsClient()
        return ws

    @staticmethod
    def _adaptive_gain(bbox, target_real_height_m: float,
                       focal_length_px: float, mm_per_norm_base: float,
                       ref_depth_m: float) -> float:
        """depth-aware gain (从 .compute_depth 拿深度, 调 mm_per_norm)."""
        from . import ArmVisionClient
        if (target_real_height_m and target_real_height_m > 0
                and bbox is not None and bbox.height > 0):
            depth_m = ArmVisionClient.compute_depth(bbox, target_real_height_m, focal_length_px)
            return mm_per_norm_base * (depth_m / ref_depth_m)
        return mm_per_norm_base

    def find_target_realtime(self, selector, *,
                             x_mm: float, y_mm: float,
                             hz: float = 30.0,
                             mm_per_norm: float = 30.0,
                             settle_tol_norm: float = 0.05,
                             setpoint_x_norm: float = 0.0,
                             setpoint_y_norm: float = 0.0,
                             min_step_mm: float = 1.0,
                             timeout: float = 10.0,
                             on_missing_track: str = "abort",
                             # 2026-08-01 升级可选参数 (默认 0 走原 P 行为)
                             mm_per_norm_base: Optional[float] = None,
                             target_real_height_m: Optional[float] = None,
                             focal_length_px: float = 600.0,
                             ref_depth_m: float = 0.30,
                             kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                             settle_stable_frames: int = 1,
                             arm_dx_threshold_norm: float = 0.3,
                             on_strategic_4dof: Optional[Callable[[str, Detection], None]] = None,
                             move_fn: Optional[Callable[[float, float], dict]] = None,
                             ws=None) -> ServoResult:
        ws = self._ensure_ws(ws)
        stop_event = threading.Event()
        abort_reason: dict = {"reason": None}
        state = {
            "x_mm": x_mm, "y_mm": y_mm,
            "last_detection": None,
            "consecutive_misses": 0,
            "locked_track_id": None,
            "current_selector": selector,
            "last_updated_at": None,
            # PID state
            "int_err_x": 0.0, "int_err_y": 0.0,
            "last_err_x": 0.0, "last_err_y": 0.0,
            "last_t": 0.0,
            "consecutive_settle": 0,
            "triggered_arm": False,
        }
        # 兼容: 传 mm_per_norm_base=30.0 即走 depth-aware; 否则用 mm_per_norm (旧 P)
        use_depth = mm_per_norm_base is not None

        def _on_push(raw: dict) -> None:
            if stop_event.is_set():
                return
            ts = raw.get("task_state") or {}
            updated_at = ts.get("updated_at")
            if updated_at is not None and updated_at == state["last_updated_at"]:
                return
            state["last_updated_at"] = updated_at
            now = time.time()
            dt = max(1e-3, now - state["last_t"]) if state["last_t"] > 0 else 0.033
            state["last_t"] = now
            try:
                dets = _parse_cache(raw)
            except Exception:
                return
            cur_sel = state["current_selector"]
            candidates = [d for d in dets if cur_sel.matches(d)]
            if cur_sel.strategy == SelectionStrategy.LOCK_FIRST_SEEN.value:
                if state["locked_track_id"] is None:
                    pick = cur_sel.apply_strategy(candidates)
                    if pick is None:
                        state["consecutive_misses"] += 1
                        if on_missing_track == "abort" and state["consecutive_misses"] >= 5:
                            abort_reason["reason"] = "miss_abort"
                            stop_event.set()
                        return
                    state["locked_track_id"] = pick.track_id
                    cur_sel = dataclasses.replace(cur_sel, track_id=pick.track_id)
                    state["current_selector"] = cur_sel
                candidates = [d for d in candidates if d.track_id == state["locked_track_id"]]
            elif cur_sel.track_id is not None:
                candidates = [d for d in candidates if d.track_id == cur_sel.track_id]

            pick = cur_sel.apply_strategy(candidates) if candidates else None
            if pick is None:
                state["consecutive_misses"] += 1
                if on_missing_track == "abort" and state["consecutive_misses"] >= 5:
                    abort_reason["reason"] = "miss_abort"
                    stop_event.set()
                return
            state["consecutive_misses"] = 0
            state["last_detection"] = pick
            dx_norm = pick.bbox_norm.x_center - setpoint_x_norm
            dy_norm = pick.bbox_norm.y_center - setpoint_y_norm
            if abs(dx_norm) <= settle_tol_norm and abs(dy_norm) <= settle_tol_norm:
                state["consecutive_settle"] += 1
                if state["consecutive_settle"] >= settle_stable_frames:
                    abort_reason["reason"] = "converged"
                    stop_event.set()
                return
            state["consecutive_settle"] = 0

            # 4DOF 大偏移 → 大臂转 (一次性)
            if (not state["triggered_arm"] and on_strategic_4dof is not None
                    and abs(dx_norm) > arm_dx_threshold_norm):
                try:
                    on_strategic_4dof("arm_rotate", pick)
                except Exception as exc:
                    logger.warning("on_strategic_4dof arm_rotate 异常: %s", exc)
                state["triggered_arm"] = True
                return

            # depth-aware gain
            if use_depth:
                mm_per_norm_eff = self._adaptive_gain(
                    pick.bbox_pixels, target_real_height_m or 0.0,
                    focal_length_px, mm_per_norm_base or 30.0, ref_depth_m)
                # PID
                if ki > 0 or kd > 0:
                    out_x, state["int_err_x"], state["last_err_x"] = _pid_step(
                        dx_norm, state["int_err_x"], state["last_err_x"], dt,
                        kp, ki, kd)
                    out_y, state["int_err_y"], state["last_err_y"] = _pid_step(
                        dy_norm, state["int_err_y"], state["last_err_y"], dt,
                        kp, ki, kd)
                    dx_mm = -out_x * mm_per_norm_eff
                    dy_mm = -out_y * mm_per_norm_eff
                else:
                    # 纯 P (depth-aware)
                    dx_mm = -dx_norm * mm_per_norm_eff
                    dy_mm = -dy_norm * mm_per_norm_eff
            else:
                # 旧 P 路径
                dx_mm = -dx_norm * mm_per_norm
                dy_mm = -dy_norm * mm_per_norm
            if abs(dx_mm) < min_step_mm: dx_mm = 0.0
            if abs(dy_mm) < min_step_mm: dy_mm = 0.0
            new_x = state["x_mm"] + dx_mm
            new_y = state["y_mm"] + dy_mm
            if move_fn is not None:
                move_fn(new_x, new_y)
            else:
                self.http.execute_arm_action(
                    "goto_position",
                    x=new_x / 1000.0, y=new_y / 1000.0,
                    timeout=5.0, sync=False,
                )
            state["x_mm"], state["y_mm"] = new_x, new_y

        stop = ws.subscribe_task_detection(_on_push, hz=hz)
        t0 = time.time()
        try:
            stop_event.wait(timeout=timeout)
            elapsed = time.time() - t0
        finally:
            try:
                stop()
            except Exception:
                pass
        last = state["last_detection"]
        approx_iter = max(1, int(elapsed * 30.0))
        stable = (state["consecutive_settle"] >= settle_stable_frames)
        if abort_reason["reason"] == "miss_abort":
            raise RuntimeError(
                f"find_target_realtime: 连续 {state['consecutive_misses']} 帧未检测到 {selector}"
            )
        if abort_reason["reason"] == "converged" and last is not None:
            return ServoResult(
                converged=True, selector=selector,
                x_mm=state["x_mm"], y_mm=state["y_mm"],
                confidence=last.score, iterations=approx_iter,
                elapsed_s=elapsed, final_detection=last, trace=(),
                settle_stable=stable,
            )
        return ServoResult(
            converged=False, selector=selector,
            x_mm=state["x_mm"], y_mm=state["y_mm"],
            confidence=last.score if last else 0.0,
            iterations=approx_iter, elapsed_s=elapsed,
            final_detection=last, trace=(),
            settle_stable=False,
        )

    def find_target_track(self, selector, *,
                          x_mm: float, y_mm: float,
                          hz: float = 30.0,
                          mm_per_norm: float = 30.0,
                          settle_tol_norm: float = 0.10,
                          setpoint_x_norm: float = 0.0,
                          setpoint_y_norm: float = 0.0,
                          min_step_mm: float = 1.0,
                          max_iter: int = 500,
                          timeout: float = 30.0,
                          on_missing_track: str = "wait",
                          # 2026-08-01 升级可选
                          mm_per_norm_base: Optional[float] = None,
                          target_real_height_m: Optional[float] = None,
                          focal_length_px: float = 600.0,
                          ref_depth_m: float = 0.30,
                          kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                          arm_dx_threshold_norm: float = 0.3,
                          on_strategic_4dof: Optional[Callable[[str, Detection], None]] = None,
                          move_fn: Optional[Callable[[float, float], dict]] = None,
                          ws=None) -> ServoResult:
        ws = self._ensure_ws(ws)
        stop_event = threading.Event()
        state = {
            "x_mm": x_mm, "y_mm": y_mm,
            "last_detection": None,
            "consecutive_misses": 0,
            "locked_track_id": None,
            "current_selector": selector,
            "last_updated_at": None,
            "iter_count": 0,
            "int_err_x": 0.0, "int_err_y": 0.0,
            "last_err_x": 0.0, "last_err_y": 0.0,
            "last_t": 0.0,
            "triggered_arm": False,
        }
        use_depth = mm_per_norm_base is not None

        def _on_push(raw: dict) -> None:
            if stop_event.is_set():
                return
            ts = raw.get("task_state") or raw.get("data") or raw
            if "detections" not in ts:
                ts = raw
            updated_at = ts.get("updated_at")
            if updated_at is not None and updated_at == state["last_updated_at"]:
                return
            state["last_updated_at"] = updated_at
            now = time.time()
            dt = max(1e-3, now - state["last_t"]) if state["last_t"] > 0 else 0.033
            state["last_t"] = now
            try:
                dets = _parse_cache(raw)
            except Exception:
                return
            cur_sel = state["current_selector"]
            candidates = [d for d in dets if cur_sel.matches(d)]
            if cur_sel.strategy == SelectionStrategy.LOCK_FIRST_SEEN.value:
                if state["locked_track_id"] is None:
                    pick = cur_sel.apply_strategy(candidates)
                    if pick is None:
                        state["consecutive_misses"] += 1
                        if on_missing_track == "abort" and state["consecutive_misses"] >= 5:
                            stop_event.set()
                        return
                    state["locked_track_id"] = pick.track_id
                    cur_sel = dataclasses.replace(cur_sel, track_id=pick.track_id)
                    state["current_selector"] = cur_sel
                candidates = [d for d in candidates if d.track_id == state["locked_track_id"]]
            elif cur_sel.track_id is not None:
                candidates = [d for d in candidates if d.track_id == cur_sel.track_id]

            pick = cur_sel.apply_strategy(candidates) if candidates else None
            if pick is None:
                state["consecutive_misses"] += 1
                if on_missing_track == "abort" and state["consecutive_misses"] >= 5:
                    stop_event.set()
                return
            state["consecutive_misses"] = 0
            state["last_detection"] = pick
            state["iter_count"] += 1

            dx_norm = pick.bbox_norm.x_center - setpoint_x_norm
            dy_norm = pick.bbox_norm.y_center - setpoint_y_norm
            if (not state["triggered_arm"] and on_strategic_4dof is not None
                    and abs(dx_norm) > arm_dx_threshold_norm):
                try:
                    on_strategic_4dof("arm_rotate", pick)
                except Exception as exc:
                    logger.warning("on_strategic_4dof arm_rotate 异常: %s", exc)
                state["triggered_arm"] = True

            if use_depth:
                mm_per_norm_eff = self._adaptive_gain(
                    pick.bbox_pixels, target_real_height_m or 0.0,
                    focal_length_px, mm_per_norm_base or 30.0, ref_depth_m)
                if ki > 0 or kd > 0:
                    out_x, state["int_err_x"], state["last_err_x"] = _pid_step(
                        dx_norm, state["int_err_x"], state["last_err_x"], dt,
                        kp, ki, kd)
                    out_y, state["int_err_y"], state["last_err_y"] = _pid_step(
                        dy_norm, state["int_err_y"], state["last_err_y"], dt,
                        kp, ki, kd)
                    dx_mm = -out_x * mm_per_norm_eff
                    dy_mm = -out_y * mm_per_norm_eff
                else:
                    dx_mm = -dx_norm * mm_per_norm_eff
                    dy_mm = -dy_norm * mm_per_norm_eff
            else:
                dx_mm = -dx_norm * mm_per_norm
                dy_mm = -dy_norm * mm_per_norm
            if abs(dx_mm) < min_step_mm: dx_mm = 0.0
            if abs(dy_mm) < min_step_mm: dy_mm = 0.0
            new_x = state["x_mm"] + dx_mm
            new_y = state["y_mm"] + dy_mm
            if move_fn is not None:
                move_fn(new_x, new_y)
            else:
                self.http.execute_arm_action(
                    "goto_position",
                    x=new_x / 1000.0, y=new_y / 1000.0,
                    timeout=5.0, sync=False,
                )
            state["x_mm"], state["y_mm"] = new_x, new_y

        stop = ws.subscribe_task_detection(_on_push, hz=hz)
        t0 = time.time()
        try:
            deadline = t0 + timeout
            while time.time() < deadline and state["iter_count"] < max_iter:
                if stop_event.is_set():
                    break
                if stop_event.wait(timeout=0.05):
                    break
            elapsed = time.time() - t0
        finally:
            try:
                stop()
            except Exception:
                pass

        last = state["last_detection"]
        if stop_event.is_set() and state["consecutive_misses"] >= 5:
            raise RuntimeError(
                f"find_target_track: 连续 {state['consecutive_misses']} 帧未检测到 {selector}"
            )
        return ServoResult(
            converged=False, selector=selector,
            x_mm=state["x_mm"], y_mm=state["y_mm"],
            confidence=last.score if last else 0.0,
            iterations=state["iter_count"], elapsed_s=elapsed,
            final_detection=last, trace=(),
        )
