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


class RealtimeLoop:
    """WS 推送路径 mixin. 需 self.http + 注入 self.ws (默认懒建)."""

    def _ensure_ws(self, ws):
        if ws is None:
            try:
                from main.ws_client import RuntimeWsClient
            except ImportError:  # pragma: no cover
                from ws_client import RuntimeWsClient  # type: ignore
            ws = RuntimeWsClient()
        return ws

    def find_target_realtime(self, selector, *,
                             x_mm: float, y_mm: float,
                             hz: float = 30.0,
                             mm_per_norm: float = 30.0,
                             settle_tol_norm: float = 0.05,
                             min_step_mm: float = 1.0,
                             timeout: float = 10.0,
                             on_missing_track: str = "abort",
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
        }

        def _on_push(raw: dict) -> None:
            if stop_event.is_set():
                return
            ts = raw.get("task_state") or {}
            updated_at = ts.get("updated_at")
            if updated_at is not None and updated_at == state["last_updated_at"]:
                return
            state["last_updated_at"] = updated_at
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
            dx_norm, dy_norm = pick.bbox_norm.x_center, pick.bbox_norm.y_center
            if pick.bbox_norm.is_centered_at(settle_tol_norm):
                abort_reason["reason"] = "converged"
                stop_event.set()
                return
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
            )
        return ServoResult(
            converged=False, selector=selector,
            x_mm=state["x_mm"], y_mm=state["y_mm"],
            confidence=last.score if last else 0.0,
            iterations=approx_iter, elapsed_s=elapsed,
            final_detection=last, trace=(),
        )

    def find_target_track(self, selector, *,
                          x_mm: float, y_mm: float,
                          hz: float = 30.0,
                          mm_per_norm: float = 30.0,
                          settle_tol_norm: float = 0.10,
                          min_step_mm: float = 1.0,
                          max_iter: int = 500,
                          timeout: float = 30.0,
                          on_missing_track: str = "wait",
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
        }

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

            dx_norm, dy_norm = pick.bbox_norm.x_center, pick.bbox_norm.y_center
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
