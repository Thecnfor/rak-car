"""main/arm/vision/servo.py — find_target HTTP 轮询主路径 (纯 P 兼容版)."""
from __future__ import annotations

import dataclasses
import logging
import time
from typing import Callable, List, Optional

from .parsers import _parse_cache
from .selector import SelectionStrategy, TargetSelector
from .types import Detection, ServoResult, ServoTrace

logger = logging.getLogger(__name__)


class ServoLoop:
    """HTTP 轮询主路径 mixin.

    本 mixin 提供 find_target_legacy (纯 P, 旧行为) — 业务层默认走 PID 升级版
    在 realtime.py / 后续算法升级. 此处保留旧入口用于回归测试.
    """

    def find_target_legacy(self, selector: TargetSelector, *,
                            x_mm: float, y_mm: float,
                            mm_per_norm: float = 30.0,
                            settle_tol_norm: float = 0.05,
                            min_step_mm: float = 1.0,
                            max_iter: int = 500,
                            timeout: float = 10.0,
                            on_missing_track: str = "abort",
                            move_fn: Optional[Callable[[float, float], dict]] = None) -> ServoResult:
        t0 = time.time()
        trace: List[ServoTrace] = []
        locked_track_id: Optional[int] = None
        consecutive_misses = 0
        last_x_mm, last_y_mm = x_mm, y_mm
        last_detection: Optional[Detection] = None
        current_selector = selector
        for i in range(max_iter):
            if time.time() - t0 > timeout:
                break
            candidates = self.get_state_filtered(current_selector)
            if current_selector.strategy == SelectionStrategy.LOCK_FIRST_SEEN.value:
                if locked_track_id is None:
                    pick = current_selector.apply_strategy(candidates)
                    if pick is None:
                        consecutive_misses += 1
                        if consecutive_misses >= 5 and on_missing_track == "abort":
                            raise RuntimeError(f"find_target_legacy: 首帧未检测到 {current_selector}")
                        continue
                    locked_track_id = pick.track_id
                    current_selector = dataclasses.replace(current_selector, track_id=locked_track_id)
                candidates = [d for d in candidates if d.track_id == locked_track_id]
            elif current_selector.track_id is not None:
                candidates = [d for d in candidates if d.track_id == current_selector.track_id]
            pick = current_selector.apply_strategy(candidates) if candidates else None
            if pick is None:
                consecutive_misses += 1
                trace.append(ServoTrace(t_s=time.time() - t0, iteration=i, dx_norm=0.0, dy_norm=0.0,
                                        x_mm=last_x_mm, y_mm=last_y_mm, score=0.0,
                                        selected_track_id=None, is_miss=True))
                if on_missing_track == "abort" and consecutive_misses >= 5:
                    raise RuntimeError(f"find_target_legacy: 连续 {consecutive_misses} 帧未检测到 {current_selector}")
                continue
            consecutive_misses = 0
            last_detection = pick
            dx_norm, dy_norm = pick.bbox_norm.x_center, pick.bbox_norm.y_center
            if pick.bbox_norm.is_centered_at(settle_tol_norm):
                trace.append(ServoTrace(t_s=time.time() - t0, iteration=i, dx_norm=dx_norm, dy_norm=dy_norm,
                                        x_mm=last_x_mm, y_mm=last_y_mm, score=pick.score,
                                        selected_track_id=pick.track_id))
                return ServoResult(converged=True, selector=current_selector,
                                   x_mm=last_x_mm, y_mm=last_y_mm, confidence=pick.score,
                                   iterations=i + 1, elapsed_s=time.time() - t0,
                                   final_detection=pick, trace=tuple(trace))
            dx_mm = -dx_norm * mm_per_norm
            dy_mm = -dy_norm * mm_per_norm
            if abs(dx_mm) < min_step_mm: dx_mm = 0.0
            if abs(dy_mm) < min_step_mm: dy_mm = 0.0
            new_x_mm, new_y_mm = last_x_mm + dx_mm, last_y_mm + dy_mm
            trace.append(ServoTrace(t_s=time.time() - t0, iteration=i, dx_norm=dx_norm, dy_norm=dy_norm,
                                    x_mm=new_x_mm, y_mm=new_y_mm, score=pick.score,
                                    selected_track_id=pick.track_id))
            if move_fn is not None:
                move_fn(new_x_mm, new_y_mm)
            else:
                self.http.execute_arm_action("goto_position",
                    x=new_x_mm / 1000.0, y=new_y_mm / 1000.0, timeout=5.0, sync=True)
            last_x_mm, last_y_mm = new_x_mm, new_y_mm
        return ServoResult(converged=False, selector=current_selector,
                           x_mm=last_x_mm, y_mm=last_y_mm,
                           confidence=last_detection.score if last_detection else 0.0,
                           iterations=max_iter, elapsed_s=time.time() - t0,
                           final_detection=last_detection, trace=tuple(trace))

    # ---- T3 之前暂用 legacy 当 find_target, 保持现有业务代码 / tests 通过 ----
    # T3 算法升级会替换为 PID+depth+4DOF 版本, 但 find_target_legacy 仍保留.
    def find_target(self, selector, **kwargs):
        return self.find_target_legacy(selector, **kwargs)

    def find_targets_sequence(self, selectors, *, x_mm, y_mm, **kwargs):
        """按顺序对每个 selector 调 find_target; 返回 list of ServoResult."""
        return [self.find_target(sel, x_mm=x_mm, y_mm=y_mm, **kwargs) for sel in selectors]

    def pick_one(self, selectors, *, x_mm, y_mm, **kwargs):
        """按优先级找第一个能匹配的目标并伺服; 返回首个成功的 ServoResult 或 None."""
        for sel in selectors:
            try:
                result = self.find_target(sel, x_mm=x_mm, y_mm=y_mm, **kwargs)
                if result.converged:
                    return result
            except RuntimeError:
                continue
        return None
