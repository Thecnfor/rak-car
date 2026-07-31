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
        # 兼容旧调用: 不传新参数走 legacy (纯 P, 单帧收敛, 无 settle_stable).
        # 业务层想用 PID+depth+4DOF 至少传一个 kp/ki/kd/target_real_height_m
        # 即触发新版算法 (新算法默认 ki>0, settle_stable 默认 3 帧).
        new_algo_keys = {"kp", "ki", "kd", "target_real_height_m",
                         "focal_length_px", "mm_per_norm_base", "ref_depth_m",
                         "settle_stable_frames", "arm_dx_threshold_norm",
                         "on_strategic_4dof"}
        if new_algo_keys & set(kwargs.keys()):
            return self.find_target_pid(selector, **kwargs)
        return self.find_target_legacy(selector, **kwargs)

    def find_target_pid(self, selector, *,
                         x_mm: float, y_mm: float,
                         mm_per_norm_base: float = 30.0,
                         ref_depth_m: float = 0.30,
                         focal_length_px: float = 600.0,
                         target_real_height_m: Optional[float] = None,
                         kp: float = 1.0, ki: float = 0.05, kd: float = 0.2,
                         settle_tol_norm: float = 0.05,
                         settle_stable_frames: int = 3,
                         min_step_mm: float = 1.0,
                         max_iter: int = 500,
                         timeout: float = 10.0,
                         on_missing_track: str = "abort",
                         arm_dx_threshold_norm: float = 0.3,
                         on_strategic_4dof: Optional[Callable[[str, "Detection"], None]] = None,
                         move_fn: Optional[Callable[[float, float], dict]] = None) -> ServoResult:
        """视觉伺服 PID+depth+4DOF 主路径 (2026-08-01 升级).

        算法:
          1. depth-aware adaptive gain: depth_m = compute_depth(bbox, real_height, focal)
             → mm_per_norm_eff = mm_per_norm_base * (depth_m / ref_depth_m)
          2. PID: out = kp*err + ki*∫err + kd*derr  (饱和限幅 ±1.0)
          3. 4 自由度策略: |dx_norm| > arm_dx_threshold_norm 时, 通过 on_strategic_4dof 回调
             触发大臂转 (业务层在回调里调 composite_run)
          4. 稳定收敛: 连续 settle_stable_frames 帧满足阈值才 converged=True + settle_stable=True
        """
        t0 = time.time()
        trace: List[ServoTrace] = []
        locked_track_id: Optional[int] = None
        consecutive_misses = 0
        last_x_mm, last_y_mm = x_mm, y_mm
        last_detection: Optional[Detection] = None
        current_selector = selector
        # PID state
        last_err_x, last_err_y = 0.0, 0.0
        int_err_x, int_err_y = 0.0, 0.0
        last_t = t0
        # settle stable
        consecutive_settle = 0
        # 4DOF 触发记录 (避免重复)
        triggered_arm = False

        for i in range(max_iter):
            now = time.time()
            if now - t0 > timeout:
                break
            dt = max(1e-3, now - last_t)
            last_t = now

            candidates = self.get_state_filtered(current_selector)
            if current_selector.strategy == SelectionStrategy.LOCK_FIRST_SEEN.value:
                if locked_track_id is None:
                    pick = current_selector.apply_strategy(candidates)
                    if pick is None:
                        consecutive_misses += 1
                        if consecutive_misses >= 5 and on_missing_track == "abort":
                            raise RuntimeError(f"find_target: 首帧未检测到 {current_selector}")
                        continue
                    locked_track_id = pick.track_id
                    current_selector = dataclasses.replace(current_selector, track_id=locked_track_id)
                candidates = [d for d in candidates if d.track_id == locked_track_id]
            elif current_selector.track_id is not None:
                candidates = [d for d in candidates if d.track_id == current_selector.track_id]

            pick = current_selector.apply_strategy(candidates) if candidates else None
            if pick is None:
                consecutive_misses += 1
                trace.append(ServoTrace(t_s=now - t0, iteration=i, dx_norm=0.0, dy_norm=0.0,
                                        x_mm=last_x_mm, y_mm=last_y_mm, score=0.0,
                                        selected_track_id=None, is_miss=True))
                if on_missing_track == "abort" and consecutive_misses >= 5:
                    raise RuntimeError(f"find_target: 连续 {consecutive_misses} 帧未检测到 {current_selector}")
                continue
            consecutive_misses = 0
            last_detection = pick

            dx_norm, dy_norm = pick.bbox_norm.x_center, pick.bbox_norm.y_center

            # ---- settle stable: 连续 N 帧满足阈值才收敛 ----
            if pick.bbox_norm.is_centered_at(settle_tol_norm):
                consecutive_settle += 1
                if consecutive_settle >= settle_stable_frames:
                    trace.append(ServoTrace(t_s=now - t0, iteration=i, dx_norm=dx_norm, dy_norm=dy_norm,
                                            x_mm=last_x_mm, y_mm=last_y_mm, score=pick.score,
                                            selected_track_id=pick.track_id))
                    return ServoResult(converged=True, selector=current_selector,
                                       x_mm=last_x_mm, y_mm=last_y_mm, confidence=pick.score,
                                       iterations=i + 1, elapsed_s=now - t0,
                                       final_detection=pick, trace=tuple(trace),
                                       settle_stable=True)
                # 未达稳定帧数, 继续追 (但不再下发 move)
                continue
            consecutive_settle = 0

            # ---- 4DOF 策略: 大偏移 → 大臂转 (一次) ----
            if (not triggered_arm and on_strategic_4dof is not None
                    and abs(dx_norm) > arm_dx_threshold_norm):
                try:
                    on_strategic_4dof("arm_rotate", pick)
                except Exception as exc:
                    logger.warning("on_strategic_4dof arm_rotate 异常: %s", exc)
                triggered_arm = True
                continue

            # ---- depth-aware adaptive gain ----
            if (target_real_height_m and target_real_height_m > 0
                    and pick.bbox_pixels is not None and pick.bbox_pixels.height > 0):
                depth_m = ArmVisionClient.compute_depth(
                    pick.bbox_pixels, target_real_height_m, focal_length_px)
                mm_per_norm_eff = mm_per_norm_base * (depth_m / ref_depth_m)
            else:
                mm_per_norm_eff = mm_per_norm_base

            # ---- PID ----
            err_x, err_y = dx_norm, dy_norm
            int_err_x = max(-1.0, min(1.0, int_err_x + err_x * dt))
            int_err_y = max(-1.0, min(1.0, int_err_y + err_y * dt))
            deriv_x = (err_x - last_err_x) / dt
            deriv_y = (err_y - last_err_y) / dt
            last_err_x, last_err_y = err_x, err_y
            out_x = kp * err_x + ki * int_err_x + kd * deriv_x
            out_y = kp * err_y + ki * int_err_y + kd * deriv_y
            out_x = max(-1.0, min(1.0, out_x))
            out_y = max(-1.0, min(1.0, out_y))

            dx_mm = -out_x * mm_per_norm_eff
            dy_mm = -out_y * mm_per_norm_eff
            if abs(dx_mm) < min_step_mm: dx_mm = 0.0
            if abs(dy_mm) < min_step_mm: dy_mm = 0.0

            new_x_mm = last_x_mm + dx_mm
            new_y_mm = last_y_mm + dy_mm
            trace.append(ServoTrace(t_s=now - t0, iteration=i, dx_norm=dx_norm, dy_norm=dy_norm,
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
                           final_detection=last_detection, trace=tuple(trace),
                           settle_stable=False)

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
