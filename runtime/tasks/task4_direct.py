#!/usr/bin/env python3
"""任务四底层直连实现：不经过 main、HTTP 或 WebSocket。"""
from __future__ import annotations

import argparse
import time
from typing import Any, Dict, Optional


class Task4Direct:
    """用 MyCar 直接完成搜索、抓取和放置。"""

    def __init__(self, car: Any, *, max_seconds: float = 120.0,
                 creep_m: float = 0.8, creep_speed_mps: float = 0.12):
        self.car = car
        self.max_seconds = max_seconds
        self.creep_m = creep_m
        self.creep_speed_mps = creep_speed_mps
        self.picked = 0
        self._start = time.monotonic()
        self._start_x = self._odom_x()

    def _odom_x(self) -> float:
        value = self.car.get_odometry()
        if isinstance(value, dict):
            value = value.get("x", value.get("odom_x", 0.0))
        if hasattr(value, "__getitem__") and not isinstance(value, str):
            value = value[0]
        try:
            return float(value)
        except (TypeError, ValueError, IndexError):
            return 0.0

    def _walked(self) -> float:
        return abs(self._odom_x() - self._start_x)

    def _stopped(self) -> bool:
        return bool(getattr(self.car, "_must_exit", lambda: False)())

    def _near_ir(self) -> bool:
        state = self.car.get_all_ir_distance()
        if isinstance(state, dict):
            values = state.values()
        else:
            values = state
        try:
            return any(0.0 < float(v) <= 0.7 for v in values if v not in (None, "", "---"))
        except (TypeError, ValueError):
            return False

    def _target(self) -> Optional[list]:
        detections = self.car.get_detection_results(sort_pos=(0, 0)) or []
        for detection in detections:
            if isinstance(detection, (list, tuple)) and len(detection) >= 8:
                label = str(detection[2]).lower()
                if label in {"ball", "crop", "fruit", "tomato"}:
                    return detection
        return None

    def _pick_and_place(self, target: list) -> None:
        label = target[2]
        grasped = False
        try:
            self.car.move_to_detection_target(label=label, time_out=2.0)
            self.car.arm.move_y_position(-180)
            self.car.arm.move_x_position(-30)
            self.car.arm.grasp(True)
            grasped = True
            self.car.arm.move_y_position(0)
            self.car.arm.move_x_position(0)
            self.picked += 1
        finally:
            if grasped:
                try:
                    self.car.arm.grasp(False)
                except Exception:
                    pass

    def run(self) -> Dict[str, Any]:
        reason = "timeout"
        try:
            while time.monotonic() - self._start < self.max_seconds:
                if self._stopped():
                    reason = "stopped"
                    break
                target = self._target()
                if target is not None and self._near_ir():
                    self._pick_and_place(target)
                    reason = "picked"
                    break
                if self._walked() >= self.creep_m:
                    reason = "search_distance_exhausted"
                    break
                self.car.move_for([self.creep_speed_mps * 0.1, 0.0, 0.0], stop=False)
                time.sleep(0.1)
        finally:
            self.car.move_for([0.0, 0.0, 0.0], stop=True)
        return {"ok": self.picked > 0, "picked": self.picked,
                "reason": reason, "elapsed_s": time.monotonic() - self._start}


def run(car: Any = None, *, dry_run: bool = False, max_seconds: float = 120.0) -> Dict[str, Any]:
    if dry_run:
        return {"ok": True, "picked": 0, "reason": "dry_run", "elapsed_s": 0.0}
    owned = car is None
    if owned:
        from runtime.services.my_car import MyCar
        car = MyCar()
    try:
        return Task4Direct(car, max_seconds=max_seconds).run()
    finally:
        if owned:
            car.close()


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="task4 底层直连执行")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=120.0)
    args = parser.parse_args(argv)
    result = run(dry_run=args.dry_run, max_seconds=args.max_seconds)
    print(result)
    return 0 if result["ok"] or args.dry_run else 1


if __name__ == "__main__":
    raise SystemExit(main())
