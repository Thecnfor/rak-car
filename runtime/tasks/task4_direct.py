#!/usr/bin/env python3
"""任务四底层直连实现：不经过 main、HTTP 或 WebSocket。"""
from __future__ import annotations

import argparse
import time
from typing import Any, Dict, Optional


class Task4Direct:
    """用 MyCar 直接完成 task4 的最小抓取-放置循环。"""

    P_POSE = {"x": -0.295, "y": -0.180, "arm": 90.0, "hand": 10.0}
    PICK_Y_M = -0.065
    TRANSIT_X_M = -0.220
    BIN_POSE = {
        "blue": {"x": 0.0, "y": -0.140, "hand": 10.0},
        "yellow": {"x": -0.060, "y": -0.065, "hand": 10.0},
    }

    def __init__(self, car: Any, *, max_seconds: float = 120.0,
                 creep_m: float = 0.8, creep_speed_mps: float = 0.20):
        self.car = car
        self.max_seconds = max_seconds
        self.creep_m = creep_m
        self.creep_speed_mps = creep_speed_mps
        self.picked = 0
        self._start = time.monotonic()
        self._last_x = self._odom_x()
        self._travelled = 0.0

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
        current_x = self._odom_x()
        self._travelled += abs(current_x - self._last_x)
        self._last_x = current_x
        return self._travelled

    def _stopped(self) -> bool:
        return bool(getattr(self.car, "_must_exit", lambda: False)())

    def _near_ir(self) -> bool:
        state = self.car.get_all_ir_distance()
        if isinstance(state, dict):
            values = state.values()
        else:
            values = state
        for value in values:
            if value in (None, "", "---"):
                continue
            try:
                if float(value) <= 0.7:
                    return True
            except (TypeError, ValueError):
                continue
        return False

    def _target(self) -> Optional[list]:
        detections = self.car.get_detection_results(sort_pos=(0, 0)) or []
        for detection in detections:
            if isinstance(detection, (list, tuple)) and len(detection) >= 8:
                label = str(detection[2]).lower()
                if label in {"blue", "blue_ball", "yellow", "yellow_ball", "ball", "crop", "fruit", "tomato"}:
                    return detection
        return None

    @staticmethod
    def _color(target: list) -> str:
        label = str(target[2]).lower()
        return "yellow" if "yellow" in label else "blue"

    def _composite(self, **kwargs) -> None:
        result = self.car.arm.composite_run(**kwargs)
        if isinstance(result, dict) and not result.get("ok", True):
            raise RuntimeError("机械臂 composite_run 失败: %s" % result)

    def _goto_p(self) -> None:
        pose = self.P_POSE
        self._composite(x=pose["x"], y=pose["y"], arm=pose["arm"], hand=pose["hand"],
                        speed=80, timeout=30.0)

    def _pick_and_place(self, target: list) -> None:
        color = self._color(target)
        grasped = False
        try:
            # 先用底盘/侧摄目标闭环完成一次对齐，再盲降抓取。
            self.car.move_to_detection_target(label=target[2], time_out=2.0)
            self._composite(y=self.PICK_Y_M, speed=80, timeout=15.0)
            self.car.arm.grasp(True)
            grasped = True
            self._composite(y=self.P_POSE["y"], x=self.TRANSIT_X_M,
                            speed=80, timeout=20.0)
            bin_pose = self.BIN_POSE[color]
            self._composite(x=bin_pose["x"], y=bin_pose["y"], hand=bin_pose["hand"],
                            speed=80, timeout=20.0)
            self.car.arm.grasp(False)
            grasped = False
            self.picked += 1
        finally:
            if grasped:
                try:
                    self.car.arm.grasp(False)
                except Exception:
                    pass

    def run(self) -> Dict[str, Any]:
        reason = "timeout"
        triggered = False
        try:
            # 先等任务触发，触发前不移动底盘，也不消耗 0.8m 任务距离。
            while time.monotonic() - self._start < self.max_seconds:
                if self._stopped():
                    reason = "stopped"
                    break
                if self._near_ir():
                    triggered = True
                    break
                time.sleep(0.05)
            if not triggered:
                return {"ok": False, "picked": self.picked, "reason": reason,
                        "elapsed_s": time.monotonic() - self._start}

            self._goto_p()
            target_done = False
            while time.monotonic() - self._start < self.max_seconds:
                if self._stopped():
                    reason = "stopped"
                    break
                if self._walked() >= self.creep_m:
                    reason = "distance_exhausted"
                    break
                target = self._target()
                if target is not None and not target_done:
                    self._pick_and_place(target)
                    target_done = True
                    reason = "picked"
                else:
                    self.car.move_for([self.creep_speed_mps * 0.1, 0.0, 0.0])
                    time.sleep(0.1)
        except Exception as exc:
            reason = "error:%s" % exc
        finally:
            self.car.stop()
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
