"""RobotState — synthesized in-memory robot state.

The /api/state/* and /api/sensors/* endpoints read from this. The state
animates over time (the pose drifts as if the robot were moving, the
battery slowly drains, the IR distances oscillate) so the UI has
something to render even when no real commands are coming in.

When the real MyCar backend is wired in, this module is the place to
swap to a thin adapter over `MyCar.get_odometry()` etc.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, asdict
from typing import Optional


@dataclass
class Pose:
    x: float
    y: float
    theta: float  # radians


@dataclass
class Velocity:
    vx: float
    vy: float
    w: float


class RobotState:
    """Thread-safe synthetic state. Animates pose + battery + IR with
    smooth motion, can be poked via `apply_command(action, params)`."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._t0 = time.time()
        self._pose = Pose(0.0, 0.0, 0.0)
        self._velocity = Velocity(0.0, 0.0, 0.0)
        self._battery_v: float = 12.0
        self._last_command_at: float = 0.0
        self._last_key: str = ""
        self._last_key_at: float = 0.0

    # -- core reads -----------------------------------------------------

    def snapshot(self) -> dict:
        with self._lock:
            self._tick()
            return {
                "pose": asdict(self._pose),
                "velocity": asdict(self._velocity),
                "battery_v": round(self._battery_v, 2),
                "battery_pct": round(self._battery_pct(), 1),
                "uptime_s": round(time.time() - self._t0, 1),
            }

    def pose(self) -> Pose:
        with self._lock:
            self._tick()
            return Pose(self._pose.x, self._pose.y, self._pose.theta)

    def velocity(self) -> Velocity:
        with self._lock:
            self._tick()
            return Velocity(self._velocity.vx, self._velocity.vy, self._velocity.w)

    def battery(self) -> dict:
        with self._lock:
            self._tick()
            return {
                "voltage_v": round(self._battery_v, 2),
                "percent": round(self._battery_pct(), 1),
            }

    def sensors(self) -> dict:
        """All sensors, polled-friendly."""
        with self._lock:
            self._tick()
            t = time.time()
            ir_l = 0.30 + 0.05 * math.sin(t * 1.3)
            ir_r = 0.30 + 0.05 * math.sin(t * 1.1 + 1.0)
            return {
                "ir_left_m": round(ir_l, 3),
                "ir_right_m": round(ir_r, 3),
                "last_key": self._last_key,
                "last_key_at": self._last_key_at,
                "battery_v": round(self._battery_v, 2),
                "uptime_s": round(time.time() - self._t0, 1),
            }

    def last_key(self) -> dict:
        with self._lock:
            return {"key": self._last_key, "at": self._last_key_at}

    def record_key(self, key: str) -> dict:
        with self._lock:
            self._last_key = key
            self._last_key_at = time.time()
        return {"key": key, "at": self._last_key_at}

    def apply_command(self, action: str, params: Optional[dict] = None) -> None:
        """Poke the synthetic state. Lets the UI feel like it has effect."""
        with self._lock:
            self._last_command_at = time.time()
            p = params or {}
            if action == "move":
                self._velocity.vx = float(p.get("vx", 0.0))
                self._velocity.vy = float(p.get("vy", 0.0))
                self._velocity.w = float(p.get("w", 0.0))
            elif action == "stop":
                self._velocity.vx = 0.0
                self._velocity.vy = 0.0
                self._velocity.w = 0.0
            # arm_set and others: no synthetic effect yet

    # -- internals ------------------------------------------------------

    def _battery_pct(self) -> float:
        # map 9.0V -> 0%, 12.5V -> 100%
        v = self._battery_v
        return max(0.0, min(100.0, (v - 9.0) / (12.5 - 9.0) * 100.0))

    def _tick(self) -> None:
        """Integrate velocity into pose, decay velocity back to 0 over time."""
        now = time.time()
        if not hasattr(self, "_last_tick_t"):
            self._last_tick_t = now
            return
        dt = min(0.1, now - self._last_tick_t)
        self._last_tick_t = now

        # decay velocity back to 0 when nothing is commanding
        if now - self._last_command_at > 0.5:
            self._velocity.vx *= 0.92
            self._velocity.vy *= 0.92
            self._velocity.w *= 0.92

        # integrate
        cos_t = math.cos(self._pose.theta)
        sin_t = math.sin(self._pose.theta)
        self._pose.x += (self._velocity.vx * cos_t - self._velocity.vy * sin_t) * dt
        self._pose.y += (self._velocity.vx * sin_t + self._velocity.vy * cos_t) * dt
        self._pose.theta = (self._pose.theta + self._velocity.w * dt + math.pi) % (
            2 * math.pi
        ) - math.pi

        # battery drains 0.5%/min from 12.0V (so a full battery lasts ~5h)
        self._battery_v = max(9.0, 12.0 - (now - self._t0) * (12.0 - 9.0) / (60 * 60))


# Module-level singleton
STATE = RobotState()
