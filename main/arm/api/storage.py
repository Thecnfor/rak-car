"""main/arm/api/storage.py — 存储仓舵机 + 真空吸 mixin (无 y 安全门, 2026-07-17 用户原话)."""
from __future__ import annotations

from ..state import STORAGE_SIDES


def _normalize_storage_side(side):
    if side is None:
        return None
    s = side.upper()
    if s not in STORAGE_SIDES:
        raise ValueError(f"storage side 必须是 {STORAGE_SIDES} 之一, 收到: {side!r}")
    return s


class StorageMixin:
    """set_storage / get_storage / set_storage_angle / grasp."""

    def set_storage(self, side: str, timeout: float = 10.0) -> dict:
        side = _normalize_storage_side(side)
        if side is None:
            raise ValueError(f"set_storage 必须给 {STORAGE_SIDES}")
        open_flag = side == "RIGHT"
        job = self._call_car("set_storage", timeout=timeout,
                             state=open_flag, sync=True)
        result = job.get("result") if isinstance(job, dict) else None
        out = {
            "ok": bool(isinstance(job, dict) and job.get("status") == "succeeded"),
            "side": None, "flag": None, "angle": None, "state": open_flag,
            "raw_job": job,
        }
        if isinstance(result, dict):
            r_side = str(result.get("side", "")).upper()
            if r_side in STORAGE_SIDES:
                out["side"] = r_side
            if "flag" in result:
                try:
                    out["flag"] = int(result["flag"])
                except (TypeError, ValueError):
                    pass
            if "angle" in result:
                try:
                    out["angle"] = int(result["angle"])
                except (TypeError, ValueError):
                    pass
        if out["side"] is None and out["ok"]:
            out["side"] = side
        if out["side"] in STORAGE_SIDES:
            self._storage_side_cache = out["side"]
        return out

    def get_storage(self) -> str:
        return getattr(self, "_storage_side_cache", "UNKNOWN")

    def grasp(self, on: bool, timeout: float = 10.0) -> dict:
        """真空吸 / 放（不移动任何电机, 无位置安全门）。

        runtime action "grasp" 透传 SDK arm_base.grasp(value):
          pump.set(not value); valve.set(value)  → True=吸气, False=放气.
        """
        job = self._call_arm("grasp", timeout=timeout, value=bool(on))
        return {
            "ok": bool(isinstance(job, dict) and job.get("status") == "succeeded"),
            "on": bool(on),
            "raw_job": job,
        }

    def set_storage_angle(self, angle: float, speed: int = 100,
                          timeout: float = 10.0) -> dict:
        job = self._call_car(
            "set_storage_angle", timeout=timeout,
            angle=angle, speed=speed, sync=True,
        )
        self._storage_side_cache = "UNKNOWN"
        return {
            "ok": bool(isinstance(job, dict) and job.get("status") == "succeeded"),
            "angle": float(angle),
            "raw_job": job,
        }
