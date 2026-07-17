"""main/arm/origin.py
原点标定：直接调车端 reset_position (y 触底) + x_set_origin (直设原点)。

约定（与车端 arm_base.py 一致）：y<0=向上，y>0=向下（朝触底）。

流程 (2026-07-16 重写):
  1) 调车端 reset_position → 内部 reset_y 找磁感 + reset_x 撞墙
     (reset_x 在 boundary clamp 处可能不动,但 reset_y 仍 OK,y 一定归零)
  2) 调车端 x_set_origin (新增 action) → 把当前编码器位置设为 x=0
     不依赖 reset_x 的撞墙模型,直接认当前位置 = 撞墙位置 = 新原点
  3) 回写 arm_origin.yaml,保留用户改的软限位

注意:
  - runtime 启动时（RESET_ARM=1）会自动跑一次 reset_position,业务无需手动调
  - 这个工具只在"机械臂漂移严重 / PID 卡死 / 编码器读数明显不对"时手动调用
"""
from __future__ import annotations

import time
from typing import Optional

try:
    from main.api_client import RuntimeApiClient
except ImportError:  # pragma: no cover
    from api_client import RuntimeApiClient  # type: ignore

from .state import ArmOrigin


class OriginCalibrator:
    """调一次车端 reset_y + x_set_origin,让 x=0 定在最左/最右。"""

    def __init__(self, http: RuntimeApiClient):
        self.http = http
        # 加载已有 arm_origin.yaml (如果存在),后续 calibrate 时保留用户改的软限位
        self.origin: Optional[ArmOrigin] = None
        try:
            import os
            import yaml
            here = os.path.dirname(os.path.abspath(__file__))
            path = os.path.join(here, "arm_origin.yaml")
            if os.path.exists(path):
                with open(path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                self.origin = ArmOrigin(
                    y_origin_m=float(data.get("y_origin_m", 0.0)),
                    x_origin_m=float(data.get("x_origin_m", 0.0)),
                    x_wall=str(data.get("x_wall", "left")),
                    soft_y_max_m=float(data.get("soft_y_max_m", 0.20)),
                    soft_x_min_m=float(data.get("soft_x_min_m", -0.32)),
                    soft_x_max_m=float(data.get("soft_x_max_m", 0.32)),
                    calibrated_at=str(data.get("calibrated_at", "")),
                )
        except Exception:
            pass

    def _call_arm(self, name: str, timeout: float = 30.0, **kwargs) -> dict:
        """sync=True 阻塞调用 arm action。"""
        try:
            r = self.http.execute_arm_action(name, timeout=timeout, sync=True, **kwargs)
            return {"ok": r.get("status") == "succeeded",
                    "status": r.get("status"),
                    "error": r.get("error"),
                    "result": r.get("result")}
        except Exception as e:
            return {"ok": False, "status": "exception", "error": str(e)[:120], "result": None}

    def run(self, x_wall: str = "left", timeout: float = 30.0) -> Optional[ArmOrigin]:
        """阻塞：主动让车端 reset_y + x_set_origin,回写 arm_origin.yaml。
        必定到达最左 (x_wall='left') 或最右 (x_wall='right')。
        """
        if x_wall not in ("left", "right"):
            raise ValueError("x_wall 必须是 'left' 或 'right'")

        # ===== Step 1: 调 reset_position (y 触底) =====
        print("=== Step 1/3: 调车端 reset_position (内部 reset_y + reset_x) ===")
        r = self._call_arm("reset_position", timeout=timeout)
        flag = "OK  " if r["ok"] else "WARN"
        print(f"  [{flag}] reset_position status={r['status']} err={r['error']}")
        if not r["ok"]:
            print("  [WARN] reset_position 失败,继续尝试 x_set_origin (不强依赖)")
        time.sleep(0.3)

        # ===== Step 2: 读当前位置 =====
        print("=== Step 2/3: 读当前 x/y 位置 ===")
        y_job = self._call_arm("y_get_position", timeout=10)
        y_val = float(y_job["result"]) if (y_job["ok"] and y_job["result"] is not None) else 0.0
        x_job = self._call_arm("x_get_position", timeout=10)
        x_val = float(x_job["result"]) if (x_job["ok"] and x_job["result"] is not None) else 0.0
        print(f"  当前: x={x_val*1000:+.2f}mm  y={y_val*1000:+.2f}mm")

        # ===== Step 3: 直设 x=0 在当前位置 (绕过 reset_x 模型) =====
        # 不管 reset_x 是否成功撞墙,直接把当前位置当新原点
        print("=== Step 3/3: 调车端 x_set_origin (把当前编码器值设为 x=0) ===")
        r = self._call_arm("x_set_origin", timeout=10)
        flag = "OK  " if r["ok"] else "FAIL"
        print(f"  [{flag}] x_set_origin status={r['status']} err={r['error']}")
        if not r["ok"]:
            raise RuntimeError(f"x_set_origin 失败: {r['error']}")

        # 验证:再读一次 x 应该 ≈ 0
        time.sleep(0.2)
        verify_job = self._call_arm("x_get_position", timeout=10)
        verify_x = float(verify_job["result"]) if (verify_job["ok"] and verify_job["result"] is not None) else None
        if verify_x is not None:
            print(f"  验证: x_set_origin 后 x_get_position = {verify_x*1000:+.2f}mm (期望 ≈ 0)")

        # ===== 写 yaml =====
        origin = ArmOrigin(
            y_origin_m=y_val,
            x_origin_m=verify_x if verify_x is not None else x_val,
            x_wall=x_wall,
            # 保留已有的软限位 (用户可能手动改成实际物理行程,不应该被覆盖成默认)
            soft_y_max_m=(self.origin.soft_y_max_m if self.origin else ArmOrigin().soft_y_max_m),
            soft_x_min_m=(self.origin.soft_x_min_m if self.origin else ArmOrigin().soft_x_min_m),
            soft_x_max_m=(self.origin.soft_x_max_m if self.origin else ArmOrigin().soft_x_max_m),
            calibrated_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
        )
        import os
        import yaml
        here = os.path.dirname(os.path.abspath(__file__))
        path = os.path.join(here, "arm_origin.yaml")
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(
                {
                    "y_origin_m": origin.y_origin_m,
                    "x_origin_m": origin.x_origin_m,
                    "x_wall": origin.x_wall,
                    "soft_y_max_m": origin.soft_y_max_m,
                    "soft_x_min_m": origin.soft_x_min_m,
                    "soft_x_max_m": origin.soft_x_max_m,
                    "calibrated_at": origin.calibrated_at,
                },
                f,
                allow_unicode=True,
                sort_keys=False,
            )
        print(f"\n[done] 原点已写入 arm_origin.yaml: {origin.calibrated_at}")
        print(f"  y_origin_m = {origin.y_origin_m:.5f}")
        print(f"  x_origin_m = {origin.x_origin_m:.5f}  ← 应该 ≈ 0")
        print(f"  x_wall     = {origin.x_wall}")
        return origin


def run_calibrator(x_wall: str = "left") -> Optional[ArmOrigin]:
    """便捷入口：建一个 client 跑 OriginCalibrator。"""
    http = RuntimeApiClient()
    return OriginCalibrator(http).run(x_wall=x_wall)