"""main/arm/origin.py
原点标定：不再用 4 键手动 jog，直接调车端 arm.reset_position 触发原点。

约定（与车端 arm_base.py 一致）：y<0=向上，y>0=向下（朝触底）。
reset_position 会主动朝下找磁感应，触发后把当前编码器值作为 0 原点。

行为变更（2026-07-16）：reset_x 已删除，x 轴无软件复位。reset_position
现在只做 y 触底定原点；x_origin_m 固定为 0.0。

注意：
  - runtime 启动时（RESET_ARM=1）会自动跑一次 reset_position，业务无需手动调。
  - 这个工具只在"机械臂漂移严重、PID 范围卡死"时手动调用一下。
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
    """调一次车端 reset_position，让 y 重新触底定原点。"""

    def __init__(self, http: RuntimeApiClient):
        self.http = http

    def run(self, x_wall: str = "left", timeout: float = 30.0) -> Optional[ArmOrigin]:
        """阻塞：主动让车端 reset_y 触底，回写 arm_origin.yaml。

        reset_x 已删除（2026-07-16），x 位置由视觉闭环控制。
        """
        if x_wall not in ("left", "right"):
            raise ValueError("x_wall 必须是 'left' 或 'right'")
        print("=== 触发车端 reset_position 重新触底定原点 ===")
        try:
            job = self.http.execute_arm_action("reset_position", timeout=timeout)
            print(f"reset_position 完成: {job.get('status')}")
        except Exception as exc:
            raise RuntimeError(f"reset_position 失败: {exc}")
        # 读一次 y 原始坐标作为新原点（x 固定为 0）
        try:
            y_job = self.http.execute_arm_action("y_get_position", timeout=10)
            y_val = float(y_job.get("result"))
        except Exception as exc:
            raise RuntimeError(f"读 y 失败: {exc}")

        origin = ArmOrigin(
            y_origin_m=y_val,
            x_origin_m=0.0,  # reset_x 已删除，x 固定为 0
            x_wall=x_wall,   # 保留字段兼容，但语义已无意义
            soft_y_max_m=ArmOrigin().soft_y_max_m,
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
                    "calibrated_at": origin.calibrated_at,
                },
                f,
                allow_unicode=True,
                sort_keys=False,
            )
        print(f"原点已写入 arm_origin.yaml: {origin.calibrated_at}")
        print(f"  y_origin_m = {origin.y_origin_m:.5f}")
        print(f"  x_origin_m = {origin.x_origin_m:.5f}（固定）")
        print(f"  x_wall     = {origin.x_wall}（仅历史标注）")
        return origin


def run_calibrator(x_wall: str = "left") -> Optional[ArmOrigin]:
    """便捷入口：建一个 client 跑 OriginCalibrator。"""
    http = RuntimeApiClient()
    return OriginCalibrator(http).run(x_wall=x_wall)
