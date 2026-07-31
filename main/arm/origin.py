"""main/arm/origin.py
原点标定：不再用 4 键手动 jog，直接调车端 arm.reset_position 触发原点。

约定（与车端 arm_base.py 一致）：y<0=向上，y>0=向下（朝触底）。
reset_position 会主动朝下找磁感应，触发后把当前编码器值作为 0 原点。

行为变更（2026-07-16 merge origin/main 后）：
- reset_x 已删除，x 轴无软件复位
- reset_position 现在只做 y 触底定原点；x_origin_m 固定为 0.0
- x 位置管理 = 视觉闭环 + realtime 读取（ARM_API.md §11）

注意：
  - runtime 启动时（RESET_ARM=1）会自动跑一次 reset_position，业务无需手动调。
  - 这个工具只在"机械臂漂移严重、PID 范围卡死"时手动调用一下。
  - **不要再调 `reset_x` / `reset_all` 来"撞墙定原点"** —— calibrate 框架有 bug
    （详见 ARM_API.md §9）。如必须，参考 `main/arm/test/aaa_origin.py` 走 4 步并行+
    串行（透传 `probe_time=0` 关闭反向探针）。
"""
from __future__ import annotations

import time
from typing import Optional

try:
    from main.api_client import RuntimeApiClient
except ImportError:  # pragma: no cover — 直接 `python origin.py` 时无包上下文，补 sys.path
    import os as _os
    import sys as _sys
    # dirname(origin.py)=main/arm → 上两级=仓库根，加进去后 `main.*` 可导入
    _root = _os.path.abspath(_os.path.join(_os.path.dirname(__file__), "..", ".."))
    if _root not in _sys.path:
        _sys.path.insert(0, _root)
    from main.api_client import RuntimeApiClient  # type: ignore

try:
    from .state import ArmOrigin
except ImportError:  # pragma: no cover — 直接运行时用绝对导入（sys.path 已含仓库根）
    from main.arm.state import ArmOrigin  # type: ignore

try:
    # 初始化前预操作：关储存仓舵机
    from .api import pre_init_close_storage
except ImportError:  # pragma: no cover — 直接运行时用绝对导入
    from main.arm.api import pre_init_close_storage  # type: ignore


class OriginCalibrator:
    """调一次车端 reset_position，让 y 重新触底定原点。"""

    def __init__(self, http: RuntimeApiClient):
        self.http = http

    def run(self, x_wall: str = "left", timeout: float = 30.0) -> Optional[ArmOrigin]:
        """阻塞：主动让车端 reset_y 触底，回写 arm_origin.yaml。

        reset_x 已删除（2026-07-16），x 位置由视觉闭环控制。

        行为变更（2026-07-18）：
          - 触发 reset_position **之前**先调 `pre_init_close_storage()` 把储存仓
            舵机打到关闭位（98°），避免开仓状态干扰磁感找底。
          - 预操作失败 warn 但不阻塞 init 主流程。
        """
        if x_wall not in ("left", "right"):
            raise ValueError("x_wall 必须是 'left' 或 'right'")
        # ---- 初始化前预操作：关储存仓舵机 ----
        # 任何 init 入口前都应先关仓（user 2026-07-18 要求）；不动 y 轴（忽略
        # test_storage_close.py 里的 y=-150 抬升，那是测试脚本的临时 workaround）。
        print("=== [init-pre] 关储存仓舵机（98°）===")
        pre_init_close_storage(self.http, timeout=10.0)
        print("=== 触发车端 reset_position 重新触底定原点 ===")
        try:
            # sync=True 必传：/v1/execute 默认异步会立即返回 status=queued/result=None，
            # 必须阻塞轮询到 succeeded 才能保证 reset 真的跑完、后续读 y 有值。
            job = self.http.execute_arm_action("reset_position", timeout=timeout, sync=True)
            print(f"reset_position 完成: {job.get('status')}")
        except Exception as exc:
            raise RuntimeError(f"reset_position 失败: {exc}")
        # 读一次 y 原始坐标作为新原点（x 固定为 0）
        # 2026-07-31 修复：原来调 y_get_position 走底层 calibrate 框架（已坏，读数飘
        # 0.3/22.5/46.9mm，且 timeout=10s 经常被打爆 —— 见 ARM_API §11）。改走
        # /v1/realtime/arm/state（arm_feed 20Hz 守护线程缓存，不进 job_queue、
        # 不打 ZMQ、不抢 car_lock，是业务层**唯一**可信 x/y 位置源）。
        try:
            state_resp = self.http.get_arm_state()
            arm_state = state_resp.get("arm_state", {}) if isinstance(state_resp, dict) else {}
            y_mm = arm_state.get("y_mm")
            if y_mm is None:
                raise RuntimeError(
                    f"realtime arm/state 无 y_mm (active={arm_state.get('active')})—— "
                    "arm_feed 守护线程可能未启动，或刚刚 reset_stop"
                )
            y_val = float(y_mm) / 1000.0
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


if __name__ == "__main__":
    # 允许直接 `python main/arm/origin.py [left|right]`（等价于 examples/01_calibrate_origin.py）。
    # 注意：会真的触发车端 reset_position，需 runtime 可达。
    import sys

    wall = sys.argv[1] if len(sys.argv) > 1 else "left"
    if wall not in ("left", "right"):
        print(f"参数错误：x_wall 必须是 left/right，收到 {wall!r}")
        sys.exit(1)
    _origin = run_calibrator(x_wall=wall)
    sys.exit(0 if _origin else 1)
