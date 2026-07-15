#!/usr/bin/python3
"""01_calibrate_origin.py
触发车端 arm.reset_position 重新定原点（漂移恢复工具）。

注意：
  - 首次上电通常不需要手跑这个脚本 —— runtime 启动时若 RAK_CAR_RESET_ARM=1
    （见 ecosystem.config.js:23），会自动跑一次 reset 并落盘 arm_origin.yaml。
  - 这个脚本只在"机械臂漂移严重 / PID 范围卡死 / 编码器读数不对"时手动调一下。
  - 旧版本需要按 4 键手动 jog（1=y 下，3=y 上，2=x 左，4=x 右）+ 同时按 1+3
    1 秒保存 —— **该流程已删除**。

用法：
    python3 main/arm/examples/01_calibrate_origin.py left    # x 撞左墙
    python3 main/arm/examples/01_calibrate_origin.py right   # x 撞右墙
"""
from __future__ import annotations

import os
import sys


def main():
    x_wall = sys.argv[1] if len(sys.argv) > 1 else "left"
    if x_wall not in ("left", "right"):
        print(f"参数错误：x_wall 必须是 left/right，收到 {x_wall!r}")
        sys.exit(1)

    # 把项目根目录加进 sys.path，便于 `from main.arm.origin import ...`
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

    from main.arm.origin import OriginCalibrator  # noqa: E402
    from main.api_client import RuntimeApiClient  # noqa: E402

    http = RuntimeApiClient()
    http.wait_until_ready()
    origin = OriginCalibrator(http).run(x_wall=x_wall)
    if origin is None:
        print("未保存。")
        sys.exit(1)
    print(f"\n[done] origin = y:{origin.y_origin_m:.5f}m x:{origin.x_origin_m:.5f}m wall:{origin.x_wall}")


if __name__ == "__main__":
    main()
