#!/usr/bin/python3
"""01_calibrate_origin.py
4 键手动硬定原点。

用法：
    python3 main/arm/examples/01_calibrate_origin.py left    # 撞左墙
    python3 main/arm/examples/01_calibrate_origin.py right   # 撞右墙
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
