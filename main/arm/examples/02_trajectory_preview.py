#!/usr/bin/python3
"""02_trajectory_preview.py
只算 S 曲线不连车：打印 (x0,y0) -> (x1,y1) 的同步轨迹表。

用法：
    python3 main/arm/examples/02_trajectory_preview.py
    python3 main/arm/examples/02_trajectory_preview.py 0 0 100 80
"""
from __future__ import annotations

import os
import sys


def main():
    args = sys.argv[1:]
    if len(args) < 4:
        x0, y0, x1, y1 = 0.0, 0.0, 100.0, 80.0
    else:
        x0, y0, x1, y1 = (float(a) for a in args[:4])

    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

    from main.arm.trajectory import TrajectoryGenerator  # noqa: E402

    gen = TrajectoryGenerator()
    plan = gen.plan_xy(x0, y0, x1, y1)

    print(plan.describe())
    print()
    print(f"{'t(s)':>8} {'x(mm)':>10} {'y(mm)':>10} {'vx(mm/s)':>12} {'vy(mm/s)':>12}")
    print("-" * 56)
    # 每 5 行显示一个
    step = max(1, len(plan.samples) // 20)
    for i, s in enumerate(plan.samples):
        if i % step == 0 or i == len(plan.samples) - 1:
            print(f"{s.t_s:8.3f} {s.x_mm:10.2f} {s.y_mm:10.2f} {s.vx_mm_s:12.2f} {s.vy_mm_s:12.2f}")


if __name__ == "__main__":
    main()
