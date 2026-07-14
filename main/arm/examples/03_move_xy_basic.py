#!/usr/bin/python3
"""03_move_xy_basic.py
同步双轴移动到指定 (x, y)，会先打印 S 曲线 dry-run。

用法：
    python3 main/arm/examples/03_move_xy_basic.py 100 80
"""
from __future__ import annotations

import os
import sys
import json


def main():
    if len(sys.argv) < 3:
        x_mm = 100.0
        y_mm = 80.0
    else:
        x_mm = float(sys.argv[1])
        y_mm = float(sys.argv[2])

    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

    from main.arm.api import ArmClient  # noqa: E402
    from main.arm.loops import ArmRunner  # noqa: E402

    client = ArmClient.connect()
    if not client.ping():
        print("runtime 不在线")
        sys.exit(1)

    runner = ArmRunner(client)
    st_before = client.get_state()
    print(f"before: {st_before.describe()}")
    job = runner.move_xy(x_mm=x_mm, y_mm=y_mm)
    print(f"\nresult: {json.dumps(job, ensure_ascii=False, indent=2)}")
    st_after = client.get_state()
    print(f"after:  {st_after.describe()}")


if __name__ == "__main__":
    main()
