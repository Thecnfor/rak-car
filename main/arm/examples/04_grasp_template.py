#!/usr/bin/python3
"""04_grasp_template.py
完整 pick-and-place 模板：
  1) set_arm_angle(-90)   [业务硬限 [0, -150]°，LEFT=+93 已禁]
  2) move_xy 到抓取点
  3) grasp(True)
  4) set_hand(DOWN)
  5) move_xy 到放置点
  6) grasp(False)
  7) go_home

用法：
    python3 main/arm/examples/04_grasp_template.py
    python3 main/arm/examples/04_grasp_template.py -90  120 -40   0 30
"""
from __future__ import annotations

import os
import sys
import json


def main():
    args = sys.argv[1:]
    arm_angle = -90.0
    pick_x, pick_y = 120.0, -40.0
    drop_x, drop_y = 0.0, -30.0
    if len(args) >= 1:
        arm_angle = float(args[0])
    if len(args) >= 3:
        pick_x = float(args[1])
        pick_y = float(args[2])
    if len(args) >= 5:
        drop_x = float(args[3])
        drop_y = float(args[4])

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
    print(f"[1] set_arm_angle({arm_angle})")
    print(json.dumps(runner.set_arm_angle(arm_angle), ensure_ascii=False, indent=2))

    print(f"[2] move_xy -> ({pick_x}, {pick_y})")
    print(json.dumps(runner.move_xy(pick_x, pick_y), ensure_ascii=False, indent=2))

    print("[3] grasp(True)")
    print(json.dumps(runner.grasp(True), ensure_ascii=False, indent=2))

    print("[4] set_hand(DOWN)")
    print(json.dumps(runner.set_hand("DOWN"), ensure_ascii=False, indent=2))

    print(f"[5] move_xy -> ({drop_x}, {drop_y})")
    print(json.dumps(runner.move_xy(drop_x, drop_y), ensure_ascii=False, indent=2))

    print("[6] grasp(False)")
    print(json.dumps(runner.grasp(False), ensure_ascii=False, indent=2))

    print("[7] go_home")
    print(json.dumps(runner.go_home(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
