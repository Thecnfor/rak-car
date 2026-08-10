#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""示例 13：姿势库 → goal→waypoint→goal 平滑 → FakeRobotSim 仿真。

用法（repo root）::

    PYTHONPATH=. python3 main/arm/examples/13_posture_route.py
    PYTHONPATH=. python3 main/arm/examples/13_posture_route.py --task 4 --route pose_p,pick,bin_blue,pose_p

演示：从 `main/arm/postures.yaml` 取命名姿势，串成闭环路线，plan_joint_trajectory
生成 4-DOF 平滑轨迹（每关键点精确经过、默认停车），再逐点喂给 FakeRobotSim
让关节真的动起来，最后打印轨迹描述 + 仿真末端姿态。

真机等价调用：把 `traj.dense_waypoints()` 逐点 composite_run 即可。
"""
from __future__ import annotations

import argparse
import sys

from main.arm.postures import load_postures


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        prog="13_posture_route",
        description="姿势库 → goal→waypoint→goal 平滑 → FakeRobotSim",
    )
    p.add_argument("--task", type=int, default=4, help="任务编号（姿势库键）")
    p.add_argument("--route", type=str,
                   default="pose_p,pick,bin_blue,pose_p",
                   help="逗号分隔的命名姿势序列")
    args = p.parse_args(argv)

    lib = load_postures()
    names = [s.strip() for s in args.route.split(",") if s.strip()]
    print(f"=== 姿势库 task{args.task} 路线: {' → '.join(names)} ===")
    route = lib.route(args.task, names)
    for pose in route:
        print(f"  {pose.to_dict()}")

    traj = lib.plan(args.task, names, close=False)
    print("\n=== 平滑轨迹 ===")
    print(traj.describe())

    dense = traj.dense_waypoints(spacing_mm=8.0)
    print(f"\n=== 重采样 {len(dense)} 个 composite_run 喂点 ===")
    for pose in dense[:6]:
        print(f"  composite_run(x_mm={pose.x_mm:.0f}, y_mm={pose.y_mm:.0f}, "
              f"arm={pose.arm_deg:.0f}, hand={pose.hand_deg:.0f}, "
              f"stop={pose.stop})")
    if len(dense) > 6:
        print(f"  ... 共 {len(dense)} 点, 关键点精确包含: "
              f"{[round(p.x_mm) for p in route]}")

    # FakeRobotSim 仿真：关节真的动起来
    from runtime.services.fake_robot import FakeRobotSim
    sim = FakeRobotSim()
    n = max(1, int(traj.total_time * 50))
    for i in range(n + 1):
        pose = traj.sample(traj.total_time * i / n)
        sim.composite_move({"x_mm": pose.x_mm, "y_mm": pose.y_mm,
                            "arm_angle": pose.arm_deg,
                            "hand_angle": pose.hand_deg})
    end = sim.arm_state_mm()
    print(f"\n=== 仿真末端姿态 (应为最后一个关键点) ===")
    print(f"  x={end['x_mm']:.1f}mm  y={end['y_mm']:.1f}mm  "
          f"arm={end['arm_angle']:.1f}°  hand={end['hand_angle']:.1f}°")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
