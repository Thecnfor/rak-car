#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""示例 13：示教器姿态 JSON → goal→waypoint→goal 平滑 → FakeRobotSim 仿真。

用法（repo root）::

    PYTHONPATH=. python3 main/arm/examples/13_posture_route.py [路径.json]

演示：加载示教器 JSON（扁平姿态列表，首尾 goal / 中间 waypoint）→
plan_joint_trajectory 生成 4-DOF 平滑轨迹（scipy PCHIP，连续经过）→
逐点喂给 FakeRobotSim 让关节真的动起来，打印轨迹描述 + 仿真末端姿态。

真机等价：`python3 -c "from main.api_client import ..."` 或 runtime 的
replay_arm_trajectory（进程内连续回放，客户端只发一次姿态 JSON）。
"""
from __future__ import annotations

import argparse
import sys

from main.arm.planning import plan_joint_trajectory
from main.arm.postures import load_teach_json


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        prog="13_posture_route",
        description="示教器姿态 JSON → goal→waypoint→goal 平滑 → FakeRobotSim",
    )
    p.add_argument("json_path", nargs="?", default="/home/xrak/Downloads/rak-car-poses.json",
                   help="示教器导出的姿态 JSON（列表，首尾 goal）")
    args = p.parse_args(argv)

    route = load_teach_json(args.json_path)
    print(f"=== 路线 {len(route)} 个姿态（首尾 goal / 中间 waypoint）===")
    for i, pose in enumerate(route):
        print(f"  [{i + 1}] x={pose.x_mm:8.1f} y={pose.y_mm:8.1f} "
              f"arm={pose.arm_deg:6.0f} hand={pose.hand_deg:5.0f}")

    traj = plan_joint_trajectory(route)
    print("\n=== 平滑轨迹 ===")
    print(traj.describe())

    dense = traj.dense_waypoints(spacing_mm=8.0)
    print(f"\n=== 重采样 {len(dense)} 个 composite_run 喂点 ===")
    for pose in dense[:5]:
        print(f"  composite_run(x_mm={pose.x_mm:.0f}, y_mm={pose.y_mm:.0f}, "
              f"arm={pose.arm_deg:.0f}, hand={pose.hand_deg:.0f})")
    if len(dense) > 5:
        print(f"  ... 共 {len(dense)} 点")

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
    print(f"\n=== 仿真末端姿态（应为最后一个 goal）===")
    print(f"  x={end['x_mm']:.1f}mm  y={end['y_mm']:.1f}mm  "
          f"arm={end['arm_angle']:.1f}°  hand={end['hand_angle']:.1f}°")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
