"""main/arm/postures.py —— 示教器姿态 JSON 加载（唯一姿势入口）。

示教器导出的 JSON 是扁平姿态列表（首尾 goal、中间 waypoint），字段
name/x_mm/y_mm/arm/hand/ts。`load_teach_json` 把它转成 JointPose 列表，
直接喂 `plan_joint_trajectory` / runtime `replay_arm_trajectory`。
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import List

from main.arm.planning.joint_trajectory import JointPose


def load_teach_json(path: str) -> List[JointPose]:
    """加载示教器导出的姿势序列（第一个/最后一个是 goal，中间 waypoint）。

    JSON 格式（顶层为列表）::

        [{"name": "pose_1", "x_mm": -223.7, "y_mm": -150.1,
          "arm": 90, "hand": -10, "ts": ...}, ...]

    返回按原顺序的 JointPose 列表；`plan_joint_trajectory(...)` 直接消费。
    """
    with Path(path).open("r", encoding="utf-8") as f:
        raw = json.load(f)
    if not isinstance(raw, list):
        raise ValueError(f"{path} 顶层必须是姿势列表（示教器导出）")
    return [JointPose.from_mapping({k: v for k, v in p.items() if k != "ts"})
            for p in raw if isinstance(p, dict)]
