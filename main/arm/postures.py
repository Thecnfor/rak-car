"""main/arm/postures.py —— 机械臂姿势库（调参面）。

把散落在 task_config.yml task_cfg / each_task 各 constants.py / task 文件里的
关节姿势收敛到一个文件 `main/arm/postures.yaml`，示教器标定值改一处即可。
业务任务可以直接用 `poses()` / `plan()` 把命名姿势串成 goal→waypoint→goal
平滑轨迹（见 main/arm/planning/joint_trajectory.py）。

- 姿势字段：x_mm / y_mm / arm_deg / hand_deg / stop（全可缺省，缺省用 0 占位；
  stop=False 表示该关键点不停车直接滑过）。
- 校验：有限值 + 关节限位（arm ±150 / hand -90..10 / y -200..0 / x ±300）。
- task 键既接受 yaml 里的任务名，也接受任务编号 1..8（映射表见 _TASK_KEYS）。
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence

from main.arm.planning.joint_trajectory import (
    JointPose, JointTrajectory, plan_joint_trajectory,
)

# 任务编号 → 姿势库键（与 main.task.TASK_RUNNERS 编号一致）
_TASK_KEYS = {
    1: "task1_seeding",
    2: "task2_water_tower",
    3: "task3_pest_scout",
    4: "task4_harvest",
    5: "task5_sort",
    6: "task6_get_order",
    7: "task7_deliver",
    8: "task3_shoot",
}
_DEFAULT_PATH = Path(__file__).resolve().parent / "postures.yaml"


class PostureLibrary:
    """YAML 姿势库加载 / 校验 / 取姿 / 保存。"""

    def __init__(self, path: Optional[str] = None):
        self.path = Path(path).resolve() if path else _DEFAULT_PATH
        self._data: Dict = self._load()

    # ---------------- 加载 ----------------

    def _load(self) -> Dict:
        if not self.path.is_file():
            raise FileNotFoundError(f"姿势库不存在: {self.path}")
        try:
            import yaml
        except ModuleNotFoundError as exc:  # pragma: no cover
            raise RuntimeError("缺少 PyYAML, 请先: python3 -m pip install pyyaml") from exc
        with self.path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise ValueError(f"{self.path} 顶层必须是 mapping")
        # 首次加载即校验全部姿势
        self.validate(data)
        return data

    @classmethod
    def validate(cls, data: Dict) -> None:
        """校验整个姿势库：每个任务的每个姿势字段都有限且落限位。"""
        for task_key, section in data.items():
            if not isinstance(section, dict):
                continue
            for name, value in section.items():
                if name.startswith("_") or name in ("y_mm", "init_y_mm"):
                    continue  # 文档 / 纯标量不校验
                if isinstance(value, dict):
                    JointPose.from_mapping(value)  # 抛错 = 校验失败
                elif isinstance(value, (int, float)) and name not in (
                        "sample_hz", "max_speed_scale"):
                    # 纯数值（如 speed / y_mm 阈值）只要求有限
                    if not math.isfinite(float(value)):
                        raise ValueError(f"{task_key}.{name} 必须有限")

    # ---------------- 取姿 ----------------

    def resolve_key(self, key: object) -> str:
        if isinstance(key, str):
            return key
        if isinstance(key, int) and key in _TASK_KEYS:
            return _TASK_KEYS[key]
        raise KeyError(f"未知任务键: {key!r}（可用 1..8 或姿势库任务名）")

    def task(self, key: object) -> Dict:
        return self._data[self.resolve_key(key)]

    def pose(self, key: object, name: str,
             default: Optional[dict] = None, *, stop: bool = True) -> JointPose:
        section = self.task(key)
        value = section.get(name)
        if not isinstance(value, dict):
            if default is None:
                raise KeyError(f"{self.resolve_key(key)} 没有姿势 '{name}'")
            value = dict(default)
        value = dict(value)
        value.setdefault("stop", stop)
        return JointPose.from_mapping(value)

    def poses(self, key: object, names: Sequence[str]) -> List[JointPose]:
        return [self.pose(key, name) for name in names]

    def route(self, key: object, names: Sequence[str],
              close: bool = False) -> List[JointPose]:
        """把命名姿势串成 goal→waypoint→goal 路径。

        close=True 时自动在末尾补回第一个姿势（回到起点，闭环路线）。
        """
        poses = self.poses(key, names)
        if close and len(poses) > 1:
            poses = poses + [poses[0]]
        return poses

    def plan(self, key: object, names: Sequence[str], *,
             close: bool = False, **plan_kw) -> JointTrajectory:
        """一步到位：姿势库命名序列 → 平滑轨迹。plan_kw 透传 plan_joint_trajectory。"""
        return plan_joint_trajectory(self.route(key, names, close=close), **plan_kw)

    # ---------------- 保存 / 现场标定 ----------------

    def save(self, path: Optional[str] = None) -> None:
        import yaml
        target = Path(path).resolve() if path else self.path
        self.validate(self._data)
        with target.open("w", encoding="utf-8") as f:
            yaml.safe_dump(self._data, f, allow_unicode=True, sort_keys=False)


def load_postures(path: Optional[str] = None) -> PostureLibrary:
    return PostureLibrary(path)
