"""main/arm/postures.py —— 机械臂姿势库（统一调参面，支持示教器 JSON 导入）。

把散落在 task_config.yml task_cfg / each_task 各 constants.py / task 文件里的
关节姿势收敛到一个文件 `main/arm/postures.yaml`，示教器标定值改一处即可。

统一管理约定（**task 动作姿势的唯一来源**）：
  - 每个任务的 section 规范化成::
        { "_doc": 说明, "poses": {姿势名: {x_mm,y_mm,arm_deg,hand_deg,stop}},
          "actions": {完整动作名: [姿势名, ...]},  # 可缺省
          **标量阈值 }
  - `poses` 既可从 YAML 提供，也可从同名 **`postures.json` sidecar** 提供
    （`_load` 自动合并）——你给示教器 JSON 时直接放到
    `main/arm/postures.json` 即可，无需改 YAML。
  - "每个完整动作" = 一条命名姿势序列（actions），`lib.action(4,"pick_and_place")`
    直接给出平滑轨迹（goal→waypoint→goal）。

姿势字段：x_mm / y_mm / arm_deg / hand_deg / stop（全可缺省；arm_angle_deg /
hand_angle_deg 别名兼容 task_config.yml）。校验：有限值 + 关节限位
（arm ±150 / hand -90..10 / y -200..0 / x ±300）。task 键接受编号 1..8 或任务名。
"""
from __future__ import annotations

import json
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


def _normalize_section(section: dict, task_key: str) -> dict:
    """把扁平（姿势 dict 平铺 + 标量）或 {poses, actions} 布局统一成
    {_doc, poses, actions, **标量}。"""
    poses: Dict[str, dict] = {}
    actions: Dict[str, list] = {}
    scalars: Dict[str, object] = {}
    doc = ""
    wrapped = isinstance(section.get("poses"), dict)
    if wrapped:
        poses.update(section["poses"])
        for k, v in section.items():
            if k == "poses":
                continue
            if k == "actions":
                if isinstance(v, dict):
                    actions.update({kk: list(vv) for kk, vv in v.items()})
                continue
            if k.startswith("_"):
                if k == "_doc":
                    doc = str(v)
                continue
            scalars[k] = v
    else:
        for k, v in section.items():
            if k.startswith("_"):
                if k == "_doc":
                    doc = str(v)
                continue
            if isinstance(v, dict):
                poses[k] = dict(v)
            else:
                scalars[k] = v
        if isinstance(section.get("actions"), dict):
            actions.update({kk: list(vv) for kk, vv in section["actions"].items()})
    out: Dict[str, object] = {"poses": poses, "actions": actions}
    if doc:
        out["_doc"] = doc
    out.update(scalars)
    return out


def _deep_merge(base: dict, new: dict) -> None:
    """把新 section 合并进已有 section（poses/actions/标量覆盖，其余保留）。"""
    base_poses = base.setdefault("poses", {})
    for name, value in (new.get("poses") or {}).items():
        base_poses[name] = dict(value)
    base_actions = base.setdefault("actions", {})
    for name, seq in (new.get("actions") or {}).items():
        base_actions[name] = list(seq)
    for k, v in new.items():
        if k in ("poses", "actions", "_doc"):
            continue
        base[k] = v


class PostureLibrary:
    """YAML(+JSON sidecar) 姿势库加载 / 校验 / 取姿 / 动作路由 / 导入 / 保存。"""

    def __init__(self, path: Optional[str] = None):
        self.path = Path(path).resolve() if path else _DEFAULT_PATH
        self._data: Dict = self._load()

    # ---------------- 加载（YAML + 同名 JSON sidecar 自动合并） ----------------

    def _load(self) -> Dict:
        if not self.path.is_file():
            raise FileNotFoundError(f"姿势库不存在: {self.path}")
        try:
            import yaml
        except ModuleNotFoundError as exc:  # pragma: no cover
            raise RuntimeError("缺少 PyYAML, 请先: python3 -m pip install pyyaml") from exc
        with self.path.open("r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}
        if not isinstance(raw, dict):
            raise ValueError(f"{self.path} 顶层必须是 mapping")
        data = {k: _normalize_section(v, str(k))
                for k, v in raw.items() if isinstance(v, dict)}
        # 同名 postures.json sidecar：示教器完整动作数据直接丢这里，自动合并
        sidecar = self.path.with_suffix(".json")
        if sidecar.is_file():
            with sidecar.open("r", encoding="utf-8") as f:
                extra = json.load(f) or {}
            for task_key, section in extra.items():
                if not isinstance(section, dict):
                    continue
                norm = _normalize_section(section, str(task_key))
                if str(task_key) in data:
                    _deep_merge(data[str(task_key)], norm)
                else:
                    data[str(task_key)] = norm
        self.validate(data)
        return data

    @classmethod
    def validate(cls, data: Dict) -> None:
        """校验整个姿势库：每个姿势有限且落限位；action 引用的姿势必须存在。"""
        for task_key, section in data.items():
            if not isinstance(section, dict):
                continue
            norm = _normalize_section(section, str(task_key))
            poses = norm.get("poses") or {}
            for name, value in poses.items():
                if isinstance(value, dict):
                    JointPose.from_mapping(value)  # 抛错 = 校验失败
            for name, seq in (norm.get("actions") or {}).items():
                if not isinstance(seq, (list, tuple)):
                    raise ValueError(f"{task_key}.actions.{name} 必须是姿势名列表")
                for pname in seq:
                    if pname not in poses:
                        raise ValueError(
                            f"{task_key}.actions.{name} 引用了不存在的姿势 '{pname}'")
            for name, value in norm.items():
                if name in ("poses", "actions", "_doc"):
                    continue
                if isinstance(value, (int, float)) and not math.isfinite(float(value)):
                    raise ValueError(f"{task_key}.{name} 必须有限")

    # ---------------- 取姿 / 动作 ----------------

    def resolve_key(self, key: object) -> str:
        if isinstance(key, str):
            return key
        if isinstance(key, int) and key in _TASK_KEYS:
            return _TASK_KEYS[key]
        raise KeyError(f"未知任务键: {key!r}（可用 1..8 或姿势库任务名）")

    def task(self, key: object) -> Dict:
        return self._data[self.resolve_key(key)]

    def pose(self, key: object, name: str,
             default: Optional[dict] = None, *, stop: bool = False) -> JointPose:
        section = self.task(key)
        poses = section.get("poses") if isinstance(section.get("poses"), dict) else {}
        value = poses.get(name)
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
        """把命名姿势串成 goal→waypoint→goal 路径。close=True 末尾补回起点。"""
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
