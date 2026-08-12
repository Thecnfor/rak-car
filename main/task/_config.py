#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/tasks/_config.py
任务配置加载器 -- 从仓库根目录的 task_config.yml 读取 task_cfg 段。

业务层只读这一个文件,不再各自 yaml.load。换场地时改 task_config.yml,
业务代码不动。
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Any, Dict, List

try:
    import yaml
except ModuleNotFoundError as exc:  # pragma: no cover
    raise RuntimeError(
        "缺少 PyYAML 依赖,请先执行: python3 -m pip install pyyaml"
    ) from exc


_DEFAULT_CONFIG_NAME = "task_config.yml"


def _repo_root() -> Path:
    """main/tasks/_config.py -> main/tasks -> main -> repo_root"""
    return Path(__file__).resolve().parents[2]


def _config_path() -> Path:
    return _repo_root() / _DEFAULT_CONFIG_NAME


def load_task_config(task_name: str) -> Dict[str, Any]:
    """读取 task_config.yml 中 task_cfg.<task_name> 段。

    返回原始 dict。调用方自己解释字段。
    """
    path = _config_path()
    if not path.is_file():
        raise FileNotFoundError(
            f"任务配置文件不存在: {path}\n"
            f"请确认仓库根目录下有 task_config.yml"
        )
    with path.open("r", encoding="utf-8") as f:
        all_cfg = yaml.safe_load(f)
    if not isinstance(all_cfg, dict):
        raise ValueError(f"{path} 顶层必须是 mapping,实际是 {type(all_cfg)}")

    task_cfg = all_cfg.get("task_cfg", {})
    if task_name not in task_cfg:
        raise KeyError(
            f"task_config.yml 里没有 task_cfg.{task_name} 段,现有: {list(task_cfg.keys())}"
        )
    return task_cfg[task_name]


def require(cfg: Dict[str, Any], key: str, kind: type) -> Any:
    """从 cfg 里取 key,类型不对/缺失直接报错。任务脚本统一用这个防止 NPE。"""
    if key not in cfg:
        raise KeyError(f"配置缺少字段 {key!r}")
    v = cfg[key]
    if not isinstance(v, kind):
        raise TypeError(f"配置字段 {key!r} 类型应为 {kind.__name__},实际 {type(v).__name__}")
    return v


def load_waypoints() -> List[Dict[str, Any]]:
    """读取 task_config.yml 中 waypoints 段 (8 task + 1 finish).

    Returns:
        List[Dict]: 每个 dict 含 name, task_id (可选), task_module, ir_threshold_m,
                    ir_side, dis_at_least_m, trigger_op, is_finish 等字段.
                    orchestrator 据此构造 Waypoint 列表.
    """
    path = _config_path()
    if not path.is_file():
        raise FileNotFoundError(f"任务配置文件不存在: {path}")
    with path.open("r", encoding="utf-8") as f:
        all_cfg = yaml.safe_load(f)
    if not isinstance(all_cfg, dict):
        raise ValueError(f"{path} 顶层必须是 mapping")
    wp = all_cfg.get("waypoints")
    if not isinstance(wp, list):
        raise KeyError(
            f"task_config.yml 里没有 waypoints 段 (或不是 list), "
            f"现有顶层 keys: {list(all_cfg.keys())}"
        )
    return wp


def _load_post_task(cfg_key: str) -> Optional[Dict[str, Any]]:
    """读取 task_config.yml 中 task_cfg.<cfg_key> 段 (任务结束后一段位移+转弯).

    返回 None = 未配置 / enabled=false (orchestrator 跳过该段)。
    字段:
      straight_m: 任务后直行距离 (m, move_for 里程计闭环, 0=不走)。
      turn_deg:   原地里程计 θ 转弯角度 (度, OdomTurnPID; 实车方向反了取负, 0=不转)。
    """
    path = _config_path()
    if not path.is_file():
        return None
    with path.open("r", encoding="utf-8") as f:
        all_cfg = yaml.safe_load(f)
    if not isinstance(all_cfg, dict):
        return None
    task_cfg = all_cfg.get("task_cfg", {})
    seg = task_cfg.get(cfg_key)
    if not isinstance(seg, dict) or not seg.get("enabled"):
        return None
    return seg


def load_post_task1() -> Optional[Dict[str, Any]]:
    """读取 task_config.yml 中 task_cfg.post_task1 段 (task1 结束后一段位移+转弯)."""
    return _load_post_task("post_task1")


def load_post_task2() -> Optional[Dict[str, Any]]:
    """读取 task_config.yml 中 task_cfg.post_task2 段 (task2 结束后巡线中途定位直行).

    task2 (水塔) 结束后保持巡线, 里程计累计到 cruise_until_m 时暂停巡线 →
    move_for 前直行 straight_m → 恢复巡线。里程计在 task2 结束时已清零,
    所以 cruise_until_m 是"task2 之后走了多少米"。
    """
    return _load_post_task("post_task2")


def load_post_task6() -> Optional[Dict[str, Any]]:
    """读取 task_config.yml 中 task_cfg.post_task6 段 (task6 结束后一段位移+转弯).

    task6 (读订单) 结束后: 清零里程 → 切断巡线视觉 → 直行 → 里程计 θ 顺时针转
    120° → 恢复视觉 → 继续巡线。turn_deg 用负值 = 顺时针 (θ 增为逆时针, 见
    MEMORY turn-sign-calibration)。
    """
    return _load_post_task("post_task6")
