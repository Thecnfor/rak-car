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
from typing import Any, Dict

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
