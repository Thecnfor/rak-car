#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/tasks/__init__.py

业务层任务编排。8 个固定顺序任务 (seed → scout pests → water → shoot pests →
harvest → sort → read order (OCR) → deliver) 中, 已实现 task1 / task2 / task6,
其余 task3-5/7/8 待补。

调用约定:
  - 业务层只通过 runtime HTTP API 调底层 car/arm action, 不直接 import smartcar/runtime
  - 每个 task 模块导出 run(client=None) -> dict, client=None 时内部 new RuntimeApiClient
  - helpers 集中在 _helpers (motion / runtime 就绪 / 推理就绪 / 取放 / 校验)
  - 任务配置在仓库根 task_config.yml (task1/2) + test/task6_config.yml (task6 独立)

已实现 task (按 CLAUDE.md 8-task 顺序编号):
  task1 -> auto_seeding:  播种 (right cylinders -> left purple circles)
  task2 -> water_tower_task: 水塔取水
  task6 -> get_order:    智能接单 (push-bar + OCR + 蔬菜取放)

未实现 (待补):
  task3 / task4 / task5 / task7 / task8
"""
from __future__ import annotations

from main.tasks.auto_seeding import run as run_task1_auto_seeding
from main.tasks.water_tower_task import run as run_task2_water_tower
from main.tasks.get_order import run as run_task6_get_order

# 任务编号 -> run 函数. orchestrator/调度层用 TASK_RUNNERS[id](client) 调用.
TASK_RUNNERS = {
    1: run_task1_auto_seeding,
    2: run_task2_water_tower,
    6: run_task6_get_order,
}

__all__ = [
    "TASK_RUNNERS",
    "run_task1_auto_seeding",
    "run_task2_water_tower",
    "run_task6_get_order",
]