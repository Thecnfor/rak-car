#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/task —— 8 任务编排薄封装工具包.

跟 main/arm/、main/chassis/ 平级,做"按编号找人"的索引层。
具体业务逻辑在 main.arm.each_task.{task1..7}/,本包只做包装 + 适配。

文件命名约定: task{N}_{业务名}.py, 编号 = 比赛中的任务次序.
每个模块暴露 run(client=None) -> Dict[str, Any],client=None 时内部 new RuntimeApiClient.

8 任务清单 (跟 main.start.orchestrator.DEFAULT_WAYPOINTS 一一对应):
  task1_seeding        播种 (auto_seeding 既有实现)
  task2_water_tower    取水 (water_tower_task 既有实现)
  task3_pest_scout     害虫侦察 + 射击 (TODO)
  task4_harvest        抓取作物 (main.arm.each_task.task4)
  task5_sort           分拣作物 (main.arm.each_task.task5)
  task6_get_order      接单 + 识别 (get_order 既有实现)
  task7_deliver        投放外卖 (main.arm.each_task.task7.the_final)

调用约定:
  - 业务层只通过 RuntimeApiClient 调 runtime HTTP API
  - orchestrator 通过 TASK_RUNNERS[id](client) 调用
  - 未实现 task (3) 抛 NotImplementedError, orchestrator 捕获后跳过 (task7 已实现, 薄封装 the_final.main)
"""
from __future__ import annotations

from typing import Any, Callable, Dict, Optional

from main.api_client import RuntimeApiClient

# 已有 task (1/2/6) —— 在本目录下
from main.task.task1_seeding import run as run_task1_seeding
from main.task.task2_water_tower import run as run_task2_water_tower
from main.task.task6_get_order import run as run_task6_get_order

# 新增 task (3/4/5/7) —— 新建占位或包装
from main.task.task3_pest_scout import run as run_task3_pest_scout
from main.task.task4_harvest import run as run_task4_harvest
from main.task.task5_sort import run as run_task5_sort
from main.task.task7_deliver import run as run_task7_deliver

TASK_RUNNERS: Dict[int, Callable[[Optional[RuntimeApiClient]], Dict[str, Any]]] = {
    1: run_task1_seeding,
    2: run_task2_water_tower,
    3: run_task3_pest_scout,
    4: run_task4_harvest,
    5: run_task5_sort,
    6: run_task6_get_order,
    7: run_task7_deliver,
}

__all__ = [
    "TASK_RUNNERS",
    "run_task1_seeding",
    "run_task2_water_tower",
    "run_task3_pest_scout",
    "run_task4_harvest",
    "run_task5_sort",
    "run_task6_get_order",
    "run_task7_deliver",
]
