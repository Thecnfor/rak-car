# main/task —— 8 任务编排薄封装工具包

跟 `main/arm/`、`main/chassis/` 平级。**工具包定位**: 不放业务逻辑,只放"按编号找人"的编排索引。

## 目录结构

```
main/task/
├── README.md                  # 本文件
├── __init__.py                # TASK_RUNNERS = {1: run_task1_seeding, ..., 7: ...}
├── _config.py                 # load_task_config(task_name) + load_waypoints()
├── _helpers.py                # 公共 motion/runtime/推理 helpers (已有, 跟 main/tasks 同源)
├── task1_seeding.py           # 播种 (auto_seeding 既有实现)
├── task2_water_tower.py       # 取水 (water_tower_task 既有实现)
├── task3_pest_scout.py        # 害虫侦察 + 射击 (TODO, NotImplementedError)
├── task4_harvest.py           # 抓取作物 → main.arm.each_task.task4.target4
├── task5_sort.py              # 分拣作物 → main.arm.each_task.task5.the_final
├── task6_get_order.py         # 接单 + 识别 (get_order 既有实现)
├── task7_deliver.py           # 投放外卖 (TODO, NotImplementedError)
└── tests/
    ├── test_task_index.py              # TASK_RUNNERS 字典 1..7 完整
    ├── test_task_wrappers.py           # task4/5 wrapper signature
    ├── test_orchestrator_yaml.py       # task_config.yml waypoints 段加载
    └── test_task_skips_unimplemented.py # task3/7 抛 NotImpl 但不崩
```

## 8 任务清单

| ID | 名称 | 业务逻辑 | 状态 |
|---|---|---|---|
| 1 | task1_seeding | (本文件, 既有 auto_seeding) | ✅ |
| 2 | task2_water_tower | (本文件, 既有 water_tower_task) | ✅ |
| 3 | task3_pest_scout | (TODO) | ⏳ |
| 4 | task4_harvest | `main.arm.each_task.task4.target4` | ✅ (am 移植) |
| 5 | task5_sort | `main.arm.each_task.task5.the_final` | ✅ (am 移植) |
| 6 | task6_get_order | (本文件, 既有 get_order) | ✅ |
| 7 | task7_deliver | (TODO) | ⏳ |

## 调用约定

```python
from main.task import TASK_RUNNERS
from main.api_client import RuntimeApiClient

client = RuntimeApiClient()
result = TASK_RUNNERS[1](client)  # task1_seeding.run(client)
# → {"ok": bool, "task": "task1_seeding", "detail": ...}
```

每个 `taskN_xxx.run(client=None) -> Dict[str, Any]`:
- `client=None` 时内部 `new RuntimeApiClient()`
- 已实现的 task (1/2/4/5/6) 返回 `{"ok": bool, "task": ..., "detail": ...}`
- 未实现的 task (3/7) raise `NotImplementedError`,**orchestrator 捕获后 warning + 跳过,不中断流程**

## 跟 arm/each_task/ 的对应关系

`arm/each_task/` 是**具体业务逻辑**(从 origin/am 移植过来, 2026-08-01 commit `a51f634`):
- `arm/each_task/task1/` ← task1_seeding 备选入口 (`a_approach → run_one → c_finish`)
- `arm/each_task/task4/` ← task4_harvest 用 `target4.step_target4`
- `arm/each_task/task5/` ← task5_sort 用 `the_final.main`

`arm/each_task/task2/`, `task6/`, `task7/` 当前空(业务在 main/task/taskN_xxx.py 自身)。

## 维护指南

1. **改业务**: 改 `main/arm/each_task/taskN/` 里的对应 step
2. **加新 task**: 新建 `main/task/taskN_xxx.py` + 在 `__init__.py` 加 `TASK_RUNNERS[N] = run`
3. **换场地**: 改根目录 `task_config.yml` `waypoints:` 段的 IR/odom 阈值, **不动代码**
4. **task3/task7 业务实现后**: 把逻辑写到 `main/arm/each_task/taskN/`, 然后本文件改包装(跟 task4/5 同样模板)

## 设计依据

- 单数命名: 跟 `arm/` `chassis/` 平级 (工具包)
- 业务命名: 文件名 = 业务名 (`task4_harvest.py`), 一眼能找到
- 8 任务顺序: 跟 `main.start.orchestrator.DEFAULT_WAYPOINTS` 一一对应 (从 task_config.yml 加载)