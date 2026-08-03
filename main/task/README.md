# main/task —— 8 任务编排薄封装工具包

跟 `main/arm/`、`main/chassis/` 平级。**工具包定位**: 不放业务逻辑,只放"按编号找人"的编排索引。

## 目录结构

```
main/task/
├── README.md                  # 本文件
├── __init__.py                # TASK_RUNNERS = {1: run_task1_seeding, ..., 7: ...}
├── _config.py                 # load_task_config(task_name) + load_waypoints()
├── _constants.py              # 任务间共享常量
├── task1_seeding.py           # 播种 (ArmRunner.track_velocity_pick + track_chassis)
├── task2_water_tower.py       # 取水 (track_velocity_pick + track_chassis + move_for)
├── task3_pest_scout.py        # 害虫侦察 + 射击 (subprocess 薄封装 → task3/task3_pipeline.py)
├── task3/                     # task3 流水线工作区 (~50 脚本: pipeline/fire/scan/llm/audit)
├── task4_harvest.py           # 抓取作物 → main.arm.each_task.task4.target4
├── task5_sort.py              # 分拣作物 → main.arm.each_task.task5.the_final
├── task6_get_order.py         # 接单 + 识别 (ArmRunner + composite_run + OCR)
├── task7_deliver.py           # 投放外卖 (TODO, NotImplementedError)
└── tests/
    ├── test_orchestrator_yaml.py       # task_config.yml waypoints 段加载
    └── test_task1_vision_grasp.py      # task1 视觉抓取流程 (unittest 收 14 用例; `_Scan*` 前缀类是共享夹具,不被收集)
```

> **架构历史**: `_helpers.py` 在 2026-08 重构中被彻底删除. 该文件曾用 `client.execute("arm", "move_x", ...)` 等
> HTTP 薄封装重新实现了 `main.arm.ArmRunner` + `CompositeMixin` 已有的全部能力,
> 同时缺失 y 保护区 / 角度硬限 / 丢步核对 / composite_* 并发等 SafetyMixin 自动保护.
> 重构后, task1/2/6 改为直接使用 `main.arm.ArmRunner` + `composite_pick / composite_release / composite_run`,
> 业务层不再自己拼装串行 HTTP 调用. 详见各 task 文件头部 docstring 的「架构说明」段.

## 8 任务清单

| ID | 名称 | 业务逻辑 | 状态 |
|---|---|---|---|
| 1 | task1_seeding | (本文件, track_velocity_pick 播种/投放) | ✅ |
| 2 | task2_water_tower | (本文件, track_velocity_pick + move_for 蠕行) | ✅ |
| 3 | task3_pest_scout | `task3/task3_pipeline.py` (drive + LLM 判别 + shoot, subprocess) | ✅ |
| 4 | task4_harvest | `main.arm.each_task.task4.target4` | ✅ (am 移植) |
| 5 | task5_sort | `main.arm.each_task.task5.the_final` | ✅ (am 移植) |
| 6 | task6_get_order | (本文件, OCR + composite_run) | ✅ |
| 7 | task7_deliver | (TODO) | ⏳ |

## 每个 task 现用的现成方法

写新 task / 改老 task 时照抄兄弟 task 的开场和主力方法（完整速查表见 [../README.md](../README.md)）：

| task | 主力现成方法 |
|---|---|
| task1 | `runner.track_velocity_pick(label)` + `track_velocity_pick(label, mode="drop")`（现场热调参 gain_arm=2.5/gain_x=0.55/skip_pose_align=True）+ `track_chassis`（对准圆柱标记） |
| task2 | `runner.track_velocity_pick`（arm_min/arm_max 限位内 +95° 抓）+ `track_chassis(vx_only=...)` + `ChassisClient.move_for`（探测重试蠕行） |
| task3 | 不用 main.arm/main.chassis；纯 `RuntimeApiClient` + subprocess 流水线 |
| task4 | `track_chassis(select_mode="leftmost")` 定位球 + `track_velocity_pick(skip_pose_align=True)` + `composite_run` + `goto_pose_p`（each_task/common.py 的 P 姿态恢复） |
| task5 | `runner.track_velocity_pick`（pick_and_place 视觉模式）+ `make_align_runner(ref_area=...)`（料箱面积对准）+ `move_x_with_split` |
| task6 | `runner.move_x/move_y` + `arm_client.set_hand_angle` + `runner.client.composite_run` + `main.misc.test_order_read.run()` |

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
- 已实现的 task (1/2/3/4/5/6) 返回 `{"ok": bool, "task": ..., "detail": ...}`
- task3 走 subprocess 调 `task3/task3_pipeline.py`，子进程非零退出码冒泡为 status；首次跑会停在人工确认（`--no-pause` 跳过）
- 未实现的 task (7) raise `NotImplementedError`,**orchestrator 捕获后 warning + 跳过,不中断流程**

## 跟 arm/each_task/ 的对应关系

`arm/each_task/` 是**具体业务逻辑**(从 origin/am 移植过来, 2026-08-01 commit `a51f634`):

| each_task 目录 | 接线状态 |
|---|---|
| `task4/` | **活**：task4_harvest 用 `target4.step_target4` |
| `task5/` | **活**：task5_sort 用 `the_final.main` |
| `task1/`, `task2/`, `task6/`, `task7/` | **未接线**（wrapper 自己实现了业务）。⚠️ 这些脚本调用了**已不存在**的 `runner.set_side` / `runner.set_hand`（task5 的 target_yellow/target_blue 还调 `stop_x_speed_safety`），直接 run 会 AttributeError；想复用前先修 |

## 维护指南

1. **改业务**: 改 `main/arm/each_task/taskN/` 里的对应 step（task4/5）或 `main/task/taskN_xxx.py` 自身（task1/2/6）
2. **加新 task**: 新建 `main/task/taskN_xxx.py` + 在 `__init__.py` 加 `TASK_RUNNERS[N] = run`；先查 [../README.md](../README.md) 速查表里有没有现成方法
3. **换场地**: 改根目录 `task_config.yml` `waypoints:` 段的 IR/odom 阈值, **不动代码**
4. **task7 业务实现后**: 把逻辑写到 `main/arm/each_task/task7/`, 然后本文件改包装(跟 task4/5 同样模板)

## 设计依据

- 单数命名: 跟 `arm/` `chassis/` 平级 (工具包)
- 业务命名: 文件名 = 业务名 (`task4_harvest.py`), 一眼能找到
- 8 任务顺序: 跟 `main.start.orchestrator.DEFAULT_WAYPOINTS` 一一对应 (从 task_config.yml 加载)
