# 8-Task Orchestration Rewrite Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 `main/tasks/` (复数) 重组为 `main/task/` (单数) 8 任务薄封装工具包,业务命名 + README + task_config.yml 编排配置,orchestrator 走新路径。

**Architecture:**
- 业务逻辑层: `main/arm/each_task/{task1..7}/` (已存在, am 移植; task4/5 是 am 移植, task1 是从 main.tasks 迁移)
- 编排封装层: `main/task/{task1_seeding.py, task2_water_tower.py, ..., task7_deliver.py}` (新建)
- 编排索引层: `main/task/__init__.py` + `main/task/README.md` (新建)
- 配置层: 根目录 `task_config.yml` 加 `waypoints` 段 (改)
- 调度层: `main/start/orchestrator.py` `DEFAULT_WAYPOINTS` 改用 `task_id` + `main.task.taskN_xxx` 路径 (改)

**Tech Stack:** Python 3.8 / dataclass / yaml / PyYAML / RuntimeApiClient

## Global Constraints

- **路径冻结:** `main/` 之外的所有目录仍冻结 (pre-commit hook 拦截)
- **Python:** `/usr/bin/python3` 3.8; `python3 -m venv` 用 `--system-site-packages`
- **OpenCV:** 4.5.x 系统包,不要编译
- **命名约定:** 业务命名 (task1_seeding.py) 而非技术命名 (_crop_harvesting.py); `main/task/` 单数跟 `arm/` `chassis/` 平级
- **注释:** 中文 docstring 跟周围代码一致
- **测试:** 现有 85 tests 必须不破坏
- **commit message:** 中文 `feat(arm/task):` `chore:` 风格
- **不引入新依赖:** 只用 PyYAML (已用)

---

## File Structure (变更总览)

| 路径 | 动作 | 责任 |
|------|------|------|
| `main/tasks/` | git mv → `main/task/` | 整目录重命名 |
| `main/tasks/auto_seeding.py` | mv → `main/task/task1_seeding.py` | 已实现 task1 |
| `main/tasks/water_tower_task.py` | mv → `main/task/task2_water_tower.py` | 已实现 task2 |
| `main/tasks/get_order.py` | mv → `main/task/task6_get_order.py` | 已实现 task6 |
| `main/tasks/_helpers.py` | mv → `main/task/_helpers.py` | 公共工具 |
| `main/tasks/_config.py` | mv → `main/task/_config.py` (path 解析改) | 配置加载 |
| `main/tasks/__init__.py` | 改 → `main/task/__init__.py` | TASK_RUNNERS 字典 |
| (新建) | `main/task/README.md` | 目录说明 + 8 任务清单 |
| (新建) | `main/task/task3_pest_scout.py` | NotImplementedError |
| (新建) | `main/task/task4_harvest.py` | 包装 each_task/task4 |
| (新建) | `main/task/task5_sort.py` | 包装 each_task/task5 |
| (新建) | `main/task/task7_deliver.py` | NotImplementedError |
| (新建) | `main/task/tests/test_task_index.py` | smoke: TASK_RUNNERS 字典完整 |
| (新建) | `main/task/tests/test_task_wrappers.py` | smoke: 各 task run() signature |
| (新建) | `main/task/tests/test_orchestrator_yaml.py` | smoke: yaml waypoints 加载 |
| (新建) | `main/task/tests/test_task_skips_unimplemented.py` | smoke: task3/7 抛 NotImpl 但 orchestrator 不崩 |
| `task_config.yml` | 改 | 加 `task_cfg.waypoints` 段 (8 waypoint IR/odom) |
| `main/start/orchestrator.py` | 改 | Waypoint 加 `task_id`; DEFAULT_WAYPOINTS 改路径; 加载 yaml waypoints 段 |

---

## Task 1: Rename `main/tasks/` to `main/task/` (目录重命名 + 内容微调)

**Files:**
- Modify (rename): `main/tasks/` → `main/task/`
- Modify (rename + content): `main/task/auto_seeding.py` → `main/task/task1_seeding.py`
- Modify (rename + content): `main/task/water_tower_task.py` → `main/task/task2_water_tower.py`
- Modify (rename + content): `main/task/get_order.py` → `main/task/task6_get_order.py`
- Modify (path fix): `main/task/_config.py` (内部路径解析 `parents[2]` → `parents[2]`, 但 import 路径变了需更新)
- Modify (path fix): `main/task/_helpers.py` (更新注释里的 `main.tasks` → `main.task`)

**Rationale:** 单数命名跟 `arm/` `chassis/` 平级; 文件名直接是业务名。

### Step 1.1: 验证 git mv 前状态干净

```bash
cd /home/jetson/workspace/rak-car
git status --short
# 预期: 干净 (no output) 或仅未提交改动 (无 main/tasks/ 相关)
```

如果输出含 main/tasks/ 相关未提交改动,先 `git stash push -m "pre-task-rename"`。

### Step 1.2: git mv 整目录

```bash
cd /home/jetson/workspace/rak-car
git mv main/tasks main/task
git status --short | head -10
# 预期: "R  main/tasks/__init__.py -> main/task/__init__.py" 等 rename 行
```

### Step 1.3: git mv 重命名 3 个 task 文件

```bash
cd /home/jetson/workspace/rak-car
git mv main/task/auto_seeding.py main/task/task1_seeding.py
git mv main/task/water_tower_task.py main/task/task2_water_tower.py
git mv main/task/get_order.py main/task/task6_get_order.py
ls main/task/*.py
# 预期:
#   __init__.py _config.py _helpers.py
#   task1_seeding.py task2_water_tower.py task6_get_order.py
```

### Step 1.4: 修 `_config.py` 路径解析

`_config.py` 的 `_repo_root()` 用 `Path(__file__).resolve().parents[2]`,从 `main/tasks/_config.py` 跳到 `main/tasks -> main -> repo_root`,从 `main/task/_config.py` 跳到 `main/task -> main -> repo_root`。**深度不变**,但要确认。

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -c "
from main.task._config import _repo_root, _config_path
p = _repo_root()
print(f'repo_root = {p}')
print(f'config_path = {_config_path()}')
print(f'config exists = {_config_path().is_file()}')
"
# 预期: 路径解析正确, config_path = /home/jetson/workspace/rak-car/task_config.yml, exists=True
```

### Step 1.5: 修 `_helpers.py` docstring 里的旧模块引用

```bash
cd /home/jetson/workspace/rak-car
grep -n "main\.tasks\." main/task/_helpers.py
# 预期: 注释里可能有 main.tasks.X 旧路径,如果有 sed 替换
sed -i 's|main\.tasks\.|main.task.|g' main/task/_helpers.py
sed -i 's|from main\.tasks\.|from main.task.|g' main/task/_helpers.py
grep -n "main\.task" main/task/_helpers.py
# 预期: 全部更新到 main.task.*
```

### Step 1.6: 修 `__init__.py` docstring 和 import

`main/tasks/__init__.py` 当前 import 路径是 `main.tasks.auto_seeding` 等。重命名为 `main/task/__init__.py` 后必须改 import。

**期望最终 `main/task/__init__.py` 内容** (重写):

```python
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
  task7_deliver        投放外卖 (TODO)

调用约定:
  - 业务层只通过 RuntimeApiClient 调 runtime HTTP API
  - orchestrator 通过 TASK_RUNNERS[id](client) 调用
  - 未实现 task (3/7) 抛 NotImplementedError, orchestrator 捕获后跳过
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
```

**操作:**

```bash
cd /home/jetson/workspace/rak-car
# 备份当前内容
cp main/task/__init__.py /tmp/main_task_init_backup.py
# 用上面期望内容完整重写
```

### Step 1.7: 验证 task1/2/6 旧文件 docstring 里的路径引用

3 个旧文件 docstring 里可能引用 `main.tasks.X`,统一 sed:

```bash
cd /home/jetson/workspace/rak-car
for f in main/task/task1_seeding.py main/task/task2_water_tower.py main/task/task6_get_order.py; do
    sed -i 's|main\.tasks\.|main.task.|g' "$f"
done
# 不应改 from main.tasks.X import Y 的运行时代码 (无 import, 因为内部 _config / _helpers 路径都改了)
grep -nE "^from main\.tasks|^import main\.tasks" main/task/*.py
# 预期: 0 行 (无运行期 import 残留)
```

### Step 1.8: 跑现有 85 tests 验证 rename 未破坏

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pytest main/arm/tests/ main/chassis/ --tb=short -q 2>&1 | tail -5
# 预期: 85 passed
```

如果 taskN file 有问题 (如 sed 把不该改的改了), 用 git checkout 部分恢复:

```bash
cd /home/jetson/workspace/rak-car
# 验证 task1/2/6 仍可 import + run(client) 调用
/usr/bin/python3 -c "
from main.task.task1_seeding import run as r1
from main.task.task2_water_tower import run as r2
from main.task.task6_get_order import run as r6
print('task1/2/6 import: OK')
print(f'r1 signature: {r1.__annotations__}')
"
# 预期: import OK
```

### Step 1.9: 验证 task_config.yml 仍能加载

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -c "
from main.task._config import load_task_config
cfg = load_task_config('auto_seeding')
print(f'auto_seeding cfg keys: {list(cfg.keys())[:5]}')
print('OK')
"
# 预期: OK
```

### Step 1.10: Commit

```bash
cd /home/jetson/workspace/rak-car
git add -A
git status --short | head -20
git commit -m "refactor(task): main/tasks/ → main/task/ 单数目录 + 业务命名

8 任务文件重命名:
  main/tasks/auto_seeding.py        → main/task/task1_seeding.py
  main/tasks/water_tower_task.py    → main/task/task2_water_tower.py
  main/tasks/get_order.py           → main/task/task6_get_order.py
  main/tasks/__init__.py            → main/task/__init__.py (内容重写)
  main/tasks/_config.py / _helpers.py → main/task/_* (路径解析改)

理由:
- main/task/ 单数跟 main/arm/、main/chassis/ 平级 (工具包定位)
- 文件名直接是业务名 (task1_seeding.py) 而非技术名 (_crop_harvesting.py)
- 编号 = 比赛次序, 文件名一眼对应

内部逻辑零改动 (除 __init__.py 扩到 8 task 索引, 后续 task 加)。

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 2: Add `task_config.yml` `waypoints` 段 + `load_waypoints()` helper

**Files:**
- Modify: `task_config.yml` (在 `task_cfg:` 平级加 `waypoints:` 段)
- Modify: `main/task/_config.py` (加 `load_waypoints() -> List[Waypoint]` 函数)
- Test: `main/task/tests/test_orchestrator_yaml.py`

**Interfaces:**
- `load_waypoints() -> List[Dict[str, Any]]` 返回 8 个 waypoint dict + 1 finish dict
- 每个 dict 含 keys: `name`, `task_id`, `task_module`, `ir_threshold_m`, `ir_side`, `dis_at_least_m`, `trigger_op`, `is_finish`

### Step 2.1: 写失败测试

`main/task/tests/test_orchestrator_yaml.py`:

```python
"""main/task/tests/test_orchestrator_yaml.py

验证 task_config.yml 的 waypoints 段能被加载,且 8 个 task + 1 finish 全在。
"""
from pathlib import Path
import sys

# 路径: main/task/tests/ → repo_root
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from main.task._config import load_waypoints  # noqa: E402


def test_load_waypoints_returns_list_of_dicts():
    wp = load_waypoints()
    assert isinstance(wp, list)
    assert len(wp) >= 9  # 8 tasks + 1 finish
    for w in wp:
        assert isinstance(w, dict)
        assert "name" in w
        assert "task_id" in w or w.get("is_finish")


def test_load_waypoints_covers_all_7_tasks():
    wp = load_waypoints()
    task_ids = {w.get("task_id") for w in wp if "task_id" in w}
    assert task_ids == {1, 2, 3, 4, 5, 6, 7}, f"missing tasks: {set(range(1,8)) - task_ids}"


def test_load_waypoints_includes_finish():
    wp = load_waypoints()
    finish = [w for w in wp if w.get("is_finish")]
    assert len(finish) == 1
    assert "dis_at_least_m" in finish[0]


def test_load_waypoints_threshold_fields():
    """每个 waypoint 至少有一个触发条件 (IR 或 odom)。"""
    wp = load_waypoints()
    for w in wp:
        if w.get("is_finish"):
            continue  # finish 只看 odom
        assert (w.get("ir_threshold_m") is not None or
                w.get("dis_at_least_m") is not None), \
            f"{w.get('name')} 缺触发条件"
```

### Step 2.2: 跑测试确认失败

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pytest main/task/tests/test_orchestrator_yaml.py -v 2>&1 | tail -15
# 预期: ImportError 或 NameError (load_waypoints not defined)
```

### Step 2.3: 在 task_config.yml 加 waypoints 段

**期望内容 (在 `task_cfg:` 平级加):**

```yaml
# 8 任务 + 1 终点 waypoint 编排配置 (main.start.orchestrator 用).
# 触发条件: IR (红外接近) + dis_at_least_m (累计里程计) AND/OR.
# 换场地改这里, 业务代码不动.
waypoints:

  # ----- task1 -----
  - name: task1_seeding
    task_id: 1
    task_module: main.task.task1_seeding
    ir_threshold_m: 0.6
    ir_side: right
    dis_at_least_m: 1.00
    trigger_op: AND

  # ----- task2 -----
  - name: task2_water_tower
    task_id: 2
    task_module: main.task.task2_water_tower
    ir_threshold_m: 0.50
    ir_side: right
    dis_at_least_m: 5.20
    trigger_op: AND

  # ----- task3 (TODO) -----
  - name: task3_pest_scout
    task_id: 3
    task_module: main.task.task3_pest_scout
    ir_threshold_m: 0.50
    ir_side: right
    dis_at_least_m: 7.00
    trigger_op: AND

  # ----- task4 -----
  - name: task4_harvest
    task_id: 4
    task_module: main.task.task4_harvest
    ir_threshold_m: 0.50
    ir_side: right
    dis_at_least_m: 9.00
    trigger_op: AND

  # ----- task5 -----
  - name: task5_sort
    task_id: 5
    task_module: main.task.task5_sort
    ir_threshold_m: 0.50
    ir_side: right
    dis_at_least_m: 11.0
    trigger_op: AND

  # ----- task6 -----
  - name: task6_get_order
    task_id: 6
    task_module: main.task.task6_get_order
    ir_threshold_m: 0.50
    ir_side: left
    dis_at_least_m: 13.0
    trigger_op: AND

  # ----- task7 (TODO) -----
  - name: task7_deliver
    task_id: 7
    task_module: main.task.task7_deliver
    ir_threshold_m: 0.50
    ir_side: right
    dis_at_least_m: 14.5
    trigger_op: AND

  # ----- finish -----
  - name: cruise_done
    task_module: null
    ir_threshold_m: null
    dis_at_least_m: 16.5
    is_finish: true
```

**操作:**

```bash
cd /home/jetson/workspace/rak-car
# 在 task_config.yml 末尾追加 (注意: 文件目前以 task_cfg 段结尾)
# 用 Python 追加避免 yaml 缩进错
/usr/bin/python3 << 'PYEOF'
from pathlib import Path
import yaml

p = Path("task_config.yml")
data = yaml.safe_load(p.read_text(encoding="utf-8"))

# waypoints 段在 task_cfg 平级
data["waypoints"] = [
    {"name": "task1_seeding",     "task_id": 1, "task_module": "main.task.task1_seeding",
     "ir_threshold_m": 0.6, "ir_side": "right", "dis_at_least_m": 1.00, "trigger_op": "AND"},
    {"name": "task2_water_tower", "task_id": 2, "task_module": "main.task.task2_water_tower",
     "ir_threshold_m": 0.50, "ir_side": "right", "dis_at_least_m": 5.20, "trigger_op": "AND"},
    {"name": "task3_pest_scout",  "task_id": 3, "task_module": "main.task.task3_pest_scout",
     "ir_threshold_m": 0.50, "ir_side": "right", "dis_at_least_m": 7.00, "trigger_op": "AND"},
    {"name": "task4_harvest",     "task_id": 4, "task_module": "main.task.task4_harvest",
     "ir_threshold_m": 0.50, "ir_side": "right", "dis_at_least_m": 9.00, "trigger_op": "AND"},
    {"name": "task5_sort",        "task_id": 5, "task_module": "main.task.task5_sort",
     "ir_threshold_m": 0.50, "ir_side": "right", "dis_at_least_m": 11.0, "trigger_op": "AND"},
    {"name": "task6_get_order",   "task_id": 6, "task_module": "main.task.task6_get_order",
     "ir_threshold_m": 0.50, "ir_side": "left",  "dis_at_least_m": 13.0, "trigger_op": "AND"},
    {"name": "task7_deliver",     "task_id": 7, "task_module": "main.task.task7_deliver",
     "ir_threshold_m": 0.50, "ir_side": "right", "dis_at_least_m": 14.5, "trigger_op": "AND"},
    {"name": "cruise_done",       "task_module": None,
     "ir_threshold_m": None, "dis_at_least_m": 16.5, "is_finish": True},
]

# dump 回文件, 保留原 task_cfg 内容 (utf-8 + 中文注释 + allow_unicode)
p.write_text(
    yaml.safe_dump(data, allow_unicode=True, sort_keys=False, default_flow_style=False, width=120),
    encoding="utf-8",
)
print("OK")
PYEOF
```

### Step 2.4: 在 _config.py 加 `load_waypoints()`

修改 `main/task/_config.py`,在末尾追加:

```python
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
```

并在文件顶部加 `from typing import List`。

### Step 2.5: 跑测试验证通过

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pytest main/task/tests/test_orchestrator_yaml.py -v 2>&1 | tail -10
# 预期: 4 passed
```

### Step 2.6: Commit

```bash
cd /home/jetson/workspace/rak-car
git add task_config.yml main/task/_config.py main/task/tests/test_orchestrator_yaml.py
git commit -m "feat(task): task_config.yml 加 waypoints 段 + load_waypoints()

编排配置从 orchestrator.DEFAULT_WAYPOINTS 硬编码搬到 yaml:
- 8 task + 1 finish,每个含 task_id/task_module/IR/odom/trigger_op
- main/task/_config.py 加 load_waypoints() 返回 List[Dict]
- 4 smoke test (返回 list / 覆盖 7 task / 含 finish / 触发条件)

orchestrator 后续 task 改为读 yaml;换场地改 yaml 即可。

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 3: Add `main/task/task3_pest_scout.py` (NotImplementedError placeholder)

**Files:**
- Create: `main/task/task3_pest_scout.py`

**Interface:**
- `run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]` raises NotImplementedError

### Step 3.1: 写失败测试

`main/task/tests/test_task_skips_unimplemented.py`:

```python
"""验证 task3/task7 的 run() 抛 NotImplementedError 但 orchestrator 不会崩."""
from pathlib import Path
import sys

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import pytest  # noqa: E402


def test_task3_pest_scout_raises_not_implemented():
    from main.task.task3_pest_scout import run
    with pytest.raises(NotImplementedError) as exc_info:
        run()
    assert "task3_pest_scout" in str(exc_info.value).lower() or "害虫" in str(exc_info.value)


def test_task7_deliver_raises_not_implemented():
    from main.task.task7_deliver import run
    with pytest.raises(NotImplementedError):
        run()
```

### Step 3.2: 跑测试确认失败 (task3/7 不存在)

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pytest main/task/tests/test_task_skips_unimplemented.py -v 2>&1 | tail -10
# 预期: ModuleNotFoundError: No module named 'main.task.task3_pest_scout'
```

### Step 3.3: 写 task3_pest_scout.py

```python
#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task3 / task3_pest_scout —— 害虫侦察 + 射击 (TODO).

业务未实现. 期望功能:
  - 识别场地里的害虫 (YOLO 推理 / paddleocr 标牌识别)
  - 用 PoutD (气枪) 射击

实现时机: 等 task3 业务方案确认后,把具体逻辑写到 main.arm.each_task.task3/,
        然后本文件改为包装.

orchestrator 捕获 NotImplementedError 后 warning + 跳过, 不中断流程.
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """task3 入口. 当前 raise NotImplementedError."""
    raise NotImplementedError(
        "task3_pest_scout (害虫侦察 + 射击) 业务未实现, "
        "等业务方案确认后写到 main.arm.each_task.task3/ 再包装."
    )
```

### Step 3.4: 跑测试验证 task3 通过

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pytest main/task/tests/test_task_skips_unimplemented.py::test_task3_pest_scout_raises_not_implemented -v 2>&1 | tail -5
# 预期: PASS
```

### Step 3.5: Commit task3 (单独 commit 让 task7 复用相同模式)

```bash
cd /home/jetson/workspace/rak-car
git add main/task/task3_pest_scout.py main/task/tests/test_task_skips_unimplemented.py
git commit -m "feat(task): task3_pest_scout (NotImplementedError placeholder)

业务未实现的 task 占位:
- 害虫侦察 + 射击 (YOLO 识别 + PoutD 气枪)
- run() 抛 NotImplementedError, orchestrator 捕获后跳过

实现时机: 业务方案确认后写到 main.arm.each_task.task3/ 再包装。

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 4: Add `main/task/task7_deliver.py` (NotImplementedError placeholder)

**Files:**
- Create: `main/task/task7_deliver.py`

**Interface:** Same as task3.

### Step 4.1: 写 task7_deliver.py (跟 task3 模板一致)

```python
#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task7 / task7_deliver —— 投放外卖 (TODO).

业务未实现. 期望功能:
  - 把识别出来的蔬菜投放到外卖柜 (底盘外环 + 机械臂抓取)
  - 投放完后回到寻路位

实现时机: 等 task7 业务方案确认后,把具体逻辑写到 main.arm.each_task.task7/,
        然后本文件改为包装.

orchestrator 捕获 NotImplementedError 后 warning + 跳过, 不中断流程.
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """task7 入口. 当前 raise NotImplementedError."""
    raise NotImplementedError(
        "task7_deliver (投放外卖) 业务未实现, "
        "等业务方案确认后写到 main.arm.each_task.task7/ 再包装."
    )
```

### Step 4.2: 跑测试验证 task7 通过

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pytest main/task/tests/test_task_skips_unimplemented.py -v 2>&1 | tail -5
# 预期: 2 passed
```

### Step 4.3: Commit task7

```bash
cd /home/jetson/workspace/rak-car
git add main/task/task7_deliver.py
git commit -m "feat(task): task7_deliver (NotImplementedError placeholder)

业务未实现的 task 占位:
- 投放外卖 (识别结果 → 机械臂 → 外卖柜)
- run() 抛 NotImplementedError, orchestrator 跳过

实现时机: 业务方案确认后写到 main.arm.each_task.task7/ 再包装。

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 5: Add `main/task/task4_harvest.py` (包装 each_task/task4)

**Files:**
- Create: `main/task/task4_harvest.py`

**Interface:**
- `run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]`
- 内部 new `ArmClient(http=client or RuntimeApiClient())` + `ArmRunner(arm)`,调 `each_task.task4.target4.step_target4(arm, client, runner=runner)`
- 返回 `{"ok": bool, "task": "task4_harvest", "detail": result}`

### Step 5.1: 写失败测试

`main/task/tests/test_task_wrappers.py`:

```python
"""验证 task4/task5 包装层 signature + import chain."""
from pathlib import Path
import sys

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import inspect  # noqa: E402


def test_task4_harvest_run_signature():
    from main.task.task4_harvest import run
    sig = inspect.signature(run)
    params = list(sig.parameters.keys())
    # 第一参数名是 client (Optional[RuntimeApiClient])
    assert "client" in params
    assert sig.return_annotation != inspect.Signature.empty


def test_task5_sort_run_signature():
    from main.task.task5_sort import run
    sig = inspect.signature(run)
    params = list(sig.parameters.keys())
    assert "client" in params


def test_task4_wraps_each_task_target4():
    """task4 run() 内部应 import main.arm.each_task.task4.target4."""
    from main.task import task4_harvest
    src = inspect.getsource(task4_harvest)
    assert "each_task.task4" in src or "each_task/task4" in src, \
        f"task4 wrapper 似乎没引用 each_task.task4: {src[:200]}"


def test_task5_wraps_each_task_target():
    from main.task import task5_sort
    src = inspect.getsource(task5_sort)
    assert "each_task.task5" in src, f"task5 wrapper 似乎没引用 each_task.task5: {src[:200]}"
```

### Step 5.2: 跑测试确认 task4/5 wrapper 不存在

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pytest main/task/tests/test_task_wrappers.py -v 2>&1 | tail -10
# 预期: ModuleNotFoundError: No module named 'main.task.task4_harvest'
```

### Step 5.3: 写 task4_harvest.py

```python
#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task4 / task4_harvest —— 抓取作物.

业务逻辑: main.arm.each_task.task4.target4.step_target4
(4642 LOC, am 分支移植). 完整业务流程:
  - 摆臂到 target1 (y=-133, arm=+90°, hand=0°)
  - 循环: 底盘前移 + 视觉识别 + 抓球
  - 多轮抓取后回 init 位

本文件只做薄封装: new ArmClient + ArmRunner, 调 step_target4.
如果 client 已传就复用 (orchestrator 场景);否则内部 new RuntimeApiClient.

失败语义: step_target4 抛任何异常都往上抛, orchestrator 捕获.
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient
from main.arm import ArmClient, ArmRunner


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """task4 入口. 包装 each_task.task4.target4.step_target4.

    Args:
        client: 复用的 RuntimeApiClient;None 时内部 new.

    Returns:
        {"ok": bool, "task": "task4_harvest", "detail": step_target4 返回值}
    """
    cli = client or RuntimeApiClient()
    arm = ArmClient(http=cli)
    runner = ArmRunner(arm)

    # lazy import: each_task 包是业务代码,不进 cold path
    from main.arm.each_task.task4.target4 import step_target4

    detail = step_target4(
        arm_client=arm,
        http_client=cli,
        runner=runner,
    )

    ok = bool(detail.get("ok")) if isinstance(detail, dict) else bool(detail)
    return {"ok": ok, "task": "task4_harvest", "detail": detail}
```

### Step 5.4: 验证 task4 wrapper 可 import

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -c "
from main.task.task4_harvest import run
import inspect
print(f'sig: {inspect.signature(run)}')
print('task4 wrapper import: OK')
"
# 预期: import OK (不调用,避免硬件依赖)
```

### Step 5.5: Commit task4

```bash
cd /home/jetson/workspace/rak-car
git add main/task/task4_harvest.py main/task/tests/test_task_wrappers.py
git commit -m "feat(task): task4_harvest —— 包装 main.arm.each_task.task4.target4

薄封装:
- new ArmClient(http=client or RuntimeApiClient()) + ArmRunner
- lazy import step_target4 (avoid cold path bloat)
- 调 step_target4(arm_client, http_client, runner) → 包装 dict 返回

业务 4642 LOC 在 each_task.task4, 本文件 ~40 行包装。

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 6: Add `main/task/task5_sort.py` (包装 each_task/task5)

**Files:**
- Create: `main/task/task5_sort.py`

**Interface:**
- `run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]`
- 内部: `arm = ArmClient.connect()` + `runner = ArmRunner(arm)` → 调 `each_task.task5.the_final.main([])` (argv 模式) → 包装 return code

**关键差异 vs task4**: `the_final.main()` **不**接收 client 参数,内部 `client = ArmClient.connect()`。所以 task5 wrapper 不能透传 client。

### Step 6.1: 写 task5_sort.py

```python
#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""task5 / task5_sort —— 分拣作物.

业务逻辑: main.arm.each_task.task5.the_final.main
(4544 LOC, am 分支移植). 完整业务流程:
  1. 识别高仓颜色 (blue/yellow) → color A
  2. 同色球进高仓 (last_X_to_high)
  3. 底盘后撤 165mm
  4. 反色球进 LOW 仓 (last_X_to_low)

⚠️ the_final.main(argv) 不接收 client, 内部 client = ArmClient.connect().
   本 wrapper 因此忽略传入 client 参数 (call site 必须 no-arg).
"""
from __future__ import annotations

from typing import Any, Dict, Optional

from main.api_client import RuntimeApiClient


def run(client: Optional[RuntimeApiClient] = None) -> Dict[str, Any]:
    """task5 入口. 包装 each_task.task5.the_final.main.

    Args:
        client: 传入但被忽略 (the_final 不接收).

    Returns:
        {"ok": bool, "task": "task5_sort", "rc": main 的 return code, "detail": str}
    """
    # lazy import: each_task 包是业务代码
    from main.arm.each_task.task5 import the_final

    rc = the_final.main(argv=None)  # 用默认 CLI args (内部识别色)

    # the_final 用 EXIT_OK/EXIT_BAD_COLOR 常量
    ok = (rc == 0)  # EXIT_OK = 0
    return {"ok": ok, "task": "task5_sort", "rc": rc, "detail": "see the_final.main logs"}
```

### Step 6.2: 验证 task5 wrapper 可 import

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -c "
from main.task.task5_sort import run
import inspect
print(f'sig: {inspect.signature(run)}')
print('task5 wrapper import: OK')
"
# 预期: import OK
```

### Step 6.3: 跑 wrappers 测试验证全部 4 通过

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pytest main/task/tests/test_task_wrappers.py -v 2>&1 | tail -10
# 预期: 4 passed
```

### Step 6.4: Commit task5

```bash
cd /home/jetson/workspace/rak-car
git add main/task/task5_sort.py
git commit -m "feat(task): task5_sort —— 包装 main.arm.each_task.task5.the_final

薄封装:
- the_final.main() 不接收 client (内部 ArmClient.connect()), 所以忽略入参
- 调 main(argv=None) → 用默认 CLI args (内部识别颜色)
- 包装 return code: rc=0 → ok=True, else → ok=False

业务 4544 LOC 在 each_task.task5, 本文件 ~30 行包装。

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 7: Add `main/task/README.md` + Update orchestrator DEFAULT_WAYPOINTS

**Files:**
- Create: `main/task/README.md`
- Modify: `main/start/orchestrator.py`:
  - `Waypoint` dataclass 加 `task_id: Optional[int] = None` 字段
  - `DEFAULT_WAYPOINTS` 8 个 waypoint 改 `task_module` 路径为 `main.task.taskN_xxx`
  - 加 `task_id=1..7` 字段
  - `_run_task` 改: 不再 `importlib.import_module(wp.task_module)`, 而是 `TASK_RUNNERS[wp.task_id](client)`

### Step 7.1: 写失败测试

`main/task/tests/test_task_index.py`:

```python
"""验证 main/task 索引: TASK_RUNNERS 字典 1..7 全 callable."""
from pathlib import Path
import sys

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


def test_task_runners_keys_cover_1_to_7():
    from main.task import TASK_RUNNERS
    assert set(TASK_RUNNERS.keys()) == {1, 2, 3, 4, 5, 6, 7}


def test_task_runners_all_callable():
    from main.task import TASK_RUNNERS
    for tid, runner in TASK_RUNNERS.items():
        assert callable(runner), f"task {tid} 不是 callable: {runner}"


def test_task_runners_importable():
    """每个 task 模块都能 import (即使内部 raise NotImplementedError)。"""
    from main.task import (
        task1_seeding, task2_water_tower, task3_pest_scout,
        task4_harvest, task5_sort, task6_get_order, task7_deliver,
    )
    for m in [task1_seeding, task2_water_tower, task3_pest_scout,
              task4_harvest, task5_sort, task6_get_order, task7_deliver]:
        assert hasattr(m, "run"), f"{m.__name__} 缺 run()"
```

### Step 7.2: 跑测试

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pytest main/task/tests/test_task_index.py -v 2>&1 | tail -10
# 预期: 3 passed (已有 __init__.py, TASK_RUNNERS 已就绪)
```

### Step 7.3: 写 main/task/README.md

```markdown
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
```

### Step 7.4: 改 orchestrator `Waypoint` dataclass 加 `task_id`

`main/start/orchestrator.py` line 40-63 `Waypoint` dataclass:

**改动**: 在 `name` 后加 `task_id: Optional[int] = None`

```python
@dataclass
class Waypoint:
    """一个任务点位.

    Attributes:
        task_id:          任务编号 (1..7), 用于 TASK_RUNNERS[id] 查表. None 表示纯导航/finish.
        name:             人类可读名字,出现在日志.
        task_module:      (兼容旧字段,实际不再用 —— orchestrator 走 TASK_RUNNERS).
        ir_threshold_m:   IR 接近阈值 (None 表示不参与 IR 判断).
        ir_side:          IR 哪一侧触发: "left" / "right" / "any" (默认 "right").
        dis_at_least_m:   累计里程计 ≥ 该值才算"到了这个点" (None 表示不参与).
        trigger_op:       "AND" (默认,严格防误触) / "OR".
        pause_before_s:   触发后、调 task 前的停顿.
        pause_after_s:    任务跑完、恢复巡线前的停顿.
        is_finish:        True = 这是终点 (里程计达到即整个流程结束).
    """
    task_id: Optional[int] = None
    name: str = ""
    task_module: Optional[str] = None
    ir_threshold_m: Optional[float] = None
    ir_side: str = "right"
    dis_at_least_m: Optional[float] = None
    trigger_op: str = "AND"
    pause_before_s: float = 0.0
    pause_after_s: float = 0.0
    is_finish: bool = False
```

注意: 把 `name` 放第二、`task_id` 第一,因为 `name` 在 dataclass 里如果没默认值的字段必须放最前——所以**把 `task_id` 设为有默认值 (Optional[int] = None)**,这样 `name` (无默认) 仍在第一个无默认值位置,代码兼容。

### Step 7.5: 改 orchestrator DEFAULT_WAYPOINTS

替换 `main/start/orchestrator.py` line 67-95 的 `DEFAULT_WAYPOINTS` 列表:

```python
# 默认 8 任务点位 + 1 终点. 换场地改 task_config.yml 的 waypoints 段 (业务代码不动).
# 保留 DEFAULT_WAYPOINTS 作为 fallback —— 启动时优先从 yaml 加载.
DEFAULT_WAYPOINTS: List[Waypoint] = [
    Waypoint(task_id=1, name="task1_seeding",
             ir_threshold_m=0.6, ir_side="right",
             dis_at_least_m=1.00, trigger_op="AND"),
    Waypoint(task_id=2, name="task2_water_tower",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=5.20, trigger_op="AND"),
    Waypoint(task_id=3, name="task3_pest_scout",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=7.00, trigger_op="AND"),
    Waypoint(task_id=4, name="task4_harvest",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=9.00, trigger_op="AND"),
    Waypoint(task_id=5, name="task5_sort",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=11.0, trigger_op="AND"),
    Waypoint(task_id=6, name="task6_get_order",
             ir_threshold_m=0.50, ir_side="left",
             dis_at_least_m=13.0, trigger_op="AND"),
    Waypoint(task_id=7, name="task7_deliver",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=14.5, trigger_op="AND"),
    # 终点: 里程计达到 16.5m → 整个流程结束
    Waypoint(name="cruise_done", ir_threshold_m=None,
             dis_at_least_m=16.5, is_finish=True),
]
```

### Step 7.6: 改 orchestrator `_run_task` 走 `TASK_RUNNERS`

`main/start/orchestrator.py` line 296-314 `_run_task`:

```python
@staticmethod
def _run_task(client: RuntimeApiClient, wp: Waypoint) -> bool:
    """按 task_id 查 TASK_RUNNERS 字典, 调 run(). 返回 True 表示成功."""
    if wp.task_id is None:
        # 纯导航段或 finish, 不应到这里
        return True
    try:
        from main.task import TASK_RUNNERS
        runner = TASK_RUNNERS[wp.task_id]
    except (ImportError, KeyError) as exc:
        logger.warning("task_id=%d not registered in TASK_RUNNERS: %s",
                       wp.task_id, exc)
        return False
    try:
        result = runner(client)
    except NotImplementedError as exc:
        # 未实现 task (3/7) 抛 NotImplementedError, warning + 跳过
        logger.warning("task_id=%d not implemented, skipping: %s",
                       wp.task_id, exc)
        return False
    except Exception:
        logger.exception("task %s raised exception", wp.name)
        return False
    if isinstance(result, dict) and not result.get("ok"):
        logger.warning("task %s failed: %s", wp.name,
                       result.get("error", result.get("detail", "?")))
        return False
    logger.info("task %s (id=%d) succeeded -> %s", wp.name, wp.task_id, result)
    return True
```

### Step 7.7: 在 Orchestrator.__init__ 加 yaml 加载

修改 `Orchestrator.__init__` (line 101-107):

```python
def __init__(self,
             waypoints: Optional[List[Waypoint]] = None,
             lane_hz: float = 50.0,
             ir_interval_s: float = 0.1,
             config_path: Optional[str] = None):
    """config_path: 自定义 task_config.yml 路径, None 走默认 (根目录 task_config.yml)."""
    if waypoints is not None:
        self.waypoints = waypoints
    else:
        # 优先从 yaml 加载; 失败 fallback DEFAULT_WAYPOINTS
        self.waypoints = self._load_waypoints_from_yaml(config_path) or DEFAULT_WAYPOINTS
    self.lane_hz = lane_hz
    self.ir_interval_s = ir_interval_s

@staticmethod
def _load_waypoints_from_yaml(config_path: Optional[str]) -> Optional[List[Waypoint]]:
    """从 yaml 加载 waypoints, 失败返 None."""
    try:
        from main.task._config import load_waypoints
        wp_dicts = load_waypoints()
    except (FileNotFoundError, KeyError, ValueError) as exc:
        logger.warning("yaml load_waypoints failed, fallback DEFAULT_WAYPOINTS: %s", exc)
        return None
    out = []
    for w in wp_dicts:
        out.append(Waypoint(
            task_id=w.get("task_id"),
            name=w.get("name", ""),
            task_module=w.get("task_module"),  # 保留旧字段, 不参与 _run_task
            ir_threshold_m=w.get("ir_threshold_m"),
            ir_side=w.get("ir_side", "right"),
            dis_at_least_m=w.get("dis_at_least_m"),
            trigger_op=w.get("trigger_op", "AND"),
            is_finish=w.get("is_finish", False),
        ))
    return out

### Step 7.8: 验证 orchestrator 改动可 import

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -c "
from main.start.orchestrator import Orchestrator, DEFAULT_WAYPOINTS, Waypoint
print(f'DEFAULT_WAYPOINTS count: {len(DEFAULT_WAYPOINTS)}')
print(f'task_ids: {[w.task_id for w in DEFAULT_WAYPOINTS]}')
print(f'is_finish: {[w.is_finish for w in DEFAULT_WAYPOINTS]}')

# yaml 加载路径
orch = Orchestrator()
print(f'loaded waypoints: {len(orch.waypoints)} (from yaml)')
for w in orch.waypoints[:3]:
    print(f'  {w.task_id} {w.name} IR={w.ir_threshold_m} dis={w.dis_at_least_m}')
"
# 预期: DEFAULT_WAYPOINTS count=8, task_ids=[1,2,3,4,5,6,7,None], yaml 加载 8 个
```

### Step 7.9: 跑全部 smoke tests

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pytest main/arm/tests/ main/chassis/ main/task/tests/ --tb=short -q 2>&1 | tail -10
# 预期: 85+8+smoke 全过 (具体数 taskN 各 1-3 个 test)
```

### Step 7.10: Commit

```bash
cd /home/jetson/workspace/rak-car
git add main/task/README.md main/start/orchestrator.py
git commit -m "feat(orchestrator): DEFAULT_WAYPOINTS 改 task_id + yaml 驱动

Waypoint 加 task_id 字段; DEFAULT_WAYPOINTS 重写为 8 task + 1 finish.
新增 _load_waypoints_from_yaml(): 优先从 task_config.yml 加载, 失败
fallback DEFAULT_WAYPOINTS. _run_task 走 TASK_RUNNERS[task_id](client)
而非 importlib.import_module.

NotImplementedError 捕获: task3/task7 抛 NotImpl → warning + 跳过,
不中断整体流程.

main/task/README.md 新建: 8 任务清单 + 目录结构 + 调用约定 + 维护指南.

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Task 8: End-to-end smoke + Final commit

**Files:**
- Run-only (no code change): `python3 -c "from main.start.orchestrator import Orchestrator; ..."`
- Test: `main/task/tests/test_e2e_imports.py` (新建)

### Step 8.1: 写 e2e smoke test

```python
"""e2e: orchestrator + 全部 task + arm/each_task + runtime_guard 都能 import."""
from pathlib import Path
import sys

_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


def test_orchestrator_imports():
    from main.start.orchestrator import Orchestrator, DEFAULT_WAYPOINTS, Waypoint
    assert len(DEFAULT_WAYPOINTS) == 8


def test_all_8_tasks_importable():
    from main.task import TASK_RUNNERS
    assert set(TASK_RUNNERS.keys()) == {1, 2, 3, 4, 5, 6, 7}


def test_arm_each_task_imports():
    from main.arm.each_task import common
    from main.arm.each_task.task4 import target4
    from main.arm.each_task.task5 import the_final
    assert hasattr(target4, "step_target4")
    assert hasattr(the_final, "main")


def test_orchestrator_yaml_loads():
    from main.start.orchestrator import Orchestrator
    orch = Orchestrator()
    assert len(orch.waypoints) >= 8
    task_ids = [w.task_id for w in orch.waypoints if w.task_id is not None]
    assert set(task_ids) == {1, 2, 3, 4, 5, 6, 7}


def test_runtime_guard_imports():
    from main.arm.runtime_guard._runtime_guard import preflight, postflight
    assert callable(preflight)
    assert callable(postflight)
```

### Step 8.2: 跑 e2e

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pytest main/task/tests/test_e2e_imports.py -v 2>&1 | tail -15
# 预期: 5 passed
```

### Step 8.3: 全量跑 main/task + 现有

```bash
cd /home/jetson/workspace/rak-car
/usr/bin/python3 -m pytest main/task/tests/ main/arm/tests/ main/chassis/ --tb=short -q 2>&1 | tail -10
# 预期: 全部 passed (具体数累计)
```

### Step 8.4: 最终 commit (如果 test_e2e_imports.py 是新文件)

```bash
cd /home/jetson/workspace/rak-car
git add main/task/tests/test_e2e_imports.py
git commit -m "test(task): e2e import smoke (orchestrator + 8 task + each_task + runtime_guard)

5 个 smoke 覆盖:
- orchestrator DEFAULT_WAYPOINTS = 8
- TASK_RUNNERS 字典 1..7
- arm/each_task/{common,task4.target4,task5.the_final}
- Orchestrator() yaml 加载 8 waypoint
- main.arm.runtime_guard preflight/postflight

Co-Authored-By: Claude <noreply@anthropic.com>"
```

---

## Self-Review Checklist (执行前过一遍)

- [ ] **Spec coverage**: 设计文档 5 个点都对应到任务了 (renaming / yaml / 4 个 wrapper / README / orchestrator 改动)
- [ ] **Placeholder scan**: 全文 grep "TBD" "TODO" "implement later" "fill in details" → 0 处 (TODO 在 task3/7 placeholder docstring 是设计意图, 不是 plan placeholder)
- [ ] **Type consistency**:
  - `load_waypoints()` 返回 `List[Dict[str, Any]]`, 在 orchestrator `_load_waypoints_from_yaml` 接收为 `wp_dicts: List[Dict]` ✓
  - `Waypoint` 新字段 `task_id: Optional[int]`, 所有 waypoint 构造处都用 `task_id=N` 或省略 ✓
  - `TASK_RUNNERS: Dict[int, Callable]` → orchestrator `_run_task` 索引 `[wp.task_id]` ✓
  - `task4_harvest.run(client: Optional[RuntimeApiClient] = None)` 跟 task1/2/6 一致 ✓
  - `task5_sort.run(client: Optional[RuntimeApiClient] = None)` 忽略 client, docstring 注明 ✓
- [ ] **Test naming**: 所有 test 文件前缀 `test_` + 描述性 snake_case
- [ ] **Commit messages**: 中文 `feat/fix/refactor/test/chore(scope):` 格式
- [ ] **No new deps**: 全程 PyYAML (已用)
- [ ] **85 existing tests**: Task 1 Step 1.8 强制验证

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-08-01-task-orchestration-rewrite.md`.

**执行选项:**
1. **Subagent-Driven (推荐)** - 每个 Task 派一个 subagent,review between tasks,快速迭代
2. **Inline Execution** - 当前 session 执行,带 checkpoint 批执行