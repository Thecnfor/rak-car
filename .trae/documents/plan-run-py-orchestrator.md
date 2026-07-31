# run.py 高级调度入口 — 计划

## Summary

把 `run.py`（仓库根目录）做成**唯一的全流程入口**，替代 `main/start/whole_no_task.py` 的「巡线 + IR 触发暂停」架构。

### **极薄壳 + 调度核心拆层**

| 层          | 文件                                                | 角色                                                                                              |
| ---------- | ------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| **入口壳**（薄） | `run.py`（根目录） | 解析 CLI、调 `Orchestrator().run()`。**≤ 15 行** |
| **调度核心** | `main/start/orchestrator.py` | `_lane_loop` 后台线程、`_wait_until_triggered`、`_pause_lane`、`_resume_lane`、`_run_task` —— 全部实现细节在这里，**默认 8 任务点位表 `DEFAULT_WAYPOINTS` 也在本文件** |

设计原则：

* **`run.py`** **极薄壳**：高级、简洁。只做两件事 —— (1) 解析 CLI (2) `Orchestrator().run()`。不写线程、不写循环、不读 yaml。≤ 15 行。
* **点位表直接放 `main/start/orchestrator.py` 顶部**，作为模块级常量 `DEFAULT_WAYPOINTS`。换场地改代码这个常量即可（之后如果再要剥离，再渐进移到 yaml —— 但**当前不引入配置文件**）。
* **main/* 是唯一任务来源**：`main.tasks.*` 通过 runtime HTTP API 走车端。
* **每个点位的「IR + 里程计」AND/OR 组合语义留给点位定义时填**，默认 AND。
* **`main/start/` 不再被删** —— 改为 `main/start/orchestrator.py`，成为 run.py 的实现后端。

***

## Current State Analysis

### 现有入口架构

| 入口                            | 作用                                    | 是否保留                      |
| ----------------------------- | ------------------------------------- | ------------------------- |
| `car_start_2026.py`（根目录）      | legacy monolith，全局 `MyCar()` 单例跑 8 任务 | 不动（老路径，不归 run.py 管）       |
| `runtime/server`（FastAPI）     | runtime 服务，pm2 守护 `rak-car-api`       | 不动                        |
| `main/car_start_api.py`       | API 风格 8 任务编排模板（全注释）                  | 不动（是客户端 demo，不归 run.py 管） |
| `main/start/whole_no_task.py` | 巡线 + IR 触发暂停（不跑任务）                    | **删除**                    |
| `main/start/__init__.py`      | 空 `main/start` 子包入口                   | **删除**                    |
| `run.py`（根目录）                 | 空文件（0 字节）                             | **填为最终入口**                |

### 已有可复用的部件（run.py 直接 import）

| 模块                                                                | 提供                                                         | 文件                                                                                                                                                                                             |
| ----------------------------------------------------------------- | ---------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `main.chassis.subscribe_lane_state`                               | 一键装配外环：profile → outer / smoother → DoubleLoopRunner       | [main/chassis/__init__.py](file:///home/jetson/workspace/rak-car/main/chassis/__init__.py)                                                                                                     |
| `main.chassis.ChassisClient`                                      | 底盘 HTTP/WS client（IR、odometry、轮速下发、start/stop\_lane\_feed） | [main/chassis/api.py](file:///home/jetson/workspace/rak-car/main/chassis/api.py)                                                                                                               |
| `main.chassis.tasks.read_ir`                                      | 读左右红外距离，返回 dict                                            | [main/chassis/tasks/read\_ir.py](file:///home/jetson/workspace/rak-car/main/chassis/tasks/read_ir.py)                                                                                          |
| `main.chassis.tasks.read_dis`                                     | 累计里程计回调                                                    | [main/chassis/tasks/read\_dis.py](file:///home/jetson/workspace/rak-car/main/chassis/tasks/read_dis.py)                                                                                        |
| `main.tasks.auto_seeding.run` / `main.tasks.water_tower_task.run` | 8 任务中两个已实现的入口                                              | [auto\_seeding.py](file:///home/jetson/workspace/rak-car/main/tasks/auto_seeding.py#L301) / [water\_tower\_task.py](file:///home/jetson/workspace/rak-car/main/tasks/water_tower_task.py#L338) |
| `main.api_client.RuntimeApiClient`                                | 通用 runtime HTTP 客户端                                        | [main/api\_client.py](file:///home/jetson/workspace/rak-car/main/api_client.py)                                                                                                                |
| `main.tasks._config.load_task_config`                             | 读根目录 `task_config.yml` 的 `task_cfg.<name>` 段               | [main/tasks/\_config.py](file:///home/jetson/workspace/rak-car/main/tasks/_config.py)                                                                                                          |

### `whole_no_task.py` 的核心调度模式（要搬到 run.py）

```
后台线程 A — _lane_loop：50Hz 读 lane_state → 计算轮速 → 下发（受 running Event 控制）
后台线程 B — read_dis：20Hz 读累计里程计，写入共享 buffer
主线程   — 状态机 LANE → 读 IR → 触发 → running.clear() + stop_wheel_speeds() + pause → running.set()
```

**关键约束**：触发时**必须主动** **`stop_wheel_speeds()`** + `sleep(0.1)`，否则外环线程 `wait()` 到下一帧之间车端会沿最后一帧轮速继续跑。

***

## Proposed Changes

### 1. 写 `/home/jetson/workspace/rak-car/run.py` —— **极薄壳（≤ 20 行）**

```python
#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""run.py — 全流程入口（极薄壳）。

真正的调度逻辑在 main.start.orchestrator.Orchestrator。
本文件只做：① 加 sys.path ② 解析 CLI ③ Orchestrator().run()。
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.start.orchestrator import Orchestrator


def main() -> None:
    p = argparse.ArgumentParser(prog="run.py", description="rak-car 全流程入口")
    p.add_argument("--lane-hz", type=float, default=50.0)
    p.add_argument("--ir-interval-s", type=float, default=0.1)
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(message)s")
    Orchestrator(lane_hz=args.lane_hz,
                 ir_interval_s=args.ir_interval_s).run()


if __name__ == "__main__":
    main()
```

> 跑前从仓库根目录执行 `python3 run.py` 即可。`main.*` 通过 sys.path 由 orchestrator 内部处理。

### 2. 新增 `/home/jetson/workspace/rak-car/main/start/orchestrator.py`

所有调度实现细节（~200 行）：`Waypoint` dataclass、`DEFAULT_WAYPOINTS` 常量、`Orchestrator` class、后台线程、触发判断、任务调用。

```python
DEFAULT_WAYPOINTS = [
    # name,            task_module,                  ir,    ir_side, dis,   op,    is_finish
    Waypoint("seed",        "main.tasks.auto_seeding",      0.50, "right", 1.20, "AND", False),
    Waypoint("scout_pests", "main.tasks.scout_pests",      0.50, "right", 3.50, "AND", False),
    Waypoint("water",       "main.tasks.water_tower_task", 0.50, "right", 5.20, "AND", False),
    Waypoint("shoot_pests", "main.tasks.target_shooting",  0.50, "right", 7.00, "AND", False),
    Waypoint("harvest",     "main.tasks.crop_harvesting",  0.50, "right", 9.00, "AND", False),
    Waypoint("sort",        "main.tasks.sort_and_store",   0.50, "right", 11.0, "AND", False),
    Waypoint("ocr",         "main.tasks.get_order",        0.50, "left",  13.0, "AND", False),
    Waypoint("deliver",     "main.tasks.order_delivery",   0.50, "right", 14.5, "AND", False),
    # 终点：里程计达到 16.5m → 整个流程结束
    Waypoint("cruise_done", task_module=None, ir_threshold_m=None,
             dis_at_least_m=16.5, is_finish=True),
]
```

> 当前 8 个任务模块里只有 `main.tasks.auto_seeding` 和 `main.tasks.water_tower_task` 已实现，其余未实现 —— `_run_task` 走 `importlib.import_module` + try/except 跳过，不崩。

### 2. 完整 `main/start/orchestrator.py`lane_hz=50, ir_interval_s=0.1)`

```python
def run(waypoints, lane_hz=50.0, ir_interval_s=0.1):
    """调度骨架：

      后台 A — 巡线后台线程 (受 running Event 控制)
      后台 B — 里程计 read_dis，写入共享 buffer _dis_buf[0]
      主线程 — 顺序遍历 waypoints：等待触发 → 停外环 + 发零速 → 调 task → 恢复外环
    """
    client = RuntimeApiClient()
    if not client.wait_until_ready(timeout=10.0):
        raise RuntimeError("runtime not ready (pm2 logs rak-car-api)")
    api = ChassisClient.connect()
    api.start_lane_feed(hz=lane_hz)

    # 详细实现见下方完整 `main/start/orchestrator.py` 代码块。
```

### 2. 完整 `main/start/orchestrator.py`

下面是 orchestrator.py 的完整代码（~200 行），包含 `Waypoint` / `DEFAULT_WAYPOINTS` / `Orchestrator` / `_lane_loop` / `_wait_until_triggered` / `_pause_lane` / `_resume_lane` / `_run_task`。

```python
#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""main/start/orchestrator.py

run.py 的实现后端：巡线后台线程 + 里程计后台线程 + 主线程点位调度。
所有 main.start 之外的脚本都不应该 import 本文件 —— 只服务于 run.py。
"""
from __future__ import annotations

import importlib
import logging
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

# 让 main.start.orchestrator 可被仓库根目录的 run.py 直接 import
_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from main.api_client import RuntimeApiClient
from main.chassis.api import ChassisClient
from main.chassis.controllers.base import WheelSmoother
from main.chassis.controllers.curvature_adaptive import CurvatureAdaptiveOuterLoop
from main.chassis.loops.safety import EmergencyWatchdog
from main.chassis.tasks.read_dis import read_dis
from main.chassis.tasks.read_ir import read_ir

logger = logging.getLogger("main.start.orchestrator")


@dataclass
class Waypoint:
    """一个任务点位。"""
    name: str
    task_module: Optional[str] = None
    ir_threshold_m: Optional[float] = None
    ir_side: str = "right"
    dis_at_least_m: Optional[float] = None
    trigger_op: str = "AND"
    pause_before_s: float = 0.0
    pause_after_s: float = 0.0
    is_finish: bool = False


# 换场地改这里 —— 8 任务点位 + 1 终点
DEFAULT_WAYPOINTS: List[Waypoint] = [
    Waypoint("seed",        task_module="main.tasks.auto_seeding",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=1.20, trigger_op="AND"),
    Waypoint("scout_pests", task_module="main.tasks.scout_pests",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=3.50, trigger_op="AND"),
    Waypoint("water",       task_module="main.tasks.water_tower_task",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=5.20, trigger_op="AND"),
    Waypoint("shoot_pests", task_module="main.tasks.target_shooting",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=7.00, trigger_op="AND"),
    Waypoint("harvest",     task_module="main.tasks.crop_harvesting",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=9.00, trigger_op="AND"),
    Waypoint("sort",        task_module="main.tasks.sort_and_store",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=11.0, trigger_op="AND"),
    Waypoint("ocr",         task_module="main.tasks.get_order",
             ir_threshold_m=0.50, ir_side="left",
             dis_at_least_m=13.0, trigger_op="AND"),
    Waypoint("deliver",     task_module="main.tasks.order_delivery",
             ir_threshold_m=0.50, ir_side="right",
             dis_at_least_m=14.5, trigger_op="AND"),
    Waypoint("cruise_done", task_module=None, ir_threshold_m=None,
             dis_at_least_m=16.5, is_finish=True),
]


class Orchestrator:
    """巡线导航 + 任务点位调度器。"""

    def __init__(self, waypoints: Optional[List[Waypoint]] = None,
                 lane_hz: float = 50.0, ir_interval_s: float = 0.1):
        self.waypoints = waypoints if waypoints is not None else DEFAULT_WAYPOINTS
        self.lane_hz = lane_hz
        self.ir_interval_s = ir_interval_s

    def run(self) -> None:
        client = RuntimeApiClient()
        if not client.wait_until_ready(timeout=10.0):
            raise RuntimeError("runtime not ready (pm2 logs rak-car-api)")
        api = ChassisClient.connect()
        api.start_lane_feed(hz=self.lane_hz)

        running = threading.Event(); running.set()
        threading.Thread(target=self._lane_loop,
                         args=(api, running), daemon=True, name="lane").start()

        dis_buf = [0.0]
        threading.Thread(target=read_dis,
                         kwargs={"api": api, "hz": 20.0,
                                 "on_tick": lambda v: dis_buf.__setitem__(0, v)},
                         daemon=True, name="distance").start()

        completed: List[str] = []
        try:
            for wp in self.waypoints:
                logger.info("=== navigating to %s ===", wp.name)
                self._wait_until_triggered(wp, api, dis_buf)
                if wp.is_finish:
                    logger.info("finish waypoint reached, mission done")
                    completed.append(wp.name); break
                self._pause_lane(api, running)
                time.sleep(wp.pause_before_s)
                if wp.task_module:
                    self._run_task(client, wp)
                time.sleep(wp.pause_after_s)
                self._resume_lane(running)
                completed.append(wp.name)
        except KeyboardInterrupt:
            logger.info("interrupted by user")
        finally:
            running.clear()
            try: api.stop_wheel_speeds()
            except Exception: pass
            api.close()
            logger.info("mission completed: %s", completed)

    def _lane_loop(self, api: ChassisClient, running: threading.Event) -> None:
        outer = CurvatureAdaptiveOuterLoop()
        smoother = WheelSmoother()
        watchdog = EmergencyWatchdog(threshold_ms=500.0)
        dt = 1.0 / max(self.lane_hz, 1.0)
        while True:
            running.wait()
            smoother.reset([0.0, 0.0, 0.0, 0.0])
            t0 = time.monotonic()
            state = api.read_lane()
            if watchdog.should_stop(state):
                try: api.emergency_stop()
                except Exception: pass
                time.sleep(dt); continue
            raw = outer.step(state, dt)
            safe = smoother.step(raw)
            try: api.set_wheel_speeds(safe)
            except Exception: pass
            sleep_s = dt - (time.monotonic() - t0)
            if sleep_s > 0: time.sleep(sleep_s)

    @staticmethod
    def _pause_lane(api: ChassisClient, running: threading.Event) -> None:
        running.clear()
        try: api.stop_wheel_speeds()
        except Exception: pass
        time.sleep(0.1)

    @staticmethod
    def _resume_lane(running: threading.Event) -> None:
        running.set()

    @staticmethod
    def _wait_until_triggered(wp: Waypoint, api: ChassisClient,
                              dis_buf: list, interval_s: float = 0.1) -> None:
        while True:
            ir: dict = {}
            try: ir = read_ir(api, timeout=2.0)
            except Exception: pass
            right, left = ir.get("right"), ir.get("left")
            dis = dis_buf[0]
            if wp.ir_threshold_m is None:
                ir_ok = True
            elif wp.ir_side == "left":
                ir_ok = left is not None and left < wp.ir_threshold_m
            elif wp.ir_side == "any":
                ir_ok = ((left is not None and left < wp.ir_threshold_m) or
                         (right is not None and right < wp.ir_threshold_m))
            else:
                ir_ok = right is not None and right < wp.ir_threshold_m
            dis_ok = (wp.dis_at_least_m is None or dis >= wp.dis_at_least_m)
            hit = (ir_ok and dis_ok) if wp.trigger_op == "AND" else (ir_ok or dis_ok)
            if hit:
                logger.info("triggered: %s (ir_left=%s ir_right=%s dis=%.2f)",
                            wp.name, left, right, dis)
                return
            time.sleep(interval_s)

    @staticmethod
    def _run_task(client: RuntimeApiClient, wp: Waypoint) -> None:
        try:
            mod = importlib.import_module(wp.task_module)
        except ImportError:
            logger.warning("task module %s not implemented, skipping", wp.task_module)
            return
        try:
            result = mod.run(client)
            logger.info("task %s -> %s", wp.name, result)
        except Exception:
            logger.exception("task %s failed", wp.name)


__all__ = ["Waypoint", "Orchestrator", "DEFAULT_WAYPOINTS"]
```

### 3. 删 `main/start/whole_no_task.py`

* `git rm main/start/whole_no_task.py`

* `main/start/__init__.py` **保留**（orchestrator 也是 main.start 子包的一部分）

### 3. 不动 `task_config.yml`

点位表直接是 Python 常量 `DEFAULT_WAYPOINTS`，**不引入配置文件**。之后如果要剥离，再渐进迁移到 yaml —— 当前不做。

### 4. 更新 `CLAUDE.md` 第 1 节

新增一行入口描述：

> * `python run.py` — **全流程入口**（替代 `main/start/whole_no_task.py`）：薄壳调 `main.start.orchestrator.Orchestrator` 跑全程巡线 + 8 任务点位 IR/里程计双触发编排。点位表是 `main.start.orchestrator.DEFAULT_WAYPOINTS`。

## Assumptions & Decisions

1. **里程计语义**：里程计**全程累计**（背景线程一直读），但**只在两个标记点位进入「动作」判断**：

   * **任务点**：进入 IR 阈值 + 里程计达到该任务点预设距离 → 暂停巡线、调 task

   * **终点**：里程计达到终点距离 → 全流程结束

   * 其他 IR 触发但里程计未到任务点的，视为误触（障碍物等），不进入动作
2. **IR 分左右**：`Waypoint` 加 `ir_side` 字段（默认 `"right"`，因为现有 whole\_no\_task.py 用右侧 IR 触发），点位需要时显式声明左右
3. **「IR + 里程计都参与」= AND 语义**：默认严格防误触——里程计是该点的「解锁锁」，IR 是「启动键」
4. **`ir_threshold_m`** **和** **`dis_at_least_m`** **都可为 None**：都 None 表示「纯导航段」，即触发永远为真，进入即跑（用于 `cruise_done` 收尾段）。
5. **任务模块未实现 = 跳过，不崩**：用 `importlib.import_module` + try/except，避免 8 个任务没全做完时整个 run.py 启动不了。
6. **任务失败不致命**：`mod.run(...)` 抛异常只记日志、继续下一个点位。比赛现场宁愿跑下一个也别停死。
7. **`main.start`** **子包不删，反而加** **`orchestrator.py`**：orchestrator 是 run.py 的实现后端，`main/start/__init__.py` 必须保留（提供 `from main.start.orchestrator import ...` 的命名空间）。`whole_no_task.py` 单文件删除。
8. **不复用** **`subscribe_lane_state`**：它内部 `runner.run(max_seconds=...)` 是阻塞的、没法 pause。orchestrator 自己实现一个 `_lane_loop(api, running)` 后台线程，照抄 whole\_no\_task.py 的写法；这部分代码本地化到 orchestrator.py，不污染 main/chassis。
9. **不复用** **`DoubleLoopRunner`**：它也是阻塞的。orchestrator 自己写 25 行 \_lane\_loop，包含 outer / smoother / watchdog，逻辑与 DoubleLoopRunner 等价但支持 Event pause。
10. **点位表是 Python 常量 `DEFAULT_WAYPOINTS`，不引入 yaml**：换场地改 orchestrator.py 顶部的常量即可。当前项目已经在去配置化的方向上，task_config.yml 里的 task_cfg 是另一回事（运行时任务参数），不要混。
11. **不加 --dry-run**：极薄壳不需要 dry-run（参数只有 2 个），跑 `python3 run.py --help` 即可看接口。

***

## Verification

执行顺序：

1. **语法检查**：`python3 -m py_compile run.py main/start/orchestrator.py` → 必须无错
2. **依赖检查**：`grep -rn "main.start.whole_no_task" main/ runtime/ smartcar/ test/ 2>/dev/null` → 必须 0 匹配
3. **import 自检**：`python3 -c "from main.start.orchestrator import Orchestrator, Waypoint, DEFAULT_WAYPOINTS; print(len(DEFAULT_WAYPOINTS))"` → 应当打印 `9`
4. **CLI 自检**：`python3 run.py --help` 应当只显示 `--lane-hz` / `--ir-interval-s`，立刻退出 0
5. **现场试跑**：在 Jetson 上确认 `pm2 status` → `rak-car-api` 在线，再 `python3 run.py`，观察 `lane_feed started`、`=== navigating to seed ===` 日志

完成标准：

* `run.py` ≤ 20 行（不含空行/注释），仅做 sys.path + argparse + Orchestrator().run()
* `main/start/orchestrator.py` 是 ~200 行实现（`Waypoint` + `DEFAULT_WAYPOINTS` + `Orchestrator` + 5 个辅助方法）
* `main/start/whole_no_task.py` 已删；`main/start/__init__.py` 保留
* `git status` 只显示 3 个变更：`run.py`（新）、`main/start/orchestrator.py`（新）、`main/start/whole_no_task.py`（删）、`CLAUDE.md`（小更新）
* 远端 main 收到一个新 commit

