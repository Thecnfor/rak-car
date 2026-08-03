# main/ —— 业务客户端（依赖 runtime HTTP / WS）

> **`main/` 永远不会 import `runtime/services/my_car/`**，所有调用走 HTTP / WS。
> 这条边界是为了让**业务层**和**车端 runtime** 可以独立部署、独立热更、独立调试。

---

## 写 task 先查这张表（现成方法速查）

`main/arm/__init__.py` 和 `main/chassis/__init__.py` 已经把这些都导出好了，**直接 import 用，不要自己拼 HTTP**：

| 要做的事 | 直接调 |
| --- | --- |
| 任意子集姿态一步到位（并行，快） | `ArmClient.composite_run(arm=, x_mm=, y_mm=, hand=)` |
| 视觉伺服对准 + 吸/放（task1/2/4/5 主力） | `runner.track_velocity_pick(label, mode="pick"/"drop")` |
| 视觉找目标 → 下降 → 吸 | `runner.pick_by_vision(selector)` / `runner.pick_by_vision_lower(selector)` |
| 抓取 / 释放（带安全门的复合动作） | `runner.pick(arm_angle, x_mm, y_mm)` / `runner.release(...)` |
| 底盘把目标拉到画面中心（cx→vx, cy→vy） | `main.chassis.track_chassis(target=, select_mode=)` |
| 底盘按检测面积前后对准（task5 料箱） | `main.chassis.make_align_runner(ref_area=).run(max_seconds=)` |
| 底盘相对位移（**唯一合法平移方式**） | `ChassisClient.connect().move_for(dx_m=)` |
| 读里程计 (x, y, theta) | `main.chassis.get_odometry()` |
| 一键巡线（profile → 外环 → 50Hz 主循环） | `main.chassis.subscribe_lane_state(profile=LANE_FOLLOW)` |
| 一键面积视觉对准 | `main.chassis.subscribe_visual_align(ref_area=)` |
| 底盘居中 → 臂对准 → 吸 一条龙 | `VisualOrchestrator().track_and_grasp(label)`（`from main.arm.loops import VisualOrchestrator`；已实现、当前无 task 接线，欢迎第一个吃螃蟹） |
| 读 x 轴实时位置（arm_feed 真值） | `arm_client._read_x_mm_realtime()`（带下划线但事实公开，task4/5 在用） |
| OCR 读订单 / 蔬菜检测冒烟 | `main.misc.test_order_read.run()` / `main.misc.test_veggie_detect.run()` |

**不要这样绕路**（历史 task 里出现过的反模式）：

| 绕路写法 | 换成 |
| --- | --- |
| `arm_client._call_car("move_for", ...)` | `ChassisClient.move_for(dx_m=...)` |
| `client.http.get("/v1/realtime/odom/state")` 裸读里程计 | `main.chassis.get_odometry()` |
| `runner.client.xxx`（穿透 runner 拿底层 client） | 直接持有一个 `ArmClient` 变量 |
| 裸 POST `/v1/realtime/arm-velocity` | `runner.track_velocity(label)` / `track_velocity_pick` |

构造套路（所有已实现 task 的共同开场）：

```python
from main.arm import ArmClient, ArmRunner

arm_client = ArmClient(http=client) if client else ArmClient.connect()
runner = ArmRunner(arm_client)
```

每个已实现 task 具体用了哪些，见 [task/README.md「每个 task 现用的现成方法」](./task/README.md)。

---

## 0. 你应当从哪开始

| 你想做的事 | 看这里 |
| --- | --- |
| **5 分钟跑通"客户端 → runtime"通信** | [QUICKSTART.md](./QUICKSTART.md) |
| **查 HTTP / WS / 客户端方法的权威清单** | [API_INDEX.md](./API_INDEX.md) |
| **写机械臂业务** | [arm/README.md](./arm/README.md) |
| **写底盘外环 / 巡线 / 触发判定** | [chassis/README.md](./chassis/README.md) |
| **写/改比赛 task** | [task/README.md](./task/README.md) |
| **写单文件 mini 任务（射击、边走边打…）** | [misc/README.md](./misc/README.md) |
| **查车端动作注册表（`runtime/core/actions.py`）** | [API_INDEX.md §6](./API_INDEX.md#6-actions-runtimecoreactionspy-全部注册) |
| **查车端 action 内部行为** | [`runtime/services/my_car/`](../runtime/services/my_car/) 包 |

---

## 1. 子包分层

```
main/
├── README.md           ← 你正在看
├── QUICKSTART.md       ← 5 分钟起步
├── API_INDEX.md        ← 全部 HTTP / WS / 客户端方法速查
├── api_client.py       ← RuntimeApiClient：HTTP + job 异步/同步
├── ws_client.py        ← RuntimeWsClient：WS 单点 + 推送订阅
├── settings.py         ← env 解析（RAK_CAR_*）
├── arm/                ← 机械臂业务子包（client/runner/vision/loops/each_task/examples）
├── chassis/            ← 底盘外环子包（client + controllers + loops + tasks + cli + config）
├── task/               ← 8 任务编排注册表（TASK_RUNNERS，见 task/README.md）
├── start/              ← Orchestrator 任务总调度（run.py 入口）
├── test/               ← 离线硬件冒烟脚本（绕过 runtime 直打硬件，非正式测试）
└── misc/               ← 单文件 mini 任务（射击、边走边打、OCR/检测冒烟）
```

| 子包 | 定位 | 一句话入口 |
| --- | --- | --- |
| `main/arm/` | 机械臂业务（client + runner + 软限位 + 复合动作 + 视觉伺服） | `from main.arm import ArmClient, ArmRunner` |
| `main/chassis/` | 底盘外环（client + 50Hz 主循环 + 控制器 + 视觉追踪一键入口） | `from main.chassis import ChassisClient, track_chassis` |
| `main/task/` | 8 任务编号注册表（`TASK_RUNNERS[1..7]`） | `from main.task import TASK_RUNNERS` |
| `main/misc/` | 单文件 mini 任务（射击、边走边打、单发、连发） | `python3 main/misc/single_shot.py` |

---

## 2. 三条红线

1. **业务层不直连车端硬件**——所有调用走 `RuntimeApiClient` 或 `RuntimeWsClient`，连 `runtime/services/my_car/` 的方法都不 import。
2. **`/v1/execute` 默认异步**——立即返回 `job_id`，链式调用必须显式 `sync=True`（arm 业务 API 内部已加）。轮询请用 `wait_job(job_id, timeout)`。
3. **清 `_stop_flag` 必须调 `/v1/control/reset-stop`**——`lane_feed` / `arm_feed` / `task_feed` / `ir_feed` / `odom_feed` 守护线程在 stop 状态下 break 退出，急停恢复后必须重置才能继续。

---

## 3. runtime 默认守护线程（init 后自动启动）

| 守护线程 | 频率 | 缓存字段 | 客户端读法 |
| --- | --- | --- | --- |
| `lane_feed` | 50Hz | `lane_state` | `client.realtime_lane_state()` / WS `subscribe_lane` |
| `arm_feed` | 20Hz | `arm_state` | `client.get_arm_state()` / WS `subscribe_arm_state` |
| `task_feed` | 30Hz | `task_state` | `client.get_task_state()` / WS `subscribe_task_detection` |
| `ir_feed` | 50Hz | `ir_state` | `client.get_ir_state()` / WS `subscribe_ir` |
| `odom_feed` | 50Hz | `odom_state` | `client.get_odom_state()` / WS `subscribe_odom` |

- 全部是**守护线程 + meta_lock**，外环 50Hz+ 轮询安全，不进 job_queue、不打 ZMQ、不抢车锁。
- 共享 `ClintInterface` 内部 `threading.Lock()`（2026-07-31 修复 EFSM 后）。

---

## 4. 比赛流程 8 任务

固定顺序：**seed → scout pests → water → shoot pests → harvest → sort → read order via OCR → deliver**。任务状态、入口与"每个 task 用了哪些现成方法"见 [task/README.md](./task/README.md)。

| 任务 | 关键依赖 | 入口 |
| --- | --- | --- |
| seed（自动播种） | `track_velocity_pick` + `track_chassis` | `main/task/task1_seeding.py` |
| scout pests | `task3/` 三段式流水线（drive + LLM 判别 + 射击） | `main/task/task3_pest_scout.py` |
| water | `track_velocity_pick` + `track_chassis` + `move_for` | `main/task/task2_water_tower.py` |
| shoot pests | task3 流水线内 `car.shooting`（时序红线见 [misc/README.md](./misc/README.md)） | `main/task/task3/` |
| harvest | `each_task/task4/target4.step_target4` | `main/task/task4_harvest.py` |
| sort | `each_task/task5/the_final.main` + `make_align_runner` | `main/task/task5_sort.py` |
| read order | OCR + `composite_run` | `main/task/task6_get_order.py` |
| deliver | （TODO，NotImplementedError） | `main/task/task7_deliver.py` |

编排入口：[main/start/orchestrator.py](./start/orchestrator.py)（50Hz lane_follow + waypoint 列表）。
模板：`run.py` → `main.start.orchestrator.Orchestrator`。