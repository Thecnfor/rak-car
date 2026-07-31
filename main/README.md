# main/ —— 业务客户端（依赖 runtime HTTP / WS）

> **`main/` 永远不会 import `runtime/services/my_car.py`**，所有调用走 HTTP / WS。
> 这条边界是为了让**业务层**和**车端 runtime** 可以独立部署、独立热更、独立调试。

---

## 0. 你应当从哪开始

| 你想做的事 | 看这里 |
| --- | --- |
| **5 分钟跑通"客户端 → runtime"通信** | [QUICKSTART.md](./QUICKSTART.md) |
| **查 HTTP / WS / 客户端方法的权威清单** | [API_INDEX.md](./API_INDEX.md) |
| **写机械臂业务** | [arm/README.md](./arm/README.md) + [arm/QUICKSTART.md](./arm/QUICKSTART.md) |
| **写底盘外环 / 巡线 / 触发判定** | [chassis/README.md](./chassis/README.md) |
| **写单文件 mini 任务（射击、边走边打…）** | [misc/README.md](./misc/README.md) |
| **查车端动作注册表（`runtime/core/actions.py`）** | [API_INDEX.md §6](./API_INDEX.md#6-actions-runtimecoreactionspy-全部注册) |
| **查车端 action 内部行为** | [`runtime/services/my_car.py`](../runtime/services/my_car.py) |

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
├── arm/                ← 机械臂业务子包（client/runner/state/origin/trajectory/tasks/examples）
├── chassis/            ← 底盘外环子包（client + controllers + loops + tasks + cli + config）
└── misc/               ← 单文件 mini 任务 + 调研笔记
```

| 子包 | 定位 | 一句话入口 |
| --- | --- | --- |
| `main/arm/` | 机械臂业务（client + runner + 软限位 + 复合动作 + S 曲线） | `from main.arm import ArmClient, ArmRunner` |
| `main/chassis/` | 底盘外环（client + 50Hz 主循环 + 控制器 + IR 触发判定） | `from main.chassis import ChassisClient, run_lane_follow` |
| `main/misc/` | 单文件 mini 任务（射击、边走边打、单发、连发）+ 笔记 | `python3 main/misc/single_shot.py` |

---

## 2. 三条红线

1. **业务层不直连车端硬件**——所有调用走 `RuntimeApiClient` 或 `RuntimeWsClient`，连 `my_car.py` 的方法都不 import。
2. **`/v1/execute` 默认异步**——立即返回 `job_id`，链式调用必须显式 `sync=True`（arm 业务 API 内部已加）。轮询请用 `wait_job(job_id, timeout)`。
3. **清 `_stop_flag` 必须调 `/v1/control/reset-stop`**——`lane_feed` / `arm_feed` / `task_feed` / `ir_feed` / `odom_feed` 守护线程在 stop 状态下 break 退出，急停恢复后必须重置才能继续。

---

## 3. runtime 默认守护线程（init 后自动启动）

| 守护线程 | 频率 | 缓存字段 | 客户端读法 |
| --- | --- | --- | --- |
| `lane_feed` | 50Hz | `lane_state` | `client.realtime_lane_state()` / WS `subscribe_lane` |
| `arm_feed` | 20Hz | `arm_state` | `client.get_arm_state()` / WS `subscribe_arm_state` |
| `task_feed` | 10Hz | `task_state` | `client.get_task_state()` / WS `subscribe_task_detection` |
| `ir_feed` | 50Hz | `ir_state` | `client.get_ir_state()` / WS `subscribe_ir` |
| `odom_feed` | 50Hz | `odom_state` | `client.get_odom_state()` / WS `subscribe_odom` |

- 全部是**守护线程 + meta_lock**，外环 50Hz+ 轮询安全，不进 job_queue、不打 ZMQ、不抢车锁。
- 共享 `ClintInterface` 内部 `threading.Lock()`（2026-07-31 修复 EFSM 后）。

---

## 4. 比赛流程 8 任务

固定顺序：**seed → scout pests → water → shoot pests → harvest → sort → read order via OCR → deliver**。

| 任务 | 关键依赖 | 入口 |
| --- | --- | --- |
| seed（自动播种） | `car.move_to_detection_target` + `arm.move_xy` | `main/start/orchestrator.py` |
| scout pests | `task_feed` 边走边看 + 视觉判定 | 同上 |
| water | `car.set_storage` + 距离触发 | 同上 |
| shoot pests | `car.shooting` × N（每次 500ms 间隔 ≥5s） | [`main/misc/shooting_logic.md`](./misc/shooting_logic.md) |
| harvest | `arm.composite_pick` | [arm/tasks/](./arm/tasks/) |
| sort | `arm.composite_release` | 同上 |
| read order | `POST /v1/vision/ocr` label=`order` / `name` | [API_INDEX.md §2](./API_INDEX.md#2-visioncam1-车道--cam2-侧摄) |
| deliver | `car.move_to_position` + 终点对齐 | 同 seed |

编排入口：[main/start/orchestrator.py](./start/orchestrator.py)（50Hz lane_follow + waypoint 列表）。
模板：`run.py` → `main.start.orchestrator.Orchestrator`。