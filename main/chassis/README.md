# main/chassis —— 底盘组独享工作区

> **一句话定位**：`main/chassis/` = 客户端外环（50Hz 控制律 + 安全兜底）全套工具。
> 底盘同学只需要新写 `controllers/*.py` 和组合 `tasks/*.py`；其它（HTTP/WS、lane 误差源、轮速下发、安全门）都在这一层包好了。

---

## 目录

1. [架构与双环数据流](#1-架构与双环数据流)
2. [10 行起步](#2-10-行起步)
3. [`ChassisClient` 全 API 表](#3-chassisclient-全-api-表)
4. [`state.py` 数据契约](#4-statepy-数据契约)
5. [`controllers/` 控制律库](#5-controllers-控制律库)
6. [`loops/` 主循环与安全兜底](#6-loops-主循环与安全兜底)
7. [`tasks/` 高层任务组合](#7-tasks-高层任务组合)
8. [`examples/` 索引](#8-examples-索引)
9. [调参 checklist](#9-调参-checklist)
10. [三条红线（踩坑 FAQ）](#10-三条红线踩坑-faq)
11. [性能基准](#11-性能基准)
12. [目录结构](#12-目录结构)

---

## 1. 架构与双环数据流

```
┌─────────────────── 车端 (Jetson) ───────────────────┐  ┌── 客户端 (任何主机) ──┐
│                                                    │  │                       │
│  cam1 ─► ZMQ port 5001 (lane) ─► start_lane_feed ─┼──┤                       │
│                                       │            │  │                       │
│                                       ▼            │  │   ┌──────────────┐    │
│                                  lane_state  ◄────┼──┼───┤ get_lane_state│    │
│                                       │            │  │   └──────┬───────┘    │
│   调度 / 内环 PID:                    │            │  │          ▼            │
│     car.lane_* / move_* /             │            │  │   ┌──────────────┐    │
│     move_to_detection_target          │            │  │   │  OuterLoop   │    │
│                                       │            │  │   │   .step()    │    │
│   wheels_chassis ◄── set_wheel_speeds ┼────────────┼──┼───┤  → [v1..v4]  │    │
│                                       │            │  │   └──────┬───────┘    │
│                                       │            │  │          ▼            │
│                                       │            │  │   ┌──────────────┐    │
│   odometer_thread ─► odometry / encoders ◄────────┼──┼───┤ set_wheel_sp │    │
│                                                    │  │   └──────────────┘    │
└────────────────────────────────────────────────────┘  └───────────────────────┘

外环：客户端，20-50Hz，控制律只读 lane_state，写 4 轮速
内环：车端，PID（已存在），吃 4 轮速直接驱动
车端 lane_feed：单独线程，只刷 lane_state 不下发轮速（不抢 runtime 锁）
```

**核心原则**：外环和内环 **不能同时跑**。`start_lane_feed` 是"只刷 state"的旁路线程，跟外环下发轮速不冲突；但如果你又开了 `car.lane_*`（车端内环 PID），就会撞 runtime 锁，外环被串行化延迟。

**runtime 锁层次（2026-07 改造）**：realtime 端点（`/v1/realtime/*`）走 `_realtime_gate` 微秒级瞬持锁，长动作（`arm.goto_position` 1-3s PID 闭环）不持任何 runtime 锁，**arm + lane 真正并发**。详见 [`../../runtime/README.md §并发任务模型`](../../runtime/README.md#并发任务模型) / [`../../CLAUDE.md §Runtime concurrency model`](../../CLAUDE.md#runtime-concurrency-model-replaces-car_lock)。

---

## 2. 10 行起步

```python
from main.chassis import ChassisClient, DoubleLoopRunner, POuterLoop

api = ChassisClient.connect()                  # 1. 连 HTTP + WS（自动）
# 注意：lane_feed 守护线程由 runtime init 默认启起来（50Hz，2026-07-16 上调），不用手动 start/stop。
runner = DoubleLoopRunner(
    api=api,
    outer=POuterLoop(vx=0.3),                  # 2. 控制律：P 起步
    hz=50.0,
)
runner.run(max_seconds=15.0)                    # 3. 跑 15 秒（自动 zero out）
api.stop_wheel_speeds()                        # 4. 关轮速（必调）
```

完整示例：`examples/01_minimal_p_lane.py`。

---

## 3. `ChassisClient` 全 API 表

> 所有方法都走 `RuntimeApiClient`（HTTP）或 `RuntimeWsClient`（WS）。
> `set_wheel_speeds` / `set_single_motor` **优先 WS**（实时路径），WS 断了自动回退 HTTP。

| 方法 | 底层 HTTP / WS | 返回 | 量纲 | 推荐频率 |
| --- | --- | --- | --- | --- |
| `connect()` | — | `ChassisClient` | — | 启动一次 |
| `start_lane_feed(hz=50)` | runtime init 默认启（50Hz，2026-07-16 上调），无需手动调用 | `{started, hz}` | Hz | 自动 |
| `stop_lane_feed()` | 已禁用 —— lane_feed 由 runtime 一直跑 | `{stopped}` | — | 不调 |
| `stop_wheel_speeds()` | `realtime/wheel_speeds [0,0,0,0]` | `{speeds: [0,0,0,0]}` | m/s | 退出必调 |
| `emergency_stop()` | `POST /v1/control/emergency-stop` | `{stopped}` | — | 兜底 |
| `get_lane_state()` | `GET /v1/vision/lane/state` | dict | — | ≤50Hz |
| `get_odometry()` | `car.get_odometry` | `(x, y, theta)` float3 | m, m, rad | ≤50Hz |
| `get_wheel_encoders()` | `GET /v1/realtime/wheels/encoders` | list[float] (4) | rad 累计 | ≤50Hz |
| `set_wheel_speeds([v1,v2,v3,v4])` | ws `realtime/wheel_speeds` | `{speeds}` | m/s | ≤50Hz |
| `set_single_motor(port, speed, reverse=1)` | ws `realtime/motor_speed` | `{port, speed, reverse}` | — | ≤50Hz |
| `ping()` | `GET /v1/health` | bool | — | 1Hz |

**量纲约定**（与车端 SDK 一致）：

- 轮速 `m/s`
- 编码器 `rad` 累计（不是位移）
- `error_y` `m`、`error_angle` `rad`
- `odometry` `(x, y, theta)` 世界坐标系，m / m / rad

**错误码**（HTTPException 抛出）：

| 状态码 | 含义 | 处理 |
| --- | --- | --- |
| 400 | 参数错误（如 wheel_speeds 不是 4 个 float） | 检查入参 |
| 409 | car 未初始化 / runtime 卡死 | `GET /v1/health` 看 `state.initialized` |
| 504 | job 超时 | 提高 timeout 或检查 lane_feed |

---

## 4. `state.py` 数据契约

底盘组的控制律 **只接 dataclass，不接 dict**。

### `LaneState`

```python
@dataclass
class LaneState:
    error_y: Optional[float]   # 横向误差（m），左侧为正
    error_angle: Optional[float]  # 角度误差（rad），逆时针为正
    forward: Optional[float]    # 历史 forward_speed（外部 feed 时为 None）
    lateral: Optional[float]    # 历史 lateral_speed（同上）
    angular: Optional[float]    # 历史 angular_speed（同上）
    distance: Optional[float]   # 累计行驶距离（m）
    mode: Optional[str]         # tracking / external_feed / idle / stopped
    age_ms: Optional[float]     # 距离上次更新的毫秒数
```

**属性**：

- `is_fresh`：`age_ms is not None and age_ms < 500.0`（<500ms 视为新鲜）
- `has_error`：`error_y and error_angle 都不为 None`

**来源**：

```python
LaneState.from_lane_state_payload(payload)  # payload 来自 get_lane_state()
```

**控制律使用规范**：

```python
def step(self, state: LaneState, dt: float) -> List[float]:
    if not state.has_error:
        return self._safe_zero()      # 没误差 → 零速，不要硬推
    # ... 用 state.error_y / state.error_angle 算 ...
```

### `OdometryState` / `WheelsState`

只读 dataclass，给上层任务（点到点、航位推算）用，目前 `ChassisClient` 还没暴露它们的 dataclass 形式，按需扩展。

---

## 5. `controllers/` 控制律库

### P / Stanley / Pure Pursuit / 弧度自适应 公式与适用场景

| 控律 | 类 | 公式 | 适用场景 | 调参难度 |
| --- | --- | --- | --- | --- |
| **P** | `POuterLoop` | `vy = -kp_y * error_y`<br/>`omega = -kp_theta * error_angle` | 起步、调试场地、低速直线赛道 | ⭐ 最简单 |
| **Stanley** | `StanleyOuterLoop` | `delta = error_angle + atan(k * error_y / vx)`<br/>`omega = -delta` | 弯道、转向主导的赛道 | ⭐⭐ |
| **Pure Pursuit** | `PurePursuitOuterLoop` | 视觉误差当假想目标点 → 几何曲率 → omega | 占位骨架（需替换为目标轨迹） | ⭐⭐⭐ |
| **弧度自适应** | `CurvatureAdaptiveOuterLoop` | `vx = v_max * exp(-kappa)`<br/>`omega = kp_theta*ea*(1+g*kappa) + k_curv*dkappa`<br/>`axis_mix = sigmoid((kappa - center)/width)`<br/>`vy_decided = (1-axis_mix) * vy_raw`<br/>`omega_decided = axis_mix * omega_raw` | 弧度偏差 / 变化率自适应 + 横移/转向互斥（直线由 `vy` 接管、弯道由 `ω` 接管，`vx` 始终独立） | ⭐⭐⭐ |

**实测典型取值**（调参从这开始）：

| 参数 | 起步值 | 调大 | 调小 |
| --- | --- | --- | --- |
| `vx` | 0.3 m/s | 跑得更快（弯道要多调 P） | 慢速更稳 |
| `kp_y` (P) | 0.4 | 横移更猛（直线偏离修得快） | 减小震荡 |
| `kp_theta` (P) | 1.2 | 转向更猛 | 减小过冲 |
| `k` (Stanley) | 0.6 | 误差大时转向更猛 | 减少摇摆 |
| `look_ahead_m` (PP) | 0.6 m | 提前减速（弯道更平滑） | 提前切入（直线更稳） |
| `r_eff` | 0.30 m | 一般不改 | 一般不改 |

**弧度自适应（`CurvatureAdaptiveOuterLoop`）起步值**：

| 参数 | 起步值 | 含义 |
| --- | --- | --- |
| `v_max` / `v_min` | 0.30 / 0.08 | 标称 / 弯道最慢前向速度 |
| `kappa_full` / `dkappa_full` | 0.6 / 1.5 | 弧度偏差 / 变化率满量程（用于归一化） |
| `kp_y` / `kp_theta` | 0.5 / 1.2 | 横向 / 转向基础 P 项 |
| `omega_gain` / `k_curvature` | 0.35 / 0.25 | 弧度大时的 omega 增益 / 变化率牵引 |
| `omega_cap` | 1.5 rad/s（example 04 显式保守值；类默认 1.8） | omega 软上限；超过即截断，防下位机掉电压 |
| `ema_alpha` | 0.35 | curvature 估计的 EMA 平滑系数 |
| `ey_release` / `ea_release` | 0.02 / 0.05 | 恢复门控误差阈值（m / rad） |
| `hold_ms` | 250 ms | 误差小并稳定多久才放回 `v_max` |
| `kappa_axis_center` / `kappa_axis_width` | 1.0 / 0.5 | `axis_mix` sigmoid 分水岭 / 过渡带宽度。<br/>`axis_mix ≈ 0` → 由 `vy` 接管（朝向不变、质心斜向滑动）；<br/>`axis_mix ≈ 1` → 由 `ω` 接管（后轮做轴、前轮差速旋转）。<br/>现场调：<ul><li>直线上小抖就触发到 ω → 调高 `kappa_axis_width` 到 0.7+</li><li>进弯后才打到 ω 全额 → 调低 `kappa_axis_center` 到 0.7</li></ul> |
| `r_eff` | 0.30 m | 麦轮几何系数 |

**`axis_mix` 是什么**：来自「轴向互斥」设计——直线段由 `vy` 修横向偏差（`ω=0`、
朝向不变、质心在世界坐标系沿斜向移动）；弯道段由 `ω` 转向（`vy=0`、后轮
做轴前轮差速旋转）。过渡是用 `kappa`（已存在的弧度偏差 + 变化率归一化）
通过 sigmoid 平滑映射到 `[0, 1]` 实现的，避免同一帧内横向修正与转向全开互冲。
`debug_snapshot()` 已暴露 `axis_mix` 键，可直接 `print(outer.debug_snapshot())`
观察分布。

**r_eff 是什么**：麦轮几何里"角速度 → 4 轮异速"的耦合系数。等于 `(track/2 + wheel_base/2)`，从 `cfg_vehicle.yaml` 算出来是 `(0.30/2 + 0.28/2) = 0.29`，代码里写 0.30 是凑整。

### `WheelSmoother`：下发前最后一道闸（防掉电压）

麦轮逆解 `[v1..v4] = vx ± vy ± r*omega` 在大弧度差急转弯瞬间，单轮目标能从
`0.30 m/s` 直接跳到 `1.0+ m/s`（50Hz 外环下相当于 ~35 m/s² 阶跃），下位机
电源扛不住 → 掉电压。`WheelSmoother` 对每轮独立做 (a) `|v| ≤ max_abs`
(b) 单帧 `Δv ∈ [-max_decel, +max_accel]`，挂在 runner / `subscribe_lane_state`
入口作为最后一道闸。

```python
from main.chassis import WheelSmoother, DoubleLoopRunner

smoother = WheelSmoother(
    max_abs=0.55,    # 单轮 |v| 上限 (m/s)
    max_accel=0.4,   # 单帧最大加速量（50Hz → 20 m/s²）
    max_decel=0.6,   # 单帧最大减速量（急停 / 丢线更快响应）
)
runner = DoubleLoopRunner(api=api, outer=outer, hz=50.0, smoother=smoother)
```

`DoubleLoopRunner` 默认会自己 new 一个 `WheelSmoother()`；要彻底关掉就显式传
一个 `max_abs=math.inf / max_accel=math.inf` 的实例。

### 新增一个控制律

继承 `controllers/base.py:OuterLoop`，实现一个 `step()`：

```python
from typing import List
from main.chassis.controllers.base import OuterLoop
from main.chassis.state import LaneState

class MyOuterLoop(OuterLoop):
    def __init__(self, ...):
        ...

    def step(self, state: LaneState, dt: float) -> List[float]:
        if not state.has_error:
            return self._safe_zero()
        # 你的控制律
        vx, vy, omega = ...
        # 用 base.mecanum_inverse(vx, vy, omega, r_eff) 反算 4 轮速
        return mecanum_inverse(vx, vy, omega, r_eff=0.30)
```

放在 `controllers/my_controller.py`，加到 `__init__.py` 的 `__all__`。

---

## 6. `loops/` 主循环与安全兜底

### `DoubleLoopRunner`（外环主循环）

```python
DoubleLoopRunner(
    api=api,
    outer=POuterLoop(vx=0.3),
    hz=50.0,                  # 闭环频率
    watchdog_ms=500.0,        # lane_state 太久没刷就急停
    on_tick=lambda state, speeds: print(state.error_y),  # 可选回调
)
runner.run(max_seconds=15.0)
```

**调度**：

```
now = monotonic
while not self._stop and now < deadline:
    state = self._sense()                            # 拉 lane_state
    if watchdog.should_stop(state) or lost.should_alert(state):
        emergency_stop()
        break
    speeds = outer.step(state, dt)                   # 控制律
    api.set_wheel_speeds(speeds)                     # 下发
    on_tick(state, speeds)                           # 回调（try/except 兜住）
    next_tick += dt
    sleep if 落后 < 0.5*dt else 放弃补偿避免 catching up
finally:
    api.set_wheel_speeds([0, 0, 0, 0])               # 自动 zero out
```

**关键不变量**：

- 任何异常路径都走 `finally` → 零速（不会留轮速在那空转）
- 调度落后超过 `dt/2` 就放弃补偿，不 catch up（避免突发阻塞后追帧）
- `emergency_stop` 触发后立刻退出，不会再下发轮速

### `EmergencyWatchdog(threshold_ms=500)`

```python
def should_stop(self, state):
    return state.age_ms is not None and state.age_ms > threshold_ms
```

**触发场景**：车端 `lane_feed` 卡死、lane 推理超时、runtime 重启。500ms 是经验值，外环 50Hz × 2 = 100ms 一帧，500ms 已经 5 帧没数据，肯定有问题。

### `LostLineDetector(stable_ms=300, zero_eps=1e-3)`

```python
def should_alert(self, state):
    if error is None: return False
    near_zero = |error_y| < 1e-3 and |error_angle| < 1e-3
    return 已经齐 0 持续 stable_ms
```

**触发场景**：车真到了终点（误差都 ≈ 0）、或者丢线（推理炸了返回全 0）。300ms 是经验值，太短会误报（正常过直线段也接近 0）。

---

## 7. `tasks/` 高层任务组合

| 任务 | 用途 | 走哪个环 |
| --- | --- | --- |
| `follow_lane(api, outer, dis_hold=1.5, timeout_s=30)` | 起 lane feed + 外环跑 N 秒 | **外环** |
| `track_target(api, label=None, delta_x=0, delta_y=None, time_out=3)` | 让位给车端 `car.move_to_detection_target` 微调终点 | **内环** |
| `back_to_line(api, straight_seconds=0.6, vx=0.2)` | 丢线时直行恢复（占位骨架） | **外环** |

### 任务编排模板

```python
from main.chassis import ChassisClient, DoubleLoopRunner, StanleyOuterLoop
from main.chassis.tasks import follow_lane, track_target

api = ChassisClient.connect()

# Phase 1：外环巡线（沿主车道走）
follow_lane(api, outer=StanleyOuterLoop(vx=0.3), timeout_s=10.0)

# Phase 2：让位给内环（车端 PID 做视觉终点微调）
api.stop_wheel_speeds()
track_target(api, label=None, time_out=3.0)
# 注意：不需要 api.stop_lane_feed() —— lane_feed 由 runtime 一直跑
```

### `back_to_line` 注意事项

- 当前是直行兜底，**底盘组按真实"丢线恢复"策略替换**（比如倒车、转 90° 找线）
- 默认 `straight_seconds=0.6s`，按麦轮 0.2m/s 估算走过 12cm，按场地调

---

## 8. `examples/` 索引

| 文件 | 场景 | 频率 | 控制律 |
| --- | --- | --- | --- |
| `01_minimal_p_lane.py` | 起步、直线赛道、低速 | 50Hz | `POuterLoop` |
| `02_stanley_lane.py` | 弯道、中速 | 50Hz | `StanleyOuterLoop` |
| `03_p2p_with_vision.py` | 巡线 → 视觉终点微调（外环+内环切换） | 50Hz | `StanleyOuterLoop` + `track_target` |
| `04_curvature_adaptive.py` | 弧度偏差自适应巡线（弯道降速 + 加强转向） | 50Hz | `CurvatureAdaptiveOuterLoop` + `WheelSmoother` |

**用法**：

```bash
# 默认参数（vx=0.3, 15/20/5 秒）
python3 main/chassis/examples/01_minimal_p_lane.py

# 自定义
python3 -c "from main.chassis.examples import 01_minimal_p_lane; 01_minimal_p_lane.main(max_seconds=30, vx=0.4)"
```

---

## 9. 调参 checklist

按这个顺序调，最稳：

1. **`vx`**：先固定 `vx=0.3`，确认场地能直走（无 lane 误差时稳态 ≈0）
2. **`kp_theta`**：先只调转向，看弯道能不能跟（`Stanley` 或 P 的转向项）
3. **`kp_y`**：再加横向修正（误差大时横移修正）
4. **`watchdog_ms`**：如果 lane 推理慢，调大到 1000-2000ms
5. **`hz`**：外环上限受 lane_feed 推理速度限制，**实测 50Hz 跑不通就降到 30Hz**，别硬撑
6. **`r_eff`**：换车体（轮距/轴距变）才需要改
7. **`WheelSmoother`**：如果弯道掉电压 / 4 轮跳变严重，先把 `max_accel`
   收到 0.2~0.3 m/s/frame（=10~15 m/s²）；直线跟线稳后再往 0.4 放开
8. **`kappa_axis_center` / `kappa_axis_width`**：弧度自适应轴向互斥阈值。
   跑前先看 `debug_snapshot()["axis_mix"]`：
   - 直线上稳定接近 0 → 表示由 `vy` 接管修正，正常
   - 入弯瞬间平滑爬到 0.7-1.0 → 表示 ω 接管，正常
   - 直线上小幅震荡就跳到 0.3+ → 把 `kappa_axis_width` 调到 0.7+
   - 进弯一段时间 axis_mix 仍 < 0.5 → 把 `kappa_axis_center` 调到 0.7

**调参时一定要打印这几个量**：

```python
DoubleLoopRunner(
    api=api, outer=...,
    on_tick=lambda state, speeds: print(
        f"ey={state.error_y:+.4f} ea={state.error_angle:+.4f} "
        f"age={state.age_ms:.0f}ms speeds={speeds}"
    ),
)
```

---

## 10. 三条红线（踩坑 FAQ）

### 🔴 1. `start_lane_feed` 跑起来后，**不要**再调 `car.lane_*`

- **症状**：外环轮速下发被串行化，50Hz 变成 5-10Hz，车一抖一抖
- **原因**：`car.lane_*` 走 `car_lock`，外环也走 `car_lock`（虽然走的是 `realtime/wheel_speeds` 路径），相互等锁
- **检查**：调试时打印 `api.ws_ready` 和 `/v1/health.components.controller.ready`

### 🔴 2. **不要**用 `POST /v1/execute` 下发轮速

- **症状**：50Hz 走 job_queue 排队变成 5Hz
- **正确**：
  - WS：`op: realtime/wheel_speeds` 或 `op: realtime/chassis_velocity`
  - HTTP：`POST /v1/realtime/wheels/speeds` 或 `POST /v1/realtime/chassis-velocity`
- **代码层强制**：`ChassisClient.set_wheel_speeds` 内部已经只走这两个入口，业务层不会误用

### 🔴 3. **任何脚本入口都要 `try/finally: api.stop_wheel_speeds()`**

- **症状**：Ctrl-C 后车一直以最后那个轮速冲出去
- **`DoubleLoopRunner.run` 已经做了**，手写循环请照抄：

```python
# 注意：lane_feed 由 runtime init 默认启起来，不需要 start。
# 这里只需要确保 Ctrl-C 退出后把轮速归零，避免车冲出去。
try:
    while True:
        ...
finally:
    api.stop_wheel_speeds()    # ← 必加
    # 不要 api.stop_lane_feed() —— 它一直跑
```

---

## 11. 性能基准

| 项 | 实测值 | 备注 |
| --- | --- | --- |
| 外环频率 | 50Hz | 受 lane 推理 ~5-10ms + HTTP/WS RTT ~2-5ms 限制 |
| `lane_feed` 推送频率 | 守护线程 `hz=50`（2026-07-16 上调）；实测写入 ~15-20Hz | 受 ZMQ 推理 + cv2.imencode 限制。lane 模型 128x128 推理 ~20-30ms + ZMQ RTT，单循环跑超过 20ms 周期，实际写入频率是推理耗时倒数。**外环 50Hz 轮询 cache 仍能拿到每帧新值**（无重复帧），但 feed 写入频率上限约 20Hz |
| 单轮端到端 RTT（WS） | ~5ms | ws 路径比 http 快 ~3-5ms |
| 单轮端到端 RTT（HTTP） | ~10ms | 仍能跑 50Hz，但留余量小 |
| WebSocket push 5s | ~250 次 | `subscribe_lane` 实测 50Hz×5s（受 lane_feed 频率上限约束） |
| lane_outer_loop.py 3.3s | 51 push + 51 下发 | 15.5 Hz 端到端 |

---

## 12. 目录结构

```
main/chassis/
├── README.md                 ← 你正在看
├── __init__.py               ← 只导出 public API
├── api.py                    ← ChassisClient：薄封装 RuntimeApiClient/WS
├── state.py                  ← LaneState / OdometryState / WheelsState
├── loops/
│   ├── closed_loop.py        ← DoubleLoopRunner：50Hz 外环主循环
│   └── safety.py             ← EmergencyWatchdog / LostLineDetector
├── controllers/
│   ├── base.py               ← OuterLoop ABC + WheelSmoother + mecanum_inverse helper
│   ├── p_controller.py       ← POuterLoop
│   ├── stanley.py            ← StanleyOuterLoop
│   ├── pure_pursuit.py       ← PurePursuitOuterLoop（占位骨架）
│   └── curvature_adaptive.py ← CurvatureAdaptiveOuterLoop（弧度偏差自适应）
├── tasks/                    ← 高层组合（外环 + 内环事件）
│   ├── follow_lane.py        ← 起 lane feed + 外环跑 N 秒
│   ├── track_target.py       ← car.move_to_detection_target 包装
│   └── back_to_line.py       ← 丢线恢复（直走 straight_seconds）
└── examples/                 ← 4 个起步脚本
    ├── 01_minimal_p_lane.py
    ├── 02_stanley_lane.py
    ├── 03_p2p_with_vision.py
    └── 04_curvature_adaptive.py
```

---

## 在哪查 API

- 底盘专用接口子集：本文档
- 全部接口速查：[main/API_REFERENCE.md](../API_REFERENCE.md)
- 完整能力清单：[main/CAPABILITY_LIST.md](../CAPABILITY_LIST.md)
- Runtime 服务端 lane_state / WS push：[runtime/VISION_API.md](../../runtime/VISION_API.md)
- 出问题了看：[debug-controller-download-stuck.md](../../debug-controller-download-stuck.md)、[debug-runtime-init-queue.md](../../debug-runtime-init-queue.md)