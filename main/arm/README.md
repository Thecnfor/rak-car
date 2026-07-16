# main/arm —— 机械臂组独享子包

> 这份文档只回答一件事：**业务层调机械臂需要知道哪些 API、怎么搭起来**。
> 完整 API 总表见 `main/API.md` 和本目录下 [ARM_API.md](./ARM_API.md)，本文件不再重复。
> 10 行起步见 [QUICKSTART.md](./QUICKSTART.md)。

## 一句话定位

`main/arm/` = **机械臂业务子包**：业务层不需要知道 y 是步进电机 / x 是带编码器直流电机 / 手爪是 PWM 舵机 / 磁感在哪个端口，**只看到 (x_mm, y_mm, side, hand) 4 个字段** + 一组 high-level 动作。

底层不动（`arm_base.py` / `runtime/core/actions.py` / 14 个 `arm.*` action 名字全部保留），上层在这里加：

- `ArmClient`：薄封装 `RuntimeApiClient` / `RuntimeWsClient`
- `ArmState`：业务位姿（mm + 枚举）
- `TrajectoryGenerator`：双轴 S 曲线同步发生器（dry-run）
- `OriginCalibrator`：调车端 `arm.reset_position` 重新触底定原点（写 `arm_origin.yaml`）
- `ArmRunner`：业务编排入口
- `tasks/`：高层组合（go_home / pick_left / pick_right / release）
- `examples/`：4 个起步脚本

## 坐标系约定

> **以 SDK（`smartcar/whalesbot/vehicle/arm/arm_base.py`）实际行为为准。** 业务层、车端、SDK 三层统一用下面这套语义，禁止在调用链上自行取反。

| 项 | 说明 |
| --- | --- |
| `x_mm` | 水平方向位移（mm），相对"撞墙原点"（`x=0` 在撞墙位置，远离墙为正） |
| `y_mm` | 垂直方向位移（mm），**触底 = 0**（y 单边坐标，区间 `[-soft_y_max, 0]`）<br/>`y<0` = 向上（远离磁感触底），`y>0` = 向下（朝触底，安全门会拦） |
| `side` | `LEFT` / `MID` / `RIGHT`（大臂总线舵机） |
| `hand` | `UP` / `MID` / `DOWN`（手爪 PWM 舵机） |
| `grasping` | 真空泵状态（只读，业务层不可设） |
| `storage_side` | `LEFT` / `RIGHT`（车体存储仓独立 PWM 舵机，port=1；写死 -42° / 90°） |

**为什么这么定**：

- SDK 实际行为（用户实测确认）：`move_y(-50mm)` 机械臂向上走 50mm，`move_y(+50mm)` 朝磁感方向走 50mm（被安全门堵住）。
- 业务层 `move_y(y_mm)` 直传 `target=y_mm/1000` 给车端，**不在客户端取反**。
- 业务层注释/文档/软限位检查全部按 `y<0=向上、y>0=向下、y=0=触底、[-soft_y_max, 0]` 写，与 SDK 完全一致。
- 软限位常量名 `soft_y_max_mm` 是「**绝对值上限**」（200mm），不是带符号上限。
  - 业务层 valid range: `0 ≥ y_mm ≥ -soft_y_max_mm`（即 `y_mm ∈ [-200, 0]` mm）。

软限位（首次 calibrate 时手调，写入 `arm_origin.yaml`）：

| 项 | 默认值 | 含义 |
| --- | --- | --- |
| `soft_y_max_mm` | 200 | y 业务上限（实测行程 -200mm 还有富余） |
| `soft_x_min_mm` | None | x 轴软限位已取消（2026-07-16） |
| `soft_x_max_mm` | None | x 轴软限位已取消（2026-07-16） |

## 关键环境变量

`main/arm/` 业务层**只读**这些环境变量；如果改的是启动行为，请去 `ecosystem.config.js`。

| 变量 | 含义 | 默认 |
| --- | --- | --- |
| `RAK_CAR_SERVER_ORIGIN` | runtime HTTP 地址 | `http://127.0.0.1` |
| `RAK_CAR_RESET_ARM` | runtime 启动时是否自动跑一次 `arm.reset_position` 重新定原点 | `0`（部署用 `ecosystem.config.js:23` 设成 `1`） |

> `RAK_CAR_RESET_ARM=1` 时，runtime 会在 auto-init 阶段调一次 `arm.reset_position`（y 触底 + x 撞墙），并把当前编码器值落到 `arm_origin.yaml`。业务层看到 `arm_origin.yaml` 已存在且 `y_origin_m / x_origin_m` 非 0，即可认为首次定原点已完成，**不需要**手跑 `examples/01_calibrate_origin.py`。

## 10 行起步

```python
from main.arm import ArmClient, ArmRunner

client = ArmClient.connect()              # 默认走 env: RAK_CAR_SERVER_ORIGIN
runner = ArmRunner(client)
runner.move_xy(100.0, 80.0)               # 双轴同步
runner.set_side("LEFT")
runner.move_xy(120.0, 40.0)
runner.grasp(True)
```

完整示例在 [examples/](./examples/)：

| 文件 | 用途 |
| --- | --- |
| [examples/01_calibrate_origin.py](./examples/01_calibrate_origin.py) | 触发车端 `arm.reset_position` 重新定原点（漂移后手调用） |
| [examples/02_trajectory_preview.py](./examples/02_trajectory_preview.py) | 不下硬件，只算 S 曲线 |
| [examples/03_move_xy_basic.py](./examples/03_move_xy_basic.py) | 双轴同步移动 + dry-run |
| [examples/04_grasp_template.py](./examples/04_grasp_template.py) | 完整 pick-and-place |

## 机械臂业务 API 子集

| 用途 | 接口 | 推荐方式 | 推荐频率 |
| --- | --- | --- | --- |
| **重新定原点**（漂移后手调用） | `main.arm.OriginCalibrator` | `examples/01_calibrate_origin.py` | 漂移 / 换电池 |
| **主动 reset 原点** | `arm.reset_origin(x_wall)` | `ArmClient.reset_origin(x_wall="left")` | 需要时 |
| **双轴同步移动** | `arm.goto_position` | `ArmClient.move_xy(x_mm, y_mm)` / `ArmRunner.move_xy(...)` | 业务流 |
| **单轴移动** | `arm.move_x_position` / `arm.move_y_position` | `ArmClient.move_x/move_y` | 业务流 |
| **一次设位姿** | `arm.set_arm_pose` | `ArmClient.set_pose(x_mm, y_mm, side, hand)` | 业务流 |
| **大臂方向** | `arm.set_arm_angle` | `ArmClient.set_side("LEFT"/"MID"/"RIGHT")` | 业务流 |
| **手爪角度** | `arm.set_hand_angle` | `ArmClient.set_hand("UP"/"MID"/"DOWN")` | 业务流 |
| **吸盘抓取/释放** | `arm.grasp` | `ArmClient.grasp(True/False)` | 业务流 |
| **读位姿** | `arm.x_get_position` / `arm.y_get_position` / `car.get_arm_state` | `ArmClient.get_state()` | 任意 |
| **读机械臂状态** | `car.get_arm_state` | `ArmClient.get_state()` | 任意 |
| **S 曲线 dry-run** | `main.arm.trajectory.TrajectoryGenerator` | `ArmRunner.move_xy(...)` 内部调用 | 自动 |
| **高层组合** | `main.arm.tasks` | `go_home() / pick_left(x,y) / release(x,y)` | 业务流 |

### 1. `move_xy(x_mm, y_mm, v_max=150, a_max=400)`

- **做什么**：双轴同步移动到 (x_mm, y_mm)。
- **底层**：客户端用 `TrajectoryGenerator` 算 S 曲线 dry-run 给日志 / 超时，硬件调用 `arm.goto_position` 走车端 PID 闭环。
- **超时**：默认 `max(5.0, plan.T * 2.0 + 1.0)` 秒。
- **会拦**：超出 `soft_*` 软限位时**直接抛 `ValueError`**，不下发到硬件。

### 2. `set_pose(x_mm=None, y_mm=None, side=None, hand=None)`

- **任意参数为 `None` 表示不动**（避免 "我只想转 side 结果把 x 推回去了"）。
- **底层**：调 `arm.set_arm_pose`；`None` 会被服务端丢掉。
- **会拦**：超出软限位 → 抛 `ValueError`。

### 3. `reset_origin(x_wall="left")`

- **做什么**：先调 `arm.reset_position`（车端自己 y 触底 + x 撞墙），然后读 `y_get_position` / `x_get_position` 写 `arm_origin.yaml`。
- **什么时候用**：换电池 / 漂移严重 / PID 范围卡死时手调用；**首次上电不需要手动跑**。
- **首次上电**：runtime 启动时若 `RAK_CAR_RESET_ARM=1`（默认配置见 `ecosystem.config.js:23`），会自动跑一次 `arm.reset_position`，并把当前编码器值落盘到 `arm_origin.yaml`。

### 4. `ArmState`

`ArmClient.get_state()` 返回：

```python
ArmState(
    x_mm=0.0, y_mm=0.0,
    side="MID", hand="UP", grasping=False,
    y_origin_valid=False, x_origin_valid=False,
    soft_y_max_mm=200.0, soft_x_min_mm=None, soft_x_max_mm=None,
    raw_x_m=0.0, raw_y_m=0.0,
    arm_angle=None, hand_angle=None,
    fetched_at=1761234567.89,
)
```

> 注意：`ArmState.storage_side` 字段存在但 `ArmClient.get_state()` 当前未填充它，恒为默认值 `"LEFT"`，**不可信**。需要存储仓档位时请直接用 `ArmClient.get_storage()`（客户端缓存）。

`ArmState.in_safe_box(x, y)` 和 `ArmState.is_ready()` 给上层做预校验。

### 5. 客户端控制律（trajectory）

`TrajectoryGenerator` 是**纯 Python** 双轴同步 S 曲线发生器：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `v_max` | 150 mm/s | x 轴最大线速度 |
| `a_max` | 400 mm/s² | 软启动加速度 |
| `j_max` | 2000 mm/s³ | jerk 限制 |

业务层只**读** `plan.T` / `plan.peak_vx` / `plan.peak_vy` 用于日志 / 超时；**不**下发到硬件，硬件仍走车端 PID。

### 6. 安全与兜底

- `_check_safe(...)`：所有写动作（move_xy / move_x / move_y / set_pose）自动校验软限位，**不会**下发越界指令。
- `emergency_stop()`：车端立即停。
- `ping()`：runtime 健康检查。

## 目录结构

```
main/arm/
├── README.md                  ← 你正在看
├── __init__.py
├── ARM_API.md                 ← 业务层机械臂 API 速查
├── QUICKSTART.md              ← 10 行起步
├── api.py                     ← ArmClient：薄封装 main.api_client / main.ws_client
├── state.py                   ← ArmState / ArmOrigin dataclass
├── trajectory.py              ← TrajectoryGenerator：双轴同步 S 曲线（纯 Python）
├── origin.py                  ← OriginCalibrator：调车端 reset_position 重新定原点
├── arm_origin.yaml            ← 业务坐标系软限位 + 标注（gitignore）
├── loops/
│   ├── __init__.py
│   └── runner.py              ← ArmRunner：业务编排 + dry-run
├── tasks/
│   ├── __init__.py
│   ├── go_home.py             ← 回到 y=0, x=0, hand=UP, side=MID
│   ├── pick_left.py           ← 左侧抓取
│   ├── pick_right.py          ← 右侧抓取
│   └── release.py             ← 释放
└── examples/
    ├── __init__.py
    ├── 01_calibrate_origin.py
    ├── 02_trajectory_preview.py
    ├── 03_move_xy_basic.py
    └── 04_grasp_template.py
```

## 三条红线

1. **首次上电通常不需要手跑 `examples/01_calibrate_origin.py`**：runtime 启动时若 `RAK_CAR_RESET_ARM=1`（默认配置见 `ecosystem.config.js:23`）会自动跑一次 `arm.reset_position` 并把新原点落到 `arm_origin.yaml`。只有 `RAK_CAR_RESET_ARM=0` 且从未手调用过 reset 时，`arm_origin.yaml` 才可能不存在 / 全 0，此时坐标系处于**未标定**状态，软限位使用默认值。手动入口只用于漂移严重 / PID 范围卡死后的恢复。
2. **`move_xy` / `move_x` / `move_y` 越界直接抛 `ValueError`**：业务层别 try-except 后硬塞，除非你确认软限位该改了。
3. **业务层只调 `main.arm.*`，不要回退到 `client.call("arm", ...)`**：丢失软限位保护和 S 曲线 dry-run。

## 在哪查 API

- 机械臂专用接口子集：本文档（上面那张表）
- 业务侧便捷 API：[ARM_API.md](./ARM_API.md)
- 10 行起步：[QUICKSTART.md](./QUICKSTART.md)
- 全部接口速查：[main/API.md](../API.md)
- 底层细节：[smartcar/whalesbot/vehicle/arm/arm_base.py](../../smartcar/whalesbot/vehicle/arm/arm_base.py)
- **软件急停 / `reset_y` 找底方向 / 急停 HTTP 端点：[SOFTWARE_ESTOP.md](./SOFTWARE_ESTOP.md)**
