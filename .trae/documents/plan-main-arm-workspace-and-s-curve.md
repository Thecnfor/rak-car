# main/arm 子包 + 机械臂 S 曲线同步运动基础建设

## Summary

给机械臂在 `main/` 下建一个独立工作区 `main/arm/`，照 `main/chassis/` 的形态做（client / state / controllers / loops / tasks / examples / README）。底层不改（沿用现有 `arm.*` action 名字），上层加：

1. **手动硬定原点工具**：用车上 4 键连续按住（1=y↓、3=y↑、2=x←、4=x→），把 y 触底 = 0、x 撞一侧墙 = 0，写入新 `arm_origin.yaml`。
2. **x 撞墙检测**：x 是带编码器直流电机、无机械限位，通过"编码器一段时间无变化 + 推力持续"判定堵转，**主动撞一侧墙**（按用户决策）。
3. **梯形/S 曲线同步轨迹发生器**（基础设施）：给 `move_xy(x, y, v_max, a_max)` 算出 `(t, x_t, y_t, vx_t, vy_t)` 时间序列。给上层业务调，但默认走车端既有 PID `arm.goto_position`。
4. **ArmClient / ArmState / ArmRunner**：薄封装业务用得上的 high-level 动作 + 状态读取。
5. **README + 快速上手 + 文档**：`main/arm/README.md` + `main/arm/ARM_API.md` + `main/arm/QUICKSTART.md`。

不动现有 `arm.*` 名字（`arm.set_arm_pose` / `arm.move_y_position` / `arm.move_x_position` / `arm.goto_position` / `arm.go_for` / `arm.set_arm_angle` / `arm.set_hand_angle` / `arm.grasp` / `arm.x_get_position` / `arm.y_get_position` / `arm.reset_position` / `arm.reset_x`）。

## Current State Analysis

### 1. 机械臂底层（不动的部分）

- `smartcar/whalesbot/vehicle/arm/arm_base.py` `ArmController` 已经把 y 步进 / x 带编码器 / 手爪 PWM / 大臂总线舵机 / 气泵 / 限位 6 全部串起来。
- `runtime/core/actions.py` `ARM_ACTIONS` 注册了 14 个 `arm.*` action，业务层调它就行。
- 关键代码现状：
  - `reset_y` 用 `y_limit_sensor.read() > 1000` 触底归零，**y 单边坐标 (≥0)**
  - `reset_x` 用 `x_stop_check()`（编码器无变化连续 N 次）堵转归零，再靠 yml 的 `pose_horiz` 二次纠正
  - `goto_position` 是两轴独立 PID 拼起来，曲线是 L 形折线，没有同步轨迹
  - `x_speed` / `y_speed` 是开环速度下发，PID 是位置环
  - `set_manually()` 4 键是 `Key4Btn(4)`，已有：`1=上, 3=下, 4=右, 2=左`

### 2. main 现状

- `main/chassis/` 已经是"组别子包"模板：README、api.py (ChassisClient)、state.py、loops/closed_loop.py、loops/safety.py、controllers/、tasks/、examples/。
- `main/arm/` 不存在。
- 业务层与底层之间只通过 `RuntimeApiClient` / `RuntimeWsClient`，约束保留。

### 3. 用户已经拍板的设计

| 决策点 | 用户选择 |
| --- | --- |
| 原点标定方式 | **4 键手动**到 y 触底、x 撞一侧墙 |
| x 编码器 | 有编码器但无机械限位；**主动撞一侧墙**编码器读起点 |
| y 坐标原点 | y **触底 = 0**（保险，实际不触底） |
| 软限位 | **要的**，首次要靠手动调 |
| 轨迹 | **S 曲线同步**双轴，给 API 准备好基础设施 |
| main 层 API | **保持原名**（`arm.set_arm_pose` 等都不动） |
| 文档 | main 下机械臂**单独工作区** + **所有 API 文档** + **快速上手** |

## Proposed Changes

### 4. 新增目录与文件

```
main/
└── arm/
    ├── README.md                    # 总览（入口文档、类比 chassis/README.md）
    ├── __init__.py                  # 导出 ArmClient / ArmState / 关键 API
    ├── api.py                       # ArmClient：薄封装 RuntimeApiClient + RuntimeWsClient
    ├── state.py                     # ArmState dataclass：{x, y, side, hand, y_origin_valid, x_origin_valid, ...}
    ├── origin.py                    # OriginCalibrator：4 键手动硬定原点，写 arm_origin.yaml
    ├── trajectory.py                # TrajectoryGenerator：梯形 / S 曲线发生器（纯 Python，无硬件依赖）
    ├── loops/
    │   ├── __init__.py
    │   └── runner.py                # ArmRunner：把业务侧 high-level 动作（move_xy/grasp 等）包成同步接口
    ├── tasks/
    │   ├── __init__.py
    │   ├── go_home.py               # 回到 y=0, x=0
    │   ├── pick_left.py             # 左侧抓取：set_side(LEFT) + move_xy + grasp
    │   ├── pick_right.py            # 右侧抓取
    │   └── release.py               # 释放
    ├── examples/
    │   ├── __init__.py
    │   ├── 01_calibrate_origin.py   # 4 键手动定原点
    │   ├── 02_trajectory_preview.py # 不下硬件，只算 S 曲线
    │   ├── 03_move_xy_basic.py      # 同步双轴运动
    │   └── 04_grasp_template.py     # set_side + move_xy + grasp 模板
    ├── ARM_API.md                   # 业务层机械臂 API 速查（与 main/API.md 同体例）
    ├── QUICKSTART.md                # 10 行起步
    └── arm_origin.yaml              # 运行时持久化的原点配置（gitignore）
```

### 5. 文件级设计

#### 5.1 `main/arm/state.py`（dataclass）

```python
@dataclass
class ArmState:
    # 业务看到的位姿（相对原点，单位 mm）
    x_mm: float
    y_mm: float
    side: str            # "LEFT" | "MID" | "RIGHT"
    hand: str            # "UP"   | "MID" | "DOWN"
    grasping: bool
    # 坐标系可信度
    y_origin_valid: bool # False = y_pose 不可信（触底后自动恢复为 True）
    x_origin_valid: bool
    # 配置
    soft_y_max_mm: float # 软上限，0.2m = 200mm
    soft_x_min_mm: float
    soft_x_max_mm: float
    # 原始坐标（车端读数，调试用）
    raw_x_m: float
    raw_y_m: float
    # 时间戳
    fetched_at: float
```

业务层只读 `x_mm` / `y_mm` / `side` / `hand` / `y_origin_valid` / `x_origin_valid` / `soft_*`。

#### 5.2 `main/arm/api.py`（ArmClient）

薄封装 `RuntimeApiClient`，类比 `chassis/api.py:ChassisClient`：

```python
class ArmClient:
    def __init__(self, http: RuntimeApiClient, ws: Optional[RuntimeWsClient] = None): ...
    @classmethod
    def connect(cls) -> "ArmClient": ...

    # 业务动作
    def set_pose(self, x_mm=None, y_mm=None, side=None, hand=None) -> dict
    def move_xy(self, x_mm, y_mm, v_max_mms=150, a_max_mms2=400) -> dict
    def move_y(self, y_mm, v_max_mms=80) -> dict
    def move_x(self, x_mm, v_max_mms=150) -> dict
    def set_side(self, side: str) -> dict           # LEFT/MID/RIGHT
    def set_hand(self, hand: str) -> dict           # UP/MID/DOWN
    def grasp(self, on: bool) -> dict
    def reset_origin(self, x_wall: str = "left") -> dict   # x_wall = "left" | "right"
    def reset_x(self) -> dict
    def reset_y(self) -> dict

    # 状态读取
    def get_state(self) -> ArmState
    def get_pose_mm(self) -> Tuple[float, float, str, str]   # (x_mm, y_mm, side, hand)
    def get_x_mm(self) -> float
    def get_y_mm(self) -> float

    # 安全
    def emergency_stop(self) -> dict
    def ping(self) -> bool
```

**关键点**：

- `move_xy(...)` / `move_y` / `move_x` **底层都是调 `arm.goto_position` / `arm.move_y_position` / `arm.move_x_position`**（保留车端 PID + 现有逻辑），但**额外**通过 `trajectory.TrajectoryGenerator` 在客户端算 S 曲线监督：
  - 启动前 1 次 dry-run，预测 `t_total`
  - 不发到硬件
  - 给业务层记录/日志/超时
- `reset_origin(x_wall="left")` 走 `arm.reset_y` + `arm.reset_x` + 撞墙补偿
  - y 触底 = 0（`y_pose_start = motor_y.get_dis()`，现有逻辑）
  - x 撞左墙：先以慢速往左走 `x = -0.05`，等 `x_stop_check()` 触发（编码器 N 个 tick 不动），记录 `x_pose_start = motor_x.get_dis()`。这样**未来 x 坐标起点就是"撞左墙时的编码器值"**，由车端自己换算。
  - x 撞右墙同理往右走
  - **不主动往两侧都撞**，每次 `reset_origin` 只撞用户指定的那一侧
- `set_pose(...)` 走 `arm.set_arm_pose(x, y, arm, hand)`，任意参数为 `None` 保持当前值

#### 5.3 `main/arm/origin.py`（OriginCalibrator）

**这是 4 键手动定原点的工具**，独立于 `ArmClient` 的正常动作流程。

```python
class OriginCalibrator:
    def __init__(self, http: RuntimeApiClient): ...
    def run(self, x_wall: str = "left", ymax_mm: float = 180, xmin_mm: float = 10, xmax_mm: float = 300):
        """
        1. 提示用户按 4 键连续按住手动移动机械臂到目标位置
        2. 长按 (1+3) 同时 1s = 确认保存原点
        3. 落盘到 arm_origin.yaml
        """
```

**按键映射**（与现有 `ArmController.set_manually` 一致）：

| 键 | 行为 | 速度 |
| --- | --- | --- |
| 1 | y 下降 | 0.1 m/s（现有） |
| 3 | y 上升 | 0.1 m/s（现有） |
| 2 | x 左移 | 0.1 m/s（现有） |
| 4 | x 右移 | 0.1 m/s（现有） |
| 1+3 同时按 1s | 触发 `reset_origin`，落盘 `arm_origin.yaml` | — |
| 1+3 同时按 3s | 退出程序 | — |

**为什么用现有 `set_manually` 风格而不是新写**：现有 `set_manually` 是 `arm_base.py` 已经跑过的、4 键连续按住；新写反而引入第二套按键约定。

**落盘格式**（`arm_origin.yaml`，放在 `main/arm/arm_origin.yaml`，gitignore）：

```yaml
# 由 main/arm/origin.py::OriginCalibrator 写入
# 不要手动改；改完请重新跑 examples/01_calibrate_origin.py
y_origin_m: 0.1234     # y 触底时的 motor_y.get_dis() 原始值
x_origin_m: -0.5678    # x 撞墙时的 motor_x.get_dis() 原始值
x_wall: left            # 上次撞的是哪一侧
soft_y_max_m: 0.18     # 业务软上限
soft_x_min_m: 0.005
soft_x_max_m: 0.30
calibrated_at: 2026-07-14T10:00:00
```

> 注意：底层 `ArmController` 已经把 `y_pose_start` / `x_pose_start` 存在 `arm_cfg.yaml:pos_cfg`，新加的 `arm_origin.yaml` 是**业务层坐标系软限位 + 标注**，不替代 `arm_cfg.yaml:pos_cfg`。两边职责分开：
> - `arm_cfg.yaml:pos_cfg` = 车端硬件原点
> - `main/arm/arm_origin.yaml` = 业务软限位

#### 5.4 `main/arm/trajectory.py`（S 曲线发生器）

**纯 Python，不依赖硬件**，给上层做 dry-run、超时估计、日志。

```python
@dataclass
class TrajectorySample:
    t_s: float
    x_mm: float
    y_mm: float
    vx_mm_s: float
    vy_mm_s: float

class TrajectoryGenerator:
    """
    双轴梯形 + S 曲线同步发生器。

    算法（按用户选 S 曲线同步）：
    1. 先按梯形算出公共 T（两轴分别算 T_x、T_y，取 max）
    2. 在 T 内做 jerk-limited 7 段 S 曲线（加加速 → 匀加速 → 减加速 → 匀速 → 加减速 → 匀减速 → 减减速）
    3. 两轴共享同一时间轴 t，但各自算 s(t)；这样 S 曲线 + 双轴同步同时满足
    """
    def plan_xy(self, x0, y0, x1, y1, v_max, a_max, j_max) -> TrajectoryPlan: ...
    def sample(self, plan, t_s) -> TrajectorySample: ...
    def total_time(self, plan) -> float: ...
```

**关键参数**：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `v_max` | 150 mm/s | x 轴最大线速度（从 yml `pid.output_limits` 0.2 m/s 推算） |
| `a_max` | 400 mm/s² | 软启动加速度 |
| `j_max` | 2000 mm/s³ | jerk 限制，避免启停冲击 |

> 这是**给业务层 dry-run / 日志用**，不是下发到硬件。硬件还是车端 PID。

#### 5.5 `main/arm/loops/runner.py`（ArmRunner）

类比 `chassis/loops/closed_loop.py:DoubleLoopRunner`，把"业务动作 + 业务状态"包成同步调用。

```python
class ArmRunner:
    def __init__(self, client: ArmClient, timeout_s: float = 60.0): ...
    def move_xy(self, x_mm, y_mm, v_max_mms=150, a_max_mms2=400) -> dict:
        plan = self.client.traj.plan_xy(...)  # dry-run
        log.info(f"move_xy: t={plan.T:.2f}s, peak_v={plan.peak_v:.1f}mm/s")
        return self.client.move_xy(...)        # 真发到车端
    def grasp(self, on): ...
    def pick(self, side, x_mm, y_mm): ...
```

#### 5.6 `main/arm/tasks/`（高层组合）

- `go_home.py`: `move_xy(0, 0)` + `set_hand("UP")` + `set_side("MID")`
- `pick_left.py`: `set_side("LEFT")` → `move_xy(x, y)` → `grasp(True)`
- `pick_right.py`: 右侧同理
- `release.py`: `set_hand("DOWN")` → `move_xy(0, drop_y)` → `grasp(False)`

#### 5.7 `main/arm/examples/`

- `01_calibrate_origin.py`：启动 `OriginCalibrator`，按提示按 4 键。
- `02_trajectory_preview.py`：只算 S 曲线不连车，打印 t-x-y 表格。
- `03_move_xy_basic.py`：先 calibrate 一次，然后 `move_xy(100, 80)`。
- `04_grasp_template.py`：完整 pick-and-place 模板。

#### 5.8 文档

- `main/arm/README.md`：照 `main/chassis/README.md` 体例写。
- `main/arm/ARM_API.md`：业务层机械臂 API 速查，含 `arm.*` 原始 action 速查 + `ArmClient` 便捷方法 + 状态字段。
- `main/arm/QUICKSTART.md`：10 行起步：`pip install`、`calibrate_origin`、第一个 `move_xy`。
- 在 `main/README.md` 加一段"机械臂" 引导到 `main/arm/QUICKSTART.md`。
- 在 `main/API.md` 第 7 节 `arm` 末尾加一行"业务侧推荐用 `from main.arm import ArmClient`"。

#### 5.9 `.gitignore`

在 `main/test` 那一行附近加 `main/arm/arm_origin.yaml`（用户级配置不入库）。

#### 5.10 `main/arm/__init__.py`

```python
from .api import ArmClient
from .state import ArmState
from .origin import OriginCalibrator
from .trajectory import TrajectoryGenerator, TrajectoryPlan, TrajectorySample
from .loops.runner import ArmRunner

__all__ = ["ArmClient", "ArmState", "OriginCalibrator",
           "TrajectoryGenerator", "TrajectoryPlan", "TrajectorySample",
           "ArmRunner"]
```

## Assumptions & Decisions

1. **4 键连续按住**：直接复用 `arm_base.ArmController.set_manually()` 的按键约定（1=上、3=下、2=左、4=右），**不**新写一套点动逻辑（你选了连续按住）。
2. **x 撞墙方向可配**：默认 `x_wall="left"`（与车端 `reset_x` 目标 `-0.33` 一致）。
3. **S 曲线不在车端跑**：车端继续用现成 PID；S 曲线是**客户端** dry-run + 日志，避免改 runtime。
4. **`arm_origin.yaml` 与 `arm_cfg.yaml:pos_cfg` 共存**：前者业务软限位，后者车端硬件原点。**不**改 `arm_cfg.yaml`。
5. **不动 `runtime/core/actions.py`**：现有 14 个 `arm.*` action 已经够用。
6. **业务层 API 名字不变**：`arm.set_arm_pose` / `arm.move_xy_position` 等保持原名；`main/arm/` 是薄包装，不替代。
7. **首次 calibrate 必须有人**：第一次跑 `01_calibrate_origin.py` 需要按 4 键把机械臂移到触底 + 撞墙；后续 `arm.reset_origin()` 自动做。
8. **calibrate 写文件不是直接调车端 reset**：避免在 calibrate 流程中触发车端 reset 误判。

## Files to be created

- `main/arm/__init__.py`
- `main/arm/README.md`
- `main/arm/ARM_API.md`
- `main/arm/QUICKSTART.md`
- `main/arm/api.py`
- `main/arm/state.py`
- `main/arm/origin.py`
- `main/arm/trajectory.py`
- `main/arm/loops/__init__.py`
- `main/arm/loops/runner.py`
- `main/arm/tasks/__init__.py`
- `main/arm/tasks/go_home.py`
- `main/arm/tasks/pick_left.py`
- `main/arm/tasks/pick_right.py`
- `main/arm/tasks/release.py`
- `main/arm/examples/__init__.py`
- `main/arm/examples/01_calibrate_origin.py`
- `main/arm/examples/02_trajectory_preview.py`
- `main/arm/examples/03_move_xy_basic.py`
- `main/arm/examples/04_grasp_template.py`
- `main/arm/arm_origin.yaml`（含注释模板 + gitignore）

## Files to be modified

- `main/README.md` — 在目录索引里加 `arm/`
- `main/API.md` — 第 7 节 `arm` 末尾加 `main/arm` 业务封装推荐
- `.gitignore` — 第 153 行附近加 `main/arm/arm_origin.yaml`

## Files NOT touched

- `smartcar/whalesbot/vehicle/arm/arm_base.py`（底层不动）
- `runtime/core/actions.py`（action 名不动）
- `main/api_client.py`（HTTP 客户端不动）
- 现有 14 个 `arm.*` action 名字

## Verification

1. **静态校验**
   - `python3 -c "from main.arm import ArmClient, ArmState, OriginCalibrator, TrajectoryGenerator"` 不报错
   - `python3 -c "from main.arm.trajectory import TrajectoryGenerator; t = TrajectoryGenerator(); p = t.plan_xy(0, 0, 100, 80, 150, 400, 2000); print(p.T, p.peak_v)"` 输出 `t_peak`
   - `python3 main/arm/examples/02_trajectory_preview.py` 跑完打 t-x-y 表格

2. **冒烟测试**（在有车硬件时）
   - `python3 main/arm/examples/01_calibrate_origin.py` — 4 键定原点，文件写入 `arm_origin.yaml`
   - `python3 main/arm/examples/03_move_xy_basic.py` — 触发 `arm.move_xy_position` 不抛异常
   - `python3 main/arm/examples/04_grasp_template.py` — set_side + move_xy + grasp 不抛异常

3. **回归**
   - `python3 main/quick_start.py`（不动）仍能跑
   - `main/chassis/examples/01_minimal_p_lane.py`（不动）仍能跑

4. **文档**
   - 业务同学只看 `main/arm/QUICKSTART.md` 能 5 分钟起一个 move_xy

## Out of scope (本次不做)

- 不改 `arm_base.py` 的 PID / 触底逻辑 / x 撞墙判定（车端已有）
- 不动 MC602 协议层
- 不加新 action（`arm.*` 14 个不动）
- 不实现"双原点"（y 触底 + y 触顶都记）
- 不做 multi-arm
- 不加可视化（matplotlib、S 曲线 plot 留到 examples 之外再说）
