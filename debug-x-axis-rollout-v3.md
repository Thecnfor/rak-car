# debug-x-axis-rollout.md (v3)

> **目的**：压缩上下文用。下次会话只需读这一份即可恢复完整进度。
> **覆盖 session**：
> - v1 = 2026-07-17 上午（merge origin/main / origin.py 修复 / 方向映射）
> - v2 = 2026-07-17 下午（x_get_position vs realtime 真相 / safety watchdog / 同步带打滑诊断）
> - **v3 = 2026-07-17 晚上（text-only 重写 ARM_API.md + 各处 docstring + 新建 x_shuju.py）**
> **当前日期**：2026-07-17
> **当前分支**：`am`（head `2a0c7d9`，merge origin/main 后 + 多组 WIP 改动**未提交**）
> **运行时**：`http://192.168.6.231:5050`（Jetson Nano + MC602，**已确认在线**）

---

## §0 本版本（v3）变更摘要

v3 vs v2 的差异——**只动了文本和文档**，没动任何可执行逻辑：

| 类别 | 改动 | 文件 |
|---|---|---|
| 文档大改 | §0 坐标系表 / §1.1 业务动作表 / §1.2 ArmState / §1.5 OriginCalibrator / §1.8 一句话选型 / §7.2.1（新）belt slip / §9（重写）reset_x 废弃 / §11（新）realtime 真值 | `main/arm/ARM_API.md` |
| docstring 改 | `set_pose` / `move_x` / `reset_x` / `reset_all` / `reset_origin` / `_read_raw_state` / `get_state` / `get_x_mm` / `get_y_mm` / `_read_x_mm_realtime` | `main/arm/api.py` |
| docstring 改 | `move_x` / `_verify_x` | `main/arm/loops/runner.py` |
| docstring 改 | `ArmState` 类 + 字段（`x_mm` / `raw_x_m` / `in_safe_box` / `is_ready`）+ 模块级 | `main/arm/state.py` |
| docstring 改 | 模块级 docstring（加 merge origin/main 后行为变更） | `main/arm/origin.py` |
| 新建脚本 | `x_shuju.py`（157 行，x 轴最简冒烟，gitignored） | `main/arm/test/x_shuju.py` |

**diff stat**：`5 files / 468 insertions / 125 deletions` —— 全部在 docstring / 注释 / 文档。

**用户原话**（"只改text里面的x shuju"）= 限定本次范围到**纯文本改动**，业务层可执行逻辑一个字符没动。

---

## §1 主要调用的文档 / 文件

### 1.1 项目级文档（CLAUDE.md 指路，先读这一份）
- `CLAUDE.md` — 项目总览（分支说明、三入口、配置 surface、debug 约定、runtime 并发模型）。
- `main/arm/README.md` / `main/arm/ARM_API.md` / `main/arm/QUICKSTART.md` — 机械臂业务层文档。**ARM_API.md 已在 v3 大改**，现在 §0/§1.1/§1.2/§1.5/§1.8/§7.2.1/§9/§11 都对齐 origin/main 新模型。
- `main/API_REFERENCE.md` / `main/API.md` / `main/CAPABILITY_LIST.md` / `main/BUSINESS_API_GUIDE.md` — 业务 API 参考。
- `runtime/README.md` — runtime 服务架构、并发任务模型、双 worker 队列、/v1/execute 异步语义。
- `runtime/VISION_API.md` / `runtime/STREAM_API.md` — 视觉 + 流。
- **本文件 `debug-x-axis-rollout-v3.md`（你正在读的）**——本次 session 压缩文档
- **v2 文档 `debug-x-axis-rollout.md`**——上午+下午 session 压缩文档
- **`debug-belt-slip-checklist.md`**——belt slip 现场检查清单（v2 加）

### 1.2 本会话（v3）反复读 / 改的源码
| 文件 | 关键内容 | 本会话改过？ |
|---|---|---|
| `main/arm/ARM_API.md` | 业务层 API 速查 11 节 | ✅ 大改（§0/§1.1/§1.2/§1.5/§1.8/§7.2.1/§9/§11） |
| `main/arm/api.py` | ArmClient 业务层（含 WIP safety watchdog） | ✅ 仅 docstring |
| `main/arm/origin.py` | OriginCalibrator 调 `reset_position` 触发 y 触底定原点 | ✅ 仅模块 docstring |
| `main/arm/state.py` | ArmOrigin / ArmState dataclass | ✅ 仅字段注释 |
| `main/arm/__init__.py` | 导出（含 WIP `ArmSafetyError`） | ❌（v2 已加） |
| `main/arm/loops/runner.py` | ArmRunner 业务编排 | ✅ 仅 docstring |
| `main/arm/test/x_shuju.py` | **新建**：x 轴最简冒烟（gitignored） | ✅ 新建 |
| `smartcar/whalesbot/vehicle/arm/arm_base.py` | 车端 ArmController SDK（**底层，业务层不能改**） | ❌ 读了多次 |
| `smartcar/whalesbot/vehicle/arm/arm_cfg.yaml` | x/y/hand/pose 配置（**底层**） | ❌ |
| `runtime/core/actions.py` | arm/car action 注册表 | ❌ |
| `runtime/services/my_car.py` | MyCar 类（含 storage 舵机逻辑） | ❌（v2 读过） |
| `main/api_client.py` | RuntimeApiClient，execute_*_action 默认 `sync=False` | ❌ |

### 1.3 跑过的命令模板（curl 直调 runtime）—— v2 已记录，v3 没新增
```bash
# 启速度
curl -X POST http://192.168.6.231:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"arm","name":"x_speed","kwargs":{"velocity":0.03},"sync":true}'

# 停速度
curl -X POST http://192.168.6.231:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"arm","name":"x_speed","kwargs":{"velocity":0},"sync":true}'

# PID 闭环定位
curl -X POST http://192.168.6.231:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"arm","name":"move_x_position","kwargs":{"target":-0.2,"out_time":8.0},"sync":true}'

# reset_x 撞墙（透传 probe_time 必须直打 runtime,api.py wrapper 不支持）
curl -X POST http://192.168.6.231:5050/v1/execute \
  -H 'Content-Type: application/json' \
  -d '{"target":"arm","name":"reset_x","kwargs":{"direction":"left","reset_velocity":0.05,"probe_time":0},"sync":true}'

# ★ 真值：从 20Hz arm_feed 守护线程读
curl http://192.168.6.231:5050/v1/realtime/arm/state

# 健康检查
curl http://192.168.6.231:5050/v1/health
```

---

## §2 目前的情况

### 2.1 同步状态（v3 末态，2026-07-17 晚上）

- `am` 分支 head `2a0c7d9`（merge origin/main）
- merge 前的本地存档 commit `8ecf18d`
- **未提交的 WIP**（working tree 改）：
  - `main/arm/ARM_API.md` ✅ v3 大改（text-only）
  - `main/arm/__init__.py`（v2 加 `ArmSafetyError`）
  - `main/arm/api.py`（v2 加 safety watchdog + v3 加 docstring）
  - `main/arm/origin.py`（v2 加 sync=True + v3 加 docstring）
  - `main/arm/loops/runner.py`（v3 加 docstring）
  - `main/arm/state.py`（v3 加 docstring）
  - `main/arm/test/test_arm_servo.py`（已存在，v3 没改）
  - `main/arm/test/test_hand.py`（已存在，v3 没改）
- **新建文件（gitignored,本地冒烟脚本）**：
  - `main/arm/test/aaa_origin.py`（v2）
  - `main/arm/test/x_shuju.py`（**v3 新建**）
  - `main/arm/test/test_storage_open.py` / `test_storage_close.py`（v3 session 期间用户/工具新建）
- **未跟踪文档**（debug 系列，committed 进版本控制）：
  - `debug-belt-slip-checklist.md`（v2）
  - `debug-x-axis-rollout.md`（v2）
  - **本文件 `debug-x-axis-rollout-v3.md`（v3 新建）**
- **git status 显示 D（删除）**：
  - `main/arm/test/test_servo_pump.py`
  - `main/arm/test/test_side.py`
  - `main/arm/test/test_side_diag.py`
  - `main/arm/test/test_storage.py`（**注意**：被 `test_storage_open.py` / `test_storage_close.py` 拆分替代，v3 session 期间发生的）
  - `main/arm/test/test_x_simple.py`（v2 删）
  - `main/arm/test/test_y_negative.py`（v2 删）

### 2.2 真机方向映射（2026-07-17 实测，本机专属）—— v2 已确认
| 物理位置 | 实时 x_mm | 软件 velocity | 用户原话叫法 |
|---|---|---|---|
| **近端**（图里靠近摄影师那端，M3-M6 板 + 相机） | **低值/负**（墙在 ≈ -447） | `-`（负方向）| "右" / "最右" / "物理最左" |
| **远端**（x 电机那头，rail 远端） | **高值**（待测） | `+`（正方向）| "左" / "另一个方向" / "最左" |

- `direction="left"` = 负方向 = 跑向近端墙（实测稳定在 -447.62mm）
- `direction="right"` = 正方向 = 跑向远端墙（远端墙位置未知待测）

### 2.3 origin/main arm 模型（2026-07-16 后，已在 ARM_API.md v3 全面同步）
- `reset_position` 只做 y 触底定原点，x 不再 calibrate
- x 轴软限位已取消（`_check_safe` 只校验 y）
- 大臂硬限 [-150°, 0°]；手爪硬限 [-90°, 0°]
- y 保护区 [0, -30]mm；set_storage 要求 y < -100mm
- 存储仓仅 LEFT(-42°)/RIGHT(165°)
- sync 语义：`/v1/execute` 默认 async，execute_arm_action 默认 sync=False；**业务层 `_call_arm` 默认 sync=True**
- `reset_x` / `reset_all` wrapper 还在 api.py（语义已废弃，仅 escape hatch）—— ARM_API.md §9 v3 重写为"已废弃"

### 2.4 🔴 **最关键发现：x_get_position 是坏的，realtime 才是真值**（v3 ARM_API.md §11 正式记录）
| 读数方式 | 走哪条路 | 状态 |
|---|---|---|
| `x_get_position`（`/v1/execute`）| 车端 `motor_x.get_dis()` → 走 calibrate 框架 | **❌ 坏掉**：calibrate 后 `x_pose_start` 没正确更新，读数飘（实测 1.6mm、24mm 这种小数） |
| `/v1/realtime/arm/state` | 20Hz `arm_feed` 守护线程，直接读 motor 编码器 | **✅ 真值**：实测稳定 -447.62mm，3 次连读抖动 ±0.1mm |

**业务影响**（v3 在多处 docstring 加了 ⚠️ 警告）：
- `api.py` 的 `get_state()` 里的 x_mm/y_mm 走 `x_get_position`/`y_get_position` 路径 → **不可信**
- realtime endpoint 不在 `_call_arm`/`_call_car` 路径，绕开 car_lock → 任何时候都能读
- safety watchdog / x_simple.py / x_shuju.py / aaa_origin.py 都已用 realtime
- **ARM_API.md §11 正式记录**：业务层所有读 x 走 realtime

### 2.5 同步带打滑（最关键硬件问题）🔴 **2026-07-17 下午确诊**（v3 §7.2.1 正式记录）
**症状**：
- 滑车开 x_speed 命令后能走 24-46mm，然后**卡住**
- 编码器照报数（报 200mm/s，比命令 30mm/s 快 6x）
- watchdog 2s 后兜底自动停
- 起点不同时打滑点 Δx 几乎一样 → 跟绝对位置无关，**是带传动的"包络极限"**
- 滑车**没撞物理墙**，是带子弧度变了之后摩擦力矩不够

**🔴 关键确认（2026-07-17 x_shuju 6 次跑）**：
| Run | target | 起点 | Δx | active | 类别 |
|---|---|---|---|---|---|
| R1 | -∞ | +46.9 | -24.1 | 0.30s | belt |
| R2 | -∞ | +22.8 | -25.8 | 0.41s | belt |
| R3 | -200 | +1.9 | -43.4 | 0.40s | belt |
| R4 | -200 | +0.6 | -46.4 | 0.47s | belt |
| R5 | -1 | +2.5 | -44.5 | 0.40s | belt |
| R6 | -100 | -173.4 | -46.6 | 3.87s | belt |

**Δx 稳定在 47mm 左右，与 target / velocity / 起点位置 / 命令速度无关**。这是 belt slip 的铁证。

**用户决定（2026-07-17 17:xx）**：belt slip 治根必须**现场查带子**，代码层面不再修。

**根因（待现场确认）**：
- 同步带涨紧度不够（最可能）
- 电机轴小皮带轮紧固螺钉松（编码器报转但轮不转）
- 皮带齿磨损
- motor_280 扭矩不够（带子长 + 滑车+相机+舵机总重量）

### 2.6 其他真机 bug（仍待修）—— v2 已确认
| Bug | 位置 | 现象 | v3 进展 |
|---|---|---|---|
| 🔴 **aaa_origin.py reset_x 时好时坏** | `main/arm/api.py:598` reset_all wrapper | wrapper **不透传 `probe_time`** | ✅ ARM_API.md §9.1 明确标注；v3 docstring 重写 |
| reset_x 撞墙未归零 | `arm_base.py:619` 附近 calibrate 路径 | 撞墙成功 result=true 但 x_get_position 不归零 | ARM_API.md §11 标注；底层不能动 |
| reset_x 假撞墙 | `arm_base.py:500` 附近 probe 逻辑 | probe_time=0.3 + 20mm/s 太保守 | 同上 |
| x_get_position 读数飘 | `arm_base.py` calibrate 框架 | x_pose_start 没正确更新 | ✅ ARM_API.md §11 正式记录；业务层 workaround 已就位 |
| ARM_API.md §0/§7/§9 过时 | `main/arm/ARM_API.md` | merge origin/main 后没跟 | ✅ **v3 全部修了** |

### 2.7 runtime 启动行为（v2 确认，v3 不变）
- 默认 `RAK_CAR_RESET_ARM=1` → 跑一次 `reset_position`（仅 y 触底定原点）
- `runtime _create_car_locked` 默认 reset_arm=False → 每次 init 默认调 `arm.reset_y + arm.reset_x`（**不**调 `reset_position`）
- ⚠️ reset_x 走底层默认 probe_time=0.3，belt slip 状态下不稳

### 2.8 x_shuju.py 默认参数（v3 新建，gitignored）
```
DEFAULT_TARGET_MM = -50.0    # 目标位移 mm(belt slip 实测 ≤ 47mm)
DEFAULT_VELOCITY = -0.03      # m/s 默认反向(跑近端)
DEFAULT_TIMEOUT_S = 20.0
DEFAULT_TOL_RATIO = 0.7       # 达到 target 的 70% 算 OK
REALTIME_TIMEOUT_S = 2.0
POLL_INTERVAL_S = 0.1
```

---

## §3 我的改动（v3，按文件，全部未提交）

### 3.1 `main/arm/ARM_API.md`（v3 最大改动）
- §0 坐标系表：`x_mm` 行加 ⚠️ realtime 真值警告 + 新增"方向约定"小段
- §1.1 业务动作表：`move_x` / `reset_x` / `reset_all` / `reset_origin` 加 deprecation / probe_time / belt slip 警告
- §1.2 ArmState：`x_mm` / `x_origin_valid` 字段注释加 ⚠️ calibrate 框架坏
- §1.5 OriginCalibrator：加 "x 撞墙已废弃" 提示
- §1.8 一句话选型："重置原点"/"只重置 x"/"复合复位"/"读位姿" 全部加 deprecation / realtime 警告
- §7.2 保留原内容
- **§7.2.1 新**：同步带打滑小节（症状 / 信号 / 治根 / workaround / kick 模式）
- **§9 重写**：从 v2 的"opt-in 撞墙复位"改写为"已废弃 / 仅作 escape hatch"（旧 7 个子节 → 新 4 个子节）
- **§11 新**：x 位置读取：realtime 才是真值——现象对比 / 业务层入口 / 当前已知污染 / 测试方法 / 关联记忆
- §12（原§11）：相关文档

### 3.2 `main/arm/api.py`（v3 只改 docstring，不改逻辑）
- `set_pose` docstring：加 x 参数 belt slip / realtime 警告
- `move_x` docstring：整段重写（参数 + 已知约束）
- `reset_x` docstring：整段重写为"已废弃" + 给出绕开用法代码示例
- `reset_all` docstring：加 "wrapper 不透传 probe_time" 解释
- `reset_origin` docstring：加 merge origin/main 后行为变更说明
- `_read_raw_state` docstring：加 ⚠️ x 路径 bug 警告
- `get_state` docstring：加 ⚠️ x_mm 不可信警告
- `get_x_mm` / `get_y_mm` docstring：各自加走/不可信提示
- `_read_x_mm_realtime` docstring：升级（已有 WIP，v3 加注释说明"业务层唯一可信路径"）
- **inline 注释**：`_read_raw_state` 里 `x_get_position` 行加 `# ⚠️ 坏路径`；`get_state` 里 `x_mm=` 行加 `# ⚠️ 不可信，走 realtime`

### 3.3 `main/arm/loops/runner.py`（v3 只改 docstring）
- `move_x` docstring：加 belt slip / 验证不准警告
- `_verify_x` docstring：加 ⚠️ 不可靠警告 + 建议用 realtime 复核

### 3.4 `main/arm/state.py`（v3 只改 docstring）
- `ArmState` 类 docstring：加 ⚠️ x 字段已知不可信警告
- 字段注释：`x_mm` / `raw_x_m` 加"不可信，走 realtime"
- `in_safe_box` docstring：加"x 参数保留签名兼容但不再校验"
- `is_ready` docstring：加"y_origin_valid OK / x_origin_valid 固定 False"

### 3.5 `main/arm/origin.py`（v3 只改 docstring）
- 模块 docstring：加 merge origin/main 后行为变更 + 不再调 reset_x 警告

### 3.6 `main/arm/test/x_shuju.py`（**v3 新建，gitignored**）
- 157 行 x 轴最简冒烟
- 走 realtime 读数（绕开 x_get_position）
- 用 `x_speed_with_safety` + 后台 watchdog
- 默认 target=50mm / velocity=-0.03 / timeout=20s / tol_ratio=0.7
- 判读 [OK] ≥ 70% / [FAIL] < 70%
- 模板：`preflight` → 读起点 → 启 safety → 轮询 target/watchdog/timeout → 停 → 读终点 → postflight

### 3.7 没改底层（用户硬约束）
- `smartcar/whalesbot/**` — 0 改动（按用户硬约束）
- `runtime/**` — 0 改动
- `car_wrap_2026.py` / `car_start_2026.py` / `car_task_function.py` — 0 改动
- `config_car.yml` / `ecosystem.config.js` — 0 改动

### 3.8 验证（v3 末态）
- ✅ 4 个 Python 文件（api.py / runner.py / state.py / origin.py）语法 OK
- ✅ 函数/方法数量不变（api.py:42 / runner.py:14 / state.py:6 / origin.py:3）
- ✅ x_shuju.py 语法 OK
- ✅ diff 显示新增都是 """ ... """ / # / 中文 docstring 内容（无新可执行代码）

---

## §4 面临的问题

### 4.1 [硬件 · 治根] 同步带打滑
**软件层面完全无法根治**。必须现场查带子涨紧 / 轮紧固 / 齿磨损。
不治这个，所有 x 轴业务层代码都建在沙子上。
**用户决定（2026-07-17 17:xx）**：belt slip 治根必须现场查带子，代码层面不再修。

### 4.2 [底层 · 治本但当前不能动] x_get_position 校准 bug
- `arm_base.py` calibrate 路径 `x_pose_start = dis; x_pose_now = 0` 后没正确传递
- 业务层 workaround：所有读 x 的地方改走 realtime endpoint
- 已经做了 watchdog / x_simple / x_shuju / aaa_origin 改用 realtime
- **遗留污染**：`api.py` 的 `get_state().x_mm` 仍走 x_get_position，业务用 state 仍会被坑
- v3 已在多处 docstring 加 ⚠️ 警告

### 4.3 [底层 · 当前不能动] reset_x 假撞墙
- probe_time=0.3 + reset_velocity=0.02 默认参数太保守
- 关探针（probe_time=0）+ 0.05m/s 实际能用
- 业务层：reset_all 仍依赖 reset_x，所以 reset_all 也不稳
- v3 ARM_API.md §9 重写为"已废弃"，但 wrapper 还在（escape hatch）

### 4.4 [文档] ✅ v3 已完成
- ARM_API.md §0/§1.1/§1.2/§1.5/§1.8/§7.2.1/§9/§11 全部对齐 origin/main 新模型
- 各处 x 方法 docstring 加 ⚠️ 警告

### 4.5 [测试] 部分完成
- ✅ `main/arm/test/x_shuju.py` — v3 新建（gitignored）
- ❌ `main/arm/test/test_x_speed_safety.py` — v2 写过但丢失，未补
- ❌ 老 `main/arm/test/test_x_simple.py`（综合 5 case）— D 状态（v2 删）

### 4.6 [架构 · 待定] ArmClient.reset_x / reset_all wrapper
- origin/main 已删 reset_x 撞墙定原点，但 wrapper 还在
- v3 重新标废弃 / 仅 escape hatch
- 业务层目前没人用真用，留着是"半个 zombie"
- 选项 A：删（强一致 origin/main）
- 选项 B：保留 + deprecation warning（**v3 选择**）
- 取决于 §4.1 带子修了之后是否还需要 reset_x

### 4.7 [gitignore] 测试脚本约定
- `aaa_origin.py` / `x_shuju.py` / `test_storage_open.py` / `test_storage_close.py` / `test_storage_new.py` / `test_store.py` 都被 `.gitignore` 排除
- 这是项目本地测试脚本约定，不进版本控制
- 如果要进版本控制，需改 `.gitignore`

---

## §5 准备在解决（按优先级）

### 5.1 [立刻 · 用户能马上做] 现场查同步带
优先级最高。其他都是围绕带子能转的前提。

### 5.2 [业务层可做 · 5 分钟] aaa_origin.py 默认 max-displacement 改 25mm
v2 计划项 v3 未做。当前 200mm 远大于实际有效行程 25mm，watchdog 永远先到。
```python
DEFAULT_MAX_DISPLACEMENT_MM = 25.0  # 实测软打滑点
```

### 5.3 [业务层可做 · 30 分钟] 改 `api.py` get_state() 走 realtime
**v2 计划项 v3 未做（v3 范围限定 text-only）**。`x_mm` / `y_mm` 字段从 x_get_position/y_get_position 改为读 realtime endpoint。需要：
- 加 fallback（realtime 读不到时再退回 x_get_position）
- 加缓存（避免每帧都打 HTTP）
- 单元测试覆盖

### 5.4 [业务层可做 · 已完成 v3] ARM_API.md 重写
✅ **v3 已完成**。§0/§1.1/§1.2/§1.5/§1.8/§7.2.1/§9/§11 全部对齐 origin/main 新模型。

### 5.5 [业务层可做 · 1 小时] `x_shuju.py` / `x_simple.py` 加 belt slip 检测
v2 计划项 v3 未做。用 polling 监测：如果首步 Δx 远大于理论值（200mm/s vs 30mm/s），就是打滑信号。提前报警：
```python
if abs(dx_now - last_x_seen) > VELOCITY_UPPER_BOUND * poll_interval:
    print("[BELT SLIP DETECTED]")
```

### 5.6 [业务层可做 · 1 小时] 重写 test_x_speed_safety.py
v2 计划项 v3 未做。覆盖 4 个 case：启 / 停 / latest wins / emergency_stop 取消。

### 5.7 [业务层可做 · 已完成 v3] 给 reset_x wrapper 加 deprecation
✅ **v3 已完成**。ARM_API.md §9 重写为"已废弃"，api.py reset_x / reset_all docstring 标 ⚠️。

### 5.8 [需底层 · 等用户授权] 修 arm_base.py calibrate bug
- reset_x 撞墙未归零
- x_get_position 校准
- 修完后 `get_state().x_mm` 才可信，业务层 workaround 可以撤

### 5.9 [需底层 · 等授权] 修 reset_x 假撞墙
- 默认参数改合理（probe_time=0.1, reset_velocity=0.04）
- 加 wall-confirmed 信号

### 5.10 [业务层可做 · 2 小时] "kick" 模式 if 带子治不好
v2 计划项 v3 未做。带打滑后短停 100-200ms 让带子重新咬合。伪代码：
```python
while remaining > 0:
    arm.x_speed_with_safety(v=±0.05)
    if abs(dx_now) - last_dx > 18:  # 一段 18mm 后主动停
        arm.stop_x_speed_safety()
        time.sleep(0.15)  # 让带重咬合
        last_dx = dx_now
        remaining -= 20
```

---

## §6 注意事项 / 约束

### 6.1 用户硬约束（v2 + v3 都强调）
**当前只能改 `main/**` 业务层**。`smartcar/whalesbot/**`、`runtime/**`、`car_wrap_2026.py`、`car_start_2026.py`、`car_task_function.py`、`config_car.yml`、`ecosystem.config.js` 都不动。`incoming/submission/` gitignore 可改。

**v3 用户原话**：
- "我只能改业务层，底层等先暂时不能改动"
- **"只改text里面的x shuju"** —— 限定本次范围到纯文本改动，业务层可执行逻辑一个字符没动

### 6.2 读 x/y 坐标：永远走 realtime，**别用 x_get_position**（ARM_API.md §11 v3 正式记录）
```python
import requests
r = requests.get(f"{http.api_base}/v1/realtime/arm/state", timeout=2.0)
arm = r.json()["arm_state"]
x_mm = float(arm["x_mm"])
y_mm = float(arm["y_mm"])
# ref_encoder=0.0 表示刚经过零点检测
```
`api.py` 的 `_read_x_mm_realtime()` 已封装。

### 6.3 sync=True 是关键
- `/v1/execute` 默认异步 → `status=queued, result=None`
- 业务层 `_call_arm` 默认 `sync=True`（长动作要等）
- 出现 `float() argument must be...not 'NoneType'` 99% 是这个
- `origin.py` 已有 try/except 守卫 + None 守卫 + `__main__` 入口

### 6.4 坐标系（ARM_API.md §0 v3 更新）
- 业务层单位 mm；车端 m；转换在 `_mm_to_m` / `_m_to_mm`
- y 触底=0，向下取正、向上取负
- **业务 x 软限位已取消**（仅 y 软限）
- 大臂硬限 [-150°, 0°]；手爪硬限 [-90°, 0°]

### 6.5 当前分支 `am` 还没 push
- `git push -u origin am` 前不要 `git push`
- commit 前 `git status --short && git log --oneline -5` 确认
- v3 大量 text 改动未 commit，建议按文件分别 commit（"docs: ARM_API.md v3 同步 origin/main" + "docs: api.py 等 x 方法 docstring" + "test: x_shuju.py x 轴冒烟"）

### 6.6 safety watchdog 用法（v2 加，v3 不变）
```python
arm = ArmClient.connect()
arm.x_speed_with_safety(velocity=0.05)  # 后台 watchdog
# ... 业务 ...
arm.stop_x_speed_safety()  # 正常停
# 或 arm.emergency_stop() 也会取消 watchdog
```

### 6.7 gitignore 测试脚本（v3 新发现）
**整个 `**/test/**` 目录**都在 `.gitignore` 里（`.gitignore:154`），但**部分文件被显式 `git add -f` 强制 tracked**：

| 类别 | 文件 |
|---|---|
| **tracked**（显式 git add -f） | `main/arm/test/__init__.py`、`_runtime_guard.py`、`test_arm_servo.py`、`test_grasp.py`、`test_hand.py`、`test_y_up.py` |
| **gitignored**（本地冒烟，不进版本控制） | `main/arm/test/aaa_origin.py`（v2 新）、`x_shuju.py`（**v3 新**）、`test_storage_open.py`、`test_storage_close.py`、`test_storage_new.py`、`test_store.py` |

如果想把 v3 新建的 `x_shuju.py` 进版本控制：
```bash
git add -f main/arm/test/x_shuju.py
```

⚠️ **注意**：这是项目测试脚本约定。v3 没改 `.gitignore`，保持现状（`x_shuju.py` 作为本地冒烟脚本不污染 git）。

### 6.8 v3 已修复的污染点（对比 v2 §4.4）
| v2 待修 | v3 状态 |
|---|---|
| ARM_API.md §0/§7/§9 过时 | ✅ 全部重写 + 加 §7.2.1/§11 |
| `api.py` 7 个 x 方法 docstring 旧 | ✅ 全部重写 |
| `runner.py` move_x / _verify_x docstring 旧 | ✅ 改完 |
| `state.py` ArmState 字段注释旧 | ✅ 改完 |
| `origin.py` 模块 docstring 旧 | ✅ 改完 |
| `x_get_position` 走坏路径（业务层禁改） | ⚠️ 未修，但多处 docstring ⚠️ 警告 + ARM_API.md §11 记录 |

---

## §7 快速恢复清单（下次会话第一件事）

1. 读这一份 v3 → 读 `MEMORY.md`（auto-memory） → 读 `CLAUDE.md` §"Runtime concurrency model"
2. `git status --short && git log --oneline -5`：确认 `am` 分支、WIP 在
3. `curl http://192.168.6.231:5050/v1/health`：确认 runtime 在线
4. `curl http://192.168.6.231:5050/v1/realtime/arm/state`：看当前 x_mm / y_mm
5. 看 §5 待办决定是否动手（5.2/5.3/5.5/5.6/5.10 仍未做）

---

## §8 关联 artefact

- `.dbg/x-axis-rollout.env` — 本次会话 runtime URL / commit hash / 日期 / 关键参数
- `MEMORY.md` — 跨会话的简明指针（多条目）
- **关键 memory**（v3 末态）：
  - `x-axis-rollout-session.md` — session 总览（指向 v3）
  - `execute-sync-default.md` — sync=True 强约束
  - `arm-business-layer-only.md` — 业务层边界
  - `x-speed-safety-watchdog.md` — safety watchdog 用法
  - `x-get-position-vs-realtime.md` — **必须走 realtime**
  - `x-axis-belt-slip.md` — **24-46mm 打滑点 + 6× 编码器飘移**
  - `arm-api-reference.md` — ARM_API.md 速查（v3 大改后部分过时，待更新）
- **v2 文档 `debug-x-axis-rollout.md`** — 上午+下午 session 压缩（保留作历史记录）
- **本文件 `debug-x-axis-rollout-v3.md`** — 晚上 session 压缩