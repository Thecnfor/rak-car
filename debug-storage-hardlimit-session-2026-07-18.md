# Storage 业务硬限 Session 总结（2026-07-18）

> 下次会话第一件事读这个文档，能直接接上 storage 业务硬限的状态。
> 配套可读文档：`main/arm/ARM_API.md` §6、`main/arm/api.py`、memory 中 7 个 .md。

---

## 📚 主要调用的文档（优先级排序）

### 项目级
| 文档 | 用途 | 何时读 |
|---|---|---|
| `CLAUDE.md` | 项目总览：架构、entry point、硬约束、env vars | 每次会话开始 |
| `memory/MEMORY.md` | 跨会话记忆索引 | 每次会话开始 |

### 机械臂业务层（重点）
| 文档 | 用途 | 何时读 |
|---|---|---|
| **`main/arm/ARM_API.md` §6** | **储存仓章节（本次改动主战场）** | **改 storage 必读** |
| `main/arm/api.py` | 业务 wrapper 实现（含所有硬限） | 改 storage / arm 限制必读 |
| `main/arm/state.py` | ArmOrigin / ArmState dataclass | 改 y/x 坐标系读 |
| `main/arm/loops/runner.py` | ArmRunner 业务编排 | 改 runner 必读 |
| `main/arm/origin.py` | OriginCalibrator 零点标定 | 改 reset / 初始化读 |
| `main/arm/arm_origin.yaml` | 持久化的原点 + 软上限 | 改原点读 |

### 硬件层（参考但不改动）
| 文档 | 用途 | 何时读 |
|---|---|---|
| `smartcar/whalesbot/vehicle/arm/arm_base.py` | SDK 底层（PID/磁感/撞墙 calibrate） | 排查硬件行为时参考 |
| `smartcar/whalesbot/vehicle/arm/arm_cfg.yaml` | 底层 PID 限幅 / y_threshold / 减速带 | 看物理参数时读 |
| `runtime/services/my_car.py:460-510` | storage 舵机底层（servo_1_angle_list=[-42, 165]） | 看 bool 抽象映射时读 |

### Runtime 服务层（参考但不改动）
| 文档 | 用途 |
|---|---|
| `runtime/README.md` | 并发任务模型 / 锁层次 / job queue |
| `runtime/services/runtime_service.py` | 双 worker 队列 / auto-init |
| `runtime/api/routes.py` | /v1/execute 异步 / lane_feed / arm_feed |

### Memory 关键条目（auto-loaded）
- `[[arm-business-layer-only]]` — 只能改 main/**，其他层禁动
- `[[arm-api-reference]]` — ARM_API.md 速查
- `[[x-axis-belt-slip]]` — x 轴同步带打滑（与 storage 无关但同 session 上下文）
- `[[x-get-position-vs-realtime]]` — x 读数走 realtime
- `[[x-speed-safety-watchdog]]` — x_speed safety watchdog
- `[[execute-sync-default]]` — /v1/execute 默认 sync=False
- `[[x-axis-rollout-session]]` — x 轴全天 session 总览

---

## 🎯 当前情况（截至 2026-07-18 末次改动）

### 项目背景
- 百度智能车 2026 智慧农业赛道，Jetson Nano + MC602 + WhalesBot mecanum
- 三层架构：业务层 `main/**` / SDK 底层 `smartcar/whalesbot/**` / Runtime 服务 `runtime/**`
- 三个 entry point：legacy monolith / Runtime FastAPI / 业务 client

### Session 内 arm 限制条件全图（v3 终态）
| 限制 | 文件:行 | 值 | 状态 |
|---|---|---|---|
| **储存仓业务硬限（枚举）** | — | — | ❌ **Round 7 加, Round 12 撤**（user 决定） |
| **set_storage_angle 枚举校验** | — | — | ❌ **Round 7 加, Round 12 删**（任意角度通过） |
| **pre_init_close_storage 默认值** | `api.py:94` | `angle=STORAGE_CLOSE_ANGLE_DEG (=98)` | ✅ 保留 |
| **开仓 y 安全门 [-200, -150] mm** | — | — | ❌ **Round 8 加后 Round 10 撤**（user 决定） |
| **开仓前自动抬 y** | `api.py:78-83`（常量）+ `api.py:_ensure_y_for_open_storage`（方法）+ `api.py:set_storage_angle` 入口 | 每次 75° 自动 `move_y_position(target=-0.150)`；move_y 失败 fail-closed | ✅ **Round 11 重加**（Round 12 决定保留） |
| 储存仓 bool 抽象硬限 | — | — | ❌ 不存在（设计如此） |
| 底层 `car.set_storage_angle` 校验 | — | — | ❌ 不存在（memory 硬约束） |
| 大臂角度硬限 `[0, -150]°` | `api.py:425-426` | `set_arm_angle` | 早就有 |
| 手爪角度硬限 `[-90, 0]°` | `api.py:489-490` | `set_hand_angle` | 早就有 |
| y 保护区 `[0, -30] mm` | `api.py:383` | `_check_y_protected` | 早就有 |
| x 软限位 | — | **已取消** | 用户原话"灵活使用就好" |
| x_speed safety watchdog | `api.py:824-909` | 2s/0.5mm/0.2s | 早就有 |

### 三个 storage 测试脚本职责
| 脚本 | 路径 | 走哪条路径 | 角度 | 期望 |
|---|---|---|---|---|
| `test_storage_open.py` | `main/arm/test/` | **`ArmClient.set_storage_angle(75)`** 业务 wrapper | 75° (OPEN) | PASS |
| `test_storage_close.py` | `main/arm/test/` | **`ArmClient.set_storage_angle(98)`** 业务 wrapper | 98° (CLOSE) | PASS |
| `test_storage_limit.py` | `main/arm/test/` | 业务 wrapper | 5 case（默认全跑 / `--reject-only`） | reject raise, accept PASS |
| ~~`test_storage_open_y_gate.py`~~ | — | — | — | ❌ **Round 10 撤（用户决定）** |
| ~~`test_storage_open_auto.py`~~ | — | — | — | ❌ **Round 10 撤（用户决定）** |

---

## 🔧 Session 改动清单（按时间）

### Round 1-2: 调研 + 加区间硬限 [75, 98]°
- 调研 arm 限制条件分布（api.py + arm_base.py + arm_cfg.yaml）
- `api.py:71-72`: 加 `STORAGE_ANGLE_MIN_DEG=75` / `STORAGE_ANGLE_MAX_DEG=98`
- `api.py:616-664`: 新增 `ArmClient.set_storage_angle(angle, speed=10, timeout=10.0)` 业务 wrapper
- `pre_init_close_storage` 注释更新（98° 与业务硬限上界对齐，但不走 wrapper）
- `loops/runner.py:154-163`: 加 `ArmRunner.set_storage_angle` 委托
- `ARM_API.md §6.1/§6.2`: 文档化业务硬限 + escape hatch

### Round 3: 三个 storage 测试脚本
- `test_storage_open.py`: 重命名 `STORAGE_CLOSE_ANGLE_DEG` → `STORAGE_OPEN_ANGLE_DEG=75`
- `test_storage_close.py`: 100° → 改回 98°（放弃 100° 边界压力测试语义），重命名为 `STORAGE_BOUNDARY_ANGLE_DEG=100` → 恢复 `STORAGE_CLOSE_ANGLE_DEG=98`
- `test_storage_limit.py`: **新建**，5 case (50/74/75/98/99°)，加 `--reject-only` 开关
- 所有测试脚本**改走 ArmClient 业务 wrapper**

### Round 4: 验证业务 wrapper 真生效
- mock 跑 10 个 case 确认 `[75, 98]` 区间硬限有效
- mock 跑 set_storage_angle(75) 在各种 y 状态（无关，开仓 y 安全门已撤）

### Round 5: 加开仓 y 安全门 [-200, -150] mm
- `api.py:82-86`: 加 `STORAGE_OPEN_Y_MIN_MM=-200` / `STORAGE_OPEN_Y_MAX_MM=-150` 常量
- `api.py:901-935`: 新增 `_check_y_for_open_storage()` 方法
- `api.py:589-593`: `set_storage("RIGHT")` 入口加 y 检查
- `api.py:668-671`: `set_storage_angle(≤ 75°)` 入口加 y 检查
- `ARM_API.md §6.3`: 文档化开仓 y 区间

### Round 6: **撤掉开仓 y 安全门（用户决定）**
- 删 `STORAGE_OPEN_Y_MIN_MM / MAX_MM` 常量
- 删 `_check_y_for_open_storage()` 方法
- 删 `set_storage` 和 `set_storage_angle` 入口的 y 检查
- 删 `ARM_API.md §6.3`
- **保留**：业务角度硬限 [75, 98]° 区间

### Round 7（最新）: **区间改枚举两档**
- `api.py:70-71`: 删 `STORAGE_ANGLE_MIN/MAX_DEG`，改用 `STORAGE_OPEN_ANGLE_DEG=75` / `STORAGE_CLOSE_ANGLE_DEG=98`
- `api.py:81`: `pre_init_close_storage` 注释引用名更新
- `api.py:623-638`: `set_storage_angle` docstring 更新
- `api.py:655`: 校验逻辑 `if a not in (OPEN, CLOSE)` + 错误消息改为枚举语义
- `ARM_API.md §6.1`: 文档改为枚举两档 + 历史说明（"早期是区间，2026-07-18 改枚举"）

### Round 8（最新）: **开仓 y 安全门重做（Round 5 加/撤，重做）**
- `api.py:73-82`: 新增 `STORAGE_OPEN_Y_MIN_MM=-200` / `STORAGE_OPEN_Y_MAX_MM=-150` 常量
- `api.py:_check_y_for_open_storage()`: 新增方法，紧跟 `_check_y_protected` 之后
  - **fail-closed**：读不到 y → raise（与 `_check_y_protected` fail-open 不同）
  - 错误消息含实际 y 值 + 期望区间 + 解决方案
- `api.py:set_storage_angle()`: 在枚举校验通过后，仅当 `a == STORAGE_OPEN_ANGLE_DEG` 时调 gate
  - **不**影响 `set_storage_angle(98° CLOSE)` 和 `set_storage(side)` bool 抽象
- `ARM_API.md §6.3`: 新增开仓 y 安全门章节（触发条件 / fail-closed / 与 §7.1 关系 / 绕过路径）
- **`test_storage_open_y_gate.py`**: 新建，13 个 mock case（8 OPEN + 4 CLOSE 旁路 + 1 fail-closed）
  - `--reject-only`: 10 case (只跑 BLOCKED)
  - `--hardware`: 抬 y 到 -175 实跑 75°（会动舵机）
- mock 验证全绿：13/13 PASS
- **关键决策记录**：
  - **gate 只在 `set_storage_angle(75°)`**：因为只有 75° 是 OPEN 物理位
  - **`set_storage("RIGHT")` 不走 gate**：bool 抽象底层映射 165°，不是 OPEN 物理位
  - **HTTP / `pre_init_close_storage` 不走 gate**：保留逃生口 + init 阶段没 ArmClient 实例

### Round 9（最新）: **开仓前自动抬 y 到 -150mm**
- `api.py:84-85`: 新增 `STORAGE_OPEN_AUTO_Y_MM=-150` (= STORAGE_OPEN_Y_MAX_MM) / `STORAGE_OPEN_AUTO_Y_TIMEOUT_S=15.0` 常量
- `api.py:_ensure_y_for_open_storage()`: 新增方法，紧跟 `_check_y_for_open_storage` 之后
  - 调 `_call_arm("move_y_position", target=-0.150, sync=True, timeout=15.0)`
  - HTTP 异常 → raise ValueError
  - `status != "succeeded"` → raise ValueError
  - 错误消息含 status / err / 解决建议
- `api.py:set_storage_angle()`: 在枚举校验通过 + a == OPEN 时，调 `_ensure_y_for_open_storage` 再调 `_check_y_for_open_storage`
  - **顺序**：auto-move 先，gate 后（defense in depth）
- `ARM_API.md §6.3`: 加"Round 9 开仓前自动抬 y"段落
- `test_storage_open_y_gate.py`: 全模式 15 case（8 OPEN + 4 CLOSE + 1 y-read-fail + 1 move_y-failed + 1 move_y-exception）
  - `--reject-only`: 12 case (只跑 BLOCKED)
  - 全部 mock 验证 15/15 + 12/12 PASS
- **关键决策记录**：
  - **目标 y = -150mm**（与 _check_y_for_open_storage 的 MAX 对齐，不是 -175 中间位）：用户原话"调到 -150"
  - **总是触发 auto-move**（即使 y 已经在区间内）：用户原话"自动调到"，理解为始终执行
  - **auto-move 先于 gate**：move_y 完成后才校验，避免 move_y overshoot 还要 raise 重试
  - **失败模式**：HTTP 异常 / status!=succeeded 都 raise，**不**下发 75°（fail-closed）
  - **不提供 escape hatch**：用户代码永远走 auto-move（一致性 > 灵活性）

### Round 10: **撤 Round 8 + Round 9（用户决定）**
- **触发原因（用户原话）**："你前面绝对写了什么东西，让y轴只能在-150以上，现在你给我改过来啊！！！！"
  - 用户看到 y 留在 -150 后认为 session 改动让 y 卡在 -150。
  - 实际：session 改动只在 `set_storage_angle(75°)` 入口触发 auto-move，跟 `move_y` / `move_xy` / `set_pose` / `reset_y` / `reset_position` 完全无关。
  - 但用户坚持撤，按用户决定执行。
- `api.py`: 删 4 个常量 (`STORAGE_OPEN_Y_MIN_MM` / `MAX_MM` / `AUTO_Y_MM` / `AUTO_Y_TIMEOUT_S`)
- `api.py`: 删 `_check_y_for_open_storage` 和 `_ensure_y_for_open_storage` 两个方法
- `api.py:set_storage_angle()`: 删两个方法的调用，回到纯角度枚举硬限（Round 7）
- `ARM_API.md §6.3`: 整个章节删除，加一行 "Round 8/9 已撤"
- 测试脚本（`test_storage_open_y_gate.py` / `test_storage_open_auto.py`）: 用户已自行删除
- mock 验证回滚：13/13 PASS（Round 7 角度枚举硬限仍生效）
- **现状**：set_storage_angle(75°) / (98°) 仍走业务硬限（仅枚举两档），但不做 y 检查也不自动 move_y

### Round 11: **重做 Round 9 auto-move（不加 y gate，用户决定）**
- **触发原因（用户原话）**："现在你千万不要犯前面的错了，还是继续加限制条件，在开仓的时候，y轴自动调节到-150"
- **范围控制**：只加 Round 9 的 auto-move，**不加** Round 8 的 y gate
  - auto-move 是 "用户友好"（自动抬 y，不用手动 move_y）
  - y gate 是 "强约束"（y 不在区间 raise 不下发）—— 容易让人误以为 y 被卡住
- `api.py:78-83`: 加 `STORAGE_OPEN_AUTO_Y_MM=-150` / `STORAGE_OPEN_AUTO_Y_TIMEOUT_S=15.0` 常量
- `api.py:_ensure_y_for_open_storage()`: 加方法（紧跟 `_check_y_protected` 之后）
  - 调 `_call_arm("move_y_position", target=-0.150, sync=True, timeout=15.0)`
  - HTTP 异常 / status!=succeeded / timeout → raise ValueError
  - 错误消息含 `[storage_auto_y]` 前缀
- `api.py:set_storage_angle()`: 仅当 `a == STORAGE_OPEN_ANGLE_DEG (75°)` 时调 `_ensure_y_for_open_storage()`
  - 98° CLOSE 完全不触发
  - `set_storage(side)` bool 抽象完全不触发
  - **任何其他 y 轴方法（move_y / move_xy / set_pose / reset_y / reset_position）一行都没动**
- `ARM_API.md §6.3`: 重写为 Round 11 auto-move 文档（删 y gate 段落）
- mock 验证 8/8 PASS：
  - 75° 触发 move_y(target=-0.150) ✓
  - 98° 不触发 ✓
  - set_storage("RIGHT") 不触发 ✓
  - 50° 角度枚举硬限仍生效 ✓
  - move_y 失败 → fail-closed raise + 不下发舵机 ✓
  - move_y HTTP 异常 → fail-closed ✓
  - **ArmClient.move_y() 直接调用不被拦截** ✓（业务层只动 set_storage_angle 入口）
- **关键决策记录**：
  - **范围最小化**：只 auto-move 不 gate → 用户调用 `arm.move_y(-200)` 不会被任何业务层逻辑拦
  - **历史意识**：Round 9 之前用户能看到 y 自由动；Round 8 加 y gate 后用户感知 "y 卡住"；这次只加 auto-move 不加 gate 避免混淆
  - **fail-closed 一致**：move_y 失败不开仓，跟 Round 9 设计一致

### Round 12（最新）: **删 Round 7 角度枚举硬限（用户决定）**
- **触发原因（用户原话）**："现在把储存仓的限制也删了，我要重新测角度了"
- **保留**：Round 11 auto-move（用户上次特意重做的）→ auto-move 只在 `set_storage_angle(75°)` 触发，不影响其他角度测试
- **保留**：pre_init_close_storage 用 `STORAGE_CLOSE_ANGLE_DEG=98`（init 阶段固定打 98°）
- **删除**：
  - `set_storage_angle()` 入口的 `if a not in (75, 98): raise ValueError` 校验
  - `STORAGE_OPEN_ANGLE_DEG` / `STORAGE_CLOSE_ANGLE_DEG` 仍是模块常量，但仅作"参考值"而非"硬限"
- `api.py:78-83`: 加 `STORAGE_OPEN_ANGLE_DEG=75` / `STORAGE_CLOSE_ANGLE_DEG=98` 参考值注释
- `api.py:set_storage_angle()`: 删枚举校验块；保留 auto-move (Round 11)
- `ARM_API.md §6.1`: 重写为"任意角度通过"
- mock 验证 15/15 PASS：-90/0/30/50/60/70/80/90/98/100/120/150/165/200 都过；只有 75° 触发 auto-move
- **用户现在可以**：
  - 现场实测找最佳 OPEN 物理位（试 30°/45°/60°/75°/90° 等）
  - 试不同 CLOSE 物理位（试 95°/98°/100°/120° 等）
  - 测完后告诉我新的 OPEN/CLOSE 角度，再决定是否重新加硬限

### Round 12 follow-up: **清理测试脚本里的硬限残留（user 反馈"没删除彻底"）**
- **触发原因（user 反馈）**：跑 `test_storage_close.py` 输出 `[OK] succeeded` 但脚本注释仍写 "业务硬限 PASS-THROUGH" / "业务硬限上界"
- **问题**：Round 12 只清了 api.py 和 ARM_API.md，没清测试脚本里的"硬限"叙述文字
- **清理内容**：
  - `test_storage_open.py`：
    - L15: `STORAGE_OPEN_ANGLE_DEG = 75  # 75° = open 物理位（业务硬限下界）` → `# 75° = open 物理位（Round 12 后仅参考值, 任意角度都过）`
    - L17-19: 删 `业务硬限下界 PASS-THROUGH` / `业务硬限验证走 test_storage_limit.py` 引用
    - L52: 删 `98° 在业务硬限 [75, 98]° 内, PASS-THROUGH, 走 HTTP 下发舵机`
    - L54: 删 `[业务 wrapper, 业务硬限 PASS-THROUGH]`，改成 `[业务 wrapper, Round 11 auto-move 会自动抬 y 到 -150]`
    - L66-69: 删 `业务硬限拦了 — 75° 在范围内理论不会触发,但脚本应当报告而非静默吞` 这段无效 except 分支
    - L68: 删 `[FAIL] 业务硬限意外 raise (75° 应在 [75, 98]° 内 PASS-THROUGH)` → `[FAIL] 业务层 raise:`
  - `test_storage_close.py`：
    - L15: `STORAGE_CLOSE_ANGLE_DEG = 98  # 98° = close 物理位（业务硬限上界）` → `# 98° = close 物理位（Round 12 后仅参考值, 任意角度都过）`
    - L17-21: 删 `业务硬限上界 PASS-THROUGH` / `业务硬限验证走 test_storage_limit.py` / `100° 跑边界压力测试` 叙述
    - L54: 删 `98° 在业务硬限 [75, 98]° 内, PASS-THROUGH, 走 HTTP 下发舵机`
    - L56: 删 `[业务 wrapper, 业务硬限 PASS-THROUGH]`，改成 `[业务 wrapper, 98° 不走 auto-move]`
    - L68-70: 删 `业务硬限意外 raise (98° 应在 [75, 98]° 内 PASS-THROUGH)` → `业务层意外 raise:`
- **保留**：4 处历史叙述性"硬限"注释（解释 Round 12 为什么删）—— 这是有意保留的可读性，不是 active 描述
- **没动**：脚本的 auto-move 兼容性描述（Round 11 auto-move 仍生效，75° 仍触发）

## 🎯 当前最终态（Round 12 后）

| 入口 | 行为 |
|---|---|
| `set_storage_angle(75°)` | ✅ auto-move y 到 -150 → 发 75° 舵机 |
| `set_storage_angle(<其他角度>)` | ✅ 直接发舵机（任意角度，不拦） |
| `set_storage_angle(98°)` | ✅ 直接发 98°（不走 auto-move） |
| `set_storage(side="RIGHT"/"LEFT")` | ✅ bool 抽象，底层 165°/-42° |
| `pre_init_close_storage()` | ✅ init 阶段固定 98° |
| `move_y()` / `move_xy()` / `set_pose()` / `reset_y()` | ✅ 完全自由，业务层一行没动 |

---

## ⚠️ 面临的问题（当前已知）

### 已通过设计接受的限制
1. **HTTP 直调绕过业务 wrapper**：`RuntimeApiClient.execute_car_action("set_storage_angle", angle=...)` 不走业务层校验
   - 设计原因：保留逃生口供测试 / 现场调参
   - 现状：用户明确同意保留（多次确认）
2. **set_storage("RIGHT") bool 抽象不拦**：底层映射到 RIGHT=165°（超 98° 上界）
   - 设计原因：bool 抽象 vs 角度枚举是两套语义，业务层不参与
   - 现状：保持不变
3. **pre_init_close_storage() 模块函数不走 wrapper**：
   - 设计原因：init 阶段还没建 ArmClient 实例，避免循环依赖
   - 现状：98° 仍能下发，但只在 init 阶段

### 想做但被硬约束挡住的
- **改底层 `car.set_storage_angle()` 加 raise**：会全路径锁死，但违反 [[arm-business-layer-only]]，需要明确授权
  - 现状：**用户多次确认保留逃生口**，未授权

### 当前没有未解决问题
- 业务硬限、三个测试脚本、文档同步都完成
- 用户没给新的待办

---

## 🔒 注意事项（硬约束，必读）

### 来自 [[arm-business-layer-only]]
**只能改 `main/**` 业务层**。以下**不能动**：
- ❌ `smartcar/whalesbot/**`（SDK 底层，包括 `arm_base.py`、`arm_cfg.yaml`）
- ❌ `runtime/**`（FastAPI 服务层）
- ❌ `car_wrap_2026.py`、`car_start_2026.py`、`car_task_function.py`（legacy monolith）

如要动以上文件 → **必须用户明确授权**才能动。

### 测试脚本约定
- `main/arm/test/` 下脚本是**离线硬件冒烟脚本**，按 CLAUDE.md 描述**非正式测试**
- 大部分 `main/arm/test/*.py` 是 gitignored（本地冒烟，不进版本控制）
- 但我们 session 改的 `test_storage_open.py` / `test_storage_close.py` / `test_storage_limit.py` **不是 gitignored**——这次是正式进入版本控制的

### 业务硬限 vs 物理保护
- 业务硬限 [75, 98]° 在 `ArmClient` 层 → 拦**业务代码推荐入口**
- 物理保护在 `mc602` 协议层 / 舵机本身 → 拦所有路径
- 我们加的是业务层（软保护），不是物理保护

### set_storage_angle 与 set_storage(side) 的关系（Round 12 最终态）
| 入口 | 角度枚举硬限 | auto-move y | y gate | 底层映射 |
|---|---|---|---|---|
| `set_storage_angle(75° OPEN)` | ❌ **Round 12 删** | ✅ **自动 move_y(-0.150)**（Round 11 保留） | ❌ 不参与 | 透传 75° |
| `set_storage_angle(98° CLOSE)` | ❌ **Round 12 删** | ❌ 不触发 | ❌ 不参与 | 透传 98° |
| `set_storage_angle(<任意其他角度>)` | ❌ **Round 12 删** | ❌ 不触发（只在 75° 触发） | ❌ 不参与 | 透传该角度 |
| `set_storage(side)` | ❌ 不参与 | ❌ 不参与 | ❌ 不参与 | bool → [-42°, 165°] |
| `pre_init_close_storage()` | ❌ 不参与 | ❌ 不参与 | ❌ 不参与 | 98° 直调底层 |
| HTTP `/v1/execute` | ❌ 不参与 | ❌ 不参与 | ❌ 不参与 | 透传（保留逃生口） |

**Round 12 最终行为**：
- `set_storage_angle` 任意整数角度都通过
- `set_storage_angle(75°)` 自动抬 y 到 -150 后下发舵机
- 其他角度直接下发，不动 y
- 其他 y 轴方法（`move_y` / `move_xy` / `set_pose` / `reset_y` / `reset_position`）业务层一行没动，完全自由

---

## 🧪 快速验证命令

### Mock 验证（不需要 runtime，零硬件风险）
```bash
cd C:/Users/29368/Desktop/智能车/rak-car
PYTHONIOENCODING=utf-8 python -c "
from main.arm.api import ArmClient

class FakeHttp:
    def execute_arm_action(self, name, **kwargs):
        return {'result': 0.0}
    def execute_car_action(self, name, **kwargs):
        return {'status': 'succeeded', 'result': {'side': 'RIGHT', 'angle': 75}}

def run(angle):
    c = ArmClient(http=FakeHttp())
    try:
        c.set_storage_angle(angle, speed=10, timeout=5.0)
        return 'PASS'
    except ValueError as e:
        return f'BLOCKED - {str(e).split(chr(10))[0][:50]}'

for a in [50, 74, 75, 76, 80, 90, 97, 98, 99]:
    print(f'  {a:>3}°: {run(a)}')
"
```

预期：
```
   50°: BLOCKED - set_storage_angle(50) 不在业务硬限枚举内。
   74°: BLOCKED
   75°: PASS
   76°: BLOCKED    ← 关键：之前区间 PASS，现在枚举 raise
   80°: BLOCKED
   90°: BLOCKED
   97°: BLOCKED
   98°: PASS
   99°: BLOCKED
```

### 硬件测试（需要 runtime 在线）
```powershell
# 1. 物理位 open (75°)
& D:/python/python.exe c:/Users/29368/Desktop/智能车/rak-car/main/arm/test/test_storage_open.py

# 2. 物理位 close (98°)
& D:/python/python.exe c:/Users/29368/Desktop/智能车/rak-car/main/arm/test/test_storage_close.py

# 3. 业务硬限验证 (reject-only, 零硬件风险)
& D:/python/python.exe c:/Users/29368/Desktop/智能车/rak-car/main/arm/test/test_storage_limit.py --reject-only

# 4. 业务硬限 + 边界 accept 全跑（会下发 75° 和 98°）
& D:/python/python.exe c:/Users/29368/Desktop/智能车/rak-car/main/arm/test/test_storage_limit.py

# 5. y gate + auto-move 完整验证 (Round 8+9)
& D:/python/python.exe c:/Users/29368/Desktop/智能车/rak-car/main/arm/test/test_storage_open_y_gate.py

# 6. auto-move 验证 (Round 9) - 3 起点 case (y=0/-30/-175) 自动抬到 -150 后开仓
& D:/python/python.exe c:/Users/29368/Desktop/智能车/rak-car/main/arm/test/test_storage_open_auto.py --mock
& D:/python/python.exe c:/Users/29368/Desktop/智能车/rak-car/main/arm/test/test_storage_open_auto.py  # 硬件实跑
```

---

## 📝 待办（用户给的指示，目前为空）

无明确待办。等用户下一步指示（用户正在硬件现场测角度）。

**Round 12 最终状态**：
- ✅ Round 11 auto-move 保留（仅 75° 触发）
- ✅ pre_init_close_storage 保留（init 阶段 98°）
- ❌ Round 7 角度枚举硬限已删（任意角度通过）
- ❌ Round 8 y gate 已撤（Round 10 删的）

**用户现场测角度建议**：
- 起点：从 `arm.move_y(-150)` 把 y 抬到 -150（手动，安全）
- 测试 OPEN：试 `arm.set_storage_angle(60/70/75/80/90)` 看哪个角度舵机真正打开
- 测试 CLOSE：试 `arm.set_storage_angle(95/98/100/120)` 看哪个角度真正关上
- 测完告诉我新角度，我可以加回角度枚举硬限（用新值）

如用户想：
- 重新加角度枚举硬限 → 用测出的新值改 `STORAGE_OPEN_ANGLE_DEG` / `STORAGE_CLOSE_ANGLE_DEG`
- 重新加 y gate → 重新做 Round 8
- 加其他限制（如 close y 区间 / 开仓 x 区间）→ 改 `api.py` + `ARM_API.md`

---

## 📂 改动文件位置速查

| 文件 | 关键行 | 改动 |
|---|---|---|
| `main/arm/api.py` | L53-83 | 常量块（OPEN_ANGLE/CLOSE_ANGLE + OPEN_AUTO_Y_MM/TIMEOUT）—— **Round 12 角度枚举改参考值** |
| `main/arm/api.py` | L86-121 | `pre_init_close_storage()` 函数 |
| `main/arm/api.py` | L443-503 | `_ensure_y_for_open_storage()` 方法（Round 11 保留） |
| `main/arm/api.py` | L708-738 | `ArmClient.set_storage_angle()` 业务 wrapper（**Round 12 删枚举校验**, 仅留 auto-move） |
| `main/arm/loops/runner.py` | L154-163 | `ArmRunner.set_storage_angle()` 委托 |
| `main/arm/ARM_API.md` | L473-540 | §6.1 (Round 12 重写: 任意角度通过) + §6.2 (escape hatch) + §6.3 (Round 11 auto-move) |
| `main/arm/test/test_storage_open.py` | L14, L51-58 | 重命名 + 走业务 wrapper |
| `main/arm/test/test_storage_close.py` | L14, L53-60 | 重命名 + 走业务 wrapper (98°) |
| ~~`test_storage_limit.py`~~ | — | **Round 12 用户自行删除**（硬限没了脚本无意义） |
| ~~`test_storage_open_y_gate.py`~~ | — | **Round 10 用户自行删除** |
| ~~`test_storage_open_auto.py`~~ | — | **Round 10 用户自行删除** |

---

## 🔗 相关 commit / memory 写入建议

本次 session 改的代码**已落盘**但**未 commit**。下次回来如果要 commit：
- 推荐 commit message 模板：`feat(arm/storage): 业务层枚举硬限 [75° OPEN / 98° CLOSE] (2026-07-18)`
- 不要 commit 的：`main/arm/test/*_storage_*.py` 历史 gitignored 状态（这次破例提交）

更新 memory 时考虑：
- 更新 [[arm-api-reference]] 的 §6 描述（从区间改枚举）
- 更新 [[x-axis-rollout-session]] 的会话记录（如有需要）

---

**Session 时间**：2026-07-18 整天
**改动量**：6 个文件（api.py + runner.py + ARM_API.md + 3 个测试脚本）
**回滚点**：git HEAD（未 commit，可手动 revert）