# task4 上下位机构建阶段汇总（pick_up_blue 会话）

> 日期：2026-07-25 ～ 2026-07-27
> 任务：边采边存方案的具体实施 + Eye-in-Hand 思路落地
> 目的：会话上下文压缩 + 下次续接指引

---

## 一、上下文文档入口（先看这些）

### 1. CLAUDE.md（项目约定）
- 项目三层架构：runtime / smartcar / main。
- **当前业务只能改 `main/**`，底层 / runtime / smartcar / car_wrap_2026.py / car_task_function.py 都不能动**。
- 主要入口：
  - `runtime/server`（生产）
  - `main/quick_start.py`、`main/car_start_api.py`（业务脚本）
- 子包约定：
  - `main/arm/`（机械臂业务）
  - `main/chassis/`（底盘业务）
  - `main/misc/`（单文件小任务）

### 2. main/arm/ARM_API.md（机械臂业务核心）
- §1.1 大臂/手爪角度范围与默认值。
- §1.5/§1.8 业务硬限位、Y 保护区。
- §6.3 存储仓 75° / 98° / y gate `[-205, -145]`。
- §7.2.x 轴同步带打滑；§7.2.1 belt-slip 分段策略。
- §9 reset_x escape hatch（直调底层）。
- §11 **x_get_position 走坏 calibrate 框架，必须走 `_read_x_mm_realtime()` 读 x 真值**。
- §10 x_speed_with_safety watchdog。

### 3. main/arm/each_task/task4/__init__.py（task4 业务约定）
- 边采边存流程图：a_approach → b1_detect_fruit → b2_pick_fruit → b3_store_fruit → c_finish。
- 蓝 bin=`x=0`，黄 bin=`x=-65`。
- 球场几何：左手边=放球，x 负=向左，大臂 0°=平行 x。
- 关联辅助文件（target1、target2、test_blue、test_yellow、x_to_zero、open_storage）。

### 4. main/arm/each_task/task4/constants.py（任务常量）
- `SAFE_Y_TRANSIT_MM=-190`、`BIN_OPEN_Y_MM=-150`、`PICK_Y_MM=-160`（**待现场校准**）。
- `STORAGE_OPEN_ANGLE_DEG=75`、`STORAGE_CLOSE_ANGLE_DEG=98`、`STORAGE_OPEN_SPEED=5`。
- `MOVE_X_V_MAX_MMS=40`、`GRASP_HOLD_S=1.0`、`GRASP_TIMEOUT_S=10.0`。
- `BLUE_BIN_X_MM=0`、`YELLOW_BIN_X_MM=-65`。
- `X_SPEED_SAFETY_V_FALLBACK_MMS=30`、`X_SPEED_SAFETY_STALE_S=2`。
- `DEFAULT_MAX_ROUNDS=8`、颜色 `blue/yellow/unknown`。

### 5. 历史会话文档（MEMORY.md 索引）
- `task4-rebuild-2026-07-22` / `task5-rebuild-2026-07-22` / `debug-x-axis-rollout-v3` / `debug-task5-rebuild-2026-07-22` / `debug-storage-hardlimit-session-2026-07-18`。
- 关键约束：
  - `arm-business-layer-only`：业务层只能改 `main/**`。
  - `x-get-position-vs-realtime`：realtime 才是 x 真值。
  - `x-axis-belt-slip`：x 单次有效行程 24–46mm。
  - `x-speed-safety-watchdog`：safety watchdog 防带打滑空转。

### 6. PPT
- `百度智慧交通创意组21届-任务解析与技术流程(1).pptx` 第 9 页：任务4 = 作物采收。
  - 收割区 2 色 4cm 球，蓝/黄。
  - **果实完全脱离任务模型且不与场地接触 → +10 分/球**。

---

## 二、目前任务4的实现状态

### 既有脚本（不动）
- `a_approach.py`、`b1_detect_fruit.py`、`b2_pick_fruit.py`、`b3_store_fruit.py`、`c_finish.py`、`run_full.py`：六阶段主流程（git 状态显示这些被删了，外部清空过；新结构改用单文件脚本）。
- `open_storage.py`：单职责开仓模块（y gate 预检 + auto_move 容错）。
- `test_blue.py` / `test_yellow.py`：入仓位姿子工具；`test_blue.py` 已删除最后一步 set_storage_angle(75°)，开仓改走 `open_storage`。
- `x_to_zero.py`：reset_x 撞墙回 0。
- `grasp.py`：真空泵冒烟脚本。
- `target2.py`：侧摄目标识别（已完整工作）。

### 本次新建 / 修改的脚本
1. **`target3.py`**（新建）：吸气下降 → 抬回 → 默认放气，可通过 `release_after_return=False` 保持吸气。
2. **`pick_up_blue.py`**（新建）：依次 `target1 → target3(保持吸气) → test_blue → grasp(False) 放气`。

### 当前 pick_up_blue 流程
```text
1. target1.step_target1()
   - move_y(-200) → arm=0° → hand=0°（底层直调绕拦截）→ x 分段到 -260
2. target3.step_target3(release_after_return=False)
   - grasp(True)
   - move_y(-80)   下降，保持吸气
   - move_y(-200)  抬回，保持吸气
3. test_blue(client, runner)
   - move_y(-190) → hand=0°（底层直调）→ arm=0° → reset_x 撞墙定 x=0 → move_y(-155)
4. runner.grasp(False)
   - 放气
```

### 上一轮（target2.py）的实测数据
当黄色球正好位于吸盘正下方时，runtime 返回：
```text
color=yellow, cx=+0.087, cy=-0.663, w=0.375, h=0.538, score=0.929
```
这就是第一组 **吸盘命中示教像素点**：
- `GRASP_AIM_CX = 0.087`
- `GRASP_AIM_CY = -0.663`
- 归一化坐标范围比文档中的 `[-0.5, 0.5]` 大，实际约 `[-1, 1]`（实测 `cy=-0.663`）。

---

## 三、本次会话的改动

| 文件 | 操作 | 主要内容 |
|---|---|---|
| `main/arm/each_task/task4/target3.py` | 新建 | 吸气下降→抬回；新增 `release_after_return` 参数 |
| `main/arm/each_task/task4/pick_up_blue.py` | 新建 | target1 → target3(保持吸气) → test_blue → 放气 |
| `target3.py` | 改 -90 → -80 | 下降高度改为 `-80 mm` |
| `target2.py` | 仅审阅，未改动 | 文档/常量和实际值不一致（TARGET_ASPECT_TOL/TARGET_AREA_MAX） |
| `target1.py` | 仅审阅，未改动 | `x=-260` 超物理墙、可能撞墙后停在 `-119.5` |
| `test_blue.py` | 仅调用，未改动 | 内部仍走底层 `_call_arm("set_hand_angle")` 绕拦截 |

---

## 四、面临的问题

### 1. 方向未标定（最关键）
- 现在知道“球在吸盘下方时 `cx=0.087`”，但不知道：
  - `cx > 0.087` 时，机械臂 x 应该往正还是负方向运动。
  - `cx < 0.087` 时同理。
- 这正是 `pick_up_blue` 只能按固定 target1 位姿执行、不能根据检测结果动态对齐的原因。
- 解决方式：下一步用 Eye-in-Hand 小步视觉闭环（`target4.py`）做现场标定。

### 2. target1.py 的 x 越界
- 目标 `x=-260 mm`，实测物理墙约 `-119.5 mm`。
- 当前靠 stall 检测兜底，仍返回 `ok=True`，不安全。
- 业务层不能改 runtime / smartcar，仅能：
  - 调小 `target1.py` 的 `TARGET1_X_MM`；
  - 或在调用前/后加软件限位检查。

### 3. target3.py 与常量不一致
- `PICK_Y_MM=-80`（脚本当前值），与 `constants.py` 中 `PICK_Y_MM=-160` 不一致。
- `constants.py` 写明 `-160` 是“球心对应 y（4cm 球，任务模型顶面），待现场校准”。
- 现在的 `-80` 是用户临时决定；需要决定是同步改 constants.py 还是保留 `-160` 作为 long-term 值。

### 4. grasp() 路径未被 `target3.py` 自身保证
- 当前 `target3.py` 默认会在抬回 `-200 mm` 后放气。
- 只有当调用方传 `release_after_return=False` 时才保持吸气（由调用方负责放气）。
- 没有“吸气状态机”或跟踪变量，依赖调用方不漏放气，存在丢球风险。

### 5. test_blue.py 内的动作链没有吸气保护
- `test_blue.py` 步骤 5 调用 `reset_x` 撞墙，动作过程中真空泵已经打开。
- 撞墙时臂杆会有抖动，存在球被震落的可能。
- 后续如果要加固，需要在 `test_blue.py` 前/后确认机械臂稳定性，或在 pickup 顺序中插入稳定步骤。

### 6. 单文件脚本可能仍被外部清空
- MEMORY.md 提到 `task5` 目录曾被外部动作清空，自包含是为了避免被牵连。
- 本次新建的 `target3.py` / `pick_up_blue.py` 都是自包含的，仅 import `main.arm` 与 task4 包内文件。

### 7. runtime vision 接口格式与文档不一致
- `target2.py` 顶部写的 `cx_norm/cy_norm` 范围 `[-0.5, 0.5]` 错误，实测接近 `[-1, 1]`。
- 类别兜底映射 `CLS_ID_TO_COLOR = {0: blue, 1: yellow}` 与实测 `{16: ball_blue, 17: ball_yellow}` 冲突。
- 当前 `target2.py` 实际是按 label 字符串匹配，不影响本次流程，但下一次加入视觉闭环时必须先校正。

---

## 五、准备解决的问题（按优先级）

| 优先级 | 目标 | 思路 |
|---:|---|---|
| P0 | 校正 `target2.py` 类映射与坐标范围 | 改 `CLS_ID_TO_COLOR` 加 `16/17`；文档更新 `cx/cy` 范围为 `[-1, 1]` |
| P0 | 现场标定“图像误差→x 方向”符号 | 单独跑一次 probe：x 移动 +5mm → 看 `cx_norm` 变化；确定 sign |
| P0 | target1.py 加软件 x 软限位 | 在 `step_target1` 开头校验 `x_mm >= -115`，超限直接 raise；不再依赖 stall 兜底 |
| P1 | 新增 `target4.py` Eye-in-Hand 小步闭环 | 默认 dry-run；显式 `--execute --x-sign ±1` 才动硬件；不调用 grasp/move_y/舵机 |
| P1 | pick_up_blue 加入视觉对准阶段 | `target2 → target4` 把球移动到 `cx≈0.087`；再走 `target3 → test_blue` |
| P2 | pick_up_yellow 同步实现 | 与 pick_up_blue 结构对称；`test_yellow` 入仓位姿 `x=-65` |
| P2 | 多球循环（run_full_v2） | 把 a/b1/b2/b3 循环为：detect → align → pick → store；n 个球 |
| P2 | grasp 状态机 | 由 pick_up_xx 显式持有 `pump_on` 状态；任何异常路径强制 `grasp(False)` |

---

## 六、注意事项（硬约束）

### 业务层硬约束
1. **只能改 `main/**`**：`main/arm/`、`main/chassis/`、`main/misc/`、`main/test/` 允许修改。
2. **不可改**：`smartcar/whalesbot/**`、`runtime/**`、`car_wrap_2026.py`、`car_start_2026.py`、`car_task_function.py`、`config_car.yml`、`ecosystem.config.js`。
3. **runtime 修不了**：实时检测的 30Hz task_feed、20Hz arm_feed 是由 runtime 维护的；业务只能读。
4. **x 位置只能走 realtime**：禁止使用 `get_state().x_mm` 做位置判断；统一 `client._read_x_mm_realtime()`。

### 安全约束
5. **y 保护区 `[0, -30]` mm** 禁止动舵机/手爪（init 位例外）。
6. **开仓 y gate `[-205, -145]`**：open_storage.py 已做预检。
7. **x 软限位**：业务层不要再依赖撞墙/stall；主动设置软件边界（例如 `x >= -115 mm`）。
8. **belt-slip**：x 单次行程 ≤ 46mm，远距离必须分段。
9. **同色过滤**：target2 拿到的 cls_id 16/17 不要硬写 0/1；要保留 label 优先。

### 文件约定
10. **自包含**：task4 / task5 的单文件脚本尽量只 import `main.arm` 与自身包内文件，避免被外部清空牵连。
11. **CLI 默认 dry-run**：第一次跑某段新流程，先 dry-run 看打印，不真正动硬件。
12. **多帧稳定**：视觉闭环必须连续 ≥ 3 帧满足阈值才算对准；单帧不可信。
13. **常量单点真相**：`constants.py` 是 task4 业务常量唯一来源；不要在脚本里再硬编。
14. **grasp 责任归属**：每个使用吸盘的脚本必须自己决定何时 `grasp(False)`，没有全局状态机。

### 文档约定
15. **docstring / 注释用中文**：与现有 task4 脚本风格一致。
16. **每个脚本顶部写明**：调用顺序、依赖、已知风险、回退方式。
17. **改动必须写日志**：在 `debug-*.md` 或会话汇总文档里记录原因。

---

## 七、下次会话优先动作

1. **读本文件 + `debug-x-axis-rollout-v3.md` + `debug-task5-rebuild-2026-07-22.md`**。
2. 把 `target2.py` 的 `CLS_ID_TO_COLOR` 修正为 `{16: blue, 17: yellow}`；文档范围改为 `[-1, 1]`。
3. 在球固定于吸盘下方时，连续 `--loop --hz 5 --duration 5` 跑一次 `target2.py`，记录中位数作为正式示教点。
4. 用一次小幅 x probe 确定 sign：x 移动 +5mm（远离墙方向）→ 记录 `cx_norm` 变化 → 写死 sign。
5. 在 `target1.py` 加软件 x 软限位，越界直接 raise。
6. 开始实现 `target4.py`（Eye-in-Hand 小步闭环），默认 dry-run，只动 x 轴。
7. 验证完整链：`target2 → target4 → target3 → test_blue`，保证球从进入视野到入仓之间始终保持吸气。

---

## 八、当前会话未决问题

- `constants.py` 的 `PICK_Y_MM=-160` 与 `target3.py` 的 `-80` 应统一哪一个？倾向：
  - 保留 `constants.py = -160`（球心对应高度）；
  - 在 `target3.py` 加 `-80` 作为“当前临时下降位置”注释；现场校准后改回 `-160`。
- `test_blue.py` 步骤 5 的 `reset_x` 撞墙动作期间是否安全？需要现场观察是否掉球。
- `target3.py` 的“保持吸气”参数化是否扩展到所有“拾取-放置”脚本？目前只有 `pick_up_blue.py` 在用。

---

## 九、关键文件路径速查

```
main/arm/each_task/task4/
├── __init__.py           任务说明 + 流程图
├── constants.py          业务常量（单点真相）
├── target1.py            摆到 target1 位姿 (⚠️ x 超墙)
├── target2.py            侧摄识别球类 (label 优先, cls_id 兜底)
├── target3.py            吸气下降/抬回 (NEW, release_after_return 可控)
├── test_blue.py          蓝色 bin 位姿 (x=0 撞墙定原点, y=-155)
├── test_yellow.py        黄色 bin 位姿 (x=-65 belt-slip)
├── x_to_zero.py          reset_x 撞墙回 0
├── open_storage.py       单职责开仓模块 (y gate 预检)
├── grasp.py              真空泵冒烟
├── pick_up_blue.py       target1→target3(保持吸气)→test_blue→放气 (NEW)
└── pick_up_yellow.py     待实现
```

```
main/arm/
├── api.py                ArmClient (move_x / move_y / grasp / set_storage)
├── state.py              ArmState / ArmOrigin
├── origin.py             OriginCalibrator
├── trajectory.py         TrajectoryGenerator
└── loops/runner.py       ArmRunner (move_x verify, 默认 get_state 校验)
```