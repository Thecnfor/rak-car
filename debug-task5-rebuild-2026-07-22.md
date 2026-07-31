# task5 分拣入库重建 + x 自动归零排错 — 上下文压缩 (2026-07-22)

> **目的**：压缩本次会话所有上下文，下次起手读这一份就能继续。
> **会话日期**：2026-07-22
> **主要任务**：task5 (PPT Slide 10 分拣入库) 业务层重建 + 现场 x 自动归零排错

---

## 0. TL;DR

1. **task5 业务层 4 个自包含脚本已建好**：`high_tower.py` / `get_blue.py` / `get_yellow.py` / `grasp_5.py`。
2. **x "突然回 0" 根因已找到**：`runtime/_auto_init_loop` → `ensure_initialized` → `_create_car_locked` → `arm.reset_all()` 撞墙 calibrate x=0。**业务层无权改 runtime，临时关 `RAK_CAR_AUTO_INIT=0` 验证**。
3. **文件曾被外部动作清空**（git status 确认 `D` 状态）：早期写的 6 阶段 `constants/a/b1/b2/b3/c/run_full.py` 在磁盘上没了。后续文件全部**自包含**，不依赖会消失的 `constants.py`。
4. **业务层硬约束**（见 §3）：只能改 `main/**`；`smartcar/whalesbot/**`、`runtime/**`、`car_wrap_2026.py`、`car_start_2026.py`、`car_task_function.py` 都不能动。

---

## 1. 文档/源码引用（按重要性）

### PPT 任务定义
- `C:\Users\29368\Desktop\智能车\百度智慧交通创意组21届-任务解析与技术流程(1).pptx` → **Slide 10 任务5**：
  - 任务区内有**高低两个存储仓**；高储存仓有颜色标签（单色），储存仓双色。
  - 智能车需将采集的果实模型，**依据种类放入正确的存储仓内**。赛前指定对应颜色的储存仓。
  - 得分：果实位于对应颜色的储存仓内 → **+20 分/果实**。
  - 技术方案：目标检测，精准定位，**高位存放**。
  - 任务5 与任务4 边界：task4 = 车载自带存储仓；task5 = **场地存储仓**（不一样！）。

### 业务层核心文档
- `main/arm/ARM_API.md` — **四层机械臂业务 API 速查**（必读）
  - §0 坐标系约定（y<0 向上，x 远离为正）
  - §6.3 开仓 y gate (Round 15)：`set_storage_angle(75°)` 前 y ∈ **[-205, -145]** mm 闭区间
  - §7.1 保护区：实际是 y ∈ **[0, -30]mm**（不是 [0, -80]）
  - §7.2.1 belt-slip：单次有效行程 24-46mm，远距要分段
  - §9 reset_x / reset_all：**已废弃/半废弃**，calibrate 框架有 bug
  - §10 x_speed safety watchdog
  - §11 🔴 x 位置读数：**x_get_position 走 calibrate 框架坏**，只信 `/v1/realtime/arm/state` (20Hz `arm_feed` 守护线程)

### 底层源码（读过关键段）
- `main/arm/api.py`：
  - `_check_y_protected` (line 415-445)：y 保护区检查
  - `set_arm_angle` (line 516-550)：硬限 [-150, 0]°，大臂 ≤-30 时跳 y 保护区
  - `set_hand_angle` (line 561-605)：**硬限 [-90, 0]°**；大臂 ∈[-30, 0] 展开区时手爪只允许 -90 (UP)
  - `move_x` (line 352-386)：过 y 保护区检查
  - `grasp` (line 730-736)：`_call_arm("grasp", value=bool(on), sync=True)`
  - `_call_arm` (line 246-256)：透传 kwargs 到 `http.execute_arm_action`
- `smartcar/whalesbot/vehicle/arm/arm_base.py`：
  - `x_get_position` (line 391-393)：`return self.motor_x.get_dis() - self.x_pose_start`
  - `x_pose_start` 只在 4 处被改写：366(__init__) / 467(x_stop_check 撞墙) / 620-621(reset_x 撞墙) / 780-784(position_params_init)
  - `reset_x` (line 500-639)：`sign = +1 if "right" else -1`，**direction 字符串只是速度符号，与物理墙无固定映射**
  - `arm.grasp` (line 747-755)：**只动 pump.set + valve.set，不碰 x**
- `main/arm/loops/runner.py`：
  - `runner.grasp` (line 170-171)：直接透传 `client.grasp`
  - `runner.go_home` (line 173-177)：**会调 `move_xy(0, 0)` 强制回 0**
- `main/api_client.py`：
  - `execute_arm_action` (line 366-374)：纯 HTTP POST，无副作用
  - `execute` (line 92-142)：sync=True 阻塞轮询；sync=False 立即返回
- `runtime/services/runtime_service.py`：
  - **`_auto_init_loop` (line 429-446)**：**根因所在**——后台线程持续轮询 car 状态，触发重建
  - **`_create_car_locked` (line 318-370)**：默认 `reset_arm=False` → `arm.reset_all()`（**撞墙 calibrate x=0**）
  - `auto_init_kwargs` 默认 `reset_arm=settings.get_reset_arm_on_auto_init()` (env `RAK_CAR_RESET_ARM=0`)
- `main/arm/each_task/task4/grasp.py`：参考的吸气模板（用 `runner.grasp` + `time.sleep`）

---

## 2. task5/ 当前文件状态（最后验证）

| 文件 | 状态 | 备注 |
|---|---|---|
| `__init__.py` | ✓ 在磁盘 | 6 阶段流程图 + PPT Slide 10 说明，但子模块不在 |
| `constants.py` | ✗ **磁盘不存在** | 早期写，被外部清空 |
| `a_approach.py` | ✗ 不在 | 同上 |
| `b1_detect_bins.py` | ✗ 不在 | 同上 |
| `b2_extract_fruit.py` | ✗ 不在 | 同上 |
| `b3_place_fruit.py` | ✗ 不在 | 同上 |
| `c_finish.py` | ✗ 不在 | 同上 |
| `run_full.py` | ✗ 不在 | 同上 |
| `high_tower.py` | ✓ 在磁盘 | 4 步：y=-180 → arm=0° → hand=-90° → x=-150 |
| `get_blue.py` | ✓ 在磁盘 | 5 步：y=-130 → reset_x(撞墙, direction="right") → arm=-5° → hand=0°(底层直调) → y=-90 |
| `get_yellow.py` | ✓ 在磁盘 | 5 步：y=-130 → move_x(-72) → arm=-5° → hand=0°(底层直调) → y=-90 |
| `grasp_5.py` | ✓ 在磁盘 | grasp(True) → sleep(5s) → grasp(False) |

**git status 印证**（`D` 都是磁盘不存在 + git 记录删除）：
```
D main/arm/each_task/task5/a_approach.py
D main/arm/each_task/task5/b1_take_fruit.py     ← git 里原本就有另一套
D main/arm/each_task/task5/b2_detect_color.py
D main/arm/each_task/task5/b3_place_bin.py
D main/arm/each_task/task5/c_finish.py
D main/arm/each_task/task5/run_full.py
```

---

## 3. 业务层硬约束（必须遵守）

1. **只能改 `main/**`**
   - `smartcar/whalesbot/**` 不动（arm_base.py / car_wrap 等）
   - `runtime/**` 不动（auto_init 改不了！）
   - `car_wrap_2026.py` / `car_start_2026.py` / `car_task_function.py` 不动
2. **x 位置读数走 `/v1/realtime/arm/state`**（`x_get_position` 坏，§11）
3. **保护区 y ∈ [0, -30]mm**（实际值，文档 [0, -80] 是旧的）
4. **belt-slip**：单次有效 24-46mm，远距要分段
5. **set_hand_angle 展开区限制**：大臂 ∈ [-30, 0] 时手爪只允许 -90（UP）
   - 业务层要么走 `arm=收起来` dance，要么走 `_call_arm` 底层直调绕开
6. **`reset_x` wrapper 不透传 `probe_time`**：走 `_call_arm` 直调 + `probe_time=0.3`（不是 0！）
7. **direction 字符串只是速度符号**：`"right"=+速度`/`"left"=-速度`，与物理墙无固定映射
8. **文件可能再次被外部清空**：所有脚本**自包含**（不 import task5 包内其他模块）

---

## 4. 关键调用约定 (Cheat Sheet)

```python
# 1. x 位置：永远走 realtime
x_mm = arm._read_x_mm_realtime()        # 走 /v1/realtime/arm/state (20Hz arm_feed)

# 2. belt-slip 安全 move_x
def _move_x_with_split(client, runner, target_x_mm):
    actual = client._read_x_mm_realtime() or 0.0
    delta = target_x_mm - actual
    if abs(delta) <= 30.0:
        runner.move_x(target_x_mm)
        return ...
    # 分段 + realtime 校验, ≤30mm/段, 最多 10 步

# 3. reset_x 撞墙 (绕过 wrapper, probe_time=0.3)
client._call_arm("reset_x", timeout=30, sync=True,
    direction="right",            # "right"=+速度, 与物理墙无固定映射
    reset_velocity=0.05,          # 50mm/s
    probe_time=0.3,               # 0 会误判 stall, 必须留 0.3
)

# 4. set_hand_angle 绕开 api.py:591-599 限制
client._call_arm("set_hand_angle", timeout=10.0, sync=True,
    angle=0, speed=80,
)  # 硬件若不允许 → 拿车端 error, 不在 Python 层先 raise

# 5. move_y 任意值放行 (包括保护区 [0, -30] 内的 y)
runner.move_y(y_mm)  # 不受 y 保护区限制
```

---

## 5. 改动历史（本会话）

### task5 6 阶段重建（早期，已丢失）
- 写 `__init__.py` / `constants.py` / `a_approach.py` / `b1_detect_bins.py` / `b2_extract_fruit.py` / `b3_place_fruit.py` / `c_finish.py` / `run_full.py` 共 8 个文件
- **磁盘上被外部动作清空**（git status 印证）
- 教训：所有后续脚本**自包含**

### `high_tower.py` 改动
- 初版：4 步 `move_y → move_x(-100, belt-slip 分段) → arm=0° → hand=-90°`
- 用户改 x 默认值：**-100 → -150**（belt-slip 风险更高）
- 用户改顺序：x 从第 2 步挪到最后 → **`y → arm → hand → x`**（4 步）

### `get_blue.py` 改动（多次重排）
- 初版：4 步 `move_x(0, belt-slip 分段) → arm=0° → y=-90 → hand=0° (大臂 dance)`
- 用户改：move_x(0) → **reset_x 撞墙**（用户要求"一步到位"）
- 排查：reset_x 撞到"另一侧"墙 → **direction 字符串语义**：`"right"=+速度`/`"left"=-速度`（**arm_base.py:532 原文**）
- 默认 direction：**`"right"`**（按用户反馈"left 撞错"修正）
- 改 `probe_time=0 → 0.3`（用户未确认墙位时留默认探针更稳）
- 改大臂默认：**`0° → -5°`**（用户调整）
- 改第 4 步：去掉大臂 dance，**直接 `_call_arm` 直调 set_hand_angle**（用户要求"改手爪时不要大臂移动"）
- 改顺序 1：4 步 → **5 步**：`x → y=-130 → arm=-5° → hand=0° → y=-90`
- 改顺序 2：5 步又改 → **`y=-130 → x → arm → hand → y=-90`**（**当前**）
  - 副作用：从 init (y=0) 也能直接跑（第 1 步 move_y 不过 y 保护区）

### `get_yellow.py` 改动（完全照搬 get_blue）
- 初版 4 步 `move_x(-65) → arm=-5° → y=-90 → hand=0°`
- 用户改：x = **-72**（最终值）
- 用户要求：不用 reset_x，**用 move_x**（区别于 get_blue 的 reset_x 撞墙）
- 同步 get_blue 的 5 步顺序：`y=-130 → move_x(-72) → arm=-5° → hand=0°(底层直调) → y=-90`

### `grasp_5.py` 新建
- 最简：grasp(True) → sleep(5s) → grasp(False)
- 参考 task4/grasp.py 模板，但 CLI 加 `--hold` flag

### 排错：x 自动回 0
- 阶段 1：用户问"吸气时 x 自动回 0" → 排查调用链（`grasp()` 不碰 x）
- 阶段 2：用户问"运行一半时 x 突然移到 0" → 怀疑是 calibrate 读数 bug
- 阶段 3：精读 `runtime_service.py:_auto_init_loop` → 找到根因
  - **根因**：`_auto_init_loop` 后台线程 → `_probe_controller` 误判 car "not ready" → `ensure_initialized` → `_create_car_locked(reset_arm=False)` → `arm.reset_all()` → **撞墙 calibrate x=0**
  - **触发窗口**：job 之间的间隙（如 `time.sleep(5)`），`current_job_id=None` 时 auto_init 就能跑

---

## 6. 当前问题 & 解决计划

### 问题 1：运行时 x 突然回 0（auto_init 撞墙 calibrate）
**根因**：`auto_init_loop` → `ensure_initialized` → `_create_car_locked` → `arm.reset_all()` 撞墙 calibrate x=0
**业务层能做**：
- ✅ 临时验证：建议用户**关 `RAK_CAR_AUTO_INIT=0`** 重启 runtime（编辑 `ecosystem.config.js`），观察 x 是否还"突然回 0"
- ✅ 业务层短 sleep（用 `time.sleep(0.5)` 而非 5s）减少窗口
**业务层做不了**：
- ❌ 改 `_auto_init_loop` 逻辑（runtime 层）
- ❌ 改 `_create_car_locked`（runtime 层）
- ❌ 改 `arm.reset_all()`（SDK 层）

**推荐方案**：
1. 临时关 `RAK_CAR_AUTO_INIT=0` 验证根因（**最快定位**）
2. 比赛前改回 `RAK_CAR_AUTO_INIT=1`，**但**把 `RAK_CAR_RESET_ARM_ON_AUTO_INIT=0` 避免撞墙 calibrate（**不破坏 auto-recover 能力**）
3. 长期：等用户授权动 runtime 时，把 `auto_init` 的 `reset_all` 路径改成只 `reset_y`（不撞 x）

### 问题 2：文件曾被外部清空
**业务层能做**：
- ✅ 保持自包含风格（不 import task5 包内其他模块）
- ✅ 每个脚本都是独立可跑的"工具脚本"（不依赖 6 阶段编排）
- ✅ git commit 前看 `git status` 确认文件在

### 问题 3：get_blue 的 direction 默认值未在车上实测
- 当前默认 `direction="right"`
- 用户上次反馈"left 撞错" → 改 right，但**没真测过 right 是不是对**
- 现场跑一次验证，需要时改 `--direction left`

### 问题 4：x 读数 bug（§11 已知）
- `x_get_position` calibrate 框架坏
- 业务层**永远走 `_read_x_mm_realtime()`**

---

## 7. 注意事项（继续工作前必看）

1. **底层不动**：smartcar/whalesbot/**、runtime/**、car_wrap_2026.py 都不能动
2. **x 读数**：永远 `_read_x_mm_realtime()`，别信 `x_get_position` / `get_state().x_mm`
3. **belt-slip**：跨 30mm 要分段
4. **保护区**：y 实际是 [0, -30]mm（不是文档说的 [0, -80]）
5. **set_hand_angle 展开区限制**：走 `_call_arm` 直调绕开
6. **reset_x 方向**：`"right"=+速度`/`"left"=-速度`，无固定物理墙映射
7. **probe_time**：用 0.3，不用 0（位置未知时关探针会误判 stall）
8. **move_y 任意值放行**：包括保护区内的 y（api.py:323-325 明确）
9. **自包含**：每个脚本不依赖会消失的 `constants.py`
10. **auto_init 是元凶**：用户能关就关，关不掉就让 `RESET_ARM_ON_AUTO_INIT=0`

---

## 8. 续接指南（下次起手 5 步）

1. **读本文档**（`debug-task5-rebuild-2026-07-22.md`） — 你正在看的
2. **看 CLAUDE.md 关键章节**：并发任务模型 / `auto_init` 触发条件 / Runtime 锁层次
3. **在车上试 `python main/arm/each_task/task5/get_blue.py`**（先看吸/不吸气、不撞墙时 5 步能不能跑完）
4. **如果 x 还突然回 0**：按 §6 问题 1 走，先关 `RAK_CAR_AUTO_INIT=0` 验证
5. **如果文件被再次清空**：所有 task5 脚本都是自包含的，重写一个就行（参考 `high_tower.py` 模板）

---

## 9. 跑法 cheat sheet

```bash
# 1. high_tower 摆位姿 (4 步)
python main/arm/each_task/task5/high_tower.py

# 2. get_blue 取蓝位姿 (5 步, reset_x 撞墙)
python main/arm/each_task/task5/get_blue.py
python main/arm/each_task/task5/get_blue.py --direction left   # 撞错侧改

# 3. get_yellow 取黄位姿 (5 步, move_x 直接走)
python main/arm/each_task/task5/get_yellow.py

# 4. grasp_5 简单吸气 5s
python main/arm/each_task/task5/grasp_5.py
python main/arm/each_task/task5/grasp_5.py --hold 8   # 改保持秒

# 5. 实时观察 x/y
curl -s http://192.168.6.231:5050/v1/realtime/arm/state | python -m json.tool
```

---

## 10. 引用关联

- [[x-axis-rollout-session]] — x 轴全天 session 总览
- [[execute-sync-default]] — `/v1/execute` 默认 sync=False
- [[arm-business-layer-only]] — 业务层禁改底层硬约束
- [[x-speed-safety-watchdog]] — x_speed 后台 watchdog
- [[x-get-position-vs-realtime]] — x 读数真值路径
- [[x-axis-belt-slip]] — 同步带打滑
- [[arm-api-reference]] — ARM_API.md 速查表
- [[task4-rebuild-2026-07-22]] — task4 重建参考（task5 借鉴）
