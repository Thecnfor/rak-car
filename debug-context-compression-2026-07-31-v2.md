# task4 上下文压缩 v2 (2026-07-31)

承接 `debug-context-compression-2026-07-31.md` (v5+v6 内容)。本会话主要围绕:
1. **v6 pick_up_yellow.py 创建**(从 pick_up_blue.py 1:1 复制,只改 X_BIN_MM 0→-65)
2. **target1 X_MM 来回** (-260 → -80 → 改回 -260)
3. **API 复查** (确认 x 轴无软件限制,问题在物理硬件)
4. **realtime x 读数飘** (两次跑差异 195mm,SDK bug)
5. **业务层无能为力边界确认** (用户硬约束 + SDK 修复动不了)

---

## 1. 调用的文档

| 文档 | 用途 |
|---|---|
| **CLAUDE.md** | 仓库结构 + 业务层硬约束 (只能改 main/**) |
| **`main/arm/README.md`** + `ARM_API.md` | arm 业务包结构 + API 速查 (§7.2 x 轴限制, §9 reset_x, §11 realtime x) |
| **`main/arm/test/test_x_to_150.py`** | 参考模式 (单次读 x0 + 分段循环 + stall kick) |
| **`main/arm/each_task/common.py`** | `move_x_with_split` / `move_x_trust` |
| **`main/arm/api.py`** | ArmClient.move_x 实现 (api.py:406-429) |
| **`main/arm/state.py`** | step_loss_x_mm 默认 5.0 (state.py:56) |
| **memory/** | x-axis-belt-slip, x-get-position-vs-realtime, x-speed-safety-watchdog, jetson-current-ip, arm-grasp-call-arm-base, arm-target-y-position-trajectory |
| **常量** | `constants.py` (BLUE_BIN_X_MM=0, YELLOW_BIN_X_MM=-65, BALL_VERIFIED_*, X_PHYSICAL_WALL_MM) |

---

## 2. 目前情况

### 2.1 task4 业务流 x 轴状态转移图 (v6 后)

```
[init: reset_x 撞墙定原点 x=0]
    ↓
[target1.py] step 4: move_x_trust(-260)      ← 代码 -260, 实际 motor ≈ -80~-193
    ↓
[target2.py] 检测球 (不动 x)                   ← BALL_VERIFIED_* 按 -260 校准
    ↓
[target3.py] 吸气下降抓球 (不动 x)
    ↓
[pick_up_blue.py]  step 5: move_x_trust(0)    ← 蓝 bin
                  step 6-8: 放球 + 抬 y
                  step 9: move_x_trust(-260)  ← 回抓取位 (v6 新增)
    ↓
(下一轮) target1.py: motor 还在 ≈ -80, move_x(-260) = 走 180mm (从 -80 到 -260)
```

### 2.2 motor 实际能力 (2026-07-31 实测)

| 项目 | 值 | 来源 |
|---|---|---|
| **目标位置** | -260mm | 用户拍板 |
| **物理墙 (理论)** | ≈ -119.5mm | ARM_API.md §535 |
| **实测 motor 极限** | ≈ **-80mm** (用户第 1 次) / **-193mm** (用户第 2 次, realtime 读数) | 现场实测 |
| **realtime x 飘** | 同一脚本两次跑差 195mm | 现场实测 |
| **belt-slip 单次行程** | 24-46mm | memory/x-axis-belt-slip |
| **业务层能跑到的最大** | 不知道,realtime 不可信 | 物理限制 |

**关键结论**: motor 物理上**走不到 -260mm**,但**实际能到多少不确定**(realtime 读数飘,无法验证)。

### 2.3 task4 当前文件状态

| 文件 | 状态 |
|---|---|
| `target1.py` | **TARGET1_X_MM = -260.0** (用户决定保持 -260, 等硬件修复) |
| `pick_up_blue.py` (v6) | DEFAULT_RETURN_X_MM = -260.0 (用户决定) |
| `pick_up_yellow.py` (v6, 本会话新建) | DEFAULT_RETURN_X_MM = -260.0 |
| `target2.py` | 不动 x |
| `target3.py` | 不动 x |
| `target4.py` | 扫描范围 [-280, -240] 也超物理墙, 但未在本会话修复 |
| `test_blue.py` / `test_yellow.py` / `x_to_zero.py` | 摆位姿工具, 不动 |
| `constants.py` | BALL_VERIFIED_* 仍是 -260 校准, **改 target1 后必须重测** |

---

## 3. 本会话改动

### 3.1 pick_up_yellow.py (v6) 新建 (1:1 复制 pick_up_blue)

**唯一区别**: `X_BIN_MM = -65.0` (黄 bin), 其余完全一致。

```
pick_up_blue   X_BIN_MM = 0.0   ← 蓝 bin
pick_up_yellow X_BIN_MM = -65.0 ← 黄 bin (本会话新建)
```

**9 步流程** (跟 pick_up_blue 完全一致):
1. 记录 x_initial
2. grasp(True) 吸气
3. move_y(-58) 抓球位
4. move_y(-190) 中转位
5. **move_x_trust(-65) 黄 bin** ← 唯一区别
6. move_y(-155) 放球位
7. grasp(False) + sleep 放气
8. move_y(-133) 最终位
9. move_x_trust(-260) 回抓取位 (v6 设计, 默认)

### 3.2 target1 X_MM 来回历史

| 时间 | TARGET1_X_MM | DEFAULT_RETURN_X_MM | 备注 |
|---|---|---|---|
| v5 | -260 | None | 不回位 |
| v6 初版 | -260 | **-260** | 加回抓取位 |
| **临时改 -80** | **-80** | **-80** | 用户实测 motor 走不到 -260 |
| **改回 -260** (用户最终决定) | **-260** | **-260** | 等硬件修复 |

### 3.3 改动文件清单

| 文件 | 改动 |
|---|---|
| `main/arm/each_task/task4/pick_up_yellow.py` | **新建** (从 pick_up_blue 复制, X_BIN_MM 改 -65) |
| `main/arm/each_task/task4/target1.py` | TARGET1_X_MM 改 -260, docstring warning 强化 |
| `main/arm/each_task/task4/pick_up_blue.py` | DEFAULT_RETURN_X_MM 改 -260, docstring 警告 |
| `main/arm/each_task/task4/pick_up_yellow.py` | DEFAULT_RETURN_X_MM 改 -260, docstring 警告 |

**3 个文件都加了显眼警告**:
```
⚠️ **2026-07-31 已知 motor 走不到 -260**: 用户实测 motor 实际只走到 ≈ -80mm
(belt-slip / 编码器失同步 / 电流保护, 不是物理墙 -119.5mm 的限制)
trust 模式不报 stall, 业务层看不见实际停在 ≈ -80mm
代码保持 -260 是用户决定, 等硬件修复后再真到位。
```

### 3.4 v6+ 速度改动 (2026-07-31 用户要求改大)

**用户原话 (第一轮)**: "还是移动不了那个位置,你 x 轴的速度改大一点试试"
**用户原话 (第二轮)**: "速度改成 80 试一下,那个 belt 的问题已经解决了"

**改动时间线**:
- 第一轮 30→40 (API 默认上限,业务层最大), 用户怀疑速度是限制因素
- **第二轮 40→80**: **用户反馈 belt-slip 已修复**, SDK 无硬限

**SDK 关键发现 (arm_base.py:472-510)**:
- `v_max_mms` 是 **临时收紧 `x_velocity_limit`** (±v_limit), try/finally 还原
- **SDK 无硬限**, 业务层传多大就多大
- belt-slip 修复后 `x_stop_check` (arm_base.py:497) 撞墙检测能正常触发, motor 不会无限 stall

**最终改动** (target1/pick_up_blue/pick_up_yellow 三文件 `MOVE_X_V_MAX_MMS`):
- `target1.py:85` 30.0 → 40.0 → **80.0**
- `pick_up_blue.py:130` 30.0 → 40.0 → **80.0**
- `pick_up_yellow.py:107` 30.0 → 40.0 → **80.0**

**业务层上限**: SDK 无硬限, 业务层可以无限大。但实际中:
- 太快可能撞坏机械结构
- 太快可能触发 motor 过流保护
- 建议 100 mm/s 以内 (保守档)

---

## 4. API 复查结果 (本会话重要发现)

### 4.1 API 层 x 轴无软件限制 (已确认)

**ARM_API.md §520-526**:
```
x 轴软限位已取消 (2026-07-16): 用户原话"灵活使用就好,一般不会超"
业务层 ArmClient._check_safe 不再校验 x
SDK move_x_position / x_speed / goto_position 不再 clamp
arm_cfg.yaml:horiz_cfg 已删除 threshold / slow_band_m / slow_velocity / wall_* 等
```

**`_check_safe` (api.py:1177-1190)**: 只校验 y, x 参数被忽略。

### 4.2 move_x 调用链

```
ArmClient.move_x(x_mm, v_max_mms=40, out_time=15.0, timeout=30)
  │
  ├─ self._check_y_protected("move_x")           ← 检查 y, 不动 x
  │
  ├─ self._call_arm(
  │     "move_x_position",
  │     target=_mm_to_m(x_mm),                    ← 绝对位置
  │     out_time=out_time,                        ← PID 调节时间 (默认 15s)
  │     v_max_mms=v_max_mms,                      ← 业务限速 (默认 40 mm/s)
  │ )
  │
  └─ self._check_step_loss("x", ...)              ← 只 warn, 不抛错

底层: arm.move_x_position(target, out_time=6.0, v_max_mms=None)
```

### 4.3 关键参数 (api.py:406-429)

| 参数 | 默认值 | 业务层传值 | 说明 |
|---|---|---|---|
| `v_max_mms` | 40 mm/s | **30 mm/s** (target1) | SDK 临时收紧 PID 输出限幅 |
| `out_time` | **15.0 s** | **未传** (走默认) | PID 调节到目标的时间 |
| `timeout` | 30.0 s | 30.0 s | HTTP 同步超时 |

### 4.4 撞墙兜底 (SDK 层, 业务层看不见)

**ARM_API.md §530**:
> 撞墙判据 (x 无传感器): `move_x_position` 中 `x_stop_check` + 100ms dwell 后自动 calibrate `x_pose_start`

**问题**: 如果 motor 没真撞墙 (belt-slip stall), `x_stop_check` 检测不到, motor 内部 PID 卡住, **move_x 在 15s out_time 后静默完成**, 业务层以为成功。

### 4.5 realtime x 读数飘 (SDK bug, 业务层无能为力)

**两次跑差异** (同一脚本):
```
跑 #1: realtime = +2.5 mm
跑 #2: realtime = -193.42 mm
差异: 195.92 mm (> motor 总行程)
```

**业务层无能为力** (SDK bug, 用户原话 "SDK 修复是官方写的,动不了")。

### 4.6 state.py 默认 (state.py:56)
```python
step_loss_x_mm: float = 5.0     # 偏差 > 5mm warn (但不抛错)
```

---

## 5. 面临的问题

### 5.1 硬件层 (业务层无能为力, 改不了)

| 问题 | 状态 | 修复 |
|---|---|---|
| 同步带打滑/涨紧不够 | belt-slip 单次 24-46mm | 现场硬件 (用户不能改) |
| 编码器失同步 | realtime 飘 + motor stall | SDK 层 (官方, 动不了) |
| Motor 电流保护 | motor 走到某位置停机 | SDK / 硬件 |
| Realtime x 读数飘 | SDK bug | SDK 层 (官方, 动不了) |

### 5.2 SDK 层 (业务层不能改)

| 问题 | 状态 |
|---|---|
| `move_x_position` 内部 `x_stop_check` 检测不到 belt-slip stall | SDK 不暴露 |
| `arm_feed` 缓存延迟 / 计算 bug | SDK 不暴露 |
| calibrate 框架坏 (撞墙后 x_get_position 不可信) | SDK 不暴露 |

### 5.3 业务层未完成

| 问题 | 优先级 |
|---|---|
| **motor 走不到 -260, 业务层无法保证到位** | 🔴 高 (用户决定保持 -260, 等硬件修复) |
| **BALL_VERIFIED_* 必须重测** (球在新 x=-80 / -193 实际位置范围变了) | 🔴 高 |
| target4.py 扫描范围 [-280, -240] 也超物理墙 | 🟡 中 (本会话未触碰) |
| 球几何漂移根因 (aspect 0.35-1.06 反复跳变) | 🟡 中 |
| runtime detection 间歇性 (1↔2 球) | 🟡 中 |
| 多轮 pick 循环 x 衔接 (target1 → pick → target1) | 🟡 中 (trust 模式能跑, 等硬件修复) |

### 5.4 业务流程影响

**当前状态**: target1.py 跑 move_x(-260), motor 实际走到 -80 ~ -193 (依赖现场状态):
- target2 检测球失败 (BALL_VERIFIED_* 按 -260 校准, 实际位姿不一样)
- pick_up_blue/yellow 抓球失败 (吸盘不在球的位置)
- **业务流跑不通**, 但代码逻辑保持 -260 (用户决定)

---

## 6. 准备在解决

### 6.1 立即可做 (下次会话第一件事)

- [ ] **现场验证当前状态**: 跑 target1.py + target2.py, 看 realtime x 和 BALL_VERIFIED_* 实际匹配情况
- [ ] **如果检测失败**: 临时关掉 BALL_VERIFIED_* 验证 (`target2.py --no-verify-target1-pose` 默认 False) 跑通流程
- [ ] **跑一次 pick_up_blue 完整 9 步**, 看 step 9 回 -260 实际表现

### 6.2 业务层能试的优化 (硬件修复前的最后手段)

- [ ] **v_max_mms 改 50** (target1.py MOVE_X_V_MAX_MMS), 给 motor 更多动量
- [ ] **加 reset_x 撞墙定原点** (在 target1 step 4 之前先 reset_x), 让 motor 知道真实 0
- [ ] **改 out_time** (改 common.move_x_trust 增加 out_time 参数), 给 PID 更长时间
- [ ] **改回分段 move_x_with_split** (放弃 trust 模式), 用 stall kick 救 (但 realtime 飘可能误判)

### 6.3 中期 (硬件修复后)

- [ ] **同步带涨紧 / 更换**
- [ ] **编码器校准**
- [ ] **重测 BALL_VERIFIED_*** (target1 实际位姿下球 cx/cy/w/h 范围)
- [ ] **验证 target1 move_x(-260) 真到位**

### 6.4 长期 (SDK 修复后)

- [ ] `arm_feed` / realtime 路径 bug 排查
- [ ] `x_stop_check` 加 belt-slip stall 检测
- [ ] calibrate 框架修复

---

## 7. 注意事项 (硬约束)

### 7.1 业务层限定

**只能改 `main/**`** (包括 `main/arm/each_task/**`)。

❌ **不能改**:
- `smartcar/whalesbot/**` (SDK, hardware abstraction)
- `runtime/**` (FastAPI service)
- `car_wrap_2026.py` / `car_start_2026.py` / `car_task_function.py` (legacy monolith)

### 7.2 用户原话硬约束

> "SDK 修复是官方写的,动不了"

→ SDK 层 realtime 读数飘 / belt-slip stall 检测不到 / calibrate 框架坏, **业务层无法修复, 等官方**。

> "我只能改业务层,底层等先暂时不能改动"

→ `_check_safe` / `move_x_position` / `x_stop_check` 等 SDK 内部逻辑不动。

### 7.3 用户反复改主意历史 (避免重复踩坑)

| 主题 | 历史 | 最终决定 |
|---|---|---|
| pick_up_blue v5: 是否回 x_initial | 改回 (v4) → 删回 (v5) → 加回 (v6) | **v6: 加回, 默认 -260** |
| target1 X_MM | -260 (历史) → -80 (临时) → -260 (用户决定) | **-260** |
| DEFAULT_RETURN_X_MM | None (v5) → -260 (v6) → -80 (临时) → -260 | **-260** |

**结论**: target1 X_MM 保持 -260, 等硬件修复后业务流自然跑通。

### 7.4 realtime 读数飘警告

任何 `realtime x` 数值都**不可信**, 仅供日志参考。**不要基于 realtime 读数做业务判断**。

### 7.5 信任 motor PID 内部位置

`move_x` 是**绝对位置指令** (api.py:418 `target=_mm_to_m(x_mm)`), motor 内部编码器闭环。realtime 飘不影响实际指令下发。

---

## 8. 现场实测数据 (本会话)

| 时机 | x_initial | move_x 命令 | warning actual | realtime | 结论 |
|---|---|---|---|---|---|
| **pick_up_blue v4 #4** | +24.5 | move_x(0)→bin | - | -0.57 OK | 短距 OK |
| **pick_up_blue v4 #4** | - | move_x(+24.5)→回 x_initial | -1.4 (stall) | +283.67 撞墙错 | 短距 stall |
| **target1 第 1 次** | - | move_x(-260) | +2.5 | +2.5 | realtime 飘 + 实际只走 ≈ -80 (用户目测) |
| **target1 第 2 次** | - | move_x(-260) | +2.5 | **-193.42** | realtime 又飘 + 实际走到 ≈ -193 (差异 195mm) |

**realtime 两次跑差 195mm**, 说明 SDK bug 严重。

---

## 9. 关键调用链 (供下次会话快速理解)

```
[user 命令]
python target1.py
    ↓
step_target1(client, runner, y=-133, x=-260, arm=90, hand=0)
    ↓
1. runner.move_y(-133)                        ← move_y 走 y 步进电机
2. client.set_arm_angle(90)                  ← 业务硬限 [+90, -150]
3. client._call_arm(set_hand_angle, 0)       ← 底层直调绕拦截
4. common.move_x_trust(client, runner, -260) ← trust 模式
    │
    └─ client.move_x(x_mm=-260, v_max_mms=30, timeout=30)   ← 默认 out_time=15s
         │
         ├─ self._check_y_protected("move_x")  ← 检查 y 保护区
         ├─ self._call_arm("move_x_position", target=-0.260, out_time=15, v_max_mms=30)
         │    │
         │    └─ arm.move_x_position(target, out_time, v_max_mms)
         │         │  ← SDK 层 (业务层不能改)
         │         └─ motor 内部 PID + 编码器闭环
         │
         └─ self._check_step_loss("x", target=-260, actual=state.x_mm, threshold=5.0)
              ← warning: 偏差 > 5mm warn, 不抛错

5. runner.move_y(-133) 出 y 保护区 (已在)
6. client._read_x_mm_realtime()                ← 只读 1 次当日志
7. return {... x_info: {actual_x: <realtime>, result: "trust"}}
```

---

## 10. 续接 TODO (优先级排序)

1. **硬件修复后立即验证** target1.move_x(-260) 真到位 (听马达声 + 目测)
2. **重测 BALL_VERIFIED_*** (现场跑 target1 + target2 看新 cx/cy 范围)
3. **业务层能试的最后优化**:
   - v_max_mms 30 → 50
   - 加 reset_x 撞墙定原点
   - out_time 改大
4. **target4.py 修复** (扫描范围 [-280, -240] 改到 motor 能到的范围)
5. **多轮 pick 循环 x 衔接** (trust 模式能跑, 等硬件修复)
6. **球几何漂移根因** (跨会话累积, 本会话未触)

---

## 11. 一句话总结 (供下次会话开头读)

本会话围绕 target1 X_MM = -260 反复折腾: 临时改 -80 跟 motor 物理极限对齐 → 用户坚持 -260 改回。**业务层确认无能为力让 motor 走到 -260** (SDK 修复动不了, 硬件 belt-slip 动不了), 代码保持 -260 等硬件修复。**API 复查确认 x 轴无软件限制**, 问题在底层 PID + 编码器 + belt-slip 综合物理因素。**realtime x 读数两次跑差 195mm** 说明 SDK bug 严重, 业务层完全无法基于 realtime 做判断。

**v6+ 用户试最后一招: 速度 30→40→80 mm/s**:
- 30→40 (业务层最大, belt-slip 仍存在时无效)
- 40→80 (用户反馈 belt-slip 修复后, SDK 无硬限 arm_base.py:482-487 临时收紧, 验证后定档)
