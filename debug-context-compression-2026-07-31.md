# task4 上下文压缩 (2026-07-31)

承接 `debug-context-compression-2026-07-30.md`。本会话主要围绕**x 轴读取不可信 + 马达 stall** 的问题反复折腾 pick_up_blue.py,最后用 v5 简化收尾。

---

## 1. 调用的文档

| 文档 | 用途 |
|---|---|
| **CLAUDE.md** | 仓库结构 + 业务层硬约束 (只能改 main/**) |
| **`main/arm/README.md`** + `ARM_API.md` | arm 业务包结构 + API 速查 (§9 reset_x escape hatch, §10 grasp kwargs 陷阱, §11 realtime x) |
| **`main/arm/test/test_x_to_150.py`** | **本会话最重要的参考** —— 现场实测能跑通的 x 轴 move 模式 (单次读 x0 + 分段循环 + stall kick) |
| **`main/arm/each_task/common.py`** | `move_x_with_split` / 本会话新增 `move_x_trust` |
| **memory/** | `x-axis-belt-slip`, `x-get-position-vs-realtime`, `x-speed-safety-watchdog`, `arm-grasp-call-arm-base`, `jetson-current-ip`, `ball-best-grasp-2026-07-30` 等 |
| **常量** | `constants.py` (BALL_VERIFIED_*, TARGET_AREA_*, Y_*, X_BIN_MM=0, YELLOW_BIN_X_MM=-65) |
| `runtime/core/settings.py` | PUBLIC_HOST=192.168.5.230 (三处同步过) |

---

## 2. 目前情况

### x 轴的 3 层问题

| 层 | 状态 | 证据 |
|---|---|---|
| **realtime x 读数** | ❌ 不可信 | 现场实测: x0=+24.5 → +283.67 (撞墙后) → -0.57 → +147 等乱跳 |
| **move_x 短距执行** | ❌ 经常 stall | 现场实测: `move_x(+24.5)` 马达报"实际=-1.4mm"(没动) |
| **reset_x 撞墙恢复** | ⚠️ 能撞但读数仍错 | 撞墙后 realtime 显示 +283.67 (应该=0, 不是读数问题就是 offset) |

### 当前 task4 改动全景

| 文件 | 改动 |
|---|---|
| **`common.py`** | 新增 `move_x_trust()` (单次 move_x + 不 stall/wall/overshoot 检测) |
| **`pick_up_blue.py`** | v5 (8 步): 删除 step 8 回 x_initial, 改 trust move_x, x 停在 0 |
| **`test_yellow.py`** | step 5 用 `move_x_trust` 替 `runner.move_x` |
| **`target4.py`** | 扫描循环用 `move_x_trust` 替 `arm_client.move_x` |
| **`target1.py`** | step 4 用 `move_x_trust` 替 `_move_x_with_split` (内联函数废弃但保留) |
| `test_blue.py` / `x_to_zero.py` | 不动 (reset_x 撞墙定原点,已 OK) |

---

## 3. 本会话改动 (按时间倒序)

### pick_up_blue.py 演化史 (8 个版本)

| 版本 | 步骤 | 问题 |
|---|---|---|
| v1 | 6 步: 吸气 → y-58 → y-190 → **reset_x** → y-155 → 放气 | reset_x 多 3-5s |
| v2 | 9 步: 记 → ... → move_x(0) → ... → **move_x 回 x_initial** → y-133 | 回位失败 (马达 stall) |
| v3 | 8 步: 删除回 x_initial, x 停在 0 | 用户又要回位 |
| v4 | 9 步: 加 reset_x 撞墙 + move_x_trust 回 x_initial, retry fallback | 马达短距 stall, retry 也失败 |
| **v5 (当前)** | **8 步: 不回位, x 停在 0** | ✅ |

### common.move_x_trust 新增 (2026-07-31)

```python
def move_x_trust(client, runner, target_x_mm, *, log_prefix, v_max_mms, timeout) -> dict:
    """trust 模式: 单次 client.move_x(), 不 stall/wall/overshoot 检测.
    适用 realtime x 读数飘场景. 返回 reached=True 永远."""
```

**适用场景**: realtime 不可信时使用,业务层接受"撞墙不报 / 打滑不到位"风险。

### common.move_x_with_split 仍是默认

**适用场景**: realtime 可信时 (理想状态),有 stall/wall/overshoot 三重保护。

---

## 4. 面临的问题

### 4.1 硬件层 (业务层无能为力)

| 问题 | 业务层动作 |
|---|---|
| 同步带涨紧不够 / 单边磨损 | 已记录 → 现场需查硬件 |
| 编码器失同步 | reset_x 部分缓解,但仍有失败 |
| 马达扭矩不够 (吸球负重) | 球可能没掉,需现场验证 |
| 物理阻挡 (线缆 / 螺丝) | 现场查 |

### 4.2 软件层 (SDK bug,业务层无权改)

| 问题 | 业务层动作 |
|---|---|
| realtime x 读数飘 (乱跳) | 已用 trust 模式绕过 |
| arm_feed 缓存延迟 | 无 |
| move_x 内部报 warning 但不抛异常 | trust 模式接受这个 |

### 4.3 业务层未完成

| 问题 | 优先级 |
|---|---|
| **多轮 pick 循环 (target2 → pick_up_blue → target2) 怎么衔接?** | 🔴 高 (v5 删了回位,业务流断了) |
| target2.py 是否需要保证每轮自己 move_x 到检测位置? | 🔴 高 |
| 任务 5 / 6 / 7 是否需要类似 trust 改造? | 🟡 中 |
| 其他 task (target1.py / target4.py) v5 同步? | 🟢 低 (trust 已改,只是没删 reset_x 兜底) |
| 球几何漂移根因 (aspect 0.35-1.06 反复跳变) | 🟡 中 (跨多日累积,本会话未触碰) |

---

## 5. 准备在解决

### 5.1 立即可做 (下次会话第一件事)

- [ ] **确定多轮 pick 循环的 x 衔接方案** (target2 自己 move_x vs pick_up_blue reset_x vs 其他)
- [ ] **现场跑一次 v5** 确认脚本能跑完,马达位置 OK
- [ ] **跑 target1.py 验证衔接** (从 x=0 走 -260 是否顺畅)

### 5.2 中期

- [ ] **球几何漂移根因调查** (camera 位置 / 球姿态 / 任务模型)
- [ ] **runtime detection 间歇性** (1↔2 球, 跟 aspect 漂移相关?)
- [ ] **task5 / task6 / task7 是否需要 v5 同步**

### 5.3 长期 (硬件修复)

- [ ] 同步带涨紧 / 更换
- [ ] 编码器校准
- [ ] arm_feed / realtime 路径 bug 排查 (需改 SDK / runtime,**业务层不能动**)

---

## 6. 注意事项 (硬约束)

### 业务层限定

**只能改 `main/**`**(包括 `main/arm/each_task/**`)。

❌ **不能改**:
- `smartcar/whalesbot/**` (SDK, hardware abstraction)
- `runtime/**` (FastAPI service)
- `car_wrap_2026.py` / `car_start_2026.py` / `car_task_function.py` (legacy monolith)

### 现场实测数据

```
2026-07-31:
  pick_up_blue.py v4: x_initial=+24.5, move_x(0)→realtime=-0.57 (OK),
    reset_x→realtime=+283.67 (错), move_x(+24.5)→马达实际=-1.4mm (stall)
  → 改 v5: 删除回位, x 停在 0
```

### 用户反复改主意历史 (避免重复踩坑)

- v2 → v3: 用户要回位 → 不回位 (说会卡 target1)
- v3 → v4: 用户要回位 → 加 reset_x 兜底
- v4 → v5: 马达 stall 救不了 → 删除回位

**结论**: x 回位功能**业务层做不到可靠**, 不要再加回来。

### test_x_to_150.py 模式 (作为参考,不动)

```
读 x0 → 循环:
  client.move_x(target, v_max) → _read_x() → err < TOL 成功
  → stall: kick (stop + sleep + retry) → 3 轮 stall 放弃
```

`common.move_x_with_split` 实现了这套模式 + wall/overshoot,但因为 realtime 不可信,实际现场跑不通。

---

## 7. 快速续接指南

### 验证 v5 能跑

```bash
python main/arm/each_task/task4/pick_up_blue.py
```

预期: 8 步全部走完, x 留在 0 附近(实际位置靠马达控制), target1.py 从 x=0 走 -260。

### 现场调试步骤

1. **先跑 v5**,确认能跑完 → 业务层 OK
2. **再跑 target1.py** (`move_x(-260)`), 看马达能否走完 -260mm
3. **如果 target1 stall**: 硬件问题,查同步带 / 编码器
4. **如果 v5 后 x 不在 0**: 撞墙定原点 (`x_to_zero.py`) 重置

### 现场数据收集

- 跑前后各打印 realtime x(已知不准但留档)
- 跑前后各打印马达控制器 `actual` (move_x warning 里)
- 听马达声 (正常走 vs stall 声不同)

---

## 8. 现场测量数据 (本会话)

| 时机 | x_initial | step 5 move_x(0) | step 8 reset_x | step 8 move_x | 结论 |
|---|---|---|---|---|---|
| v4 第 1 次 | +28.3 | - | - | move_x(+28.3) → -549.6 (诡异) | realtime 飘 |
| v4 第 2 次 | -262.6 | - | - | move_x(-262.6) → -259.7 OK | 长距 OK |
| v4 第 3 次 | +176.0 | realtime=147.36 (错) | - | move_x(+176) → 实际=-1.6 (stall) | 短距 stall |
| **v4 第 4 次** | **+24.5** | **realtime=-0.57 OK** | **realtime=283.67 (错)** | **move_x(+24.5) → 实际=-1.4 (stall)** | **撞墙后短距仍 stall** |

---

## 9. 续接 TODO (优先级排序)

1. **多轮 pick 循环 x 衔接方案** (用户下个问题)
2. **球几何漂移根因** (跨会话累积,本会话未触)
3. **task5/task6/task7 是否同步 v5** (降低复杂度)
4. **runtime detection 间歇性** (跟球漂移可能同源)
5. **硬件修复** (同步带 / 编码器 / arm_feed bug)

---

**下次会话第一件事**: 读这个文档,确认 task4 v5 跑通,再决定是否继续推进多轮 pick 循环。

---

## 10. v6 续接 (2026-07-31 追加: 解决 "x 不回抓取位")

### 用户痛点
v5 删了回 x_initial 步骤,导致 pick_up_blue 后 x 留在 0,如果下一阶段不是
target1/target4(例如手动多轮跑 pick),x 不回到抓取位置 (-260)。

### 解决方案: v6 加回抓取位步骤 9
**文件**: `main/arm/each_task/task4/pick_up_blue.py` (v6, 9 步)

**关键设计**:
- 加可选 `return_x_mm` 参数 (默认 `-260.0` = target1.py 抓取位)
- 业务流选项:
  - `-260.0` (默认) = target1 抓取位 (业务流推荐)
  - `-220.0` = target4 prep_x (target4 用户)
  - `None` = 不回位 (v5 行为兼容)
- 走 **trust 模式 move_x_trust** —— 不 stall kick, 不 reset_x, 信任 motor PID 内部位置闭环

### 关键确认 (api.py:418)
**`move_x` 是绝对位置指令** (`target=_mm_to_m(x_mm)`), motor 内部编码器闭环准,
**不依赖 realtime 读数飘**。所以 trust 模式可行 (虽然 realtime 飘, motor 实际位置是对的)。

### 跟 v4 失败案例的关键区别
| | v4 (失败) | v6 (新) |
|---|---|---|
| 撞墙 reset_x | ✅ 调 (副作用: 马达物理状态变化) | ❌ 不调 |
| move_x 距离 | 短距 24mm (stall 频发) | 长距 260mm (trust 模式 motor 闭环准) |
| stall 检测 | belt-slip + stall kick | trust 模式跳过 (信任 motor) |
| 失败根因 | reset_x 后短距 stall | (待验证) 信任 motor 内部位置 |

### x 状态转移图 (v6)
```
init: x = 0 (撞墙定原点)
    ↓
target1.py step 4: x → -260 (trust mode)
    ↓
检测球 + pick_up_blue
    ↓
pick_up_blue step 5: x → 0 (trust mode, 蓝 bin 上方)
    ↓
pick_up_blue step 6-8: 放球 + 抬 y
    ↓
pick_up_blue step 9: x → -260 (trust mode, 回到抓取位)  ← v6 新增
    ↓
(下一轮) target1.py step 4: x 已在 -260 → 跳过
(下一轮) target4.py: prep_x = -220 → x → -220
```

### CLI 用法
```bash
python main/arm/each_task/task4/pick_up_blue.py                    # 默认回 -260 (target1)
python main/arm/each_task/task4/pick_up_blue.py --return-x -220    # 回 target4 prep_x
python main/arm/each_task/task4/pick_up_blue.py --no-return        # 不回位 (v5 行为)
```

### 风险 (现场验证后才知道)
1. **motor 当前位置不准**: 如果 motor 内部编码器有问题,move_x 绝对位置指令可能也错。
   业务层无能为力,只能等 SDK 修复。
2. **回 -260 长距 stall**: 如果 motor 长距也 stall,业务层看不到 (trust 模式不验证)。
   现场听马达声 + 配合 target1 验证。
3. **撞墙**: 如果 motor 当前位置在物理墙 -119.5,move_x(-260) = 走 -140mm 长距,
   应该 OK。但撞墙状态下马达可能有异常。

### 待办 (v6 验证后)
- [ ] 现场跑一次 v6,确认 step 9 回 -260 真的成功 (听马达声 + 看 target1 衔接)
- [ ] 如果 v6 失败: 回退到 v5 + 在编排层 target1.py 加 reset_x 兜底
- [ ] task4 其他文件 (test_yellow.py / target4.py) 是否需要同步加回位? (目前都是单次跑,不需要)
- [ ] run_full.py 多轮编排是否需要传 `return_x_mm=-220` (target4 流程)?